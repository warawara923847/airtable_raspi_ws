#include <cmath>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <airtable_interfaces/msg/thruster_setpoint.hpp>
#include <airtable_interfaces/msg/manual_motion_command.hpp>
#include <airtable_interfaces/msg/thruster_pulse_command.hpp>
#include <airtable_interfaces/msg/reaction_wheel_command.hpp>
#include <airtable_interfaces/msg/reaction_wheel_state.hpp>

#include "airtable_controller/control_allocator.hpp"
#include "airtable_controller/manual_mixer.hpp"
#include "airtable_controller/reaction_wheel_controller.hpp"
#include "airtable_interfaces/msg/manual_motion_command.hpp"


using std::placeholders::_1;

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode(): Node("airtable_controller")
{
    thruster_configs_=load_thrusters(),

    allocator_=
        std::make_unique<airtable_controller::ControlAllocator>(
            thruster_configs_);

    rw_configs_=load_reaction_wheels();

    rw_name_=
        this->declare_parameter<std::string>(
            "manual.reaction_wheel.name",
            "rw_1");

    rw_speed_step_rad_s_=
        this->declare_parameter<double>(
            "manual.reaction_wheel.speed_step_rad_s",
            20.0);

    const auto & rw_config = find_rw_config(rw_name_);

    rw_controller_ =
        std::make_unique<airtable_controller::ReactionWheelController>(
            rw_config);

    publisher_=
        create_publisher<airtable_interfaces::msg::ThrusterSetpoint>(
            "actuators/thrusters/setpoint",
            10);

    subscription_=
        create_subscription<geometry_msgs::msg::WrenchStamped>(
            "wrench_cmd",
            10,
            std::bind(
                &ControllerNode::wrench_callback,
                this,
                std::placeholders::_1));

    pulse_publisher_=
        create_publisher<airtable_interfaces::msg::ThrusterPulseCommand>(
            "actuators/thrusters/pulse",
            10);

    rw_publisher_=
        create_publisher<airtable_interfaces::msg::ReactionWheelCommand>(
            "actuators/reaction_wheels/command",
            10);

    rw_state_subscription_ =
        create_subscription<
            airtable_interfaces::msg::ReactionWheelState>(
            "actuators/reaction_wheels/state",
            10,
            std::bind(
                &ControllerNode::rw_state_callback,
                this,
                std::placeholders::_1));

    manual_subscription_=
        create_subscription<airtable_interfaces::msg::ManualMotionCommand>(
            "commands/manual/motion",
            10,
            std::bind(
                &ControllerNode::manual_callback,
                this,
                std::placeholders::_1));
    
}

private:
    struct ManualYawActuator
    {
        bool thruster{true};
        bool reaction_wheel{false};
    };

    std::vector<airtable_controller::ThrusterConfig> load_thrusters()
    {
        const auto names =
            this->declare_parameter<std::vector<std::string>>(
                "thrusters.names",
            {});

        if(names.empty()){
            throw std::runtime_error(
                "Parameter 'thrusters.names' is empty");
        }

        std::vector<airtable_controller::ThrusterConfig> configs;

        for(const auto & name: names){
            const std::string prefix = "thrusters." + name;

            const auto position =
                this->declare_parameter<std::vector<double>>(
                    prefix + ".position",
                    {});

            const auto direction =
                this->declare_parameter<std::vector<double>>(
                    prefix + ".direction",
                    {});

            const double min_thrust =
                this->declare_parameter<double>(
                    prefix + ".min_thrust_n",
                    0.0);

            const double max_thrust =
                this->declare_parameter<double>(
                    prefix + ".max_thrust_n",
                    0.0);

            if(position.size() != 2){
                throw std::runtime_error(
                    prefix + ".position must have 2 values");
            }

            if(direction.size() != 2){
                throw std::runtime_error(
                    prefix + ".direction must have 2 values");
            }

            const double norm =
                std::hypot(direction[0], direction[1]);

            if(norm <= 0.0){
                throw std::runtime_error(
                    prefix + ".direction must be non-zero");
            }

            if(max_thrust < min_thrust){
                throw std::runtime_error(
                    prefix + ":invalid thrust limits");
            }

            configs.push_back({
                name,
                position[0],
                position[1],
                direction[0] / norm,
                direction[1] / norm,
                min_thrust,
                max_thrust
            });
        }

        return configs;
    }

    std::vector<airtable_controller::ReactionWheelConfig> load_reaction_wheels()
    {
        const auto names =
            this->declare_parameter<std::vector<std::string>>(
                "reaction_wheels.names",
            {});

        if(names.empty()){
            throw std::runtime_error(
                "Parameter 'reaction_wheels.names' is empty");
        }

        std::vector<airtable_controller::ReactionWheelConfig> configs;

        for(const auto & name: names){
            const std::string prefix = "reaction_wheels." + name;

            const double speed_kp =
                this->declare_parameter<double>(
                    prefix + ".speed_kp",
                    0.05);

            const double max_torque_nm =
                this->declare_parameter<double>(
                    prefix + ".max_torque_nm",
                    0.01);

            const double max_speed_rad_s =
                this->declare_parameter<double>(
                    prefix + ".max_speed_rad_s",
                    100.0);

            if (speed_kp < 0.0) {
                throw std::runtime_error(
                    prefix + ": speed_kp must be non-negative");
            }

            if (max_torque_nm <= 0.0) {
                throw std::runtime_error(
                    prefix + ": max_torque_nm must be non-negative");
            }

            if (max_speed_rad_s <= 0.0) {
                throw std::runtime_error(
                    prefix + ": max_speed_rad_s must be non-negative");
            }

            configs.push_back({
                name,
                speed_kp,
                max_torque_nm,
                max_speed_rad_s
            });
        }

        return configs;
    }


    void wrench_callback( const geometry_msgs::msg::WrenchStamped:: SharedPtr msg) {
        const double fx = msg->wrench.force.x;
        const double fy = msg->wrench.force.y;
        const double mz = msg->wrench.torque.z;

        const auto thrusts = allocator_->allocate(fx, fy, mz);

        airtable_interfaces::msg::
            ThrusterSetpoint output;

        output.stamp =
            msg->header.stamp;

        for (std::size_t i = 0; i < thruster_configs_.size(); ++i)
        {
            output.names.push_back(thruster_configs_[i].name);
            output.thrust.push_back(thrusts[i]);
        }

        publisher_->publish(output);
    }

    void handle_manual_thruster(
        const airtable_interfaces::msg::ManualMotionCommand & msg)
    {
        const auto thrusters = 
            manual_mixer_.mix(msg.command);


        if (thrusters.empty()) {
            RCLCPP_WARN(
                this->get_logger(),
                "No thrusters mapped to command %u",
                msg.command);
            return;
        }

        airtable_interfaces::msg::ThrusterPulseCommand output;

        output.stamp = msg.stamp;
        output.names = thrusters;
        output.pulse_duration = msg.pulse_duration;

        pulse_publisher_->publish(output);
    }

    const airtable_controller::ReactionWheelConfig & find_rw_config(
            const std::string & name) const
    {
        for (const auto & config : rw_configs_) {
            if (config.name == name) {
                return config;
            }
        }

        throw std::runtime_error(
            "Reaction wheel '" + name + "' is not configured");
    }

    void handle_manual_rw(
        const airtable_interfaces::msg::ManualMotionCommand & msg)
    {
        using Command = airtable_interfaces::msg::ManualMotionCommand;

        if (msg.command == Command::RW_ROTATE_CCW) {
            rw_target_speed_rad_s_ += rw_speed_step_rad_s_;
        } else if (msg.command == Command::RW_ROTATE_CW) {
            rw_target_speed_rad_s_ -= rw_speed_step_rad_s_;
        }

        const double max_speed = rw_controller_->max_speed_rad_s();

        rw_target_speed_rad_s_ = std::clamp(
            rw_target_speed_rad_s_,
            -max_speed,
            max_speed);

        RCLCPP_INFO(
            this->get_logger(),
            "Reaction wheel target speed: %.2f rad/s",
            rw_target_speed_rad_s_);
    }
    
    void update_rw_control()
    {
        const double torque = rw_controller_->compute_torque(
            rw_target_speed_rad_s_,
            rw_current_speed_rad_s_); 

        airtable_interfaces::msg::ReactionWheelCommand output;

        output.names.push_back(rw_name_);
        output.torques.push_back(torque);

        rw_publisher_->publish(output);
    }

    void rw_state_callback(
        const airtable_interfaces::msg::ReactionWheelState::SharedPtr msg)
    {
        for (std::size_t i = 0; i < msg->names.size(); ++i) {
            if (msg->names[i] != rw_name_) {
                continue;
            }

            if (i >= msg->angular_velocities.size()) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "RWState size mismatch");
                return;
            }

            rw_current_speed_rad_s_ =
                msg->angular_velocities[i];

            update_rw_control();

            return;
        }

        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Reaction wheel '%s' not found in state message",
            rw_name_.c_str());
    }
    
    void manual_callback(
        const airtable_interfaces::msg::ManualMotionCommand::SharedPtr msg)
    {
        using Command = airtable_interfaces::msg::ManualMotionCommand;

        RCLCPP_INFO(
            this->get_logger(),
            "Received manual command: %u",
            msg->command);

        switch (msg->command) {
            case Command::FORWARD:
            case Command::BACKWARD:
            case Command::LEFT:
            case Command::RIGHT:
            case Command::THRUSTER_ROTATE_CW:
            case Command::THRUSTER_ROTATE_CCW:
                RCLCPP_INFO(
                    this->get_logger(),
                    "Handling manual thruster command");
                handle_manual_thruster(*msg);
                break;

            case Command::RW_ROTATE_CW:
            case Command::RW_ROTATE_CCW:
                handle_manual_rw(*msg);
                break;

            default:
                break;
            }
        }

    std::vector<airtable_controller::ThrusterConfig> thruster_configs_;
    std::vector<airtable_controller::ReactionWheelConfig> rw_configs_;
    std::unique_ptr<airtable_controller::ControlAllocator> allocator_;

    std::unique_ptr<airtable_controller::ReactionWheelController> rw_controller_;

    airtable_controller::ManualMixer manual_mixer_;
    ManualYawActuator manual_yaw_actuators_;

    std::string rw_name_;

    double rw_current_speed_rad_s_{0.0};
    double rw_target_speed_rad_s_{0.0};
    double rw_speed_step_rad_s_{20.0};

    rclcpp::Publisher<
        airtable_interfaces::msg::ThrusterSetpoint>::SharedPtr
        publisher_;

    rclcpp::Publisher<
        airtable_interfaces::msg::ReactionWheelCommand>::SharedPtr
        rw_publisher_;

    rclcpp::Subscription<
        airtable_interfaces::msg::ReactionWheelState>::SharedPtr
        rw_state_subscription_;

    rclcpp::Subscription<
        geometry_msgs::msg::WrenchStamped>::SharedPtr
        subscription_;

    rclcpp::Publisher<
        airtable_interfaces::msg::ThrusterPulseCommand>::SharedPtr pulse_publisher_;

    rclcpp::Subscription<
        airtable_interfaces::msg::ManualMotionCommand>::SharedPtr manual_subscription_;
};

int main(
    int argc,
    char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<ControllerNode>());

    rclcpp::shutdown();

    return 0;
}
