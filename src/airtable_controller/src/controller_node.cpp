#include <cmath>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <airtable_interfaces/msg/thruster_setpoint.hpp>
#include <airtable_interfaces/msg/manual_motion_command.hpp>
#include <airtable_interfaces/msg/thruster_pulse_command.hpp>

#include "airtable_controller/control_allocator.hpp"
#include "airtable_controller/manual_mixer.hpp"
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

    publisher_=
        create_publisher<airtable_interfaces::msg::ThrusterSetpoint>(
            "/actuators/thrusters/setpoint",
            10);

    subscription_=
        create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/wrench_cmd",
            10,
            std::bind(
                &ControllerNode::wrench_callback,
                this,
                std::placeholders::_1));

    pulse_publisher_=
        create_publisher<airtable_interfaces::msg::ThrusterPulseCommand>(
            "/actuators/thrusters/pulse",
            10);

    manual_subscription_=
        create_subscription<airtable_interfaces::msg::ManualMotionCommand>(
            "/commands/manual/motion",
            10,
            std::bind(
                &ControllerNode::manual_callback,
                this,
                std::placeholders::_1));
}

private:
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

    void manual_callback(
        const airtable_interfaces::msg::ManualMotionCommand::SharedPtr msg)
    {
        const auto thrusters =
            manual_mixer_.mix(msg->command);

        if (thrusters.empty()) {
            return;
        }

        airtable_interfaces::msg::ThrusterPulseCommand output;

        output.stamp = msg->stamp;
        output.names = thrusters;
        output.pulse_duration = msg->pulse_duration;

        pulse_publisher_->publish(output);
    }

    std::vector<airtable_controller::ThrusterConfig> thruster_configs_;
    std::unique_ptr<airtable_controller::ControlAllocator> allocator_;

    airtable_controller::ManualMixer manual_mixer_;

    rclcpp::Publisher<
        airtable_interfaces::msg::
            ThrusterSetpoint>::SharedPtr
        publisher_;

    rclcpp::Subscription<
        geometry_msgs::msg::
            WrenchStamped>::SharedPtr
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
