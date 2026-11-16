// Manual speed levels and yaw-hold PID for a speed-commanded reaction wheel
// (continuous-rotation servo standing in for the ARW-3m). See docs/adr/0018.
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <airtable_interfaces/msg/manual_motion_command.hpp>
#include <airtable_interfaces/msg/reaction_wheel_speed_command.hpp>

#include "airtable_controller/rw_attitude_controller.hpp"

namespace ac = airtable_controller;
using airtable_interfaces::msg::ManualMotionCommand;
using airtable_interfaces::msg::ReactionWheelSpeedCommand;
using geometry_msgs::msg::PoseWithCovarianceStamped;

class RwAttitudeNode : public rclcpp::Node
{
public:
    RwAttitudeNode()
    : Node("airtable_rw_attitude"),
      levels_(declare_parameter<std::vector<double>>(
              "manual.levels", {-0.5, -0.25, 0.0, 0.25, 0.5})),
      pid_(load_gains())
    {
        rw_name_ = declare_parameter<std::string>("rw_name", "rw_1");
        const double rate_hz = declare_parameter<double>("publish_rate_hz", 50.0);
        pose_timeout_s_ = declare_parameter<double>("pose_timeout_s", 0.5);
        max_rate_per_s_ = declare_parameter<double>("max_speed_rate_per_s", 2.0);
        const auto pose_topic =
            declare_parameter<std::string>("pose_topic", "measurements/external_pose");
        if (rw_name_.empty() || !(rate_hz > 0.0) || !(pose_timeout_s_ > 0.0) ||
            !std::isfinite(max_rate_per_s_))
        {
            throw std::invalid_argument(
                      "rw_name, publish_rate_hz > 0 and pose_timeout_s > 0 are required");
        }
        period_s_ = 1.0 / rate_hz;

        speed_publisher_ = create_publisher<ReactionWheelSpeedCommand>(
            "actuators/reaction_wheels/speed_command", 10);
        debug_publisher_ =
            create_publisher<geometry_msgs::msg::Vector3Stamped>("rw_attitude/debug", 10);
        manual_subscription_ = create_subscription<ManualMotionCommand>(
            "commands/manual/motion", 10,
            [this](const ManualMotionCommand::SharedPtr msg) {manual_callback(*msg);});
        pose_subscription_ = create_subscription<PoseWithCovarianceStamped>(
            pose_topic, rclcpp::SensorDataQoS(),
            [this](const PoseWithCovarianceStamped::SharedPtr msg) {pose_callback(*msg);});
        // A wall timer keeps feeding the hardware watchdog independent of /clock.
        timer_ = create_wall_timer(
            std::chrono::duration<double>(period_s_), [this]() {timer_callback();});

        RCLCPP_INFO(get_logger(),
            "RW attitude for '%s': manual levels, PID kp=%.3f ki=%.3f kd=%.3f, pose %s",
            rw_name_.c_str(), gains_.kp, gains_.ki, gains_.kd, pose_topic.c_str());
    }

private:
    ac::RwAttitudeGains load_gains()
    {
        gains_.kp = declare_parameter<double>("pid.kp", 0.5);
        gains_.ki = declare_parameter<double>("pid.ki", 0.0);
        gains_.kd = declare_parameter<double>("pid.kd", 0.0);
        gains_.integral_limit = declare_parameter<double>("pid.integral_limit", 0.2);
        gains_.output_limit = declare_parameter<double>("pid.output_limit", 0.5);
        gains_.rate_filter_tau_s = declare_parameter<double>("pid.rate_filter_tau_s", 0.1);
        gains_.max_dt_s = declare_parameter<double>("pid.max_dt_s", 0.5);
        return gains_;
    }

    bool pose_fresh() const
    {
        return last_pose_received_ &&
               (now() - *last_pose_received_).seconds() <= pose_timeout_s_;
    }

    void manual_callback(const ManualMotionCommand & msg)
    {
        switch (msg.command) {
            case ManualMotionCommand::RW_ATTITUDE_TOGGLE:
                if (pid_active_) {
                    leave_pid("toggled off");
                } else if (!pose_fresh() || !last_yaw_) {
                    RCLCPP_WARN(get_logger(),
                        "PID not started: no pose within %.2f s", pose_timeout_s_);
                } else {
                    pid_.reset(*last_yaw_);
                    pid_active_ = true;
                    RCLCPP_INFO(get_logger(), "PID ON, holding yaw %.3f rad", pid_.target());
                }
                break;
            case ManualMotionCommand::RW_ROTATE_CCW:
            case ManualMotionCommand::RW_ROTATE_CW:
                if (pid_active_) {
                    RCLCPP_INFO(get_logger(), "Manual RW step ignored while PID is on");
                    break;
                }
                levels_.step(msg.command == ManualMotionCommand::RW_ROTATE_CCW ? 1 : -1);
                RCLCPP_INFO(get_logger(), "Manual RW speed level %.3f", levels_.value());
                break;
            default:
                break;
        }
    }

    void pose_callback(const PoseWithCovarianceStamped & msg)
    {
        const auto & q = msg.pose.pose.orientation;
        const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        if (!std::isfinite(yaw)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected non-finite pose");
            return;
        }
        last_yaw_ = yaw;
        last_pose_received_ = now();
        if (pid_active_) {
            pid_.update(yaw, rclcpp::Time(msg.header.stamp).seconds());
        }
    }

    void leave_pid(const char * reason)
    {
        pid_active_ = false;
        levels_.stop();
        RCLCPP_INFO(get_logger(), "PID OFF (%s); manual level 0", reason);
    }

    void timer_callback()
    {
        if (pid_active_ && !pose_fresh()) {
            RCLCPP_WARN(get_logger(), "Pose older than %.2f s", pose_timeout_s_);
            leave_pid("pose lost");
        }
        const double target = pid_active_ ? pid_.output() : levels_.value();
        command_ = ac::slew(command_, target, max_rate_per_s_, period_s_);

        ReactionWheelSpeedCommand output;
        output.stamp = now();
        output.names.push_back(rw_name_);
        output.speeds.push_back(command_);
        speed_publisher_->publish(output);

        geometry_msgs::msg::Vector3Stamped debug;
        debug.header.stamp = output.stamp;
        debug.header.frame_id = pid_active_ ? "pid" : "manual";
        debug.vector.x = pid_.error();
        debug.vector.y = pid_.rate();
        debug.vector.z = command_;
        debug_publisher_->publish(debug);
    }

    ac::RwAttitudeGains gains_;
    ac::SpeedLevels levels_;
    ac::RwAttitudeController pid_;
    std::string rw_name_;
    double pose_timeout_s_{0.5};
    double max_rate_per_s_{2.0};
    double period_s_{0.02};
    bool pid_active_{false};
    double command_{0.0};
    std::optional<double> last_yaw_;
    std::optional<rclcpp::Time> last_pose_received_;

    rclcpp::Publisher<ReactionWheelSpeedCommand>::SharedPtr speed_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr debug_publisher_;
    rclcpp::Subscription<ManualMotionCommand>::SharedPtr manual_subscription_;
    rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RwAttitudeNode>());
    rclcpp::shutdown();
    return 0;
}
