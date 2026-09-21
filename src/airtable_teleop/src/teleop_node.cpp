#include "airtable_interfaces/msg/manual_motion_command.hpp"
#include <chrono>
#include <cstdint>
#include <memory>
#include <rclcpp/node.hpp>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <airtable_interfaces/msg/manual_motion_command.hpp>

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode(): Node("airtable_teleop")
    {
        pulse_duration_ms_ =
            this->declare_parameter<int>("pulse_duration_ms", 100);

        button_up_ =
            this->declare_parameter<int>("button_up", 11);

        button_down_ =
            this->declare_parameter<int>("button_down", 12);

        button_left_ =
            this->declare_parameter<int>("button_left", 13);

        button_right_ =
            this->declare_parameter<int>("button_right", 14);

        button_thruster_rotate_ccw_ =
            this->declare_parameter<int>("button_rotate_ccw", 9);

        button_thruster_rotate_cw_ =
            this->declare_parameter<int>("button_rotate_cw", 10);

        button_rw_rotate_ccw_ =
            this->declare_parameter<int>("button_rw_rotate_ccw", 0);

        button_rw_rotate_cw_ =
            this->declare_parameter<int>("button_rw_rotate_cw", 1); 

        publisher_ =
            this->create_publisher<
                airtable_interfaces::msg::ManualMotionCommand>(
                    "commands/manual/motion",
                    10);

        subscription_ =
            this->create_subscription<
                sensor_msgs::msg::Joy>(
                    "/joy",
                    10,
                    std::bind(
                        &TeleopNode::joy_callback,
                        this,
                        std::placeholders::_1));
    }

private:
    void joy_callback(
        const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (!initialized_) {
            previous_buttons_ = msg->buttons;
            initialized_ = true;
            return;
        }

        if (rising_edge(msg, button_up_)) {
            publish_command(
                airtable_interfaces::msg::ManualMotionCommand::FORWARD);
        }

        if (rising_edge(msg, button_down_)) {
            publish_command(
                airtable_interfaces::msg::ManualMotionCommand::BACKWARD);
        }

        if (rising_edge(msg, button_left_)) {
            publish_command(
                airtable_interfaces::msg::ManualMotionCommand::LEFT);
        }

        if (rising_edge(msg, button_right_)) {
            publish_command(
                airtable_interfaces::msg::ManualMotionCommand::RIGHT);
        }

        if (rising_edge(msg, button_thruster_rotate_ccw_)) {
            publish_command(
                airtable_interfaces::msg::ManualMotionCommand::THRUSTER_ROTATE_CCW);
        }

        if (rising_edge(msg, button_thruster_rotate_cw_)) {
            publish_command(
                airtable_interfaces::msg::ManualMotionCommand::THRUSTER_ROTATE_CW);
        }

        if (rising_edge(msg, button_rw_rotate_ccw_)) {
            publish_command(
                airtable_interfaces::msg::ManualMotionCommand::RW_ROTATE_CCW);
        }

        if (rising_edge(msg, button_rw_rotate_cw_)) {
            publish_command(
                airtable_interfaces::msg::ManualMotionCommand::RW_ROTATE_CW);
        }

        previous_buttons_ = msg->buttons;
    }

    bool rising_edge(
        const sensor_msgs::msg::Joy::SharedPtr & msg, int index) const
    {
        if (index < 0){
            return false;
        }

        const auto i = static_cast<std::size_t>(index);

        if (i >= msg->buttons.size() || i >= previous_buttons_.size()){
            return false;
        }

        return previous_buttons_[i] == 0 && msg->buttons[i] == 1;
    }

    void publish_command(std::uint8_t command)
    {
        airtable_interfaces::msg::ManualMotionCommand msg;

        msg.stamp = this->now();
        msg.command = command;

        const auto duration = std::chrono::milliseconds(pulse_duration_ms_);

        msg.pulse_duration.sec = 
            static_cast<std::int32_t>(
                    std::chrono::duration_cast<std::chrono::seconds>(duration).count());

        const auto remaining = 
            duration - std::chrono::seconds(msg.pulse_duration.sec);

        msg.pulse_duration.nanosec =
            static_cast<std::uint32_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(remaining).count());

        publisher_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "Manual command %u, pulse %d ms",
            command,
            pulse_duration_ms_);
    }

    bool initialized_{false};

    std::vector<std::int32_t> previous_buttons_;

    int pulse_duration_ms_;

    int button_up_;
    int button_down_;
    int button_left_;
    int button_right_;
    int button_thruster_rotate_ccw_;
    int button_thruster_rotate_cw_;
    int button_rw_rotate_ccw_;
    int button_rw_rotate_cw_;

    rclcpp::Publisher<
        airtable_interfaces::msg::ManualMotionCommand>::SharedPtr publisher_;

    rclcpp::Subscription<
        sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<TeleopNode>());

    rclcpp::shutdown();

    return 0;
}


