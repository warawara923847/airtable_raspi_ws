#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "airtable_controller/planar_trajectory.hpp"

namespace ac = airtable_controller;

class TrajectoryControllerNode : public rclcpp::Node
{
public:
  TrajectoryControllerNode()
  : Node("airtable_trajectory_controller")
  {
    frame_ = declare_parameter<std::string>("reference_frame", "table_1");
    body_frame_ = declare_parameter<std::string>("body_frame", "robot_1/base_link");
    const auto state_topic = declare_parameter<std::string>("state_topic", "state/odometry");
    enabled_ = declare_parameter<bool>("start_enabled", false);
    fixed_yaw_ = finite_parameter("trajectory.fixed_yaw_rad", 0.0);
    ac::TrajectoryConfig config;
    config.type = declare_parameter<std::string>("trajectory.type", "line");
    config.amplitude_m = finite_parameter("trajectory.amplitude_m", 0.1);
    config.radius_m = finite_parameter("trajectory.radius_m", 0.1);
    config.period_s = positive_parameter("trajectory.period_s", 20.0);
    config.startup_s = positive_parameter("trajectory.startup_s", 5.0);
    config.direction_rad = finite_parameter("trajectory.direction_rad", 0.0);
    const auto offset = declare_parameter<std::vector<double>>(
            "trajectory.offset_m", {0.0, 0.0});
    if (offset.size() != 2) {
      throw std::invalid_argument("trajectory.offset_m must contain two values");
    }
    config.offset_m = Eigen::Vector2d(offset[0], offset[1]);
    const auto center = declare_parameter<std::vector<double>>(
      "trajectory.circle_center_m", std::vector<double>{});
    if (!center.empty() && center.size() != 2) {
      throw std::invalid_argument("trajectory.circle_center_m must be empty or [x, y]");
    }
    if (!center.empty()) {
      config.absolute_circle_center = true;
      config.circle_center_m = Eigen::Vector2d(center[0], center[1]);
    }
    trajectory_ = std::make_unique<ac::PlanarTrajectory>(config);
    preview_duration_ = config.startup_s + config.period_s;
    gains_.mass_kg = positive_parameter("model.mass_kg", 1.0);
    gains_.inertia_kg_m2 = positive_parameter("model.inertia_kg_m2", 0.00666666666667);
    gains_.kp = positive_parameter("gains.position_kp", 0.8);
    gains_.kd = positive_parameter("gains.velocity_kd", 1.8);
    gains_.yaw_kp = positive_parameter("gains.yaw_kp", 2.0);
    gains_.yaw_kd = positive_parameter("gains.yaw_rate_kd", 2.8);
    gains_.max_accel = positive_parameter("limits.acceleration_m_s2", 0.04);
    gains_.max_yaw_accel = positive_parameter("limits.yaw_acceleration_rad_s2", 0.5);
    state_timeout_ = positive_parameter("state_timeout_s", 0.25);
    path_interval_ = 1.0 / positive_parameter("actual_path_rate_hz", 20.0);
    const auto bounds = declare_parameter<std::vector<double>>(
            "limits.position_bounds_m", {0.15, 0.85, 0.15, 1.85});
    if (bounds.size() != 4) {
      throw std::invalid_argument("Expected bounds [xmin, xmax, ymin, ymax]");
    }
    for (double bound : bounds) {
      if (!std::isfinite(bound)) {
        throw std::invalid_argument("Bounds must be finite");
      }
    }
    if (bounds[0] >= bounds[1] || bounds[2] >= bounds[3]) {
      throw std::invalid_argument("Invalid position bounds");
    }
    bounds_ = bounds;
    if (frame_.empty() || body_frame_.empty() || state_topic.empty()) {
      throw std::invalid_argument("State topic and frames must not be empty");
    }
    wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("wrench_cmd", 1);
    reference_pub_ = create_publisher<nav_msgs::msg::Odometry>("trajectory/reference", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "trajectory/path", rclcpp::QoS(1).transient_local());
    actual_pub_ = create_publisher<nav_msgs::msg::Path>("trajectory/actual_path", 1);
    error_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("trajectory/error", 10);
    active_pub_ = create_publisher<std_msgs::msg::Bool>(
            "trajectory/active", rclcpp::QoS(1).transient_local());
    set_active(false);
    state_sub_ = create_subscription<nav_msgs::msg::Odometry>(state_topic, 1,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {receive_state(*msg);});
    enable_service_ = create_service<std_srvs::srv::SetBool>("trajectory/enable",
        [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response) {
          if (!request->data) {
            enabled_ = false;
            started_ = false;
            set_active(false);
            publish_zero();
            response->success = true;
            response->message = "Output disabled; existing pulses expire, vehicle coasts";
          } else if (!fresh_state() || !inside(position_)) {
            response->success = false;
            response->message = "A fresh, valid, in-bounds Odometry is required";
          } else {
            enabled_ = true;
            started_ = false;
            response->success = true;
            response->message = "Starting from the current position";
          }
            });
    const double rate = positive_parameter("control_rate_hz", 50.0);
    timer_ = rclcpp::create_timer(this, get_clock(),
            rclcpp::Duration::from_seconds(1.0 / rate), [this]() {update();});
    RCLCPP_INFO(get_logger(), "%s trajectory, fixed yaw %.3f rad, state=%s",
            config.type.c_str(), fixed_yaw_, state_topic.c_str());
  }

private:
  double finite_parameter(const std::string & name, double fallback)
  {
    const double value = declare_parameter<double>(name, fallback);
    if (!std::isfinite(value)) {
      throw std::invalid_argument(name + " must be finite");
    }
    return value;
  }

  double positive_parameter(const std::string & name, double fallback)
  {
    const double value = finite_parameter(name, fallback);
    if (value <= 0.0) {
      throw std::invalid_argument(name + " must be positive");
    }
    return value;
  }

  bool inside(const Eigen::Vector2d & position) const
  {
    return position.x() >= bounds_[0] && position.x() <= bounds_[1] &&
           position.y() >= bounds_[2] && position.y() <= bounds_[3];
  }

  bool fresh_state() const
  {
    const double age = now().seconds() - state_stamp_;
    const double wall_age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - state_received_).count();
    return valid_state_ && age >= -0.05 && age <= state_timeout_ &&
           wall_age <= state_timeout_;
  }

  void receive_state(const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose;
    const auto & v = msg.twist.twist;
    const auto & q = p.orientation;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    const Eigen::Vector2d position(p.position.x, p.position.y);
    const Eigen::Vector2d velocity(v.linear.x, v.linear.y);
    if (msg.header.frame_id != frame_ || msg.child_frame_id != body_frame_ ||
      !position.allFinite() || !velocity.allFinite() || !std::isfinite(p.position.z) ||
      !std::isfinite(v.angular.z) || !std::isfinite(norm) || norm < 1e-6)
    {
      valid_state_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Rejected invalid Odometry or unexpected pose/twist frames");
      return;
    }
    const double stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
    const double age = now().seconds() - stamp;
    if (msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1000000000 ||
      age < -0.05 || age > state_timeout_)
    {
      valid_state_ = false;
      return;
    }
    if (valid_state_ && stamp <= state_stamp_) {
      return;
    }
    position_ = position;
    velocity_ = velocity;
    height_ = p.position.z;
    yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y) / (norm * norm),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z) / (norm * norm));
    yaw_rate_ = v.angular.z;
    state_stamp_ = stamp;
    state_received_ = std::chrono::steady_clock::now();
    valid_state_ = true;
  }

  void set_active(bool active)
  {
    std_msgs::msg::Bool status;
    status.data = active;
    active_pub_->publish(status);
  }

  void publish_zero()
  {
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = body_frame_;
    wrench_pub_->publish(msg);
  }

  void fault(const char * reason)
  {
    enabled_ = false;
    started_ = false;
    set_active(false);
    publish_zero();
    RCLCPP_ERROR(get_logger(), "%s; output disabled, explicitly enable to restart", reason);
  }

  bool initialize(double time)
  {
    origin_ = position_;
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_;
    path.header.stamp = now();
    for (int i = 0; i <= 500; ++i) {
      const auto sample = trajectory_->sample(origin_, preview_duration_ * i / 500.0);
      if (!inside(sample.position)) {
        fault("Reference path exceeds configured position bounds");
        return false;
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = sample.position.x();
      pose.pose.position.y = sample.position.y();
      pose.pose.position.z = height_;
      pose.pose.orientation.z = std::sin(fixed_yaw_ / 2.0);
      pose.pose.orientation.w = std::cos(fixed_yaw_ / 2.0);
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
    actual_path_.poses.clear();
    actual_path_.header.frame_id = frame_;
    epoch_ = time;
    next_path_time_ = time;
    started_ = true;
    set_active(true);
    return true;
  }

  void update()
  {
    const double time = now().seconds();
    if (last_tick_ >= 0.0 && time < last_tick_) {
      valid_state_ = false;
      fault("ROS time moved backwards");
    }
    last_tick_ = time;
    if (!enabled_) {
      publish_zero();
      return;
    }
    if (!fresh_state()) {
      if (started_) {
        fault("Odometry unavailable, invalid or stale");
      } else {
        publish_zero();
      }
      return;
    }
    if (!inside(position_)) {
      fault("Vehicle exceeded configured position bounds");
      return;
    }
    if (!started_ && !initialize(time)) {
      return;
    }
    const auto reference = trajectory_->sample(origin_, time - epoch_);
    const auto wrench = ac::tracking_wrench(reference, position_, velocity_, yaw_, yaw_rate_,
            fixed_yaw_, gains_);
    geometry_msgs::msg::WrenchStamped output;
    output.header.stamp = now();
    output.header.frame_id = body_frame_;
    output.wrench.force.x = wrench.x();
    output.wrench.force.y = wrench.y();
    output.wrench.torque.z = wrench.z();
    wrench_pub_->publish(output);
    nav_msgs::msg::Odometry target;
    target.header = output.header;
    target.header.frame_id = frame_;
    target.child_frame_id = body_frame_;
    target.pose.pose.position.x = reference.position.x();
    target.pose.pose.position.y = reference.position.y();
    target.pose.pose.position.z = height_;
    target.pose.pose.orientation.z = std::sin(fixed_yaw_ / 2.0);
    target.pose.pose.orientation.w = std::cos(fixed_yaw_ / 2.0);
    target.twist.twist.linear.x = std::cos(fixed_yaw_) * reference.velocity.x() +
      std::sin(fixed_yaw_) * reference.velocity.y();
    target.twist.twist.linear.y = -std::sin(fixed_yaw_) * reference.velocity.x() +
      std::cos(fixed_yaw_) * reference.velocity.y();
    reference_pub_->publish(target);
    geometry_msgs::msg::Vector3Stamped error;
    error.header = target.header;
    error.vector.x = reference.position.x() - position_.x();
    error.vector.y = reference.position.y() - position_.y();
    error.vector.z = ac::wrap_angle(fixed_yaw_ - yaw_);
    error_pub_->publish(error);
    if (time >= next_path_time_ - 1e-9) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = target.header;
      pose.pose.position.x = position_.x();
      pose.pose.position.y = position_.y();
      pose.pose.position.z = height_;
      pose.pose.orientation.z = std::sin(yaw_ / 2.0);
      pose.pose.orientation.w = std::cos(yaw_ / 2.0);
      actual_path_.header.stamp = now();
      actual_path_.poses.push_back(pose);
      if (actual_path_.poses.size() > 2000) {
        actual_path_.poses.erase(actual_path_.poses.begin());
      }
      actual_pub_->publish(actual_path_);
      next_path_time_ += path_interval_;
      if (next_path_time_ <= time) {
        next_path_time_ = time + path_interval_;
      }
    }
  }

  std::string frame_, body_frame_;
  std::unique_ptr<ac::PlanarTrajectory> trajectory_;
  ac::TrackingGains gains_;
  std::vector<double> bounds_;
  Eigen::Vector2d position_{0.0, 0.0}, velocity_{0.0, 0.0}, origin_{0.0, 0.0};
  double yaw_{0.0}, yaw_rate_{0.0}, height_{0.0}, fixed_yaw_{0.0};
  double state_timeout_{0.25}, state_stamp_{-1.0}, epoch_{0.0}, last_tick_{-1.0};
  double preview_duration_{25.0}, next_path_time_{0.0};
  double path_interval_{0.05};
  bool enabled_{false}, started_{false}, valid_state_{false};
  std::chrono::steady_clock::time_point state_received_;
  nav_msgs::msg::Path actual_path_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr reference_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, actual_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr error_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryControllerNode>());
  rclcpp::shutdown();
  return 0;
}
