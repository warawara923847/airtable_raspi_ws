#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>

#include <airtable_interfaces/msg/thruster_pulse_command.hpp>
#include <airtable_interfaces/msg/thruster_setpoint.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>

#include "airtable_controller/pulse_density_modulator.hpp"

class PulseModulatorNode : public rclcpp::Node
{
public:
  PulseModulatorNode()
  : Node("airtable_pulse_modulator")
  {
    names_ = declare_parameter<std::vector<std::string>>(
            "thrusters.names", std::vector<std::string>{});
    if (names_.empty() || std::set<std::string>(names_.begin(), names_.end()).size() !=
      names_.size())
    {
      throw std::invalid_argument("Thruster names must be nonempty and unique");
    }
    // Pulse width, and the tick at which each channel may start one pulse. A period longer
    // than the width leaves the valve closed in between (slow valves), limiting the duty
    // to width/period: the average thrust of a channel is at most on_thrust * width/period.
    quantum_ = declare_parameter<double>("pulse_duration_s", 0.01);
    period_ = declare_parameter<double>("pulse_period_s", quantum_);
    timeout_ = declare_parameter<double>("command_timeout_s", 0.25);
    if (!std::isfinite(quantum_) || quantum_ < 0.002 || quantum_ > 0.5 ||
      !std::isfinite(period_) || period_ < quantum_ - 1e-12 || period_ > 1.0 ||
      !std::isfinite(timeout_) || timeout_ < period_)
    {
      throw std::invalid_argument(
              "Require pulse in [0.002, 0.5] s, pulse <= period <= 1 s, timeout >= period");
    }
    const double duty = std::min(1.0, quantum_ / period_);
    std::vector<double> full_scale;
    for (const auto & name : names_) {
      if (name.empty()) {
        throw std::invalid_argument("Empty thruster name");
      }
      const double maximum = declare_parameter<double>(
                "thrusters." + name + ".max_thrust_n", 0.5);
      const double on_thrust = declare_parameter<double>(
                "thrusters." + name + ".on_thrust_n", maximum);
      if (!std::isfinite(maximum) || maximum <= 0.0 ||
        !std::isfinite(on_thrust) || on_thrust < maximum)
      {
        throw std::invalid_argument("Require 0 < max_thrust_n <= on_thrust_n");
      }
      if (maximum > on_thrust * duty + 1e-9) {
        throw std::invalid_argument(
                name + ".max_thrust_n exceeds on_thrust_n * pulse_duration_s / pulse_period_s (" +
                std::to_string(on_thrust * duty) + " N); lower it so the allocator stays feasible");
      }
      maxima_.push_back(maximum);
      on_thrust_.push_back(on_thrust);
      // One pulse per period delivers on_thrust * width; express it as an average thrust.
      full_scale.push_back(on_thrust * duty);
    }
    modulator_ = std::make_unique<airtable_controller::PulseDensityModulator>(full_scale);
    thrust_.resize(names_.size(), 0.0);
    pulse_pub_ = create_publisher<airtable_interfaces::msg::ThrusterPulseCommand>(
            "actuators/thrusters/pulse", 1);
    subscription_ = create_subscription<airtable_interfaces::msg::ThrusterSetpoint>(
            "actuators/thrusters/setpoint", 1,
      [this](airtable_interfaces::msg::ThrusterSetpoint::ConstSharedPtr msg) {
        receive(*msg);
            });
    timer_ = rclcpp::create_timer(this, get_clock(),
            rclcpp::Duration::from_seconds(period_), [this]() {update();});
    RCLCPP_INFO(get_logger(), "%.0f ms pulses, at most one per %.0f ms per thruster",
            quantum_ * 1e3, period_ * 1e3);
  }

private:
  void clear()
  {
    have_command_ = false;
    std::fill(thrust_.begin(), thrust_.end(), 0.0);
    modulator_->reset();
  }

  void receive(const airtable_interfaces::msg::ThrusterSetpoint & msg)
  {
    const double stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9;
    const double age = now().seconds() - stamp;
    if (msg.names.size() != names_.size() || msg.thrust.size() != names_.size() ||
      std::set<std::string>(msg.names.begin(), msg.names.end()).size() != names_.size() ||
      msg.stamp.sec < 0 || msg.stamp.nanosec >= 1000000000 ||
      age < -0.05 || age > timeout_)
    {
      clear();
      return;
    }
    if (have_command_ && stamp <= command_stamp_) {
      return;
    }
    std::vector<double> next(names_.size());
    for (std::size_t i = 0; i < names_.size(); ++i) {
      const auto match = std::find(msg.names.begin(), msg.names.end(), names_[i]);
      if (match == msg.names.end()) {
        clear();
        return;
      }
      const double force = msg.thrust[std::distance(msg.names.begin(), match)];
      if (!std::isfinite(force) || force < 0.0 || force > maxima_[i] + 1e-10) {
        clear();
        return;
      }
      next[i] = std::min(force, maxima_[i]);
    }
    thrust_ = next;
    command_stamp_ = stamp;
    received_ = std::chrono::steady_clock::now();
    have_command_ = true;
    if (std::all_of(thrust_.begin(), thrust_.end(), [](double f) {return f == 0.0;})) {
      modulator_->reset();
    }
  }

  void update()
  {
    const double time = now().seconds();
    const double elapsed = time - last_tick_;
    if (last_tick_ >= 0.0 && elapsed < 0.0) {
      clear();
    } else if (last_tick_ >= 0.0 && elapsed < period_ * 0.5) {
      return;
    } else if (last_tick_ >= 0.0 && elapsed > 3.0 * period_) {
      modulator_->reset();        // Never replay a backlog of pulses after a scheduling gap.
    }
    last_tick_ = time;
    const double wall_age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - received_).count();
    if (!have_command_ || time - command_stamp_ > timeout_ ||
      time < command_stamp_ - 0.05 || wall_age > timeout_)
    {
      clear();
      return;
    }
    const auto selected = modulator_->step(thrust_);
    if (selected.empty()) {
      return;
    }
    airtable_interfaces::msg::ThrusterPulseCommand msg;
    msg.stamp = now();
    msg.pulse_duration = rclcpp::Duration::from_seconds(quantum_);
    for (std::size_t index : selected) {
      msg.names.push_back(names_[index]);
    }
    pulse_pub_->publish(msg);
  }

  std::vector<std::string> names_;
  std::vector<double> on_thrust_, maxima_, thrust_;
  std::unique_ptr<airtable_controller::PulseDensityModulator> modulator_;
  double quantum_{0.01}, period_{0.01}, timeout_{0.25}, command_stamp_{0.0}, last_tick_{-1.0};
  bool have_command_{false};
  std::chrono::steady_clock::time_point received_;
  rclcpp::Subscription<airtable_interfaces::msg::ThrusterSetpoint>::SharedPtr subscription_;
  rclcpp::Publisher<airtable_interfaces::msg::ThrusterPulseCommand>::SharedPtr pulse_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PulseModulatorNode>());
  rclcpp::shutdown();
  return 0;
}
