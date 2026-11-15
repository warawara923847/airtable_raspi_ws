#include "airtable_controller/planar_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace airtable_controller
{

PlanarTrajectory::PlanarTrajectory(const TrajectoryConfig & config)
: config_(config)
{
  if ((config.type != "line" && config.type != "circle") ||
    !std::isfinite(config.amplitude_m) || config.amplitude_m < 0.0 ||
    !std::isfinite(config.radius_m) || config.radius_m <= 0.0 ||
    !std::isfinite(config.period_s) || config.period_s <= 0.0 ||
    !std::isfinite(config.startup_s) || config.startup_s <= 0.0 ||
    !std::isfinite(config.direction_rad) || !config.offset_m.allFinite() ||
    !config.circle_center_m.allFinite())
  {
    throw std::invalid_argument("Invalid line/circle trajectory parameters");
  }
}

PlanarReference PlanarTrajectory::sample(const Eigen::Vector2d & start, double elapsed_s) const
{
  if (!start.allFinite() || !std::isfinite(elapsed_s)) {
    throw std::invalid_argument("Trajectory time and start position must be finite");
  }
  const double t = std::max(0.0, elapsed_s);
  const double omega = 2.0 * std::acos(-1.0) / config_.period_s;
  const double u = std::min(t / config_.startup_s, 1.0);
  const double g = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
  const double gd = 30.0 * u * u * (1.0 - u) * (1.0 - u) / config_.startup_s;
  const double gdd = 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u) /
    (config_.startup_s * config_.startup_s);
  if (config_.type == "circle") {
    // Ramp angular speed, preserving the specified radius even during startup.
    const double phase_time = t < config_.startup_s ? config_.startup_s *
      (2.5 * std::pow(u, 4) - 3.0 * std::pow(u, 5) + std::pow(u, 6)) :
      t - config_.startup_s / 2.0;
    const double phase = omega * phase_time;
    const double rate = omega * g;
    const double angular_accel = omega * gd;
    const Eigen::Vector2d center = (config_.absolute_circle_center ? config_.circle_center_m :
      (start - Eigen::Vector2d(config_.radius_m, 0.0)).eval()) + config_.offset_m;
    const Eigen::Vector2d radial(std::cos(phase), std::sin(phase));
    const Eigen::Vector2d tangent(-std::sin(phase), std::cos(phase));
    const Eigen::Vector2d correction = start - center - Eigen::Vector2d(config_.radius_m, 0.0);
    return {center + config_.radius_m * radial + (1.0 - g) * correction,
      config_.radius_m * rate * tangent - gd * correction,
      config_.radius_m * (angular_accel * tangent - rate * rate * radial) - gdd * correction};
  }
  const double phase = omega * t;
  Eigen::Vector2d displacement, velocity, acceleration;
  const Eigen::Vector2d axis(std::cos(config_.direction_rad),
    std::sin(config_.direction_rad));
  displacement = config_.amplitude_m * std::sin(phase) * axis;
  velocity = config_.amplitude_m * omega * std::cos(phase) * axis;
  acceleration = -config_.amplitude_m * omega * omega * std::sin(phase) * axis;
  displacement += config_.offset_m;
  return {start + g * displacement, gd * displacement + g * velocity,
    gdd * displacement + 2.0 * gd * velocity + g * acceleration};
}

double wrap_angle(double radians)
{
  return std::atan2(std::sin(radians), std::cos(radians));
}

Eigen::Vector3d tracking_wrench(
  const PlanarReference & reference, const Eigen::Vector2d & position,
  const Eigen::Vector2d & body_velocity, double yaw, double yaw_rate,
  double target_yaw, const TrackingGains & gains)
{
  Eigen::Matrix2d rotation;
  rotation << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
  const Eigen::Vector2d velocity = rotation * body_velocity;
  Eigen::Vector2d acceleration = reference.acceleration +
    gains.kp * (reference.position - position) +
    gains.kd * (reference.velocity - velocity);
  const double norm = acceleration.norm();
  if (norm > gains.max_accel) {
    acceleration *= gains.max_accel / norm;
  }
  const Eigen::Vector2d body_force = gains.mass_kg * rotation.transpose() * acceleration;
  const double yaw_accel = std::clamp(
        gains.yaw_kp * wrap_angle(target_yaw - yaw) - gains.yaw_kd * yaw_rate,
        -gains.max_yaw_accel, gains.max_yaw_accel);
  return {body_force.x(), body_force.y(), gains.inertia_kg_m2 * yaw_accel};
}

}  // namespace airtable_controller
