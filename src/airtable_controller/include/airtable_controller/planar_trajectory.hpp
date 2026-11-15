#pragma once

#include <string>
#include <Eigen/Core>

namespace airtable_controller
{

struct TrajectoryConfig
{
  std::string type{"line"};
  double amplitude_m{0.1};
  double radius_m{0.1};
  double period_s{20.0};
  double startup_s{5.0};
  double direction_rad{0.0};
  Eigen::Vector2d offset_m{0.0, 0.0};
  bool absolute_circle_center{false};
  Eigen::Vector2d circle_center_m{0.5, 1.0};
};

struct PlanarReference
{
  Eigen::Vector2d position;
  Eigen::Vector2d velocity;
  Eigen::Vector2d acceleration;
};

class PlanarTrajectory
{
public:
  explicit PlanarTrajectory(const TrajectoryConfig & config);
  PlanarReference sample(const Eigen::Vector2d & start, double elapsed_s) const;

private:
  TrajectoryConfig config_;
};

double wrap_angle(double radians);

struct TrackingGains
{
  double mass_kg{1.0};
  double inertia_kg_m2{0.00666666666667};
  double kp{0.8};
  double kd{1.8};
  double yaw_kp{2.0};
  double yaw_kd{2.8};
  double max_accel{0.04};
  double max_yaw_accel{0.5};
};

Eigen::Vector3d tracking_wrench(
  const PlanarReference & reference, const Eigen::Vector2d & position,
  const Eigen::Vector2d & body_velocity, double yaw, double yaw_rate,
  double target_yaw, const TrackingGains & gains);

}  // namespace airtable_controller
