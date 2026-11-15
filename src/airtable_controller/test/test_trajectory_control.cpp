#include <cmath>
#include <limits>
#include <random>

#include <gtest/gtest.h>

#include "airtable_controller/control_allocator.hpp"
#include "airtable_controller/planar_trajectory.hpp"
#include "airtable_controller/pulse_density_modulator.hpp"

namespace ac = airtable_controller;

static std::vector<ac::ThrusterConfig> nominal_thrusters()
{
  const double s = std::sqrt(0.5);
  return {{"t1", 0.09, 0.09, -s, s, 0.0, 0.5},
    {"t2", 0.09, -0.09, -s, -s, 0.0, 0.5},
    {"t3", -0.09, -0.09, s, -s, 0.0, 0.5},
    {"t4", -0.09, 0.09, s, s, 0.0, 0.5}};
}

static Eigen::Vector3d realized(const std::vector<double> & forces)
{
  Eigen::Vector3d wrench = Eigen::Vector3d::Zero();
  const auto configs = nominal_thrusters();
  for (std::size_t i = 0; i < configs.size(); ++i) {
    const auto & t = configs[i];
    wrench += forces[i] * Eigen::Vector3d(t.dir_x, t.dir_y,
            t.x_m * t.dir_y - t.y_m * t.dir_x);
  }
  return wrench;
}

TEST(BoundedAllocator, ReproducesFeasibleMixedWrenches)
{
    ac::ControlAllocator allocator(nominal_thrusters());
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> distribution(0.0, 0.5);
    for (int trial = 0; trial < 200; ++trial) {
    const auto desired = realized({distribution(rng), distribution(rng),
          distribution(rng), distribution(rng)});
    const auto output = allocator.allocate(desired.x(), desired.y(), desired.z());
    ASSERT_EQ(output.size(), 4u);
    for (double force : output) {
      EXPECT_GE(force, 0.0);
      EXPECT_LE(force, 0.5);
    }
    EXPECT_LT((realized(output) - desired).norm(), 1e-9);
    }
}

TEST(BoundedAllocator, PureAxesDoNotLoseHalfTheCommand)
{
    ac::ControlAllocator allocator(nominal_thrusters());
    for (int axis = 0; axis < 3; ++axis) {
    for (double sign : {-1.0, 1.0}) {
      Eigen::Vector3d desired = Eigen::Vector3d::Zero();
      desired(axis) = sign * (axis == 2 ? 0.01 : 0.1);
      const auto output = allocator.allocate(desired.x(), desired.y(), desired.z());
      EXPECT_LT((realized(output) - desired).norm(), 1e-10);
    }
    }
    for (double force : allocator.allocate(0.0, 0.0, 0.0)) {
    EXPECT_DOUBLE_EQ(force, 0.0);
    }
}

TEST(BoundedAllocator, SaturatesWithoutInventingNegativeThrust)
{
    ac::ControlAllocator allocator(nominal_thrusters());
    const auto wrench = realized(allocator.allocate(10.0, 0.0, 0.0));
    EXPECT_NEAR(wrench.x(), std::sqrt(0.5), 1e-10);
    EXPECT_NEAR(wrench.y(), 0.0, 1e-10);
    EXPECT_NEAR(wrench.z(), 0.0, 1e-10);
    EXPECT_THROW(allocator.allocate(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
        std::invalid_argument);
}

TEST(PlanarTrajectory, StartsAtRestWithContinuousAcceleration)
{
    for (const std::string type : {"line", "circle"}) {
    ac::TrajectoryConfig config;
    config.type = type;
    config.offset_m << 0.02, -0.01;
    ac::PlanarTrajectory trajectory(config);
    const Eigen::Vector2d origin(0.5, 1.0);
    const auto initial = trajectory.sample(origin, 0.0);
    EXPECT_LT((initial.position - origin).norm(), 1e-12);
    EXPECT_LT(initial.velocity.norm(), 1e-12);
    EXPECT_LT(initial.acceleration.norm(), 1e-12);
    const auto before = trajectory.sample(origin, config.startup_s - 1e-7);
    const auto after = trajectory.sample(origin, config.startup_s + 1e-7);
    EXPECT_LT((before.acceleration - after.acceleration).norm(), 1e-7);
    }
}

TEST(PlanarTrajectory, AnalyticDerivativesMatchFiniteDifferences)
{
    for (const std::string type : {"line", "circle"}) {
    ac::TrajectoryConfig config;
    config.type = type;
    config.direction_rad = 0.7;
    ac::PlanarTrajectory trajectory(config);
    const Eigen::Vector2d origin(0.5, 1.0);
    const double h = 1e-4;
    for (double t : {0.3, 1.0, 4.9, 5.1, 11.0, 22.0}) {
      const auto sample = trajectory.sample(origin, t);
      const auto before = trajectory.sample(origin, t - h);
      const auto after = trajectory.sample(origin, t + h);
      EXPECT_LT(((after.position - before.position) / (2 * h) -
        sample.velocity).norm(), 1e-8);
      EXPECT_LT(((after.velocity - before.velocity) / (2 * h) -
        sample.acceleration).norm(), 1e-8);
    }
    }
}

TEST(PlanarTrajectory, CircleHasSpecifiedRadiusAfterStartup)
{
    ac::TrajectoryConfig config;
    config.type = "circle";
    config.offset_m << 0.01, 0.02;
    ac::PlanarTrajectory trajectory(config);
    const Eigen::Vector2d origin(0.5, 1.0);
    const Eigen::Vector2d center = origin + config.offset_m - Eigen::Vector2d(config.radius_m, 0);
    for (double t = config.startup_s; t < 40.0; t += 0.2) {
    EXPECT_NEAR((trajectory.sample(origin, t).position - center).norm(),
            config.radius_m, 1e-12);
    }
}

TEST(PlanarTrajectory, FixedCircleCenterAndFullRadiusThroughoutStartup)
{
  ac::TrajectoryConfig config;
  config.type = "circle";
  config.radius_m = 0.2;
  config.absolute_circle_center = true;
  config.circle_center_m << 0.5, 1.0;
  ac::PlanarTrajectory trajectory(config);
  const Eigen::Vector2d origin(0.7, 1.0);
  for (double t = 0; t < 30.0; t += 0.02) {
    const auto sample = trajectory.sample(origin, t);
    EXPECT_NEAR((sample.position - config.circle_center_m).norm(), config.radius_m, 1e-12);
  }
  const auto initial = trajectory.sample(origin, 0.0);
  EXPECT_LT((initial.position - origin).norm(), 1e-12);
  EXPECT_LT(initial.velocity.norm(), 1e-12);
  EXPECT_LT(initial.acceleration.norm(), 1e-12);
  const auto at_speed = trajectory.sample(origin, config.startup_s);
  EXPECT_NEAR(at_speed.velocity.norm(), config.radius_m * 2 * std::acos(-1.0) /
    config.period_s, 1e-12);
}

TEST(PlanarTrajectory, OffCircleInitialPositionJoinsTheSameFixedCircle)
{
  ac::TrajectoryConfig config;
  config.type = "circle";
  config.absolute_circle_center = true;
  config.circle_center_m << 0.5, 1.0;
  ac::PlanarTrajectory trajectory(config);
  const Eigen::Vector2d origin(0.45, 1.05);
  EXPECT_LT((trajectory.sample(origin, 0).position - origin).norm(), 1e-12);
  for (double t = 0.3; t < 25; t += 0.3) {
    const double h = 1e-4;
    const auto sample = trajectory.sample(origin, t);
    const auto before = trajectory.sample(origin, t - h);
    const auto after = trajectory.sample(origin, t + h);
    EXPECT_LT(((after.position - before.position) / (2 * h) - sample.velocity).norm(), 1e-8);
    EXPECT_LT(((after.velocity - before.velocity) / (2 * h) - sample.acceleration).norm(), 1e-8);
    if (t >= config.startup_s) {
      EXPECT_NEAR((sample.position - config.circle_center_m).norm(), config.radius_m, 1e-12);
    }
  }
}

TEST(TrackingControl, ConvertsBodyVelocityAndTableForceAtNonzeroYaw)
{
    ac::TrackingGains gains;
    gains.kp = 1.0;
    gains.max_accel = 100.0;
    const double yaw = std::acos(-1.0) / 2;
    ac::PlanarReference reference{{0, 1}, {0, 0.1}, {0, 0}};
    const auto wrench = ac::tracking_wrench(reference, {0, 0}, {0.1, 0}, yaw, 0, yaw, gains);
    EXPECT_NEAR(wrench.x(), gains.mass_kg, 1e-10);
    EXPECT_NEAR(wrench.y(), 0.0, 1e-10);
    EXPECT_NEAR(wrench.z(), 0.0, 1e-10);
}

TEST(TrackingControl, WrapsYawErrorAndBoundsAcceleration)
{
    ac::TrackingGains gains;
    const double pi = std::acos(-1.0);
    ac::PlanarReference reference{{100, 100}, {0, 0}, {0, 0}};
    const auto wrench = ac::tracking_wrench(reference, {0, 0}, {0, 0},
        pi - 0.01, 0, -pi + 0.01, gains);
    EXPECT_NEAR(wrench.head<2>().norm(), gains.mass_kg * gains.max_accel, 1e-12);
    EXPECT_NEAR(wrench.z(), gains.inertia_kg_m2 * gains.yaw_kp * 0.02, 1e-12);
}

TEST(PulseDensity, PreservesConstantAverageImpulseWithinOneQuantum)
{
    ac::PulseDensityModulator modulator({0.5, 0.5, 0.5});
    std::vector<int> counts(3, 0);
    const std::vector<double> requested{0.0007, 0.17, 0.5};
    for (int tick = 0; tick < 10000; ++tick) {
    for (auto index : modulator.step(requested)) {
      ++counts[index];
    }
    }
    for (int i = 0; i < 3; ++i) {
    const double error_in_pulses = requested[i] / 0.5 * 10000 - counts[i];
    EXPECT_GE(error_in_pulses, -1e-9);
    EXPECT_LT(error_in_pulses, 1.0);
    }
}

TEST(PulseDensity, ClearsResidualAndRejectsInvalidCommands)
{
    ac::PulseDensityModulator modulator({0.5});
    EXPECT_TRUE(modulator.step({0.4}).empty());
    EXPECT_TRUE(modulator.step({0.0}).empty());
    EXPECT_TRUE(modulator.step({0.2}).empty());
    modulator.reset();
    EXPECT_TRUE(modulator.step({0.2}).empty());
    EXPECT_THROW(modulator.step({-0.1}), std::invalid_argument);
    EXPECT_THROW(modulator.step({0.6}), std::invalid_argument);
    EXPECT_THROW(modulator.step({std::numeric_limits<double>::quiet_NaN()}),
        std::invalid_argument);
}
