#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "airtable_controller/rw_attitude_controller.hpp"

namespace ac = airtable_controller;

static ac::RwAttitudeGains p_only(double kp)
{
    ac::RwAttitudeGains gains;
    gains.kp = kp;
    gains.output_limit = 1.0;
    return gains;
}

TEST(RwAttitude, TargetCcwOfBodyTurnsTheWheelCw)
{
    ac::RwAttitudeController pid(p_only(0.5));
    pid.reset(0.2);
    // Body must turn CCW, so the wheel turns CW (negative command).
    EXPECT_NEAR(pid.update(0.0, 0.0), -0.1, 1e-12);
    pid.reset(-0.2);
    EXPECT_NEAR(pid.update(0.0, 0.0), 0.1, 1e-12);
}

TEST(RwAttitude, ErrorWrapsAcrossPi)
{
    ac::RwAttitudeController pid(p_only(1.0));
    pid.reset(3.1);
    pid.update(-3.1, 0.0);
    // Shortest way from -3.1 to 3.1 is CW by 0.083 rad, not CCW by 6.2 rad.
    EXPECT_NEAR(pid.error(), 3.1 - (-3.1) - 2.0 * M_PI, 1e-12);
    EXPECT_GT(pid.output(), 0.0);
}

TEST(RwAttitude, OutputIsLimited)
{
    auto gains = p_only(10.0);
    gains.output_limit = 0.4;
    ac::RwAttitudeController pid(gains);
    pid.reset(1.0);
    EXPECT_DOUBLE_EQ(pid.update(0.0, 0.0), -0.4);
}

TEST(RwAttitude, IntegralIsBoundedAndStopsInSaturation)
{
    ac::RwAttitudeGains gains;
    gains.ki = 1.0;
    gains.integral_limit = 0.3;
    gains.output_limit = 1.0;
    ac::RwAttitudeController pid(gains);
    pid.reset(0.5);
    double t = 0.0;
    for (int i = 0; i < 1000; ++i, t += 0.02) {
        pid.update(0.0, t);
    }
    EXPECT_NEAR(pid.output(), -0.3, 1e-12);

    gains.integral_limit = 10.0;
    gains.output_limit = 0.2;
    ac::RwAttitudeController saturated(gains);
    saturated.reset(0.5);
    t = 0.0;
    for (int i = 0; i < 1000; ++i, t += 0.02) {
        saturated.update(0.0, t);
    }
    // Integration stopped once the output saturated, so the sign flips promptly.
    saturated.reset(0.5);
    saturated.update(0.5, 100.0);
    saturated.update(0.6, 100.02);
    EXPECT_GT(saturated.output(), 0.0);
}

TEST(RwAttitude, DerivativeActsOnMeasuredRate)
{
    ac::RwAttitudeGains gains;
    gains.kd = 1.0;
    ac::RwAttitudeController pid(gains);
    pid.reset(0.0);
    EXPECT_DOUBLE_EQ(pid.update(0.0, 0.0), 0.0);
    // Body turning CCW at 0.5 rad/s: damp by spinning the wheel CCW.
    EXPECT_NEAR(pid.update(0.01, 0.02), 0.5, 1e-9);
    // A long pose gap does not produce a rate spike.
    const double before = pid.rate();
    pid.update(0.5, 10.0);
    EXPECT_DOUBLE_EQ(pid.rate(), before);
}

TEST(RwAttitude, DuplicateStampKeepsTheLastCommand)
{
    ac::RwAttitudeController pid(p_only(1.0));
    pid.reset(0.3);
    const double first = pid.update(0.0, 1.0);
    EXPECT_DOUBLE_EQ(pid.update(0.2, 1.0), first);
}

// Speed-type wheel: the body rate follows the wheel speed with the opposite sign
// (momentum exchange) after a first-order servo lag.
TEST(RwAttitude, ClosedLoopWithSpeedWheelSettlesOnTarget)
{
    ac::RwAttitudeGains gains;
    gains.kp = 0.8;
    gains.ki = 0.1;
    gains.kd = 0.05;
    gains.integral_limit = 0.3;
    gains.output_limit = 0.6;
    gains.rate_filter_tau_s = 0.05;
    ac::RwAttitudeController pid(gains);
    pid.reset(0.5);

    const double dt = 0.02;
    const double body_rate_per_command = 1.5;  // rad/s at full wheel speed (unknown in practice)
    const double servo_tau = 0.15;
    double yaw = 0.0;
    double wheel = 0.0;
    double command = 0.0;
    for (int i = 0; i < 1500; ++i) {
        const double target = pid.update(yaw, i * dt);
        command = ac::slew(command, target, 2.0, dt);
        wheel += (command - wheel) * dt / servo_tau;
        yaw += -body_rate_per_command * wheel * dt;
    }
    EXPECT_NEAR(yaw, 0.5, 0.01);
    EXPECT_NEAR(command, 0.0, 0.02);
}

TEST(SpeedLevels, StepsWithinTheTableAndStops)
{
    ac::SpeedLevels levels({-0.5, -0.25, 0.0, 0.25, 0.5});
    EXPECT_DOUBLE_EQ(levels.value(), 0.0);
    EXPECT_DOUBLE_EQ(levels.step(1), 0.25);
    EXPECT_DOUBLE_EQ(levels.step(1), 0.5);
    EXPECT_DOUBLE_EQ(levels.step(1), 0.5);
    levels.stop();
    EXPECT_DOUBLE_EQ(levels.step(-1), -0.25);
    EXPECT_DOUBLE_EQ(levels.step(-1), -0.5);
    EXPECT_DOUBLE_EQ(levels.step(-1), -0.5);
}

TEST(SpeedLevels, RejectsInvalidTables)
{
    EXPECT_THROW(ac::SpeedLevels({}), std::invalid_argument);
    EXPECT_THROW(ac::SpeedLevels({-0.5, 0.5}), std::invalid_argument);
    EXPECT_THROW(ac::SpeedLevels({0.0, -0.5}), std::invalid_argument);
    EXPECT_THROW(ac::SpeedLevels({0.0, 1.5}), std::invalid_argument);
}

TEST(RwAttitude, RejectsInvalidGains)
{
    auto gains = p_only(-1.0);
    EXPECT_THROW(ac::RwAttitudeController{gains}, std::invalid_argument);
    gains = p_only(1.0);
    gains.output_limit = 1.5;
    EXPECT_THROW(ac::RwAttitudeController{gains}, std::invalid_argument);
}

TEST(Slew, LimitsTheChangePerStep)
{
    EXPECT_DOUBLE_EQ(ac::slew(0.0, 1.0, 2.0, 0.02), 0.04);
    EXPECT_DOUBLE_EQ(ac::slew(0.0, -1.0, 2.0, 0.02), -0.04);
    EXPECT_DOUBLE_EQ(ac::slew(0.0, 0.01, 2.0, 0.02), 0.01);
    EXPECT_DOUBLE_EQ(ac::slew(0.0, 1.0, 0.0, 0.02), 1.0);
}
