#pragma once

#include <cstddef>
#include <vector>

namespace airtable_controller
{

// Yaw hold with a speed-commanded reaction wheel (a continuous-rotation servo standing in
// for the ARW-3m). The output is a normalized wheel speed command: + turns the wheel CCW,
// which turns the body CW, so a target CCW of the current yaw gives a negative command.
// The actuator gain (body rate per unit command) is unknown and absorbed into the gains.
struct RwAttitudeGains
{
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
    // Bound of |ki * integral| in output units (anti-windup).
    double integral_limit{0.0};
    // Bound of |output|, at most 1.
    double output_limit{1.0};
    // Low-pass time constant of the yaw rate differenced from poses. 0 disables the filter.
    double rate_filter_tau_s{0.0};
    // Pose gaps longer than this skip the integral and rate update.
    double max_dt_s{0.5};

    // Throws std::invalid_argument on negative or non-finite values.
    void validate() const;
};

class RwAttitudeController
{
public:
    explicit RwAttitudeController(const RwAttitudeGains & gains);

    // Holds target_yaw from now on and clears the integral and rate history.
    void reset(double target_yaw_rad);

    // yaw at time t (seconds, any monotonic origin). Returns the wheel speed command.
    double update(double yaw_rad, double t_s);

    double target() const {return target_;}
    double error() const {return error_;}
    double rate() const {return rate_;}
    double output() const {return output_;}

private:
    RwAttitudeGains gains_;
    double target_{0.0};
    double integral_{0.0};
    double rate_{0.0};
    double error_{0.0};
    double output_{0.0};
    double previous_yaw_{0.0};
    double previous_t_{0.0};
    bool has_previous_{false};
};

// Discrete manual wheel speeds, stepped one at a time like the former ESP32 firmware.
class SpeedLevels
{
public:
    // Ascending, distinct, within [-1, 1] and containing 0; throws std::invalid_argument.
    explicit SpeedLevels(std::vector<double> levels);

    // +1 selects the next faster CCW level, -1 the next CW one; stops at the ends.
    double step(int direction);
    void stop();
    double value() const {return levels_[index_];}

private:
    std::vector<double> levels_;
    std::size_t zero_index_{0};
    std::size_t index_{0};
};

double wrap_yaw(double radians);

// Moves current toward target by at most max_rate_per_s * dt_s. max_rate_per_s <= 0: no limit.
double slew(double current, double target, double max_rate_per_s, double dt_s);

}  // namespace airtable_controller
