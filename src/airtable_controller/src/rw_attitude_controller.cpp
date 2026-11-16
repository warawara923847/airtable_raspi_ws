#include "airtable_controller/rw_attitude_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace airtable_controller
{

void RwAttitudeGains::validate() const
{
    for (const double value : {kp, ki, kd, integral_limit, rate_filter_tau_s}) {
        if (!std::isfinite(value) || value < 0.0) {
            throw std::invalid_argument("RW attitude gains and limits must be finite and >= 0");
        }
    }
    if (!std::isfinite(output_limit) || output_limit <= 0.0 || output_limit > 1.0) {
        throw std::invalid_argument("RW attitude output_limit must be within (0, 1]");
    }
    if (!std::isfinite(max_dt_s) || max_dt_s <= 0.0) {
        throw std::invalid_argument("RW attitude max_dt_s must be positive");
    }
}

RwAttitudeController::RwAttitudeController(const RwAttitudeGains & gains)
: gains_(gains)
{
    gains_.validate();
}

void RwAttitudeController::reset(double target_yaw_rad)
{
    target_ = wrap_yaw(target_yaw_rad);
    integral_ = 0.0;
    rate_ = 0.0;
    error_ = 0.0;
    output_ = 0.0;
    has_previous_ = false;
}

double RwAttitudeController::update(double yaw_rad, double t_s)
{
    double dt = 0.0;
    if (has_previous_) {
        dt = t_s - previous_t_;
        if (dt <= 0.0) {
            // Duplicate or out-of-order pose: keep the last command.
            return output_;
        }
        if (dt > gains_.max_dt_s) {
            dt = 0.0;
        }
    }
    if (dt > 0.0) {
        const double raw_rate = wrap_yaw(yaw_rad - previous_yaw_) / dt;
        const double alpha = dt / (gains_.rate_filter_tau_s + dt);
        rate_ += alpha * (raw_rate - rate_);
    }
    previous_yaw_ = yaw_rad;
    previous_t_ = t_s;
    has_previous_ = true;

    error_ = wrap_yaw(target_ - yaw_rad);

    // Body-frame demand: + means "turn the body CCW". D acts on the measured rate so that a
    // target change does not kick.
    auto body_demand = [&](double integral) {
        return gains_.kp * error_ + gains_.ki * integral - gains_.kd * rate_;
    };
    if (dt > 0.0 && gains_.ki > 0.0) {
        const double limit = gains_.integral_limit / gains_.ki;
        const double candidate = std::clamp(integral_ + error_ * dt, -limit, limit);
        const double demand = body_demand(candidate);
        // Conditional integration: stop integrating further into saturation.
        const bool saturating = std::abs(demand) > gains_.output_limit &&
            std::signbit(demand) == std::signbit(error_);
        if (!saturating) {
            integral_ = candidate;
        }
    }
    // The wheel turns opposite to the body.
    output_ = std::clamp(-body_demand(integral_), -gains_.output_limit, gains_.output_limit);
    return output_;
}

SpeedLevels::SpeedLevels(std::vector<double> levels)
: levels_(std::move(levels))
{
    if (levels_.empty()) {
        throw std::invalid_argument("speed levels must not be empty");
    }
    bool has_zero = false;
    for (std::size_t i = 0; i < levels_.size(); ++i) {
        const double level = levels_[i];
        if (!std::isfinite(level) || std::abs(level) > 1.0 ||
            (i > 0 && level <= levels_[i - 1]))
        {
            throw std::invalid_argument(
                      "speed levels must be ascending, distinct and within [-1, 1]");
        }
        if (level == 0.0) {
            has_zero = true;
            zero_index_ = i;
        }
    }
    if (!has_zero) {
        throw std::invalid_argument("speed levels must contain 0 (stop)");
    }
    index_ = zero_index_;
}

double SpeedLevels::step(int direction)
{
    if (direction > 0 && index_ + 1 < levels_.size()) {
        ++index_;
    } else if (direction < 0 && index_ > 0) {
        --index_;
    }
    return value();
}

void SpeedLevels::stop()
{
    index_ = zero_index_;
}

double wrap_yaw(double radians)
{
    return std::atan2(std::sin(radians), std::cos(radians));
}

double slew(double current, double target, double max_rate_per_s, double dt_s)
{
    if (max_rate_per_s <= 0.0) {
        return target;
    }
    const double step = max_rate_per_s * std::max(dt_s, 0.0);
    return current + std::clamp(target - current, -step, step);
}

}  // namespace airtable_controller
