#include "airtable_controller/reaction_wheel_controller.hpp"

#include <algorithm>

namespace airtable_controller
{

ReactionWheelController::ReactionWheelController(
    const ReactionWheelConfig & config) : config_(config)
{
}

double ReactionWheelController::compute_torque(
    double target_speed_rad_s,
    double current_speed_rad_s) const
{
    // Compute the speed error
    const double error = target_speed_rad_s - current_speed_rad_s;

    // Limit the torque command to the maximum torque
    return std::clamp(
        config_.speed_kp * error,
        -config_.max_torque_nm,
        config_.max_torque_nm);
}

double ReactionWheelController::max_speed_rad_s() const
{
    return config_.max_speed_rad_s;
}

} // namespace airtable_controller
