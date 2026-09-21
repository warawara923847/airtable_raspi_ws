#pragma once

#include <string>

namespace airtable_controller
{

struct ReactionWheelConfig
{
    std::string name;

    double speed_kp;
    double max_torque_nm;
    double max_speed_rad_s ;
};

class ReactionWheelController
{
public:
    explicit ReactionWheelController(
        const ReactionWheelConfig & config);

    double compute_torque(
            double target_speed_rad_s,
            double current_speed_rad_s) const;

    double max_speed_rad_s() const;

private:
    ReactionWheelConfig config_;
};

} // namespace airtable_controller
