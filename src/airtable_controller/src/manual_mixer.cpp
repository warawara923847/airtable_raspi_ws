#include "airtable_controller/manual_mixer.hpp"

#include <airtable_interfaces/msg/manual_motion_command.hpp>

namespace airtable_controller
{

    std::vector<std::string> ManualMixer::mix(std::uint8_t command) const
    {
        using Command = airtable_interfaces::msg::ManualMotionCommand;

        switch (command){
            case Command::FORWARD:
                return {
                    "thruster_2",
                    "thruster_3",
                };

            case Command::BACKWARD:
                return {
                    "thruster_1",
                    "thruster_4",
                };

            case Command::RIGHT:
                return {
                    "thruster_3",
                    "thruster_4",
                };

            case Command::LEFT:
                return {
                    "thruster_1",
                    "thruster_2",
                };

            case Command::THRUSTER_ROTATE_CCW:
                return {
                    "thruster_2",
                    "thruster_4",
                };

            case Command::THRUSTER_ROTATE_CW:
                return {
                    "thruster_1",
                    "thruster_3",
                };

            default:
                return {};
        }
    }        
}
