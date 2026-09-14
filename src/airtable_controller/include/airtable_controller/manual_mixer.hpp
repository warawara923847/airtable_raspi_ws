#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace airtable_controller
{
    class ManualMixer
    {
    public:
        std::vector<std::string> mix(
                std::uint8_t command) const;
    };
}
