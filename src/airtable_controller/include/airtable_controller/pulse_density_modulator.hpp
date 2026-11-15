#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace airtable_controller
{

// One fixed-duration pulse per selected channel per tick. Fractional impulse
// carries over, allowing average thrusts smaller than one minimum pulse.
class PulseDensityModulator
{
public:
  explicit PulseDensityModulator(const std::vector<double> & on_thrust)
  : on_thrust_(on_thrust), credit_(on_thrust.size(), 0.0)
  {
    if (on_thrust.empty()) {
      throw std::invalid_argument("No pulse channels");
    }
    for (double value : on_thrust) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("ON thrust must be positive and finite");
      }
    }
  }

  std::vector<std::size_t> step(const std::vector<double> & thrust)
  {
    if (thrust.size() != credit_.size()) {
      throw std::invalid_argument("Pulse channel count mismatch");
    }
    std::vector<std::size_t> selected;
    for (std::size_t i = 0; i < thrust.size(); ++i) {
      if (!std::isfinite(thrust[i]) || thrust[i] < 0.0 || thrust[i] > on_thrust_[i]) {
        throw std::invalid_argument("Pulse thrust outside channel limits");
      }
      if (thrust[i] == 0.0) {
        credit_[i] = 0.0;
        continue;
      }
      credit_[i] += thrust[i] / on_thrust_[i];
      if (credit_[i] >= 1.0 - 1e-12) {
        selected.push_back(i);
        credit_[i] = std::max(0.0, credit_[i] - 1.0);
      }
    }
    return selected;
  }

  void reset()
  {
    std::fill(credit_.begin(), credit_.end(), 0.0);
  }

private:
  std::vector<double> on_thrust_;
  std::vector<double> credit_;
};

}  // namespace airtable_controller
