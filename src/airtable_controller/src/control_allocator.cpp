#include "airtable_controller/control_allocator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace airtable_controller
{

ControlAllocator::ControlAllocator(const std::vector<ThrusterConfig> & thrusters)
: thrusters_(thrusters)
{
    // Precompute the box-face least-squares problems for the small thruster array.
  if (thrusters_.empty() || thrusters_.size() > 8) {
    throw std::invalid_argument("Bounded allocation supports 1..8 thrusters");
  }
  const auto count = thrusters_.size();
  allocation_matrix_.resize(3, count);
  double arm = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    auto & t = thrusters_[i];
    const double norm = std::hypot(t.dir_x, t.dir_y);
    if (!std::isfinite(t.x_m) || !std::isfinite(t.y_m) ||
      !std::isfinite(norm) || norm <= 0.0 ||
      !std::isfinite(t.min_thrust_n) || !std::isfinite(t.max_thrust_n) ||
      t.min_thrust_n < 0.0 || t.max_thrust_n < t.min_thrust_n)
    {
      throw std::invalid_argument("Invalid thruster geometry or limits");
    }
    t.dir_x /= norm;
    t.dir_y /= norm;
    const double moment = t.x_m * t.dir_y - t.y_m * t.dir_x;
    allocation_matrix_.col(i) << t.dir_x, t.dir_y, moment;
    arm = std::max(arm, std::abs(moment));
  }
    // Express torque errors as equivalent forces at the nominal moment arm.
  weights_ << 1.0, 1.0, 1.0 / std::max(arm, 1e-3);
  const Eigen::MatrixXd weighted = weights_.asDiagonal() * allocation_matrix_;
  std::size_t face_count = 1;
  for (std::size_t i = 0; i < count; ++i) {
    face_count *= 3;
  }
  faces_.reserve(face_count);
  for (std::size_t code = 0; code < face_count; ++code) {
    std::size_t remaining = code;
    std::vector<std::size_t> free;
    Eigen::VectorXd fixed = Eigen::VectorXd::Zero(count);
    for (std::size_t i = 0; i < count; ++i) {
      const auto status = remaining % 3;
      remaining /= 3;
      if (status == 0) {
        free.push_back(i);
      } else {
        fixed(i) = status == 1 ? thrusters_[i].min_thrust_n :
          thrusters_[i].max_thrust_n;
      }
    }
    Eigen::MatrixXd gain = Eigen::MatrixXd::Zero(count, 3);
    if (!free.empty()) {
      Eigen::MatrixXd reduced(3, free.size());
      for (std::size_t j = 0; j < free.size(); ++j) {
        reduced.col(j) = weighted.col(free[j]);
      }
      const Eigen::MatrixXd inverse =
        reduced.completeOrthogonalDecomposition().pseudoInverse();
      for (std::size_t j = 0; j < free.size(); ++j) {
        gain.row(free[j]) = inverse.row(j);
      }
    }
    faces_.push_back({gain, fixed - gain * weighted * fixed});
  }
}

std::vector<double> ControlAllocator::allocate(double fx_n, double fy_n, double mz_nm) const
{
  Eigen::Vector3d requested(fx_n, fy_n, mz_nm);
  if (!requested.allFinite()) {
    throw std::invalid_argument("Wrench must be finite");
  }
  const Eigen::Vector3d desired = weights_.asDiagonal() * requested;
  const Eigen::MatrixXd weighted = weights_.asDiagonal() * allocation_matrix_;
  double best_error = std::numeric_limits<double>::infinity();
  double best_effort = std::numeric_limits<double>::infinity();
  Eigen::VectorXd best;
  for (const auto & face : faces_) {
    Eigen::VectorXd candidate = face.gain * desired + face.offset;
    bool feasible = true;
    for (std::size_t i = 0; i < thrusters_.size(); ++i) {
      const auto & t = thrusters_[i];
      if (candidate(i) < t.min_thrust_n - 1e-10 ||
        candidate(i) > t.max_thrust_n + 1e-10)
      {
        feasible = false;
        break;
      }
      candidate(i) = std::clamp(candidate(i), t.min_thrust_n, t.max_thrust_n);
    }
    if (!feasible) {
      continue;
    }
    const double error = (weighted * candidate - desired).squaredNorm();
        // Equal tracking errors prefer lower total propellant, including at zero wrench.
    const double effort = candidate.sum();
    if (error < best_error - 1e-18 ||
      (std::abs(error - best_error) <= 1e-18 && effort < best_effort))
    {
      best = candidate;
      best_error = error;
      best_effort = effort;
    }
  }
  return std::vector<double>(best.data(), best.data() + best.size());
}

}  // namespace airtable_controller
