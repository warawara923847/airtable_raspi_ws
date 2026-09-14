#include "airtable_controller/control_allocator.hpp"

#include <algorithm>
#include <stdexcept>

namespace airtable_controller
{
    ControlAllocator::ControlAllocator(
        const std::vector<ThrusterConfig> & thrusters)
    : thrusters_(thrusters)
    {
        if (thrusters_.empty()) {
            throw std::runtime_error(
                "No thrusters configured");
        }

        allocation_matrix_.resize(
            3,
            thrusters_.size());

        for (std::size_t i = 0; i < thrusters_.size(); ++i) {
            const auto & t = thrusters_[i];

            allocation_matrix_(0, i) = t.dir_x;
            allocation_matrix_(1, i) = t.dir_y;
            allocation_matrix_(2, i) = t.x_m * t.dir_y - t.y_m * t.dir_x;
        }

        pseudo_inverse_ =
            allocation_matrix_
                .completeOrthogonalDecomposition()
                .pseudoInverse();
    }

    std::vector<double> ControlAllocator::allocate(
        double fx_n, double fy_n, double mz_nm) const
    {
        Eigen::Vector3d desired_wrench;

        desired_wrench << fx_n, fy_n, mz_nm;

        Eigen::VectorXd thrust =
            pseudo_inverse_ * desired_wrench;

        std::vector<double> result(
            thrusters_.size());

        for (std::size_t i = 0; i < thrusters_.size(); ++i)
        {
            result[i] = std::clamp(
                thrust(i),
                thrusters_[i].min_thrust_n,
                thrusters_[i].max_thrust_n);
        }

        return result;
    }

} // namespace airtable_controller
