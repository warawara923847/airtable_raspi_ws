#pragma once

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace airtable_controller
{

struct ThrusterConfig
{
  std::string name;

  double x_m;
  double y_m;

  double dir_x;
  double dir_y;

  double min_thrust_n;
  double max_thrust_n;
};

class ControlAllocator
{
public:
  explicit ControlAllocator(
    const std::vector<ThrusterConfig> & thrusters);

  std::vector<double> allocate(
    double fx_n, double fy_n, double mz_nm) const;

private:
  std::vector<ThrusterConfig> thrusters_;

  Eigen::MatrixXd allocation_matrix_;
  struct Face
  {
    Eigen::MatrixXd gain;
    Eigen::VectorXd offset;
  };
  Eigen::Vector3d weights_;
  std::vector<Face> faces_;
};
}
