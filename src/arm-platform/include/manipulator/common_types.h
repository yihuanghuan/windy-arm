#pragma once
#include <vector>
#include <string>
#include <Eigen/Dense>

namespace manipulator {

namespace arm {
struct JointState {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> current;
  std::vector<double> voltage;
  std::vector<double> temperature;
  std::vector<std::string> name;
};
}
namespace planning {

struct JointTrajectory {
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> q;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> dq;
};

struct JointSetpoint {
  Eigen::Matrix<double, Eigen::Dynamic, 1> q;
  Eigen::Matrix<double, Eigen::Dynamic, 1> dq;
};

} // namespace planning

} // namespace manipulator
