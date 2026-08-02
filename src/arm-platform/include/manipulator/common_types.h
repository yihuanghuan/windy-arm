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

struct FloatingBaseState {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Matrix<double, 6, 1> velocity =
      Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> acceleration =
      Eigen::Matrix<double, 6, 1>::Zero();
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
