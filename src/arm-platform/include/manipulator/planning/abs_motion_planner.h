#pragma once

#include <manipulator/planning/trajectory/i_trajectory_generator.h>
#include <manipulator/robotics/arm/abs_arm.h>
#include <memory>
#include <vector>
#include <Eigen/Dense>

namespace manipulator {
namespace planning {

using JointStates = manipulator::arm::JointState;

class AbsMotionPlanner {
 public:
  using UniPtr = std::unique_ptr<AbsMotionPlanner>;
  AbsMotionPlanner() = default;
  virtual ~AbsMotionPlanner() = default;

  virtual void Plan(const JointStates& joint_states) = 0;

  void SetTrajectoryGenerator(ITrajectoryGenerator::UniPtr trajectory_generator);
  JointSetpoint GetTrajectoryPoint();
  bool IsDone() const;

 protected:
  virtual void PlanCore(const std::vector<double>& start,
                        const std::vector<double>& goal,
                        double v_max = 0,
                        double a_max = 0,
                        double j_max = 0);
  void Reset();
  
  ITrajectoryGenerator::UniPtr trajectory_generator_;
  JointTrajectory trajectory_;
  Eigen::Index current_index_ = 0;
};

} // namespace planning
} // namespace manipulator
