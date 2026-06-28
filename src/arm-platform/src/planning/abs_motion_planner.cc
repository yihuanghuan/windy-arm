#include <manipulator/planning/abs_motion_planner.h>
#include <stdexcept>
#include <iostream>

namespace manipulator::planning {

void AbsMotionPlanner::SetTrajectoryGenerator(ITrajectoryGenerator::UniPtr trajectory_generator) {
  trajectory_generator_ = std::move(trajectory_generator);
}

void AbsMotionPlanner::Reset() {
  current_index_ = 0;
}

void AbsMotionPlanner::PlanCore(const std::vector<double>& start,
                                const std::vector<double>& goal,
                                double v_max,
                                double a_max,
                                double j_max) {
  if (!trajectory_generator_) {
    throw std::runtime_error("Trajectory generator is not set");
  }
  trajectory_generator_->Generate(trajectory_, start, goal, v_max, a_max, j_max);
}

JointSetpoint AbsMotionPlanner::GetTrajectoryPoint() {
  JointSetpoint result;
  current_index_ = std::min(current_index_, trajectory_.q.cols() - 1);
  result.q = trajectory_.q.col(current_index_);
  result.dq = trajectory_.dq.col(current_index_);
  current_index_++;
  return result;
}

bool AbsMotionPlanner::IsDone() const {
  return current_index_ >= trajectory_.q.cols() - 1;
}

} // namespace manipulator::planning
