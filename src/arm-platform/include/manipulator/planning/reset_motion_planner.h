#pragma once

#include <manipulator/planning/abs_motion_planner.h>

namespace manipulator::planning {

class ResetMotionPlanner : public AbsMotionPlanner {
 public:
  ResetMotionPlanner() = default;
  ~ResetMotionPlanner() override = default;

  void Plan(const JointStates& joint_states);
};

} // namespace manipulator::planning
