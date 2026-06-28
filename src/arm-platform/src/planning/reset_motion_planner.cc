#include <manipulator/planning/reset_motion_planner.h>
#include <iostream>
namespace manipulator::planning {
void ResetMotionPlanner::Plan(const JointStates& joint_states) {
  // reset joint 4,5,6 to zero
  Reset();
  std::vector<double> goal;
  for (int i = 0; i < 3; ++i) {
    goal.push_back(joint_states.position[i]);
  }
  for (int i = 3; i < 7; ++i) {
    goal.push_back(0.0);
  }
  this->PlanCore(joint_states.position, goal);
  std::vector<double> last_goal = goal;
  goal[2] = -1.3; // reset joint 3 to < -90 degree
  this->PlanCore(last_goal, goal);

  last_goal = goal;
  goal[1] = -1.57; // reset joint 2 to -90 degree
  this->PlanCore(last_goal, goal);

  last_goal = goal;
  goal[0] = 0; // reset joint 1 to zero
  this->PlanCore(last_goal, goal);

  last_goal = goal;
  goal[1] = 1.2; // reset joint 2 to <90 degree
  this->PlanCore(last_goal, goal);
}

} // namespace manipulator::planning
