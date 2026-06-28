#pragma once
#include <manipulator/common_types.h>
#include <manipulator/planning/trajectory/i_trajectory_generator.h>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace manipulator::planning {

class SCurveGenerator : public ITrajectoryGenerator {
 public:
  SCurveGenerator() = default;
  ~SCurveGenerator() override = default;

  void Generate(
      JointTrajectory& traj,
      const std::vector<double>& start,
      const std::vector<double>& goal,
      double v_max,
      double a_max,
      double j_max) override {
    uint16_t N = static_cast<uint16_t>(traj.q.cols());
    
    size_t num_joints = start.size();
    if (num_joints == 0 || start.size() != goal.size()) {
      return;
    }
    
    double max_displacement = 0.0;
    for (size_t i = 0; i < num_joints; ++i) {
      max_displacement = std::max(max_displacement, std::abs(goal[i] - start[i]));
    }

    
    if (max_displacement < 1e-6) {
      traj.q.conservativeResize(num_joints, N + 1);
      traj.dq.conservativeResize(num_joints, N + 1);

      for (size_t i = 0; i < num_joints; ++i) {
        traj.q(i, N) = start[i];
      }
      return;
    }
    
    double duration = 0.3;
    double dt = 0.001;
    size_t steps = static_cast<size_t>(duration / dt) + 1;
    
    traj.q.conservativeResize(num_joints, N + steps);
    traj.dq.conservativeResize(num_joints, N + steps); 
    
    for (size_t i = 0; i < num_joints; ++i) {
      double start_pos = start[i];
      double end_pos = goal[i];
      double displacement = end_pos - start_pos;
      double t = 0.0;
      for (size_t ind = 0; ind < steps; ind++) {
        double tau = t / duration;
        double tau2 = tau * tau;
        double tau3 = tau2 * tau;
        double tau4 = tau3 * tau;
        double tau5 = tau4 * tau;
        
        double ratio = 10 * tau3 - 15 * tau4 + 6 * tau5;
        double vel_ratio = (30 * tau2 - 60 * tau3 + 30 * tau4) / duration;
        
        traj.q(i, N + ind) = start_pos + displacement * ratio;
        traj.dq(i, N + ind) = displacement * vel_ratio;
  
        t += dt;
      }
    }
  }
};

} // namespace manipulator::planning
