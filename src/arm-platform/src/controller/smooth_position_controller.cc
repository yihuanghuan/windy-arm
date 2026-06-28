#include <manipulator/controller/smooth_position_controller.h>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace manipulator::controller {
SmoothPositionController::SmoothPositionController() {
  kp_ = {10, 10, 10, 1, 1, 1, 1};
  kd_ = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
}
JointCommand SmoothPositionController::Compute(
  const JointStates& joint_states,
  const JointSetpoint& joint_setpoint,
  double dt) {
    JointCommand cmd;
    size_t num_joints = joint_states.position.size();

    if(joint_setpoint.q.size() == 0) {
      // return zero command if no setpoint
      return cmd;
    }

    if(pos_set_.empty()) {
      pos_set_ = joint_states.position;
    }

    cmd.position.resize(num_joints);
    cmd.velocity.resize(num_joints);
    cmd.current.resize(num_joints);
    cmd.p.resize(num_joints);

    for (size_t i = 0; i < num_joints; ++i) {
      cmd.p[i] = kp_[i];
    }
    cmd.d.resize(num_joints);

    for (size_t i = 0; i < num_joints; ++i) {
      cmd.d[i] = kd_[i];
    }
    
    if (joint_setpoint.q.size() < static_cast<Eigen::Index>(num_joints)) {
      std::cout << "Joint setpoint size" << joint_setpoint.q.size() 
        << " is smaller than joint position size " << num_joints << std::endl;
      return cmd;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      double pos_err = joint_setpoint.q[i] - joint_states.position[i];
      pos_err = std::clamp(pos_err, -max_velocity_ * dt, max_velocity_ * dt);
      // double delta = pos_err * dt;
      // // 最小步长限制
      // if (std::abs(delta) < 0.01) {
      //     delta = (delta >= 0) ? 0.01 : -0.01;
      // }
      pos_set_[i] += pos_err;
      // clamp（防止超调）
      pos_set_[i] = std::clamp(
        pos_set_[i],
        std::min(joint_setpoint.q[i], joint_states.position[i]),
        std::max(joint_setpoint.q[i], joint_states.position[i])
      );
      cmd.position[i] = pos_set_[i];
      double vel_ff = 0.0;
      if (
        joint_setpoint.dq.size() > static_cast<Eigen::Index>(i)
        && std::isfinite(joint_setpoint.dq[i])
      ) {
        vel_ff = std::clamp(joint_setpoint.dq[i], -max_velocity_, max_velocity_);
      }
      cmd.velocity[i] = vel_ff;
    }
  return cmd;
}

void SmoothPositionController::SetKpKd(const std::vector<double>& kp, const std::vector<double>& kd) {
  kp_ = kp;
  kd_ = kd;
}

void SmoothPositionController::SetMaxVelocity(double max_velocity) {
  max_velocity_ = max_velocity;
}
} // namespace manipulator::controller
