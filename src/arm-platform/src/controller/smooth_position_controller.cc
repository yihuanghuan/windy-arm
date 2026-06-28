#include <manipulator/controller/smooth_position_controller.h>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace manipulator::controller {
namespace {
constexpr double kMinPositiveValue = 1e-6;
}

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
      ResetVelocityState(num_joints);
      return cmd;
    }

    if(pos_set_.size() != num_joints) {
      pos_set_ = joint_states.position;
    }

    if (
      velocity_feedforward_config_.enable_velocity_feedforward !=
      last_enable_velocity_feedforward_
    ) {
      ResetVelocityState(num_joints);
      pos_set_ = joint_states.position;
    }
    last_enable_velocity_feedforward_ =
      velocity_feedforward_config_.enable_velocity_feedforward;

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

    if (velocity_feedforward_config_.enable_velocity_feedforward) {
      if (!std::isfinite(dt) || dt <= 0.0) {
        ResetVelocityState(num_joints);
        return JointCommand();
      }
      if (last_velocity_cmd_.size() != num_joints) {
        ResetVelocityState(num_joints);
      }

      for (size_t i = 0; i < num_joints; ++i) {
        if (
          !std::isfinite(joint_states.position[i]) ||
          !std::isfinite(joint_setpoint.q[i])
        ) {
          ResetVelocityState(num_joints);
          return JointCommand();
        }
      }

      const auto& config = velocity_feedforward_config_;
      for (size_t i = 0; i < num_joints; ++i) {
        const double q_actual = joint_states.position[i];
        const double q_ref = joint_setpoint.q[i];
        double qdot_ref = 0.0;
        if (
          joint_setpoint.dq.size() > static_cast<Eigen::Index>(i) &&
          std::isfinite(joint_setpoint.dq[i])
        ) {
          qdot_ref = joint_setpoint.dq[i];
        }

        const double v_ff = std::clamp(qdot_ref, -max_velocity_, max_velocity_);
        const double v_fb = std::clamp(
          (q_ref - q_actual) / config.position_feedback_tau,
          -config.feedback_velocity_limit,
          config.feedback_velocity_limit
        );
        const double v_raw = std::clamp(
          config.velocity_feedforward_gain * v_ff + v_fb,
          -max_velocity_,
          max_velocity_
        );

        double v_cmd = v_raw;
        if (config.enable_acceleration_limit) {
          double v_last = last_velocity_cmd_[i];
          if (!std::isfinite(v_last)) {
            v_last = 0.0;
          }
          const double acceleration_step = config.max_acceleration * dt;
          v_cmd = std::clamp(
            v_raw,
            v_last - acceleration_step,
            v_last + acceleration_step
          );
        }

        double q_cmd = q_actual + v_cmd * dt;
        q_cmd = ClampToJointLimits(q_cmd, i);
        v_cmd = (q_cmd - q_actual) / dt;

        last_velocity_cmd_[i] = v_cmd;
        pos_set_[i] = q_cmd;
        cmd.position[i] = q_cmd;
        cmd.velocity[i] = v_cmd;
      }
      return cmd;
    }

    ResetVelocityState(num_joints);
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
  max_velocity_ = std::max(0.0, max_velocity);
}

void SmoothPositionController::SetJointLimits(
  const std::vector<double>& lower_limits,
  const std::vector<double>& upper_limits) {
  joint_lower_limits_ = lower_limits;
  joint_upper_limits_ = upper_limits;
}

void SmoothPositionController::SetVelocityFeedforwardConfig(
  const VelocityFeedforwardConfig& config) {
  velocity_feedforward_config_ = config;
  SanitizeVelocityFeedforwardConfig();
}

void SmoothPositionController::ResetVelocityState(size_t num_joints) {
  last_velocity_cmd_.assign(num_joints, 0.0);
}

double SmoothPositionController::ClampToJointLimits(double q, size_t joint_index) const {
  if (
    joint_index < joint_lower_limits_.size() &&
    joint_index < joint_upper_limits_.size() &&
    std::isfinite(joint_lower_limits_[joint_index]) &&
    std::isfinite(joint_upper_limits_[joint_index]) &&
    joint_lower_limits_[joint_index] <= joint_upper_limits_[joint_index]
  ) {
    return std::clamp(
      q,
      joint_lower_limits_[joint_index],
      joint_upper_limits_[joint_index]
    );
  }
  return q;
}

void SmoothPositionController::SanitizeVelocityFeedforwardConfig() {
  if (!std::isfinite(velocity_feedforward_config_.velocity_feedforward_gain)) {
    velocity_feedforward_config_.velocity_feedforward_gain = 0.0;
  }
  if (
    !std::isfinite(velocity_feedforward_config_.position_feedback_tau) ||
    velocity_feedforward_config_.position_feedback_tau <= 0.0
  ) {
    velocity_feedforward_config_.position_feedback_tau = kMinPositiveValue;
  }
  if (
    !std::isfinite(velocity_feedforward_config_.feedback_velocity_limit) ||
    velocity_feedforward_config_.feedback_velocity_limit < 0.0
  ) {
    velocity_feedforward_config_.feedback_velocity_limit = 0.0;
  }
  if (
    !std::isfinite(velocity_feedforward_config_.max_acceleration) ||
    velocity_feedforward_config_.max_acceleration < 0.0
  ) {
    velocity_feedforward_config_.max_acceleration = 0.0;
  }
}
} // namespace manipulator::controller
