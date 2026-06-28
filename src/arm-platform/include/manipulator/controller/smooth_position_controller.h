#pragma once

#include <vector>
#include <manipulator/controller/i_arm_controller.h>

namespace manipulator::controller {
struct VelocityFeedforwardConfig {
  bool enable_velocity_feedforward = false;
  double velocity_feedforward_gain = 0.7;
  double position_feedback_tau = 0.15;
  double feedback_velocity_limit = 0.4;
  bool enable_acceleration_limit = true;
  double max_acceleration = 5.0;
};

class SmoothPositionController : public IArmController {
 public:
  SmoothPositionController();
  virtual ~SmoothPositionController() = default;
  void SetKpKd(const std::vector<double>& kp, const std::vector<double>& kd);
  void SetMaxVelocity(double max_velocity);
  void SetJointLimits(
      const std::vector<double>& lower_limits,
      const std::vector<double>& upper_limits);
  void SetVelocityFeedforwardConfig(const VelocityFeedforwardConfig& config);
  
  JointCommand Compute(
      const JointStates& joint_states,
      const JointSetpoint& joint_setpoint,
      double dt) override;
 private:
  void ResetVelocityState(size_t num_joints);
  double ClampToJointLimits(double q, size_t joint_index) const;
  void SanitizeVelocityFeedforwardConfig();

  std::vector<double> pos_set_;
  std::vector<double> last_velocity_cmd_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> kp_;
  std::vector<double> kd_;
  double max_velocity_ = 2.0;
  VelocityFeedforwardConfig velocity_feedforward_config_;
  bool last_enable_velocity_feedforward_ = false;
};
} // namespace manipulator::controller
