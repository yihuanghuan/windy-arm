#pragma once

#include <memory>
#include <vector>
#include <array>
#include <string>
#include <functional>
#include <mutex>

#include <sensor_msgs/msg/joint_state.hpp>
#include <dummy_interface/msg/motor_state.hpp>
#include <dummy_interface/msg/motor_control.hpp>

#include <manipulator/robotics/arm/abs_arm.h>
#include <manipulator/robotics/arm_data_subscriber.h>
#include <manipulator/controller/i_arm_controller.h>
#include <manipulator/planning/abs_motion_planner.h>
#include <manipulator/collision/collision_avoidance.h>

namespace manipulator {

class ArmPlatform {
 public:
  using UniPtr = std::unique_ptr<ArmPlatform>;

  explicit ArmPlatform();
  ~ArmPlatform();

  void SetArm(arm::AbsArm::UniPtr arm);
  void SetController(controller::IArmController::UniPtr controller);
  void SetPlanner(planning::AbsMotionPlanner::UniPtr planner);
  void SetJointSetpoint(const planning::JointSetpoint& setpoint);
  void SetCollisionAvoidance(collision::CollisionAvoidance::Ptr collision_avoidance);
  void EnableCollisionAvoidance(bool enable);

  bool ExecuteControlCycle(double dt);
  bool ExecuteDirectMotorCommand(
      const dummy_interface::msg::MotorControl& cmd, double dt);
  void PublishCurrentState();
  void SetBaseState(const arm::FloatingBaseState& state);

  /**
   * @brief 转发到 arm 的串口心跳检查；未设置 arm 时视为不活（保守）
   */
  bool IsArmLinkAlive(double timeout_sec) const {
    return arm_ && arm_->IsLinkAlive(timeout_sec);
  }
  bool HasFreshFeedback(double timeout_sec) const {
    return arm_ && arm_->HasFreshFeedback(timeout_sec);
  }
  uint64_t GetFeedbackSequence() const { return arm_->GetFeedbackSequence(); }
  std::vector<std::string> GetJointNames() const { return arm_->GetJointNames(); }
  void AddSubscribe(IArmDataSubscriber::SharedPtr subscriber);
  void PrintDebugInfo();

 private:
  void NotifyJointState();
  void NotifyMotorFeedback();
  void NotifyFreshState();
  bool ApplyCollisionAvoidance() const;

  arm::AbsArm::UniPtr arm_;
  arm::JointState arm_state_;
  controller::IArmController::UniPtr controller_;
  planning::AbsMotionPlanner::UniPtr planner_;
  collision::CollisionAvoidance::Ptr collision_avoidance_;
  dummy_interface::msg::MotorControl cmd_;
  planning::JointSetpoint joint_setpoint_;

  std::vector<IArmDataSubscriber::WeakPtr> subscribers_;
  std::vector<std::string> joint_names_;
  bool collision_avoidance_enabled_ = false;
  uint64_t published_feedback_sequence_ = 0;
};

}  // namespace manipulator
