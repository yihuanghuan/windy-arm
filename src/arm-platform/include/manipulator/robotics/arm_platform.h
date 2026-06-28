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
  void AddSubscribe(IArmDataSubscriber::SharedPtr subscriber);
  void PrintDebugInfo();

 private:
  void NotifyJointState();
  void NotifyMotorFeedback();
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
};

}  // namespace manipulator
