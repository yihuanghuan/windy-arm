#pragma once

#include <memory>
#include <vector>
#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <dummy_interface/msg/motor_state.hpp>
#include <manipulator/robotics/arm_data_subscriber.h>
#include <manipulator/robotics/arm_platform.h>

namespace manipulator {

static constexpr size_t kJointCount = 7;
static constexpr double kControlPeriodMs = 10.0;
static const std::array<std::string, kJointCount> kJointNames = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};

class ArmHardwareNode : public rclcpp::Node, public IArmDataSubscriber {
 public:
  ArmHardwareNode();
  ~ArmHardwareNode();

  void UpdateJointState(sensor_msgs::msg::JointState& msg) override;
  void UpdateMotorFeedback(dummy_interface::msg::MotorState& msg) override;

  void Init();

 private:
  template<typename T>
  T GetParam(const std::string& name, T default_value) {
    if (!this->has_parameter(name)) {
      this->declare_parameter<T>(name, default_value);
    }
    return this->get_parameter(name).get_value<T>();
  }
  // Main control loop, runs periodically to send commands and publish state
  void SetArmPlatform();
  void MoveItCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<dummy_interface::msg::MotorState>::SharedPtr pub_joint_feedback_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_ctrl_;
  ArmPlatform::UniPtr arm_platform_;
  arm::JointState arm_current_state_;
};

} // namespace manipulator
