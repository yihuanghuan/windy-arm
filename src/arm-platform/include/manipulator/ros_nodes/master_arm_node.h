#pragma once

#include <memory>
#include <vector>
#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <dummy_interface/msg/motor_state.hpp>
#include <dummy_interface/msg/motor_control.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <manipulator/robotics/arm_data_subscriber.h>
#include <manipulator/robotics/arm_platform.h>

namespace manipulator {

static constexpr double kControlPeriodMs = 10.0;

class MasterArmNode : public rclcpp::Node, public IArmDataSubscriber {
 public:
  MasterArmNode();
  ~MasterArmNode();
  void Init();

  void UpdateJointState(sensor_msgs::msg::JointState& msg) override;
  void UpdateMotorFeedback(dummy_interface::msg::MotorState& msg) override;

 private:
  template<typename T>
  T GetParam(const std::string& name, T default_value) {
    if (!this->has_parameter(name)) {
      this->declare_parameter<T>(name, default_value);
    }
    return this->get_parameter(name).get_value<T>();
  }

  // void DebugInfoCallback();
  void Reset();
  void SetArmPlatform();
  void PublishHealth();

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
  rclcpp::Publisher<dummy_interface::msg::MotorState>::SharedPtr pub_joint_feedback_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_health_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_uav_pose_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_slave_state_;

  ArmPlatform::UniPtr arm_platform_;

  bool auto_reset_ = true;
  bool publish_joint_state_ = true;
  bool publish_joint_feedback_ = true;

  std::array<double, 7> uav_joint_currents = {0};
  std::array<double, 7> uav_compensation_torques = {0};
};

} // namespace manipulator
