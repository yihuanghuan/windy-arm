#pragma once

#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <dummy_interface/msg/motor_control.hpp>
#include <dummy_interface/msg/floating_base_state.hpp>
#include <dummy_interface/msg/motor_state.hpp>
#include <manipulator/robotics/arm_data_subscriber.h>
#include <manipulator/robotics/arm_platform.h>

namespace manipulator {

/**
 * @brief 学生机械臂控制接口节点
 *
 * 学生通过发布 /student/joint_command (sensor_msgs/JointState) 控制机械臂，
 * 本节点发布 /joint_states 供 robot_state_publisher / RViz 可视化。
 *
 * 安全层：
 *  - 关节限位钳制
 *  - 拒绝 NaN / 维度错误的非法指令
 *  - 指令超时后保持当前位置
 *  - SmoothPositionController 限速
 *
 * 通过参数 arm_type 切换虚拟臂 (sim) 与真实硬件 (a_l1)。
 */
class StudentArmNode : public rclcpp::Node, public IArmDataSubscriber {
 public:
  StudentArmNode();
  ~StudentArmNode();
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

  void SetArmPlatform();
  void StudentCommandCallback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg);
  void StudentEffortCommandCallback(const dummy_interface::msg::MotorControl::ConstSharedPtr& msg);
  void FloatingBaseStateCallback(
      const dummy_interface::msg::FloatingBaseState::ConstSharedPtr& msg);
  void ControlLoop();
  bool ValidateEffortCommand(const dummy_interface::msg::MotorControl& msg);
  dummy_interface::msg::MotorControl MakeZeroEffortCommand();

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
  rclcpp::Publisher<dummy_interface::msg::MotorState>::SharedPtr pub_joint_feedback_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_student_cmd_;
  rclcpp::Subscription<dummy_interface::msg::MotorControl>::SharedPtr sub_student_effort_cmd_;
  rclcpp::Subscription<dummy_interface::msg::FloatingBaseState>::SharedPtr sub_base_state_;

  ArmPlatform::UniPtr arm_platform_;

  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> last_known_position_;
  size_t joint_count_ = 7;

  bool got_command_ = false;
  bool holding_ = false;
  bool direct_effort_enabled_ = false;
  bool step_on_effort_command_ = false;
  bool got_effort_command_ = false;
  bool effort_zeroed_ = false;
  double last_command_stamp_ = 0.0;
  double last_effort_command_stamp_ = 0.0;
  double last_initial_state_publish_stamp_ = 0.0;
  double command_timeout_sec_ = 1.0;
  double effort_command_timeout_sec_ = 0.05;
  double effort_control_period_sec_ = 0.01;
  bool publish_joint_feedback_ = false;
  dummy_interface::msg::MotorControl last_effort_command_;
};

}  // namespace manipulator
