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
#include <manipulator/controller/gravity_controller.h>
#include <manipulator/ros_nodes/speed_safety.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>

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
  dummy_interface::msg::MotorControl MakeHoldPositionCommand(
      bool include_gravity = true);
  std::vector<double> ComputeHoldGravityCurrent();
  void ExecuteEffortGuarded(const dummy_interface::msg::MotorControl& cmd);
  bool FeedbackSafe() const;
  void UpdateSpeedSafety();
  void PublishAppliedCommand(const dummy_interface::msg::MotorControl& cmd);
  void UpdateNoTouchGate();
  void SetupHandover();
  void MaskUnsafeHoldFeedforward(dummy_interface::msg::MotorControl& hold) const;

 public:
  /**
   * @brief 构造"位置保持"兜底指令：默认 kp/kd + 保持给定位置 + 零前馈
   *
   * 旧实现发 p=0,d=0,current=0 —— 在 MIT 模式下同时杀掉位置环，机械臂
   * 直接在重力下垮落。而力矩通道的超时(默认 0.05s)会被扫描程序里任何
   * >50ms 的间隙常规触发，等于每次实验都会垮一次臂。位置保持才是兜底。
   *
   * 静态方法以便单元测试（不依赖 ROS 节点状态）。
   */
  static dummy_interface::msg::MotorControl BuildHoldCommand(
      const std::vector<double>& positions,
      const std::vector<double>& p_gains,
      const std::vector<double>& d_gains) {
    const size_t n = positions.size();
    dummy_interface::msg::MotorControl cmd;
    cmd.position = positions;
    cmd.velocity.assign(n, 0.0);
    cmd.current.assign(n, 0.0);            // 兜底状态绝不输出前馈力矩
    cmd.p.resize(n);
    cmd.d.resize(n);
    for (size_t i = 0; i < n; ++i) {
      // 增益向量为空/过短时退回一个保守正增益，绝不允许落到 0：
      // MIT 模式下 p=d=0 意味着该关节彻底失力。
      cmd.p[i] = (i < p_gains.size() && p_gains[i] > 0.0) ? p_gains[i] : 5.0;
      cmd.d[i] = (i < d_gains.size() && d_gains[i] > 0.0) ? d_gains[i] : 0.1;
    }
    cmd.enable = 0;
    return cmd;
  }

  /**
   * @brief 非零力矩前馈是否必须被拦下
   *
   * 串口心跳断了(link_alive=false)意味着关节反馈已经陈旧，基于模型算出的
   * 力矩不可信；此时只允许位置保持，不允许任何非零 current。
   */
  static bool ShouldBlockTorque(bool link_alive,
                                const dummy_interface::msg::MotorControl& cmd) {
    if (link_alive) {
      return false;
    }
    for (double c : cmd.current) {
      if (c != 0.0) {
        return true;
      }
    }
    return false;
  }

 private:

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
  bool read_only_ = false;
  bool familiarization_mode_ = false;
  bool wait_for_first_effort_command_ = false;
  bool feedback_fault_latched_ = false;
  bool motor_command_sent_ = false;
  std::vector<double> last_sent_effort_;
  double feedback_timeout_sec_ = 0.05;
  double feedback_velocity_limit_ = 0.5;
  std::vector<double> last_known_velocity_;
  std::unique_ptr<SpeedSafety> speed_safety_;
  bool last_output_guide_ = false;
  size_t speed_feedback_generation_ = 0;
  std::array<bool, 6> speed_fault_axes_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_speed_safety_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_speed_fault_;
  bool bumpless_handover_ = false, operator_hold_ = false, awaiting_handover_ = false;
  bool manual_push_active_ = false, trial_counted_ = false, has_seed_ = false;
  double prepare_stamp_ = 0., quiet_since_ = -1.;
  double quiet_duration_ = 0., quiet_tolerance_ = 0., quiet_speed_ = 0.;
  double handover_joint_margin_ = 0.;
  int completed_handovers_ = 0, required_handovers_ = 3;
  std::vector<double> quiet_q_, seed_expected_q_;
  double initial_hold_position_tolerance_ = .03;
  dummy_interface::msg::MotorControl last_applied_command_, initial_hold_command_, expected_first_command_;
  std::vector<double> previous_total_;
  rclcpp::Publisher<dummy_interface::msg::MotorControl>::SharedPtr pub_applied_command_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_support_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prepare_service_, operator_hold_service_, manual_service_;

  bool got_command_ = false;
  bool holding_ = false;
  bool direct_effort_enabled_ = false;
  bool allow_position_handover_ = false;
  bool step_on_effort_command_ = false;
  bool got_effort_command_ = false;
  bool effort_zeroed_ = false;
  // 直驱模式绝不能把构造时的全零缓存当作实物位姿。启动时先只读串口，
  // 收到心跳后再连续确认数帧关节反馈，随后才允许发送任何保持/力矩命令。
  bool direct_startup_feedback_ready_ = false;
  bool direct_startup_link_seen_ = false;
  size_t direct_startup_feedback_generation_ = 0;
  size_t joint_feedback_generation_ = 0;
  // 保持目标只锁存一次，避免「每周期重新以当前位置为基准」的棘轮下垂
  bool effort_hold_latched_ = false;
  double last_command_stamp_ = 0.0;
  double last_effort_command_stamp_ = 0.0;
  double last_initial_state_publish_stamp_ = 0.0;
  double command_timeout_sec_ = 1.0;
  double effort_command_timeout_sec_ = 0.05;
  double effort_control_period_sec_ = 0.01;
  double heartbeat_timeout_sec_ = 0.2;   // 板子心跳 20Hz，0.2s = 丢 4 拍
  bool publish_joint_feedback_ = false;
  dummy_interface::msg::MotorControl last_effort_command_;
  // 兜底保持所用增益，取自 p_gain/d_gain 参数（与位置模式一致）
  std::vector<double> hold_p_gain_;
  std::vector<double> hold_d_gain_;
  // 直驱力矩通道的兜底保持不能只有 PD：承载关节会在命令超时后下垂。
  // 重力模型只用于串口心跳正常、已有实测反馈时的保持；链路陈旧时严格退回
  // 零前馈位置保持，避免以陈旧姿态施加错误力矩。
  bool gravity_hold_enabled_ = false;
  std::unique_ptr<controller::GravityController> hold_gravity_controller_;
  std::vector<double> gravity_hold_zero_offset_rad_;
  std::vector<double> gravity_hold_torque_limit_;
};

}  // namespace manipulator
