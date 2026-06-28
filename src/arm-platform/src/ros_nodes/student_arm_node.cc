#include <manipulator/ros_nodes/student_arm_node.h>
#include <manipulator/robotics/arm/arm_factory.h>
#include <manipulator/controller/smooth_position_controller.h>
#include <manipulator/common_types.h>

#include <algorithm>
#include <cmath>
#include <chrono>

namespace manipulator {

static constexpr double kStudentControlPeriodMs = 10.0;

StudentArmNode::StudentArmNode() : Node("student_arm_node") {
  command_timeout_sec_ = GetParam<double>("command_timeout_sec", 1.0);
  publish_joint_feedback_ = GetParam<bool>("publish_joint_feedback", false);
  joint_lower_limits_ = GetParam<std::vector<double>>(
      "joint_lower_limits", {-4.0, -3.1415, -1.5708, -4.0, -2.0, -4.0, -4.0});
  joint_upper_limits_ = GetParam<std::vector<double>>(
      "joint_upper_limits", {4.0, 2.0, 1.5708, 4.0, 2.0, 4.0, 4.0});
  joint_count_ = joint_lower_limits_.size();

  pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  pub_joint_feedback_ = this->create_publisher<dummy_interface::msg::MotorState>(
      "/student/joint_feedback", 10);

  sub_student_cmd_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/student/joint_command", 10,
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
        StudentCommandCallback(msg);
      });

  arm_platform_ = std::make_unique<ArmPlatform>();
  SetArmPlatform();

  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(kStudentControlPeriodMs)),
      [this]() { ControlLoop(); });

  RCLCPP_INFO(this->get_logger(),
              "StudentArmNode started. Publish sensor_msgs/JointState to "
              "/student/joint_command to control the arm.");
}

StudentArmNode::~StudentArmNode() {
  control_timer_->cancel();
}

void StudentArmNode::SetArmPlatform() {
  std::string port = GetParam<std::string>("port_name", "/dev/ttyUSB0");
  std::string arm_type = GetParam<std::string>("arm_type", "sim");
  std::string arm_version = GetParam<std::string>("arm_version", "gamma");
  std::string motor_config_path = GetParam<std::string>("motor_config_path", "");
  std::string arm_config_path = GetParam<std::string>("arm_config_path", "");

  auto arm = arm::ArmFactory::Instance().Create(arm_type);
  if (!motor_config_path.empty() && !arm_config_path.empty()) {
    arm->InitFromConfig(port, 921600, motor_config_path, arm_config_path, arm_version);
    RCLCPP_INFO(this->get_logger(), "Arm type '%s' initialized from config", arm_type.c_str());
  } else {
    arm->Init(port, 921600);
    RCLCPP_INFO(this->get_logger(), "Arm type '%s' initialized", arm_type.c_str());
  }
  arm_platform_->SetArm(std::move(arm));

  auto controller = std::make_unique<controller::SmoothPositionController>();
  // 学生模式默认限低速，保证安全
  double max_velocity = GetParam<double>("max_velocity", 0.5);
  controller->SetMaxVelocity(max_velocity);
  controller->SetJointLimits(joint_lower_limits_, joint_upper_limits_);
  controller::VelocityFeedforwardConfig velocity_feedforward_config;
  velocity_feedforward_config.enable_velocity_feedforward =
      GetParam<bool>("enable_velocity_feedforward", false);
  velocity_feedforward_config.velocity_feedforward_gain =
      GetParam<double>("velocity_feedforward_gain", 0.7);
  velocity_feedforward_config.position_feedback_tau =
      GetParam<double>("position_feedback_tau", 0.15);
  velocity_feedforward_config.feedback_velocity_limit =
      GetParam<double>("feedback_velocity_limit", 0.4);
  velocity_feedforward_config.enable_acceleration_limit =
      GetParam<bool>("enable_acceleration_limit", true);
  velocity_feedforward_config.max_acceleration =
      GetParam<double>("max_acceleration", 5.0);
  controller->SetVelocityFeedforwardConfig(velocity_feedforward_config);
  std::vector<double> p_gain = GetParam<std::vector<double>>(
      "p_gain", {30, 30, 30, 5, 5, 5, 1});
  std::vector<double> d_gain = GetParam<std::vector<double>>(
      "d_gain", {1, 1, 1, 0.1, 0.1, 0.1, 0.1});
  controller->SetKpKd(p_gain, d_gain);
  RCLCPP_INFO(
      this->get_logger(),
      "SmoothPositionController: max_velocity=%.3f, velocity_feedforward=%s, "
      "ff_gain=%.3f, fb_tau=%.3f, fb_vel_limit=%.3f, acceleration_limit=%s, "
      "max_acceleration=%.3f",
      max_velocity,
      velocity_feedforward_config.enable_velocity_feedforward ? "true" : "false",
      velocity_feedforward_config.velocity_feedforward_gain,
      velocity_feedforward_config.position_feedback_tau,
      velocity_feedforward_config.feedback_velocity_limit,
      velocity_feedforward_config.enable_acceleration_limit ? "true" : "false",
      velocity_feedforward_config.max_acceleration);
  arm_platform_->SetController(std::move(controller));
}

void StudentArmNode::StudentCommandCallback(
    const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
  // 安全检查：维度
  if (msg->position.size() != joint_count_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Rejected command: expected %zu joint positions, got %zu",
        joint_count_, msg->position.size());
    return;
  }
  // 安全检查：NaN / Inf
  for (double p : msg->position) {
    if (!std::isfinite(p)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Rejected command: position contains NaN/Inf");
      return;
    }
  }

  planning::JointSetpoint setpoint;
  setpoint.q.resize(joint_count_);
  setpoint.dq.resize(joint_count_);
  for (size_t i = 0; i < joint_count_; ++i) {
    // 安全层：关节限位钳制
    double q = std::clamp(msg->position[i], joint_lower_limits_[i], joint_upper_limits_[i]);
    if (q != msg->position[i]) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Joint %zu command %.3f clamped to [%.3f, %.3f]",
          i + 1, msg->position[i], joint_lower_limits_[i], joint_upper_limits_[i]);
    }
    setpoint.q[i] = q;
    setpoint.dq[i] = (i < msg->velocity.size() && std::isfinite(msg->velocity[i]))
                         ? msg->velocity[i] : 0.0;
  }
  arm_platform_->SetJointSetpoint(setpoint);

  got_command_ = true;
  holding_ = false;
  last_command_stamp_ = now().seconds();
}

void StudentArmNode::ControlLoop() {
  // 安全层：指令超时后保持当前位置
  if (got_command_ && !holding_ &&
      now().seconds() - last_command_stamp_ > command_timeout_sec_) {
    if (last_known_position_.size() == joint_count_) {
      planning::JointSetpoint hold;
      hold.q.resize(joint_count_);
      hold.dq = Eigen::VectorXd::Zero(joint_count_);
      for (size_t i = 0; i < joint_count_; ++i) {
        hold.q[i] = last_known_position_[i];
      }
      arm_platform_->SetJointSetpoint(hold);
    }
    holding_ = true;
    RCLCPP_INFO(this->get_logger(), "Command timeout, holding current position");
  }
  arm_platform_->ExecuteControlCycle(kStudentControlPeriodMs / 1000.0);
}

void StudentArmNode::UpdateJointState(sensor_msgs::msg::JointState& msg) {
  last_known_position_ = msg.position;
  msg.header.stamp = this->now();
  pub_joint_state_->publish(msg);
}

void StudentArmNode::UpdateMotorFeedback(dummy_interface::msg::MotorState& msg) {
  if (!publish_joint_feedback_) {
    return;
  }
  msg.header.stamp = this->now();
  pub_joint_feedback_->publish(msg);
}

void StudentArmNode::Init() {
  auto sub = std::dynamic_pointer_cast<IArmDataSubscriber>(shared_from_this());
  if (sub) {
    arm_platform_->AddSubscribe(sub);
  }
}

}  // namespace manipulator

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulator::StudentArmNode>();
  node->Init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
