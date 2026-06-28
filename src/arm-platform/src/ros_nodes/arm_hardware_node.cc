#include <manipulator/ros_nodes/arm_hardware_node.h>
#include <manipulator/robotics/arm/arm_factory.h>
#include <manipulator/controller/smooth_position_controller.h>
#include <manipulator/common_types.h>

#include <sstream>

using namespace std::chrono_literals;

namespace manipulator {
namespace {
std::string FormatVector(const std::vector<double>& values) {
  std::ostringstream stream;
  stream << "[";
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      stream << ", ";
    }
    stream << values[i];
  }
  stream << "]";
  return stream.str();
}
}

ArmHardwareNode::ArmHardwareNode() : Node("robot_arm_node") {
  arm_platform_ = std::make_unique<ArmPlatform>();
  SetArmPlatform();
  pub_joint_feedback_ = this->create_publisher<dummy_interface::msg::MotorState>(
    "arm/joint_feedback", 10);
  sub_joint_ctrl_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      MoveItCallback(msg);
    });

  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(kControlPeriodMs)),
      [this]() { return arm_platform_->ExecuteControlCycle(kControlPeriodMs/1000.0f); });
}

ArmHardwareNode::~ArmHardwareNode() {
  control_timer_->cancel();
}

void ArmHardwareNode::SetArmPlatform() {
  std::string port = GetParam<std::string>("port_name", "/dev/ttyUSB0");
  std::string arm_type = GetParam<std::string>("arm_type", "a_l1");
  std::string arm_version = GetParam<std::string>("arm_version", "gamma");
  std::string motor_config_path = GetParam<std::string>("motor_config_path", "");
  std::string arm_config_path = GetParam<std::string>("arm_config_path", "");
  
  auto arm = arm::ArmFactory::Instance().Create(arm_type);
  if (!motor_config_path.empty() && !arm_config_path.empty()) {
    arm->InitFromConfig(port, 921600, motor_config_path, arm_config_path, arm_version);
  } else {
    arm->Init(port, 921600);
    RCLCPP_INFO(this->get_logger(), "Arm type '%s' initialized", arm_type.c_str());
  }

  arm_platform_->SetArm(std::move(arm));
  auto smooth_position_controller = std::make_unique<controller::SmoothPositionController>();
  std::vector<double> p_gain = GetParam<std::vector<double>>("p_gain", {30, 30, 30, 5, 5, 5, 1});
  std::vector<double> d_gain = GetParam<std::vector<double>>("d_gain", {1, 1, 1, 0.1, 0.1, 0.1, 0.1});
  smooth_position_controller->SetKpKd(p_gain, d_gain);
  const auto p_gain_text = FormatVector(p_gain);
  const auto d_gain_text = FormatVector(d_gain);
  RCLCPP_INFO(this->get_logger(), "ArmHardwareNode set p_gain: %s, d_gain: %s", p_gain_text.c_str(), d_gain_text.c_str());
  arm_platform_->SetController(std::move(smooth_position_controller));
}

void ArmHardwareNode::MoveItCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "ArmHardwareNode MoveItCallback");
  if (msg->name.empty()) {
    return;
  }

  planning::JointSetpoint joint_setpoint;
  joint_setpoint.q.resize(kJointCount);
  joint_setpoint.dq.resize(kJointCount);
  joint_setpoint.q.setZero();
  joint_setpoint.dq.setZero();

  for (size_t i = 0; i < msg->name.size(); ++i) {
    const auto& joint_name = msg->name[i];
    for (size_t j = 0; j < kJointCount; ++j) {
      if (joint_name == kJointNames[j]) {
        joint_setpoint.q[j] = msg->position[i];
        joint_setpoint.dq[j] = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0;
        break;
      }
    }
  }
  if (msg->name.size() == 6) {
    joint_setpoint.q[kJointCount - 1] = 0;
    joint_setpoint.dq[kJointCount - 1] = 0.0;
  }

  arm_platform_->SetJointSetpoint(joint_setpoint);
}

void ArmHardwareNode::UpdateJointState(sensor_msgs::msg::JointState& msg) {
  
}

void ArmHardwareNode::UpdateMotorFeedback(dummy_interface::msg::MotorState& msg) {
  msg.header.stamp = rclcpp::Clock().now();
  pub_joint_feedback_->publish(msg);
}

void ArmHardwareNode::Init() {
  auto sub = std::dynamic_pointer_cast<IArmDataSubscriber>(shared_from_this());
  if (sub) {
    arm_platform_->AddSubscribe(sub);
  }
}
} // namespace manipulator

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<manipulator::ArmHardwareNode>();
    node->Init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}