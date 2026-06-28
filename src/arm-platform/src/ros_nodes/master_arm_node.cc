#include <manipulator/ros_nodes/master_arm_node.h>
#include <manipulator/robotics/arm/arm_factory.h>
#include <manipulator/controller/gravity_controller.h>
#include <manipulator/controller/smooth_position_controller.h>
#include <manipulator/planning/reset_motion_planner.h>
#include <manipulator/planning/trajectory/scurve_generator.h>
#include <sstream>

namespace manipulator {

MasterArmNode::MasterArmNode()
    : Node("master_arm_node") {
  bool debug_info = GetParam<bool>("debug_info", false);
  double debug_rate = GetParam<double>("debug_rate", 1.0);
  publish_joint_state_ = GetParam<bool>("publish_joint_state", true);
  publish_joint_feedback_ = GetParam<bool>("publish_joint_feedback", false);
  auto_reset_ = GetParam<bool>("auto_reset", true);
  
  pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10); 
  pub_joint_feedback_ = this->create_publisher<dummy_interface::msg::MotorState>("joint_feedback", 10);
  pub_health_ = this->create_publisher<std_msgs::msg::String>("/health/master_arm", 10);
  health_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { PublishHealth(); });
  
  sub_slave_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/slave/joint_states", 10,
    [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) { 
      size_t joint_num = msg->velocity.size();
      for (size_t i = 0; i < joint_num; ++i) {
        uav_compensation_torques[i] = msg->effort[i];
        uav_joint_currents[i] = msg->velocity[i];
      }
  });
  arm_platform_ = std::make_unique<ArmPlatform>();
  SetArmPlatform();
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(kControlPeriodMs)),
      [this]() { return arm_platform_->ExecuteControlCycle(kControlPeriodMs/1000.0f); });

  if (debug_info) {
    auto debug_period = std::chrono::duration<double>(1.0 / debug_rate);
    debug_timer_ = this->create_wall_timer(
        debug_period,
        [this]() { return arm_platform_->PrintDebugInfo(); });
  }
  RCLCPP_INFO(this->get_logger(), "MasterArmNode initialized (100Hz control loop)");
}

void MasterArmNode::SetArmPlatform() {
  std::string port = GetParam<std::string>("port_name", "/dev/ttyUSB0");
  std::string arm_type = GetParam<std::string>("arm_type", "a_l1");
  std::string arm_version = GetParam<std::string>("arm_version", "gamma");

  std::string motor_config_path = GetParam<std::string>("motor_config_path", "");
  std::string arm_config_path = GetParam<std::string>("arm_config_path", "");
  std::string urdf_path = GetParam<std::string>("urdf_path", "");
  
  auto gravity_controller = std::make_unique<manipulator::controller::GravityController>();
  if (!gravity_controller->LoadModel(urdf_path)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load URDF model from: %s", urdf_path.c_str());
  } 
  double gravity = GetParam<double>("GRAVITY", 9.81);
  double force_threshold = GetParam<double>("FORCE_FEEDBACK_THRESHOLD", 0.5);
  double force_gain = GetParam<double>("FORCE_FEEDBACK_GAIN", 0.5);
  gravity_controller->SetParams(gravity, force_threshold, force_gain);
  arm_platform_->SetController(std::move(gravity_controller));
  
  auto arm = arm::ArmFactory::Instance().Create(arm_type);
  if (!motor_config_path.empty() && !arm_config_path.empty()) {
    arm->InitFromConfig(port, 921600, motor_config_path, arm_config_path, arm_version);
  } else {
    arm->Init(port, 921600);
    RCLCPP_INFO(this->get_logger(), "Arm type '%s' initialized", arm_type.c_str());
  }
  arm_platform_->SetArm(std::move(arm));
}

MasterArmNode::~MasterArmNode() {
  control_timer_->cancel();
  if (debug_timer_) {
    debug_timer_->cancel();
  }
  if (auto_reset_) {
    Reset();
  }
}

void MasterArmNode::Reset() {
  RCLCPP_INFO(this->get_logger(), "Resetting arm to home position...");
  auto controller = std::make_unique<controller::SmoothPositionController>();
  auto trajectory_generator = std::make_unique<planning::SCurveGenerator>();
  auto planner = std::make_unique<planning::ResetMotionPlanner>();
  planner->SetTrajectoryGenerator(std::move(trajectory_generator));

  arm_platform_->SetController(std::move(controller));
  arm_platform_->SetPlanner(std::move(planner));
  while (!arm_platform_->ExecuteControlCycle(kControlPeriodMs/1000.0f)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(kControlPeriodMs)));
  }
}

void MasterArmNode::UpdateJointState(sensor_msgs::msg::JointState& msg) {
  msg.header.stamp = rclcpp::Clock().now();
  pub_joint_state_->publish(msg);
}

void MasterArmNode::UpdateMotorFeedback(dummy_interface::msg::MotorState& msg) {
  msg.header.stamp = rclcpp::Clock().now();
  if (publish_joint_feedback_) {
    pub_joint_feedback_->publish(msg);
  }
}

void MasterArmNode::Init() {
  auto sub = std::dynamic_pointer_cast<IArmDataSubscriber>(shared_from_this());
  if (sub) {
    arm_platform_->AddSubscribe(sub);
  }
}

void MasterArmNode::PublishHealth() {
  std_msgs::msg::String msg;
  std::ostringstream payload;
  payload << "{";
  payload << "\"module\":\"master_arm\",";
  payload << "\"status\":\"OK\",";
  payload << "\"code\":\"OK\",";
  payload << "\"message\":\"running\",";
  payload << "\"stamp\":" << now().seconds();
  payload << "}";
  msg.data = payload.str();
  pub_health_->publish(msg);
}
} // namespace manipulator


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulator::MasterArmNode>();
  node->Init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}