#include <manipulator/ros_nodes/slave_arm_node.h>
#include <manipulator/robotics/arm/arm_factory.h>
#include <manipulator/controller/smooth_position_controller.h>
#include <manipulator/planning/reset_motion_planner.h>
#include <manipulator/planning/trajectory/scurve_generator.h>
#include <manipulator/collision/collision_avoidance.h>
#include <manipulator/common_types.h>
#include <Eigen/Geometry>
#include <chrono>
#include <thread>
#include <sstream>

namespace manipulator {

SlaveArmNode::SlaveArmNode()
    : Node("slave_arm_node"),
      got_feedback_(false) {
  
  bool debug_info = GetParam<bool>("debug_info", false);
  double debug_rate = GetParam<double>("debug_rate", 1.0);
  publish_joint_state_ = GetParam<bool>("publish_joint_state", true);
  publish_joint_feedback_ = GetParam<bool>("publish_joint_feedback", false);
  master_feedback_timeout_sec_ = GetParam<double>("master_feedback_timeout_sec", 1.0);

  pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  pub_joint_feedback_ = this->create_publisher<dummy_interface::msg::MotorState>("joint_feedback", 10);
  auto marker_qos = rclcpp::QoS(1).transient_local();
  pub_collision_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("collision_obstacles", marker_qos);
  pub_health_ = this->create_publisher<std_msgs::msg::String>("/health/slave_arm", 10);
  health_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { PublishHealth(); });

  sub_master_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/master/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
        MasterStateCallback(msg);
      });

  arm_platform_ = std::make_unique<ArmPlatform>();
  SetArmPlatform();
  SetCollisionAvoidance();
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(kControlPeriodMs)),
      [this]() { return arm_platform_->ExecuteControlCycle(kControlPeriodMs/1000.0f); });

  if (debug_info) {
    auto debug_period = std::chrono::duration<double>(1.0 / debug_rate);
    debug_timer_ = this->create_wall_timer(
        debug_period,
        [this]() { return arm_platform_->PrintDebugInfo(); });
  }

  RCLCPP_INFO(this->get_logger(), "SlaveArmNode initialized (controlled mode, 100Hz control loop)");
}

void SlaveArmNode::MasterStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
  planning::JointSetpoint joint_setpoint;
  size_t joint_num = msg->position.size();
  joint_setpoint.q.resize(joint_num);
  joint_setpoint.dq.resize(joint_num);
  for (size_t i = 0; i < joint_num; ++i) {
    joint_setpoint.q[i] = msg->position[i];
    joint_setpoint.dq[i] = msg->velocity[i];
  }
  arm_platform_->SetJointSetpoint(joint_setpoint);

  got_feedback_ = true;
  last_master_feedback_stamp_ = now().seconds();
}

void SlaveArmNode::SetArmPlatform() {  
  std::string port = GetParam<std::string>("port_name", "/dev/ttyUSB0");
  std::string arm_type = GetParam<std::string>("arm_type", "a_l1");
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
  
  auto smooth_position_controller = std::make_unique<controller::SmoothPositionController>();
  double max_velocity = GetParam<double>("max_velocity", 2);
  smooth_position_controller->SetMaxVelocity(max_velocity);
  std::vector<double> p_gain = GetParam<std::vector<double>>("p_gain", {30, 30, 30, 5, 5, 5, 1});
  std::vector<double> d_gain = GetParam<std::vector<double>>("d_gain", {1, 1, 1, 0.1, 0.1, 0.1, 0.1});
  smooth_position_controller->SetKpKd(p_gain, d_gain);
  arm_platform_->SetController(std::move(smooth_position_controller));
}

void SlaveArmNode::SetCollisionAvoidance() {
  std::string urdf_path = GetParam<std::string>("urdf_path", "");
  bool enable_collision_avoidance = GetParam<bool>("enable_collision_avoidance", false);
  double safety_distance = GetParam<double>("safety_distance", 0.1);
  std::string collision_marker_frame = GetParam<std::string>("collision_marker_frame", "base_link");
  if (enable_collision_avoidance && !urdf_path.empty()) {
    auto collision_avoidance = std::make_shared<collision::CollisionAvoidance>();
    if (collision_avoidance->LoadModel(urdf_path)) {
      collision_avoidance->SetSafetyDistance(safety_distance);
      
      std::vector<double> cylinder1_center = GetParam<std::vector<double>>("cylinder1_center", {0.0, -0.15, 0.0});
      double cylinder1_radius = GetParam<double>("cylinder1_radius", 0.05);
      double cylinder1_height = GetParam<double>("cylinder1_height", 0.1);
      std::vector<double> cylinder1_axis = GetParam<std::vector<double>>("cylinder1_axis", {0, 0, 1});
      
      std::vector<double> cylinder2_center = GetParam<std::vector<double>>("cylinder2_center", {0.0, 0.15, 0.0});
      double cylinder2_radius = GetParam<double>("cylinder2_radius", 0.05);
      double cylinder2_height = GetParam<double>("cylinder2_height", 0.1);
      std::vector<double> cylinder2_axis = GetParam<std::vector<double>>("cylinder2_axis", {0, 0, 1});
      
      if (cylinder1_center.size() == 3) {
        collision::Vec3 axis1 = (cylinder1_axis.size() == 3) 
          ? collision::Vec3(cylinder1_axis[0], cylinder1_axis[1], cylinder1_axis[2]) 
          : collision::Vec3(0, 0, 1);
        collision::Cylinder cyl1(
          collision::Vec3(cylinder1_center[0], cylinder1_center[1], cylinder1_center[2]),
          cylinder1_radius,
          cylinder1_height,
          axis1
        );
        collision_avoidance->AddCylinder(cyl1);
        RCLCPP_INFO(this->get_logger(), "Cylinder1: center=[%.2f, %.2f, %.2f], r=%.2f, h=%.2f",
          cylinder1_center[0], cylinder1_center[1], cylinder1_center[2],
          cylinder1_radius, cylinder1_height);
      }
      
      if (cylinder2_center.size() == 3) {
        collision::Vec3 axis2 = (cylinder2_axis.size() == 3) 
          ? collision::Vec3(cylinder2_axis[0], cylinder2_axis[1], cylinder2_axis[2]) 
          : collision::Vec3(0, 0, 1);
        collision::Cylinder cyl2(
          collision::Vec3(cylinder2_center[0], cylinder2_center[1], cylinder2_center[2]),
          cylinder2_radius,
          cylinder2_height,
          axis2
        );
        collision_avoidance->AddCylinder(cyl2);
        RCLCPP_INFO(this->get_logger(), "Cylinder2: center=[%.2f, %.2f, %.2f], r=%.2f, h=%.2f",
          cylinder2_center[0], cylinder2_center[1], cylinder2_center[2],
          cylinder2_radius, cylinder2_height);
      }
      
      collision_marker_cylinders_ = collision_avoidance->GetCylinders();
      collision_marker_frame_ = collision_marker_frame;
      PublishCollisionMarkers();
      collision_marker_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(500),
          [this]() { return PublishCollisionMarkers(); });
      arm_platform_->SetCollisionAvoidance(collision_avoidance);
      arm_platform_->EnableCollisionAvoidance(true);
      RCLCPP_INFO(this->get_logger(), "Collision avoidance enabled with safety distance: %.2f", safety_distance);
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to load URDF for collision avoidance, disabled");
    }
  }
}

void SlaveArmNode::PublishCollisionMarkers() {
  if (collision_marker_cylinders_.empty()) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  auto stamp = this->now();

  for (size_t i = 0; i < collision_marker_cylinders_.size(); ++i) {
    const auto& cylinder = collision_marker_cylinders_[i];
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = collision_marker_frame_;
    marker.header.stamp = stamp;
    marker.ns = "collision_cylinders";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = cylinder.center.x();
    marker.pose.position.y = cylinder.center.y();
    marker.pose.position.z = cylinder.center.z();

    collision::Vec3 axis = cylinder.axis;
    if (axis.norm() < 1e-9) {
      axis = collision::Vec3(0, 0, 1);
    }
    axis.normalize();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(collision::Vec3(0, 0, 1), axis);
    orientation.normalize();
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();

    marker.scale.x = cylinder.radius * 2.0;
    marker.scale.y = cylinder.radius * 2.0;
    marker.scale.z = cylinder.height;
    marker.color.r = 1.0;
    marker.color.g = 0.2;
    marker.color.b = 0.1;
    marker.color.a = 0.45;
    marker_array.markers.push_back(marker);
  }

  pub_collision_markers_->publish(marker_array);
}

SlaveArmNode::~SlaveArmNode() {
  control_timer_->cancel();
  if (debug_timer_) {
    debug_timer_->cancel();
  }
  if (collision_marker_timer_) {
    collision_marker_timer_->cancel();
  }
}

void SlaveArmNode::UpdateJointState(sensor_msgs::msg::JointState& msg) {
  msg.header.stamp = rclcpp::Clock().now();
  pub_joint_state_->publish(msg);
}

void SlaveArmNode::UpdateMotorFeedback(dummy_interface::msg::MotorState& msg) {
  msg.header.stamp = rclcpp::Clock().now();
  pub_joint_feedback_->publish(msg);
}

void SlaveArmNode::Init() {
  auto sub = std::dynamic_pointer_cast<IArmDataSubscriber>(shared_from_this());
  if (sub) {
    arm_platform_->AddSubscribe(sub);
  }
}

void SlaveArmNode::PublishHealth() {
  if (got_feedback_ && now().seconds() - last_master_feedback_stamp_ > master_feedback_timeout_sec_) {
    got_feedback_ = false;
  }

  std_msgs::msg::String msg;
  std::ostringstream payload;
  const char* status = got_feedback_ ? "OK" : "WARN";
  const char* code = got_feedback_ ? "OK" : "NO_MASTER_FEEDBACK";
  const char* message = got_feedback_ ? "running" : "waiting for master feedback";
  payload << "{";
  payload << "\"module\":\"slave_arm\",";
  payload << "\"status\":\"" << status << "\",";
  payload << "\"code\":\"" << code << "\",";
  payload << "\"message\":\"" << message << "\",";
  payload << "\"stamp\":" << now().seconds();
  payload << "}";
  msg.data = payload.str();
  pub_health_->publish(msg);
}
} // namespace manipulator

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulator::SlaveArmNode>();
  node->Init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}