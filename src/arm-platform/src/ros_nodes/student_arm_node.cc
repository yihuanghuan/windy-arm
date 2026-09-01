#include <manipulator/ros_nodes/student_arm_node.h>
#include <manipulator/robotics/arm/arm_factory.h>
#include <manipulator/controller/smooth_position_controller.h>
#include <manipulator/common_types.h>

#include <algorithm>
#include <cmath>
#include <chrono>

namespace manipulator {

StudentArmNode::StudentArmNode() : Node("student_arm_node") {
  command_timeout_sec_ = GetParam<double>("command_timeout_sec", 1.0);
  effort_command_timeout_sec_ = GetParam<double>("effort_command_timeout_sec", 0.05);
  const double control_rate_hz =
      GetParam<double>("control_rate_hz", 100.0);
  if (!std::isfinite(control_rate_hz) || control_rate_hz <= 0.0) {
    throw std::invalid_argument("control_rate_hz must be finite and positive");
  }
  effort_control_period_sec_ = 1.0 / control_rate_hz;
  direct_effort_enabled_ = GetParam<bool>("enable_direct_effort_command", false);
  allow_position_handover_ = GetParam<bool>("allow_position_handover", false);
  step_on_effort_command_ =
      GetParam<bool>("step_on_effort_command", false);
  publish_joint_feedback_ = GetParam<bool>("publish_joint_feedback", false);
  gravity_hold_enabled_ = GetParam<bool>("gravity_hold_enabled", false);
  gravity_hold_zero_offset_rad_ = GetParam<std::vector<double>>(
      "gravity_hold_zero_offset_rad", {});
  gravity_hold_torque_limit_ = GetParam<std::vector<double>>(
      "gravity_hold_torque_limit", {});
  heartbeat_timeout_sec_ = GetParam<double>("heartbeat_timeout_sec", 0.2);
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
  sub_student_effort_cmd_ = this->create_subscription<dummy_interface::msg::MotorControl>(
      "/student/joint_effort_command", 10,
      [this](const dummy_interface::msg::MotorControl::ConstSharedPtr& msg) {
        StudentEffortCommandCallback(msg);
      });
  sub_base_state_ =
      this->create_subscription<dummy_interface::msg::FloatingBaseState>(
          "/floating_base/truth", 10,
          [this](
              const dummy_interface::msg::FloatingBaseState::ConstSharedPtr&
                  msg) { FloatingBaseStateCallback(msg); });

  arm_platform_ = std::make_unique<ArmPlatform>();
  SetArmPlatform();

  control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(effort_control_period_sec_)),
      [this]() { ControlLoop(); });

  RCLCPP_INFO(this->get_logger(),
              "StudentArmNode started. Publish sensor_msgs/JointState to "
              "/student/joint_command to control the arm.");
  RCLCPP_INFO(this->get_logger(),
              "Direct effort command topic /student/joint_effort_command is %s",
              direct_effort_enabled_ ? "enabled" : "disabled");
  if (step_on_effort_command_) {
    RCLCPP_INFO(
        this->get_logger(),
        "Torque simulation lockstep enabled: one plant step per effort command");
  }
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
  std::string urdf_path = GetParam<std::string>("urdf_path", "");
  std::vector<double> initial_joint_positions =
      GetParam<std::vector<double>>("initial_joint_positions", {});

  auto arm = arm::ArmFactory::Instance().Create(arm_type);
  arm->SetModelPath(urdf_path);
  arm->SetInitialJointPositions(initial_joint_positions);
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
  // 力矩通道的兜底保持使用与位置模式相同的增益
  hold_p_gain_ = p_gain;
  hold_d_gain_ = d_gain;
  if (gravity_hold_enabled_) {
    hold_gravity_controller_ = std::make_unique<controller::GravityController>();
    if (!hold_gravity_controller_->LoadModel(urdf_path)) {
      RCLCPP_ERROR(this->get_logger(),
          "gravity_hold_enabled requested but URDF gravity model failed to load; "
          "falling back to PD-only hold");
      hold_gravity_controller_.reset();
      gravity_hold_enabled_ = false;
    } else {
      RCLCPP_INFO(this->get_logger(),
          "Direct-effort fallback hold: gravity feedforward enabled");
    }
  }
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

void StudentArmNode::FloatingBaseStateCallback(
    const dummy_interface::msg::FloatingBaseState::ConstSharedPtr& msg) {
  arm::FloatingBaseState state;
  state.position =
      Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y,
                      msg->pose.position.z);
  state.orientation =
      Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                         msg->pose.orientation.y, msg->pose.orientation.z);
  if (state.orientation.norm() < 1e-12) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Rejected floating-base state with zero quaternion");
    return;
  }
  state.orientation.normalize();
  state.velocity <<
      msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
      msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
  state.acceleration <<
      msg->acceleration.linear.x, msg->acceleration.linear.y,
      msg->acceleration.linear.z, msg->acceleration.angular.x,
      msg->acceleration.angular.y, msg->acceleration.angular.z;
  if (!state.position.array().isFinite().all()
      || !state.velocity.array().isFinite().all()
      || !state.acceleration.array().isFinite().all()) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Rejected floating-base state containing NaN/Inf");
    return;
  }
  arm_platform_->SetBaseState(state);
}

void StudentArmNode::StudentCommandCallback(
    const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
  if (direct_effort_enabled_ && got_effort_command_ && !effort_zeroed_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Ignoring position command while direct effort command is active");
    return;
  }
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

bool StudentArmNode::ValidateEffortCommand(const dummy_interface::msg::MotorControl& msg) {
  if (!direct_effort_enabled_) {
    return false;
  }
  if (msg.enable == 0) {
    return true;
  }
  if (msg.current.size() != joint_count_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Rejected direct effort command: expected %zu current/torque values, got %zu",
        joint_count_, msg.current.size());
    return false;
  }
  auto finite_vector = [](const std::vector<double>& values) {
    return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
  };
  if (!finite_vector(msg.current) || !finite_vector(msg.p) || !finite_vector(msg.d) ||
      !finite_vector(msg.position) || !finite_vector(msg.velocity)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Rejected direct effort command: contains NaN/Inf");
    return false;
  }
  return true;
}

std::vector<double> StudentArmNode::ComputeHoldGravityCurrent() {
  std::vector<double> current(joint_count_, 0.0);
  if (!gravity_hold_enabled_ || !hold_gravity_controller_ ||
      last_known_position_.size() != joint_count_) {
    return current;
  }
  arm::JointState state;
  state.position = last_known_position_;
  state.velocity.assign(joint_count_, 0.0);
  for (size_t i = 0; i < state.position.size() && i < gravity_hold_zero_offset_rad_.size(); ++i) {
    state.position[i] += gravity_hold_zero_offset_rad_[i];
  }
  planning::JointSetpoint unused_setpoint;
  try {
    const auto gravity = hold_gravity_controller_->Compute(state, unused_setpoint, 0.0);
    for (size_t i = 0; i < current.size() && i < gravity.current.size(); ++i) {
      const double limit = (i < gravity_hold_torque_limit_.size() &&
                            gravity_hold_torque_limit_[i] > 0.0)
          ? gravity_hold_torque_limit_[i] : 0.0;
      current[i] = limit > 0.0
          ? std::clamp(gravity.current[i], -limit, limit) : gravity.current[i];
    }
  } catch (const std::exception& exc) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "gravity fallback calculation failed: %s", exc.what());
  }
  return current;
}

dummy_interface::msg::MotorControl StudentArmNode::MakeHoldPositionCommand(
    bool include_gravity) {
  std::vector<double> positions(joint_count_, 0.0);
  for (size_t i = 0; i < joint_count_; ++i) {
    positions[i] = (i < last_known_position_.size()) ? last_known_position_[i] : 0.0;
  }
  auto cmd = BuildHoldCommand(positions, hold_p_gain_, hold_d_gain_);
  if (include_gravity) {
    cmd.current = ComputeHoldGravityCurrent();
    // Hardware ignores enable, but keeping it asserted documents that this is
    // an active MIT hold rather than a user-requested torque disable.
    cmd.enable = 1;
  }
  cmd.header.stamp = this->now();
  return cmd;
}

void StudentArmNode::ExecuteEffortGuarded(
    const dummy_interface::msg::MotorControl& cmd) {
  const bool alive = arm_platform_->IsArmLinkAlive(heartbeat_timeout_sec_);
  if (ShouldBlockTorque(alive, cmd)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "serial heartbeat stale (> %.2fs): blocking torque feedforward, "
        "holding position instead", heartbeat_timeout_sec_);
    arm_platform_->ExecuteDirectMotorCommand(
        MakeHoldPositionCommand(false), effort_control_period_sec_);
    return;
  }
  arm_platform_->ExecuteDirectMotorCommand(cmd, effort_control_period_sec_);
}

void StudentArmNode::StudentEffortCommandCallback(
    const dummy_interface::msg::MotorControl::ConstSharedPtr& msg) {
  if (!ValidateEffortCommand(*msg)) {
    return;
  }
  if (msg->enable == 0) {
    last_effort_command_ = MakeHoldPositionCommand();
    got_effort_command_ = true;
    effort_zeroed_ = true;
    last_effort_command_stamp_ = now().seconds();
    if (step_on_effort_command_) {
      ExecuteEffortGuarded(last_effort_command_);
    }
    return;
  }

  last_effort_command_ = *msg;
  if (last_effort_command_.position.size() != joint_count_) {
    last_effort_command_.position.resize(joint_count_);
    for (size_t i = 0; i < joint_count_; ++i) {
      last_effort_command_.position[i] =
          (i < last_known_position_.size()) ? last_known_position_[i] : 0.0;
    }
  }
  if (last_effort_command_.velocity.size() != joint_count_) {
    last_effort_command_.velocity.assign(joint_count_, 0.0);
  }
  if (last_effort_command_.p.size() != joint_count_) {
    last_effort_command_.p.assign(joint_count_, 0.0);
  }
  if (last_effort_command_.d.size() != joint_count_) {
    last_effort_command_.d.assign(joint_count_, 0.0);
  }
  got_effort_command_ = true;
  effort_zeroed_ = false;
  effort_hold_latched_ = false;
  last_effort_command_stamp_ = now().seconds();
  if (step_on_effort_command_) {
    ExecuteEffortGuarded(last_effort_command_);
  }
}

void StudentArmNode::ControlLoop() {
  if (direct_effort_enabled_) {
    // 串口刚打开时 available() 可能还是 0；这时 AbsArm 的位置缓存仍是
    // AddMotor() 初始化出来的全零。此前在这里直接锁存保持会把真机拉向
    // 零位。直驱启动阶段必须先轮询且确认真实反馈，期间严禁写任何指令。
    if (!direct_startup_feedback_ready_) {
      arm_platform_->PublishCurrentState();
      const bool link_alive =
          arm_platform_->IsArmLinkAlive(heartbeat_timeout_sec_);
      if (!link_alive) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Direct effort startup: waiting for serial heartbeat/feedback; "
            "no motor command is sent");
        return;
      }
      if (!direct_startup_link_seen_) {
        direct_startup_link_seen_ = true;
        direct_startup_feedback_generation_ = joint_feedback_generation_;
        return;
      }
      if (joint_feedback_generation_ <
          direct_startup_feedback_generation_ + 3) {
        return;
      }
      direct_startup_feedback_ready_ = true;
      RCLCPP_INFO(
          this->get_logger(),
          "Direct effort startup: serial heartbeat and 3 feedback frames "
          "confirmed; holding at measured pose");
    }
    if (step_on_effort_command_) {
      if (!got_effort_command_ &&
          now().seconds() - last_initial_state_publish_stamp_ >= 0.1) {
        arm_platform_->PublishCurrentState();
        last_initial_state_publish_stamp_ = now().seconds();
      }
      return;
    }
    // Allow one process to perform position prepose before the first effort
    // command switches the node permanently to direct-effort control.
    if (allow_position_handover_ && !got_effort_command_ && got_command_) {
      if (!holding_ &&
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
      }
      arm_platform_->ExecuteControlCycle(effort_control_period_sec_);
      return;
    }
    if (!got_effort_command_) {
      // 【只锁存一次】。这里曾每个周期都重新取当前位置作为保持目标，
      // 于是形成棘轮：本周期「保持在现在的位置」-> 重力把它拉低一点 ->
      // 下周期又以更低的位置为基准。实机上因此三次垂到桌面。
      if (!effort_hold_latched_) {
        last_effort_command_ = MakeHoldPositionCommand();
        effort_hold_latched_ = true;
        effort_zeroed_ = true;
        RCLCPP_INFO(this->get_logger(),
            "effort mode: latching hold position until the first command");
      }
    } else if (!effort_zeroed_ &&
        now().seconds() - last_effort_command_stamp_ > effort_command_timeout_sec_) {
      last_effort_command_ = MakeHoldPositionCommand();
      effort_zeroed_ = true;
      RCLCPP_WARN(this->get_logger(),
          "Direct effort command timeout, holding position with default gains");
    }
    ExecuteEffortGuarded(last_effort_command_);
    return;
  }

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
  arm_platform_->ExecuteControlCycle(effort_control_period_sec_);
}

void StudentArmNode::UpdateJointState(sensor_msgs::msg::JointState& msg) {
  last_known_position_ = msg.position;
  ++joint_feedback_generation_;
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
  arm_platform_->PublishCurrentState();
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
