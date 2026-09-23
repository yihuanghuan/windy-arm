#include <manipulator/ros_nodes/student_arm_node.h>
#include <manipulator/robotics/arm/arm_factory.h>
#include <manipulator/controller/smooth_position_controller.h>
#include <manipulator/common_types.h>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <sstream>
#include <iomanip>

namespace manipulator {

namespace {
double SteadySeconds() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
template <typename Values>
void JsonArray(std::ostream& out, const Values& values) {
  out << '[';
  for (size_t i=0; i<values.size(); ++i) out << (i ? "," : "") << values[i];
  out << ']';
}
std::string CommandJson(const dummy_interface::msg::MotorControl& c) {
  std::ostringstream out; out << std::setprecision(17) << "{\"position\":";
  JsonArray(out,c.position); out << ",\"velocity\":"; JsonArray(out,c.velocity);
  out << ",\"current\":"; JsonArray(out,c.current);
  out << ",\"p\":"; JsonArray(out,c.p); out << ",\"d\":"; JsonArray(out,c.d);
  out << '}'; return out.str();
}
}

StudentArmNode::StudentArmNode() : Node("student_arm_node") {
  read_only_ = GetParam<bool>("read_only", false);
  familiarization_mode_ = GetParam<bool>("familiarization_mode", false);
  wait_for_first_effort_command_ = GetParam<bool>("wait_for_first_effort_command", false);
  feedback_timeout_sec_ = GetParam<double>("feedback_timeout_sec", 0.05);
  feedback_velocity_limit_ = GetParam<double>("feedback_velocity_limit", 0.5);
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
  if (joint_count_ == 0 || joint_upper_limits_.size() != joint_count_ ||
      (familiarization_mode_ && joint_count_ != 6))
    throw std::invalid_argument("invalid joint limit dimensions");
  for (size_t i = 0; i < joint_count_; ++i) {
    if (!std::isfinite(joint_lower_limits_[i]) || !std::isfinite(joint_upper_limits_[i]) ||
        joint_lower_limits_[i] >= joint_upper_limits_[i])
      throw std::invalid_argument("invalid joint limits");
  }
  for (double value : {command_timeout_sec_, effort_command_timeout_sec_,
                       heartbeat_timeout_sec_, feedback_timeout_sec_, feedback_velocity_limit_})
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("timeouts and velocity limit must be positive/finite");

  if (GetParam<bool>("phase_speed_protection", false)) {
    if (!familiarization_mode_ || joint_count_ != 6)
      throw std::invalid_argument("phase speed protection requires six-axis familiarization");
    speed_safety_ = std::make_unique<SpeedSafety>(SpeedSafetyConfig{
      GetParam<double>("transition_warning_speed", 0.), GetParam<double>("transition_abort_speed", 0.),
      GetParam<double>("guide_warning_speed", 0.), GetParam<double>("guide_sustained_abort_speed", 0.),
      GetParam<double>("guide_sustained_abort_duration", 0.), GetParam<double>("guide_instant_abort_speed", 0.)});
    speed_safety_->SetManual(GetParam<std::vector<double>>("guide_manual_warning_speed", {}),
        GetParam<std::vector<double>>("guide_manual_sustained_abort_speed", {}),
        GetParam<std::vector<double>>("guide_manual_instant_abort_speed", {}),
        GetParam<double>("guide_manual_sustained_duration", 0.));
    pub_speed_safety_ = create_publisher<std_msgs::msg::String>("/student/speed_safety", 10);
    // Safety-only notification resolves cross-process sampling races: it can only
    // remove FF/latch hold, never enable torque or rearm. MotorControl is unchanged.
    sub_speed_fault_ = create_subscription<std_msgs::msg::UInt8>(
        "/arm6_manual_guidance/speed_fault_axes", 10,
        [this](const std_msgs::msg::UInt8::SharedPtr msg) {
          if (read_only_ || !motor_command_sent_ || msg->data == 0 || (msg->data & 0xc0)) return;
          for (size_t i = 0; i < 6; ++i)
            speed_fault_axes_[i] = speed_fault_axes_[i] || (msg->data & (1u << i));
          if (!feedback_fault_latched_) last_effort_command_ = MakeHoldPositionCommand();
          MaskUnsafeHoldFeedforward(last_effort_command_);
          feedback_fault_latched_ = effort_zeroed_ = true;
          RCLCPP_ERROR(get_logger(), "guidance speed fault axes=%u: fault latched, restart required", msg->data);
        });
  }

  pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  pub_joint_feedback_ = this->create_publisher<dummy_interface::msg::MotorState>(
      "/student/joint_feedback", 10);

  sub_student_cmd_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/student/joint_command", 10,
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
        StudentCommandCallback(msg);
      });
  sub_student_effort_cmd_ = this->create_subscription<dummy_interface::msg::MotorControl>(
      "/student/joint_effort_command", 1,
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
  bumpless_handover_ = GetParam<bool>("bumpless_handover", false);
  if (bumpless_handover_) SetupHandover();

  control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(effort_control_period_sec_)),
      [this]() { ControlLoop(); });

  RCLCPP_INFO(this->get_logger(),
              "StudentArmNode started: %s",
              read_only_ ? "READ ONLY: feedback polling, all motor writes disabled" :
              familiarization_mode_ ? "six-axis familiarization; position commands disabled" :
              "position command interface /student/joint_command available");
  RCLCPP_INFO(this->get_logger(),
              "Direct effort command topic /student/joint_effort_command is %s",
              (direct_effort_enabled_ && !read_only_) ? "enabled" : "disabled");
  if (step_on_effort_command_) {
    RCLCPP_INFO(
        this->get_logger(),
        "Torque simulation lockstep enabled: one plant step per effort command");
  }
}

void StudentArmNode::SetupHandover() {
  if (!familiarization_mode_ || !speed_safety_) throw std::invalid_argument("handover requires familiarization safety");
  GetParam<bool>("manual_push_test", false);
  quiet_duration_ = GetParam<double>("no_touch_duration_sec", 0.);
  quiet_tolerance_ = GetParam<double>("no_touch_position_tolerance", 0.);
  quiet_speed_ = GetParam<double>("no_touch_speed_limit", 0.);
  handover_joint_margin_ = GetParam<double>("joint_limit_margin", 0.);
  required_handovers_ = GetParam<int>("required_handover_trials", 3);
  if (!std::isfinite(handover_joint_margin_) || handover_joint_margin_<=0. ||
      required_handovers_ < 3 || !std::isfinite(quiet_duration_) || quiet_duration_<3. ||
      !std::isfinite(quiet_tolerance_) || quiet_tolerance_<=0. ||
      !std::isfinite(quiet_speed_) || quiet_speed_<=0. || quiet_speed_>speed_safety_->config.transition_abort)
    throw std::invalid_argument("invalid no-touch gate");
  pub_applied_command_ = create_publisher<dummy_interface::msg::MotorControl>("/student/applied_motor_command", 10);
  pub_support_ = create_publisher<std_msgs::msg::String>("/student/support_diagnostics", 10);
  initial_hold_command_.position = GetParam<std::vector<double>>("initial_hold_position", {});
  initial_hold_command_.velocity.assign(6,0.);
  initial_hold_command_.current = GetParam<std::vector<double>>("initial_hold_current", {});
  initial_hold_command_.p = GetParam<std::vector<double>>("initial_hold_p", {});
  initial_hold_command_.d = GetParam<std::vector<double>>("initial_hold_d", {});
  initial_hold_command_.enable = 1;
  seed_expected_q_ = GetParam<std::vector<double>>("initial_hold_expected_q", {});
  initial_hold_position_tolerance_ = GetParam<double>("initial_hold_position_tolerance", .03);
  if (!std::isfinite(initial_hold_position_tolerance_) || initial_hold_position_tolerance_<=0. ||
      initial_hold_position_tolerance_>.03) throw std::invalid_argument("invalid startup position tolerance");
  if (GetParam<bool>("initial_hold_is_support_estimate", false))
    RCLCPP_WARN(get_logger(),"SUPPORTED_STARTUP: first HOLD uses measured-effort estimate, not old-command readback; external support required");
  has_seed_ = !initial_hold_command_.position.empty();
  if (has_seed_ && seed_expected_q_.size()!=6) throw std::invalid_argument("hold seed needs expected six-axis feedback");

  prepare_service_ = create_service<std_srvs::srv::Trigger>("~/prepare_guidance",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        if (read_only_ || feedback_fault_latched_ || !FeedbackSafe() || !operator_hold_ ||
            !motor_command_sent_ || awaiting_handover_) { res->message="healthy authoritative hold required"; return; }
        for (double v:last_known_velocity_) if (std::abs(v)>.15) {res->message="arm must be stationary"; return;}
        expected_first_command_=last_applied_command_;
        awaiting_handover_=true; prepare_stamp_=SteadySeconds();
        trial_counted_=false; quiet_since_=-1.; manual_push_active_=false;
        res->success=true; res->message=CommandJson(expected_first_command_);
        RCLCPP_INFO(get_logger(), "HANDOVER_START: authoritative hold snapshot, completed=%d",completed_handovers_);
      });
  operator_hold_service_ = create_service<std_srvs::srv::Trigger>("~/operator_hold",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        if (read_only_ || feedback_fault_latched_ || !FeedbackSafe() || !motor_command_sent_) {
          res->message="cannot clear a fault or hold invalid feedback"; return;
        }
        auto hold=MakeHoldPositionCommand();
        // Healthy GUIDE -> HOLD also preserves total command at the measured state.
        for(size_t i=0;i<6;++i) {
          double total=last_applied_command_.p[i]*(last_applied_command_.position[i]-last_known_position_[i])-
              last_applied_command_.d[i]*last_known_velocity_[i]+last_applied_command_.current[i];
          hold.position[i]=last_known_position_[i]+(total-hold.current[i]+hold.d[i]*last_known_velocity_[i])/hold.p[i];
        }
        if (!ValidateEffortCommand(hold)) {res->message="hold outside command limits"; return;}
        last_effort_command_=hold; operator_hold_=true; awaiting_handover_=false;
        manual_push_active_=false; effort_zeroed_=true; got_effort_command_=true;
        res->success=true; res->message="operator HOLD; fault latch unchanged";
        RCLCPP_INFO(get_logger(), "HOLD: operator stop; completed no-touch handovers=%d",completed_handovers_);
      });
  manual_service_ = create_service<std_srvs::srv::Trigger>("~/begin_manual_push",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        if (read_only_ || !get_parameter("manual_push_test").as_bool() || feedback_fault_latched_ ||
            !FeedbackSafe() || !last_output_guide_ || operator_hold_ || !trial_counted_ ||
            completed_handovers_<required_handovers_ || quiet_since_<0. ||
            SteadySeconds()-quiet_since_<quiet_duration_) {
          res->message="manual push requires explicit flag, GUIDE and three successful no-touch handovers"; return;
        }
        manual_push_active_=true; res->success=true; res->message="MANUAL_PUSH enabled explicitly";
        RCLCPP_INFO(get_logger(), "MANUAL_PUSH: explicit operator request accepted");
      });
}

void StudentArmNode::UpdateNoTouchGate() {
  if (!bumpless_handover_ || manual_push_active_) return;
  bool quiet=last_output_guide_ && !feedback_fault_latched_ && FeedbackSafe();
  for(double v:last_known_velocity_) quiet=quiet && std::abs(v)<=quiet_speed_;
  if (!quiet) {quiet_since_=-1.; return;}
  if (quiet_since_>=0.) for(size_t i=0;i<6;++i)
    if(std::abs(last_known_position_[i]-quiet_q_[i])>quiet_tolerance_) quiet_since_=-1.;
  if (quiet_since_<0.) {quiet_since_=SteadySeconds(); quiet_q_=last_known_position_;}
  if (!trial_counted_ && SteadySeconds()-quiet_since_>=quiet_duration_) {
    ++completed_handovers_; trial_counted_=true;
    RCLCPP_INFO(get_logger(), "NO_TOUCH_PASS: completed handovers=%d",completed_handovers_);
  }
}

void StudentArmNode::PublishAppliedCommand(const dummy_interface::msg::MotorControl& cmd) {
  if (!bumpless_handover_) return;
  std::vector<double> pd(6),total(6),step(6);
  for(size_t i=0;i<6;++i) {
    pd[i]=cmd.p[i]*(cmd.position[i]-last_known_position_[i])-cmd.d[i]*last_known_velocity_[i];
    total[i]=pd[i]+cmd.current[i];
    if(last_applied_command_.position.size()==6)
      step[i]=total[i]-(last_applied_command_.p[i]*(last_applied_command_.position[i]-last_known_position_[i])-
          last_applied_command_.d[i]*last_known_velocity_[i]+last_applied_command_.current[i]);
  }
  last_applied_command_=cmd; last_applied_command_.header.stamp=now();
  pub_applied_command_->publish(last_applied_command_);
  std::ostringstream out; out<<std::setprecision(17)<<"{\"q\":";JsonArray(out,last_known_position_);
  out<<",\"dq\":";JsonArray(out,last_known_velocity_);
  out<<",\"command\":"<<CommandJson(cmd)<<",\"estimated_pd_torque\":";JsonArray(out,pd);
  out<<",\"estimated_total_torque\":";JsonArray(out,total);
  out<<",\"delta_total_command_from_previous_cycle\":";JsonArray(out,step);
  out<<",\"completed_handovers\":"<<completed_handovers_<<",\"manual_push\":"<<(manual_push_active_?"true":"false")<<'}';
  std_msgs::msg::String msg;msg.data=out.str();pub_support_->publish(msg);
}

StudentArmNode::~StudentArmNode() {
  control_timer_->cancel();
  // Best-effort hold, never reset/home. No writes on a read-only session.
  if (familiarization_mode_ && !read_only_ && motor_command_sent_) {
    try {
      arm_platform_->PublishCurrentState();
      if (last_known_position_.size() == joint_count_) {
        auto hold = (feedback_fault_latched_ || (bumpless_handover_ && operator_hold_)) ?
            last_effort_command_ : MakeHoldPositionCommand();
        MaskUnsafeHoldFeedforward(hold);
        ExecuteEffortGuarded(hold);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "shutdown hold failed: %s", e.what());
    }
  }
}

void StudentArmNode::SetArmPlatform() {
  std::string port = GetParam<std::string>("port_name", "/dev/ttyUSB0");
  std::string arm_type = GetParam<std::string>("arm_type", "sim");
  std::string arm_version = GetParam<std::string>("arm_version", "gamma");
  if (familiarization_mode_ && (arm_type != "a_l1" || arm_version != "gamma6"))
    throw std::invalid_argument("familiarization requires a_l1/gamma6");
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
  if (arm->GetJointNames().size() != joint_count_)
    throw std::invalid_argument("configured motors and joint limits differ");
  if (familiarization_mode_ && arm->GetJointNames() !=
      std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"})
    throw std::invalid_argument("unexpected six-axis joint mapping");
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
  if (familiarization_mode_) {
    if (p_gain.size() != 6 || d_gain.size() != 6 ||
        gravity_hold_zero_offset_rad_.size() != 6 || gravity_hold_torque_limit_.size() != 6)
      throw std::invalid_argument("familiarization parameters must be exactly six-dimensional");
    for (size_t i = 0; i < 6; ++i) {
      if (!std::isfinite(p_gain[i]) || p_gain[i] <= 0 ||
          !std::isfinite(d_gain[i]) || d_gain[i] <= 0 ||
          !std::isfinite(gravity_hold_zero_offset_rad_[i]) ||
          !std::isfinite(gravity_hold_torque_limit_[i]) || gravity_hold_torque_limit_[i] <= 0)
        throw std::invalid_argument("invalid familiarization gains/offsets/limits");
    }
  }
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
  if (read_only_ || familiarization_mode_) return;
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
  last_command_stamp_ = SteadySeconds();
}

bool StudentArmNode::ValidateEffortCommand(const dummy_interface::msg::MotorControl& msg) {
  if (read_only_ || feedback_fault_latched_ || !direct_effort_enabled_) {
    return false;
  }
  if (msg.enable == 0) {
    return true;
  }
  if (familiarization_mode_) {
    const double age = now().seconds() - rclcpp::Time(msg.header.stamp).seconds();
    if (age < -0.01 || age > effort_command_timeout_sec_) return false;
  }
  if (msg.current.size() != joint_count_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Rejected direct effort command: expected %zu current/torque values, got %zu",
        joint_count_, msg.current.size());
    return false;
  }
  if (msg.position.size() != joint_count_ || msg.velocity.size() != joint_count_ ||
      msg.p.size() != joint_count_ || msg.d.size() != joint_count_) return false;
  auto finite_vector = [](const std::vector<double>& values) {
    return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
  };
  if (!finite_vector(msg.current) || !finite_vector(msg.p) || !finite_vector(msg.d) ||
      !finite_vector(msg.position) || !finite_vector(msg.velocity)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Rejected direct effort command: contains NaN/Inf");
    return false;
  }
  for (size_t i = 0; i < joint_count_; ++i) {
    if (msg.p[i] < 0 || msg.d[i] < 0 || msg.position[i] < joint_lower_limits_[i] ||
        msg.position[i] > joint_upper_limits_[i]) return false;
    if (familiarization_mode_ && (msg.p[i] > hold_p_gain_[i] ||
        msg.d[i] > hold_d_gain_[i] || msg.velocity[i] != 0.0 ||
        std::abs(msg.current[i]) > gravity_hold_torque_limit_[i])) return false;
  }
  return true;
}

bool StudentArmNode::FeedbackSafe() const {
  if (!arm_platform_->HasFreshFeedback(feedback_timeout_sec_) ||
      !arm_platform_->IsArmLinkAlive(heartbeat_timeout_sec_) ||
      last_known_position_.size() != joint_count_ ||
      last_known_velocity_.size() != joint_count_) return false;
  for (size_t i = 0; i < joint_count_; ++i) {
    if (!std::isfinite(last_known_position_[i]) || !std::isfinite(last_known_velocity_[i]) ||
        (!speed_safety_ && std::abs(last_known_velocity_[i]) > feedback_velocity_limit_)) return false;
    if (bumpless_handover_ && (last_known_position_[i]<=joint_lower_limits_[i]+handover_joint_margin_ ||
        last_known_position_[i]>=joint_upper_limits_[i]-handover_joint_margin_)) return false;
  }
  if (speed_safety_ && speed_safety_->fault) return false;
  return true;
}

void StudentArmNode::UpdateSpeedSafety() {
  if (!speed_safety_ || speed_feedback_generation_ == joint_feedback_generation_ ||
      !arm_platform_->HasFreshFeedback(feedback_timeout_sec_) || last_known_velocity_.size() != 6)
    return;
  speed_feedback_generation_ = joint_feedback_generation_;
  for (double v : last_known_velocity_) if (!std::isfinite(v)) return;
  const auto previous = speed_safety_->warning;
  const bool was_fault = speed_safety_->fault;
  speed_safety_->Update(last_known_velocity_, last_output_guide_, SteadySeconds(), manual_push_active_);
  auto& s = *speed_safety_;
  std::ostringstream out;
  out << "{\"phase\":\"" << (feedback_fault_latched_ ? "FAULT" :
      s.manual_phase ? "MANUAL_PUSH" : s.guide_phase ? "GUIDE" : "HOLD_OR_TRANSITION") << "\",\"warning\":[";
  for (size_t i = 0; i < 6; ++i) out << (i ? "," : "") << (s.warning[i] ? "true" : "false");
  out << "],\"sustained_duration\":[";
  for (size_t i = 0; i < 6; ++i) out << (i ? "," : "") << s.duration[i];
  out << "],\"fault_axes\":[";
  for (size_t i = 0; i < 6; ++i) {
    speed_fault_axes_[i] = speed_fault_axes_[i] || s.axes[i];
    out << (i ? "," : "") << (speed_fault_axes_[i] ? "true" : "false");
    if (s.warning[i] && !previous[i])
      RCLCPP_WARN(get_logger(), "SPEED WARNING joint%zu phase=%s dq=%.6f", i+1,
                  s.guide_phase ? "GUIDE" : "TRANSITION", last_known_velocity_[i]);
    if (!was_fault && s.axes[i])
      RCLCPP_ERROR(get_logger(), "SPEED HARD_ABORT joint%zu phase=%s type=%s dq=%.6f duration=%.6f",
                   i+1, s.guide_phase ? "GUIDE" : "TRANSITION", s.type[i].c_str(),
                   last_known_velocity_[i], s.duration[i]);
  }
  out << "]}";
  std_msgs::msg::String msg;
  msg.data = out.str();
  pub_speed_safety_->publish(msg);
}

void StudentArmNode::MaskUnsafeHoldFeedforward(
    dummy_interface::msg::MotorControl& hold) const {
  if (!arm_platform_->HasFreshFeedback(feedback_timeout_sec_) ||
      !arm_platform_->IsArmLinkAlive(heartbeat_timeout_sec_) ||
      last_known_velocity_.size() != joint_count_) {
    hold.current.assign(joint_count_, 0.0);
    return;
  }
  // An overspeed joint does not invalidate gravity support on the other axes.
  // Values are only removed here, never restored after a latched fault.
  for (size_t i = 0; i < joint_count_; ++i) {
    if (!std::isfinite(last_known_velocity_[i]) ||
        (speed_safety_ ? speed_fault_axes_[i] ||
          std::abs(last_known_velocity_[i]) > (speed_safety_->guide_phase ?
            (speed_safety_->manual_phase ? speed_safety_->manual_instant[i] : speed_safety_->config.instant_speed) : speed_safety_->config.transition_abort) :
          std::abs(last_known_velocity_[i]) > feedback_velocity_limit_))
      hold.current[i] = 0.0;
  }
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
    if (familiarization_mode_) {
      // Preserve only the last transmitted bounded support. Recomputing full g
      // during a partial ramp or an abort would create a new feedforward step.
      if (last_sent_effort_.size() == joint_count_) cmd.current = last_sent_effort_;
      MaskUnsafeHoldFeedforward(cmd);
    } else {
      cmd.current = ComputeHoldGravityCurrent();
    }
    // Hardware ignores enable, but keeping it asserted documents that this is
    // an active MIT hold rather than a user-requested torque disable.
    cmd.enable = 1;
  }
  cmd.header.stamp = this->now();
  return cmd;
}

void StudentArmNode::ExecuteEffortGuarded(
    const dummy_interface::msg::MotorControl& cmd) {
  if (read_only_) return;
  const bool alive = arm_platform_->IsArmLinkAlive(heartbeat_timeout_sec_) &&
      arm_platform_->HasFreshFeedback(feedback_timeout_sec_);
  if (ShouldBlockTorque(alive, cmd)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "serial heartbeat stale (> %.2fs): blocking torque feedforward, "
        "holding position instead", heartbeat_timeout_sec_);
    auto hold = cmd;
    hold.current.assign(joint_count_, 0.0);
    // Preserve the latched target; stale feedback must not move the hold reference.
    hold.p = hold_p_gain_;
    hold.d = hold_d_gain_;
    arm_platform_->ExecuteDirectMotorCommand(hold, effort_control_period_sec_);
    motor_command_sent_ = true;
    last_sent_effort_ = hold.current;
    last_output_guide_ = false;
    PublishAppliedCommand(hold);
    return;
  }
  arm_platform_->ExecuteDirectMotorCommand(cmd, effort_control_period_sec_);
  motor_command_sent_ = true;
  last_sent_effort_ = cmd.current;
  last_output_guide_ = cmd.enable && cmd.p.size() == 6 &&
      std::all_of(cmd.p.begin(), cmd.p.end(), [](double p) { return p == 0.; });
  PublishAppliedCommand(cmd);
}

void StudentArmNode::StudentEffortCommandCallback(
    const dummy_interface::msg::MotorControl::ConstSharedPtr& msg) {
  if (familiarization_mode_ && (!direct_startup_feedback_ready_ || !FeedbackSafe())) return;
  if (!ValidateEffortCommand(*msg)) {
    return;
  }
  if (bumpless_handover_ && msg->enable && operator_hold_) {
    if (!awaiting_handover_) return;
    auto equal=[](const auto& a,const auto& b) {
      if(a.size()!=b.size())return false;
      for(size_t i=0;i<a.size();++i)if(std::abs(a[i]-b[i])>1e-8)return false;
      return true;
    };
    if (!equal(msg->position,expected_first_command_.position) || !equal(msg->current,expected_first_command_.current) ||
        !equal(msg->p,expected_first_command_.p) || !equal(msg->d,expected_first_command_.d) ||
        !equal(msg->velocity,expected_first_command_.velocity)) {
      feedback_fault_latched_=true;
      RCLCPP_ERROR(get_logger(),"HANDOVER first command mismatch: fault latched"); return;
    }
    operator_hold_=false;awaiting_handover_=false;
  }
  if (msg->enable == 0) {
    last_effort_command_ = MakeHoldPositionCommand();
    got_effort_command_ = true;
    effort_zeroed_ = true;
    if (familiarization_mode_) feedback_fault_latched_ = true;
    last_effort_command_stamp_ = SteadySeconds();
    if (step_on_effort_command_) {
      ExecuteEffortGuarded(last_effort_command_);
    }
    return;
  }

  last_effort_command_ = *msg;
  got_effort_command_ = true;
  effort_zeroed_ = false;
  effort_hold_latched_ = false;
  last_effort_command_stamp_ = SteadySeconds();
  if (step_on_effort_command_) {
    ExecuteEffortGuarded(last_effort_command_);
  }
}

void StudentArmNode::ControlLoop() {
  if (read_only_) {
    arm_platform_->PublishCurrentState();
    return;
  }
  // Poll before evaluating liveness, so only fresh serial data authorizes output.
  if (direct_effort_enabled_) arm_platform_->PublishCurrentState();
  UpdateSpeedSafety();
  UpdateNoTouchGate();
  if (familiarization_mode_ && direct_startup_feedback_ready_ &&
      !feedback_fault_latched_ && !FeedbackSafe()) {
    feedback_fault_latched_ = true;
    // Keep a single last measured hold target. Re-latching each tick causes sag.
    last_effort_command_ = MakeHoldPositionCommand();
    effort_zeroed_ = true;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "feedback stale/invalid/overspeed: fault latched, restart required");
  }
  if (feedback_fault_latched_) {
    MaskUnsafeHoldFeedforward(last_effort_command_);
    if (motor_command_sent_ && last_known_position_.size() == joint_count_)
      ExecuteEffortGuarded(last_effort_command_);
    return;
  }
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
      if (!FeedbackSafe() || (familiarization_mode_ ?
          arm_platform_->GetFeedbackSequence() < 3 :
          joint_feedback_generation_ < direct_startup_feedback_generation_ + 3)) {
        return;
      }
      direct_startup_feedback_ready_ = true;
      RCLCPP_INFO(
          this->get_logger(),
          "Direct effort startup: serial heartbeat and 3 feedback frames "
          "confirmed; ready for configured effort/hold mode");
    }
    if (bumpless_handover_ && !motor_command_sent_) {
      if (!has_seed_) return;  // unknown firmware command must never be guessed
      auto seed=initial_hold_command_;seed.header.stamp=now();
      bool valid=ValidateEffortCommand(seed);
      for(size_t i=0;i<6 && valid;++i) {
        valid=seed.p[i]>0. && std::abs(last_known_velocity_[i])<=.15 &&
            std::isfinite(seed_expected_q_[i]) && std::abs(last_known_position_[i]-seed_expected_q_[i])<=initial_hold_position_tolerance_ &&
            std::abs(seed.p[i]*(seed.position[i]-last_known_position_[i])-seed.d[i]*last_known_velocity_[i]+seed.current[i])<=gravity_hold_torque_limit_[i];
      }
      if (!valid) {feedback_fault_latched_=true;RCLCPP_ERROR(get_logger(),"hold seed invalid: fault latched; no startup write");return;}
      last_effort_command_=seed;got_effort_command_=true;operator_hold_=true;effort_zeroed_=true;
      ExecuteEffortGuarded(seed);return;
    }
    if (bumpless_handover_ && operator_hold_) {
      if(awaiting_handover_ && SteadySeconds()-prepare_stamp_>.5) {
        awaiting_handover_=false;feedback_fault_latched_=true;
        RCLCPP_ERROR(get_logger(),"HANDOVER command timeout: fault latched");
      }
      ExecuteEffortGuarded(last_effort_command_);return;
    }
    if (wait_for_first_effort_command_ && !got_effort_command_) return;
    if (step_on_effort_command_) {
      if (!got_effort_command_ &&
          SteadySeconds() - last_initial_state_publish_stamp_ >= 0.1) {
        arm_platform_->PublishCurrentState();
        last_initial_state_publish_stamp_ = SteadySeconds();
      }
      return;
    }
    // Allow one process to perform position prepose before the first effort
    // command switches the node permanently to direct-effort control.
    if (allow_position_handover_ && !got_effort_command_ && got_command_) {
      if (!holding_ &&
          SteadySeconds() - last_command_stamp_ > command_timeout_sec_) {
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
        SteadySeconds() - last_effort_command_stamp_ > effort_command_timeout_sec_) {
      last_effort_command_ = MakeHoldPositionCommand();
      effort_zeroed_ = true;
      if (familiarization_mode_) feedback_fault_latched_ = true;
      RCLCPP_WARN(this->get_logger(),
          "Direct effort command timeout, holding position with default gains");
    }
    ExecuteEffortGuarded(last_effort_command_);
    return;
  }

  // 安全层：指令超时后保持当前位置
  if (got_command_ && !holding_ &&
      SteadySeconds() - last_command_stamp_ > command_timeout_sec_) {
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
  last_known_velocity_ = msg.velocity;
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
