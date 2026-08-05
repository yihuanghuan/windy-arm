#include <manipulator/robotics/arm/torque_sim_arm.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <yaml-cpp/yaml.h>

#include <manipulator/robotics/config/arm_config.h>

namespace manipulator::arm {

namespace {

constexpr int kBaseVelocitySize = 6;
// 关节数由 URDF 推导，不再写死。6 关节实物上 joint7 不存在 ——
// 写死 7 会让 SetInitialJointPositions 直接抛异常，节点起不来。
// 构造时先按最大值预留，BuildModel 后按实际 nv-6 重置。
constexpr int kArmDofMax = 7;

bool IsFinite(const Eigen::VectorXd& values) {
  return values.array().isFinite().all();
}

}  // namespace

TorqueSimArm::TorqueSimArm()
    : data_(model_),
      q_(Eigen::VectorXd::Zero(kArmDofMax)),
      qd_(Eigen::VectorXd::Zero(kArmDofMax)),
      tau_command_(Eigen::VectorXd::Zero(kArmDofMax)),
      torque_limit_(Eigen::VectorXd::Constant(kArmDofMax, 1.0)),
      velocity_limit_(Eigen::VectorXd::Constant(kArmDofMax, 3.0)),
      lower_limit_(Eigen::VectorXd::Constant(kArmDofMax, -M_PI)),
      upper_limit_(Eigen::VectorXd::Constant(kArmDofMax, M_PI)) {}

void TorqueSimArm::SetModelPath(const std::string& model_path) {
  model_path_ = model_path;
}

void TorqueSimArm::SetInitialJointPositions(
    const std::vector<double>& positions) {
  // 此时 URDF 还没加载，关节数未知，只做上界检查；确切校验在 InitializeState。
  if (!positions.empty() && positions.size() > static_cast<size_t>(kArmDofMax)) {
    throw std::invalid_argument(
        "TorqueSimArm initial_joint_positions has more than " +
        std::to_string(kArmDofMax) + " values");
  }
  initial_positions_ = positions;
}

void TorqueSimArm::Init(const std::string& /*port*/, uint32_t /*baud*/) {
  if (model_path_.empty()) {
    throw std::runtime_error(
        "TorqueSimArm requires the student_arm_node urdf_path parameter");
  }
  BuildModel();
  joint_names_.clear();
  for (int i = 1; i <= arm_dof_; ++i) {
    joint_names_.push_back("joint" + std::to_string(i));
  }
  InitializeState();
}

void TorqueSimArm::InitFromConfig(
    const std::string& port, uint32_t baud,
    const std::string& motor_config_path,
    const std::string& arm_config_path,
    const std::string& arm_name) {
  const YAML::Node motor_yaml = YAML::LoadFile(motor_config_path);
  const YAML::Node arm_yaml = YAML::LoadFile(arm_config_path);
  const auto motor_models =
      config::ConfigLoader::LoadMotorModels(motor_yaml);
  const auto arm_config =
      config::ConfigLoader::LoadArmConfig(arm_yaml, "a_l1_" + arm_name);
  // 关节数以【配置】为准。此处 BuildModel 还没跑，arm_dof_ 仍是默认值，
  // 拿它校验会把 6 关节配置误判为错误（实测：a_l1_gamma6 被拒，节点起不来）。
  // URDF 的一致性在 BuildModel 之后交叉验证。
  arm_dof_ = static_cast<int>(arm_config.joints.size());
  if (arm_dof_ < 1) {
    throw std::runtime_error("TorqueSimArm: arm config has no joints");
  }
  q_ = Eigen::VectorXd::Zero(arm_dof_);
  qd_ = Eigen::VectorXd::Zero(arm_dof_);
  tau_command_ = Eigen::VectorXd::Zero(arm_dof_);
  torque_limit_ = Eigen::VectorXd::Constant(arm_dof_, 1.0);
  velocity_limit_ = Eigen::VectorXd::Constant(arm_dof_, 3.0);
  lower_limit_ = Eigen::VectorXd::Constant(arm_dof_, -M_PI);
  upper_limit_ = Eigen::VectorXd::Constant(arm_dof_, M_PI);

  joint_names_.clear();
  for (int i = 0; i < arm_dof_; ++i) {
    const auto& joint = arm_config.joints[static_cast<size_t>(i)];
    const auto model_it = motor_models.find(joint.model);
    if (model_it == motor_models.end()) {
      throw std::runtime_error(
          "TorqueSimArm motor model not found: " + joint.model);
    }
    joint_names_.push_back(joint.name);
    lower_limit_[i] = joint.lower_limit;
    upper_limit_[i] = joint.upper_limit;
    // 被控对象模拟【硬件物理能力】，取电机峰值力矩；软件策略上限由
    // SMC 侧的 ActuatorAdapter 按 config 的 limits.torque 执行。
    //
    // 这里原先取 rated_torque，导致【双重截断】：控制器按自己的上限
    // 限幅后，被控对象又按额定值截一次。后果是把控制器配置里调高上限
    // 变成空操作，且让 SMC 自身的饱和行为被掩盖、无法从日志区分。
    // max_torque 此前被解析却从未使用。
    torque_limit_[i] = model_it->second.max_torque;
    velocity_limit_[i] = model_it->second.max_velocity;
  }
  Init(port, baud);
}

void TorqueSimArm::BuildModel() {
  if (!std::filesystem::exists(model_path_)) {
    throw std::runtime_error(
        "TorqueSimArm URDF does not exist: " + model_path_);
  }
  pinocchio::urdf::buildModel(
      model_path_, pinocchio::JointModelFreeFlyer(), model_);
  data_ = pinocchio::Data(model_);
  // FreeFlyer 贡献 nq=7 / nv=6，其余就是手臂关节数。
  const int urdf_dof = model_.nv - 6;
  if (urdf_dof < 1 || model_.nq != 7 + urdf_dof) {
    throw std::runtime_error(
        "TorqueSimArm: unexpected URDF dimensions nq=" +
        std::to_string(model_.nq) + " nv=" + std::to_string(model_.nv));
  }
  // 配置已设定 arm_dof_ 时交叉校验；未设定（走 Init 而非 InitFromConfig）则采纳。
  if (arm_dof_ != urdf_dof) {
    if (!joint_names_.empty()) {
      throw std::runtime_error(
          "TorqueSimArm: arm config has " + std::to_string(arm_dof_) +
          " joints but the URDF has " + std::to_string(urdf_dof));
    }
    arm_dof_ = urdf_dof;
  }
  // 【只在尺寸不符时才重置】。BuildModel 在 InitFromConfig 之后执行，
  // 无条件重置会把配置里的力矩上限(9/1.6 Nm)冲成 1.0、关节限位冲成 ±π ——
  // plant 连自重都撑不住，仿真必然发散（实测：所有关节卡在 ±π，
  // 末端误差 371 mm）。
  if (q_.size() != arm_dof_) {
    q_ = Eigen::VectorXd::Zero(arm_dof_);
    qd_ = Eigen::VectorXd::Zero(arm_dof_);
    tau_command_ = Eigen::VectorXd::Zero(arm_dof_);
  }
  if (torque_limit_.size() != arm_dof_) {
    torque_limit_ = Eigen::VectorXd::Constant(arm_dof_, 1.0);
    velocity_limit_ = Eigen::VectorXd::Constant(arm_dof_, 3.0);
    lower_limit_ = Eigen::VectorXd::Constant(arm_dof_, -M_PI);
    upper_limit_ = Eigen::VectorXd::Constant(arm_dof_, M_PI);
  }
  ApplyArmature();
  model_ready_ = true;
}

void TorqueSimArm::ApplyArmature() {
  // 转子惯量经减速比反射的等效值，加在质量矩阵对角上。
  //
  // 【必须与控制器用同一套值】。控制器侧 (model.py) 从 arm6_calibration.yaml
  // 读 armature，而这里的 plant 是独立的 pinocchio 模型 —— 不加的话控制器按
  // M=0.11~0.20 算力矩、plant 按 M=0.0006~0.079 积分，实际加速度比预期大
  // 2.5~20 倍，仿真必然发散（实测：关节冲出限位，RViz 里模型散开）。
  //
  // 离线仿真不会暴露这个问题，因为它用同一个模型既做控制又做积分，
  // 模型误差自动抵消。
  const std::string cal_path =
      std::filesystem::path(model_path_).parent_path() / "arm6_calibration.yaml";
  if (!std::filesystem::exists(cal_path)) {
    return;
  }
  const YAML::Node cal = YAML::LoadFile(cal_path);
  if (!cal["armature_kgm2"]) {
    return;
  }
  const auto values = cal["armature_kgm2"];
  if (static_cast<int>(values.size()) != arm_dof_) {
    // 标定值是【6 关节实物专属】的。7 关节模型没有实测转子惯量，
    // 此时跳过、退回 URDF 名义值，而不是报错 —— 抛异常会让 7 关节
    // 这条已验证的通路无法启动。
    std::cout << "[TorqueSimArm] armature calibration has " << values.size()
              << " values but this arm has " << arm_dof_
              << " joints; skipping (URDF nominal inertia is used)."
              << std::endl;
    return;
  }
  for (int i = 0; i < arm_dof_; ++i) {
    if (!values[static_cast<size_t>(i)].IsNull()) {
      model_.armature[6 + i] = values[static_cast<size_t>(i)].as<double>();
    }
  }
  data_ = pinocchio::Data(model_);
  std::cout << "[TorqueSimArm] armature ";
  for (int i = 0; i < arm_dof_; ++i) std::cout << model_.armature[6 + i] << " ";
  std::cout << "| torque_limit ";
  for (int i = 0; i < arm_dof_; ++i) std::cout << torque_limit_[i] << " ";
  std::cout << "| lower ";
  for (int i = 0; i < arm_dof_; ++i) std::cout << lower_limit_[i] << " ";
  std::cout << std::endl;
}

void TorqueSimArm::InitializeState() {
  q_.setZero();
  qd_.setZero();
  tau_command_.setZero();
  if (!initial_positions_.empty()) {
    if (static_cast<int>(initial_positions_.size()) != arm_dof_) {
      throw std::invalid_argument(
          "TorqueSimArm initial_joint_positions has " +
          std::to_string(initial_positions_.size()) + " values but the URDF "
          "has " + std::to_string(arm_dof_) + " actuated joints");
    }
    q_ = Eigen::Map<const Eigen::VectorXd>(
        initial_positions_.data(), arm_dof_);
  }
  q_ = q_.cwiseMax(lower_limit_).cwiseMin(upper_limit_);

  joint_state_.name = joint_names_;
  joint_state_.position.resize(arm_dof_);
  joint_state_.velocity.resize(arm_dof_);
  joint_state_.current.resize(arm_dof_);
  joint_state_.voltage.assign(arm_dof_, 48.0);
  joint_state_.temperature.assign(arm_dof_, 25.0);
  PublishState();
}

bool TorqueSimArm::SetMotorCommand(
    const dummy_interface::msg::MotorControl& cmd) {
  if (cmd.enable == 0) {
    tau_command_.setZero();
    return true;
  }
  if (cmd.current.size() != arm_dof_) {
    return false;
  }
  for (int i = 0; i < arm_dof_; ++i) {
    const double value = cmd.current[static_cast<size_t>(i)];
    if (!std::isfinite(value)) {
      tau_command_.setZero();
      return false;
    }
    tau_command_[i] =
        std::clamp(value, -torque_limit_[i], torque_limit_[i]);
  }
  return true;
}

void TorqueSimArm::SetBaseState(const FloatingBaseState& state) {
  base_state_ = state;
  base_state_.orientation.normalize();
}

Eigen::VectorXd TorqueSimArm::MakeFullConfiguration() const {
  Eigen::VectorXd q_full = pinocchio::neutral(model_);
  q_full.segment<3>(0) = base_state_.position;
  q_full.segment<4>(3) = base_state_.orientation.coeffs();
  q_full.tail(arm_dof_) = q_;
  return q_full;
}

Eigen::VectorXd TorqueSimArm::MakeFullVelocity() const {
  Eigen::VectorXd v_full = Eigen::VectorXd::Zero(model_.nv);
  v_full.head(kBaseVelocitySize) = base_state_.velocity;
  v_full.tail(arm_dof_) = qd_;
  return v_full;
}

Eigen::VectorXd TorqueSimArm::ComputeArmAcceleration() {
  const Eigen::VectorXd q_full = MakeFullConfiguration();
  const Eigen::VectorXd v_full = MakeFullVelocity();
  const Eigen::MatrixXd mass =
      pinocchio::crba(model_, data_, q_full).selfadjointView<Eigen::Upper>();
  const Eigen::VectorXd bias = pinocchio::rnea(
      model_, data_, q_full, v_full, Eigen::VectorXd::Zero(model_.nv));
  const Eigen::MatrixXd arm_mass =
      mass.bottomRightCorner(arm_dof_, arm_dof_);
  const Eigen::MatrixXd base_coupling =
      mass.bottomLeftCorner(arm_dof_, kBaseVelocitySize);
  const Eigen::VectorXd rhs =
      tau_command_ - base_coupling * base_state_.acceleration
      - bias.tail(arm_dof_);
  return arm_mass.ldlt().solve(rhs);
}

void TorqueSimArm::Step(double dt) {
  if (!model_ready_ || !std::isfinite(dt) || dt <= 0.0) {
    return;
  }
  const int substeps =
      std::max(1, static_cast<int>(std::ceil(dt / substep_sec_)));
  const double h = dt / static_cast<double>(substeps);
  for (int step = 0; step < substeps; ++step) {
    const Eigen::VectorXd qdd = ComputeArmAcceleration();
    if (!IsFinite(qdd)) {
      tau_command_.setZero();
      qd_.setZero();
      break;
    }
    qd_ += qdd * h;
    qd_ = qd_.cwiseMax(-velocity_limit_).cwiseMin(velocity_limit_);
    q_ += qd_ * h;
    for (int i = 0; i < arm_dof_; ++i) {
      if (q_[i] < lower_limit_[i]) {
        q_[i] = lower_limit_[i];
        qd_[i] = std::max(0.0, qd_[i]);
      } else if (q_[i] > upper_limit_[i]) {
        q_[i] = upper_limit_[i];
        qd_[i] = std::min(0.0, qd_[i]);
      }
    }
  }
  PublishState();
}

void TorqueSimArm::PublishState() {
  for (int i = 0; i < arm_dof_; ++i) {
    joint_state_.position[static_cast<size_t>(i)] = q_[i];
    joint_state_.velocity[static_cast<size_t>(i)] = qd_[i];
    joint_state_.current[static_cast<size_t>(i)] = tau_command_[i];
  }
}

JointState& TorqueSimArm::GetJointStates() {
  return joint_state_;
}

std::vector<std::string> TorqueSimArm::GetJointNames() const {
  return joint_names_;
}

}  // namespace manipulator::arm
