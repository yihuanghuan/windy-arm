#include <manipulator/robotics/arm/torque_sim_arm.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <yaml-cpp/yaml.h>

#include <manipulator/robotics/config/arm_config.h>

namespace manipulator::arm {

namespace {

constexpr int kBaseVelocitySize = 6;
constexpr int kArmDof = 7;

bool IsFinite(const Eigen::VectorXd& values) {
  return values.array().isFinite().all();
}

}  // namespace

TorqueSimArm::TorqueSimArm()
    : data_(model_),
      q_(Eigen::VectorXd::Zero(kArmDof)),
      qd_(Eigen::VectorXd::Zero(kArmDof)),
      tau_command_(Eigen::VectorXd::Zero(kArmDof)),
      torque_limit_(Eigen::VectorXd::Constant(kArmDof, 1.0)),
      velocity_limit_(Eigen::VectorXd::Constant(kArmDof, 3.0)),
      lower_limit_(Eigen::VectorXd::Constant(kArmDof, -M_PI)),
      upper_limit_(Eigen::VectorXd::Constant(kArmDof, M_PI)) {}

void TorqueSimArm::SetModelPath(const std::string& model_path) {
  model_path_ = model_path;
}

void TorqueSimArm::SetInitialJointPositions(
    const std::vector<double>& positions) {
  if (!positions.empty() && positions.size() != kArmDof) {
    throw std::invalid_argument(
        "TorqueSimArm initial_joint_positions must contain 7 values");
  }
  initial_positions_ = positions;
}

void TorqueSimArm::Init(const std::string& /*port*/, uint32_t /*baud*/) {
  if (model_path_.empty()) {
    throw std::runtime_error(
        "TorqueSimArm requires the student_arm_node urdf_path parameter");
  }
  joint_names_ = {
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
  BuildModel();
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
  if (arm_config.joints.size() != kArmDof) {
    throw std::runtime_error(
        "TorqueSimArm requires a seven-joint arm configuration");
  }

  joint_names_.clear();
  for (int i = 0; i < kArmDof; ++i) {
    const auto& joint = arm_config.joints[static_cast<size_t>(i)];
    const auto model_it = motor_models.find(joint.model);
    if (model_it == motor_models.end()) {
      throw std::runtime_error(
          "TorqueSimArm motor model not found: " + joint.model);
    }
    joint_names_.push_back(joint.name);
    lower_limit_[i] = joint.lower_limit;
    upper_limit_[i] = joint.upper_limit;
    torque_limit_[i] = model_it->second.rated_torque;
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
  if (model_.nq != 14 || model_.nv != 13) {
    throw std::runtime_error(
        "TorqueSimArm expected free-flyer nq=14,nv=13");
  }
  model_ready_ = true;
}

void TorqueSimArm::InitializeState() {
  q_.setZero();
  qd_.setZero();
  tau_command_.setZero();
  if (!initial_positions_.empty()) {
    q_ = Eigen::Map<const Eigen::VectorXd>(
        initial_positions_.data(), kArmDof);
  }
  q_ = q_.cwiseMax(lower_limit_).cwiseMin(upper_limit_);

  joint_state_.name = joint_names_;
  joint_state_.position.resize(kArmDof);
  joint_state_.velocity.resize(kArmDof);
  joint_state_.current.resize(kArmDof);
  joint_state_.voltage.assign(kArmDof, 48.0);
  joint_state_.temperature.assign(kArmDof, 25.0);
  PublishState();
}

bool TorqueSimArm::SetMotorCommand(
    const dummy_interface::msg::MotorControl& cmd) {
  if (cmd.enable == 0) {
    tau_command_.setZero();
    return true;
  }
  if (cmd.current.size() != kArmDof) {
    return false;
  }
  for (int i = 0; i < kArmDof; ++i) {
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
  q_full.tail(kArmDof) = q_;
  return q_full;
}

Eigen::VectorXd TorqueSimArm::MakeFullVelocity() const {
  Eigen::VectorXd v_full = Eigen::VectorXd::Zero(model_.nv);
  v_full.head(kBaseVelocitySize) = base_state_.velocity;
  v_full.tail(kArmDof) = qd_;
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
      mass.bottomRightCorner(kArmDof, kArmDof);
  const Eigen::MatrixXd base_coupling =
      mass.bottomLeftCorner(kArmDof, kBaseVelocitySize);
  const Eigen::VectorXd rhs =
      tau_command_ - base_coupling * base_state_.acceleration
      - bias.tail(kArmDof);
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
    for (int i = 0; i < kArmDof; ++i) {
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
  for (int i = 0; i < kArmDof; ++i) {
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
