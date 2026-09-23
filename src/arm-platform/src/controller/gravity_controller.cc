#include <manipulator/controller/gravity_controller.h>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <iostream>

namespace manipulator::controller {

GravityController::GravityController()
    : model_loaded_(false) {
  params_.GRAVITY = 9.81;
  params_.FORCE_FEEDBACK_THRESHOLD = 0.2;
  params_.FORCE_FEEDBACK_GAIN = 0.2;

  soft_limit_params_.margin = 0.02;
  soft_limit_params_.stiffness = {20.0, 20.0, 20.0, 5, 5, 5, 5};
  soft_limit_params_.damping = {2.0, 2.0, 2.0, 3.0, 3.0, 3.0, 3.0};
  soft_limit_params_.tau_max = {9.0, 9.0, 9.0, 1.5, 1.5, 1.5, 1.5};

  uav_pose_.x = 0.0;
  uav_pose_.y = 0.0;
  uav_pose_.z = 0.0;

  rotation_matrix_ = Eigen::Matrix3d::Identity();
  gravity_vector_ = Eigen::Vector3d(0, 0, -params_.GRAVITY);
}

bool GravityController::LoadModel(const std::string& urdf_path) {
  try {
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    model_loaded_ = true;
    UpdateGravityVector();
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Failed to load URDF model: " << e.what() << std::endl;
    model_loaded_ = false;
    return false;
  }
}

void GravityController::SetParams(double GRAVITY,
                                  double FORCE_FEEDBACK_THRESHOLD, double FORCE_FEEDBACK_GAIN) {
  params_.GRAVITY = GRAVITY;
  params_.FORCE_FEEDBACK_THRESHOLD = FORCE_FEEDBACK_THRESHOLD;
  params_.FORCE_FEEDBACK_GAIN = FORCE_FEEDBACK_GAIN;
  params_.collision_coeffs = {1.0, 1.0, 1.0, 0.3, 0.3, 0.3, 0.3};
  UpdateGravityVector();
}

void GravityController::SetCollisionCoeffs(const std::array<double, 7>& coeffs) {
  params_.collision_coeffs = coeffs;
}

void GravityController::SetUavPose(const geometry_msgs::msg::Point& pose) {
  uav_pose_ = pose;
  UpdateGravityVector();
}

void GravityController::SetRotationAngle(double roll, double pitch, double yaw) {
  Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  
  rotation_matrix_ = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
  UpdateGravityVector();
}

void GravityController::SetSoftLimitParams(double margin, const std::array<double, 7>& stiffness,
                                            const std::array<double, 7>& damping,
                                            const std::array<double, 7>& tau_max) {
  soft_limit_params_.margin = margin;
  soft_limit_params_.stiffness = stiffness;
  soft_limit_params_.damping = damping;
  soft_limit_params_.tau_max = tau_max;
}

void GravityController::UpdateGravityVector() {
  Eigen::Vector3d gravity_world(0, 0, -params_.GRAVITY);
  gravity_vector_ = rotation_matrix_ * gravity_world;
  
  if (model_loaded_) {
    model_.gravity.linear(gravity_vector_);
  }
}

JointCommand GravityController::Compute(const JointStates& joint_states,
    const JointSetpoint& joint_setpoint,
    double dt) {

  if (!model_loaded_) {
    throw std::runtime_error("Model not loaded");
  }
  const size_t num_joints = joint_states.position.size();
  if (num_joints != static_cast<size_t>(model_.nq) || model_.nq != model_.nv)
    throw std::invalid_argument("gravity model/state dimension mismatch");
  Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(
      joint_states.position.data(), num_joints);
  if (!q.allFinite()) throw std::invalid_argument("nonfinite gravity state");
  // URDF inertias are authoritative; the arm6 mass fit is already baked in.
  // Caller supplies model coordinates with its explicit zero offset applied once.
  pinocchio::computeGeneralizedGravity(model_, data_, q);
  if (!data_.g.allFinite()) throw std::runtime_error("nonfinite gravity torque");
  JointCommand cmd;
  cmd.position = joint_states.position;
  cmd.velocity.assign(num_joints, 0.0);
  cmd.current.assign(data_.g.data(), data_.g.data() + num_joints);
  cmd.p.assign(num_joints, 0.0);
  cmd.d.assign(num_joints, 0.0);
  if (force_feedback_available_) {
    for (size_t i = 0; i < num_joints && i < params_.collision_coeffs.size(); ++i) {
      const double delta = uav_joint_currents_[i] - uav_compensation_torques_[i];
      if (std::abs(delta) > params_.FORCE_FEEDBACK_THRESHOLD)
        cmd.current[i] -= params_.collision_coeffs[i] * params_.FORCE_FEEDBACK_GAIN * delta;
    }
  }

  return cmd;
}

std::array<double, 7> GravityController::CollisionDetection(
    const std::array<double, 7>& tau_comp,
    const std::array<double, 7>& joint_currents_,
    const std::array<double, 7>& compensation_torques) {
  std::array<double, 7> tau_current;
  tau_current = tau_comp;
  for (int i = 0; i < 7; ++i) {
    if (std::abs(joint_currents_[i] - compensation_torques[i]) > params_.FORCE_FEEDBACK_THRESHOLD) {
      tau_current[i] += -params_.collision_coeffs[i] * params_.FORCE_FEEDBACK_GAIN * (joint_currents_[i] - compensation_torques[i]);
    } else {
      tau_current[i] += 0.0;
    }
  }
  
  return tau_current;
}

}
