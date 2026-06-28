#pragma once

#include <array>
#include <string>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <manipulator/controller/i_arm_controller.h>

namespace manipulator::controller {

struct SoftLimitParams {
  double margin;
  std::array<double, 7> stiffness;
  std::array<double, 7> damping;
  std::array<double, 7> tau_max;
};

class GravityController : public IArmController {
 public:
  GravityController();
  ~GravityController() = default;

  JointCommand Compute(const JointStates& joint_states,
      const JointSetpoint& joint_set_point,
      double dt) override;

  bool LoadModel(const std::string& urdf_path);
  void SetParams(double GRAVITY, double FORCE_FEEDBACK_THRESHOLD, double FORCE_FEEDBACK_GAIN);
  void SetCollisionCoeffs(const std::array<double, 7>& coeffs);
  void SetForceFeedback(const std::array<double, 7>& joint_currents, const std::array<double, 7>& compensation_torques);
  void SetUavPose(const geometry_msgs::msg::Point& pose);
  void SetRotationAngle(double roll, double pitch, double yaw);
  void SetSoftLimitParams(double margin, const std::array<double, 7>& stiffness,
                         const std::array<double, 7>& damping, const std::array<double, 7>& tau_max);
 private:
  void UpdateGravityVector();
  std::array<double, 7> CollisionDetection(const std::array<double, 7>& tau_comp,
                                           const std::array<double, 7>& joint_currents_,
                                           const std::array<double, 7>& compensation_torques);

  pinocchio::Model model_;
  pinocchio::Data data_;

  struct CompensationParams {
    double GRAVITY;
    double FORCE_FEEDBACK_THRESHOLD;
    double FORCE_FEEDBACK_GAIN;
    std::array<double, 7> collision_coeffs;
  };
  CompensationParams params_;
  SoftLimitParams soft_limit_params_;

  geometry_msgs::msg::Point uav_pose_;
  Eigen::Matrix3d rotation_matrix_;
  Eigen::Vector3d gravity_vector_;
  bool model_loaded_;
  std::array<double, 7> uav_joint_currents_ = {0};
  std::array<double, 7> uav_compensation_torques_ = {0};
  bool force_feedback_available_ = false;
};

}