#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <manipulator/robotics/arm/abs_arm.h>

namespace manipulator::arm {

class TorqueSimArm final : public AbsArm {
 public:
  TorqueSimArm();

  void Init(const std::string& port, uint32_t baud) override;
  void InitFromConfig(const std::string& port, uint32_t baud,
                      const std::string& motor_config_path,
                      const std::string& arm_config_path,
                      const std::string& arm_name) override;

  bool SetMotorCommand(
      const dummy_interface::msg::MotorControl& cmd) override;
  JointState& GetJointStates() override;
  std::vector<std::string> GetJointNames() const override;
  void Step(double dt) override;
  void SetBaseState(const FloatingBaseState& state) override;
  void SetModelPath(const std::string& model_path) override;
  void SetInitialJointPositions(
      const std::vector<double>& positions) override;

 private:
  void BuildModel();
  void InitializeState();
  Eigen::VectorXd MakeFullConfiguration() const;
  Eigen::VectorXd MakeFullVelocity() const;
  Eigen::VectorXd ComputeArmAcceleration();
  void PublishState();

  std::string model_path_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  bool model_ready_ = false;

  std::vector<std::string> joint_names_;
  std::vector<double> initial_positions_;
  Eigen::VectorXd q_;
  Eigen::VectorXd qd_;
  Eigen::VectorXd tau_command_;
  Eigen::VectorXd torque_limit_;
  Eigen::VectorXd velocity_limit_;
  Eigen::VectorXd lower_limit_;
  Eigen::VectorXd upper_limit_;
  FloatingBaseState base_state_;
  JointState joint_state_;
  double substep_sec_ = 0.001;
};

}  // namespace manipulator::arm
