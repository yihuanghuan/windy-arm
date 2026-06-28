#include <manipulator/robotics/arm/abs_arm.h>
#include <stdexcept>
#include <algorithm>
#include <thread>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

auto mylogger = rclcpp::get_logger("AbsArm");

namespace manipulator::arm {
AbsArm::AbsArm() {
  
}
AbsArm::AbsArm(bus::IBus::UniquePtr bus) : bus_(std::move(bus)){

}

void AbsArm::SetBus(bus::IBus::UniquePtr bus) {
  bus_ = std::move(bus);
}

bool AbsArm::SetMotorCommand(const dummy_interface::msg::MotorControl& cmd) {
  for (auto& [_, motor] : motor_map_ ) {
    motor->UpdateCommand(cmd);
  }
  bus_->Send(); 
  return true;
}

void AbsArm::AddMotor(const std::string& name, motor::IMotor::SharedPtr motor) {
  std::cout << "add motor" << std::endl;

  motor_map_.emplace(name, motor);
  joint_names_.push_back(name);
  joint_states_.position.push_back(0.0);
  joint_states_.velocity.push_back(0.0);
  joint_states_.current.push_back(0.0);
  joint_states_.voltage.push_back(0.0);
  joint_states_.temperature.push_back(0.0);
  joint_states_.name.push_back(name);
}

void AbsArm::RemoveMotor(const std::string& name) {
  motor_map_.erase(name);
  joint_names_.erase(std::remove(joint_names_.begin(), joint_names_.end(), name), joint_names_.end());

  auto it = std::find(joint_states_.name.begin(), joint_states_.name.end(), name);
  if (it != joint_states_.name.end()) {
    size_t index = std::distance(joint_states_.name.begin(), it);
    joint_states_.name.erase(joint_states_.name.begin() + index);
    joint_states_.position.erase(joint_states_.position.begin() + index);
    joint_states_.velocity.erase(joint_states_.velocity.begin() + index);
    joint_states_.current.erase(joint_states_.current.begin() + index);
    joint_states_.voltage.erase(joint_states_.voltage.begin() + index);
    joint_states_.temperature.erase(joint_states_.temperature.begin() + index);
  }
}

void AbsArm::UpdateJointStates() {
  bus_->Read();

  for (uint8_t i = 0; i < motor_map_.size(); i++) {
    std::string name = joint_states_.name[i];
    auto motor = motor_map_[name];
    joint_states_.position[i] = motor->GetPosition();
    joint_states_.velocity[i] = motor->GetVelocity();
    joint_states_.current[i] = motor->GetCurrent();
    joint_states_.voltage[i] = motor->GetVoltage();
    joint_states_.temperature[i] = motor->GetTemperature();
  }
}

JointState& AbsArm::GetJointStates() {
  UpdateJointStates();
  return joint_states_;
}

std::vector<std::string> AbsArm::GetJointNames() const {
  return joint_names_;
}
} // namespace manipulator::arm
