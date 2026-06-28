#pragma once
#include <vector>
#include <memory>
#include <map>
#include <string>
#include <chrono>

#include <sensor_msgs/msg/joint_state.hpp>
#include <dummy_interface/msg/motor_control.hpp>
#include <manipulator/robotics/arm/i_arm.h>
#include <manipulator/robotics/motor/i_motor.h>
#include <manipulator/robotics/bus/i_bus.h>
#include <manipulator/common_types.h>

namespace manipulator::arm {

class AbsArm : public IArm {
 public:
  using UniPtr = std::unique_ptr<AbsArm>;

  AbsArm();

  explicit AbsArm(bus::IBus::UniquePtr bus);
    
  AbsArm(const AbsArm&) = delete;
  
  AbsArm& operator=(const AbsArm&) = delete;   
  
  ~AbsArm() = default;

  bool SetMotorCommand(const dummy_interface::msg::MotorControl& cmd);

  JointState& GetJointStates();

  virtual void Init(const std::string& port, uint32_t baud) = 0;
  
  virtual void InitFromConfig(const std::string& port, uint32_t baud,
                              const std::string& motor_config_path, 
                              const std::string& arm_config_path,
                              const std::string& arm_name) {}
  
  std::vector<std::string> GetJointNames() const;

protected:
  void AddMotor(const std::string& name, motor::IMotor::SharedPtr motor);

  void RemoveMotor(const std::string& name);
  
  void SetBus(bus::IBus::UniquePtr bus);
 private:
  void UpdateJointStates() final;

  bus::IBus::UniquePtr bus_;
  
  std::map<std::string, motor::IMotor::SharedPtr> motor_map_;
  std::vector<std::string> joint_names_;
  JointState joint_states_;
};

}