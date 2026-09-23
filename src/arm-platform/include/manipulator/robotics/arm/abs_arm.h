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

  bool SetMotorCommand(const dummy_interface::msg::MotorControl& cmd) override;

  virtual JointState& GetJointStates();

  virtual void Init(const std::string& port, uint32_t baud) = 0;
  
  virtual void InitFromConfig(const std::string& port, uint32_t baud,
                              const std::string& motor_config_path, 
                              const std::string& arm_config_path,
                              const std::string& arm_name) {}
  
  virtual std::vector<std::string> GetJointNames() const;

  virtual void Step(double /*dt*/) {}

  /**
   * @brief 串口链路是否存活（板子心跳 0x01 在 timeout_sec 内到过）
   *
   * 仿真臂无串口，恒为 true。真机臂 (AL1/AL1Gamma) 用 ProtocolV1 的
   * 心跳计数实现。力矩使能链在输出非零前馈之前必须检查它：心跳断了
   * 意味着关节反馈已经陈旧，基于模型算出的力矩不可信。
   */
  virtual bool IsLinkAlive(double /*timeout_sec*/) const { return true; }
  virtual bool HasFreshFeedback(double /*timeout_sec*/) const { return true; }
  // Zero denotes a simulated arm with no serial frame sequence.
  virtual uint64_t GetFeedbackSequence() const { return 0; }
  virtual void SetBaseState(const FloatingBaseState& /*state*/) {}
  virtual void SetModelPath(const std::string& /*model_path*/) {}
  virtual void SetInitialJointPositions(
      const std::vector<double>& /*positions*/) {}

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
