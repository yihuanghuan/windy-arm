#pragma once
#include <memory>
#include <dummy_interface/msg/motor_control.hpp>
#include <manipulator/robotics/motor/i_motor.h>
#include <manipulator/robotics/protocol/i_protocol.h>

namespace manipulator::motor {

class AbsMotor : public IMotor {
 public:
  AbsMotor(protocol::IProtocol::SharedPtr protocol);
  virtual ~AbsMotor() = default;

  // IMotor interface implementation
  void UpdateState(const std::vector<uint8_t>& data) override;
  void UpdateCommand(const dummy_interface::msg::MotorControl& cmd) override;
 
 private:
  protocol::IProtocol::SharedPtr protocol_;
};

} // namespace manipulator::motor