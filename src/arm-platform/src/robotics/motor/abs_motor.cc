#include <manipulator/robotics/motor/abs_motor.h>

namespace manipulator::motor {
AbsMotor::AbsMotor(bus::IBus::SharedPtr bus, protocol::IProtocol::SharedPtr protocol) {

}

void AbsMotor::UpdateState(const std::vector<uint8_t>& data) {
  protocol_->Pop(data);
}

void AbsMotor::UpdateCommand(const dummy_interface::msg::MotorControl& cmd) {
  std::vector<uint8_t> bytes_to_send;
  protocol_->MakeFrame(cmd, bytes_to_send);
}
}