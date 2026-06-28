#include <manipulator/robotics/arm/a_l1_gamma.h>
#include <manipulator/robotics/protocol/protocol_factory.h>
#include <manipulator/robotics/protocol/protocol_v1.h>
#include <manipulator/robotics/bus/serial_bus.h>
#include <manipulator/robotics/motor/dm_motor.h>

namespace manipulator::arm {

AL1Gamma::AL1Gamma() {

}

void AL1Gamma::Init(const std::string& port, uint32_t baud) {
  protocol_ = std::make_shared<protocol::ProtocolV1>();

  auto joint1 = std::make_shared<motor::DMMotor>(protocol_, 0, motor::CoordinateSystem::RightHand);
  AddMotor("joint1", joint1);
  auto joint2 = std::make_shared<motor::DMMotor>(protocol_, 1, motor::CoordinateSystem::RightHand);
  AddMotor("joint2", joint2);
  auto joint3 = std::make_shared<motor::DMMotor>(protocol_, 2, motor::CoordinateSystem::RightHand);   
  AddMotor("joint3", joint3);
  auto joint4 = std::make_shared<motor::DMMotor>(protocol_, 3, motor::CoordinateSystem::LeftHand);
  AddMotor("joint4", joint4);
  auto joint5 = std::make_shared<motor::DMMotor>(protocol_, 4, motor::CoordinateSystem::LeftHand);
  AddMotor("joint5", joint5);
  auto joint6 = std::make_shared<motor::DMMotor>(protocol_, 5, motor::CoordinateSystem::LeftHand);
  AddMotor("joint6", joint6);
  auto joint7 = std::make_shared<motor::DMMotor>(protocol_, 6, motor::CoordinateSystem::RightHand);
  AddMotor("joint7", joint7);

  protocol_->Attach(joint1);
  protocol_->Attach(joint2);
  protocol_->Attach(joint3);
  protocol_->Attach(joint4);
  protocol_->Attach(joint5);
  protocol_->Attach(joint6);
  protocol_->Attach(joint7);

  auto bus = std::make_unique<bus::SerialBus>(port, baud);
  bus->SetProtocol(protocol_);
  SetBus(std::move(bus));
}
}