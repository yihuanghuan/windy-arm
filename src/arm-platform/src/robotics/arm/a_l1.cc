#include <manipulator/robotics/arm/a_l1.h>
#include <manipulator/robotics/protocol/protocol_factory.h>
#include <manipulator/robotics/protocol/protocol_v1.h>
#include <manipulator/robotics/bus/serial_bus.h>
#include <manipulator/robotics/motor/dm_motor.h>
#include <manipulator/robotics/config/arm_config.h>
#include <yaml-cpp/yaml.h>

namespace manipulator::arm {

AL1::AL1() {

}

void AL1::Init(const std::string& port, uint32_t baud) {
  throw std::runtime_error("AL1::Init is not implemented, use InitFromConfig instead");
}

void AL1::InitFromConfig(const std::string& port, uint32_t baud,
                         const std::string& motor_config_path, 
                         const std::string& arm_config_path,
                         const std::string& arm_name) {
  YAML::Node yaml = YAML::LoadFile(motor_config_path);
  YAML::Node arm_yaml = YAML::LoadFile(arm_config_path);
  
  auto motor_models = config::ConfigLoader::LoadMotorModels(yaml);
  auto arm_config = config::ConfigLoader::LoadArmConfig(arm_yaml, "a_l1_" + arm_name);
  
  if (arm_config.joints.empty()) {
    throw std::runtime_error("Arm config is empty");
  }

  protocol_ = std::make_shared<protocol::ProtocolV1>();
  
  for (const auto& joint : arm_config.joints) {
    auto& model_config = motor_models[joint.model];
    auto coord_system = model_config.coord_system == "right_hand" ?
      motor::CoordinateSystem::RightHand : motor::CoordinateSystem::LeftHand;
    auto motor = std::make_shared<motor::DMMotor>(protocol_, joint.id, coord_system);
    motor->SetRateTorque(model_config.rated_torque);
    motor->SetJointLimit(joint.lower_limit, joint.upper_limit);
    motor->SetDefaultGains(model_config.p_gain_default, model_config.d_gain_default);
    AddMotor(joint.name, motor);
    protocol_->Attach(motor);
  }
  
  auto bus = std::make_unique<bus::SerialBus>(port, baud);
  bus->SetProtocol(protocol_);
  SetBus(std::move(bus));
}

}