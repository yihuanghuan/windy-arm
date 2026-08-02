#include <manipulator/robotics/arm/arm_factory.h>
#include <manipulator/robotics/arm/a_l1.h>
#include <manipulator/robotics/arm/a_l1_gamma.h>
#include <manipulator/robotics/arm/sim_arm.h>
#include <manipulator/robotics/arm/torque_sim_arm.h>
#include <stdexcept>

namespace manipulator::arm {

ArmFactory::ArmFactory() {
  arms_.emplace("a_l1", std::make_unique<AL1>());
  arms_.emplace("a_l1_gamma", std::make_unique<AL1Gamma>());
  arms_.emplace("sim", std::make_unique<SimArm>());
  arms_.emplace("torque_sim", std::make_unique<TorqueSimArm>());
  arms_.emplace("ll100_sim", std::make_unique<LL100SimArm>());
}

ArmFactory& ArmFactory::Instance() {
  static ArmFactory instance;
  return instance;
}

AbsArm::UniPtr ArmFactory::Create(const std::string& arm_name) {
  auto it = arms_.find(arm_name);
  if (it == arms_.end()) {
    throw std::invalid_argument("Unknown arm name: " + arm_name);
  }
  return std::move(it->second);
}

}
