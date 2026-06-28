#pragma once

#include <string>
#include <map>
#include <manipulator/robotics/arm/abs_arm.h>

namespace manipulator::arm {

class ArmFactory {
 public:
  static ArmFactory& Instance();
  AbsArm::UniPtr Create(const std::string& arm_name);
 private:
  ArmFactory();
  ~ArmFactory() = default;
  // ArmFactory(const ArmFactory&) = delete;
  // ArmFactory& operator=(const ArmFactory&) = delete;
  // ArmFactory(ArmFactory&&) = delete;
  // ArmFactory& operator=(ArmFactory&&) = delete;
  std::map<std::string, AbsArm::UniPtr> arms_;
};

}  // namespace manipulator::arm
