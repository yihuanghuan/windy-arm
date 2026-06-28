#pragma once

#include <manipulator/robotics/arm/abs_arm.h>
#include <manipulator/robotics/protocol/protocol_v1.h>

namespace manipulator::arm {

class AL1 final : public AbsArm {
 public:
  AL1();

  void Init(const std::string& port, uint32_t baud) override;
  
  void InitFromConfig(const std::string& port, uint32_t baud,
                      const std::string& motor_config_path, 
                      const std::string& arm_config_path,
                      const std::string& arm_name) override;

 private:
  protocol::ProtocolV1::SharedPtr protocol_;
};

}