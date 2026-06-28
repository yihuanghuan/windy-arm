#pragma once

#include <manipulator/robotics/arm/abs_arm.h>
#include <manipulator/robotics/protocol/protocol_v1.h>

namespace manipulator::arm {

class AL1Gamma final : public AbsArm {
 public:
  AL1Gamma();

  /**
   * @brief Initialize arm hardware
   * @param port Serial port device path (e.g., "/dev/ttyUSB0")
   * @param baud Baud rate for serial communication
   */ 
  void Init(const std::string& port, uint32_t baud);  

 private:
  protocol::ProtocolV1::SharedPtr protocol_;
};

}