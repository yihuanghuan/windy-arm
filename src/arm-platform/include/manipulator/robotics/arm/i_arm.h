#pragma once
#include <vector>
#include <memory>
#include <string>
#include <map>
#include <sensor_msgs/msg/joint_state.hpp>
#include <dummy_interface/msg/motor_control.hpp>

namespace manipulator::arm {

struct JointLimit {
  double min_position;
  double max_position;
  double max_velocity;
  double max_effort;
};

class IArm {
 public:
  virtual ~IArm() = default;

  virtual bool SetMotorCommand(const dummy_interface::msg::MotorControl& cmd) = 0;
  virtual void UpdateJointStates() = 0;
};

} // namespace manipulator::arm