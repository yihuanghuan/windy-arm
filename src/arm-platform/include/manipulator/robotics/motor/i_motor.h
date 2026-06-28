#pragma once
#include <memory>
#include <dummy_interface/msg/motor_control.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace manipulator::motor {

class IMotor {
 public:
  using SharedPtr = std::shared_ptr<IMotor>;

  virtual ~IMotor() = default;

  virtual void UpdateState() = 0;
  virtual void UpdateCommand(const dummy_interface::msg::MotorControl& cmd) = 0;

  virtual double GetPosition() const = 0;
  virtual double GetVelocity() const = 0;
  virtual double GetCurrent() const = 0;
  virtual double GetTemperature() const = 0;
  virtual double GetVoltage() const = 0;

  virtual double GetRatedTorque() const = 0;
  virtual void SetRateTorque(double rate_torque) = 0;
  virtual void SetJointLimit(double lower_limit, double upper_limit) = 0;
  virtual void SetDefaultGains(double kp, double kd) = 0;
};

} // namespace manipulator::motor