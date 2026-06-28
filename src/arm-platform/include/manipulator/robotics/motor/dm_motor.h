#pragma once
#include <memory>
#include <manipulator/robotics/motor/i_motor.h>
#include <manipulator/robotics/protocol/protocol_v1.h>

namespace manipulator::motor {

enum class CoordinateSystem {
  RightHand,
  LeftHand
};

class DMMotor final : public IMotor {
 public:
  DMMotor(protocol::ProtocolV1::SharedPtr protocol, uint8_t id, 
          CoordinateSystem coord_system = CoordinateSystem::RightHand);
    
  virtual ~DMMotor() = default;

  void UpdateCommand(const dummy_interface::msg::MotorControl& cmd) override;
  void UpdateState() override;
  void SetRateTorque(double rate_torque) override;

  double GetPosition() const override;
  double GetVelocity() const override;
  double GetCurrent() const override;
  double GetTemperature() const override;
  double GetVoltage() const override;
  double GetRatedTorque() const override;
  void SetJointLimit(double lower_limit, double upper_limit) override;
  void SetDefaultGains(double kp, double kd);
 
 private:
  void SetPositionAndVelocity(double pos, double vel);
  void SetCurrent(double current) const;

  protocol::ProtocolV1::SharedPtr protocol_;
  uint8_t id_;
  double p_gain_default_;
  double d_gain_default_;

  double position_;
  double velocity_;
  double torque_;
  double temperature_;
  double voltage_;
  double rate_torque_;

  double pos_set_;
  double vel_set_;
  bool is_received_;
  const double DT_ = 0.005;
  double lower_limit_;
  double upper_limit_;
  double pos_limit_;
  CoordinateSystem coord_system_;
};

}