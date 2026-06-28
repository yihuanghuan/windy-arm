#include <manipulator/robotics/motor/dm_motor.h>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>

auto logger = rclcpp::get_logger("Controller");
namespace manipulator::motor {

DMMotor::DMMotor(protocol::ProtocolV1::SharedPtr protocol, uint8_t id, 
                 CoordinateSystem coord_system)
 : protocol_(protocol), id_(id), position_(0), velocity_(0), torque_(0),
   temperature_(0), voltage_(0), rate_torque_(0), pos_set_(0), vel_set_(0),
   is_received_(false), lower_limit_(0), upper_limit_(0), pos_limit_(0),
   coord_system_(coord_system) {
}

void DMMotor::SetRateTorque(double rate_torque) {
  rate_torque_ = rate_torque;
}

void DMMotor::SetJointLimit(double lower_limit, double upper_limit) {
  lower_limit_ = lower_limit;
  upper_limit_ = upper_limit;
}

void DMMotor::SetDefaultGains(double kp, double kd) {
  p_gain_default_ = kp;
  d_gain_default_ = kd;
}

double DMMotor::GetRatedTorque() const {
  return rate_torque_;
}

void DMMotor::UpdateState() {
  position_ = protocol_->GetPosition(id_);
  velocity_ = protocol_->GetVelocity(id_);
  torque_ = protocol_->GetCurrent(id_);
  temperature_ = protocol_->GetTemperature(id_);
  
  if (coord_system_ == CoordinateSystem::LeftHand) {
    position_ = -position_;
    velocity_ = -velocity_;
    torque_ = -torque_;
  }
  
  is_received_ = true;
}

void DMMotor::UpdateCommand(const dummy_interface::msg::MotorControl& cmd) {
  if (not is_received_) return;

  if (id_ >= cmd.p.size() or id_ >= cmd.d.size() or id_ >= cmd.current.size()) {
    return;
  }

  protocol_->SetKp(id_, cmd.p[id_]);
  protocol_->SetKd(id_, cmd.d[id_]);
  SetCurrent(cmd.current[id_]);

  double desired_pos = cmd.position[id_];
  
  // set kp kd to default value and set position to limit if position is out of range
  if (position_ < lower_limit_+0.05 or position_ > upper_limit_-0.05) {
    if (cmd.p[id_] == 0) {
      protocol_->SetKp(id_, p_gain_default_);
      protocol_->SetKd(id_, d_gain_default_);
      desired_pos = std::clamp(position_, lower_limit_+0.05, upper_limit_-0.05);
    }
    // if position is in range, set desired position to position
    else if(cmd.position[id_] >= lower_limit_+0.05 and cmd.position[id_] <= upper_limit_-0.05) {
      desired_pos = cmd.position[id_];
    } else {
      desired_pos = std::clamp(position_, lower_limit_+0.05, upper_limit_-0.05);
    }
  }
  
  // set position to current position if limit is reached
  SetPositionAndVelocity(desired_pos, cmd.velocity[id_]);

}

void DMMotor::SetCurrent(double current) const {
  if (coord_system_ == CoordinateSystem::LeftHand) {
    current = -current;
  }
  current = std::clamp(current, -rate_torque_, rate_torque_);
  protocol_->SetCurrent(id_, current);
}

void DMMotor::SetPositionAndVelocity(double pos, double vel) {

  double pos_err = pos - position_;
  pos_set_ = pos;
  double vel_cmd = vel;
  double pos_set_send = pos_set_;

  if (coord_system_ == CoordinateSystem::LeftHand) {
    vel_cmd = -vel_cmd;
    pos_set_send = -pos_set_;
  }
  
  protocol_->SetPosition(id_, pos_set_send);
  protocol_->SetVelocity(id_, vel_cmd);
}

double DMMotor::GetPosition() const {
  return position_;
}

double DMMotor::GetVelocity() const {
  return velocity_;
}

double DMMotor::GetCurrent() const {
  return torque_;
}

double DMMotor::GetTemperature() const {
  return temperature_;
}

double DMMotor::GetVoltage() const {
  return voltage_;
}
}