#pragma once

#include <memory>
#include <sensor_msgs/msg/joint_state.hpp>
#include <dummy_interface/msg/motor_state.hpp>

class IArmDataSubscriber {
 public:
  using WeakPtr = std::weak_ptr<IArmDataSubscriber>;
  using SharedPtr = std::shared_ptr<IArmDataSubscriber>;

  IArmDataSubscriber() = default;
  virtual ~IArmDataSubscriber() = default;

  virtual void UpdateJointState(sensor_msgs::msg::JointState& msg) = 0;
  virtual void UpdateMotorFeedback(dummy_interface::msg::MotorState& msg) = 0;
};
