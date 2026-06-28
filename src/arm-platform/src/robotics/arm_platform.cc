#include <algorithm>
#include <iostream>
#include <manipulator/robotics/arm_platform.h>

namespace manipulator {

ArmPlatform::ArmPlatform() = default;

ArmPlatform::~ArmPlatform() = default;

void ArmPlatform::SetArm(arm::AbsArm::UniPtr arm) {
  arm_ = std::move(arm);
}

void ArmPlatform::SetController(controller::IArmController::UniPtr controller) {
  controller_ = std::move(controller);
}

void ArmPlatform::SetPlanner(planning::AbsMotionPlanner::UniPtr planner) {
  planner_ = std::move(planner);
  // Plan and generate when planner is set
  planner_->Plan(arm_state_);
}

void ArmPlatform::SetJointSetpoint(const planning::JointSetpoint& setpoint) {
  joint_setpoint_ = setpoint;
}

void ArmPlatform::SetCollisionAvoidance(collision::CollisionAvoidance::Ptr collision_avoidance) {
  collision_avoidance_ = collision_avoidance;
}

void ArmPlatform::EnableCollisionAvoidance(bool enable) {
  collision_avoidance_enabled_ = enable;
}

void ArmPlatform::AddSubscribe(IArmDataSubscriber::SharedPtr subscriber) {
  if (!subscriber) return;

  auto it = std::find_if(subscribers_.begin(), subscribers_.end(),
    [&](const auto& w) {
        auto sp1 = w.lock();
        return sp1 && sp1 == subscriber;
    });

  if (it == subscribers_.end()) {
      subscribers_.push_back(subscriber);
  }
}

bool ArmPlatform::ExecuteControlCycle(double dt) {
  if (dt < 0.01) {
    throw std::runtime_error("Control cycle dt must be greater than 0.01");
  }
  if (!arm_) {
    throw std::runtime_error("Arm not set");
  }

  arm_state_ = arm_->GetJointStates();
  joint_names_ = arm_->GetJointNames();

  if (planner_) {
    joint_setpoint_ = planner_->GetTrajectoryPoint();
  }
  bool has_collision_risk = false;
  if (collision_avoidance_enabled_ && collision_avoidance_) {
    has_collision_risk = ApplyCollisionAvoidance();
  }

  if (controller_) {
    if (joint_setpoint_.q.size() > 0 and joint_setpoint_.q.size() != arm_state_.position.size()) {
      std::cout << "Joint setpoint size " << joint_setpoint_.q.size() 
        << " is not equal than joint position size " << arm_state_.position.size() << std::endl;
      return false;
    }

    if (not has_collision_risk) {
      cmd_ = controller_->Compute(arm_state_, joint_setpoint_, dt);
      arm_->SetMotorCommand(cmd_);
    }

  } else {
    return false;
  }

  NotifyJointState();
  NotifyMotorFeedback();

  // Check if planner is done
  if(planner_ && planner_->IsDone()) {
    return true;
  }

  return false;
}

void ArmPlatform::NotifyJointState() {
  sensor_msgs::msg::JointState joint_state_msg;
  // joint_state_msg.header.stamp = rclcpp::Clock().now();
  joint_state_msg.header.frame_id = "base_link";
  joint_state_msg.name.assign(joint_names_.begin(), joint_names_.end());
  joint_state_msg.position = arm_state_.position;
  joint_state_msg.velocity = arm_state_.velocity;
  joint_state_msg.effort = arm_state_.current;
  for (auto& subscriber : subscribers_) {
    auto shared_subscriber = subscriber.lock();
    if (shared_subscriber) {
      shared_subscriber->UpdateJointState(joint_state_msg);
    }
  }
}

void ArmPlatform::NotifyMotorFeedback() {
  dummy_interface::msg::MotorState joint_feedback_msg;
  // joint_feedback_msg.header.stamp = rclcpp::Clock().now();
  joint_feedback_msg.position = arm_state_.position;
  joint_feedback_msg.velocity = arm_state_.velocity;
  joint_feedback_msg.current = arm_state_.current;
  joint_feedback_msg.temperature = arm_state_.temperature;
  for (auto& subscriber : subscribers_) {
    auto shared_subscriber = subscriber.lock();
    if (shared_subscriber) {
      shared_subscriber->UpdateMotorFeedback(joint_feedback_msg);
    }
  }
}

bool ArmPlatform::ApplyCollisionAvoidance() const {
  if (!collision_avoidance_ || joint_setpoint_.q.size() == 0) {
    return false;
  }

  Eigen::VectorXd current_q(arm_state_.position.size());
  for (size_t i = 0; i < arm_state_.position.size(); ++i) {
    current_q[i] = arm_state_.position[i];
  }

  Eigen::VectorXd target_q = joint_setpoint_.q;

  collision::CollisionResult result = collision_avoidance_->CheckCollision(target_q);
  
  if (result.has_collision_risk) {
    // DO NOT MOVE when collision risk detected, consider adjusting target position later
    // Eigen::VectorXd adjusted_q = collision_avoidance_->AdjustTarget(current_q, target_q);
    // joint_setpoint_.q = adjusted_q;
    
    std::cout << "Collision risk detected! Min distance: " << result.min_distance 
              << ", adjusted target." << std::endl;
    return true;
  }
  
  return false;
}

void ArmPlatform::PrintDebugInfo() {
  std::cout << "Joint position (rad): [" 
            << arm_state_.position[0] << ", " 
            << arm_state_.position[1] << ", " 
            << arm_state_.position[2] << ", " 
            << arm_state_.position[3] << ", " 
            << arm_state_.position[4] << ", " 
            << arm_state_.position[5] << ", " 
            << arm_state_.position[6] << "]" << std::endl;
            
  std::cout << "Joint currents (A): [" 
            << arm_state_.current[0] << ", " 
            << arm_state_.current[1] << ", " 
            << arm_state_.current[2] << ", " 
            << arm_state_.current[3] << ", " 
            << arm_state_.current[4] << ", " 
            << arm_state_.current[5] << ", " 
            << arm_state_.current[6] << "]" << std::endl;
}

}  // namespace manipulator
