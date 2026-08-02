# Codex Discovery Report: Floating-Base 7DoF Task-Space SMC

## Confirmed Facts

- Local repository already contains the requested codebase under `/home/windylab/code/windylab_ws`.
- `src/arm-platform/config/ll100_lh420.urdf` exists, but it is not a floating-base A-L1-GAMMA 7DoF model.  It has LL100 base/rotor links plus a six-joint arm chain named `A_Joint` through `F_Joint`.
- `src/arm-platform/config/ll100_lh420_nmpc.urdf` is a lightweight NMPC model and also has only six revolute arm joints.
- The 7DoF A-L1-GAMMA URDF available in this repository is `src/arm-platform/config/arm.urdf`, equivalent to `src/dummy_description/urdf/arm/A-L1-GAMMA/arm.urdf`.
- The A-L1-GAMMA TCP/end-effector frame used by existing Pinocchio IK code is `link7`.
- The seven actual arm joints and order are `joint1`, `joint2`, `joint3`, `joint4`, `joint5`, `joint6`, `joint7`.
- Hardware source `AL1Gamma::Init()` maps motor ids 0..6 to `joint1..joint7`.  Coordinate signs are right-hand for joints 1, 2, 3 and 7, left-hand for joints 4, 5 and 6.
- `dummy_interface/msg/MotorControl` fields are:
  - `std_msgs/Header header`
  - `float64[] position`
  - `float64[] velocity`
  - `float64[] current`
  - `float64[] p`
  - `float64[] d`
  - `uint8 enable`
- Command path confirmed from source:
  - `MotorControl.current`
  - `DMMotor::UpdateCommand`
  - `DMMotor::SetCurrent`
  - `ProtocolV1::SetCurrent`
  - `ProtocolV1::Pop()`
  - protocol frame command `0x12` with seven `float` values, 28 data bytes.
- `ProtocolV1` comments state current feedback and desired current are in Amperes, but higher-level controller code uses the field as torque.
- `DMMotor::SetCurrent()` applies coordinate sign conversion and clamps the command to `rate_torque_`.
- `AL1::InitFromConfig()` sets `DMMotor::rate_torque_` from `motor_config.yaml` field `rated_torque`.
- `motor_config.yaml` defines `rated_torque` and `max_torque`, not current limits or torque constants.
- `GravityController::Compute()` writes Pinocchio gravity compensation torques directly to `cmd.current[i]`.
- Existing `/student/joint_command` interface is `sensor_msgs/msg/JointState` position/velocity input.  It does not directly expose `current`.
- Existing `ArmPlatform::NotifyJointState()` publishes measured `current` as `sensor_msgs/JointState.effort`.
- Existing base-motion demos mostly use analytic simulated base trajectories.  `flight_status_bridge` reads MAVROS state/battery only; no source file provides the full fused free-flyer state `(p_B, R_B, v_B, omega_B, a_B, alpha_B)` required by this controller.
- Existing moving-base demo code uses world/base analytic transforms, but it is NMPC/demo code and was not modified.

## Confirmed Repository Command Semantics

- In this repository's abstraction, `MotorControl.current` is torque-like and should be interpreted as joint torque / torque feedforward in Nm-scale units.
- Evidence:
  - `GravityController::Compute()` sends `tau_gravity` through `cmd.current`.
  - `DMMotor::SetCurrent()` clamps by `rated_torque`.
  - `ArmPlatform::PrintDebugInfo()` labels feedback as current in text, but controller math treats it as generalized effort.

## Still Unconfirmed At Firmware Level

- The external firmware/driver scaling behind protocol frame `0x12` is not documented in this repository.
- Source comments use "Amperes", so a bench test should still verify whether `0x12` is exactly Nm, motor-side current mapped internally to torque, or MIT torque feedforward.
- No torque constant, gear ratio, efficiency, or driver scaling document was found.

## Currently Implementable

- Build a strict free-flyer Pinocchio model from `arm.urdf` with `JointModelFreeFlyer()`.
- Use `q_full` dimension 14 and `v_full` dimension 13.
- Compute world-frame end-effector pose, LOCAL_WORLD_ALIGNED twist/Jacobian, mass matrix, bias classical acceleration and full floating-base RNEA.
- Implement quaternion `[w,x,y,z]` math, shortest-arc left error quaternion and SO(3) logarithmic orientation error.
- Implement strict Lie orientation error rate and analytic left-Jacobian-inverse derivative.
- Implement task-space SMC, base acceleration compensation, dynamic-consistent damped inverse, nullspace PD and offline arm dynamics integration with prescribed base motion.
- Publish a new ROS2 `dummy_interface/msg/MotorControl` command on `/student/joint_effort_command` without changing `/student/joint_command`.
- Route `/student/joint_effort_command` through `StudentArmNode` when `enable_direct_effort_command:=true`, bypassing `SmoothPositionController`.

## Interfaces Still Needed

- Bench validation of protocol `0x12` firmware scaling for safe real-hardware torque control.
- A fused base-state ROS source with world-frame position, orientation, linear/angular velocity, linear acceleration and angular acceleration in Pinocchio free-flyer tangent convention.
- Real-time safety supervision around torque/current enable, driver state, estop, stale state, and command watchdog before physical testing.

## Coordinate Convention Used by the New Module

- Base pose is world to base body placement: `p_B^W`, `R_B^W`.
- Internal quaternions are `[w,x,y,z]`; Pinocchio free-flyer `q` stores base quaternion as `[x,y,z,w]`.
- End-effector twist, acceleration, Jacobian, reference trajectory and errors use `pin.LOCAL_WORLD_ALIGNED`.
- The full Jacobian is split as `J_base = J_full[:, :6]`, `J_arm = J_full[:, 6:]`.

## Safety Conclusion

This implementation is suitable for unit tests, offline dynamics simulation, and ROS direct-command integration testing.  It should only be used on real hardware at very low torque limits after confirming protocol `0x12` scaling on the bench.
