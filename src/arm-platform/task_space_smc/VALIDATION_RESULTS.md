# Validation Results

Date: 2026-07-30

## Passed Unit Tests

Command:

```bash
PYTHONPATH=src/arm-platform:${PYTHONPATH} python3 -m pytest -q \
  src/arm-platform/test/test_quaternion_math.py \
  src/arm-platform/test/test_pose_error.py \
  src/arm-platform/test/test_floating_base_model.py \
  src/arm-platform/test/test_dynamic_consistent_inverse.py \
  src/arm-platform/test/test_nullspace.py \
  src/arm-platform/test/test_smc_offline.py
```

Result:

```text
10 passed
```

Covered checks:

- Quaternion sign invariance, zero error, 90 degree, 179 degree and small-angle errors.
- Quaternion logarithmic error consistency with `pin.log3`.
- ROS `[x,y,z,w]` and internal `[w,x,y,z]` conversion.
- `J_l J_l^{-1} ~= I` and analytic `dot(J_l^{-1})` finite-difference validation.
- Strict orientation-rate error using `omega_d - R_e omega`.
- Floating-base dimensions `nq=14`, `nv=13`, `J_full=(6,13)`, `J_arm=(6,7)` and 7D RNEA arm torque.
- Pinocchio task bias classical acceleration against central-difference end-effector twist acceleration.
- Full-rank dynamic-consistent inverse and nullspace projection.
- Damped nullspace leakage reporting.
- Short offline simulation smoke test with finite metrics.

## Passed Offline Dynamics Simulation

Command:

```bash
PYTHONPATH=src/arm-platform:${PYTHONPATH} python3 \
  src/arm-platform/task_space_smc/offline_simulation.py \
  --profile sixdof --reference-mode hold --duration 1.0 \
  --output-dir /tmp/task_space_smc_offline
```

Metrics:

- Final position error: `0.006766705388093337 m`
- Max position error: `0.011448717881889807 m`
- Final orientation geodesic error: `0.06987147299637812 rad`
- Max orientation geodesic error: `0.07010124990164006 rad`
- Min scaled singular value: `0.012176767644474602`
- Max damping: `0.05`
- Max nullspace leakage: `0.0008553275948442221`
- Torque saturation ratio: `0.0`
- CSV: `/tmp/task_space_smc_offline/offline_sixdof_hold.csv`
- Metrics JSON: `/tmp/task_space_smc_offline/metrics_sixdof_hold.json`

Command:

```bash
PYTHONPATH=src/arm-platform:${PYTHONPATH} python3 \
  src/arm-platform/task_space_smc/offline_simulation.py \
  --profile sixdof --reference-mode circle --duration 1.0 \
  --output-dir /tmp/task_space_smc_offline
```

Metrics:

- Final position error: `0.005231631285164656 m`
- Max position error: `0.009184774981018507 m`
- Final orientation geodesic error: `0.06998159712670311 rad`
- Max orientation geodesic error: `0.07019584329927123 rad`
- Min scaled singular value: `0.012082320382413966`
- Max damping: `0.05`
- Max nullspace leakage: `0.0008663582122621687`
- Torque saturation ratio: `0.0`
- CSV: `/tmp/task_space_smc_offline/offline_sixdof_circle.csv`
- Metrics JSON: `/tmp/task_space_smc_offline/metrics_sixdof_circle.json`

## ROS Interface Status

- Implemented `torque_controller_node.py`.
- Publishes `dummy_interface/msg/MotorControl` to `/student/joint_effort_command`.
- Does not modify or replace the existing `/student/joint_command` position interface.
- `StudentArmNode` now subscribes to `/student/joint_effort_command` when `enable_direct_effort_command:=true`.
- Direct commands bypass `SmoothPositionController` and call `ArmPlatform::ExecuteDirectMotorCommand()`.
- The whole-task runner uses fixed-step simulation time in `mode:=simulation`, so ROS timer jitter does not get injected into the virtual plant dynamics.
- The simulation plant clamps the 7 revolute arm joints to the URDF limits and zeros outward velocity at a saturated limit.
- Whole-task redundancy resolution uses `nullspace.joint_acceleration_weight` to avoid the low-inertia wrist joints absorbing long-horizon orientation motion.

## Passed ROS Whole-Task Simulation

Command:

```bash
source install/setup.bash
ROS_LOG_DIR=/tmp/ros_logs PYTHONPATH=src/arm-platform:${PYTHONPATH} python3 \
  src/arm-platform/task_space_smc/whole_task_smc_ros_runner.py \
  --ros-args \
  -p config_path:=src/arm-platform/task_space_smc/config/smc_params_whole.yaml \
  -p hardware_enabled:=true \
  -p mode:=simulation \
  -p data_dir:=/tmp/task_space_smc_whole_full
```

Metrics:

- Whole period: `73.03949045888578 s`
- Samples: `36520`
- Position RMS error: `1.8925362412729963e-05 m`
- Position peak error: `0.0001546893692140795 m`
- Orientation RMS error: `0.008680174417458855 rad`
- Orientation peak error: `0.12114731714266513 rad`
- Requirement check, RMS `< 0.001 m` and peak `< 0.002 m`: `true`
- Per-joint peak torque command: `[3.19884508, 4.29202701, 3.0, 0.678162867, 0.463012204, 0.0555524942, 0.000307126606] Nm`
- The per-joint peaks are below the repository motor rated-torque clamps: `[9, 9, 9, 1.6, 1.6, 1.6, 1.6] Nm`.
- CSV: `/tmp/task_space_smc_whole_full/whole_task_smc_metrics.csv`

Runtime note:

- This sandbox reports FastDDS UDP socket permission warnings (`getifaddrs` / `TRANSPORT_UDP`), but the single-process ROS runner still executes and writes the metrics above.

Build command:

```bash
colcon build --packages-select manipulator --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Result:

```text
Summary: 1 package finished
```

## Real-Hardware Current Unit Status

- Confirmed at repository abstraction level as torque-like Nm-scale command.
- Evidence: `GravityController::Compute()` sends Pinocchio torques through `cmd.current`; `DMMotor::SetCurrent()` clamps by `rated_torque`.
- Firmware scaling of protocol `0x12` still requires bench validation before aggressive hardware use.

## Blocking Conditions For Real Torque/Current Control

- Confirm protocol `0x12` firmware scaling from driver documentation or bench calibration.
- Provide a fused ROS base-state source with world-frame acceleration and angular acceleration in the Pinocchio free-flyer tangent convention.
- Physically validate watchdog, stale-state rejection, estop, torque/current limits and rate limits.
