# Real Robot Calibration Handoff

This folder contains the frozen calibration values used by the moving-base candidate test.
It is a handoff copy. The runtime launch still reads the source-tree files listed below.

## Parameter Files

- `arm6_calibration_candidate.yaml`: joint zero, gravity ratio, friction and inertia candidate.
- `vicon_extrinsics.yaml`: Y-up Vicon base transform and fixed link6-to-marker transform.

Runtime source files:

- `src/arm-platform/config/arm6_calibration_candidate.yaml`
- `src/arm-platform/task_space_smc/config/vicon_extrinsics.yaml`

## Launch Order

### 1. Check the serial device

```bash
ls -l /dev/ttyUSB0
```

This confirms that the USB-UART device is visible. The user must also confirm that the arm is powered and mechanically clear.

### 2. Start the six-joint hardware driver

```bash
cd /home/zhangziqian/windylab_ws/lagrange_smc/LL100-LH420
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch manipulator student_arm6.launch.py \
  arm_type:=a_l1 \
  port_name:=/dev/ttyUSB0 \
  enable_direct_effort_command:=true \
  use_rviz:=true
```

This starts `student_arm_node` for the six-joint A-L1 hardware and publishes joint feedback. It does not automatically unfold the arm.

### 3. Manually place the arm safely

Move the arm to the known safe, expanded starting posture using the approved position/manual procedure. Do not start the hold controller from a folded posture.

### 4. Start Vicon and the candidate moving-base controller

```bash
cd /home/zhangziqian/windylab_ws/lagrange_smc/LL100-LH420
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch manipulator vicon_moving_base_candidate_test.launch.py \
  vicon_server:=192.168.200.60 \
  vicon_port:=3883 \
  hardware_enabled:=true \
  control_rate_hz:=200.0
```

This starts the VRPN listener, Vicon bridge, and `whole_task_smc_ros_runner.py`. The runner captures the current FK end-effector pose as the hold target instead of using a prewritten trajectory.

## Startup Checks

Before allowing effort control, verify the launch output contains:

```text
ARM6_CALIBRATION_YAML=.../src/arm-platform/config/arm6_calibration_candidate.yaml
VICON_EXTRINSICS_YAML=.../src/arm-platform/task_space_smc/config/vicon_extrinsics.yaml
hardware_enabled=true
```

Also verify `/joint_states`, `/vrpn/arm_base/pose`, `/vrpn/arm_ee/pose`, and `/floating_base/vicon` are active. Stop immediately with `Ctrl+C` if Vicon reports stale data, source timestamp disorder, `No response`, missing acceleration, joint-limit errors, or an invalid FK state.

## Parameter Semantics

The active model uses:

```text
q_model = q_encoder + zero_offset_rad
tau_corrected = (tau_rnea - gravity) + first_moment_ratio * gravity
```

Directional kinetic friction uses `kinetic_friction_pos_nm` or `kinetic_friction_neg_nm` according to joint velocity sign. The fixed Vicon base transform is used for moving-base state; the arm_ee orientation is validation-only and control orientation remains the Pinocchio/link6 FK orientation.

## Important Network Note

The last diagnostic observed Vicon source/order and transport gaps up to the second range. Do not run the `hardware_enabled:=true` command until the Vicon stream is stable and the safety operator has approved the test.
