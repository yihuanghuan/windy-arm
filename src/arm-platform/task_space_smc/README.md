# Floating-Base Task-Space SMC

AI continuation should start with [AI_HANDOFF.md](AI_HANDOFF.md), which records
the protected dirty-worktree state, validated shared torque chain, known stale
results, unresolved hardware semantics, and prioritized next steps.

This module is independent from the existing NMPC demo code.  It builds the A-L1-GAMMA 7DoF arm model with a Pinocchio `JointModelFreeFlyer`, tracks world-frame end-effector pose with task-space SMC, compensates prescribed base motion, resolves redundancy with a dynamic-consistent inverse, and computes the seven arm torques with full floating-base RNEA.

## Current Safety Conclusion

Within this repository's control abstraction, `MotorControl.current` is used as a joint torque / torque-feedforward command despite the name.  The strongest source evidence is `GravityController::Compute()`, which writes Pinocchio gravity torques directly into `cmd.current`, and `DMMotor::SetCurrent()`, which clamps by configured `rated_torque`.

Real hardware still needs a bench check before high-gain SMC: protocol command `0x12` is confirmed by source as the path used for this torque-like command, but the external driver firmware scaling is not documented in this repository.

## Model

- URDF: `src/arm-platform/config/arm.urdf`
- End-effector frame: `link7`
- Arm joints: `joint1..joint7`
- Pinocchio dimensions with free-flyer: `nq=14`, `nv=13`
- Twist, acceleration, Jacobian and pose error convention: `pin.LOCAL_WORLD_ALIGNED`

The repository has `src/arm-platform/config/ll100_lh420.urdf`, but it contains the LL100 base links and six `A_Joint..F_Joint` arm joints, not the 7DoF A-L1-GAMMA chain used by the hardware code.  Therefore this module uses `arm.urdf` and adds the free-flyer root in Pinocchio.

## Run Offline

```bash
PYTHONPATH=src/arm-platform python3 src/arm-platform/task_space_smc/offline_simulation.py \
  --profile sixdof --reference-mode hold --duration 1.0
```

Outputs go to `/tmp/task_space_smc_offline` by default: CSV traces and JSON metrics for position error, orientation geodesic error, sliding surfaces, singular value, damping, nullspace leakage and torque saturation.

## ROS2 Execution Chain

```bash
PYTHONPATH=src/arm-platform ros2 run manipulator torque_controller_node.py
```

The SMC publishes `dummy_interface/msg/MotorControl` on
`/student/joint_effort_command`. `StudentArmNode` has an opt-in direct command
path:

```bash
ros2 run manipulator student_arm_node --ros-args \
  -p enable_direct_effort_command:=true \
  -p effort_command_timeout_sec:=0.05
```

When enabled, `/student/joint_effort_command` bypasses `SmoothPositionController` and goes directly to `ArmPlatform::ExecuteDirectMotorCommand() -> DMMotor::UpdateCommand() -> ProtocolV1::SetCurrent()`.

In shared simulation the same ROS boundary is retained, but the arm backend is
`TorqueSimArm`:

```text
whole_task_smc_ros_runner
  -> /student/joint_effort_command (MotorControl.current, Nm)
  -> student_arm_node
  -> ArmPlatform::ExecuteDirectMotorCommand
  -> TorqueSimArm::SetMotorCommand + Step
  -> /joint_states
  -> whole_task_smc_ros_runner
```

`step_on_effort_command=true` makes this simulation deterministic: one effort
command advances the plant by exactly one configured control period, and the
next command is not issued until the corresponding joint feedback arrives.
The normal position-command path remains unchanged for MPC and damped IK.

## RViz Whole-Task Simulation

The launcher runs `student_arm_node` with the Pinocchio `TorqueSimArm`
backend, task-space SMC, `robot_state_publisher`, and RViz. It does not require
acados. Use the workspace helper to isolate this package from stale ROS
overlays:

```bash
cd /home/windylab/code/windylab_ws
src/arm-platform/scripts/run_task_space_smc_whole.sh \
  mode:=simulation start_delay_sec:=3.0 \
  duration_sec:=0.0 hardware_enabled:=false
```

`duration_sec:=0.0` means one complete `whole` period. The helper clears old
overlay variables, builds into this workspace's explicit `build/` and
`install/` directories, verifies the selected package prefix, applies the
Fast DDS UDP-only environment, and launches the demo.

`start_delay_sec:=3.0` starts ROS and RViz immediately, displays the complete
green reference path, and moves the simulated arm from an offset initial pose
to the trajectory start with a smooth reset motion. The red history and formal
metrics start only when the trajectory clock begins at zero.

It rebuilds automatically when the local install is missing or has a stale
prefix. After changing C++ code, force an incremental rebuild with:

```bash
WINDYLAB_SMC_REBUILD=1 \
  src/arm-platform/scripts/run_task_space_smc_whole.sh \
  mode:=simulation duration_sec:=0.0 hardware_enabled:=false
```

The equivalent manual sequence is:

```bash
cd /home/windylab/code/windylab_ws
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash
colcon build \
  --build-base /home/windylab/code/windylab_ws/build \
  --install-base /home/windylab/code/windylab_ws/install \
  --symlink-install --cmake-clean-cache \
  --packages-select dummy_interface serial dummy_description manipulator \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source /home/windylab/code/windylab_ws/install/setup.bash
source src/arm-platform/demo/setup_fastdds_udp_only.sh
ros2 pkg prefix manipulator
ros2 launch manipulator task_space_smc_whole.launch.py \
  mode:=simulation duration_sec:=0.0 hardware_enabled:=false
```

`ros2 pkg prefix manipulator` must print
`/home/windylab/code/windylab_ws/install/manipulator`.

For regression comparisons only, `mode:=internal_simulation` selects the old
Python-internal plant and does not start `student_arm_node`. `mode:=simulation`
is the primary RViz mode and exercises the shared command/feedback chain.

In RViz, the fixed frame is `world`. The green line is the reference path,
the red line is the actual path, and the green/red spheres are the current
target and end-effector. The base moves through `world -> base_link` TF while
the arm compensates to track the world-frame path.

For a short smoke test, pass a finite duration:

```bash
ros2 launch manipulator task_space_smc_whole.launch.py \
  mode:=simulation duration_sec:=5.0 hardware_enabled:=false
```

Every run writes a timestamped full trajectory CSV, JSON summary, and four PNG
reports under:

```text
/home/windylab/code/windylab_ws/src/arm-platform/task_space_smc/data
```

The CSV includes desired/actual/error XYZ, base pose, all joint positions and
velocities, all torque commands, singularity diagnostics, and enable state.
Live numeric topics are `/task_space_smc/tracking` and
`/task_space_smc/joint_torque`.

## Automatic Gain Sweep

Run a quick 20-second candidate sweep:

```bash
cd /home/windylab/code/windylab_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run manipulator tune_whole_task_smc.py
```

For final acceptance, evaluate candidates over the complete whole period:

```bash
ros2 run manipulator tune_whole_task_smc.py --full-period --max-candidates 12
```

Results are written to `task_space_smc/data/autotune/<timestamp>/`. The tuner
never overwrites the production YAML; it emits `smc_params_whole_best.yaml`
for review.

## Tuning Order

1. Set `eta` to zero and validate model dimensions, Jacobian blocks and RNEA.
2. Use small `position.lambda` and `orientation.lambda`.
3. Increase `k` until the sliding surfaces converge smoothly.
4. Increase `eta` only after disturbances or model errors are characterized.
5. Adjust `phi` as the tanh smoothing width.
6. Enable nullspace gains after task tracking is stable.
7. Finally test near singularity and torque/current saturation.

Parameter meanings:

- `Lambda`: convergence speed along the sliding manifold.
- `K`: speed at which the state returns to the sliding manifold.
- `eta`: robust matched-disturbance term.
- `phi`: tanh smoothing transition width.

In the ideal model the smoothed SMC law can still converge to zero.  Under disturbance, saturation, noise or delay, the practical guarantee is usually uniform ultimate boundedness.
