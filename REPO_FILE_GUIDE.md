# Repository File Guide

This guide records the files currently used for the 6-DOF arm calibration,
fixed-base SMC circle simulation, and hardware validation. It also identifies
debug-only files that are candidates for archiving. No files are deleted by
this guide.

## Current Main Workflow

### Fixed-base 6-DOF circle test

| File | Role |
| --- | --- |
| `src/arm-platform/launch/task_space_smc_whole6_sim.launch.py` | Starts the 6-DOF torque simulation, `student_arm_node`, robot state publisher, SMC runner, and RViz. |
| `src/arm-platform/launch/task_space_smc_whole6.launch.py` | Main 6-DOF hardware/analytic-base launch. |
| `src/arm-platform/task_space_smc/config/smc_params_hwtest.yaml` | Current `hwtest` circle controller parameters. Current position values are `lambda=[14,14,14]`, `k=[8,8,8]`, `eta=[0.08,0.08,0.08]`, `phi=[0.05,0.05,0.05]`. |
| `src/arm-platform/task_space_smc/hw/whole_task_smc_ros_runner.py` | Main task-space SMC runner. It computes task errors, SMC acceleration, Jacobian mapping, RNEA torque, friction compensation, logging, and safety checks. |
| `src/arm-platform/demo/moving_base_trajectory_config.py` | Defines `hwtest`: center, radius, speed, orientation, and trajectory period. |
| `src/arm-platform/task_space_smc/core/trajectory_reference.py` | Converts trajectory samples into position, velocity, acceleration, and orientation references. |
| `src/arm-platform/task_space_smc/core/task_space_smc.py` | Task-space SMC law. It consumes position/orientation errors and produces task-space commands. |
| `src/arm-platform/task_space_smc/core/model.py` | Pinocchio model wrapper for FK, Jacobian, gravity, RNEA, and calibrated joint coordinates. |
| `src/arm-platform/task_space_smc/core/pose_error.py` | Position and SO(3) orientation error calculation. |
| `src/arm-platform/task_space_smc/core/redundancy_resolution.py` | Damped Jacobian/redundancy resolution and feasibility scaling. |
| `src/arm-platform/task_space_smc/core/actuator_adapter.py` | Converts computed joint torque into the motor command message and applies command-side limits. |
| `src/arm-platform/task_space_smc/core/friction_feedforward.py` | Directional static/kinetic friction feedforward, including `F+` and `F-`. |
| `src/arm-platform/config/arm6_calibration_candidate.yaml` | Current real-arm zero, gravity ratio, friction, and armature candidate parameters. |

### Standard calibration flow

| Phase | File | Role |
| --- | --- | --- |
| Phase 1 | `src/arm-platform/task_space_smc/hw/fk_gravity_verify.py` | FK, joint zero, gravity direction, and model/encoder consistency check. |
| Phase 2 | `src/arm-platform/task_space_smc/hw/phase2_torque_probe.py` | Torque path, static friction, and motor response probe. |
| Phase 3 | `src/arm-platform/task_space_smc/hw/phase3_gravity_calibration.py` | Original zero-offset and gravity-ratio calibration. |
| Phase 4 | `src/arm-platform/task_space_smc/hw/phase4_gravity_compensation.py` | Real-arm gravity-compensation acceptance from L0 through L4. |
| Phase 5 | `src/arm-platform/task_space_smc/hw/phase5_inertia_id.py` | Original inertia and dynamic parameter identification. |
| Positioning | `src/arm-platform/task_space_smc/hw/full_prepose.py` | Slow position-mode prepose before a hardware experiment. |
| Read-only check | `src/arm-platform/task_space_smc/hw/pose_check_readonly.py` | Serial/joint-state check that must not send commands. |

### Vicon and moving-base workflow

| File | Role |
| --- | --- |
| `src/arm-platform/task_space_smc/hw/vicon_floating_base_bridge.py` | VRPN-to-ROS bridge for Vicon base/EE data, including timestamp and stale-data handling. |
| `src/arm-platform/launch/vicon_moving_base_candidate_test.launch.py` | Candidate moving-base validation launch. |
| `src/arm-platform/task_space_smc/config/smc_params_vicon_candidate_test.yaml` | Configuration for the candidate moving-base validation. |
| `src/arm-platform/task_space_smc/config/vicon_extrinsics.yaml` | Current Vicon base and link6-to-EE geometric parameters. |
| `src/arm-platform/task_space_smc/hw/vicon_base_extrinsic_calibrate.py` | Multi-pose Vicon base-extrinsic acquisition and fitting. |
| `src/arm-platform/task_space_smc/core/base_state.py` | Base-state representation and validation. |
| `src/arm-platform/task_space_smc/core/ee_estimator.py` | FK/Vicon EE position fusion and FK-only orientation policy. |

### Observer and delay-related code

| File | Role |
| --- | --- |
| `src/arm-platform/task_space_smc/core/leso.py` | Joint LESO implementation. The J5 experiment used it as an observation/diagnostic channel, not direct closed-loop compensation. |
| `src/arm-platform/task_space_smc/core/state_estimator.py` | Joint state estimation interface. |
| `src/arm-platform/task_space_smc/core/smith_predictor.py` | Smith-prediction utilities for delay experiments; not required by the fixed-base `hwtest` path. |
| `src/arm-platform/task_space_smc/core/loop_delay.py` | Delay and command/feedback timing utilities. |
| `src/arm-platform/task_space_smc/core/velocity_filter.py` | Joint velocity filtering. |

### Hardware interface files

| File | Role |
| --- | --- |
| `src/arm-platform/launch/student_arm6.launch.py` | Six-axis student-arm driver launch and motor/position-mode defaults. |
| `src/arm-platform/src/ros_nodes/student_arm_node.cc` | ROS driver node, direct-effort path, position path, watchdog fallback, and joint-state publication. |
| `src/arm-platform/include/manipulator/ros_nodes/student_arm_node.h` | Driver node declarations and safety helpers. |
| `src/arm-platform/src/robotics/motor/dm_motor.cc` | DM motor protocol implementation. |
| `src/arm-platform/config/motor_config.yaml` | Motor model, torque/velocity limits, and MIT default gains. |

## Parameters That Must Stay Together

The following files form the current real-arm candidate chain:

```text
arm6_calibration_candidate.yaml
    -> model.py
    -> FK/Jacobian/b6/RNEA/friction

vicon_extrinsics.yaml
    -> vicon_floating_base_bridge.py / EE estimator / moving-base launch

smc_params_hwtest.yaml
    -> whole_task_smc_ros_runner.py
    -> task-space SMC and hardware circle test
```

Do not replace the candidate calibration with the original `arm6_calibration.yaml`
when reproducing the current hardware results.

## Debug-only and Archive Candidates

These files were created for specific investigations. They are not required by
the standard Phase 1-5 workflow or the current fixed-base `hwtest` launch.
They can be moved to an archive after the associated raw data and conclusions
have been retained.

| Files | Reason for archiving |
| --- | --- |
| `hw/phase3_multi_pose_zero_calibration.py` | Offline/multi-pose zero experiment; superseded by the original Phase 3 for the standard workflow. |
| `hw/phase3_zero_observability_diagnose.py` | J3/J4 observability and profile-cost diagnostic. |
| `hw/phase3_dynamic_zero_calibration.py` | Experimental dynamic zero solver; not the original Phase 3 flow. |
| `hw/phase3_dynamic_zero_evaluate.py` | Evaluation companion for the experimental dynamic solver. |
| `hw/phase3_dynamic_zero_fixed45_refit.py` | Fixed-J4/J5 refit experiment. |
| `hw/phase3_dynamic_zero_friction_calibration.py` | Dynamic zero plus friction experiment. |
| `hw/phase3_dynamic_even_zero_fit.py` | Even/odd trajectory residual experiment. |
| `hw/phase3_dynamic_j45_coupling_zero_calibration.py` | J4/J5 coupling experiment. |
| `hw/phase3_dynamic_j5_coupling_zero_scan.py` | J5-only coupling and zero scan. |
| `hw/phase3_j4_multi_posture_gravity.py` | J4-specific multi-posture gravity experiment. |
| `hw/phase3_j5_leso_observer_tune.py` | J5 LESO tuning/diagnostic experiment. |
| `hw/phase4_zero_ablation.py` | Candidate zero-parameter ablation experiment. |
| `hw/phase5_constant_velocity_residual.py` | Constant-velocity residual experiment. |
| `hw/phase5_constrained_friction_optimizer.py` | Constrained friction optimizer experiment. |
| `hw/phase5_directional_friction_dynamic.py` | Directional friction dynamic experiment. |
| `hw/phase_candidate_repeat_check.py` | Candidate repeatability experiment helper. |
| `hw/phase_candidate_yaml.py` | Candidate YAML generation helper; not a runtime controller. |
| `hw/phase_j1235_full_repeat.py` | J1/J2/J3/J5 repeatability experiment. |
| `hw/pd_gravity_prepose.py` | Experimental torque/PD prepose. Use `full_prepose.py` for the standard prepose path. |
| `hw/manual_guidance.py` | Manual gravity/damping guidance tool; not part of the standard automated flow. |
| `hw/vicon_fk_base_calibrate.py` | Earlier Vicon/FK calibration implementation. Use `vicon_base_extrinsic_calibrate.py` for the current flow. |
| `hw/vicon_link_check.py` | Earlier Vicon link diagnostic. Keep only if the old test suite is still needed. |
| `hw/vicon_timing_diagnostic.py` | One-off three-layer timing diagnosis. |
| `config/smc_params_hold.yaml` and `launch/vicon_hold_demo.launch.py` | Older Vicon hold workflow, separate from fixed-base `hwtest`; retain only for legacy moving-base hold tests. |
| `config/smc_params_whole6_hw.yaml` | Older six-axis hardware configuration retained for compatibility, not used by the current `hwtest` command. |
| `config/vicon_extrinsics.yaml.bak_20260825_094003` | Local timestamped backup only; not a runtime configuration and not needed in Git. |

The last backup file is currently left on the local machine and is not part of
the committed files. Delete it only after confirming the current Vicon YAML and
the copy under `share/real_robot_parameters_20260825` are both preserved.

## Data and Build Artifacts

The following are intentionally excluded from normal commits:

```text
build/
install/
log/
src/arm-platform/task_space_smc/hw/data/
Python __pycache__ files
```

Keep raw calibration and experiment data separately when needed; do not treat
large generated logs as source code.

## Current Reproduction Entry Points

Fixed-base simulation:

```bash
ros2 launch manipulator task_space_smc_whole6_sim.launch.py \
  trajectory_shape:=hwtest duration_sec:=0.0 rviz:=true
```

Hardware `hwtest` uses:

```text
task_space_smc_whole6.launch.py
  -> whole_task_smc_ros_runner.py
  -> smc_params_hwtest.yaml
  -> arm6_calibration_candidate.yaml
```

The hardware command must be run only after the robot has been positioned in a
safe starting posture and the serial/watchdog checks have passed.
