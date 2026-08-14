# Arm calibration handoff package

This package contains the current candidate compensation parameters, the relevant calibration/control code, and Vicon diagnostic records.

## Parameter status

- `parameters/arm6_calibration_candidate.yaml` is the current candidate compensation file. It contains the accepted J1/J2/J3/J5 values and preserves the rejected/uncertain joints according to the quality gates.
- `parameters/vicon_extrinsics_yup_baseline.yaml` is a copy of the old formal Y-up Vicon configuration. It is a baseline only; it was not overwritten during the recent tests.
- `parameters/vicon_joint_se3_diagnostic_20260814.yaml` is diagnostic-only. Do not load it as a runtime YAML: its position RMS is 11.61 mm and the fit is ill-conditioned.

## Vicon coordinate requirement

The calibration data and baseline external parameters use `Y-up` Vicon axes. The VRPN server currently used is `192.168.200.73:3883`.

Do not mix `Z-up` measurements with the Y-up parameters. A world-axis change cancels from:

```text
T_meas = inverse(T_W_arm_base) * T_W_arm_ee
```

Before any external calibration, compare this relative transform with:

```text
T_pred = T_arm_base_to_base_link * FK(q_encoder + candidate_zero) * T_link6_to_arm_ee
```

Only re-fit external parameters if the relative transform is still inconsistent after the Vicon axis convention has been made identical.

## Code layout

- `code/task_space_smc/hw/`: Phase1-5, candidate management, J5 LESO calibration, Vicon calibration/bridge, and control runner.
- `code/task_space_smc/core/`: model loading, zero/gravity/friction use, state estimator, LESO, Smith predictor, and SMC core.
- `launch/`: real-arm, Vicon, hold, and whole-arm launch entry points.
- `data/`: source diagnostic JSON used for the Vicon joint SE(3) analysis.

The repository remains the source of truth for installation and package dependencies. These files are a handoff snapshot, not a replacement for the repository checkout.
