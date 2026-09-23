# ARM6 familiarization branch handoff

Continuation branch: **`arm6-familiarization`**, created from local `lagrange-smc`
at `b496a4f` on 2026-09-23. The snapshot preserves the accumulated working-tree
changes, including pre-existing real-arm audit work. Future work should start on
this branch; it does not change the GitHub repository's default branch.

## Scope and invariants

- Real A-L1 gamma6: joint1–joint6 only, `arm6.urdf`, base_link → link6.
- Existing student_arm_node owns the serial port; guidance uses ROS direct effort.
- Gravity is computed from the calibrated model with explicit zero offsets once;
  do not reapply historical first_moment_ratio or add a ghost joint7.
- Scope remains manual gravity guidance and real-feedback RViz. No D435i, SLAM,
  SMC, base-disturbance or world-frame endpoint stabilization is introduced.
- Preserve freshness, heartbeat, torque/joint limits, six-axis checks, fault latch
  and bounded-support fallback. Do not automatically retry a physical fault.

## Operator and evidence entry points

- [Terminal operator guide](ARM6_ZERO_GRAVITY_OPERATOR.md).
- [Hardware understanding and investigation](ARM6_FAMILIARIZATION.md).
- [Earlier six-axis/manual acceptance](arm6_familiarization_validation/restart_after_power_20260918/ACCEPTANCE_REPORT.md).
- [Latest warning-change review](arm6_familiarization_validation/wrist_warning_15_20260918/REPORT.md).
- [Gravity slew analysis](arm6_familiarization_validation/wrist_warning_15_20260918/SLEW_ANALYSIS.md).
- [Raw-log restoration](VALIDATION_LOG_ARCHIVES.md).

## Known incomplete work — do not treat this snapshot as a fresh PASS

1. Startup occasionally rejects an old/future JointState during driver preparation;
   the in-flight prepare request can then produce a handover timeout. Reproduced on
   hardware and in the pseudo-serial test. Freshness guards have not been loosened.
2. The `release > 0` gravity-slew residual check remains active in GUIDE/MANUAL_PUSH.
   Manual motion can exceed its 0.02 nominal Nm error threshold below the speed
   abort limit. Offline counterexamples and missing incident telemetry are recorded.
3. Source configuration raises wrist **warning only** to 1.5 rad/s; sustained abort
   remains 1.8 rad/s for 120 ms and wrist instantaneous abort remains 2.2 rad/s.
   The latest recorded deployed session still used warning 1.0. The proposed warning
   change was not deployed or physically accepted as part of the recorded review.
4. Latest recorded tests for that proposal: Python 67 passed; four C++ functional
   suites passed; pseudo-serial 12 passed, one startup timestamp failure. These are
   historical results, not a new complete test run for this branch-creation task.

The conversation subsequently included further user-led operation. Process IDs,
device ownership, actual firmware command, support and power state must be freshly
inspected before future hardware work. Do not reuse historical PIDs or seed files.
Branch creation/commit/push performs no hardware restart, fault reset or actuation.
