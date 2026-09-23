# KNOWN-GOOD VS CURRENT — new arm, 2026-09-18

No control code or parameters changed for this acceptance session.

## Evidence and limits

- `docs/real_arm_audit_20260917/j6_software/historical_REFERENCE.md`, §13 explicitly records real six-joint gravity compensation: **kp = 0 下不下坠**. This statement is already present in git history beginning with `e5531d9` (2026-08-04). It supports prior success, but does not identify the exact executable/configuration hashes of that run.
- At `e5531d9`, Phase4 and GravityModel refer to `arm6.urdf`, but that URDF/calibration file is absent from the tracked tree. At `ef84a9d`, `arm6.urdf` is tracked and explicitly has calibrated mass/inertia baked in. Its model loader explicitly says not to multiply the ratio again. Phase4 nevertheless still multiplies the ratio. Consequently the historical successful runtime cannot be reconstructed unambiguously from these commits; this inconsistency is not evidence that a second ratio is correct.
- `.Trash/zzq_v2_archive_20260831/src/arm-platform/task_space_smc/hw/manual_guidance.py` (also introduced in `f95039a`) is a later design reference, not a preserved successful run. It accepts a required calibration YAML and multiplies its ratio; its damping appears both in FF and MIT Kd. It is not imported by the active feature.
- The August 14 handoff describes candidate calibration and Phase4 acceptance, but no complete successful L4 time series was located. Candidate zero offsets/ratios are hardware-specific and differ from the August 2 values.
- `docs/real_arm_audit_20260917/gravity_tuning/GRAVITY_TUNING_RESULT.md` concludes INCONCLUSIVE/HARD_ABORT, not successful manual guidance. Those old-arm trials do not certify this replacement arm.
- External reference package: `~/westlake/windylab-arm-real/windylab-arm/windylab_ws/src/arm-platform`. Its gravity drag architecture is useful, but includes gripper/channel 7 and a different model/configuration. No seventh motor is introduced here.

| Item | Historical evidence/reference | Current acceptance configuration |
|---|---|---|
| Mechanical model | Six-joint Phase4: arm6.urdf/link6; exact successful file not preserved with run | Calibrated arm6.urdf, a_l1/gamma6, base_link→link6, six motors |
| Zero offsets | Aug 2 radians: [0.039576,-0.297745,-0.020218,-0.192988,-0.111667,0]; later candidate differs | Explicit [0,0,0,0,0,0], no automatic historical transplant |
| Gravity | Old Phase4/archive multiply ratio; contemporaneous model documentation forbids that for calibrated URDF | g_arm6(q_encoder + explicit_offset), ratio not reapplied |
| MotorControl | MIT position/velocity/current(FF)/p/d; 0x12 documented as nominal torque, historical measured alpha=1.033±0.024 | Same existing driver; nominal Nm, no alpha multiplier; first 3 signs +, last 3 signs − applied once at motor boundary |
| Kp | Phase4 reduces [30,30,30,5,5,5] through levels to zero; archive starts p=0 | Same hold Kp, smoothly withdrawn to zero |
| Kd | Phase4 L4 [.2,.2,.2,.02,.02,.02]; archive .20 MIT plus .20*dq subtracted from FF | [.2,.2,.2,.05,.05,.05], solely MIT damping |
| FF | Phase4 ratio*g; archive ratio*g − D*dq; no friction bias | Calibrated gravity only; no friction boost or duplicate damping |
| Rate | 100 Hz | 100 Hz |
| Torque limit | Driver rated [9,9,9,1.6,1.6,1.6] Nm; external drag uses lower [3,3,3,1,1,1] | Rated limits; rejects gravity+damping budget violations; slew [3,3,3,.5,.5,.5] Nm/s |
| Speed limit | Phase4 .5 rad/s; archive 1.5 rad/s | .5 rad/s unchanged; takeover requires ≤.15 rad/s |
| Transition | Phase4 stepped gains, 3s settle + 4s sample per level; archive no ramp; external reference 3s gravity ramp | Initial measured-pose HOLD command, 3s gravity ramp, 3s smooth Kp withdrawal, GUIDE; no separate HOLD dwell |
| Shutdown/fault | Phase4 falls back to previous level; archive stops publishing; reference configurable hold | Fresh feedback/NaN/heartbeat/six-axis gates, latched faults, PD hold; overspeed-axis FF removed, valid other axes retain last transmitted bounded FF; fully invalid feedback clears all FF. Serial close is not motor disable. |
| Serial | Same ProtocolV1 family; historical scripts mention ttyUSB0 but exact success device not archived | Verified CP2102 by-id → ttyUSB0, 921600 baud; sole serial owner student_arm_node |

The offset choice and wrist damping differ materially from historical candidates; these differences are disclosed, not silently tuned. Current offsets follow the replacement-arm assembly/readonly verification, while old fitted offsets belong to another setup. Current wrist damping is modest and used only once. All existing safety enhancements are retained. No evidence justifies reapplying the historical first-moment ratio.
