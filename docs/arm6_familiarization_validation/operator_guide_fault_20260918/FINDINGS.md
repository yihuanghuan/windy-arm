# Operator-run GUIDE speed abort — 2026-09-18

The newest run was ordinary GUIDE, **not MANUAL_PUSH**. Both driver and Python identify J4 `type=instantaneous`, dq=-1.501831 rad/s, at epoch 1789722449.1119 (17:07:29 local). Driver reported approximately 9.99 ms sustained time; the instantaneous rule, not duration, caused this abort.

Timeline: driver starts 17:06:05; prepare at 17:07:05; ACTIVE GUIDE at 17:07:11; NO_TOUCH_PASS completed=1 at 17:07:16. Only one handover/quiet trial is logged. No manual-push service acknowledgment or ACTIVE MANUAL_PUSH exists. The capability flag in the YAML is true, but this does not activate the profile.

Configured ordinary GUIDE: warning 0.5, sustained 1.0 / 80 ms, instant 1.2 rad/s. Configured MANUAL_PUSH: warning 1.0, sustained 1.8 / 120 ms, instant J1–J3 2.0 / J4–J6 2.2 rad/s. The reported J4 sample exceeds ordinary GUIDE's instantaneous limit but is below MANUAL_PUSH's sustained and instantaneous limits.

Offline replay using the exact saved parameter file and SpeedSafety: J4 -1.501831 rad/s in GUIDE immediately faults; the same sample for one second in MANUAL_PUSH produces warning only, no fault, no sustained timer accumulation. This establishes protection semantics only, not a real-arm stability claim.

No threshold changed and no live command/restart was performed during this investigation. Doubling the existing manual cap would make J1–J3 4.0 rad/s, above their configured motor max_velocity 3.0 rad/s; current launch validation rejects this. Wrist caps 4.4 rad/s would approach their configured 5.0 rad/s capability. Motor maxima are configuration values, not certified safe manual motion limits.

The launch log records user SIGINT at 17:07:53.932 and driver clean exit at 17:07:54.098. Process inspection confirms no active student_arm_node or guidance; only RViz remains. No live q/dq/FF can be read from ROS. Do not describe this as current software-maintained fault HOLD. No motor disable command was sent by this investigation; firmware's retained physical state is not known.

This operator run has text logs but no continuous q/effort/applied-command capture. Exact q, Kp/Kd/FF and physical fallback behavior at the fault cannot be reconstructed from these logs alone. A previous HOLD seed is not a fresh readback after this run and must not be reused for restart.

Next action: restore a verified HOLD through the documented supported initialization if a fresh actual command cannot be read, then complete the required no-touch gates and wait for ACTIVE MANUAL_PUSH before pushing. Retain transition protections and all other safeguards.
