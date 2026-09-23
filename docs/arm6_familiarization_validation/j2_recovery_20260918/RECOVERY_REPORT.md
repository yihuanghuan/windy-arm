# J2 supported manual recovery — 2026-09-18

**J2 position recovery: VERIFIED. Full manual-guidance acceptance remains NOT YET VERIFIED.**

The operator reported an earlier whole-arm power disconnection, which is a
plausible explanation for the later all-axis FF removal during latched fallback.
The precise earlier power-off timestamp was not logged, so this is not a proved
time correlation. No fault/limit bypass or limit increase was used for recovery.

After the operator explicitly confirmed the arm was supported, the active
driver was shut down cleanly, its serial owner was confirmed gone, and the
existing familiarization launch was restarted in `mode:=readonly`. `strace`
monitored write/writev/pwrite64 on the verified serial device; zero motor writes
were observed throughout this recovery. No second serial connection was opened.

The operator disconnected whole-arm power with the arm supported and subsequently
reported power restored and recovery complete. Feedback stopped during the power
interruption. Host readonly does not certify firmware-side motor disable or tell
us the motor controller's current internal hold gains/reference after power-up.

Fresh feedback after power restoration, final five-second verification:

| Joint | q (rad) | Clearance inside unchanged safety margin (rad) |
|---|---:|---:|
| J1 | 0.067330 | 3.832670 |
| J2 | 1.259823 | 0.210177 |
| J3 | -1.350233 | 0.119767 |
| J4 | 0.000192 | 3.899425 |
| J5 | -0.000575 | 1.899042 |
| J6 | 0.000575 | 3.899425 |

J2 is 72.18°, approximately 12.04° below the unchanged 1.470 rad safety boundary.
The underlying model upper bound remains 1.570 rad and margin remains 0.100 rad.
All six joints are strictly inside their existing safety margins.

501 JointState messages over 4.99993 seconds: 100.001 Hz, maximum observed receive
interval 10.47 ms. J1–J3 position span was zero at encoder resolution; J4–J6 span
was 0.0003834 rad. All feedback was finite, six-dimensional and named joint1–joint6.

Current running state: readonly driver, robot_state_publisher and RViz. Guidance
has not been restarted. The earlier authoritative HOLD seed predates the power
cycle and must not be reused. A future active takeover needs a newly established
and verified hold; position recovery alone is not bumpless-takeover validation.

Evidence: `recovery_verification.json`, `feedback.jsonl`, `unchanged_limits.json`,
`readonly_launch.log`, `readonly_serial.strace`, operator confirmations.
No gravity/model/gain/URDF/limit/control-code change, commit, push, reset or checkout.
