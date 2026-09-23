# Operator-requested no-contact retry — wrist 2.2 profile

**Final: NOT YET VERIFIED.** Two no-contact handovers passed; the third aborted before its first guidance command. Manual-push profile was never enabled in this retry. No speed HARD_ABORT occurred here.

The operator explicitly requested one retry after the first startup fault. Only startup scheduling changed: driver + robot_state_publisher first, RViz and ROS recorder after HOLD stabilized; RX/TX strace retained. The first startup failure remains unresolved and is not claimed fixed by this run.

## Handover results

| Trial | J2 peak speed rad/s | J2 span rad | J2 max command step nominal Nm | 5 s quiet |
|---|---:|---:|---:|---|
| 1 | 0.004885 | 0.000000 | 0.00334041 | PASS |
| 2 | 0.004885 | 0.000000 | 0.00001396 | PASS |
| 3 | — | — | — | NOT STARTED: timestamp gate |

J1–J3 had zero encoder position span in both completed handovers. First handover J5 shifted 0.004987 rad (0.286°); subsequent GUIDE quiet remained within 0.000768 rad on all wrists. Gravity/gains were unchanged.

## Third attempt failure

Python reported `GUIDANCE STOPPED: old/future JointState stamp; restart required` during startup verification, before `ACTIVE HANDOVER_START` and before any guidance command. The prepare request was already in flight; the driver acknowledged `HANDOVER_START` 1.36 ms later, then its 0.5 s HANDOVER command deadline expired and latched fault. This is a startup feedback-age/prepare sequencing failure, not a speed limit event. The log does not contain the offending age, so old versus future cannot be resolved. Node startup queueing is a hypothesis, not a proven root cause. No freshness check or timeout was loosened.

Current driver remains in fault HOLD with **all six previous bounded FF values preserved**:
`[0.590439990, -0.656886957, 1.120604950, -0.004716905, 0.131101087, 0.000176928]` nominal Nm.
Kp `[30,30,30,5,5,5]`, Kd `[1,1,1,0.1,0.1,0.1]`. Last 3 s check: J1–J3 position span zero, wrist span ≤0.000768 rad. Guidance process exited; hardware driver, TF and RViz remain running. No further restart, no fault clearing, no manual push.

The prior successful six-axis acceptance applies to its prior profile/operating conditions; it does not certify this revised profile. A startup reliability investigation is needed before another active attempt.

## Evidence

- `analysis.json`, `final_hold.json`, `events.jsonl`, `hardware_launch.txt`, `trial{1,2,3}.log`.
- [Overview](overview.png), [J2 trial 1](trial1_j2_handover.png), [J2 trial 2](trial2_j2_handover.png), [fault ±2 s](fault1_window.png), `fault_window.json`.
- 9,994 wire command batches in the analysis interval passed checksum, six-axis and zero ghost-slot checks. `serial_tx.strace` also includes serial reads for this retry.
