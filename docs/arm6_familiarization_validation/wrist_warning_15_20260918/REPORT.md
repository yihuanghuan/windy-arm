# Wrist warning threshold review — 2026-09-18

## Real evidence and limits

This time MANUAL_PUSH was actually enabled: driver accepted the explicit operator request at epoch 1789723439.280945 (17:23:59); live diagnostics showed MANUAL_PUSH, completed handovers=4, all six Kp=0. Driver text warning logs still label manual subphase GUIDE; structured status and Python logs correctly identify MANUAL_PUSH.

Full Python log, flushed when the already-faulted guidance process was exited, contains J4 22 warning entries (maximum warning-entry magnitude 1.282051 rad/s), J5 11 entries (max 1.306473), no J6 warning entries. These are threshold-entry events, not continuous q/dq recording or guaranteed peak speeds. The earlier movement cannot be reconstructed fully. RS05/MIT is the repository configuration; no physical cause for perceived step-like motion is established.

## Change (source only, not loaded on the real driver)

MANUAL_PUSH warning array: `[1,1,1,1,1,1]` → `[1,1,1,1.5,1.5,1.5]` rad/s. Sustained threshold remains 1.8 rad/s for 120 ms on all axes. Instantaneous cap remains `[2,2,2,2.2,2.2,2.2]`. Ordinary GUIDE and transition thresholds, all torque limits, model, offsets, gains and fault behavior unchanged. Warning does not modify the motor command or create a fault.

Python tests updated for warning boundaries/per-axis separation and retained sustained protection; C++ test updated equivalently. No production controller logic changed.

## Verification

- Python: 67 passed.
- C++: four functional suites passed.
- Separate test-overlay build passed.
- Pseudo-serial: **12 passed, 1 failed**. On the third warm start the node failed `old/future JointState stamp`, then the driver latched its handover deadline. This reproduces the previously observed startup sequencing problem before MANUAL_PUSH is enabled. It must not be hidden by rerunning until green or by loosening freshness checks.
- No real-driver restart, no fault reset, no installation into the live workspace install, no update of the operator parameter file. The test overlay under /tmp contains the proposed config; the operator's workspace install and live process retain the old 1.0 wrist warning. Do not mix these overlays for real operation.

## New real fault observed while reviewing

During the review the running guidance entered fault. Python's first recorded stop reason is:
`GUIDANCE STOPPED: gravity slew lag during stiffness release; restart required`
at epoch 1789723675.183126 (17:27:55), after MANUAL_PUSH had been active for several minutes.

The policy computes slew-limited FF, then checks `release > 0 and max(abs(tau - gravity)) > 0.02`. Since release stays 1 in GUIDE/MANUAL_PUSH, this guard remains active during dragging despite the message mentioning stiffness release. It can stop motion even below speed limits. This guard/model/slew was not modified. No pre-fault continuous state/torque capture exists for this event, so the axis and peak residual causing this check cannot be determined.

A subsequent live driver speed diagnostic shows J1 fault_axes=true. This is distinct from the initial Python stop reason; the exact order/type/dq of any secondary driver speed event is unavailable from its still-buffered partial log. Do not label the initial event a wrist overspeed from this later state.

Latest measured fallback: Kp `[30,30,30,5,5,5]`, Kd `[1,1,1,.1,.1,.1]`; FF `[0, .3102361153, .9578595348, 0, 0, -.0001179638]` nominal Nm. J2/J3 support remains. No claim that *only* J1 FF was cleared: J4/J5 FF are also zero in the observed fallback. `live_review.json` records five seconds after fault; `fault_hold.json` is a later snapshot.

The faulted guidance PID 63914 was sent SIGINT solely to stop the inactive node and flush its log; hardware driver PID 58752 remains running with fault latched. No further actuation trial was started. Warning adjustment is prepared but **NOT YET VERIFIED / NOT DEPLOYED**; startup reliability and the slew-guard stop remain separate blockers.

Further code-level analysis, offline counterexamples and current restart eligibility: [SLEW_ANALYSIS.md](SLEW_ANALYSIS.md).
