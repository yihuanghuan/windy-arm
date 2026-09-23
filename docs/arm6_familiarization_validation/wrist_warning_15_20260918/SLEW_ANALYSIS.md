# Gravity slew lag: cause and retest path

## Confirmed code-level cause

`Guidance._command` computes g(q), then limits FF changes to `last_tau ± torque_rate_limits*dt`. Rates are `[3,3,3,.5,.5,.5]` nominal Nm/s. At 100 Hz these allow `[.03,.03,.03,.005,.005,.005]` nominal Nm per cycle. The next check faults whenever `release > 0` and ANY joint's `abs(tau-g(q)) > .02` nominal Nm. `release` stays 1 forever after Kp withdrawal, so this check applies throughout GUIDE/MANUAL_PUSH, although its error message says stiffness release. There is no duration/debounce for this residual.

Consequently a manual movement whose gravity demand changes faster than the configured slew can fail even while its velocity is below all speed abort limits. The FF warning threshold change has no effect on this condition. The residual is a difference between two software torque values, NOT measured current disagreement or proof of a wrong gravity model.

## Offline reproduction

Used the operator's installed URDF and saved runtime parameters without changing model, offset, gravity scale or any protection. Starting pose from the earlier live review:
`[.679980278,.111963272,-.900091171,-1.174427986,.192380905,-.648495674]`.
Initialize a known gravity-supported HOLD, run the six-second handover on stationary synthetic feedback, enable MANUAL_PUSH, then integrate constant dq at 10 ms steps.

Representative results from `slew_reproduction.json`:

- J1 dq=-1.0 rad/s: residual on J1 exceeds .02 after 60 ms; residual=.0208839153 nominal Nm.
- J1 dq=+1.7 rad/s: residual=.0253814471 on first 10 ms step, below the manual 1.8 sustained / 2.0 instantaneous speed limits.
- All joints dq=-.75 rad/s: J1 residual=.020681071 after 100 ms, below all manual warning/abort limits.

These are illustrative counterexamples, NOT replay of the actual fault. The actual event lacks pre-fault continuous q/dq/gravity/limited-FF/dt data, so its failing axis, residual magnitude, and timing cannot be established. A later J1 speed fault mask does not establish the first cause.

## Recommended correction direction, not implemented here

Keep strict support matching for handover/Kp withdrawal. Explicitly distinguish steady manual guidance and define its FF slew/residual policy against expected g(q) variation and torque budgets. Add per-axis residual, dt, active phase and slew saturation diagnostics before changing thresholds. Do not just delete the residual guard or increase speed caps. Startup JointState/prepare sequencing is a separate unresolved issue.

## Retest and current HOLD

This inspection read live diagnostics only. Driver PID 58752 still runs, guidance is stopped, fault remains latched. Latest q1=.5369272232, actual HOLD target q_ref1=.4537649155; absolute error=.0831623077 rad (~4.765°). The operator warm-capture tool requires max |q_ref-q|≤.03 rad, so this current HOLD does not qualify. This is a HOLD snapshot validation failure, NOT evidence of a J1 angle-limit violation. Actual q was stationary over the 2.2 s read window. See `retest_hold_check.json`.

Healthy stop/resume: keep driver; no cold initialization needed.
Fault with valid fresh stationary actual HOLD snapshot: documented warm restart is possible; fresh file expires in 120 seconds and is invalid after power loss.
Current situation: do not loosen snapshot tolerance or replace q_ref with q. Use external support and the documented supported-initialization procedure (section 7) if re-establishing HOLD: support first, stop old driver, readonly capture, then supported startup within 15 s. A power cycle is not inherently required by that procedure. After driver restart, the three no-touch gates are reset.

Re-establishing HOLD clears no underlying slew-policy issue. Fix and offline-validate the protection behavior before repeating the same active manual test. No model, control gains, protection logic, live parameters, latch or motor command changed in this analysis.
