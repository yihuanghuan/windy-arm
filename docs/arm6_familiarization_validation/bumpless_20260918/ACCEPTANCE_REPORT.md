# ARM6 bumpless handover / explicit manual-push acceptance — 2026-09-18

Follow-up: [supported J2 recovery](../j2_recovery_20260918/RECOVERY_REPORT.md)
subsequently restored J2 to 1.259823 rad without changing limits. The fault-hold
state described below is historical; current operation is readonly, with no
guidance restart after the power cycle.

Branch `lagrange-smc`. Real gamma6, joint1–joint6 only, arm6.urdf/link6.
No gravity/model/URDF/offset changes in this iteration. No ratio rescaling,
GUIDE gain changes, trajectory, SMC, camera integration, commit, push or checkout.

## 1. Root cause of startup discontinuity

The previous controller replaced an already loaded PD hold with `q_ref=q`,
`FF=0`. This erased the position-error contribution before gravity ramp-in.
The previous no-touch abort therefore was not evidence for increasing transition
speed limits. Before this repair's first takeover, J2 was at 1.391050 rad while
the actual held reference was 1.320859 rad, Kp=30, Kd=1, FF=0. At the sampled
dq=-0.004885 rad/s, estimated PD/total support was -2.100857 nominal Nm.
Replacing that reference with measured q would remove approximately 2.1057 Nm
of spring support. MotorState effort was approximately -2.19 nominal Nm.

## 2. Bumpless implementation

The existing driver now publishes its actually applied six-axis MotorControl
and support diagnostics. An explicit prepare service snapshots a fresh, healthy
authoritative HOLD. Guidance's first command must match every q_ref/v_ref/Kp/Kd/FF
entry of that snapshot. A missing/mismatched snapshot is refused, not guessed.

For the three-second gravity ramp, keep old P/D and use
`q_ref = old_q_ref - (FF-old_FF)/old_Kp`. Thus `Kp*(q_ref-q)+FF` remains
constant at the same measured state while support transfers from PD to gravity.
Only after gravity matches does a separate three-second smooth withdrawal take
Kp to zero and Kd to the unchanged configured GUIDE damping. Healthy GUIDE→HOLD
also preserves estimated total command; it does not clear any fault latch.

Each phase logs q/dq/q_ref/Kp/Kd/FF/gravity/estimated PD/total and the change
between successive commands evaluated at the **same current feedback**. This
last metric isolates command changes from encoder/velocity quantization.
These are MIT-model estimates, not directly measured joint total torque.

The driver upgrade required one process restart. Its initial HOLD seed came
from fresh, checksum-verified actual wire commands on the same serial device;
six-axis signs were inverted once and the ghost slot was verified zero. The
launch only accepts a seed captured within 120 seconds, and the driver checks
fresh feedback, nearby q, low speed and torque bounds before writing it. The
saved seed is historical evidence, **not reusable for future startups**.
All three subsequent takeovers used the same live driver and its live snapshot.

Files: manual_guidance core/node, Python/C++ speed safety, student node/header,
familiarization YAML/launch, CMake/package dependencies and offline tests.
The launch supports `start_guidance:=false` for repeated warm takeovers.

## 3. Transition protection

HOLD/HANDOVER/RAMP/KP_WITHDRAWAL retain warning 0.5 and immediate abort
0.5 rad/s. Per-joint feedback freshness, heartbeat, NaN, six-axis checks,
FF/estimated torque budgets, limits including 0.1 rad margin, and fault latch
remain enabled. Manual flags cannot change transition thresholds.

## 4. Manual-push profile

Default GUIDE remains warning 0.5, sustained 1.0 for 80 ms, instant 1.2 rad/s.
Explicit manual profile uses six-entry arrays: warning 0.8, sustained 1.5 for
120 ms, instant 2.0 rad/s. Per-axis monotonic timers reset below threshold.
Instant limits remain below configured motor capabilities: 3 rad/s for J1–3,
5 rad/s for J4–6. The parameter alone cannot enable this profile: healthy GUIDE,
current five-second no-touch pass, three driver-counted successful handovers,
and an explicit operator service request are required. The operator confirmed
all three runs were untouched before that request was accepted.

## 5. Offline results

- 29 Python gravity/guidance tests passed.
- 4 C++ suites passed.
- 11 pseudo-serial tests passed, including installed-node three-trial gate,
  first-command equality/rejection, manual timing/instant limits, transition
  rejection, fault masking and no automatic restart.
- Isolated build, Python/launch syntax and launch argument checks passed.

Logs: `python_tests.txt`, `cpp_tests.txt`, `pty_tests.txt`, `launch_args.txt`.
Installed test build: `/tmp/windy-arm6-handover-install`.

## 6. Three no-touch handovers

All three entered GUIDE and passed at least five seconds of no-contact static
observation. The operator explicitly confirmed no contact throughout.

| Trial | J2 max abs(dq), rad/s | J2 q span, rad | J2 max command step, Nm | All-axis max command step, Nm |
|---|---:|---:|---:|---:|
| 1 | 0.004885 | 0 | 0.00048043 | 0.00292161 (J3) |
| 2 | 0.004885 | 0 | 0.00000256 | 0.00007733 |
| 3 | 0.004885 | 0 | 0.00000254 | 0.00007400 |

J2 and J3 had zero observable position change in every handover and in the
first five GUIDE seconds. Across all axes, handover max abs(dq)=0.158730 rad/s
and maximum position span=0.000767 rad, on wrist feedback. First-five-second
GUIDE maximum net position change was 0.000383 rad, with no sustained drift.
All three first commands had zero estimated torque step from the inherited
HOLD command. Exact per-axis data are in `analysis.json`.

At the first J2 takeover, total support stayed approximately -2.100857 Nm
through gravity matching, then moved smoothly to approximately -2.196460 Nm
during withdrawal. This contrasts with the old zero-support first command.

Plots: `trial1_j2_handover.png`, `trial2_j2_handover.png`,
`trial3_j2_handover.png`, `overview.png`. All values are nominal driver units.

## 7. Push/release results

| Joint | Result |
|---|---|
| J1 | PASS: approximately 18.95° travel, peak abs(dq)=0.776556 rad/s; operator: easily pushed, holds on release, no return or abnormality. |
| J2 | Incomplete: operator confirmed pushing in the opposite direction, into the upper safety margin. No successful release acceptance. |
| J3 | Not tested this manual session. |
| J4 | Not tested this manual session. |
| J5 | Not tested this manual session. |
| J6 | Not tested this manual session. |

`j1_push_release.png` and `j2_push_release.png` contain actual feedback, wire
reference/FF/gains, measured effort and estimated support. The latter includes
the fault interval and is not a successful release trial.

## 8. Fault chronology

Primary fault: **J2 upper joint-limit margin**, not manual-push overspeed.
At 1789717032.4186 UTC epoch seconds the driver latched a fault. J2 q=1.473068
rad, dq=+0.610500 rad/s crossed the allowed upper margin `1.57-0.10=1.47 rad`.
The last Python command at q=1.466964, dq=+0.649572 had Kp=0, Kd=0.2,
FF=-2.224472 nominal Nm. Nearest measured J2 effort was -2.263248 nominal Nm.
The Python controller independently reported `joint limit margin reached`.
At the original fault all manual sustained timers were zero; no manual speed
threshold was reached.

Approximately 10 ms **after** the limit fault, the driver had already entered
PD fallback; its transition speed guard then logged J2 dq=+0.620268 >0.5,
`SPEED HARD_ABORT type=transition duration=0`. This secondary log must not be
misreported as the cause of ending MANUAL_PUSH or as failure of its 1.5/2.0
profile. No thresholds were increased and no automatic retry occurred.

J2 reached 1.558518 rad in the recorded five-second fallback window, below the
URDF 1.57 upper bound but still inside the safety-margin region. Post-fault peak
abs(dq)=0.747253 rad/s. Software margin protection is not a mechanical stop.
Full ±2 seconds: `fault_window.json`, `fault1_window.png`.

## 9. Fault fallback

Actual serial/driver evidence: J2 FF→0 after the secondary overspeed mask; J1,
J3–J6 preserved exactly the last bounded support
`[-0.324037, 0.952131, -0.0109284, 0.0328931, 0.000165912]` nominal Nm.
J3 remained at -1.406310 rad. J2 PD reference latched at 1.473068 rad,
Kp=30/Kd=1. The other axes did not all lose gravity support. The driver remains
fault-latched; guidance publishing was stopped. This trial does not establish
absence of J2 post-fault motion: the recorded J2 displacement was 0.08545 rad.

**Later read-only follow-up, after the acceptance recorder stopped:** actual
serial writes changed to all-six FF=0 at epoch 1789717318.01482, about 285.6 s
after the original fault. A later ROS snapshot showed J2=1.532960 rad and
J3=-1.443694 rad, with the same latched PD references and hold gains. Thus the
initial five-second evidence of retained support must not be described as the
current state. `late_fallback_wire_audit.json` records the exact FF transitions
(raw wire signs). No corresponding detailed feedback/heartbeat event was
recorded at that later instant; its cause is not proven. The existing stale
feedback/heartbeat mask can clear all FF and never restore it while latched.
Current state is PD-only fault hold. No limit or latch was changed in response
to the request for temporary limit relaxation.

## 10. Final status / recovery condition

**ARM6 GRAVITY / MANUAL GUIDANCE: NOT YET VERIFIED.**
Bumpless startup passed 3/3; J1 passed; J2–J6 remain incomplete.
After the operator confirmed the wrong-direction push, they explicitly requested
a retry. Current J2 remains above 1.47 rad, so a restart cannot satisfy the
unchanged near-limit gate. No reset/retry or motion command has been issued.
A supported physical reposition using the site's established safe procedure
is needed before a new explicit trial. Do not disable limits or force GUIDE at
the current pose. Driver PD fallback and RViz remain running.

## 11. Git diff

`git diff --stat`: 21 tracked files, 709 insertions, 137 deletions; includes
pre-existing local edits and excludes untracked guidance/config/test/docs files.
Full output: `git_diff_stat.txt`. No commit/push/reset/checkout performed.
