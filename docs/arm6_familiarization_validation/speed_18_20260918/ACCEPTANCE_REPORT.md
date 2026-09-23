# ARM6 manual-push speed profile validation — 2026-09-18

Current status: NOT YET VERIFIED for the enlarged-speed whole-arm acceptance; stopped by the retained instantaneous cap. Previous profile acceptance remains in `../restart_after_power_20260918/ACCEPTANCE_REPORT.md`.

## Change and scope

Only explicit MANUAL_PUSH warning/sustained thresholds changed: warning 0.8 → 1.0 rad/s; sustained 1.5 → 1.8 rad/s for 0.12 s. Instantaneous abort remains 2.0 rad/s. Six per-axis values, independent monotonic timers. Transition abort remains 0.5 rad/s; ordinary GUIDE remains warning 0.5 / sustained 1.0 for 0.08 s / instant 1.2 rad/s.

The model, zero offsets, gravity scaling, GUIDE Kp/Kd, torque budgets, joint limits/margin, freshness, heartbeat, latch and fallback are unchanged. Motor config speed capabilities are 3 rad/s for J1–J3 and 5 rad/s for J4–J6; the 2.0 rad/s instantaneous cap remains below these values. No intentional overspeed test on hardware.

Production configuration: `src/arm-platform/config/arm6_familiarization.yaml`. Added/extended offline tests in `test/core/test_manual_guidance.py`, `test/test_familiarization_pty.py`, `test/test_effort_safety.cc`; existing conservative-profile tests retained.

## Offline verification

- Python: 52 passed (`python_tests.txt`).
- Pseudo-serial: 13 passed (`pty_tests.txt`), including independent speed timing, warning-only operation, fault support and latch.
- C++: 4 functional test suites passed (`cpp_tests.txt`).
- Package build and launch argument/syntax validation passed (`launch_args.txt`, `/tmp/windy-arm6-speed18-build.txt`).

## Real setup

Existing actual healthy HOLD command captured and supplied through `hold_command_file` for driver restart. No power cycle and no guessed cold-start command. The new driver runtime parameters were read back as sustained 1.8 × 6, instant 2.0 × 6. Six-dimensional joint feedback measured approximately 102.6 Hz. ROS domain 95, the previously verified CP2102 by-id serial device, gamma6 / arm6.urdf. Recording covers applied commands, feedback, policy and driver diagnostics, and decoded serial TX.

Three warm no-touch handovers passed, each followed by at least 5 seconds in GUIDE, before enabling manual-push via both explicit flags and the gated service. Operator explicitly confirmed readiness. No HARD_ABORT observed through handovers.

## Results

| Trial | J2 peak handover speed rad/s | J2 position span rad | J2 maximum command step Nm | 5 s GUIDE quiet |
|---|---:|---:|---:|---|
| 1 | 0.004885 | 0.000000 | 0.00001411 | PASS |
| 2 | 0.004885 | 0.000000 | 0.00001476 | PASS |
| 3 | 0.004885 | 0.000000 | 0.00001427 | PASS |

First-five-second quiet position spans: J1–J3 zero at encoder resolution; maximum wrist span 0.000767 rad. Three transitions and quiet windows passed without faults.

Manual motion ran 95.57 s before abort. Per-axis peak |dq| rad/s: `[0.7570209503173828, 0.6398048400878906, 0.6300373077392578, 2.0146522521972656, 1.6483497619628906, 0.1831512451171875]`; spans degrees: `[57.70231237248148, 54.48933674539413, 68.45592612628964, 140.38202548305264, 76.20234419232457, 0.06595230662564312]`. No separate per-joint reacceptance was performed in this run.

### HARD_ABORT and fallback

At MANUAL_PUSH, J4 instantaneous speed −2.014652 rad/s exceeded 2.0 rad/s. Sustained duration reported 0 because the instantaneous rule takes priority. At trigger J4 q=1.0060215 rad, actual pre-fault Kp=0, Kd=0.05, last applied FF=−0.00743188 nominal Nm, measured effort=+0.14212465 nominal Nm. Python and driver both stopped/latch. The driver log labels this GUIDE; its structured speed diagnostic and Python log identify the enabled MANUAL_PUSH subprofile. This is a log-label limitation, not use of ordinary GUIDE thresholds.

The operator explicitly confirmed active human pushing and no abnormal behavior after fault. Retained FF on J2/J3 was −0.659740826 / +1.118405525 nominal Nm; only J4 FF became zero, and PD hold was applied. The same retained support persisted over the recorded 5 seconds. Post-fault J2 position span was 0.094224 rad during settling/operator contact; do not describe this as zero motion. No whole-arm FF drop and no automatic restart occurred.

25,252 actual serial command batches passed checksum, six-axis and zero ghost-slot checks. Plots: [overview](overview.png), [fault ±2 seconds](fault1_window.png), [whole-arm](whole_arm_push_release.png), and `trial{1,2,3}_j2_handover.png`. Raw fault window is `fault_window.json`.

The earlier six-axis gravity/manual-guidance PASS is preserved as historical evidence. This expanded-profile whole-arm run ended at protection, so its final acceptance is NOT YET VERIFIED. Driver remains in latched fault HOLD. No restart or threshold change was performed during this run. A subsequent explicit operator request for modest wrist-only threshold expansion is tracked separately.
