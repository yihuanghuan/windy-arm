# Wrist-only instantaneous-cap extension — 2026-09-18

Status: **NOT YET VERIFIED**. Offline tests passed. First real restart faulted in HOLD; one explicitly requested retry passed two handovers then failed its third startup timestamp gate. Manual-push profile was not enabled.

Explicit operator request after the `speed_18_20260918` run. Operator confirmed human pushing at J4 −2.014652 rad/s instantaneous abort and no abnormal behavior afterward. The 2.0 rad/s cap worked as configured. No automatic restart occurred.

Only MANUAL_PUSH instantaneous caps change: J1–J3 remain 2.0 rad/s; J4–J6 increase to 2.2 rad/s. All six warning thresholds remain 1.0 rad/s and sustained protection remains 1.8 rad/s for 120 ms. Wrist motor configured max velocity is 5 rad/s; this is a modest 10% cap extension, not permission to approach motor maximum. Transition cap remains 0.5 rad/s. Ordinary GUIDE thresholds, gravity, offset, gains, joint limits/margin, torque limits, freshness, heartbeat, latch and fallback remain unchanged.

Regression tests retain prior profiles. Added six-axis instantaneous/sustained separation and phase exclusion tests in Python/C++, plus a short 2.1 rad/s wrist burst in the three-handover pseudo-serial test. Previous real run report and fault ±2 s plots remain unchanged under `../speed_18_20260918/`.

No fault-clearing service or automatic rearm added. Any real restart requires no-contact readiness and preservation of the actual currently applied HOLD command, followed by fresh three-trial handover gates.

## Offline verification

- Python 61 passed (`python_tests.txt`).
- Four functional C++ suites passed (`cpp_tests.txt`).
- Build passed (9.60 s; `/tmp/windy-arm6-wrist22-build.txt`); launch argument/syntax validation passed.
- Initial pseudo-serial run: 12 passed, one failure. The synthetic 2.1 rad/s burst remained active while the accumulated serial log was decoded; actual stimulus exceeded 120 ms and correctly triggered sustained protection. Fixed the test to reset velocity before log parsing. No production protection change was made for this failure. Original failure retained in `pty_initial_failure.txt`; rerun result in `pty_tests.txt`.

Latest read-only check of the live fault HOLD confirmed J4 FF=0; other axes retained FF. Six-axis feedback was fresh and nearly stationary. No fault reset or new physical activation has occurred during offline preparation.

Pseudo-serial rerun: **13 passed in 60.53 s**. New cap is present in the installed YAML but has not been loaded into the currently running hardware process. Await explicit no-contact readiness before restarting from the current fault HOLD. No power cycle requested.

## First physical attempt

Operator confirmed no-contact restart. First wire command correctly inherited actual fault-HOLD. After 0.398 s the generic feedback-safe check faulted, before any guidance node start. A 338.537 ms wire-command gap preceded fault. Exact cause (feedback age, heartbeat age, or another input validity condition) is not identifiable from this generic log and TX-only trace. It is not demonstrated to be a speed threshold fault. ROS recorder discovery completed only after fault, missing pre-fault feedback and rosout; hardware launch log and serial trace establish fault.

Fallback cleared all FF and latched PD hold. J3 settled by 0.021363 rad (1.22 degrees), peak recorded speed 0.210012 rad/s; J2 position unchanged. Completed guidance handovers: 0. GUIDE not entered. `startup_analysis.json`, `startup_fault.png`, `fault_window.json` retain evidence. 5,637 command batches passed checksum and zero ghost-slot verification. Operator did not observe the startup event and explicitly requested another attempt; no absence-of-abnormality claim is made.

A single separately logged operator-requested retry is in `retry01/`, with RViz startup deferred and RX/TX tracing enabled. This changes startup scheduling only, not any protection or model. Root cause remains unconfirmed.

## Final retry outcome

See [retry report](retry01/ACCEPTANCE_REPORT.md). Two handovers + quiet passed; third rejected a JointState timestamp before sending any guidance command, followed by the prepared-handover 0.5 s timeout. Stopped with bounded gravity-supported fault HOLD and RViz active; no additional retry. New profile manual push remains unverified.
