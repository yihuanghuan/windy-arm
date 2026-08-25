# Hardware Bring-Up Status — EE Stabilisation Under Base Disturbance

Snapshot of where the hardware bring-up stands, as of **2026-08-25**, on the NX with the
A-L1-GAMMA arm and Vicon at `192.168.200.60:3883`.

Companion to `README_jasyach.md`, which holds the *procedure*. This file holds the
*state*: what has been proven, what is blocking, and what to do next. Every number below
came from a runner summary JSON on real hardware.

---

## 1. State of play in one paragraph

The control stack is proven good — a stationary hold reached **1.196 mm RMS** with no
torque clamping and no QP fallbacks. Every large tracking failure so far traces to Vicon
geometry, not to the controller. The base marker cluster and the end-effector marker
cluster were both physically moved, which invalidated the 2026-08-05 extrinsic
calibration. zzq's 2026-08-24 file is closer but still insufficient. **Base extrinsic
recalibration is the critical path.** A second, independent fault — the base state going
stale under hand-moved base motion, causing total torque loss — is not yet explained and
matters more for safety.

---

## 2. Runs performed

| # | Time | Launch | Base source | `ee_feedback.mode` | Torque | Outcome |
|---|---|---|---|---|---|---|
| 1 | 21:33 | `vicon_hold_demo` | Vicon, old extrinsics | `vicon_direct` | **on** | watchdog @ 2.17 s, 54.5 mm |
| 2 | 22:01 | `vicon_hold_demo` | Vicon, old extrinsics | `fk_only` | **on** | ran clean; limp collapse when base moved |
| 3 | 22:21 | `vicon_hold_demo` | Vicon, zzq extrinsics | `vicon_direct` | off | start posture rejected @ 5.7 s |
| 4 | 23:03 | `whole6 hwtest` | analytic (no Vicon) | `fk_only` | off | watchdog @ 24.4 s — dry-run artifact |
| 5 | 23:08 | `whole6 hwtest` | analytic (no Vicon) | `fk_only` | **on** | watchdog @ 22.2 s, j2 overspeed |
| 6 | 23:27 | `whole6 hwtest` | analytic (no Vicon) | `fk_only` | **on** | prepose correct; watchdog @ 6.3 s, j2 overspeed, **visible shaking** |

### Measured results

| Metric | Run 1 | Run 2 | Run 3 | Run 4 | Run 5 |
|---|---|---|---|---|---|
| `samples` | 324 | 2807 | 674 | 2548 | 2227 |
| `position_rms_m` | 44.86 mm | **1.196 mm** | 18.5 mm | 54.19 mm | 58.39 mm |
| `position_peak_m` | 121.75 mm | 1.61 mm | 50.0 mm | 59.18 mm | 74.41 mm |
| `orientation_rms_rad` | 17.85° | 0.78° | 9.18° | 98.3° | 91.1° |
| `ee_residual_rms_m` | 46.9 mm | `null` | **31.2 mm** | `null` | `null` |
| `ee_orientation_residual_deg_p95` | 76.3° | `null` | 128.1° | `null` | `null` |
| `ee_velocity_reject_ticks` | 202/324 = 62% | 0 | 79/674 = 12% | 0 | 0 |
| `ee_degraded_ticks` | 83 | 0 | 174 | 0 | 0 |
| `backend_clamp_total` | 0 | 0 | 0 | 0 | 0 |
| `qp_fallback_ticks` | 0 | 0 | 0 | 0 | 0 |
| `qp_enabled` | true | true | true | false | false |

Position figures for runs 3–5 are not control-quality measures: run 3 had no torque while
the base was moved by hand, and runs 4–5 started 175° out of position on j4. Only run 2 is
a clean control result.

---

## 3. Established

1. **The controller stack is sound.** Run 2: 1.196 mm RMS / 1.61 mm peak, zero
   `backend_clamp_total`, zero `qp_fallback_ticks`. Narrowly misses the 1 mm RMS gate,
   passes the 2 mm peak gate, and does so with a stale base extrinsic still feeding `b6`.
   Arm, driver, gravity model, friction feedforward, QP and SMC law are all fine.

2. **The 2026-08-05 extrinsics are stale, not wrong-in-fit.** They were an excellent
   1.77 mm RMS fit to geometry that no longer exists. Run 1 measured 46.9 mm residual
   against them — 26× degradation.

3. **zzq's 2026-08-24 extrinsics are better but insufficient.** Velocity rejection fell
   from 62% to 12%, so his base rotation is much closer to current hardware. But
   `ee_residual_rms_m` plateaus at 31.2 mm against `innovation_reject_m: 0.05`, leaving
   under 19 mm of headroom — any base motion crosses it.

4. **The fault is the base *rotation*, not the translation.** Run 3 was stable while
   stationary and failed only when the base was moved. A translation error shifts the FK
   and Vicon end-effector paths identically and is invisible; a rotation error separates
   them in proportion to arm reach × sin(error). The offending field is
   `arm_base_from_base_link.quaternion_xyzw`.

5. **zzq re-fitted only the base half.** His `arm_ee_from_link6.translation_m` is
   byte-identical to the 2026-08-05 file. With both clusters reported moved, that
   un-refitted lever arm is likely part of why his residual stops at 31 mm.

6. **The calibration tool cannot fit the end-effector lever arm.**
   `vicon_base_extrinsic_calibrate.py` says so twice — freeing both unknowns becomes an
   AX = XB hand-eye problem needing different excitation, so it assumes the `arm_ee`
   marker origin coincides with link6's origin. This contradicts the 17.8 mm currently in
   the config; see §5.

7. **A dry run validates a hold, not a moving trajectory.** With `hardware_enabled:=false`
   the arm cannot follow, so any moving reference walks away from it and trips the 50 mm
   watchdog regardless of correctness. Run 4 confirms the arithmetic: offset 54.5 mm,
   threshold 50 mm, blend 30 s → trip predicted at 27.5 s, observed 24.4 s.

8. **`start_delay_sec:=0` is correct for `hwtest` on hardware.** The parent README states
   the handover blend replaces the approach segment and warns against reverting to an 8 s
   approach. `approach.rows_logged: 0` in runs 4–5 confirms the approach never executes,
   so the `峰值速度约 2609 rad/s` line is a diagnostic for a segment that does not run.

9. **The Vicon stream itself is healthy.** `/floating_base/vicon` holds 120 Hz with a
   25 ms maximum interval *including while the base is moved* — nowhere near the 100 ms
   `max_sample_gap_sec` that triggers withholding.

---

## 4. Blockers

### Blocker 1 — base extrinsic recalibration (critical path)

Both candidate files have been measured on hardware and both fail. There is no third file.

```bash
smc_hw
python3 src/arm-platform/task_space_smc/hw/vicon_base_extrinsic_calibrate.py --plan-only
```

`--plan-only` previews the 16-pose sweep without moving anything. Run it from an unfolded
posture. Output is JSON at `hw/data/vicon_base_extrinsic/base_extrinsic_<ts>.json`; there
is no automatic writer, so transcribe `translation_m` and `quaternion_xyzw` into the
`arm_base_from_base_link` block of `config/vicon_extrinsics.yaml` by hand.

`hw/vicon_fk_base_calibrate.py` is a static-pose variant of the same fit. Because every
sample is taken at rest it needs no Vicon/encoder clock alignment, which may make it the
safer choice given blocker 2.

### Blocker 2 — base state goes stale under base motion (safety-critical)

Run 2 logged `Waiting for fresh Vicon base state: base state is stale` and the arm went
limp. This is **not** the watchdog and behaves oppositely:

| | Watchdog | Base-state staleness |
|---|---|---|
| Trigger | error > 50 mm, or joint velocity over envelope | no fresh `/floating_base/vicon` within 0.1 s |
| Action | **holds position** at the frozen pose | **stops publishing entirely** |
| Result | arm stays put | torque → 0 after 0.03 s, **arm falls** |

The frame-gap hypothesis is **disproven** by §3.9. `ros2 topic hz` measures arrival
intervals, not message age, so it cannot see latency or clock-offset error — and
staleness is judged against `stamp_local = source + offset`, where `offset` is the
~11.992 h Vicon-to-local clock difference estimated online. The right diagnostic is:

```bash
smc_hw
python3 src/arm-platform/task_space_smc/hw/vicon_link_check.py --seconds 60
```

Its docstring describes this exact trap: 话题、rate、ping 都正常，实测传输延迟 p95 却可能比
基线差 5 倍（79.9 ms → 431.8 ms），一半的控制拍因此停发力矩.

**Resolve this before any torque-enabled test that involves moving the base.**

---

## 5. Open questions

- **Is the 17.8 mm end-effector lever arm real?** The calibration tool assumes it is zero;
  `smc_params_hold.yaml` warns that disabling `apply_control_point_offset` sends 17.9 mm
  of lever error into torque. Both cannot be right. The old file attributes the number to
  "model D", one of five candidates, but the tool in the tree does not implement model D,
  so its provenance is untraced.

  Resolution once the base is recalibrated — derive it from data at several static poses:

  ```
  r_world = p_vicon_ee − p_fk_link6
  t_ee    = R_fk_link6ᵀ · r_world          # link6's frame, not base_link's
  ```

  Consistent across poses → that is the lever arm. Varying with pose → residual base
  error remains. Converging near zero → the tool's convention was right and
  `apply_control_point_offset: false` becomes the honest setting. No script exists for
  this yet.

- ~~What repositions the arm?~~ **Closed 2026-08-25 — nothing does.** `start_q_rad` in
  the summary is not telemetry: it echoes `initial_pose_rad` from
  `smc_params_hwtest.yaml` line 38, byte-identical
  (`[-0.0635, 1.5558, -1.4086, -0.0002, 0.6938, 0.0033]`), described there as a
  model-sanity-check posture. The arm was never folded. Run 6 confirms preposing sticks:
  `handover_shortfall_rad` max 3.68° against 175° in runs 4–5.

- **Why does the arm shake during `hwtest` tracking?** Run 6 started correctly
  (handover offset 13.4 mm, joint shortfall max 3.68°) yet `position_rms_m` grew to
  42.6 mm and j2 hit −3.77 rad/s within 6.3 s, with visible shaking, while the trajectory
  clock was still frozen and the reference nearly stationary. Three candidates, untested:
  **(a)** the driver runs at 100 Hz (`student_arm6.launch.py` default) while the
  controller is configured for 200 Hz — sample counts across runs 2, 4, 5 and 6 all land
  near 100–117 Hz; **(b)** a 29.3° handover orientation offset arising from only 3.68° of
  joint shortfall, suggesting the trajectory's t=0 attitude does not match FK at the
  nominal start posture; **(c)** stick-slip on j2, which carries 1.775 N·m stiction and
  the widest deadband. The generated `02_tracking_error.png` and `03_joint_torque.png`
  should discriminate between (b) and (c).

- **Why did the orientation residual worsen from 76.3° to 128.1°** when the base extrinsic
  improved? Unexplained. Also the least trustworthy signal, since the `arm_ee` rigid body
  is documented as rolled 114.4° about link6's tool axis with orientation validation-only.

---

## 6. Next actions, in order

1. **`vicon_link_check.py --seconds 60`** — resolve blocker 2. Free, read-only, and it
   gates every torque-enabled base-motion test.
2. **Prepose** to the hold start posture. From the last observed pose this is a single
   move (largest travel 1.00 rad):
   ```bash
   python3 src/arm-platform/task_space_smc/hw/full_prepose.py \
     --target-rad="0.4571390408,-0.1429996220,-0.9965683534,0.7027214717,-0.6167795475,0.0178438264" \
     --duration 30 --settle 3
   ```
   Then verify it stuck (§5, second bullet) before proceeding.
3. **Interim base-disturbance test with `fk_only`.** This still rejects base disturbance —
   the base pose comes from Vicon either way, so FK sees the base move and the controller
   compensates; it only drops the direct Vicon *tip* measurement, removing the 31.2 mm
   residual from the loop. Run with `hardware_enabled:=false` first, then `true`.
4. **Recalibrate the base extrinsic** — blocker 1.
5. **Resolve the end-effector lever arm** — §5, first bullet.
6. **Switch back to `vicon_direct`** and repeat the hold demo. Target: `ee_residual_rms_m`
   near 2 mm, `ee_velocity_reject_ticks` near zero, `position_rms_m` near the 1.196 mm
   baseline.

The `hwtest` trajectory on `whole6` is a **separate** experiment — fixed analytic base,
moving end-effector, no Vicon in the loop. Useful as an independent check of the arm and
controller, but it is not the base-disturbance test and does not block it. It needs its
own two-segment prepose to reach j4 = −77.7°.

---

## 7. Current configuration state

| Item | Value | Note |
|---|---|---|
| `config/vicon_extrinsics.yaml` | zzq 2026-08-24 values | swapped in with a provenance header; **trial fit, measured insufficient** |
| `ee_feedback.mode` | `vicon_direct` | set to `fk_only` on the NX for run 2, reverted for run 3 |
| Vicon server default | `192.168.200.60` | now the default in both launch files and `vicon_link_check.py` |
| `base_state_mode` (`whole6`) | `analytic` | default; Vicon only when explicitly passed |
| `.bashrc` `smc_vicon` | `vicon_perception/src/install/setup.bash` | note the `src/` element |

Known-failing test: `test/core/test_base_state_fusion.py::test_extrinsics_yaml_still_carries_both_vectors_unswapped`
hardcodes the 2026-08-05 base translation. It will fail until the extrinsics are settled,
then its constant should be updated to the final calibration. This is the test doing its
job.

---

## 8. Related documents

- `README_jasyach.md` — adapted run procedure, and the extrinsics verification in §6
- `BASE_ACCELERATION_PIPELINE.md` — how `a_base` is derived; why the VRPN twist/accel
  channels are inert
- `RUNBOOK_NX.md` — general simulation and hardware procedures
- `CONTROL_ARCHITECTURE.md` — control law and frame conventions
- `QA_record.md` — measured failures behind several of the guards described here
