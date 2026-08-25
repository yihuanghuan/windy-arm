# Moving-Base Hold Test — Adapted Procedure

Adaptation of `README_zzq.md` to this checkout. zzq's handoff was written against a
different working copy; several names in it do not exist here. This document records the
translation, the verification of his two calibration files, and the run procedure for a
**headless** NX (no monitor attached).

The three handoff files are left untouched in the repository root. Nothing in this
document has been applied to the source tree.

Status of each claim below: **verified** means checked against source in this checkout.
Everything unverified is marked as such.

---

## 1. Environment mapping

| Item | zzq | Here | Status |
|---|---|---|---|
| Workspace root | `/home/zhangziqian/windylab_ws/lagrange_smc/LL100-LH420` | `~/jasyach_repo/windylab_arm_control_FanC` | differs |
| Vicon workspace | not stated | `~/jasyach_repo/vicon_perception` | — |
| Serial port | `/dev/ttyUSB0` | `/dev/ttyUSB0` | **confirmed present** |
| Vicon server | `192.168.200.60` | `192.168.200.60` | **matches** |
| Vicon port | `3883` | `3883` | **matches** |
| `ROS_DOMAIN_ID` | 42 | 42 (hardware) | matches |

The serial device appears only after the arm is powered and the USB-UART adapter is
plugged in — the first `ls` returned only `ttyTHS1` and `ttyTHS2`. Note that
`student_arm6.launch.py` still documents `/dev/ttyTHS3` as the NX port; that device does
not exist on this machine. Use `/dev/ttyUSB0`.

`/dev/ttyUSB0` is `crw-rw---- root dialout`, so the account must be in the `dialout`
group. Check once with `id -nG | grep dialout`; if absent,
`sudo usermod -aG dialout $USER` and log out and back in.

---

## 2. Launch name resolution

`vicon_moving_base_candidate_test.launch.py` **does not exist in this checkout**
(verified: `ls launch/`). The equivalent is **`vicon_hold_demo.launch.py`**.

Evidence that these are the same launch under a different name — all four arguments zzq
passes are declared in `vicon_hold_demo.launch.py`, with no others required:

| zzq's argument | Declared here | Default |
|---|---|---|
| `vicon_server` | yes, around line 193 | `192.168.200.60` |
| `vicon_port` | yes, around line 194 | `3883` |
| `hardware_enabled` | yes, around line 167 | `false` |
| `control_rate_hz` | yes, around line 198 | `200.0` |

The behaviour also matches. zzq writes "the runner captures the current FK end-effector
pose as the hold target instead of using a prewritten trajectory." This launch declares
`hold_capture_measured`, default `true`, and its module docstring states the target pose
is captured from the *measured* end-effector pose on the first controlled cycle, so the
first-cycle task-space error is exactly zero.

This is a strong match but not proof — I cannot see zzq's file. Treat it as the
best available equivalent, not a certainty.

Two further differences from zzq's command:

- `vicon_server` used to default to the stale `172.16.20.21`. It now defaults to
  `192.168.200.60`, so passing it is optional. The commands in section 3 still pass
  it explicitly, which is harmless and makes the target unambiguous in logs.
- The argument for the visualiser is `rviz`, not `use_rviz`, and it already defaults to
  `false`. `student_arm6.launch.py` uses `use_rviz` and defaults to `True`, so that one
  must be overridden for headless operation.

---

## 3. Run procedure (headless)

### 3.1 Shell helpers

Use the `.bashrc` functions rather than sourcing by hand. They set `ROS_DOMAIN_ID`,
source the right overlays in the right order, and land in the right directory:

| Function | Domain | Overlays sourced | Use for |
|---|---|---|---|
| `smc_hw` | 42 | arm | hardware terminals |
| `smc_sim` | 77 | arm | simulation |
| `smc_vicon` | 42 | arm **+** vicon | the Vicon terminal only |

`smc_vicon` calls `smc_hw` first and then adds the Vicon overlay. That chaining is not
optional. `vicon_floating_base.launch.py` is a `manipulator` launch file — from the arm
workspace — that spawns nodes from **two different packages** (verified, around lines
85–96 and 121):

```python
vrpn_listener = Node(package="vrpn_listener", executable="vrpn_listener", ...)
bridge        = Node(package="manipulator",   executable="vicon_floating_base_bridge.py", ...)
return [vrpn_listener, bridge]
```

Sourcing only the Vicon workspace would fail to resolve `ros2 launch manipulator`;
sourcing only the arm workspace would fail to find the `vrpn_listener` executable. The
two packages do not collide — `manipulator` exists in one workspace and
`vrpn_listener` / `VRPN` in the other — so chaining them is safe.

> **Check your `.bashrc` path.** If `smc_vicon` still points at
> `~/jasyach_repo/WindyLab-ViconPerception`, it is stale — that clone was deleted. The
> line should read:
>
> ```bash
> smc_vicon() { smc_hw
>               source ~/jasyach_repo/vicon_perception/src/install/setup.bash; }
> ```
>
> Note the `src/` element. The Vicon workspace was built by running `colcon build`
> from inside `vicon_perception/src/`, so colcon treated that directory as the
> workspace root and created `src/build`, `src/install`, `src/log` there. The overlay
> is one level deeper than the arm workspace's.

Reload with `source ~/.bashrc` after editing, or open a new terminal.

### 3.2 Terminal 1 — hardware driver

```bash
smc_hw

ros2 launch manipulator student_arm6.launch.py \
  arm_type:=a_l1 \
  port_name:=/dev/ttyUSB0 \
  enable_direct_effort_command:=true \
  use_rviz:=False
```

`use_rviz:=False` is the headless change from zzq's command; this launch defaults it to
`True`. Leave the terminal running for the rest of the session.

### 3.3 Terminal 2 — move the arm to a safe posture

Do not start the hold controller from a folded posture.

```bash
smc_hw
python3 src/arm-platform/task_space_smc/hw/wrist_prepose.py \
  --target-deg 40 --duration 20 --settle 3
```

**Not `ros2 run`.** `ros2 run manipulator wrist_prepose.py` returns
`No executable found`, because `ros2 run` searches `lib/manipulator/` and this script is
not in the `install(PROGRAMS ... DESTINATION lib/${PROJECT_NAME})` list in
`CMakeLists.txt` (around lines 246–279). Only seven `hw/` scripts are — the runner, the
bridge, the phase tools, and the probes. `wrist_prepose.py` does land on disk, via the
`install(DIRECTORY task_space_smc/ ...)` rule around lines 281–288, but one level deeper
at `lib/manipulator/task_space_smc/hw/` and without the executable bit. Invoking it
through `python3` from the source tree sidesteps both issues and matches every other
document in this repository.

This moves joint5 out of the wrist singularity. `vicon_hold_demo.launch.py`'s own
docstring lists it as step 2 of the intended sequence. `--target-deg 40` is the value
the parent README specifies; the allowed range is 5–60.

**Let it run to completion.** Interrupting with `Ctrl+C` leaves the joint partway and
prints an `RCLError: rcl_shutdown already called` traceback — a harmless double-shutdown
in the script (around lines 148–150), but the move itself is left unfinished.

**If the arm is in a folded posture, `wrist_prepose.py` is not enough** — it only moves
joint5. Use `full_prepose.py` to reach the canonical hold start configuration, which is
the posture `config/smc_params_hold.yaml` is built around (around lines 184–186):

```bash
smc_hw
python3 src/arm-platform/task_space_smc/hw/full_prepose.py \
  --target-rad="0.4571390408,-0.1429996220,-0.9965683534,0.7027214717,-0.6167795475,0.0178438264" \
  --duration 30 --settle 3
```

A folded start (for example j2 near +78° against j3 near −77°) is near-singular and must
not be the starting point for the hold controller.

### 3.4 Terminal 3 — Vicon

**This is the one terminal that needs `smc_vicon`, not `smc_hw`.**

```bash
smc_vicon

ros2 launch manipulator vicon_floating_base.launch.py \
  server:=192.168.200.60 \
  port:=3883
```

`server` and `port` now match the launch defaults, so both may be omitted. They are
kept here so the target appears verbatim in the logs.

Note that this launch passes `server` and `port` as **node parameters**, which override
`config/params.yaml` in the Vicon workspace. The IP in that file only takes effect when
running the listener standalone through `vrpn_client.launch`. The launch also overrides
`mainloop_frequency` to 200.0 and sets `frame_id` to `vicon_world`.

Confirm data before continuing (§4).

### 3.5 Terminal 4 — dry run first, with torque disabled

**Do not go straight to `hardware_enabled:=true`.** Run the identical command with the
default `hardware_enabled:=false` first:

```bash
smc_hw

ros2 launch manipulator vicon_hold_demo.launch.py \
  vicon_server:=192.168.200.60 \
  vicon_port:=3883 \
  hardware_enabled:=false \
  control_rate_hz:=200.0 \
  start_vicon:=false \
  rviz:=false
```

With `hardware_enabled:=false` the launch description states 适配器输出恒为零，机械臂不动
— the actuator adapter emits identically zero and the arm cannot move. Everything else
runs: the world frame is established, the hold target is captured, `b6` is computed, and
diagnostics publish.

Watch the reported end-effector position error. If it is not small and stable, the
extrinsics do not match the hardware and **torque must stay off** — see §6.

### 3.6 Terminal 4 — hold controller with torque

Only after the dry run looks clean:

```bash
smc_hw

ros2 launch manipulator vicon_hold_demo.launch.py \
  vicon_server:=192.168.200.60 \
  vicon_port:=3883 \
  hardware_enabled:=true \
  control_rate_hz:=200.0 \
  start_vicon:=false \
  rviz:=false
```

`smc_hw` is correct here — this launch starts no VRPN node of its own when
`start_vicon:=false`, so the Vicon overlay is not needed.

`start_vicon:=false` because terminal 3 already started it. Its default is already
`false`, but stating it makes the intent explicit and guards against double-launching
the bridge, which would produce two nodes racing to define the controller world frame.

---

## 4. Startup checks

### 4.0 What "verify the topics are live" means

Nothing in this section runs before you launch anything. On an idle machine
`ros2 topic list` returns exactly two entries:

```text
/parameter_events
/rosout
```

Both are ROS 2 built-ins created automatically by every node — including the
`ros2 topic list` command itself, which is briefly a node while it runs. Seeing only
those two is the correct result for a system with nothing started, not a fault.

The purpose of these checks is a safety ordering. Terminals 1 and 3 only **read** from
the robot and the motion-capture system. Terminal 4, launched with
`hardware_enabled:=true`, is the one that publishes to `/student/joint_effort_command`
and therefore the one that makes the arm apply torque. "Verify before enabling effort
control" means: confirm the inputs are healthy while nothing can move, then start the
thing that moves it.

Run the checks in a **fifth terminal** that stays open for the whole session, and run
`smc_hw` in it first. Two check points, both before any torque is enabled:

| When | Run | Section |
|---|---|---|
| after terminal 1 is up, before terminal 2 | driver topics | §4.1 |
| after terminal 3 is up, before terminal 4 | Vicon topics | §4.2 |

Terminal 2 needs no check of its own — `wrist_prepose.py` prints its own progress and
exits. This is the most
common way these checks appear to fail: `ROS_DOMAIN_ID` partitions DDS discovery, so a
terminal on the wrong domain sees an empty topic list even though everything is running
normally. Domain 42 is hardware; a plain new terminal has no domain set at all.

Also note the distinction between two commands:

- `ros2 topic list` — shows a topic if any node has *declared* a publisher. It does not
  mean data is flowing.
- `ros2 topic hz <topic>` — measures the actual publication rate. This is the one that
  tells you the sensor is alive.

A topic that lists but reports no rate is a publisher with nothing behind it, which is
exactly the failure mode worth catching before torque is enabled.

### 4.1 After terminal 1 (hardware driver)

```bash
smc_hw
ros2 topic list | grep -E 'joint_states|student'
ros2 topic hz /joint_states
```

Expected (verified against `src/ros_nodes/student_arm_node.cc`, around lines 36–50):

| Topic | Direction | Meaning |
|---|---|---|
| `/joint_states` | published | joint positions and velocities |
| `/student/joint_feedback` | published | `MotorState`, includes the 0x22 torque report |
| `/student/joint_command` | subscribed | position commands |
| `/student/joint_effort_command` | subscribed | **torque commands — terminal 4 writes here** |

`/joint_states` should show a steady rate near the driver's `control_rate_hz`. If the
topic exists but `hz` prints nothing, the node started but the serial link is not
returning data — check the port and the `dialout` group.

Subscribed topics appear in `ros2 topic list` too. Seeing `/student/joint_effort_command`
at this stage is normal and does not mean anything is commanding torque; use
`ros2 topic info /student/joint_effort_command` to confirm it has 1 subscriber and
0 publishers until terminal 4 starts.

### 4.2 After terminal 3 (Vicon)

```bash
smc_hw
ros2 topic list | grep -E 'vrpn|floating_base|vicon'
ros2 topic hz /vrpn/arm_base/pose
ros2 topic hz /floating_base/vicon
```

Expected:

| Topic | Source | Note |
|---|---|---|
| `/vrpn/arm_base/pose` | `vrpn_listener` | ~125 Hz |
| `/vrpn/arm_base/twist` | `vrpn_listener` | present but unused — see `BASE_ACCELERATION_PIPELINE.md` §3 |
| `/vrpn/arm_base/accel` | `vrpn_listener` | present but unused, same reason |
| `/vrpn/arm_ee/pose` | `vrpn_listener` | ~125 Hz |
| `/floating_base/vicon` | bridge | the controller's actual base input |
| `/vicon/arm_ee` | bridge | end-effector state |

Two things specific to this stage:

**The VRPN topics do not exist until a rigid body is discovered.** `vrpn_listener`
creates publishers per discovered sender, and the launch sets
`refresh_trackers_frequency: 1.0`, so discovery runs once per second. Allow a few
seconds. If the topics never appear, the listener is connected but the Vicon server is
streaming nothing.

**The topic names come from the rigid-body names in Vicon Tracker.** `arm_base` and
`arm_ee` are not chosen by this code — `createSynchronizer` builds the topic string from
the VRPN sender name, replacing spaces with underscores. If Tracker calls the bodies
something else, the topics will be named accordingly and the launch's
`pose_topic` / `ee_pose_topic` arguments must be pointed at the real names.

`/floating_base/vicon` should track the Vicon frame rate. If it is materially slower
than `/vrpn/arm_base/pose`, the bridge is withholding frames — most likely the
missing-acceleration guard, which suppresses the whole message rather than publishing a
zero acceleration.

### 4.3 Before enabling torque

zzq records Vicon transport and source-order gaps "up to the second range" in his last
diagnostic. The bridge's `max_sample_gap_sec` is 0.10 s, so second-scale gaps reset the
derivative chain repeatedly and make the acceleration feedforward meaningless.

**Do not launch terminal 4 with `hardware_enabled:=true` until those gaps are gone.**
Watch terminal 3's log for `Vicon sample gap ...s; reset derivative estimates` warnings.
A clean run over 30 seconds is the gate.

Once terminal 4 is running, stop immediately with `Ctrl+C` on any of: stale Vicon data,
source-timestamp disorder, `No response`, missing acceleration, joint-limit errors, or
invalid FK state. This part of zzq's guidance carries over unchanged.

---

## 5. Where zzq's setup has diverged from this one

These are not path differences. They are code differences, and each was checked against
source here.

**(a) The `ARM6_CALIBRATION_YAML` and `VICON_EXTRINSICS_YAML` environment variables do
not exist.** Verified: a repository-wide grep finds no reference to either name. zzq's
startup check expects them in the launch output; they will never appear. The calibration
filename is hardcoded in `core/model.py`, around line 39:

```python
ARM6_CALIBRATION_NAME = "arm6_calibration.yaml"
```

So `arm6_calibration_candidate.yaml` is not read by anything here. Selecting a different
calibration file means replacing `config/arm6_calibration.yaml`, not adding a candidate
alongside it.

**(b) The gravity model zzq describes is not the one running here.** He writes:

```text
tau_corrected = (tau_rnea - gravity) + first_moment_ratio * gravity
```

Verified: `first_moment_ratio` is read in exactly two places in this checkout,
`hw/phase4_gravity_compensation.py` (around line 85) and `hw/phase5_inertia_id.py`
(around line 64). Both are standalone calibration tools. The SMC control path —
`whole_task_smc_ros_runner.py` — never reads it. From the calibration file it takes only
`stiction_nm` (around line 1554), and `core/model.py` takes `armature_kgm2` for the mass
matrix. Gravity comes from the URDF.

**(c) Directional friction is not implemented here.** zzq's file supplies
`kinetic_friction_pos_nm` and `kinetic_friction_neg_nm`, and his README describes
selecting between them by the sign of joint velocity. Verified: `load_arm6_calibration`
reads only `kinetic_friction_nm`. The two directional keys would be loaded as YAML and
silently ignored.

The practical consequence: **copying zzq's calibration files here will not reproduce
zzq's behaviour**, because the code that consumes them differs. The files are inputs to
a model this checkout does not implement.

---

## 6. Verification of `vicon_extrinsics_zzq.yaml`

> **Update 2026-08-25 — the deciding question below has been answered.** The user
> confirmed the marker clusters on both the base and the end-effector have been
> physically moved. The current `vicon_extrinsics.yaml` is therefore **stale**: it is an
> excellent fit (1.77 mm RMS) to geometry that no longer exists. This was observed
> after a torque-enabled hold attempt produced violent motion — consistent with a
> corrupted world frame, since the 85.3° `q_rb` rotation is what keeps
> `zero_on_first_pose` from mistaking the marker-cluster frame for the controller world.

**Verdict: neither file should be adopted. Re-calibrate.** The current file is stale
(markers moved). zzq's file is from the right era but is labelled diagnostic-only by its
own author and fits five times worse.

```bash
smc_hw
python3 src/arm-platform/task_space_smc/hw/vicon_base_extrinsic_calibrate.py --plan-only
```

`--plan-only` previews the pose sequence without moving the arm. Run it before the real
calibration — the tool drives the arm through 16 postures, so the arm must already be in
a safe, unfolded configuration (§3.3).

Rotation error matters more than translation error here. A 24 mm lever change is
serious; a wrong world rotation is catastrophic, because it mis-orients every base
velocity and acceleration entering `b6`.

The original comparison follows, and still explains why zzq's file is not the
replacement.

Schema is fine. `load_base_extrinsic` in `launch/vicon_floating_base.launch.py` (around
lines 55–67) requires `arm_base_from_base_link.translation_m` (3 values) and
`quaternion_xyzw` (4 values). zzq's file has both. The extra `arm_ee_from_link6.
quaternion_xyzw` he adds is ignored — `core/ee_estimator.py` reads only `translation_m`.
The missing `rotation_matrix` block is harmless; the current file's own comment says no
code reads it.

The problems are substantive:

| Metric | Current | zzq | Change |
|---|---|---|---|
| Position residual RMS | 1.77 mm | 8.61 mm | **4.9× worse** |
| Position residual max | 2.75 mm | 13.24 mm | **4.8× worse** |
| Leave-one-out RMS | 2.15 mm | *absent* | no overfitting check |
| Base lever arm $\lVert t\rVert$ | 33.07 mm | 57.68 mm | **+24.6 mm** |
| Base rotation | — | — | **~20.6° apart** |
| `metadata.status` | *absent* | `diagnostic_only_...` | author's own caveat |
| `metadata.model` | `D — raw encoder angles...` | *absent* | provenance lost |
| `caveats` block | 3 open items | *absent* | provenance lost |

Three points on this.

**The author labelled it himself.** `status: "diagnostic_only_for_position_and_orientation_quality"`
is zzq stating this result is not for production. His notes add that the result "was not
used to claim sub-10-mm geometry accuracy across all conditions." Taking it as the active
extrinsic would use it for something he explicitly declined to use it for.

**The geometry change is physical, not numerical.** A 24.6 mm translation shift and a
20.6° rotation are not a refinement of the same fit — they describe the marker cluster
sitting somewhere else on the robot. This repository is unusually alert to that lever
arm: `base_state_fusion.py` warns that confusing it with the 17.9 mm end-marker lever
produces a 15.2 mm error feeding straight into `compute_task_bias_acceleration`. A
24.6 mm change is larger than the error that warning exists to prevent.

**It breaks a test.** `test/core/test_base_state_fusion.py::
test_extrinsics_yaml_still_carries_both_vectors_unswapped` opens the YAML and asserts the
base translation matches a hardcoded constant to 1e-6. zzq's values fail it. The
companion test `test_lever_arm_uses_the_base_extrinsic_not_the_end_marker_lever` would
still pass, but only because it checks the hardcoded constant rather than the file —
which would leave the constant silently disagreeing with the config.

**The question that decides this was physical, not analytical:** *has the `arm_base`
marker cluster been unmounted, moved, or remounted since 2026-08-05?*

**Answered 2026-08-25: yes, on both the base and the end-effector.** So the current file
is stale, but zzq's is not automatically the replacement. Re-run
`hw/vicon_base_extrinsic_calibrate.py` on your own hardware. An 8.6 mm RMS fit is not a
good enough basis for a torque-enabled test when the local procedure demonstrably
achieves 1.77 mm.

---

## 7. Verification of `arm6_calibration_zzq.yaml`

**Verdict: do not swap wholesale. One field in it is arguably more correct than ours;
the rest describes a different calibration session.**

Schema is compatible. `load_arm6_calibration` (`core/model.py`, around lines 50–80)
reads eight keys; zzq's file supplies all of them with correct lengths. Missing
`metadata.torque_scale_alpha` and `metadata.temperature_c` are documentation only.

### 7.1 The one genuine improvement

`first_moment_ratio` — zzq's values are close to 1.0 where ours are 1.2–1.35:

| Joint | Current | zzq |
|---|---|---|
| 1 | 1.265 | 1.0025 |
| 2 | 1.349 | 1.0088 |
| 3 | 1.228 | 1.0233 |

Our own file predicts exactly this. From `config/arm6_calibration.yaml`:

> ⚠️⚠️ 【此比值已烘进 arm6.urdf 与 arm6_tool.urdf 的连杆惯性参数】…
> 若按修正后的 URDF 重跑阶段 3，应当测得比值 ≈ 1.0

The correction was baked into the URDF, so a re-measurement against the corrected URDF
should return ≈1.0. zzq's numbers are that re-measurement, and they land where predicted.
That is a consistency check passing, and it is good evidence his fit is methodologically
sound even though the extrinsics one was not.

**This exposes a latent inconsistency in our checkout, independent of zzq.**
`phase4_gravity_compensation.py` computes its feedforward as (around line 107):

```python
return self.model.gravity(qc) * self.ratio
```

It multiplies URDF gravity by `first_moment_ratio`. If the URDF already carries the
correction — as our own file states in bold — then phase4 currently double-corrects by
roughly 26% on joint 1. I have not run phase4 to confirm the symptom, so treat this as a
code-reading finding that needs a bench check, not an established bug. It affects only
phase4 and phase5, never the SMC control path (§5b).

### 7.2 Why the rest should not come along

Zero offsets are from a different session and differ substantially:

| Joint | Current (°) | zzq (°) | Δ |
|---|---|---|---|
| 1 | 2.268 | 2.357 | 0.09 |
| 2 | −17.060 | −9.254 | **7.81** |
| 3 | −1.158 | 2.024 | **3.18** |
| 4 | −11.057 | −11.057 | 0.00 |
| 5 | −6.398 | −2.433 | **3.96** |

Joint 4 is bit-identical while joints 2, 3 and 5 differ by degrees. That pattern says
zzq inherited our joint-4 value and re-fitted the others — so these are two different
calibrations of the arm, not two views of one. Which is right depends on the physical
state of the hardware, which I cannot determine from here.

Zero offset feeds gravity compensation directly. An 7.8° error on joint 2, the
heaviest-loaded joint, is not a small perturbation.

**It breaks a test.** `test/core/test_arm6_model.py::test_unconverged_joints_are_flagged_not_hidden`
asserts joints 4 and 5 are both flagged unconverged. zzq's file marks joint 5 as
converged (`zero_offset_converged: [true, true, true, false, true, false]`), so the
assertion fails. His own note says joint 5 is the mean of three repeat fits, which is a
reasonable basis for upgrading the flag — but our file documents a *systematic drift*
across repeats that makes them non-independent. The two files disagree about whether
that drift was resolved. That disagreement should be settled before the flag changes,
not silently by a file swap.

Other differences, for completeness: `stiction_nm` joint 1 rises 72% (0.725 → 1.25) and
joint 5 falls 25%; `kinetic_friction_nm` joint 2 falls from 1.675 to 0.674;
`inertia_kgm2` joints 3–5 shift by 4–17%. `armature_kgm2` is identical across all six
joints in both files.

One coupling to be aware of: `config/smc_params_whole6.yaml` hardcodes
`friction.coulomb_nm: [0.725, 1.775, 0.625, 0.125, 0.100, 0.100]`, which matches our
current `stiction_nm` exactly. Swapping the calibration file would silently desynchronise
the simulation's friction model from the hardware's.

### 7.3 Recommendation

If you want the ratio fix, change that one field in the existing file and record why,
rather than replacing the whole file. That keeps the zero offsets, the converged flags,
and the caveats block that document how those numbers were obtained.

---

## 8. Open items before the hardware run

1. **Re-calibrate the Vicon extrinsics** (§6). The marker-cluster question is answered
   — they moved — so the current file is stale and zzq's is not a substitute. This gates
   the entire moving-base test. Run `--plan-only` first, from an unfolded posture.
2. **Confirm the Vicon stream is stable.** zzq's second-scale gaps are disqualifying.
3. **Decide on the `first_moment_ratio` / phase4 double-correction** (§7.1) if you
   intend to run phase4.
4. **Ignore zzq's three expected startup log lines** (§5a) — two of them cannot appear
   in this checkout.
5. ~~`vicon_server` default is stale.~~ **Done 2026-08-25** — the default is now
   `192.168.200.60` in `vicon_hold_demo.launch.py`, `vicon_floating_base.launch.py`,
   and `hw/vicon_link_check.py`.

---

## 9. Related documents

- `docs/RUNBOOK_NX.md` — general simulation and hardware procedures on the NX
- `docs/BASE_ACCELERATION_PIPELINE.md` — how `a_base` is derived, and why the VRPN
  twist/accel channels are currently inert
- `docs/CONTROL_ARCHITECTURE.md` — control law and frame conventions
- `README_zzq.md` (repository root) — the original handoff this document adapts
