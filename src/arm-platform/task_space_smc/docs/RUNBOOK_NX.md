# NX Runbook (jasyach fork)

Operating steps for running the 6-DOF A-L1-GAMMA task-space SMC on the Jetson Orin NX.

This document uses the **actual paths of this fork on the NX**, which differ from the
`/home/windylab/code/windylab_ws` hardcoded throughout the parent `README.md`. Copying
commands from there verbatim will fail with "no such file or directory".

| | Path |
|---|---|
| Control workspace | `~/jasyach_repo/windylab_arm_control_FanC` |
| Vicon workspace | `~/jasyach_repo/WindyLab-ViconPerception` |
| Vicon server | `192.168.200.60:3883` |
| Branch | `lagrange-smc` |

> Both Vicon addresses in the parent README section 5 (`172.16.20.21` and
> `192.168.200.78`) are stale. Use `192.168.200.60`.

---

## 1. One-time setup

### 1.1 `~/.bashrc` helper functions

There are multiple working copies on the NX. **Do not** put any `install/setup.bash`
into `.bashrc`: sourcing two workspaces into the same shell makes `manipulator`
resolve to whichever was sourced last. This produces no error — it silently runs the
wrong code. Bind the domain ID and the overlay together in a function, and call it
once per terminal.

```bash
cat >> ~/.bashrc <<'EOF'

# --- windylab arm workspaces ---
smc_hw()  { export ROS_DOMAIN_ID=42
            source /opt/ros/humble/setup.bash
            source ~/jasyach_repo/windylab_arm_control_FanC/install/setup.bash
            cd ~/jasyach_repo/windylab_arm_control_FanC; }

smc_sim() { export ROS_DOMAIN_ID=77
            source /opt/ros/humble/setup.bash
            source ~/jasyach_repo/windylab_arm_control_FanC/install/setup.bash
            cd ~/jasyach_repo/windylab_arm_control_FanC; }

smc_vicon() { smc_hw
              source ~/jasyach_repo/WindyLab-ViconPerception/install/setup.bash; }
EOF
```

The quotes in `<<'EOF'` are required, otherwise `~` and `$` are expanded at write time.

**Any fixed `export ROS_DOMAIN_ID=...` must also be removed from `.bashrc`.** demo2
uses four terminals. If one terminal misses the domain ID, that node sits alone on a
different domain: DDS raises no error, it simply never discovers the others. The
symptom is "`/joint_states` never arrives" or "the arm ignores torque commands", and
it is hard to trace.

Convention: hardware `42`, simulation `77`.

### 1.2 Build the control workspace

All four packages are vendored in the repo. There are no submodules, and no separate
`git clone` of `serial` is needed despite what the root README implies.

```bash
cd ~/jasyach_repo/windylab_arm_control_FanC
source /opt/ros/humble/setup.bash

colcon build --packages-select dummy_interface serial dummy_description manipulator \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

**`--symlink-install` is mandatory, not a preference.** With it, edits to
`task_space_smc/**/*.py` and `config/*.yaml` take effect on the next launch. Without
it you run a stale copy from the install tree — **the change has no effect and nothing
reports an error**. Verify:

```bash
ls -l install/manipulator/lib/manipulator/whole_task_smc_ros_runner.py
# must show  -> .../src/arm-platform/task_space_smc/hw/whole_task_smc_ros_runner.py
```

Rebuild is required for: C++ sources, `dummy_interface` message definitions,
`CMakeLists.txt`, and **launch files** (launch files are copy-installed). Python and
YAML changes need no rebuild.

### 1.3 Build the Vicon workspace

`vicon_floating_base.launch.py` starts the `vrpn_listener` node, which lives in this
workspace rather than in the control repo.

VRPN here is a **plain CMake package, not an ament package**, and the `vrpn_listener`
CMakeLists has an upstream linking bug. Both must be handled or the build fails.

**1. The two packages must be built separately.** `vrpn_listener` locates VRPN through
`cmake/FindVRPN.cmake`, a plain find module rather than a declared dependency. colcon
cannot see that edge in the dependency graph, so a single `colcon build` runs both in
parallel and `vrpn_listener` searches for a library that is still compiling.

**2. The install prefix must be passed explicitly.** Because VRPN is not an ament
package, `source install/setup.bash` does **not** add `install/VRPN` to
`CMAKE_PREFIX_PATH`. Sourcing between the two builds accomplishes nothing; the find
modules fall back to their hardcoded `/usr/local` paths and never look in the workspace.

**3. `vrpn_listener/CMakeLists.txt` needs a one-line patch.** It links the bare token
`vrpn`, which the linker resolves only if `libvrpn` sits on a default search path. With
VRPN built into a colcon prefix the link fails with `/usr/bin/ld: cannot find -lvrpn`.
`FindVRPN.cmake` already resolves the absolute path into `VRPN_LIBRARIES` (around lines
133-135), together with `QUATLIB_LIBRARIES` and the pthread flag — using it also
pre-empts the equivalent `-lquat` failure. The bug is invisible on machines that have
VRPN installed to `/usr/local`, which is where the find module's fallback paths point.

```bash
cd ~/jasyach_repo/WindyLab-ViconPerception
source /opt/ros/humble/setup.bash

# 3. patch the bare-name link (one line, carry this across re-clones)
sed -i 's/target_link_libraries(vrpn_listener vrpn)/target_link_libraries(vrpn_listener ${VRPN_LIBRARIES})/' \
  src/vrpn_listener/CMakeLists.txt

# 1. build VRPN first, on its own
colcon build --packages-select VRPN

# 2. then vrpn_listener, with the prefix passed explicitly
colcon build --packages-select vrpn_listener \
  --cmake-args -DVRPN_ROOT_DIR=$PWD/install/VRPN \
               -DQUATLIB_ROOT_DIR=$PWD/install/VRPN
```

`FindVRPN.cmake` feeds `VRPN_ROOT_DIR` into `HINTS` with
`PATH_SUFFIXES include, lib, lib64` (around lines 55-75), matching what VRPN installs:
`install/VRPN/include/vrpn_Connection.h` and `install/VRPN/lib/libvrpn.a`.

> The `sed` patch edits a repository cloned from elsewhere. A fresh clone reintroduces
> the bug, so keep it on a local branch or push it upstream.

Note that VRPN is **not** available as an Ubuntu package — `libvrpn-dev` does not
exist in any Ubuntu repository. The bundled `src/vrpn` is the only source.

#### If it still reports `Could NOT find VRPN`

**Clear the build directory first.** CMake caches *negative* results: once
`VRPN_INCLUDE_DIR-NOTFOUND` is written to `CMakeCache.txt`, that search is never
retried, regardless of what gets installed or which flags are passed afterward. A
configure step finishing in well under a second is the tell.

```bash
rm -rf build/vrpn_listener install/vrpn_listener
```

Then confirm what is actually on disk and which find modules are available:

```bash
find install/VRPN -name 'vrpn_Connection.h' -o -name 'libvrpn*' \
                  -o -name 'quat.h' -o -name 'libquat*'
ls src/vrpn_listener/cmake/
```

A healthy tree shows `include/vrpn_Connection.h`, `include/quat.h`, `lib/libvrpn.a`,
and `lib/libquat.a`, with both `FindVRPN.cmake` and `Findquatlib.cmake` present.

`FindVRPN.cmake` requires three things to pass (around lines 125-131):
`VRPN_INCLUDE_DIR`, `VRPN_LIBRARY`, and `QUATLIB_FOUND` — that last one comes from a
**separate** `find_package(quatlib)` at around line 105, so a correct VRPN install
still fails if quatlib is not located.

If the `_ROOT_DIR` variable names turn out to be wrong for this copy of the modules
(`grep ROOT_DIR src/vrpn_listener/cmake/Findquatlib.cmake` gives the real name), the
environment route reaches both modules without needing either name:

```bash
export CMAKE_PREFIX_PATH=$PWD/install/VRPN:$CMAKE_PREFIX_PATH
rm -rf build/vrpn_listener install/vrpn_listener
colcon build --packages-select vrpn_listener
```

Verify with `ros2 pkg list | grep vrpn`.

---

## 2. Simulation

One terminal, one command. `task_space_smc_whole6_sim.launch.py` starts everything
itself: `student_arm_node` (`arm_type: torque_sim`), `robot_state_publisher`, the
runner, and RViz.

```bash
smc_sim
ros2 launch manipulator task_space_smc_whole6_sim.launch.py \
  trajectory_shape:=hold duration_sec:=12 rviz:=true
```

In RViz the fixed frame is `world`; the green line is the desired path and the thicker
red line is the actual path. Each run writes timestamped CSV, JSON, and PNG into
`task_space_smc/data/`.

### 2.1 Choosing `trajectory_shape`

The three shapes vary along **two independent axes** — what the end-effector reference
does, and what the base does:

| shape | EE reference | base | plant model | sim RMS |
|---|---|---|---|---|
| `hwtest` | 4 cm vertical circle, +x side | **fixed** | ideal | 0.0041 mm |
| `whole` | five-station full task | **fixed** | ideal | 0.0060 mm |
| `hold` | **fixed point in world frame** | sine + gust | all non-idealities | 1.71 mm |

The parent README table labels `whole` as having analytic base swing, but that applies
to the 7-DOF entry point. The 6-DOF sim launch pins `MB_BASE_RADIUS` and
`MB_BASE_Z_AMP` to `0.0` (`task_space_smc_whole6_sim.launch.py`, around lines 30-31),
so no shape gets analytic swing on this path.

**In the 6-DOF simulation, base motion comes only from `simulation_injection`, and of
the three shapes only `hold` enables it** (`smc_params_whole6.yaml` sets the global
default to `false` around line 70; only the `hold` profile overrides it to `true`). So
for studying end-effector stabilization under base disturbance, `hold` is not merely
the recommended starting point — it is the only one of the three that exercises it.

### 2.2 The 1.71 mm in `hold` is not all base disturbance

`simulation_injection` enables far more than base motion:

- Coulomb and viscous joint friction (`coulomb_nm` uses this arm's measured stiction)
- Backlash torque deadband
- 12 ms motor drive delay, torque scale error, bias, white noise, and ripple
- 0.08 kg end-effector payload mass error
- A gust every 4 s lasting 0.8 s
- Base estimator noise and 20 ms latency (motion capture is not an ideal sensor)

So the 400x gap between `hwtest` at 0.0041 mm and `hold` at 1.71 mm is the **sum of
all non-idealities**, not the contribution of base disturbance alone.

The config does make the comparison deliberately clean (`smc_params_whole6.yaml`,
around line 151): everything except the base — starting posture, gains, limits — is
item-for-item identical to `hwtest`. To isolate base disturbance specifically, run
`hold` twice while toggling `simulation_injection.enabled`, or zero only
`translation_amplitude_m` and `rpy_amplitude_deg` while leaving the friction and motor
terms active.

---

## 3. Hardware demo2: end-effector hold under a floating base

Four terminals, in strict order. Each starts with `smc_hw` (the Vicon one uses
`smc_vicon`).

### 3.0 Three safety constraints that cannot be worked around

1. **The protocol has no enable/disable frame. There is no software E-stop channel —
   an emergency stop means cutting power physically.**
2. **The arm locks its current pose on power-up and cannot be moved by hand.**
3. Once SMC starts, the same torque path takes over from the first frame, merging the
   current pose into the trajectory via handover reference-offset decay.

### 3.1 Clear leftovers and check the serial port

```bash
pkill -f "ros2 launch manipulator"; pkill -f student_arm_node
pkill -f robot_state_publisher; pkill -f vrpn_listener
pkill -f vicon_floating_base_bridge
sleep 2
ps -ef | grep -E 'ros2 launch|student_arm_node|vrpn' | grep -v grep   # must be empty

ls -l /dev/serial/by-id/
```

A leftover `robot_state_publisher` becomes a second publisher on `/joint_states` and
produces physically impossible readings. Device numbers change across replug, so
confirm through `by-id`; when installed on the NX the port may be `/dev/ttyTHS3`
rather than `/dev/ttyUSB0`. Confirm your user is in the `dialout` group.

### 3.2 Read-only posture check

```bash
smc_hw
python3 src/arm-platform/task_space_smc/hw/pose_check_readonly.py \
  --port /dev/ttyUSB0 --seconds 3
```

This script is structurally incapable of sending bytes.

### 3.3 Terminal A: start the driver (leave it running throughout)

```bash
smc_hw
ros2 launch manipulator student_arm6.launch.py arm_type:=a_l1 \
  port_name:=/dev/ttyUSB0 control_rate_hz:=200.0 \
  enable_direct_effort_command:=true max_velocity:=0.5 use_rviz:=false
```

- Without `enable_direct_effort_command:=true` the SMC has no torque path.
- Use `max_velocity:=0.2` on a first run.
- 200 Hz is the torque-mode ceiling; position mode caps at 100 Hz. Do not conflate them.

### 3.4 Terminal B: move joint5 out of the wrist singularity

```bash
smc_hw
python3 src/arm-platform/task_space_smc/hw/wrist_prepose.py \
  --target-deg 40 --duration 20 --settle 3
```

### 3.5 Terminal B: motion capture

```bash
smc_vicon
ros2 launch manipulator vicon_floating_base.launch.py \
  server:=192.168.200.60 port:=3883 \
  zero_on_first_pose:=true prefer_vicon_angular_accel:=true
```

This launch starts both `vrpn_listener` (Vicon workspace) and
`vicon_floating_base_bridge.py` (manipulator). Sourcing two overlays here is
legitimate — they are different packages and cannot shadow each other.

**Confirm both streams are live before continuing:**

```bash
ros2 topic hz /vrpn/arm_base/pose
ros2 topic hz /vrpn/arm_ee/pose
```

### 3.6 Terminal C: start the demo

```bash
smc_hw
ros2 launch manipulator vicon_hold_demo.launch.py \
  hardware_enabled:=true duration_sec:=86400 \
  data_dir:=$HOME/jasyach_repo/windylab_arm_control_FanC/src/arm-platform/task_space_smc/data/vicon_hold_manual
```

Without `hardware_enabled:=true` the adapter emits identical zeros and the arm never
moves. This launch starts only `robot_state_publisher`, the runner, and RViz — it
deliberately does not start the hardware node, so the serial port is never opened twice.

### 3.7 Shutdown (order cannot be reversed)

```
1) Ctrl-C the demo    (terminal C)
2) Ctrl-C the Vicon   (terminal B)
3) Ctrl-C the driver  (terminal A, last)
```

---

## 4. Safety watchdog and repositioning mid-session

### 4.1 What happens on a trip

The watchdog runs during both the approach and tracking phases. Either criterion
stops tracking:

| Criterion | Threshold |
|---|---|
| Joint velocity | 1.2x the `[3,3,3,5,5,5]` rad/s envelope |
| End-effector position error | > 50 mm |

The criteria consume **measured** values, not estimator output — the estimator is the
thing under test on this path, and the fallback must not have a second point of
failure. Search logs for `安全看门狗触发`.

On a trip the runner freezes the joint vector at that instant and publishes a position
hold with `p=30.0, d=1.0, current=0` (`whole_task_smc_ros_runner.py`, around lines
1496-1506). After `watchdog_hold_sec` (default **1.0 s**) it writes its summary and
**exits** (around lines 2278-2286). What holds the arm after that is no longer the
runner but `student_arm_node` sitting on its last received command — which is why the
arm stays stiff even once the SMC process is gone.

### 4.2 Why it does not fall back to gravity compensation

The watchdog's trip conditions — end-effector error over 50 mm, joint overspeed — have
a model error as one of their more likely causes. Gravity-compensation float is held up
*entirely* by the gravity feedforward `tau = ratio * g(q)`. Falling back to float on a
fault that may itself be caused by a wrong model would let the arm collapse under its
own weight. A position lock requires no model to be correct. Given that the protocol
has no enable/disable frame and there is no software E-stop, the conservative default
is the right one.

### 4.3 Moving to a better posture mid-session

The float capability does exist. `phase4_gravity_compensation.py` progressively strips
position feedback across five levels; at **L4 (float)**, `kp_frac=0.00, kd_frac=0.20`,
the arm is carried by gravity feedforward alone and can be pushed by hand.

However, phase4 is an **acceptance sweep, not a mode**: it walks L0 through L4 (3 s
settle plus 4 s hold per level) and then exits. There is currently **no** persistent
free-drive tool in this repository.

So repositioning mid-session goes through a command rather than by hand:

```bash
smc_hw
python3 src/arm-platform/task_space_smc/hw/full_prepose.py \
  --target-rad="0.4571,-0.1430,-0.9966,0.7027,-0.6168,0.0178" \
  --duration 30 --settle 3
```

> Before building a persistent free-drive mode, confirm that L4 converges on this
> particular arm. A diverging L4 residual means the gravity feedforward cannot hold the
> arm, and removing the position loop will let it sag. Also, with no disable frame, the
> last command a script published stays in effect after that script exits.

---

## 5. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `/joint_states` never arrives; arm ignores commands | One terminal on a different domain | Check `echo $ROS_DOMAIN_ID` in every terminal: all 42 (hardware) or all 77 (sim) |
| `colcon build` reports `Could NOT find VRPN` | Parallel build, or a cached `NOTFOUND` | See [1.3](#13-build-the-vicon-workspace) |
| Python/YAML edits have no effect and no error | Built without `--symlink-install` | Run the verification command in [1.2](#12-build-the-control-workspace), then rebuild |
| `manipulator` resolves to another workspace | Two `install/setup.bash` sourced in one shell | Open a new terminal, run `smc_hw` once |
| `/joint_states` shows physically impossible values | Leftover `robot_state_publisher` acting as a second publisher | See [3.1](#31-clear-leftovers-and-check-the-serial-port) |
| `import pinocchio` picks the wrong build | `PYTHONPATH` in `.bashrc` puts `/usr/local` ahead of ROS | `python3 -c "import pinocchio; print(pinocchio.__file__)"` should resolve under `/opt/ros/humble` |
| Launch argument changes do not take effect | Launch files are copy-installed | Rebuild `manipulator` |

---

## 6. Relationship to the other documents

- Control law, ROS computation graph, frame conventions →
  [`CONTROL_ARCHITECTURE.md`](CONTROL_ARCHITECTURE.md)
- Package structure, config table, demo1 (fixed-base trajectory tracking), calibration
  phases → [`../README.md`](../README.md)
- acados / GRAMPC deployment → section 6 of the repository root `README.md`.
  **`task_space_smc` uses neither solver** — `grep -rn "acados\|grampc" task_space_smc/`
  returns nothing, and `manipulator`'s `CMakeLists.txt` has no corresponding
  `find_package`. They serve only the LL100 and 7-DOF NMPC demos.
