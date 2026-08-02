# 浮动基座 7DoF 机械臂任务空间 SMC 项目交接

> 交接日期：2026-08-02
> 工作区：`/home/windylab/code/windylab_ws`
> 当前分支：`lagrange-smc`
> 基线提交：`5ce9d49`（远端 `origin/position-only-baseline`）
> 重要：本文描述的大部分 SMC 工作尚未提交，必须先阅读“工作区保护”一节。

## 1. 项目目标

在 `src/arm-platform/` 下实现一套独立于现有 NMPC 的控制链：

- 浮动基座 + A-L1-GAMMA 7DoF 机械臂；
- 世界系末端位置和姿态轨迹跟踪；
- 任务空间拉格朗日滑模控制（SMC）；
- 四元数及严格 SO(3) 对数姿态误差；
- 基座运动补偿；
- 动力学一致阻尼广义逆；
- 零空间 PD；
- Pinocchio 自由基座动力学；
- 输出七关节力矩/力矩前馈指令；
- 在 ROS 2 Humble 和 RViz 中显示基座晃动、机械臂补偿、期望轨迹和实际轨迹；
- `whole` 轨迹位置 RMS 小于 1 mm、峰值小于 2 mm。

目标场景称为“鸡头稳定”：基座运动时，末端在世界系中保持稳定或跟踪世界系轨迹。

原始完整需求保存在 Codex 附件：

```text
/home/windylab/.codex/attachments/
  fbae0c61-2d0c-4818-9db0-1a8e916ed694/pasted-text.txt
```

## 2. 当前结论

### 2.1 已实现

1. 建立 Pinocchio `JointModelFreeFlyer` 模型：`nq=14`、`nv=13`。
2. 使用 `arm.urdf`、末端 `link7`、关节 `joint1..joint7`。
3. 实现严格四元数/SO(3) 姿态误差及其解析导数。
4. 实现位置与姿态任务空间 SMC。
5. 实现任务尺度、自适应阻尼、动力学一致广义逆和零空间 PD。
6. 使用完整自由基座 RNEA 计算机械臂七关节力矩。
7. 新增 `/student/joint_effort_command` 直力矩接口，绕过 `SmoothPositionController`。
8. 新增 C++ `TorqueSimArm`，使 `student_arm_node` 能消费 `MotorControl.current` 并进行动力学仿真。
9. 新增 `/floating_base/truth`，把规定基座状态传给 `TorqueSimArm`。
10. 仿真采用“每帧力矩只推进一步”的显式锁步，避免两个异步 500 Hz 定时器重复执行同一帧力矩。
11. RViz 能显示绿色期望轨迹、较粗红色实际轨迹、移动基座和机械臂。
12. 轨迹按 stroke 分段，不连接落笔间隙或不同字母。
13. 启动后先显示期望轨迹，再从偏置关节位姿平滑靠近起点；靠近过程不画红线。
14. CSV、JSON 和 PNG 报告写入 `task_space_smc/data/`。
15. 原 MPC/阻尼 IK 位置链保留，默认仍为：

```text
/student/joint_command
  -> student_arm_node
  -> SmoothPositionController
  -> SimArm.position
```

`enable_direct_effort_command` 默认是 `false`，只有 SMC launch 显式开启。

### 2.2 当前架构的准确数学定位

当前是：

> 完整自由基座多刚体模型建模 + 规定基座运动下的机械臂约化控制/约化仿真。

Pinocchio 中存在完整状态、完整 `13x13` 惯性矩阵和完整 RNEA：

```text
q_f = [p_B^W, q_B^W, q_arm]       (14)
v_f = [nu_B, qd_arm]              (13)
M_f = [[M_BB, M_Ba], [M_aB, M_aa]]
```

控制力矩等价于：

```text
tau = M_aa qdd_cmd + M_aB a_B + h_a
```

仿真关节加速度等价于：

```text
qdd = solve(M_aa, tau - M_aB a_B - h_a)
```

但是系统没有求解基座动力学行：

```text
M_BB a_B + M_Ba qdd + h_B = F_B + d_B
```

基座轨迹是外部规定量，机械臂反作用不会反过来改变基座。这里的 `whole` 是完整绘图轨迹名称，不代表已经实现无人机与机械臂联合 whole-body wrench control。

## 3. 当前 ROS 执行链

主仿真模式 `mode:=simulation`：

```text
whole_task_smc_ros_runner.py
  |  tau[7] in MotorControl.current, p=d=0
  v
/student/joint_effort_command
  v
student_arm_node
  v
ArmPlatform::ExecuteDirectMotorCommand
  v
TorqueSimArm::SetMotorCommand + Step
  |  Pinocchio reduced prescribed-base dynamics
  v
/joint_states
  +-> robot_state_publisher -> RViz
  +-> whole_task_smc_ros_runner.py feedback
```

基座链：

```text
whole_task_smc_ros_runner.py
  -> /floating_base/truth (FloatingBaseState)
  -> student_arm_node
  -> TorqueSimArm::SetBaseState
```

运行时 ROS 图已经验证：

- `/joint_states` 只有一个 publisher：`/student_arm_node`；
- `/student/joint_effort_command` 为 runner 发布、student node 订阅；
- `/floating_base/truth` 为 runner 发布、student node 订阅。

## 4. 三种运行模式

### `simulation`（当前主模式）

- 自动启动 `student_arm_node`；
- `arm_type=torque_sim`；
- `step_on_effort_command=true`；
- SMC 与被控对象通过 ROS 消息连接；
- 用于 RViz 和当前正式无扰动验收。

### `internal_simulation`（旧回归模式）

- 不启动 `student_arm_node`；
- runner 内部 Python plant 自己积分；
- 当前只有这个模式实现了 payload、friction、backlash、motor delay/error 和 joint gust 等完整注入；
- 仅用于回归和旧消融，不应冒充共享执行链结果。

### `hardware`（尚未具备直接上机条件）

- launch 不会自动启动真实 `student_arm_node`，防止误开串口；
- runner 等待外部 `/joint_states` 并发布力矩命令；
- 目前 whole runner 没有订阅真实无人机基座状态，在该模式仍会使用解析 `base_sample()`；
- 因此当前 `hardware` 不是可直接飞行/实机验收的完整模式。

## 5. 关键文件地图

### 数学与控制

- `task_space_smc/model.py`：自由基座 Pinocchio 模型、质量矩阵、RNEA、正/逆动力学。
- `task_space_smc/quaternion_math.py`：四元数、SO(3) log、左雅可比及导数。
- `task_space_smc/pose_error.py`：严格位姿误差。
- `task_space_smc/task_space_smc.py`：位置和姿态 SMC。
- `task_space_smc/redundancy_resolution.py`：尺度、阻尼、动力学一致广义逆、零空间。
- `task_space_smc/trajectory_reference.py`：世界系末端参考和规定基座运动。
- `task_space_smc/actuator_adapter.py`：力矩、力矩变化率、超时和 enable 安全门。

### ROS、仿真与可视化

- `task_space_smc/whole_task_smc_ros_runner.py`：主控制循环、记录、Marker、模式选择。
- `launch/task_space_smc_whole.launch.py`：启动 student node、runner、RSP、RViz。
- `scripts/run_task_space_smc_whole.sh`：清理错误 overlay、检查 prefix、启动演示。
- `config/task_space_smc_whole.rviz`：RViz 配置。
- `task_space_smc/config/smc_params_whole.yaml`：当前 whole 参数和注入配置。
- `task_space_smc/plot_whole_task_smc.py`：CSV 报告作图。
- `task_space_smc/tune_whole_task_smc.py`：离线确定性调参。
- `task_space_smc/ablate_simulation_injection.py`：离线注入消融。

### C++ 执行链

- `include/manipulator/robotics/arm/torque_sim_arm.h`
- `src/robotics/arm/torque_sim_arm.cc`
- `src/robotics/arm/arm_factory.cc`
- `include/manipulator/robotics/arm/abs_arm.h`
- `include/manipulator/robotics/arm_platform.h`
- `src/robotics/arm_platform.cc`
- `include/manipulator/ros_nodes/student_arm_node.h`
- `src/ros_nodes/student_arm_node.cc`
- `include/manipulator/common_types.h`

### 消息与测试

- `src/dummy-interface/msg/FloatingBaseState.msg`
- `src/dummy-interface/msg/MotorControl.msg`
- `test/test_torque_sim_arm.cc`
- `test/test_quaternion_math.py`
- `test/test_pose_error.py`
- `test/test_floating_base_model.py`
- `test/test_dynamic_consistent_inverse.py`
- `test/test_nullspace.py`
- `test/test_smc_offline.py`
- `test/test_smc_plot_segmentation.py`

### 调研与历史文档

- `task_space_smc/CODEX_DISCOVERY_REPORT.md`：源码接口调研，仍值得先读。
- `task_space_smc/README.md`：当前启动说明基本有效。
- `task_space_smc/VALIDATION_RESULTS.md`：部分内容已过时，见后文。
- `task_space_smc.zip`：旧小型归档，不保证包含最新锁步和共享链修改，不要作为源码真相。

## 6. 当前参数与硬件假设

控制频率：

```text
500 Hz
```

电机分配：

```text
joint1..joint3: DM-J4340 / DM4340-48V
joint4..joint7: RobStride RS05
```

当前默认持续力矩限幅：

```text
[9.0, 9.0, 9.0, 1.6, 1.6, 1.6, 1.6] Nm
```

已知峰值规格仅作为参考，当前默认控制不使用：

```text
DM-J4340: about 27 Nm peak
RS05:     about 5.5 Nm peak
```

`MotorControl.current` 在仓库上层控制抽象中被当作 Nm 量级力矩/力矩前馈：

```text
MotorControl.current
  -> DMMotor::UpdateCommand
  -> DMMotor::SetCurrent
  -> ProtocolV1::SetCurrent
  -> ProtocolV1::Pop command 0x12
```

但固件端 `0x12` 的真实单位、缩放和模式未由仓库文档证明。源代码注释称 A，上层重力控制器却直接写入 Nm，且限幅使用 `rated_torque`。在台架验证前禁止把当前高增益 SMC 直接接真机。

## 7. 坐标系约定与已知语义风险

### 末端

- 位姿在世界系；
- 末端 twist、classical acceleration、Jacobian 使用 `pin.LOCAL_WORLD_ALIGNED`；
- 内部四元数 `[w,x,y,z]`；
- ROS 四元数 `[x,y,z,w]`，接口处显式转换。

### 自由基座

Pinocchio 自由关节切空间实际使用局部/机体系速度：

```text
nu_B = [R_WB^T p_dot_W, omega_B]
```

`trajectory_reference.py` 正确计算了 `v_local`、`omega_body` 以及相应局部广义加速度。

但是当前变量名 `velocity_lwa`、`acceleration_lwa` 和消息 `header.frame_id=world` 容易让人误以为六维速度/加速度都在世界系。这是迁移无人机前必须修复的协议语义问题。

不要把原始 IMU specific force 直接当 `a_B`。RNEA 已在 `h` 中处理重力，IMU 需要先恢复运动学加速度并完成坐标变换，否则会重复或漏掉重力。

## 8. 已验证结果

### 8.1 正式共享链 whole 周期

可信验收文件：

```text
task_space_smc/data/whole_task_smc_summary_20260730_230225.json
task_space_smc/data/whole_task_smc_trajectory_20260730_230225.csv
task_space_smc/data/whole_task_smc_report_20260730_230225/
```

配置：

```text
mode=simulation
simulation_injection.enabled=false
control_rate_hz=500
samples=36520
whole period about 73.039 s
```

结果：

```text
position RMS  = 9.655746766e-06 m = 0.00966 mm
position peak = 1.588129168e-04 m = 0.15881 mm
orientation RMS  = 0.00194199 rad
orientation peak = 0.02721249 rad
pass_1mm_rms_2mm_peak = true
```

正式周期观察到的各关节最大命令力矩：

```text
J1 2.5425, J2 4.2023, J3 3.3513, J4 0.6972,
J5 0.2334, J6 0.0214, J7 0.000086 Nm
```

该次运行没有触发持续力矩限幅。

### 8.2 测试状态

已验证：

```text
colcon build: passed
Python functional tests: 11 passed
TorqueSimArm gtest: passed
ROS helper smoke test: passed
ROS topic graph: passed
```

全包 `colcon test` 并非绿色：仓库现有 lint 会递归扫描 `dist/build` 等生成目录和大量旧文件，产生数千条既有 flake8、pep257、uncrustify、xmllint 问题。不要为了本任务批量格式化整个仓库。

### 8.3 不要当作当前结论的旧结果

以下失败文件是显式锁步修复前的调试记录：

```text
whole_task_smc_summary_20260730_225335.json  # 100 Hz async, RMS about 0.428 m
whole_task_smc_summary_20260730_225518.json  # 500 Hz async, RMS about 0.440 m
```

失败原因是 C++ plant 用独立 wall timer 重复执行旧力矩，而 runner 假设每帧只推进一次。不要据此判断当前控制器不稳定。

`VALIDATION_RESULTS.md` 中“10 passed”和旧 `mode=simulation` 内部 plant 描述也已过时；当前是 11 个 Python 测试，并以共享 `student_arm_node/TorqueSimArm` 为主模式。

旧 ablation 目录中也混有锁步修复前、旧力矩限幅 `[24,24,24,5,5,4.5,...]` 或短时离线结果。后续必须在统一架构和当前 `[9,9,9,1.6,...]` 限幅下重新生成正式消融表。

## 9. 启动、构建与测试

### 9.1 用户正常启动（默认打开 RViz）

```bash
cd /home/windylab/code/windylab_ws

src/arm-platform/scripts/run_task_space_smc_whole.sh \
  mode:=simulation \
  start_delay_sec:=3.0 \
  duration_sec:=0.0 \
  hardware_enabled:=false
```

参数含义：

- `mode:=simulation`：共享 student node + TorqueSimArm 链；
- `start_delay_sec:=3.0`：显示期望路径并平滑靠近起点；
- `duration_sec:=0.0`：完整 whole 周期；
- `hardware_enabled:=false`：不允许真实硬件输出；仿真模式内部会正常启用虚拟力矩；
- `rviz` 默认 `true`。

runner 完成并写出 summary 后，student node、RSP 和 RViz 仍保持运行，用户可用 `Ctrl-C` 退出。

### 9.2 修改 C++ 或消息后强制重建

```bash
cd /home/windylab/code/windylab_ws

WINDYLAB_SMC_REBUILD=1 \
src/arm-platform/scripts/run_task_space_smc_whole.sh \
  mode:=simulation \
  start_delay_sec:=1.0 \
  duration_sec:=1.0 \
  hardware_enabled:=false \
  rviz:=false
```

helper 会清除可能把包指向 `/home/windylab/install` 的旧 overlay，并确认：

```text
ros2 pkg prefix manipulator
== /home/windylab/code/windylab_ws/install/manipulator
```

### 9.3 手动构建

```bash
cd /home/windylab/code/windylab_ws
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select dummy_interface manipulator \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON

source install/setup.bash
```

### 9.4 功能测试

```bash
cd /home/windylab/code/windylab_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 -m pytest -q src/arm-platform/test

./build/manipulator/test_torque_sim_arm
```

不要用 `PYTHONPATH=src/arm-platform` 覆盖整个 ROS `PYTHONPATH`，否则可能找不到 `/opt/ros/humble/.../pinocchio`。如需添加路径，应保留原值。

### 9.5 短时冒烟

```bash
src/arm-platform/scripts/run_task_space_smc_whole.sh \
  mode:=simulation \
  start_delay_sec:=1.0 \
  duration_sec:=1.0 \
  hardware_enabled:=false \
  rviz:=false
```

验收：500 个正式样本左右，summary 中 `pass_1mm_rms_2mm_peak=true`。

## 10. 工作区保护

当前工作区非常脏，且不是所有改动都属于 SMC：

- SMC、TorqueSimArm、launch、消息和测试大多仍是 untracked/modified；
- `moving_base_uncertainty*`、`base_trust_estimator.py`、`move_base_circle_mpc_ik_demo.py`、B4 测试等包含另一条 NMPC/B4 工作流改动；
- `task_space_smc/data/` 约 389 MB，包含调试、失败、消融和正式数据；
- `task_space_smc.zip` 不是可信最新版。

下一位 AI 必须遵守：

1. 不执行 `git reset --hard`、`git checkout -- .` 或大范围 clean。
2. 不删除或回滚未理解的 B4/NMPC 文件。
3. 不把 389 MB 数据目录整体加入提交。
4. 提交前用 `git status --short` 和逐文件 diff 确认范围。
5. 最好先建立新分支，并将 SMC 代码、测试、文档与实验数据分开提交。
6. 不修改现有 NMPC 算法来“顺手统一”架构，原始需求明确要求 SMC 独立。

建议提交分组：

```text
1. dummy_interface FloatingBaseState message
2. AbsArm/ArmPlatform/student_arm_node direct-effort API
3. TorqueSimArm backend and gtest
4. task_space_smc Python module and tests
5. whole launch/helper/RViz/config/docs
6. selected small validation summaries only (optional)
```

## 11. 未完成工作（优先级顺序）

### P0：统一共享链中的扰动注入

现状：

- `mode=simulation` 的 `TorqueSimArm` 只接收基座真实状态；
- runner 可对控制器输入应用 base estimator noise/delay；
- payload、friction、backlash、motor error/delay、joint gust 只在 `internal_simulation` 生效；
- runner 已明确打印这一限制。

下一步：

1. 为 `TorqueSimArm` 增加仿真参数结构或专用 disturbance model；
2. 在 C++ plant 中计算 `tau_actual`；
3. 将 payload、摩擦、齿隙、电机增益/偏置/噪声/纹波/延迟、gust 迁移到共享 backend；
4. 在 `/joint_states.effort` 或专用诊断消息中区分 `tau_command`、`tau_actual`、`tau_disturbance`；
5. 重跑当前限幅下的 one-at-a-time 和 all-enabled ablation；
6. `simulation_injection.enabled=false` 必须继续保持当前无扰动效果。

不要在完成共享注入前继续对旧 internal plant 的失败结果盲目调参。

### P0：固定基座状态消息的坐标契约

建议不要继续使用含糊的 `velocity_lwa` 名称。至少明确：

```text
pose:          p_WB, q_WB
base twist:    Pinocchio free-flyer LOCAL/body tangent
base accel:    derivative of the same LOCAL tangent, excluding gravity
EE quantities: LOCAL_WORLD_ALIGNED
```

可选方案：

- 扩展消息增加 `pose_frame`、`twist_frame`；
- 或在 ROS 边界统一接受标准 odometry 语义，再在 adapter 内转换成 Pinocchio LOCAL；
- 增加世界系到机体系速度/加速度转换单元测试；
- 增加 IMU specific force 到运动学加速度的测试。

### P0：确认真机协议 `0x12`

必须从驱动器/主控板文档或台架实验确认：

- `0x12` 是 A、Nm、MIT `t_ff` 还是归一化量；
- DM-J4340 和 RS05 是否经过同一块主控板统一缩放；
- 正负方向；
- 连续/峰值限制及峰值持续时间；
- Kp=Kd=0 时是否允许纯力矩；
- watchdog、enable、掉线后的驱动器行为。

推荐先写低限幅单关节 probe，使用外部力矩传感器或已知力臂砝码标定。没有这一步，不得声称 real torque control 已完成。

### P1：真实无人机基座状态适配器

当前没有完整来源提供：

```text
p_WB, q_WB, v_B, omega_B, a_B, alpha_B
```

需要实现 PX4/MAVROS/状态估计器 adapter：

1. 同步飞控、IMU、里程计和关节反馈时间戳；
2. 统一 ENU/NED、FLU/FRD；
3. 转成 Pinocchio free-flyer LOCAL tangent；
4. 估计并滤波线/角加速度；
5. 处理延迟、丢包和 stale state；
6. 替换 hardware 模式中的解析 `base_sample()`。

### P1：真实力矩模式安全链

- 增加明确的硬件 launch，默认仍禁用非零力矩；
- 启动前检查串口、驱动模式、反馈、急停和基座状态；
- 低增益、低限幅、单关节、机械臂固定台架逐步测试；
- 测量串口是否能稳定支持 500 Hz；必要时降低频率并重新做离散控制设计；
- Python ROS runner 不是硬实时实现，最终应考虑 C++/ros2_control 或实时线程；
- 超时后应由驱动层真正归零，而不只依赖上层 ROS timer。

### P2：无人机-机械臂联合 whole-body 控制（如确实需要）

当前规定基座方法适合“飞控稳定基座，机械臂补偿已测基座运动”。如果机械臂反作用显著影响无人机，需要新增：

- 基座动力学行；
- 推力/力矩到旋翼推力的控制分配；
- 无人机欠驱动约束；
- 基座 wrench 与关节 torque 联合优化/控制；
- SE(3) 基座状态积分；
- 飞控与机械臂控制器带宽和稳定性分析。

这不是当前 `whole` 绘图任务已经覆盖的功能。

### P2：测试和文档清理

- 更新 `VALIDATION_RESULTS.md` 到共享锁步架构；
- 更新 `torque_controller_node.py` 中“hardware nodes do not subscribe”这条过时日志；
- 给共享注入和坐标转换增加测试；
- 限定 lint 扫描范围，排除 `dist/`、`build/`、`install/`；
- 为 data 建立保留策略或 `.gitignore`，只保存精选 summary/图；
- 将运行元数据写入 summary：git commit、dirty hash、模式、config hash、后端、控制频率。

## 12. 下一位 AI 建议执行顺序

1. 阅读本文、`CODEX_DISCOVERY_REPORT.md` 和原始附件。
2. 执行 `git status --short`，确认没有新的用户改动。
3. 运行 1 秒 `simulation_injection.enabled=false` smoke test，建立基线。
4. 不调参，先完成共享 `TorqueSimArm` 扰动 backend。
5. 对每个扰动写独立测试，再跑统一 ablation。
6. 修正基座消息坐标语义并保证基线不退化。
7. 只有在共享链数据可信后再调 SMC 参数。
8. 真机前完成 `0x12` 台架确认和真实基座 adapter。
9. 最后再决定是否升级完整无人机机械臂 whole-body 控制。

## 13. 完成定义

### 共享仿真下一阶段完成条件

- `/joint_states` 仍只有 `student_arm_node` 一个 publisher；
- 每帧命令只推进一次 plant；
- `simulation_injection.enabled=false` whole 仍满足 RMS < 1 mm、peak < 2 mm；
- 所有注入都作用于共享 `TorqueSimArm`，而不是隐藏 internal plant；
- CSV 能明确区分命令、实际和扰动力矩；
- ablation 使用同一控制频率、限幅、周期和后端；
- 结果可由单条命令复现。

### 真机阶段完成条件

- `0x12` 单位和缩放有文档或台架证据；
- 每个关节方向、限幅、反馈单位已标定；
- 基座状态坐标和时间戳契约有测试；
- 急停、超时、掉线和驱动器归零经过实测；
- 先台架/运动平台，再系留飞行；
- 不以理想仿真 RMS 作为实机安全依据。

## 14. 给下一位 AI 的一句话任务

> 保留当前无扰动共享锁步基线，不修改 NMPC；先把全部物理扰动迁入 `TorqueSimArm` 并统一坐标/诊断，再重新做消融。真机前必须确认协议 `0x12` 和实现真实无人机基座状态适配器。
