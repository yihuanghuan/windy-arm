# 任务空间 SMC 交接文档

日期：2026-08-04  
读者：下一位接手 `src/arm-platform/task_space_smc` 的 AI

这份文档是旧 `docs/` 目录里 9 份文档的合并版。旧文档已经删除，因为里面混有 7 关节时代、旧路径、旧启动架构和被 2026-08-04 实测推翻的结论。

接手时先读本文件，再读上一级 `README.md` 的运行命令。若两者冲突，以本文件为准。

---

## 0. 当前真相，先覆盖脑内缓存

### 0.1 已被推翻的旧说法

| 旧说法 | 现在的实测结论 |
|---|---|
| 电机自报速度比位置多滞后 15 ms | 错。最新互相关结论是位置与速度 **0 拍滞后**，相关系数 `r = 0.90~0.94`。旧文档和旧注释里所有“额外 15 ms 群延迟”“速度路 25 ms、位置路 10 ms”的说法都不能再当真。 |
| `c0 = 216` 是位置增益 | 不准确。`216` 是边界层内的线性化值；实机约 **95% 时间不在边界层内**，有效值约 **151~187**，且随误差变大而变软。 |
| joint2 静摩擦标定 `1.775 Nm` 太高，应降到 `0.767 Nm` | 方向反了。单独降这项会让 joint2 前馈从 **0.390 Nm** 掉到 **0.169 Nm**，更补不住。 |
| 改了 `task_space_smc/*.py` 后不重编不生效 | 对当前仓库是错的。当前用 `--symlink-install`，runner 是源码树符号链接，并且启动时把源码包父目录插入 `sys.path`；改 `task_space_smc/**/*.py` 或 `config/*.yaml` 后，下次启动立即生效。C++、消息、launch、CMake 仍需重编。 |
| `task_space_smc/xxx.py` 平铺路径 | 全部过时。当前分层是 `core/`、`sim/`、`hw/`。旧文档中的裸文件名和旧 import 路径不要照抄。 |
| 6 关节主线仍是 5D 笔轴任务 + `arm6_tool.urdf` | 已过时。当前 `smc_params_hwtest.yaml` 使用 `urdf: arm6`、`ee_frame: link6`、`task.orientation_mode: full`，即 6 关节解 6D 全姿态方阵任务。早期 5D/笔架论证只作为历史背景保留：它解释为什么当时要加虚拟笔架，但不是当前上机配置。 |

### 0.2 2026-08-04 新增或固化的机制

- 目录已按依赖方向分层：`core/` 不 import ROS，`sim/` 和 `hw/` 只依赖 `core/`，二者互不依赖。
- 就位不再依赖旧关节空间 approach。当前主思路是 SMC 从第一帧接管，靠 `handover` 的参考偏置衰减把当前位姿平滑拉到轨迹上。
- 起点奇异门已经是硬安全条件：起步时会检查 `sigma_min`，低于 `sigma_critical = 0.012` 时拒绝发力矩。
- 工作台护栏已写入配置：桌面 `z = 0`，`+x` 侧无桌面，`-x/+y` 侧有围栏；路径检查只保证末端点，不保证整条连杆扫掠体积。
- 看门狗吃的是测量值，不是估计值。估计器属于被测对象，不能喂给兜底逻辑。
- 看门狗触发后会保位并收尾退出，不再把剩余运行时长耗完。

### 0.3 今天新增的硬数

- 增益天花板：`c1 ≈ 35` 已经接近实机硬件天花板，实测可在约 **5 s** 量级发散；不要用解析裕度继续往上加。
- 速度量化是 12 bit：大臂 `j1~j3` 速度 LSB 约 **0.009768 rad/s**，腕部 `j4~j6` 速度 LSB 约 **0.024418 rad/s**。位置 LSB 约 **0.000381 rad**。
- joint2 失力后曾下沉 **14.6 deg**。任何“停驱动后仍能保持”的假设都不成立。
- 轮次间方差已经压过很多参数效应。调参必须先做重复基线，否则会把噪声当改进。

---

## 1. 仓库当前形状

核心目录：

```text
src/arm-platform/task_space_smc/
├── core/      控制律、模型、误差、伪逆、状态估计、摩擦前馈、handover、config_loader
├── sim/       offline_simulation、tune_whole_task_smc、ablate_simulation_injection、plot
├── hw/        whole_task_smc_ros_runner、上机工具、标定脚本、serial_tap
├── config/    SMC 参数 YAML
├── data/      运行产物，已 gitignore
└── docs/      现在只保留本文件
```

关键文件：

| 文件 | 作用 |
|---|---|
| `core/task_space_smc.py` | 滑模控制律：`s = e_dot + Lambda e`，`nu = xdd_ref + Lambda e_dot + K s + eta tanh(s/phi)` |
| `core/model.py` | Pinocchio 自由基座模型、FK/Jacobian/CRBA/RNEA、标定值注入 |
| `core/redundancy_resolution.py` | 动力学一致伪逆、阻尼、零空间投影、可行性盒遗留实现 |
| `core/friction_feedforward.py` | 静摩擦前馈，注意 `scale < 1` 是硬约束 |
| `core/handover_blend.py` | 参考偏置衰减，把当前位姿并入轨迹 |
| `core/state_estimator.py` | motor / finite_difference / LESO / Smith 组合层，缺省逐位直通 |
| `hw/whole_task_smc_ros_runner.py` | 主 ROS runner，仿真与实机共用路径 |
| `hw/pose_check_readonly.py` | 上机前只读位形核对，结构上不发送字节 |
| `hw/serial_tap.py` | 伪终端透明中继，能同时抓上行和下行 |
| `hw/wrist_prepose.py` | 只把 joint5 挪出腕部奇异 |
| `hw/full_prepose.py` | 位置模式整臂挪到安全位形，不能当正常实验就位真值 |

---

## 2. 构建与生效规则

必须使用：

```bash
colcon build --packages-select dummy_interface manipulator \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

当前生效规则：

| 改动 | 是否要重编 | 原因 |
|---|---|---|
| `task_space_smc/**/*.py` | 不用 | runner 是源码树符号链接，且把源码包父目录插进 `sys.path` |
| `task_space_smc/config/*.yaml` | 不用 | 运行时读取配置 |
| C++ 源码 / 消息定义 | 要 | 需要重新编译安装 |
| launch 文件 / CMake 新增入口脚本 | 要 | launch 和入口安装仍走 colcon |
| 移动或删除 Python 文件 | 建议清 install 后重编 | `install(DIRECTORY ...)` 只拷不删，旧副本会残留误导排查 |

验证 symlink：

```bash
ls -l install/manipulator/lib/manipulator/whole_task_smc_ros_runner.py
```

应指向：

```text
.../src/arm-platform/task_space_smc/hw/whole_task_smc_ros_runner.py
```

不要用 `PYTHONPATH=src/arm-platform` 覆盖 ROS 路径；如需添加，用：

```bash
export PYTHONPATH="$PWD/src/arm-platform:$PYTHONPATH"
```

---

## 3. 运行入口

详细命令在 `README.md`，这里只写接手时最容易误用的事实。

### 3.1 仿真

6 关节仿真入口：

```bash
ros2 launch manipulator task_space_smc_whole6.launch.py \
  trajectory_shape:=hwtest duration_sec:=0.0 rviz:=true
```

完整五工作站任务要显式换配置：

```bash
WINDYLAB_SMC_CONFIG=$PWD/src/arm-platform/task_space_smc/config/smc_params_whole6.yaml \
ros2 launch manipulator task_space_smc_whole6.launch.py \
  trajectory_shape:=whole duration_sec:=0.0 rviz:=true
```

仿真 domain 惯例用 `ROS_DOMAIN_ID=77`。

### 3.2 实物

实物只有 6 个关节；固件仍上报第 7 通道，那是幽灵数据。硬件驱动和 SMC runner 是两个终端。

驱动终端：

```bash
export ROS_DOMAIN_ID=42
ros2 launch manipulator student_arm6.launch.py arm_type:=a_l1 \
  port_name:=/dev/ttyUSB0 control_rate_hz:=200.0 \
  enable_direct_effort_command:=true max_velocity:=0.5 use_rviz:=false
```

SMC 终端：

```bash
export ROS_DOMAIN_ID=42
export WINDYLAB_SMC_CONFIG=$PWD/src/arm-platform/task_space_smc/config/smc_params_hwtest.yaml
export MB_HWTEST_SPEED=0.01
ros2 launch manipulator task_space_smc_whole6.launch.py \
  mode:=hardware hardware_enabled:=true rviz:=false \
  trajectory_shape:=hwtest duration_sec:=60 start_delay_sec:=0 \
  data_dir:=/path/to/run
```

注意：

- `hardware_enabled:=true` 必须显式写。否则适配器会输出禁用命令，机械臂不会按 SMC 力矩走。
- 上实物必须用非 0 domain，并确认 `/joint_states` 只有一个 publisher。
- 启动前先用 `pose_check_readonly.py` 读实物位形，再判断是否需要 `wrist_prepose.py`。
- 协议没有软急停；急停只能物理断电。驱动进程停止后机械臂会失去保持力矩。

---

## 4. 安全与硬件事实

### 4.1 物理约束

- 实物是 A-L1-GAMMA 6 关节臂；第 7 通道为幽灵数据。
- 桌面在 `z = 0`，与基座安装面同平面。
- `+x` 侧无桌面，`-x` 侧有桌面。
- 围栏在 `-x/+y` 侧，约到 `y = 350 mm`，高度约“基座到 joint3”。
- 位置模式控制频率不超过 **100 Hz**；力矩模式按反馈不超过 **200 Hz**。
- 上行反馈实测约 **200.6 Hz**；这不是串口带宽限制，而是固件发布速率。

### 4.2 协议与单位

帧格式：

```text
0xFE 0xAA | cmd | len | payload | sumCheck | addCheck    PC -> 板
0xFE 0x55 | cmd | len | payload | sumCheck | addCheck    板 -> PC
```

上行：

| cmd | 内容 | 速率 |
|---|---|---|
| `0x20` | position | 200.6 Hz |
| `0x21` | velocity | 200.6 Hz |
| `0x22` | current/torque feedback | 200.6 Hz |
| `0x23` | voltage，全通道恒 0 | 2.0 Hz |
| `0x24` | temperature | 200.6 Hz |
| `0x01` | heartbeat | 20.0 Hz |
| `0x25` | 未知，payload `[1,1,1,0,0,0,0]` | 2.0 Hz |

量化反解：

| 量 | 实测 LSB | 反解 |
|---|---|---|
| position | `3.833771e-04 rad` | 16 bit / `±4π rad` |
| velocity 旧记录 | `2.441788e-02 rad/s` | 12 bit / `±50 rad/s` |
| velocity 最新分型 | `j1~j3 0.009768 rad/s`，`j4~j6 0.024418 rad/s` | 12 bit，大小电机量程不同 |
| torque feedback J1-3 | `1.367378e-02 Nm` | 12 bit / `±28.00 Nm` |
| torque feedback J4-6 | `2.930164e-03 Nm` | 12 bit / `±6.00 Nm` |

额定连续力矩：`j1~j3 ±9 Nm`，`j4~j6 ±1.6 Nm`。  
硬件反馈量程：`j1~j3 ±28 Nm`，`j4~j6 ±6 Nm`。

`0x12` 命令标度阶段 2 合并结果：`alpha = 1.033 ± 0.024`。

### 4.3 第 7 通道

第 7 通道不是实物电机：

| 流 | joint1-6 范围 | 通道 7 |
|---|---|---|
| position | `-1.381 ... 1.324 rad` | `-6.5231 rad`，超限 |
| velocity | `-0.208 ... 0.159 rad/s` | `+27.9731 rad/s`，超 RS05 上限 5.6 倍 |
| temperature | `27 ... 31 C` | `1` |

屏蔽靠 `arm_version:=gamma6`。

### 4.4 只读位形核对

```bash
python3 src/arm-platform/task_space_smc/hw/pose_check_readonly.py --offline-selftest
python3 src/arm-platform/task_space_smc/hw/pose_check_readonly.py --port /dev/ttyUSB0 --seconds 3
```

这个脚本打开串口后会把 `write` / `flush` 等方法替换成抛异常的桩，不 import `rclpy`，不发布话题。读实物真值只信它，不要信可能被多节点污染的 `/joint_states`。

---

## 5. 标定值与模型

标定文件在：

```text
src/arm-platform/config/arm6_calibration.yaml
```

当前关键值：

| 项 | 数值 |
|---|---|
| `torque_scale_alpha` | `1.033 ± 0.024` |
| `zero_offset_rad` | `[0.039576, -0.297745, -0.020218, -0.192988, -0.111667, 0.0]` |
| `zero_offset_deg` | `[2.268, -17.060, -1.158, -11.057, -6.398, 0.0]` |
| `zero_offset_converged` | `[true, true, true, false, false, false]` |
| `zero_offset_uncertainty_deg` | `[0.141, 0.316, 0.163, 2.0, 1.161, null]` |
| `first_moment_ratio` | `[1.265, 1.349, 1.228, 0.972, 1.021, 1.0]` |
| `stiction_nm` | `[0.725, 1.775, 0.625, 0.125, 0.100, 0.100]` |
| `deadband_rad` | `[0.0242, 0.0592, 0.0208, 0.0250, 0.0200, 0.0200]` |
| `inertia_kgm2` | `[0.11198, 0.19707, 0.12985, 0.01231, 0.01154, null]` |
| `inertia_stderr_kgm2` | `[0.00106, 0.01567, 0.00324, 0.00035, 0.00074, null]` |
| `inertia_urdf_kgm2` | `[0.0129, 0.0788, 0.0138, 0.0006, 0.0008, null]` |
| `kinetic_friction_nm` | `[-0.023, 1.675, 0.770, 0.032, 0.018, null]` |
| `armature_kgm2` | `[0.09527, 0.09405, 0.11219, 0.01154, 0.01050, 0.01050]` |

要点：

- `first_moment_ratio` 已烘进 `arm6.urdf` 和 `arm6_tool.urdf` 的连杆惯性参数。对这两个 URDF 不要再乘一次。
- `config/arm.urdf` 是 7 关节名义模型，未做同样修正。
- 改连杆惯性后必须同步重算 `armature`，保证 `M_effective = M_link + armature` 不重复计入。
- `stiction_nm` 是静摩擦，`kinetic_friction_nm` 是动摩擦，二者不能混用。
- joint6 没有重力信号：link6 质心在 joint6 转轴上，径向偏移约 `0.113 mm`；它也未完成惯量辨识，按同型号 joint5 取 armature。

方法学：

- 斜率法免疫摩擦：阶段 2 用 `d(delta_q)/d(t_ff) = alpha/kp`，阶段 5 用 `d(qdd)/d(tau) = 1/M`；摩擦进截距，不进斜率。
- `resid/A < 0.05` 只在 `A/f > 5` 时能指示几何错误。joint4/5 的 `resid/A = 0.29~0.35` 是摩擦主导。
- 相位精度由拟合条件数决定，不是 `deadband/sqrt(N)`；joint4 窄区间 `0.8 rad` 覆盖 `13%` 周期时 ratio 错了 **59%**，宽区间 `7.0 rad` 覆盖 `111%` 周期才可信。

---

## 6. 当前控制配置

首次上实机的小幅轨迹配置：

```text
task_space_smc/config/smc_params_hwtest.yaml
```

当前主配置的关键事实：

| 项 | 当前值 |
|---|---|
| URDF | `arm6` |
| EE frame | `link6` |
| 控制频率 | `200 Hz` |
| task | `orientation_mode: full`，6D 全姿态 |
| 位置增益 | `lambda = [18,18,18]`，`k = [8,8,8]`，`eta = [0.08,0.08,0.08]`，`phi = [0.02,0.02,0.02]` |
| 姿态增益 | `lambda = [3.6,3.6,3.6]`，`k = [6,6,6]`，`eta = [0.018,0.018,0.018]`，`phi = [0.02,0.02,0.02]` |
| `initial_pose_rad` | `[-0.0635, 1.5558, -1.4086, -0.0002, 0.6938, 0.0033]` |
| `sigma_warn` / `sigma_critical` | `0.03` / `0.012` |
| torque limit | `[9,9,9,1.6,1.6,1.6] Nm` |
| torque_rate | `[1500,1500,1500,500,500,350] Nm/s` |
| tracking friction feedforward | enabled, `scale=0.8`, `velocity_epsilon=0.02`, `error_gain=1.0` |
| estimator | `source: motor`，也就是基线直通 |
| handover | `reference_blend_sec=30.0`，`freeze_trajectory_during_blend=true` |

注意：

- 配置里的长注释有些仍是旧分析结论，尤其是“速度额外 15 ms”相关段落，已被 0.1 节推翻。
- `approach` 段参数保留作回退和历史对照；当前主架构看 `handover`。
- `MB_HWTEST_SPEED` 默认由 6 关节 launch 压到 `0.01 m/s`；轨迹模块自身默认曾是 `0.02 m/s`。绕过 launch 时要当心。
- 提速会压缩拉入段时间轴。实测起步位形离轨迹起点 `78 mm` 时，速度 `0.01 -> 0.015` 使拉入段 j1 峰值速度 `0.25 -> 3.04 rad/s`，增幅 **12 倍**，撞看门狗。

---

## 7. 已验证能力与不能再重做的排除项

### 7.1 仿真与模型

旧文档中很多“达标”数字属于 7 关节、5D、虚拟笔架或旧锁步架构，不能混用。

仍可保留的历史数字：

- 6 关节 `hwtest` 轨迹历史 ROS 仿真曾达到 **RMS 0.0041 mm / 峰值 0.029 mm**。
- 6 关节 `whole` 历史 ROS 仿真曾达到 **RMS 0.061 mm / 峰值 1.13 mm**。
- 这些数字用于说明任务定义修正后模型可自洽，不是当前实机稳定性保证。

### 7.2 已排除方向

以下方向做过真实 ROS 或实物反事实，不要重复当作新想法：

- 放松速度上限：`max_velocity ×1.5` 无统计改善；`×3` 峰值变坏；`×30` 发散。
- 放松力矩上限：`tau_scale=2` 使 RMS 变差。
- 50 Hz 速度低通：存活时间 `0.47 s -> 0.35 s`，更早发散，说明当时回路相位受限，不是噪声受限。
- 饱和感知解算：单元测试全绿，但真实 ROS 坏约 100 倍；代码保留在 `redundancy_resolution.py`，runner 中不要重新打开，除非重新做真实 ROS 验证。
- 旧零空间姿态拉向固定 `q_nominal`：当前 6D 方阵任务零空间秩为 0，零空间通道对 j1/j2 崩没有解释力。

---

## 8. j1/j2 崩机理，调查中

这部分来自正在运行的机理工作流中间结果。最后“综合”阶段曾因 session limit 中断，所以不要把下面写成最终论文式结论；但这些事实已经有多角度支持。

### 8.1 站得住的事实

- j1/j2 看门狗线是 `1.2 × [3,3,3,5,5,5] rad/s`，即大臂阈值 **3.6 rad/s**。
- 多次触发读数集中在 `-3.6~-3.8 rad/s`，j1 也出现过 `+3.61 / +3.77 rad/s`。
- 触发不是单纯的特殊码或通信假值；但“触发那一拍位置通道逐拍完全印证”的旧证据被削弱，位置与速度采样并非总是同拍同步。
- 饱和每次都在场，但更像症状和幅值夹子，不是单独起因。
- gravity、nullspace、timing、kinematics 都不能单独解释 j1/j2。
- stick-slip 作为 j1/j2 崩的主因被证伪；更像是延迟驱动的 `±9 Nm` bang-bang 极限环。半周期约 `15~25 ms` 的饱和角冲量把 j1/j2 推到 `3.4~4.0 rad/s`，刚好跨过 `3.6 rad/s` 看门狗线。
- 跨轮统计显示：参考量开始运动后的起振风险高于冻结段，但在已有 19 轮里，没有一个配置变量能稳定预测 `t_trip`；轮次间方差很强。

### 8.2 调查时不要回头踩的坑

- 不要再用“位置/速度多 15 ms”解释，最新互相关是 0 拍。
- 不要把 `c0=216` 当实际位置增益；有效 `151~187`，且随误差变软。
- 不要把单独降低 joint2 `stiction_nm` 当修复；它会削弱前馈。
- 不要只看单次 run 的好坏。至少先跑重复基线，估计轮次方差。

---

## 9. 上机前检查清单

1. `git status --short`，确认自己只在文档或目标文件范围内工作。
2. `source /opt/ros/humble/setup.bash && source install/setup.bash`。
3. `export ROS_DOMAIN_ID=42`，实物不要用 0。
4. 确认没有残留节点：

```bash
ps -eo pid,lstart,cmd | grep -E 'ros2 launch|student_arm_node|robot_state_publisher' | grep -v grep
```

5. 确认串口：

```bash
ls -l /dev/ttyUSB*
ls -l /dev/serial/by-id/
```

6. 只读位形：

```bash
python3 src/arm-platform/task_space_smc/hw/pose_check_readonly.py --offline-selftest
python3 src/arm-platform/task_space_smc/hw/pose_check_readonly.py --port /dev/ttyUSB0 --seconds 3
```

7. 若 joint5 仍接近 0，先挪出腕部奇异：

```bash
python3 task_space_smc/hw/wrist_prepose.py --target-deg 40 --duration 20 --settle 3
```

8. 起驱动后确认 `/joint_states` publisher count 是 1，频率约 200 Hz。排查 topic 时优先加 `--no-daemon`。
9. 启 SMC 时确认 `mode:=hardware hardware_enabled:=true rviz:=false`。
10. 全程手在物理断电开关旁。

---

## 10. 旧文档处置

旧文档已经合并后删除。原因如下：

| 旧文件 | 处置 | 原因 |
|---|---|---|
| `AI_HANDOFF.md` | 删除 | 7DoF、旧锁步仿真、旧 `0x12` 未确认状态、旧工作流保护信息混杂，当前只保留少量坐标/架构提醒。 |
| `CODEX_DISCOVERY_REPORT.md` | 删除 | 7 关节开工踩点笔记，接口事实已被后续文档覆盖。 |
| `HANDOFF_2026_08_02.md` | 删除 | 有大量 2026-08-02 夜间事实，但包含“改 Python 必须重编”等被推翻结论；有效安全经验已并入本文件。 |
| `OVERNIGHT_REPORT.md` | 删除 | 阶段性汇报，后续实测已覆盖；标定数值并入第 5 节。 |
| `PHASE1_FINDINGS.md` | 删除 | 方法学价值高，但内部也有自我推翻的旧表格；关键标定和判别流程已压缩并入。 |
| `PHASE4_PLAN.md` | 删除 | “待确认执行方案”已过期；安全原则和阶段 4 意义已并入。 |
| `PHASE_PLAN.md` | 删除 | 阶段计划大量完成或失效；保留斜率法、拟合条件数、阶段 5/动捕提醒。 |
| `REFERENCE.md` | 删除 | 深入手册内容太多且混有旧 5D/笔架/延迟结论；当前事实由本文件集中维护。 |
| `VALIDATION_RESULTS.md` | 删除 | 旧验收快照，7 关节和旧仿真链数字会误导。 |

以后不要重新引入多份互相覆盖的“交接”“参考”“阶段报告”。新事实直接追加到本文件，过时结论就地标为过时或删除。
