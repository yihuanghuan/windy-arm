# 浮动基座任务空间滑模控制 (SMC)

A-L1-GAMMA 机械臂的任务空间力矩控制。控制律在世界系里算末端位姿误差，经动力学一致伪逆
映射成关节力矩；基座可以是运动的（浮动基座），臂在其上补偿。

本文件讲**怎么跑**：文件结构、部署、仿真启动、实物启动。
**为什么这么设计、实测到了什么、出问题怎么查**在 [docs/REFERENCE.md](docs/REFERENCE.md)。
接手正在进行的工作先读 [docs/HANDOFF_2026_08_02.md](docs/HANDOFF_2026_08_02.md)。

| 我想…… | 去哪 |
|---|---|
| 知道某个文件是干什么的 | [1. 文件结构](#1-文件结构) |
| 第一次把环境跑起来 | [2. 部署](#2-部署) |
| 在 RViz 里看仿真 | [3. RViz 仿真](#3-rviz-仿真) |
| 接实物跑 | [4. 实物启动](#4-实物启动) |
| 理解控制律 / 标定 / 调参 / 排错 | [docs/REFERENCE.md](docs/REFERENCE.md) |

---

## 1. 文件结构

2026-08-04 按**依赖方向**分层，不是按文件大小或新旧分：

```
task_space_smc/
├── README.md            本文件
├── core/                不 import ROS，不区分仿真与实物
├── sim/                 只在离线 / 仿真里跑
├── hw/                  需要真机或 ROS 才有意义
├── docs/                文档
├── config/              参数 yaml（不分层）
└── data/                运行产物（已 gitignore）
```

**依赖是单向的**：`hw/` 和 `sim/` 都依赖 `core/`，两者之间互不依赖。

### 1.1 `core/` — 核心控制算法

| 文件 | 作用 |
|---|---|
| `task_space_smc.py` | **滑模控制律本体**。`ν = ẍ_r + Λė + K·s + η·tanh(s/φ)` |
| `model.py` | Pinocchio 浮动基座模型封装（正运动学、雅可比、质量阵、重力） |
| `pose_error.py` | 世界系位姿误差（位置差 + 姿态测地误差） |
| `quaternion_math.py` | 四元数与 SO(3) 运算。被依赖 8 次，最底层 |
| `redundancy_resolution.py` | 动力学一致伪逆、零空间加速度分配、笔轴任务、限位规避 |
| `trajectory_reference.py` | 世界系参考轨迹 |
| `base_state.py` | 基座状态源（仿真注入 / ROS 反馈） |
| `config_loader.py` | yaml → `SMCGains` → `TaskSpaceSMC` 的装配 |
| `actuator_adapter.py` | 力矩 → 电机指令，含 NaN/Inf 归零与饱和保护 |
| `approach_reference.py` | 就位段的末端直线 + SO(3) 测地线参考轨迹 |
| `handover_blend.py` | 交接瞬间的参考量偏置衰减（接管时不让机械臂猛跳） |
| `friction_feedforward.py` | 静摩擦前馈，把"增益下界"从摩擦约束里解耦出来 |
| `velocity_filter.py` | 关节速度反馈一阶低通（缺省关闭） |
| `loop_delay.py` | 仿真侧回路延迟建模（实物模式返回全零 = 直通） |
| `leso.py` | 线性扩张状态观测器，用位置反推速度并估扰动 |
| `smith_predictor.py` | 死区时间补偿，把滞后的反馈状态往前推 |
| `state_estimator.py` | 上面三者的组合层 + 配置解析。**缺省逐位直通** |

### 1.2 `sim/` — 仿真上层

| 文件 | 作用 |
|---|---|
| `offline_simulation.py` | 离线验证 harness（不起 ROS，直接积分） |
| `tune_whole_task_smc.py` | 确定性增益扫描 |
| `ablate_simulation_injection.py` | 注入项消融研究 |
| `plot_whole_task_smc.py` | 从运行 CSV 生成跟踪 / 误差 / 力矩 / 3D 报告 |

### 1.3 `hw/` — 实物上层

| 文件 | 作用 |
|---|---|
| `whole_task_smc_ros_runner.py` | **主运行器**。同时支持 `mode:=simulation` 与 `mode:=hardware` |
| `full_prepose.py` | 整臂位置模式就位（6 轴一起挪到指定位形） |
| `wrist_prepose.py` | 只把 joint5 挪离腕部奇异 |
| `pose_check_readonly.py` | 上机前**只读**位形核对，结构上不可能发字节 |
| `fk_gravity_verify.py` | 阶段 1：正运动学 / 关节零位校验 |
| `merge_fk_verify.py` | 合并多次阶段 1 结果，输出零位标定向量 |
| `phase2_torque_probe.py` | 阶段 2：0x12 力矩通路验证 |
| `phase3_gravity_calibration.py` | 阶段 3：迭代重力标定 |
| `phase4_gravity_compensation.py` | 阶段 4：整臂重力补偿（分级降 kp 到"悬浮"） |
| `phase5_inertia_id.py` | 阶段 5：惯量辨识 |
| `serial_tap.py` | 双向串口抓包（伪终端透明中继） |
| `torque_interface_probe.py` | 力矩通路小幅探针 |
| `torque_controller_node.py` | ⚠️ **遗留**：7 关节时代的独立节点，无任何 launch 引用，已被主运行器取代 |

> `whole_task_smc_ros_runner.py` 放在 `hw/` 是因为它面向真机，但它**同时跑仿真和实物，
> 而且刻意是同一条代码路径**。这条约束是故意的 —— 否则仿真里得到的结论带不到实物上。

### 1.4 `config/` 与 `data/`

`config/` **不跟着分层**，`core/config_loader.py` 按包根定位它。

| 配置 | 用于 |
|---|---|
| `smc_params_hwtest.yaml` | 6 关节实机（`hwtest` 轨迹）。**上机默认用这份** |
| `smc_params_hwtest_simbest.yaml` | 同上，但取仿真最优增益。已验证基线，不要改 |
| `smc_params_whole6.yaml` | 6 关节完整五工作站任务 |
| `smc_params_whole.yaml` | 7 关节完整任务 |
| `smc_params.yaml` | 离线仿真默认配置 |

`data/` 是每次运行的 CSV / JSON / PNG，已 gitignore，不入库。

### 1.5 测试

`test/core/`（19）、`test/sim/`（2）、`test/hw/`（5），与源码同构。
`test/` 根只留 `conftest.py` 和 C++ 包自己的 `.cc`。

```bash
cd /home/windylab/code/windylab_ws
python3 -m pytest src/arm-platform/test -q      # 应为 549 passed
```

`conftest.py` 做两件事：把包父目录插进 `sys.path`；在任何测试模块导入前设好
`MB_STATION_LAYOUT=horizontal`（轨迹模块在**模块级**读它，谁先 import 谁定布局）。

---

## 2. 部署

### 2.1 编译

```bash
cd /home/windylab/code/windylab_ws
source /opt/ros/humble/setup.bash

colcon build --packages-select dummy_interface manipulator \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON

source install/setup.bash
```

每开一个新终端都要先
`source /opt/ros/humble/setup.bash && source install/setup.bash`。

### 2.2 `--symlink-install` 不是可选项

带了它，`install/manipulator/lib/manipulator/whole_task_smc_ros_runner.py` 是一个
**指向源码树的符号链接**；运行器启动时逐级上溯出 `__init__.py`，把**源码树**的包父目录
插进 `sys.path`。结果是：

> **改 `task_space_smc/` 下的 `.py` 或 `config/*.yaml`，不用重编，下次启动就生效。**

不带 `--symlink-install` 的话它是个副本，`sys.path` 会落到 install 目录，跑的就是
`install(DIRECTORY)` 拷过去的那份旧代码 —— 改了源码没反应，而且不报错。

验证当前装的是哪种：

```bash
ls -l install/manipulator/lib/manipulator/whole_task_smc_ros_runner.py
# 应看到 -> .../src/arm-platform/task_space_smc/hw/whole_task_smc_ros_runner.py
```

### 2.3 什么时候必须重编

| 改了什么 | 要重编吗 |
|---|---|
| `task_space_smc/**/*.py` | **不用** |
| `config/*.yaml` | **不用** |
| C++ 源码、消息定义（`dummy_interface`） | **要** |
| `CMakeLists.txt`（新增 / 移动入口脚本、装新文件） | **要** |
| launch 文件 | **要**（launch 是拷贝安装的） |

⚠️ `install(DIRECTORY task_space_smc/ ...)` **只拷不删**。移动或删除源码文件后
install 目录会留下旧副本。运行器虽然用不到它们，但查问题时会看错文件。搬过文件后清一次：

```bash
rm -rf install/manipulator/lib/manipulator/task_space_smc
colcon build --symlink-install --packages-select manipulator
```

### 2.4 不要覆盖 `PYTHONPATH`

用 `PYTHONPATH=src/arm-platform` 会顶掉 ROS 自己的路径，找不到 `pinocchio`。要**追加**：

```bash
export PYTHONPATH="$PWD/src/arm-platform:$PYTHONPATH"
```

### 2.5 ROS_DOMAIN_ID

**上实物必须用专属的非 0 domain**，否则同网段别人的仿真节点会和硬件驱动串到一起。
惯例：实物 `42`，仿真 `77`。

```bash
export ROS_DOMAIN_ID=42     # 实物
export ROS_DOMAIN_ID=77     # 仿真
```

---

## 3. RViz 仿真

两条 launch。**7 关节和 6 关节是不同的机械臂构型，不是同一条链上的开关。**

| | 7 关节 | 6 关节 |
|---|---|---|
| launch | `task_space_smc_whole.launch.py` | `task_space_smc_whole6.launch.py` |
| URDF | `arm.urdf` | `arm6_tool.urdf` |
| 末端 frame | 自动探测 | `link7`（**显式指定**） |
| 默认控制频率 | 500 Hz | 200 Hz |
| 对应实物 | 无 | **有** |

### 3.1 7 关节

```bash
cd /home/windylab/code/windylab_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=77

src/arm-platform/scripts/run_task_space_smc_whole.sh \
  mode:=simulation start_delay_sec:=3.0 duration_sec:=0.0 hardware_enabled:=false
```

helper 脚本会清掉可能指向别处的旧 overlay，并确认 `ros2 pkg prefix manipulator`
指向本工作区。也可以直接 `ros2 launch manipulator task_space_smc_whole.launch.py ...`。

| 参数 | 默认 | 说明 |
|---|---|---|
| `mode` | `simulation` | 见 [3.3](#33-三种-mode) |
| `start_delay_sec` | `3.0` | 先显示期望路径，再从偏置位姿平滑靠近起点。**0 = 直接进跟踪段** |
| `duration_sec` | `0.0` | `0` = 一个完整轨迹周期 |
| `hardware_enabled` | `false` | 是否允许适配器输出非零 `MotorControl` |
| `control_rate_hz` | `500.0` | SMC 与 `student_arm_node` 共用 |
| `rviz` | `true` | 是否启动 RViz |
| `data_dir` | `task_space_smc/data` | CSV / summary / PNG 输出目录 |

短时冒烟：`start_delay_sec:=1.0 duration_sec:=1.0 rviz:=false`。
改了 C++ 或消息：加 `WINDYLAB_SMC_REBUILD=1`。

### 3.2 6 关节

这条 launch 有**两种轨迹**：

| `trajectory_shape` | 内容 | 基座 | 力矩上限 | 周期 | 仿真 RMS |
|---|---|---|---|---|---|
| **`hwtest`（默认）** | +x 侧竖直小圆，半径 4 cm | **固定** | 额定 9 / 1.6 | 12.57 s | 0.0041 mm |
| `whole` | 五工作站完整任务 | 解析摆动 | 2× 额定 18 / 3.2 | 73.04 s | 0.061 mm |

`hwtest` 是上实机用的：幅度小、余量大、放在**无桌面的 +x 侧**。
`whole` 是研发用的完整任务，基座带摆动、跨四个方位，难度高得多。

```bash
cd /home/windylab/code/windylab_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=77

# 默认：上机前的小幅轨迹
ros2 launch manipulator task_space_smc_whole6.launch.py \
  trajectory_shape:=hwtest duration_sec:=0.0 rviz:=true

# 完整五工作站任务（必须显式换配置）
WINDYLAB_SMC_CONFIG=$PWD/src/arm-platform/task_space_smc/config/smc_params_whole6.yaml \
ros2 launch manipulator task_space_smc_whole6.launch.py \
  trajectory_shape:=whole duration_sec:=0.0 rviz:=true
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `trajectory_shape` | `hwtest` | `hwtest` / `whole` |
| `mode` | `simulation` | 见 [3.3](#33-三种-mode) |
| `start_delay_sec` | `3.0` | 靠近段时长。**上机架构用 `0`** |
| `duration_sec` | `0.0` | `0` = **该形状的**一个完整周期（按形状查，不是写死值） |
| `hardware_enabled` | `false` | 是否允许适配器输出非零 `MotorControl` |
| `control_rate_hz` | `200.0` | SMC 与 `student_arm_node` 共用 |
| `rviz` | `true` | 是否启动 RViz |
| `data_dir` | `task_space_smc/data` | 输出目录 |

### 3.3 三种 `mode`

| 模式 | 说明 |
|---|---|
| `simulation` | 主模式。SMC 与被控对象通过 ROS 消息连接，`student_arm_node` + `TorqueSimArm` |
| `hardware` | 不自动起硬件节点（防止误开串口），需要外部先起好驱动。见 [第 4 节](#4-实物启动) |

### 3.4 环境变量

`WINDYLAB_SMC_CONFIG` 之外，轨迹模块在**模块级**读下面这些，所以 launch 文件在顶部
`os.environ.setdefault` 设好 —— 命令行里覆盖要写在 `ros2 launch` **前面**。

| 变量 | 生效默认 | 谁设的 | 说明 |
|---|---|---|---|
| `WINDYLAB_SMC_CONFIG` | 按 launch 选 | launch | 配置文件绝对路径。调参循环靠它给每个候选喂不同配置 |
| `MB_STATION_LAYOUT` | `horizontal` | 6 关节 launch | 书写面布局。6 关节水平、7 关节竖直板 |
| `MB_BASE_RADIUS` | `0.0` | 6 关节 launch | 基座摆动半径。**实物基座固定在桌边，所以是 0** |
| `MB_BASE_Z_AMP` | `0.0` | 6 关节 launch | 基座竖直摆幅 |
| `MB_HWTEST_SPEED` | `0.01` | 6 关节 launch | `hwtest` 圆的**切向速度 m/s**。只改周期不改半径：`0.01` → 25.1 s/圈 |
| `MB_HWTEST_RADIUS` | `0.04` | 轨迹模块 | `hwtest` 圆半径 m。launch 不设，用模块默认 |

⚠️ `MB_HWTEST_SPEED` 的**模块默认是 `0.02`**，6 关节 launch 把它压到 `0.01`。
绕过 launch 直接跑轨迹模块时会拿到 `0.02`（周期减半）。

> ⚠️ `MB_HWTEST_SPEED` 同时压缩**交接拉入段**的时间轴。2026-08-04 实测：起步位形离
> 轨迹起点 78 mm 时，速度从 `0.01` 调到 `0.015`，拉入段 j1 峰值速度从 0.25 涨到
> 3.04 rad/s（**12 倍**），撞包线触发看门狗 —— 而那一段参考轨迹**根本没动**。
> 要提速就先让机械臂从轨迹起点附近开机。

### 3.5 RViz 里看什么

固定坐标系 `world`。绿线是期望路径，红线（较粗）是实际路径，绿 / 红球是当前目标与末端。
基座通过 `world → base_link` TF 运动。

实时话题：`/task_space_smc/tracking`、`/task_space_smc/joint_torque`。
每次运行在 `data_dir` 下写带时间戳的 CSV、JSON summary 和 PNG 报告。

---

## 4. 实物启动

**只有 6 关节构型有实物。** 实物是 6 个关节，固件的第 7 通道是幽灵数据
（position −6.5231 rad、velocity 27.9731 rad/s，超限位、超电机上限 5.6 倍），
靠 `arm_version:=gamma6` 屏蔽，已写死在 `student_arm6.launch.py` 里。

### 4.0 三条不能绕的安全约束

1. **协议里没有使能 / 失能帧。软件层没有急停通道，急停只能物理断电。**
2. **驱动进程一停，电机立刻失去保持力矩，机械臂会软掉。** 起了就不要关。
3. **上电即锁当前位姿，掰不动。** 起步位形离实验起点多远都不用管，
   SMC 运行的前几秒会自己把它开过去（[4.3](#43-起-smc就位与跟踪是同一次运行)）。

工作台约束（本台）：桌面在 `z = 0`，与基座安装面同平面；**+x 方向无桌面**，−x 有；
桌上围栏沿 −x 延伸、+y 到约 `y = 350 mm`、高度约"基座到 joint3"。

### 4.1 上机前检查

```bash
# 1) 有没有遗留 ROS 进程（仿真 launch 自带 student_arm_node，会和硬件驱动并存）
ps -eo pid,lstart,cmd | grep -E 'ros2 launch|student_arm_node|robot_state_publisher' | grep -v grep

# 2) 串口在不在。插拔后设备号会变，稳定路径用 by-id
ls -l /dev/ttyUSB*
ls -l /dev/serial/by-id/

# 3) 只读核对实物位形
python3 src/arm-platform/task_space_smc/hw/pose_check_readonly.py --offline-selftest
python3 src/arm-platform/task_space_smc/hw/pose_check_readonly.py --port /dev/ttyUSB0 --seconds 3
```

`pose_check_readonly.py` **结构上不可能发字节**：打开串口后把 `write` / `flush` 全替换成
抛异常的桩，任何代码路径想发送都会当场崩掉而不是悄悄发出去。它不 import `rclpy`、不发话题。
判据是 0.05 rad（≈2.9°）—— 各关节摩擦死区 0.020~0.059 rad，小于死区的偏差本来就消不掉。

> **读实物真值只信它。** `/joint_states` 可能有第二个发布者 —— 实测出现过间歇性的、
> 相差 74° 的位姿读数。任何物理上不可能的跳变，**先查数据源，不要查动力学**。
> 判断机械臂状态还要**看摄像头**：关节读数本身可能就是错的那一环。

### 4.2 起驱动

驱动和 SMC 是**两个终端、两条 launch**。先起驱动，**之后全程不关**。

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42
cd /home/windylab/code/windylab_ws/src/arm-platform

ros2 launch manipulator student_arm6.launch.py arm_type:=a_l1 \
  port_name:=/dev/ttyUSB0 control_rate_hz:=200.0 \
  enable_direct_effort_command:=true max_velocity:=0.5 use_rviz:=false
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `arm_type` | `sim` | `sim` 虚拟臂 / **`a_l1` 真机** |
| `port_name` | `/dev/ttyUSB0` | 本机 CP2102；装机到 NX 时为 `/dev/ttyTHS3` |
| `control_rate_hz` | `100.0` | 见下面两道上限 |
| `max_velocity` | `0.5` | 关节最大速度 rad/s。**首次上机用 `0.2`** |
| `enable_direct_effort_command` | `False` | 启用 0x12 力矩前馈通路。**跑 SMC 必须 `true`** |
| `publish_joint_feedback` | `True` | 发布 `/student/joint_feedback`，标定读 0x22 力矩要用 |
| `step_on_effort_command` | `False` | 收到力矩指令即步进一次控制周期（锁步仿真用） |
| `use_rviz` | `True` | 实物跑 SMC 时设 `false`，RViz 交给 SMC 那条 launch |

> **控制频率有两道限制，不要混淆：**
> ```
> 位置模式 ≤ 100 Hz   ExecuteControlCycle 对 dt < 0.01 直接抛异常
> 力矩模式 ≤ 200 Hz   板子实测上报 200.6 Hz，再快就有周期拿不到新反馈
> ```

### 4.3 起 SMC（就位与跟踪是同一次运行）

**没有独立的就位步骤，也不用位置模式。** 上电后机械臂锁在固定的初始位形，
通常离实验起点很远（实测断电重启后 j2 差 +66.7°、j4 +76.7°、j6 −88.1°，
末端离轨迹起点 200 mm）。这段距离由 SMC 运行的**前 `start_delay_sec` 秒**走完
—— 就位段与跟踪段是**同一个控制器、同一套增益、同一条代码路径**。

另开一个终端，**同一个 `ROS_DOMAIN_ID`**：

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42
export WINDYLAB_SMC_CONFIG=$PWD/src/arm-platform/task_space_smc/config/smc_params_hwtest.yaml
export MB_HWTEST_SPEED=0.01
cd /home/windylab/code/windylab_ws/src/arm-platform

ros2 launch manipulator task_space_smc_whole6.launch.py \
  mode:=hardware hardware_enabled:=true rviz:=false \
  trajectory_shape:=hwtest duration_sec:=60 start_delay_sec:=8 \
  data_dir:=/path/to/run
```

**与仿真的差别只有两个参数**，其余含义见 [3.2](#32-6-关节)：

| 参数 | 实物取值 | 为什么 |
|---|---|---|
| `mode` | `hardware` | 不自动起硬件节点，接你在 [4.2](#42-起驱动) 起好的那个 |
| `hardware_enabled` | `true` | **不设的话适配器输出恒为零，机械臂不动** |
| `start_delay_sec` | `8`（按距离调） | 就位段时长。**不能是 0** —— 0 等于跳过就位直接跟踪 |

#### 就位段做什么

`smc_params_hwtest.yaml` 里 `approach.mode: task_space`：

1. **第一个控制拍**读实物当前位形，正运动学得到末端当前位姿
2. 规划一条到轨迹起点的**末端直线**（位置）+ **SO(3) 测地线**（姿态），
   时间用五次 smoothstep，两端速度与加速度都为零
3. 这条参考量喂给**跟踪段那个同一个 Lagrange-SMC**，力矩模式执行

参考量从**当前位姿**起步，所以 SMC 看到的初始误差是 0；直接把参考量设成轨迹起点
的话第一拍就有 245 mm 误差乘 c₀=216 → 53 m/s² 的需求，实机上就是一次猛冲。

启动日志会打出规划量，**跑之前先看这一行**：

```
就位（任务空间 SMC）: 末端直线 245.2 mm、姿态 64.89°，8.0 s 走完；
峰值速度 0.0575 m/s、峰值加速度 0.022 m/s²
```

**为什么不用位置模式**（`full_prepose.py`）：它不经过 SMC，是另一套控制器，而且
顶不过静摩擦 —— 实测 j3 在 kp=80 下仍差约 **2°**、j2 差约 **10°**（j2 静摩擦
1.775 Nm 是其余关节的 2.4~18 倍，同时承载最重）。于是每次实验的**起步位形都不是
设定的那个**，末端差约 10 mm，跨天对比失真。力矩模式没有这个问题：仿真同参对照下
就位末端残差 **0.052 mm**。

**为什么在末端空间规划而不是关节空间**：关节空间插值时末端走的是一条没人算过的
曲线，可能扫过桌面或围栏。末端直线是显式的，可以事先检查。

#### 工作台护栏

规划完成后自动检查，任一条不过就**一条力矩都不发**（电机由驱动的指令超时保位接管，
那是协议里唯一设计过的安全兜底），并在日志里打 ERROR：

| 判据 | 缺省阈值 |
|---|---|
| 末端直线在 −x 侧离台面的间隙 | ≥ 80 mm |
| 撞围栏（x<20 mm 且 y>370 mm 且 z<331 mm） | 禁止 |
| 峰值线速度 | ≤ 0.15 m/s |
| 峰值线加速度 | ≤ 0.60 m/s² |

阈值在 `approach.workspace_guard` 里改。超速 / 超加速度时提示**加大
`start_delay_sec`**，不要去放宽阈值。

> ⚠️ 护栏**只查末端点**。连杆本体的扫掠体积没查 —— 末端空间规划沿途没有关节解，
> 算不出连杆在哪。这是必要条件不是充分条件，**第一次用新起点时人要在旁边看着**。

#### 手动就位工具（不在正常流程里）

`hw/full_prepose.py`（整臂位置模式）与 `hw/wrist_prepose.py`（只动 j5）保留，
用于 SMC 起不来时把机械臂挪到安全位形，或者做位置模式的对照实验。

```bash
python3 task_space_smc/hw/full_prepose.py \
  --target-rad="-0.1081,0.2375,-0.7410,-1.3375,0.6976,1.5400" \
  --duration 20 --settle 3 --kp="30,30,80,30,30,30"
```

内置校验：目标越限拒绝、单轴行程 > 2 rad 拒绝、峰值速度 > 0.5 rad/s 拒绝、
到位后读回残差 > 3° 报错。**它有上面说的 2°/10° 到不了位的问题**，
不要拿它的落点当实验起点。

### 4.4 安全看门狗

**就位段与跟踪段都在跑**，任一条触发就**停止跟踪并原地保持位置**：

| 判据 | 阈值 |
|---|---|
| 关节速度 | 超包线 `[3, 3, 3, 5, 5, 5]` rad/s 的 **1.2 倍** |
| 末端位置误差 | > 50 mm |

判据吃的是**测量值**，不是估计器输出 —— 估计器在这条路上是被测对象，
兜底不能有第二个失效点。日志里搜 `安全看门狗触发`。

### 4.5 收工

```bash
# 1) 先把机械臂挪到安全位形（位置模式）
python3 task_space_smc/hw/full_prepose.py --target-rad="<安全位形>" --duration 25

# 2) 再停驱动（Ctrl-C）。顺序不能反 —— 驱动一停就失去保持力矩
```

---

## 更多

| 内容 | 位置 |
|---|---|
| 控制律推导、三种构型、5D 笔轴任务定义 | [docs/REFERENCE.md](docs/REFERENCE.md) §1、§2、§8 |
| 离线仿真（不起 ROS） | [docs/REFERENCE.md](docs/REFERENCE.md) §4 |
| 实机标定体系（phase 1~5） | [docs/REFERENCE.md](docs/REFERENCE.md) §9 |
| 真机接口实测特性（延迟、噪声、摩擦、量化） | [docs/REFERENCE.md](docs/REFERENCE.md) §10 |
| 参数整定规则与已被实测否定的做法 | [docs/REFERENCE.md](docs/REFERENCE.md) §11 |
| 当前能力边界、常见问题 | [docs/REFERENCE.md](docs/REFERENCE.md) §13、§14 |
| 硬件当前状态与工作交接 | [docs/HANDOFF_2026_08_02.md](docs/HANDOFF_2026_08_02.md) |
