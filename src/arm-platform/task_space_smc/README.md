# 浮动基座任务空间滑模控制 (SMC)

A-L1-GAMMA 机械臂的任务空间力矩控制。控制律在世界系里算末端位姿误差，经动力学一致伪逆
映射成关节力矩；基座可以是运动的（浮动基座），臂在其上补偿。


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
| `task_space_smc.py` | 滑模控制律本体。`ν = ẍ_r + Λė + K·s + η·tanh(s/φ)` |
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
| `whole_task_smc_ros_runner.py` | 主运行器。同时支持 `mode:=simulation` 与 `mode:=hardware` |
| `full_prepose.py` | 整臂位置模式就位（6 轴一起挪到指定位形） |
| `wrist_prepose.py` | 只把 joint5 挪离腕部奇异 |
| `pose_check_readonly.py` | 上机前只读位形核对，结构上不可能发字节 |
| `vicon_floating_base_bridge.py` | VRPN 刚体 pose/twist/accel + `arm_ee` → `FloatingBaseState`，源时钟打戳、杠杆臂搬运、一致性对照 |
| `fk_gravity_verify.py` | 阶段 1：正运动学 / 关节零位校验 |
| `merge_fk_verify.py` | 合并多次阶段 1 结果，输出零位标定向量 |
| `phase2_torque_probe.py` | 阶段 2：0x12 力矩通路验证 |
| `phase3_gravity_calibration.py` | 阶段 3：迭代重力标定 |
| `phase4_gravity_compensation.py` | 阶段 4：整臂重力补偿（分级降 kp 到"悬浮"） |
| `phase5_inertia_id.py` | 阶段 5：惯量辨识 |
| `serial_tap.py` | 双向串口抓包（伪终端透明中继） |
| `torque_interface_probe.py` | 力矩通路小幅探针 |

> `whole_task_smc_ros_runner.py` 放在 `hw/` 是因为它面向真机，但它**同时跑仿真和实物，
> 而且刻意是同一条控制代码路径**。控制律和机器人模型共享；仿真专用 launch 另行管理
> 起始位姿、反馈通路和交接设置，避免把实物上机参数带进仿真。

### 1.4 `config/` 与 `data/`

`config/` 不跟着分层，`core/config_loader.py` 按包根定位它。

| 配置 | 用于 |
|---|---|
| `smc_params_whole.yaml` | 7 关节仿真 |
| `smc_params_whole6.yaml` | 6 关节仿真；`simulation_profiles` 内区分 `hwtest` / `whole` / `hold` |
| `smc_params_hwtest.yaml` | 6 关节实物。上机默认用这份 |
| `smc_params_hold.yaml` | 6 关节实物 Vicon hold。只供 `vicon_hold_demo.launch.py` 使用 |

`data/` 是每次运行的 CSV / JSON / PNG，已 gitignore，不入库。

### 1.5 测试

`test/core/`（19）、`test/sim/`（2）、`test/hw/`（5），与源码同构。
`test/` 根只留 `conftest.py` 和 C++ 包自己的 `.cc`。

```bash
cd /home/windylab/code/windylab_ws
python3 -m pytest src/arm-platform/test -q      # 应为 549 passed
```

`conftest.py` 做两件事：把包父目录插进 `sys.path`；在任何测试模块导入前设好
`MB_STATION_LAYOUT=horizontal`（轨迹模块在模块级读它，谁先 import 谁定布局）。

### 1.6 运行入口、ROS 话题与信息流

下图中的 `runner` 都是 `whole_task_smc_ros_runner.py`。它输入关节状态和（按模式
选择的）基座 / 末端状态，输出关节力矩；RViz 只订阅可视化信息，不参与控制。

ROS 中以 `/` 开头的名字是**话题**，即节点之间传递数据的通道；
`sensor_msgs/JointState`、`dummy_interface/MotorControl` 等是**消息类型**，即该通道中
每条数据的字段格式，不是话题名。

#### 1.6.1. RViz 仿真：6 关节与 7 关节

| 入口 | 机械臂 | 控制频率 | 被控对象 |
|---|---:|---:|---|
| `task_space_smc_whole.launch.py` | 7 关节 | 500 Hz | `student_arm_node` 的 `torque_sim` |
| `task_space_smc_whole6_sim.launch.py` | 6 关节 | 200 Hz | `student_arm_node` 的 `torque_sim` |

两条仿真的实时闭环相同：

```text
runner -- /student/joint_effort_command --> student_arm_node (torque_sim)
runner <-- /joint_states ------------------- student_arm_node

/joint_states --> robot_state_publisher --> /tf, /tf_static --> RViz
runner --> /task_space_smc/reference_markers --> RViz
runner --> /task_space_smc/markers --> RViz
runner --> /task_space_smc/tracking --> 终端记录与 CSV
runner --> /task_space_smc/joint_torque --> 终端记录与 CSV
```

`/student/joint_effort_command` 是 runner 发给机械臂的关节力矩命令；它使用项目自定义的
`dummy_interface/MotorControl`，字段包括各关节的目标位置、速度、电流（本控制器中为
力矩命令）、P/D 增益和使能位。`/joint_states` 是机械臂反馈的关节名称、位置、速度和
力矩；它使用 ROS 标准 `sensor_msgs/JointState`。

`/tf` 和 `/tf_static` 是机器人各坐标系之间的实时 / 固定坐标变换。
`/task_space_smc/reference_markers` 是期望轨迹标记，`/task_space_smc/markers` 是实际末端和
误差标记，二者都供 RViz 显示；`/task_space_smc/tracking` 是记录跟踪量的数值数组，
`/task_space_smc/joint_torque` 是记录计算关节力矩的数值数组，二者都使用
`std_msgs/Float64MultiArray`。

输入是 launch 选定的 URDF、SMC YAML、轨迹参数，以及仿真臂发布的
`/joint_states`；输出是给仿真臂的 `MotorControl` 力矩、TF、RViz 标记和运行记录。
仿真 launch 只启动 `torque_sim`，不启动实物驱动；`hardware_enabled:=false` 不会阻断
`mode:=simulation` 下发给仿真臂的力矩。

#### 1.6.2. demo1：固定基座的 6 关节实物轨迹跟踪

demo1 的硬件驱动由 `student_arm6.launch.py` 单独启动，
`task_space_smc_whole6.launch.py mode:=hardware` 只启动 runner 和可选 RViz：

```text
实物机械臂 --> student_arm_node --> /joint_states --> runner
runner --> /student/joint_effort_command --> student_arm_node --> 实物机械臂

/joint_states --> robot_state_publisher --> /tf, /tf_static --> RViz
runner --> /task_space_smc/reference_markers, /task_space_smc/markers --> RViz
runner --> /task_space_smc/tracking, /task_space_smc/joint_torque --> CSV
```

demo1 的基座状态来自固定基座 / 解析模型；末端实际位姿由关节状态经 FK 得到。
轨迹参考由 YAML 和 `trajectory_shape` 在 runner 内生成，不通过 ROS 参考话题输入。

#### 1.6.3. demo2：Vicon 基座浮动下的末端位置保持

demo2 在 demo1 的关节力矩闭环外，再接入动捕状态链路：

```text
Vicon --> vrpn_listener --> /vrpn/arm_base/{pose,twist,accel} --┐
                                                               ├--> vicon_floating_base_bridge
Vicon --> vrpn_listener --> /vrpn/arm_ee/{pose,twist,accel} ---┘
vicon_floating_base_bridge --> /floating_base/vicon --> runner
vicon_floating_base_bridge --> /vicon/arm_ee -------> runner

实物机械臂 --> /joint_states --> runner --> /student/joint_effort_command --> 实物机械臂
runner --> /task_space_smc/reference_markers, /task_space_smc/markers --> RViz
runner --> /task_space_smc/tracking, /task_space_smc/joint_torque --> CSV
```

`/vrpn/arm_base/pose`、`/vrpn/arm_base/twist`、`/vrpn/arm_base/accel` 分别是 Vicon 测得的
基座刚体位姿、线 / 角速度、线 / 角加速度；`/vrpn/arm_ee/pose`、`/vrpn/arm_ee/twist`、
`/vrpn/arm_ee/accel` 是末端刚体的同类原始数据。三者分别使用 ROS 标准的
`geometry_msgs/PoseStamped`、`TwistStamped`、`AccelStamped`：`Stamped` 表示消息带有采样时间戳。

`/floating_base/vicon` 是桥接后给控制器的基座完整状态；`/vicon/arm_ee` 是末端完整状态。
二者使用项目自定义的 `dummy_interface/FloatingBaseState`，在同一条消息内放入位姿、速度和
加速度，因此这三类量共享一个时间戳。

在当前 demo2 配置中，基座使用 Vicon 位姿、线速度、角速度、线加速度和角加速度；
末端使用 Vicon 位置与线速度，末端姿态及其导数仍由 FK 计算。保持目标在接管首帧
由 runner 捕获，随后 runner 根据这些反馈计算力矩并发给实物驱动。

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

两条 launch。7 关节和 6 关节是不同的机械臂构型，不是同一条链上的开关。

| | 7 关节 | 6 关节 |
|---|---|---|
| 仿真 launch | `task_space_smc_whole.launch.py` | `task_space_smc_whole6_sim.launch.py` |
| 实物 launch | — | `task_space_smc_whole6.launch.py` |
| URDF | `arm.urdf` | 配置决定；`hwtest` 当前为 `arm6.urdf` |
| 末端 frame | 自动探测 | 配置决定；`hwtest` 当前为 `link6` |
| 默认控制频率 | 500 Hz | 200 Hz |
| 对应实物 | 无 | 有 |

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

这条 launch 有**三种轨迹**：

| `trajectory_shape` | 内容 | 基座 | 力矩上限 | 周期 | 仿真 RMS |
|---|---|---|---|---|---|
| **`hwtest`（默认）** | +x 侧竖直小圆，半径 4 cm | **固定** | 额定 9 / 1.6 | 12.57 s | 0.0041 mm |
| `whole` | 五工作站完整任务（`link6`） | 解析摆动 | 2× 额定 18 / 3.2 | 73.04 s | 0.0060 mm |
| `hold` | 末端世界系**定点保持**（参考量恒定） | 注入正弦 + 阵风 | 额定 9 / 1.6 | 无（缺省 60 s） | 1.71 mm |

`hwtest` 是上实机用的：幅度小、余量大、放在**无桌面的 +x 侧**。
`whole` 是研发用的完整任务，基座带摆动、跨四个方位，难度高得多。
`hold` 是[第 5 节](#5-实物-demo基座浮动下的末端位置保持)那条动捕 demo 的**仿真干跑**：
实物上基座由动捕给，这里改由 `simulation_injection` 驱动，并保留基座估计器的
噪声与延迟 —— 动捕不是理想传感器，干跑不该假装它是。

```bash
cd /home/windylab/code/windylab_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=77

# 默认：小幅 hwtest 圆
ros2 launch manipulator task_space_smc_whole6_sim.launch.py \
  trajectory_shape:=hwtest duration_sec:=0.0 rviz:=true

# 完整五工作站任务
ros2 launch manipulator task_space_smc_whole6_sim.launch.py \
  trajectory_shape:=whole duration_sec:=0.0 rviz:=true

# 保持 demo 的干跑：基座在动，末端参考量不动
ros2 launch manipulator task_space_smc_whole6_sim.launch.py \
  trajectory_shape:=hold duration_sec:=12 rviz:=true
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `trajectory_shape` | `hwtest` | `hwtest` / `whole` / `hold` |
| `mode` | `simulation` | 见 [3.3](#33-三种-mode) |
| `start_delay_sec` | `0.0` | 仿真从轨迹起点开始，不经过实物靠近段 |
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
| `WINDYLAB_SMC_CONFIG` | 按 launch 选 | launch | 配置文件绝对路径。调参循环靠它给每个候选喂不同配置；**实物 hold 入口刻意忽略它** |
| `MB_STATION_LAYOUT` | `horizontal` | 6 关节 launch | 书写面布局。6 关节水平、7 关节竖直板 |
| `MB_BASE_RADIUS` | `0.0` | 6 关节 launch | 基座摆动半径。**实物基座固定在桌边，所以是 0** |
| `MB_BASE_Z_AMP` | `0.0` | 6 关节 launch | 基座竖直摆幅 |
| `MB_HWTEST_SPEED` | `0.02` | 6 关节 launch | `hwtest` 圆的**切向速度 m/s**。只改周期不改半径：`0.02` → 12.57 s/圈 |
| `MB_HWTEST_RADIUS` | `0.04` | 轨迹模块 | `hwtest` 圆半径 m。launch 不设，用模块默认 |

`MB_HWTEST_SPEED` 的模块默认与 6 关节 launch 默认均为 `0.02`。实机首次运行可
在启动前显式设为 `0.01`。

> ⚠️ 实机上提高 `MB_HWTEST_SPEED` 会同时压缩交接拉入段的时间轴。若起步位形离
> 轨迹起点较远，先让机械臂靠近起点，或显式使用 `MB_HWTEST_SPEED=0.01`。

### 3.5 RViz 里看什么

固定坐标系 `world`。绿线是期望路径，红线（较粗）是实际路径，绿 / 红球是当前目标与末端。
基座通过 `world → base_link` TF 运动。

实时话题：`/task_space_smc/tracking`、`/task_space_smc/joint_torque`。
每次运行在 `data_dir` 下写带时间戳的 CSV、JSON summary 和 PNG 报告。

---

## 4. 实物demo1：基座稳定下的末端轨迹跟踪

**只有 6 关节构型有实物。** 实物是 6 个关节，固件的第 7 通道是幽灵数据
（position −6.5231 rad、velocity 27.9731 rad/s，超限位、超电机上限 5.6 倍），
靠 `arm_version:=gamma6` 屏蔽，已写死在 `student_arm6.launch.py` 里。

### 4.0 三条不能绕的安全约束

1. **协议里没有使能 / 失能帧。软件层没有急停通道，急停只能物理断电。**
2. **上电即锁当前位姿，掰不动。** 起 SMC 后由同一条力矩控制路径从第一帧接管，
   通过 handover 参考偏置衰减把当前位姿并入轨迹（[4.4](#44-起-smc从第一帧接管)）。


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

### 4.2 起驱动（起了全程不关）

```bash
cd /home/windylab/code/windylab_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42

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

### 4.3 把 joint5 挪出腕部奇异

```bash
cd /home/windylab/code/windylab_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42

python3 src/arm-platform/task_space_smc/hw/wrist_prepose.py \
  --target-deg 40 --duration 20 --settle 3

```

| 参数 | 默认 | 说明 |
|---|---|---|
| `--target-deg` | `20` | j5 目标角度，允许 5~60。**用 40** |
| `--duration` | `10` | 移动时长 s |
| `--settle` | `2` | 到位后保持时长 s |


### 4.4 起 SMC（从第一帧接管）

```bash
cd /home/windylab/code/windylab_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42
export WINDYLAB_SMC_CONFIG=$PWD/src/arm-platform/task_space_smc/config/smc_params_hwtest.yaml
export MB_HWTEST_SPEED=0.01
cd src/arm-platform

ros2 launch manipulator task_space_smc_whole6.launch.py \
  mode:=hardware hardware_enabled:=true rviz:=false \
  trajectory_shape:=hwtest duration_sec:=60 start_delay_sec:=0 \
  data_dir:=/home/windylab/code/windylab_ws/src/arm-platform/task_space_smc/data/demo1_manual

```

| 参数 | 实物取值 | 为什么 |
|---|---|---|
| `mode` | `hardware` | 不自动起硬件节点，接你在 [4.2](#42-起驱动) 起好的那个 |
| `hardware_enabled` | `true` | **不设的话适配器输出恒为零，机械臂不动** |
| `start_delay_sec` | `0` | 当前靠 `handover` 冻结轨迹并衰减参考偏置；不要按旧文档改回 8 秒 approach。 |

`smc_params_hwtest.yaml` 里仍保留 `approach` 段，作为回退和历史对照。当前主线看
`handover.reference_blend_sec = 30.0` 与 `freeze_trajectory_during_blend = true`。

#### 工作台护栏【demo1需要有，demo2忽略】

工作台护栏按配置检查，任一条不过就**一条力矩都不发**，并在日志里打 ERROR：

| 判据 | 缺省阈值 |
|---|---|
| 末端直线在 −x 侧离台面的间隙 | ≥ 80 mm |
| 撞围栏（x<20 mm 且 y>370 mm 且 z<331 mm） | 禁止 |
| 峰值线速度 | ≤ 0.15 m/s |
| 峰值线加速度 | ≤ 0.60 m/s² |

阈值在 `approach.workspace_guard` 里改。不要为了跑通一次就放宽阈值。

> ⚠️ 护栏**只查末端点**。连杆本体的扫掠体积没查。这是必要条件不是充分条件，
> 第一次用新起点时人要在旁边看着。

#### 手动就位工具（不在正常流程里）

```bash
cd /home/windylab/code/windylab_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

python3 src/arm-platform/task_space_smc/hw/full_prepose.py \
  --target-rad="0.4571,-0.1430,-0.9966,0.7027,-0.6168,0.0178" \
  --duration 30 --settle 3

```

### 4.5 安全看门狗

**就位段与跟踪段都在跑**，任一条触发就**停止跟踪并原地保持位置**：

| 判据 | 阈值 |
|---|---|
| 关节速度 | 超包线 `[3, 3, 3, 5, 5, 5]` rad/s 的 **1.2 倍** |
| 末端位置误差 | > 50 mm |

判据吃的是**测量值**，不是估计器输出 —— 估计器在这条路上是被测对象，
兜底不能有第二个失效点。日志里搜 `安全看门狗触发`。

### 4.6 收工

```bash
# 1) Ctrl-C 停 SMC（终端 B）
# 2) 最后 Ctrl-C 停驱动（终端 A）
```

---

## 5. 实物 demo2：基座浮动下的末端位置保持

场景：基座随便动，末端不动。

动捕坐标系：y向上，joint1的转轴是x

### 5.1 基座固定偏置标定

#### 5.1.1 数据收集

用圆弧标定脚本 vicon_base_extrinsic_calibrate.py。确保没有任何 SMC/hold 控制器在发布 /student/joint_effort_command，只保留驱动和 Vicon。

终端 1：驱动（若已经在运行，不要重复启动）
```bash
cd /home/windylab/code/windylab_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch manipulator student_arm6.launch.py \
  arm_type:=a_l1 port_name:=/dev/ttyUSB0 \
  control_rate_hz:=200.0 \
  enable_direct_effort_command:=true \
  max_velocity:=0.5 use_rviz:=false
```

终端 2：Vicon（若已有 vrpn_listener 正在发布 /vrpn/arm_base/pose 和 /vrpn/arm_ee/pose，不用重复启动）
```bash
cd /home/windylab/code/windylab_ws
source /opt/ros/humble/setup.bash
source /home/windylab/code/vicon_perception_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch manipulator vicon_floating_base.launch.py \
  server:=172.16.20.21 port:=3883 zero_on_first_pose:=false
```

启动后，在另一个同样完成上述 source 的终端确认两路数据：
```bash
ros2 topic hz /vrpn/arm_base/pose
ros2 topic hz /vrpn/arm_ee/pose
```
两者稳定有频率后，可以继续执行命令

终端 3：先只读检查轨迹，不运动：
```bash
cd /home/windylab/code/windylab_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

python3 src/arm-platform/task_space_smc/hw/vicon_base_extrinsic_calibrate.py \
  --points 16 --plan-only
```

确认计划正常后，执行标定：
```bash
python3 src/arm-platform/task_space_smc/hw/vicon_base_extrinsic_calibrate.py \
  --points 16
```

它会低速扫 16 个静止位姿、每点采集 arm_base、arm_ee 和编码器数据，结束时回到起始姿态。结果保存在：
```bash
src/arm-platform/task_space_smc/hw/data/vicon_base_extrinsic/base_extrinsic_时间戳.json
```
运行中如发现姿态异常或人靠近机械臂，直接 Ctrl-C；脚本会停止扫轨迹并交回驱动的重力保位。

#### 5.1.2 标定





### 5.3 实物流程

#### 5.3.1 先清残留 + 查串口

断电会让原来的 student_arm_node 抛 SerialException 死掉，但 launch 空壳和孤儿 robot_state_publisher 会留着，必须先清干净（否则 /joint_states 上出现第二个发布者会给出物理上不可能的读数）。

```bash

pkill -f "ros2 launch manipulator"
pkill -f student_arm_node
pkill -f robot_state_publisher
pkill -f vrpn_listener
pkill -f vicon_floating_base_bridge
sleep 2
ps -ef | grep -E 'ros2 launch|student_arm_node|robot_state_publisher|vrpn' | grep -v grep    # 应为空

```

#### 5.3.2 只读核对位形

```bash
cd /home/windylab/code/windylab_ws
source /opt/ros/humble/setup.bash
python3 src/arm-platform/task_space_smc/hw/pose_check_readonly.py --port /dev/ttyUSB0 --seconds 3

```

#### 5.3.3 起驱动（终端 A）
```bash
cd /home/windylab/code/windylab_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch manipulator student_arm6.launch.py arm_type:=a_l1 \
  port_name:=/dev/ttyUSB0 control_rate_hz:=200.0 \
  enable_direct_effort_command:=true max_velocity:=0.5 use_rviz:=false

```

#### 5.3.4 joint5 出腕部奇异（终端 B）
```bash
cd /home/windylab/code/windylab_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

python3 src/arm-platform/task_space_smc/hw/wrist_prepose.py \
  --target-deg 40 --duration 20 --settle 3

```

#### 5.3.5 Vicon（终端 B）
```bash
# source /home/windylab/code/vicon_perception_ws/install/ros/setup.bash

# ros2 launch manipulator vicon_floating_base.launch.py \
#   server:=172.16.20.21 port:=3883 pose_topic:=/vrpn/arm_base/pose

cd /home/windylab/code/windylab_ws
source /opt/ros/humble/setup.bash
source /home/windylab/code/vicon_perception_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch manipulator vicon_floating_base.launch.py \
  server:=192.168.200.78 port:=3883 \
  zero_on_first_pose:=true \
  prefer_vicon_angular_accel:=true


```

#### 5.3.6 demo（终端 C）
```bash
cd /home/windylab/code/windylab_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch manipulator vicon_hold_demo.launch.py \
  hardware_enabled:=true \
  duration_sec:=86400 \
  data_dir:=/home/windylab/code/windylab_ws/src/arm-platform/task_space_smc/data/vicon_hold_manual

```

#### 5.3.6 收工（顺序不能反）
```bash
# 1) Ctrl-C 停 demo（终端 C）
# 2) Ctrl-C 停 Vicon（终端 B）
# 3) 最后 Ctrl-C 停驱动（终端 A）

```
