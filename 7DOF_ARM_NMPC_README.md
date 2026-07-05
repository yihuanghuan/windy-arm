# ACADOS NMPC IK 运行说明

当前 NMPC 做的是固定基 7 自由度机械臂的末端位置 MPC,可选末端姿态软约束。控制器最终发布的是 7 个关节角到 `/student/joint_command`。ACADOS 是外部依赖,仓库不会提交 ACADOS 源码、生成 C 代码或 build 产物。

## 1. 环境部署

> 环境部署(系统依赖、编译工作空间、安装 acados、环境自检)已统一到**总 [README.md](README.md) 的《六、部署步骤》**,并区分**一般电脑 / Jetson** 两类。请先按那里完成部署,再回到本文跑 demo。

部署完成后,**每个新终端**运行本文的 demo 前先激活环境:

```bash
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
```

验证 ACADOS Python 接口:`python3 -c "import acados_template; print('acados ok')"`。

## 2. 最小自测 — `mpc_pinocchio_ik.py`

进入 demo 目录:

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source ../scripts/setup_acados_env.sh
```

运行 NMPC 自测:

```bash
python3 mpc_pinocchio_ik.py  # 随机局部目标 MPC 回代验证
```

在你自己的代码中使用:

```python
import numpy as np
from mpc_pinocchio_ik import MpcPinocchioIK

ik = MpcPinocchioIK()
q_terminal, ok, info = ik.solve(np.array([0.4, 0.0, 0.2]))
q_next, ok, info = ik.step(np.array([0.4, 0.0, 0.2]), q_current=q_terminal)
```

如果希望同时软约束末端姿态,给配置设置非零姿态权重,再传入目标旋转矩阵:

```python
from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK

ik = MpcPinocchioIK(config=MpcIkConfig(
    orientation_weight=2.0,
    terminal_orientation_weight=20.0,
))
target_pos, target_rot = ik.forward(q_terminal)
q_next, ok, info = ik.step(target_pos, q_current=q_terminal, target_rotation=target_rot)
print(info["terminal_orientation_error"])
```

第一次运行 ACADOS 后端时会生成并编译 C 求解器,可能需要几秒到几十秒。后续在线求解通常是毫秒以内,这个首次编译时间不是每个控制周期都会发生。

ACADOS 求解器缓存规则:

- 第一次运行某个配置时,程序使用 `generate=True, build=True`,在 `MpcIkConfig.acados_build_dir` 下生成 `c_generated_code/`、`acados_ocp.json` 和 `acados_solver_metadata.json`。
- 后续同配置运行时,如果检测到 `c_generated_code/libacados_ocp_solver_*.so`、`acados_ocp.json` 和 `acados_solver_metadata.json` 都存在且配置指纹一致,程序使用 `generate=False, build=False`,直接加载已编译的求解器。
- 如果配置变了,例如 `horizon_steps`、`max_joint_step`、权重、姿态软约束开关、前桨避障开关、关键点数量、避障球数量、机身 box 数量、capsule 半径或 capsule 采样点数量变化,配置指纹会不一致,程序会自动重新生成并编译。
- 如果怀疑缓存损坏,可以删除对应的 `acados_build_dir` 目录,下次启动会重新生成。

## 3. 启动 demo

先启动学生机械臂仿真:

```bash
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
ros2 launch manipulator student_arm.launch.py
```

### 3.1 竖直圆 — `move_arm_mpc_ik_demo.py`

末端在 y-z 平面画半径 8 cm 的圆。

配合绘图程序查看 MPC 轨迹:

```bash
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=circle 

# 另开一个终端
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_arm_mpc_ik_demo.py
```

### 3.2 直线 — `move_arm_mpc_ik_demo.py`

末端在两点间沿直线平滑往返(余弦插值,端点速度为零)。

配合绘图程序查看 MPC 轨迹:

```bash
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=line \
  -p ignore_initial_sec:=2.0

# 另开一个终端
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_arm_mpc_ik_demo.py --ros-args -p trajectory_mode:=line
```

### 3.3 水平圆 — `move_arm_mpc_ik_demo.py 或 move_arm_horizontal_circle_mpc_ik_demo`

末端在 x-y 平面画半径 8 cm 的圆。

①直接启动
```bash
cd ~/code/windylab_ws/src/arm-platform/demo
python3 move_arm_horizontal_circle_mpc_ik_demo.py
```
②也可以直接用通用 MPC demo 切换模式:
```bash
python3 move_arm_mpc_ik_demo.py --ros-args -p trajectory_mode:=horizontal_circle
```

配合绘图程序查看 MPC 轨迹:

```bash
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=horizontal_circle \
  -p ignore_initial_sec:=2.0

# 另开一个终端
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_arm_mpc_ik_demo.py --ros-args -p trajectory_mode:=horizontal_circle
```

### 3.4 软锁姿态水平圆 — `move_arm_mpc_ik_demo.py 或 move_arm_horizontal_circle_mpc_ik_demo`

锁定起始末端姿态运行水平圆:

```bash
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=horizontal_circle \
  -p ignore_initial_sec:=2.0

# 另开一个终端
cd ~/code/windylab_ws/src/arm-platform/demo
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_arm_horizontal_circle_mpc_ik_demo.py --ros-args \
  -p lock_orientation:=true
```

水平圆并约束末端 x 轴竖直朝下:

```bash
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=horizontal_circle \
  -p ignore_initial_sec:=2.0

# 另开一个终端
cd ~/code/windylab_ws/src/arm-platform/demo
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_arm_horizontal_circle_mpc_ik_demo.py --ros-args \
  -p lock_orientation:=true \
  -p orientation_target:=vertical_down
```

其中 `vertical_down` 定义为:末端坐标系 x 轴对齐世界坐标 `-Z`,y 轴对齐世界坐标 `+Y`,z 轴对齐世界坐标 `+X`。

### 3.5 基座运动下的 world 系末端跟踪

`move_base_circle_visual_demo.py` 和 `move_base_circle_mpc_ik_demo.py` 都使用 `demo/moving_base_trajectory_config.py` 里的同一套轨迹参数。如果要修改轨迹,只改这个共享配置文件里的 `BASE_CIRCLE_RADIUS`、`BASE_HEIGHT`、`BASE_ATTITUDE_AMPLITUDE_DEG`,然后重新 build。

末端目标定义在 `world` 坐标系下。控制脚本会把 world 系末端目标变换到当前 `base_link` 系,再用普通 IK 或 MPC 求关节角。作图程序也从同一个共享配置读取 `p_wb(t)` / `R_wb(t)` / `p_ref_world(t)`,避免控制和作图参数不一致。

启动机械臂和 RViz:

```bash
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
ros2 launch manipulator student_arm.launch.py \
  max_velocity:=1.0 \
  velocity_feedforward_gain:=0.7 \
  position_feedback_tau:=0.15 \
  feedback_velocity_limit:=0.4
```

> 角速度控制(关节速度前馈)已默认开启,`enable_velocity_feedforward` 开关已移除,无需也不能再传。

RViz 左侧 `Global Options -> Fixed Frame` 建议切成 `world`。如果 Fixed Frame 仍是 `base_link`,视觉上会像是 `world`/网格在动,其实是观察坐标系固定在晃动基座上造成的相对运动效果。

另开终端启动控制 demo:

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_base_circle_mpc_ik_demo.py
```

再开一个终端用现有作图程序记录 world 系末端误差和控制信号:

```bash
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=moving_base_circle \
  -p frame_id:=world \
  -p actual_topic:=/joint_states \
  -p command_topic:=/student/joint_command \
  -p record_command_fk:=true \
  -p record_control_signal:=true \
  -p ignore_initial_sec:=3.0
```

MPC 版本可调参数:

| 参数 | 默认值 | 说明 |
|---|---|---|
| `solver_backend` | `acados` | 求解后端:`acados` |
| `horizon_steps` | `2` | MPC 预测步数 |
| `max_joint_step` | `0.02` | 单步最大关节变化量(rad) |
| `terminal_tolerance` | `0.01` | 终端位置误差 warning 阈值(m) |
| `orientation_weight` | `0.375` | 时域内姿态软约束权重 |
| `terminal_orientation_weight` | `5.0` | 终端姿态软约束权重 |
| `acados_rti_iterations` | `3` | 每帧 SQP_RTI 重复次数 |

MPC 版本会向 `/student/joint_command` 同时发布 `position` 和 `velocity`。速度由本帧 MPC 结果计算:

```text
dq_cmd = (q_next - q_current) / control_dt
```

`student_arm_node` 里的 `SmoothPositionController` 会继续对位置做平滑限速,并把 velocity 作为速度前馈传到底层,但速度会按 launch 参数 `max_velocity` 限幅。因此比较 IK/MPC 时,要让 `max_joint_step` 与底层 `max_velocity * control_dt` 大致匹配。

`moving_base_circle` 模式下作图程序使用:

```text
actual_world(t) = p_wb(t) + R_wb(t) @ FK_base(q)
expected_world(t) = p_ref_world(t)
```

因此 CSV 和曲线里的误差是 world 系末端误差。如果图里记录到了复位阶段,把 `ignore_initial_sec` 调大一点。
控制 demo 会在 `/student/joint_command` 的 `header.frame_id` 中标记 `moving_base_reset` / `moving_base_track`;作图程序在 `moving_base_circle` 模式下只从 `moving_base_track` 开始记录。
`record_command_fk:=true` 会额外保存 `*_command_fk.csv/png`,用于对比“命令关节角对应的末端轨迹”和 `/joint_states` 反馈轨迹。

## 4. 作图模块 — `trajectory_visualizer_demo.py`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `trajectory_mode` | `circle` | 期望轨迹模式:`circle` / `horizontal_circle` / `line` / `moving_base_circle`,要和运动 demo 保持一致 |
| `frame_id` | `base_link` | RViz Marker 使用的坐标系 |
| `actual_topic` | `/joint_states` | 实际关节状态输入 topic |
| `command_topic` | `/student/joint_command` | 轨迹命令 topic,用来判断轨迹开始 |
| `control_signal_topic` | `/student/joint_command` | 控制信号记录 topic |
| `record_control_signal` | `False` | 是否记录控制信号 CSV/图 |
| `record_command_fk` | `True` | 是否把 `/student/joint_command` 的关节角做 FK,记录命令末端轨迹 CSV/图 |
| `publish_rate_hz` | `10.0` | RViz Marker 发布频率 |
| `max_actual_points` | `2000` | 最多保留的实际轨迹点数 |
| `expected_path_points` | `240` | 期望轨迹离散点数 |
| `show_matplotlib` | `True` | 是否打开 Matplotlib 实时曲线窗口 |
| `start_on_command` | `True` | 是否等收到命令后再开始记录 |
| `use_message_stamp` | `True` | 是否使用消息时间戳计算轨迹时间 |
| `reset_gap_sec` | `2.0` | 命令间隔超过该值时重置记录 |
| `stale_command_timeout_sec` | `0.2` | 命令超时后暂停误差记录 |
| `sample_only_on_actual_change` | `True` | 只有实际关节变化时才采样 |
| `actual_change_epsilon` | `1e-9` | 实际关节变化判定阈值 |
| `control_signal_change_epsilon` | `1e-12` | 控制信号变化判定阈值 |
| `ignore_initial_sec` | `1.0` | 开始后忽略前多少秒数据 |
| `data_dir` | `/home/windylab/code/windylab_ws/src/arm-platform/demo/data` | CSV/PNG 保存目录 |


## 5. MPC IK 模块 — `move_arm_mpc_ik_demo.py/move_arm_horizontal_circle_mpc_ik_demo.py`

参数介绍：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `trajectory_mode` | `circle` | 运动轨迹:`circle` 垂直圆 / `horizontal_circle` 水平圆 / `line` 直线 |
| `terminal_tolerance` | `0.01` | MPC IK 末端位置误差容忍值,单位 m |
| `solver_backend` | `acados` | 求解器后端:`acados` |
| `horizon_steps` | `2` | MPC 预测步数 |
| `max_joint_step` | `0.02` | 每个控制周期最大关节步长,单位 rad |
| `lock_orientation` | `False` | 是否启用末端姿态软约束 |
| `orientation_target` | `initial` | 姿态目标:`initial` 保持初始姿态 / `vertical_down` 末端朝下 |
| `orientation_weight` | `0.375` | 姿态约束权重,仅 `lock_orientation:=true` 时生效 |
| `terminal_orientation_weight` | `5.0` | 终端姿态约束权重,仅 `lock_orientation:=true` 时生效 |

### 5.1 核心文件介绍

- `move_arm_mpc_ik_demo.py`:核心启动 demo —— 内部生成期望末端轨迹、按 50 Hz 调用 MPC 求解、把关节命令发布到 `/student/joint_command`,并负责 reset→track 切换与求解失败降级。
- `move_arm_horizontal_circle_mpc_ik_demo.py`:上面那个节点类的水平圆封装(同一个类,仅把默认 `trajectory_mode` 设为 `horizontal_circle`)。
- `MpcPinocchioIK`(`mpc_pinocchio_ik.py`):NMPC 公式与求解器调用(Pinocchio 从 URDF 建模 + acados 后端)。
- 期望轨迹:固定臂没有单独的轨迹发布节点,`circle`/`horizontal_circle`/`line` 由 demo 内部的 `_target()` 直接生成。


### 5.2 话题的使用

- **MPC 接受的当前状态话题**:订阅 `/joint_states` 中的 `joint1`~`joint7`,作为 NMPC 闭环求解的当前状态(求解起点)。
- **MPC 接收的期望状态话题**:无 —— demo 内部直接生成笛卡尔轨迹(`circle`/`horizontal_circle`/`line`),不订阅外部期望话题。
- **MPC 输出的控制量话题**:发布 `/student/joint_command`(`sensor_msgs/JointState`,`position` = 下一拍关节角 `q_next`,`velocity` = 关节速度前馈 `dq_cmd`)。

## 6. 调参

统一参数闭环离线调参脚本(固定臂 + 浮动基座臂四场景, 一套参数):

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source ../scripts/setup_acados_env.sh
python3 tune_unified_mpc_params.py
```

详细说明见 `MPC_TUNING_README.md`。

## 7. 常见问题

### 找不到 `acados_template`

先 source 环境:

```bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
```

如果仍失败,运行:

```bash
cd ~/code/windylab_ws/src/arm-platform
scripts/check_nmpc_env.sh
```

### 找不到 `libacados.so`

通常是没有 source `setup_acados_env.sh`,或者 ACADOS 没有 `make install`。修复:

```bash
cd ~/code/windylab_ws/src/arm-platform
scripts/install_acados.sh
source scripts/setup_acados_env.sh
```

### 第一次运行很慢

ACADOS 第一次会生成 C 代码并编译求解器。这个是启动/建模开销,不是每帧在线求解开销。只要 `horizon_steps`、权重维度、模型结构等配置不变,后续会复用生成目录。

### 端载设备上不够快

先降 `horizon_steps`,例如:

```bash
python3 move_arm_mpc_ik_demo.py --ros-args -p horizon_steps:=4
```

再根据跟踪误差调整 `max_joint_step` 和姿态权重。机械臂 demo 默认 50 Hz,每个控制周期是 20 ms。

### ACADOS 后端失败

本仓库已裁剪为只部署 acados 后端。先确认已完成部署并激活环境:

```bash
cd ~/code/windylab_ws/src/arm-platform
source scripts/setup_acados_env.sh
scripts/check_nmpc_env.sh
```
