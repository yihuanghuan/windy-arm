# 机械臂学生使用说明

本文档面向使用 A-L1-GAMMA 7 自由度机械臂做实验的学生。你将学会:启动仿真/真实机械臂、通过 ROS 2 话题控制关节、使用逆运动学 (IK) 控制末端位置。

说明：目前机械臂均为6自由度，如果末端有电机则为7自由度，但是该框架兼容二者。
---

## 1. 环境要求

- Ubuntu 22.04 + ROS 2 Humble
- 系统依赖:

```bash
sudo apt install ros-humble-robot-state-publisher ros-humble-rviz2 \
                 ros-humble-pinocchio libeigen3-dev
```

- Python 依赖(运行 IK 示例需要):

```bash
pip3 install numpy scipy casadi
# pinocchio 的 python 绑定随 ros-humble-pinocchio 一起安装
```

## 2. 获取与编译

工作空间包含 3 个机械臂相关的包:

| 包 | 作用 |
|---|---|
| `manipulator` (arm-platform) | 机械臂控制框架,含学生接口节点 |
| `dummy_interface` | 自定义消息 `MotorState`(关节反馈) |
| `dummy_description` | 机械臂 3D 模型 (RViz 可视化用) |

```bash
cd ~/code/windylab_ws/src
# 如果还没有 serial 依赖包，先拉下来
[ -d serial ] || git clone https://github.com/ZhaoXiangBox/serial.git serial

cd ~/code/windylab_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select serial dummy_interface manipulator
source install/setup.bash
```

> 之后每开一个新终端,都要先执行:
> ```bash
> source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
> ```

## 3. 启动机械臂

### 3.1 仿真模式(默认,推荐先用)

```bash
ros2 launch manipulator student_arm.launch.py
```

会启动:
- `student_arm_node` — 学生控制接口(虚拟臂)
- `robot_state_publisher` + RViz — 3D 可视化

### 3.2 真实硬件模式

**务必先在仿真中验证你的程序,确认无误后再上真机!**

```bash
# 确认串口设备(通常是 /dev/ttyUSB0),并确保有读写权限
sudo usermod -aG dialout $USER   # 首次需要,执行后重新登录

ros2 launch manipulator student_arm.launch.py arm_type:=a_l1 port_name:=/dev/ttyTHS3
```

无人机上默认使用 `/dev/ttyTHS3` 串口（如果按照SOP安装将其连接到NX上），如果你是其他串口（比如说连接到自己的电脑上），需要修改 `port_name` 参数。

### 3.3 launch 参数一览

| 参数 | 默认值 | 说明 |
|---|---|---|
| `arm_type` | `sim` | `sim` 仿真 / `a_l1` 真机 |
| `port_name` | `/dev/ttyUSB0` | 真机串口(仿真模式忽略) |
| `max_velocity` | `0.5` | 关节最大速度 (rad/s),安全限速 |
| `use_rviz` | `True` | 是否启动 RViz |

## 4. 话题接口

### 4.1 控制指令(你发布)

**话题:** `/student/joint_command`
**类型:** `sensor_msgs/msg/JointState`

| 字段 | 要求 |
|---|---|
| `position` | **必填**,长度必须为 7,单位 rad |
| `velocity` | 选填,前馈速度,单位 rad/s |
| `name` | 选填,`joint1` ~ `joint7` |

命令行快速测试:

```bash
ros2 topic pub --once /student/joint_command sensor_msgs/msg/JointState \
  "{position: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

### 4.2 状态反馈(你订阅)

| 话题 | 类型 | 内容 |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 当前关节位置/速度(RViz 也用它) |
| `/student/joint_feedback` | `dummy_interface/MotorState` | 电机位置/速度/电流/电压/温度(需启动时设参数 `publish_joint_feedback:=true`) |

### 4.3 关节限位

| 关节 | joint1 | joint2 | joint3 | joint4 | joint5 | joint6 | joint7 |
|---|---|---|---|---|---|---|---|
| 下限 (rad) | -4.0 | -3.14 | -1.57 | -4.0 | -2.0 | -4.0 | -4.0 |
| 上限 (rad) | 4.0 | 2.0 | 1.57 | 4.0 | 2.0 | 4.0 | 4.0 |

### 4.4 内置安全层(节点自动执行,无需你处理)

1. **限位钳制** — 超限指令被钳到边界并警告
2. **非法指令拒收** — 含 NaN/Inf 或维度 ≠ 7 的指令直接丢弃
3. **超时保持** — 超过 1 s 没收到新指令,机械臂保持当前位置
4. **限速** — 关节速度被限制在 `max_velocity` 以内

> 安全层只是兜底。上真机时仍要从小幅度、低速度开始测试。

## 5. 示例程序

示例位于 `src/arm-platform/demo/`,按由浅入深的顺序学习。运行前确保 `student_arm.launch.py` 已启动。

### 5.1 入门:关节空间摆动 — `move_arm_demo.py`

前 3 个关节做小幅正弦摆动,演示最基本的指令发布。

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
python3 move_arm_demo.py
```

### 5.2 IK 模块 — `pinocchio_ik.py`

基于 Pinocchio 的逆运动学求解器(阻尼最小二乘迭代),供其他脚本调用,也可单独运行自测:

```bash
python3 pinocchio_ik.py    # 随机 10 点 FK/IK 回代验证
```

在你自己的代码中使用:

```python
import numpy as np
from pinocchio_ik import PinocchioIK

ik = PinocchioIK()
q, ok = ik.solve(np.array([0.4, 0.0, 0.2]))  # 目标末端位置 (base_link 系, m)
pos, rot = ik.forward(q)                      # 正解验证
```

说明:
- 末端 frame 为 `link7`,坐标系原点在 `base_link`
- 默认只解 3D 位置(本臂 joint6/joint7 轴线共线,末端姿态存在病态方向)
- `q_init` 传入上一次的解可热启动,轨迹跟踪时保证关节连续

### 5.3 进阶:IK末端画圆 — `move_arm_ik_demo.py`

末端在 y-z 平面画半径 8 cm 的圆,演示笛卡尔空间轨迹 + IK。

```bash
python3 move_arm_ik_demo.py
```
新增绘图程序

```bash
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=circle \
  -p actual_topic:=/student/joint_command \
  -p ignore_initial_sec:=2.0

# 另开一个终端
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
python3 move_arm_ik_demo.py
```

### 5.4 进阶:IK末端直线往返 — `move_arm_line_demo.py`

末端在两点间沿直线平滑往返(余弦插值,端点速度为零)。

```bash
python3 move_arm_line_demo.py
```

新增绘图程序

```bash
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=circle \
  -p actual_topic:=/student/joint_command \
  -p ignore_initial_sec:=2.0

# 另开一个终端
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
python3 move_arm_line_demo.py
```

可修改文件顶部的 `POINT_A` / `POINT_B` / `PERIOD_SEC` 改变运动范围与速度。


### 5.5 作图模块 — `trajectory_visualizer_demo.py`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `trajectory_mode` | `circle` | 期望轨迹模式:`circle` / `horizontal_circle` / `line` / `moving_base_circle`,要和运动 demo 保持一致 |
| `frame_id` | `base_link` | RViz Marker 使用的坐标系 |
| `actual_topic` | `/joint_states` | 实际关节状态输入 topic,MPC 使用 `/student/joint_command` |
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

### 5.6 进阶:基座运动/固定基座下的 world 系末端跟踪

`move_base_circle_visual_demo.py` 和 `move_base_circle_mpc_ik_demo.py` 都使用 `demo/moving_base_trajectory_config.py` 里的同一套轨迹参数。当前默认是基座不浮动;如果要恢复基座圆周晃动,只改这个共享配置文件里的 `BASE_CIRCLE_RADIUS`、`BASE_HEIGHT`、`BASE_ATTITUDE_AMPLITUDE_DEG`,然后重新 build。

末端目标定义在 `world` 坐标系下。控制脚本会把 world 系末端目标变换到当前 `base_link` 系,再用普通 IK 或 MPC 求关节角。作图程序也从同一个共享配置读取 `p_wb(t)` / `R_wb(t)` / `p_ref_world(t)`,避免控制和作图参数不一致。

启动机械臂和 RViz:

```bash
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
ros2 launch manipulator student_arm.launch.py
```

RViz 左侧 `Global Options -> Fixed Frame` 建议切成 `world`。如果 Fixed Frame 仍是 `base_link`,视觉上会像是 `world`/网格在动,其实是观察坐标系固定在晃动基座上造成的相对运动效果。

另开终端启动控制 demo:

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
python3 move_base_circle_visual_demo.py
```

如果要运行同一场景的 ACADOS MPC 版本,换成:

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_base_circle_mpc_ik_demo.py
```

MPC 版本可调参数:

| 参数 | 默认值 | 说明 |
|---|---|---|
| `solver_backend` | `acados` | 求解后端:`acados` / `casadi` / `scipy` |
| `horizon_steps` | `8` | MPC 预测步数 |
| `max_joint_step` | `0.08` | 单步最大关节变化量(rad) |
| `terminal_tolerance` | `0.01` | 终端位置误差 warning 阈值(m) |
| `orientation_weight` | `0.2` | 时域内姿态软约束权重 |
| `terminal_orientation_weight` | `5.0` | 终端姿态软约束权重 |
| `acados_rti_iterations` | `3` | 每帧 SQP_RTI 重复次数 |

MPC 版本会向 `/student/joint_command` 同时发布 `position` 和 `velocity`。速度由本帧 MPC 结果计算:

```text
dq_cmd = (q_next - q_current) / control_dt
```

`student_arm_node` 里的 `SmoothPositionController` 会继续对位置做平滑限速,并把 velocity 作为速度前馈传到底层,但速度会按 launch 参数 `max_velocity` 限幅。因此比较 IK/MPC 时,要让 `max_joint_step` 与底层 `max_velocity * control_dt` 大致匹配。

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

`moving_base_circle` 模式下作图程序使用:

```text
actual_world(t) = p_wb(t) + R_wb(t) @ FK_base(q)
expected_world(t) = p_ref_world(t)
```

因此 CSV 和曲线里的误差是 world 系末端误差。如果图里记录到了复位阶段,把 `ignore_initial_sec` 调大一点。
控制 demo 会在 `/student/joint_command` 的 `header.frame_id` 中标记 `moving_base_reset` / `moving_base_track`;作图程序在 `moving_base_circle` 模式下只从 `moving_base_track` 开始记录。
`record_command_fk:=true` 会额外保存 `*_command_fk.csv/png`,用于对比“命令关节角对应的末端轨迹”和 `/joint_states` 反馈轨迹。

### 5.7 MPC IK 模块 — `mpc_pinocchio_ik.py`

Pinocchio读取关节信息，然后从 URDF 读取 joint axis，生成 CasADi 符号 FK，最后用ACADOS求解。需要锁定末端姿态时,可以开启姿态软约束。默认使用 ACADOS 实时 NMPC 后端;CasADi `Opti + IPOPT` 和 SciPy SLSQP 仍作为回退后端保留。

```bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 mpc_pinocchio_ik.py    # 随机局部目标 MPC 回代验证
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

### 5.8 进阶:MPC IK轨迹 — `move_arm_mpc_ik_demo.py/move_arm_horizontal_circle_mpc_ik_demo.py`

二者参数一致,只有 `trajectory_mode` 默认值不同:`move_arm_mpc_ik_demo.py` 默认为 `circle`,`move_arm_horizontal_circle_mpc_ik_demo.py` 默认为 `horizontal_circle`。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `trajectory_mode` | `circle` | 运动轨迹:`circle` 垂直圆 / `horizontal_circle` 水平圆 / `line` 直线 |
| `terminal_tolerance` | `0.01` | MPC IK 末端位置误差容忍值,单位 m |
| `solver_backend` | `acados` | 求解器后端:`acados` / `casadi` / `scipy` |
| `horizon_steps` | `8` | MPC 预测步数 |
| `max_joint_step` | `0.08` | 每个控制周期最大关节步长,单位 rad |
| `lock_orientation` | `False` | 是否启用末端姿态软约束 |
| `orientation_target` | `initial` | 姿态目标:`initial` 保持初始姿态 / `vertical_down` 末端朝下 |
| `orientation_weight` | `2.0` | 姿态约束权重,仅 `lock_orientation:=true` 时生效 |
| `terminal_orientation_weight` | `20.0` | 终端姿态约束权重,仅 `lock_orientation:=true` 时生效 |

默认 ACADOS 后端需要先在系统外部安装 ACADOS。本机约定安装位置是 `/home/windylab/code/acados`,每次新开终端运行前 source 仓库里的环境脚本:

```bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh # 每次新开终端运行MPC之前都需要运行这个
python3 -c "import acados_template; print('acados ok')" #这个只是检查acados环境是否搭建完善
```

ACADOS 求解器会按配置缓存:
- 第一次运行某个配置时,程序会 `generate=True, build=True`,生成并编译 C 求解器。
- 后续同配置运行时,如果检测到 `libacados_ocp_solver_*.so`、`acados_ocp.json` 和 `acados_solver_metadata.json` 都存在且配置指纹一致,程序会 `generate=False, build=False`,直接复用已编译求解器。
- 如果配置变了,例如 `horizon_steps`、`max_joint_step`、姿态权重、前桨避障开关、关键点数量、避障球数量、机身 box 数量、capsule 半径或 capsule 采样点数量变化,配置指纹会不一致,程序会自动重新生成并编译。

环境变量配置完成后直接运行默认 ACADOS 版本:

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_arm_mpc_ik_demo.py
```

### 5.9 进阶:MPC 竖直圆 — `move_arm_mpc_ik_demo.py`

末端在 y-z 平面画半径 8 cm 的圆：

```bash
python3 move_arm_mpc_ik_demo.py
```

配合绘图程序查看 MPC 轨迹:

```bash
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=circle \
  -p actual_topic:=/student/joint_command \
  -p ignore_initial_sec:=2.0

# 另开一个终端
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_arm_mpc_ik_demo.py
```

### 5.10 进阶:MPC 直线 — `move_arm_mpc_ik_demo.py`

末端在两点间沿直线平滑往返(余弦插值,端点速度为零)。

```bash
python3 move_arm_mpc_ik_demo.py --ros-args -p trajectory_mode:=line
```

配合绘图程序查看 MPC 轨迹:

```bash
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=line \
  -p actual_topic:=/student/joint_command \
  -p ignore_initial_sec:=2.0

# 另开一个终端
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_arm_mpc_ik_demo.py --ros-args -p trajectory_mode:=line
```

### 5.11 进阶:MPC 水平圆 — `move_arm_mpc_ik_demo.py`

末端在 x-y 平面画半径 8 cm 的圆：

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
  -p actual_topic:=/student/joint_command \
  -p ignore_initial_sec:=2.0

# 另开一个终端
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_arm_mpc_ik_demo.py --ros-args -p trajectory_mode:=horizontal_circle
```

### 5.12 进阶:MPC 软锁姿态水平圆 — `move_arm_mpc_ik_demo.py`

锁定起始末端姿态运行水平圆:

```bash
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=horizontal_circle \
  -p actual_topic:=/student/joint_command \
  -p ignore_initial_sec:=2.0

# 另开一个终端
cd ~/code/windylab_ws/src/arm-platform/demo
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_arm_horizontal_circle_mpc_ik_demo.py --ros-args \
  -p horizon_steps:=4 \
  -p max_joint_step:=0.0600 \
  -p lock_orientation:=true \
  -p orientation_weight:=0.9375 \
  -p terminal_orientation_weight:=25.0000
```

末端竖直朝下运行水平圆:

```bash
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=horizontal_circle \
  -p actual_topic:=/student/joint_command \
  -p ignore_initial_sec:=2.0

# 另开一个终端
cd ~/code/windylab_ws/src/arm-platform/demo
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_arm_horizontal_circle_mpc_ik_demo.py --ros-args \
  -p horizon_steps:=8 \
  -p max_joint_step:=0.1100 \
  -p lock_orientation:=true \
  -p orientation_target:=vertical_down \
  -p orientation_weight:=0.2000 \
  -p terminal_orientation_weight:=5.0000
```

### 5.13 进阶:MPC 软锁姿态水平圆批量调参 — `tune_mpc_params.py`

批量调参可以使用 `demo/tune_mpc_params.py`,详细说明见 `demo/MPC_TUNING_README.md`。

针对末端竖直朝下姿态做离线调参:

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash
source install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 tune_mpc_params.py --orientation-target vertical_down
```

如果已有一份调参 CSV,可以从它的 Top 参数继续细搜索:

```bash
python3 tune_mpc_params.py \
  --orientation-target vertical_down \
  --fine-from-csv data/mpc_tuning_<时间戳>.csv \
  --fine-top 5 \
  --max-combinations 60
```

## 6. 模型文件

查看机械臂模型:

- URDF:`src/arm-platform/config/arm.urdf`(7 自由度,含关节限位,IK 模块直接读取)
- RViz 配置:`src/arm-platform/config/student_arm.rviz`

## 7. 常见问题

**Q: RViz 里模型不动?**
检查是否有节点在发布 `/student/joint_command`,以及指令 `position` 长度是否为 7。

**Q: 终端提示 "Rejected command"?**
指令维度不是 7,或包含 NaN/Inf。

**Q: 提示 "Command timeout, holding current position"?**
正常保护行为:你的程序超过 1 s 没发新指令。连续控制时建议以 ≥ 10 Hz 持续发布。

**Q: IK 不收敛 (ok=False)?**
目标点超出工作空间。本臂大致可达范围:距基座 0.2 ~ 0.6 m。把目标点移近些再试。

**Q: 真机串口打不开?**
确认 `ls /dev/ttyUSB*` 有设备、已加入 dialout 组(需重新登录)、launch 的 `port_name` 正确。

## 8. 真机操作注意事项

1. 先仿真,后真机
2. 上电前确认机械臂周围无人、无障碍物
3. 第一次运行新程序时,把 `max_velocity` 调小(如 0.2)
4. 随时准备急停(断电开关在手边)
5. 程序退出前让机械臂回到安全位姿
