# ACADOS NMPC IK 运行说明

当前 NMPC 做的是固定基 7 自由度机械臂的末端位置 MPC,可选末端姿态软约束。控制器最终发布的是 7 个关节角到 `/student/joint_command`。ACADOS 是外部依赖,仓库不会提交 ACADOS 源码、生成 C 代码或 build 产物。

## 1. 系统要求

- Ubuntu 22.04
- ROS 2 Humble
- Python 3
- Pinocchio Python 绑定
- NumPy、SciPy、CasADi
- ACADOS 和 `acados_template`

基础依赖:

```bash
sudo apt update
sudo apt install -y \
  git cmake build-essential python3-pip \
  ros-humble-pinocchio \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  libeigen3-dev

python3 -m pip install --user numpy scipy casadi
```

如果你的系统已经通过 ROS 包安装了 `numpy/scipy`,重复安装通常也没有问题。

## 2. 编译机械臂工作空间

```bash
cd ~/code/windylab_ws/src
[ -d serial ] || git clone https://github.com/ZhaoXiangBox/serial.git serial

cd ~/code/windylab_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select serial dummy_interface manipulator
source install/setup.bash
```

每个新终端运行 ROS demo 前都要 source:

```bash
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
```

## 3. 安装 ACADOS

仓库提供源码安装脚本,默认安装到:

```text
$HOME/code/acados
```

安装:

```bash
cd ~/code/windylab_ws/src/arm-platform
scripts/install_acados.sh
```

如果想安装到其他位置:

```bash
export ACADOS_SOURCE_DIR=/path/to/acados
scripts/install_acados.sh
```

安装完成后,每个新终端运行 NMPC 前 source 环境脚本:

```bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
```

验证:

```bash
python3 -c "import acados_template; print('acados ok')"
```

## 4. 环境检查

可以用检查脚本一次性确认依赖:

```bash
cd ~/code/windylab_ws/src/arm-platform
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
source scripts/setup_acados_env.sh
scripts/check_nmpc_env.sh
```

检查脚本只读环境,不会安装或修改系统。

## 5. 最小自测 — `mpc_pinocchio_ik.py`

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

## 6. 启动 demo

先启动学生机械臂仿真:

```bash
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
ros2 launch manipulator student_arm.launch.py
```

### 6.1 竖直圆 — `move_arm_mpc_ik_demo.py`

末端在 y-z 平面画半径 8 cm 的圆。

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

### 6.2 直线 — `move_arm_mpc_ik_demo.py`

末端在两点间沿直线平滑往返(余弦插值,端点速度为零)。

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

### 6.3 水平圆 — `move_arm_mpc_ik_demo.py 或 move_arm_horizontal_circle_mpc_ik_demo`

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
  -p actual_topic:=/student/joint_command \
  -p ignore_initial_sec:=2.0

# 另开一个终端
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_arm_mpc_ik_demo.py --ros-args -p trajectory_mode:=horizontal_circle
```

### 6.4 软锁姿态水平圆 — `move_arm_mpc_ik_demo.py 或 move_arm_horizontal_circle_mpc_ik_demo`

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

水平圆并约束末端 x 轴竖直朝下:

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

其中 `vertical_down` 定义为:末端坐标系 x 轴对齐世界坐标 `-Z`,y 轴对齐世界坐标 `+Y`,z 轴对齐世界坐标 `+X`。

## 7. 作图模块 — `trajectory_visualizer_demo.py`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `trajectory_mode` | `circle` | 期望轨迹模式:`circle` / `horizontal_circle` / `line`,要和运动 demo 保持一致 |
| `frame_id` | `base_link` | RViz Marker 使用的坐标系 |
| `actual_topic` | `/joint_states` | 实际关节状态输入 topic,MPC 使用 `/student/joint_command` |
| `command_topic` | `/student/joint_command` | 轨迹命令 topic,用来判断轨迹开始 |
| `control_signal_topic` | `/student/joint_command` | 控制信号记录 topic |
| `record_control_signal` | `False` | 是否记录控制信号 CSV/图 |
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


## 8. MPC IK 模块 — `move_arm_mpc_ik_demo.py/move_arm_horizontal_circle_mpc_ik_demo.py`

`move_arm_mpc_ik_demo.py` 和 `move_arm_horizontal_circle_mpc_ik_demo.py` 使用同一个节点类,参数一致，默认值不同:`move_arm_mpc_ik_demo.py` 默认为 `circle`,`move_arm_horizontal_circle_mpc_ik_demo.py` 默认为 `horizontal_circle`。

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

端载设备算力较弱时,优先尝试降低:

```bash
-p horizon_steps:=4
```

如果 ACADOS 环境暂时不可用,可以切换回退后端:

```bash
python3 move_arm_mpc_ik_demo.py --ros-args -p solver_backend:=casadi
python3 move_arm_mpc_ik_demo.py --ros-args -p solver_backend:=scipy
```

## 9. 调参

离线调参脚本:

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source ../scripts/setup_acados_env.sh
python3 tune_mpc_params.py
```

末端 x 轴竖直朝下:

```bash
python3 tune_mpc_params.py --orientation-target vertical_down
```

详细说明见 `MPC_TUNING_README.md`。

## 10. 常见问题

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

### ACADOS 后端失败但想先跑通 demo

切换回退后端:

```bash
python3 move_arm_mpc_ik_demo.py --ros-args -p solver_backend:=casadi
```

如果 CasADi/IPOPT 也不可用,再用:

```bash
python3 move_arm_mpc_ik_demo.py --ros-args -p solver_backend:=scipy
```
