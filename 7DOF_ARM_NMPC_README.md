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

`move_base_circle_mpc_ik_demo.py` 和 `trajectory_visualizer_demo.py` 都使用
`demo/moving_base_trajectory_config.py` 中的同一套基座运动、末端轨迹和抬笔参数。
控制节点和可视化节点的 `trajectory_shape` 必须完全相同,否则机械臂跟踪的目标与
RViz/Matplotlib 中显示的期望轨迹会不一致。

末端目标定义在 `world` 坐标系下。控制脚本会把 world 系末端目标变换到当前 `base_link` 系,再用普通 IK 或 MPC 求关节角。作图程序也从同一个共享配置读取 `p_wb(t)` / `R_wb(t)` / `p_ref_world(t)`,避免控制和作图参数不一致。

#### 3.5.1 可选择的绘图任务

启动 `move_base_circle_mpc_ik_demo.py` 时通过 ROS 参数
`trajectory_shape` 选择任务。允许值为
`circle`、`triangle`、`square`、`sine`、`windylab` 和 `whole`;
默认值是 `circle`。下表中的周期均为 `trajectory_time_scale:=1.0`
时的默认值,所有任务都会循环执行。

| `trajectory_shape` | 默认周期 | world 系轨迹和抬笔行为 |
|---|---:|---|
| `circle` | 4.0 s | 兼容原有任务:中心 `[0.30, 0.01, 0.10]` m、半径 0.08 m,在水平 XY 平面连续画圆 |
| `triangle` | 6.0 s | 在同一水平平面连续画三角形,宽 0.14 m、高 0.12 m;每条边使用平滑起停 |
| `square` | 8.0 s | 在同一水平平面连续画边长 0.12 m 的正方形;每条边使用平滑起停 |
| `sine` | 8.0 s | 5.0 s 画一周期正弦,宽 0.16 m、振幅 0.05 m;随后抬笔 0.5 s、悬空返回 2.0 s、落笔 0.5 s |
| `windylab` | 约 32.590 s | 绘制水平镜像的小写 `windylab`:字宽 0.016 m、字间距 0.016 m(原 0.006 m 上增加 0.010 m)、字高 0.050 m,总宽约 0.24 m。共有 10 个独立笔画(`i` 的点和 `y` 的下行笔画独立),每次换笔和换字都真实抬起末端 |
| `whole` | 约 73.039 s | 完整五面任务:正面圆 → 左面三角形 → 后面镜像 `windylab` → 右面正弦 → 上面正方形;任务之间抬笔、移动并改变基座朝向 |

`whole` 的五个绘图面和顺序如下:

| 顺序 | 方位/绘图面 | 图形 | 默认绘图面中心 |
|---:|---|---|---|
| 1 | 正面,竖直面 `x=+0.30` m | 圆 | `[+0.30, 0.00, 0.12]` m |
| 2 | 左面,竖直面 `y=+0.30` m | 三角形 | `[0.00, +0.30, 0.12]` m |
| 3 | 后面,竖直面 `x=-0.30` m | `windylab` | `[-0.30, 0.00, 0.12]` m |
| 4 | 右面,竖直面 `y=-0.30` m | 正弦 | `[0.00, -0.30, 0.12]` m |
| 5 | 上面,水平面 `z=+0.30` m | 正方形 | `[+0.22, 0.00, 0.30]` m |

每两个面之间默认执行 0.6 s 抬笔、2.4 s 悬空转场、0.6 s 落笔。
基座在这些转场期间依次转向 0°、90°、180°、270° 和 360°,
同时仍叠加共享配置中的小幅浮动扰动。第五个任务结束后也会抬笔并返回第一个任务,
因此循环边界不会连笔。

独立 `windylab` 是周期任务,写完最后一个 `b` 后会抬笔返回下一轮 `w` 的起点;
这个返回全程 `pen_down=false`,不会点到绘图面。`whole` 中的 `windylab` 是一次性子任务,
最后一个 `b` 结束后直接由五面任务执行抬笔转场,不会先返回或触碰 `w` 起点。

`windylab`、`sine` 的返回段以及 `whole` 的转场段都会生成真实的悬空末端目标,
不是只在图上隐藏连线。轨迹样本使用 `pen_down` 和 `stroke_id` 标记笔画;
RViz Marker、实时/离线 Matplotlib 曲线仅连接同一 `stroke_id` 的落笔点。
保存的轨迹 CSV 也包含 `pen_down` 和 `stroke_id`,所以字母间和各绘图面之间不会出现
可视化连笔。

#### 3.5.2 三个终端的启动方式

首次修改过 `moving_base_trajectory_config.py` 或可视化脚本后,先重新安装 ROS 包:

```bash
cd ~/code/windylab_ws
source /opt/ros/humble/setup.bash
PATH=/usr/bin:$PATH colcon build \
  --packages-select manipulator \
  --symlink-install
source ~/code/windylab_ws/install/setup.bash
```

启动机械臂和 RViz(浮动基座任务使用速度前馈增益 1.0;launch 默认 0.7 面向其他任务,故此处显式传参):

```bash
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
ros2 launch manipulator student_arm.launch.py velocity_feedforward_gain:=1.0
```

> 角速度控制(关节速度前馈)已默认开启,`enable_velocity_feedforward` 开关已移除,无需也不能再传。
> `velocity_feedforward_gain:=1.0` 与本 demo 的二阶求解器配套(demo 默认
> `second_order_dq_ff_compensation=1.0` 假定执行器增益为 1.0);若用 launch 默认 0.7 启动,
> 需给 demo 传 `-p second_order_dq_ff_compensation:=0.7`,否则跟踪精度明显下降。

RViz 左侧 `Global Options -> Fixed Frame` 建议切成 `world`。如果 Fixed Frame 仍是 `base_link`,视觉上会像是 `world`/网格在动,其实是观察坐标系固定在晃动基座上造成的相对运动效果。

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_base_circle_mpc_ik_demo.py --ros-args \
  -p trajectory_shape:=whole
```

再开一个终端启动可视化和数据记录。

```bash
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=moving_base_circle \
  -p trajectory_shape:=whole \
  -p frame_id:=world \
  -p record_control_signal:=true \
  -p ignore_initial_sec:=3.0
```

#### 3.5.2 不确定性实验

终端 1 同上不变。终端 3 加两个参数（真值端，负责按"名义+扰动"评估）：
```bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=moving_base_circle \
  -p trajectory_shape:=whole \
  -p frame_id:=world \
  -p record_control_signal:=true \
  -p ignore_initial_sec:=3.0 \
  -p show_matplotlib:=false \
  -p base_disturbance_amplitude_m:=0.01 \
  -p base_disturbance_seed:=0
```
终端 2 按实验档位三选一（amplitude/seed 必须与终端 3 一致）：

B1 第 1 级·无知下界（MPC 只知名义轨迹，预期误差 ≈ 扰动本身 ~5mm）：
```bash
python3 move_base_circle_mpc_ik_demo.py --ros-args \
  -p trajectory_shape:=whole \
  -p record_loop_csv:=true \
  -p base_disturbance_amplitude_m:=0.01 \
  -p base_disturbance_seed:=0 \
  -p base_belief_mode:=nominal
```

B2 全知对照上界（MPC 精确知道扰动，预期 ~0.5-0.8mm）：
```bash
python3 move_base_circle_mpc_ik_demo.py --ros-args \
  -p trajectory_shape:=whole \
  -p record_loop_csv:=true \
  -p base_disturbance_amplitude_m:=0.01 \
  -p base_disturbance_seed:=0 \
  -p base_belief_mode:=truth
```

B3 第 2 级·模拟状态估计（噪声 σ=2mm、延迟 40ms，预期 ~4.3mm；σ/延迟可改做退化曲线）：
```bash
python3 move_base_circle_mpc_ik_demo.py --ros-args \
  -p trajectory_shape:=whole \
  -p record_loop_csv:=true \
  -p base_disturbance_amplitude_m:=0.01 \
  -p base_disturbance_seed:=0 \
  -p base_belief_mode:=estimated \
  -p base_estimator_noise_std_m:=0.002 \
  -p base_estimator_delay_ms:=40.0 \
  -p base_estimator_lpf_hz:=10.0
```
#### 3.5.3 与任务有关的运行时参数

以下 ROS 参数必须在控制节点与可视化节点中使用相同的值:

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `trajectory_shape` | `circle` | 绘图任务:`circle` / `triangle` / `square` / `sine` / `windylab` / `whole` |
| `trajectory_time_scale` | `1.0` | 末端绘图时间倍率。实际任务周期 = 表中周期 × 此值;大于 1 更慢,小于 1 更快,必须大于 0 |
| `base_motion_time_scale` | `1.0` | 小幅基座浮动的时间倍率。实际浮动周期 = `BASE_PERIOD_SEC` × 此值;大于 1 更慢,必须大于 0 |

控制内部使用:

```text
t_path = elapsed_time / trajectory_time_scale
t_base = elapsed_time / base_motion_time_scale
```

例如把完整任务放慢到两倍时间:

```bash
# 控制节点和可视化节点均追加:
-p trajectory_shape:=whole -p trajectory_time_scale:=2.0
```

`online_time_scaling_enabled` 默认是 `false`。如果打开,控制节点会根据跟踪状态动态推进
内部路径时间,而独立可视化节点当前无法获知这个动态路径时间;需要做严格的期望误差对比时
应保持默认关闭。

#### 3.5.4 共享配置与环境变量

轨迹几何、速度和基座浮动定义在
`src/arm-platform/demo/moving_base_trajectory_config.py`。可在启动进程前使用以下环境变量
覆盖常用值;控制节点和可视化节点必须设置成相同值。环境变量在 Python 导入配置文件时读取,
因此修改后需要重启两个节点。

| 环境变量 | 默认值 | 作用 |
|---|---:|---|
| `MB_EE_SHAPE` | `circle` | `trajectory_shape` 的进程默认值;显式 ROS 参数优先 |
| `MB_EE_PEN_LIFT` | `0.03` | 换笔/换面时末端离开绘图面的高度或距离(m) |
| `MB_EE_DRAW_SPEED` | `0.04` | `windylab` 的落笔绘制速度(m/s) |
| `MB_EE_TRAVEL_SPEED` | `0.08` | `windylab` 的抬笔移动速度(m/s) |
| `MB_EE_DRAW_Z` | `0.10` | 本地水平绘图坐标中 `sine`/`windylab` 的绘图高度(m) |
| `MB_BASE_RADIUS` | `0.01` | `circle_z` 基座水平浮动半径(m) |
| `MB_BASE_PERIOD` | `0.5` | 基座浮动周期(s) |
| `MB_BASE_Z_AMP` | `0.01` | `circle_z` 基座 z 方向浮动幅值(m);设为 0 可关闭 |
| `MB_BASE_SHAPE` | `circle_z` | 基座浮动形状:`circle_z` 或 `sphere` |
| `MB_BASE_SPHERE_R` | `0.02` | `sphere` 模式的倾斜大圆半径(m) |

例如把抬笔距离改成 4 cm、写字速度放慢到 3 cm/s:

```bash
export MB_EE_PEN_LIFT=0.04
export MB_EE_DRAW_SPEED=0.03
```

固定常量 `BASE_HEIGHT=0.01` m、`BASE_ATTITUDE_AMPLITUDE_DEG=0.1`°、
`EE_WINDYLAB_LETTER_GAP=0.016` m、`EE_WINDYLAB_MIRROR_X=True`,
以及其他图形尺寸、`whole` 的五个绘图面位置和 0.6/2.4/0.6 s 转场时间,
目前没有对应的 ROS 参数或环境变量;需要时统一修改共享配置文件并重新
`colcon build --packages-select manipulator`。

#### 3.5.5 MPC 求解与记录参数

本 demo 有两套求解器,由 `solver_model` 选择,默认 `second_order`:

- `second_order`(默认):状态为 `[q, dq]`、控制量为关节加速度的二阶运动学 NMPC
  (`second_order_mpc_ik.py`)。求解器同时输出下一拍位置与速度,速度直接作为前馈发布,
  含关节位置/速度/加速度硬约束和平移雅可比最小奇异值约束。全速工况
  (基座 1s 周期 + 末端 4s 圆)稳态误差 P95 约 2mm。
- `first_order`:只规划关节位置的一阶 NMPC(`mpc_pinocchio_ik.py`),速度用位置差分
  `dq_cmd = (q_next - q_current)/control_dt` 发布。同工况稳态误差 P95 约 10mm,
  配套执行器增益 0.7。
- `shadow`:一阶控制,二阶只计算并记录诊断,不发布二阶命令(对照实验用)。

主要可调参数(浮动基座任务默认值):

| 参数 | 默认值 | 说明 |
|---|---|---|
| `solver_model` | `second_order` | `second_order` / `first_order` / `shadow` |
| `guard_mode` | `state_safe_filter` | `state_safe_filter`=以预计命令生效时的当前状态为锚点,依次尝试原命令、最大安全投影和主动 DLS 纠偏,不使用可能过期的“上一接受命令”作为可行域原点;`expected_residual`=旧的接受命令锚点残差守卫;`legacy_fixed_step`=固定 6mm 阈值;`expected_shadow`=旧规则控制、只记录残差诊断 |
| `second_order_horizon_steps` | `10` | 二阶预测步数;50Hz 下覆盖 0.2s。真机 whole-task 中 N=20 的完整周期 P95/P99 达 26.7/32.9ms,超过 20ms 截止期并造成反馈积压,故生产值回退到实时 N=10 |
| `second_order_position_weight` | `5000.0` | 二阶位置权重 |
| `second_order_terminal_position_weight` | `500.0` | 二阶终端位置权重；`1000.0` 虽改善重复圆，但完整 whole-task 在线 A/B 变差，故未纳入默认 |
| `second_order_cartesian_residual_mode` | `external_affine` | 复用 `5e740ca` 的 Pinocchio 逐 stage 外部仿射化;每次 RTI 后按新名义轨迹重算 |
| `second_order_cartesian_velocity_weight` | `0.0` | 外部仿射速度残差实验开关;生产默认关闭,避免与未投影位置参考冲突 |
| `second_order_cartesian_acceleration_weight` | `0.0` | 加速度残差结构已接入;当前 2Hz 固定约束 A/B 中非零值增加横向误差,故默认关闭 |
| `second_order_terminal_cartesian_velocity_weight` | `0.0` | 终端笛卡尔速度实验权重;生产默认关闭 |
| `second_order_feasible_reference_projection` | `false` | 导数参考投影实验开关;生产默认关闭,结构与诊断仍保留 |
| `second_order_feasible_reference_damping` | `1e-4` | 7-DOF 冗余参考投影阻尼 |
| `second_order_contour_tracking_enabled` | `false` | 轮廓/滞后误差实验开关;结构保留,但 N=20 轮廓候选的真机结果被控制周期超时污染,生产默认使用已验证的 position-only N=10 |
| `second_order_contour_position_weight` | `5000.0` | 垂直于绘图切线的轮廓误差权重 |
| `second_order_lag_position_weight` | `750.0` | 沿绘图切线的时间滞后权重;自动调参在轮廓稳定性和进度滞后之间的折中 |
| `second_order_transition_position_weight` | `5000.0` | 抬笔/换面期间的普通三维位置权重 |
| `reference_derivative_dt_sec` | `0.005` | 解析 world/base 参考中心差分半步,必须不超过半个控制周期 |
| `second_order_velocity_weight` | `0.02` | 关节速度正则权重;投影关闭时参考为零 |
| `second_order_acceleration_weight` | `0.0` | `qddot_ff` 实验权重;结构保留但生产默认关闭 |
| `second_order_terminal_velocity_weight` | `0.0` | tracking-priority 默认不强迫 0.4 s 预测域末端降速;非零值只用于慢速/驻点实验 |
| `second_order_dq_ff_compensation` | `1.0` | 发布速度 = `dq_next / 该值`;需等于执行器 `velocity_feedforward_gain` |
| `second_order_velocity_source` | `commanded` | 二阶 x0 速度来源:`commanded`=上一拍已接受的模型速度,与二阶预测状态一致;`measured` 仅用于完成速度观测器辨识后的实验 |
| `second_order_state_prediction` | `false` | 实验开关:把带时间戳的实测 `[q,dq]` 用上一拍加速度前推;未完成执行器/反馈延迟联合辨识前默认关闭 |
| `latency_compensation_enabled` | `false` | 实验开关:同时前移状态和参考时刻;资源抖动由当前状态安全过滤器处理,不再默认改变控制相位 |
| `latency_nominal_publish_ms` | `6.0` | 第一拍使用的求解开始到发布延迟估计(ms) |
| `latency_publish_ewma_alpha` | `0.15` | 实际求解到发布时延 EWMA 更新系数 |
| `latency_actuator_delay_ms` | `0.0` | 已标定的发布后执行器固定延迟(ms);有实测值时填写 |
| `latency_prediction_max_ms` | `60.0` | 反馈状态前推和应用时延补偿的安全上限(ms) |
| `late_tick_recovery_enabled` | `true` | 控制回调迟到时丢弃过期二阶 warm start 和 commanded-velocity 状态,从该拍最新实测状态重新求解 |
| `late_tick_recovery_factor` | `1.5` | 迟到阈值相对控制周期的倍数;50Hz 默认即 30ms |
| `guard_backup_damping` | `0.03` | 状态安全过滤器 DLS 纠偏阻尼 |
| `guard_backup_position_gain` | `0.8` | 状态安全过滤器单拍 world 位置误差纠偏比例 |
| `solver_backend` | `acados` | 求解后端 |
| `horizon_steps` | `2` | 一阶预测步数 |

控制节点使用单线程确定性 executor，反馈 QoS 深度为 1：ACADOS 求解期间
`/joint_states` 只保留最新状态，求解结束后不会逐条消费过期反馈。这里不使用
Python 两线程 executor，因为实测 100Hz 反馈回调与 50Hz NMPC timer 会争抢
GIL，反而引入大量 30--44ms 控制间隔。这不是放宽硬件约束。2026-07-25 的
N=20 whole-task 失败数据里，反馈时间戳曾单拍跳过 109.9ms，同时 command-FK
跳变 23.4mm；对应运行的实际 RMS/最大误差为 7.64/28.23mm。
因此 N=20 已从生产默认撤回。后续候选除了 ACADOS `solve_time`，还必须满足
“参考生成 + Python 封装 + solver”的完整计算 P99 小于 12ms，给发布、记录和
ROS 调度保留至少 8ms 裕量。
| `max_joint_step` | `0.02` | 单步最大关节变化量(rad),应等于 `max_velocity * control_dt` |
| `control_rate_hz` | `50.0` | 控制频率 |
| `terminal_tolerance` | `0.01` | 终端位置误差 warning 阈值(m) |
| `orientation_weight` | `0.09375` | 一阶时域内姿态软约束权重 |
| `terminal_orientation_weight` | `1.25` | 一阶终端姿态软约束权重 |
| `posture_reference_mode` | `reset_nominal` | `reset_nominal`=固定复位姿态软锚点;`current`=逐帧当前姿态参考 |
| `acados_rti_iterations` | `3` | 每帧 SQP_RTI 重复次数(须 >=1,非法值抛错) |
| `reference_lead_steps` | `1` | 参考前移拍数(命令瞄准 t+dt) |
| `dq_prev_mode` | `measured_delta` | 一阶 x0 的 dq_prev 来源:`measured_delta`=上一控制周期实测执行增量;`zero`=清零 |
| `rti_iteration_mode` | `fixed_iterations` | `fixed_iterations`=成功时跑满迭代数;`legacy_early_exit`=按误差阈值提前退出 |

MPC 版本会向 `/student/joint_command` 同时发布 `position` 和 `velocity`。
`student_arm_node` 里的 `SmoothPositionController` 会对位置做平滑限速,并把 velocity 作为
速度前馈传到底层(按 launch 参数 `max_velocity` 限幅、`max_acceleration` 限加速度)。
`max_joint_step` 应与底层 `max_velocity * control_dt` 匹配。

二阶求解器已选择性纳入提交 `5e740ca` 的 world/base 显式导数与外部仿射化。
原始参考为:

```text
v_ref_B = R_WB.T * (v_ee_W - v_base_W) - omega_B × r_B

a_ref_B = R_WB.T * (a_ee_W - a_base_W)
          - 2 * omega_B × v_ref_B
          - alpha_B × r_B
          - omega_B × (omega_B × r_B)
```

这些导数及外部仿射残差保留为显式实验接口，但生产默认不把它们加入代价。
原因是逐 stage 投影导数、同时保留未投影的定时位置会产生不一致参考，并在
2Hz 基座工况形成周期追赶。设置
`second_order_feasible_reference_projection:=true` 并配置非零导数权重时，
每个预测 stage 才会执行固定盒约束阻尼最小二乘:

```text
qdot_ff  = argmin ||J qdot - v_ref_B||² + lambda ||qdot||²
           s.t. |qdot| <= 1

qddot_ff = argmin ||J qddot + Jdot qdot_ff - a_ref_B||²
                  + lambda ||qddot||²
           s.t. |qddot| <= 5
                |dq_bar + dt*qddot| <= 1
```

再用 `J*qdot_ff` 和 `J*qddot_ff+Jdot*qdot_ff` 作为可行笛卡尔参考。这里的
`1` 和 `5` 直接来自现有硬件约束,没有被放宽。三个笛卡尔导数权重均为零时，
后端跳过 Pinocchio 外部仿射线性化，避免无效计算占用 20ms 控制预算。

落笔阶段把位置误差 `e=p_ee-p_ref` 按 world 绘图切线在 base 系的表示
`t_B` 分解为:

```text
e_contour = (I - t_B*t_B.T) * e
e_lag     = t_B.T * e
```

轮廓权重约为沿线滞后权重的 6.67 倍。因此控制能力不足时优先保持几何线条,
允许沿线方向稍微落后,而不是在直线两侧周期绕行。抬笔、换面和切线速度接近零
时自动退回普通定时点跟踪。

2Hz、`velocity_feedforward_gain=1.0`、硬约束保持 `±1 rad/s` 和
`±5 rad/s²` 的自动搜索命令:

```bash
python3 tune_unified_mpc_params.py \
  --second-order-tune --second-order-stable-contour \
  --second-order-base-period 0.5 --second-order-ee-period 4.0 \
  --second-order-horizons 10,15,20 \
  --second-order-velocity-source commanded \
  --second-order-guard-rule expected_residual \
  --plant-gain 1.0
```

完整 `whole` 五工作面离线一周期 A/B 中，生产旧候选与新默认分别为:

| 指标 | 旧候选 `N=10, cv=2, projection=true, lag=250` | 新默认 `N=20, cv=0, projection=false, lag=750` |
|---|---:|---:|
| 三维 RMS | 2.705 mm | 2.082 mm |
| 三维 P95 | 5.194 mm | 4.217 mm |
| 落笔轮廓 RMS | 1.096 mm | 0.836 mm |
| 落笔轮廓 P95 | 2.256 mm | 1.845 mm |
| 2Hz 残差 RMS | 1.258 mm | 1.057 mm |
| 加速度饱和平均关节占比 | 66.7% | 51.6% |
| 求解 P99 | 2.55 ms | 7.64 ms |

两组均为 `guard=0`、`solver_failure=0`；新默认仍低于 20ms 控制周期。

固定 `±1/±5` 约束、2Hz 基座的离线闭环消融中,相对于 position-only:

| 配置 | 总 RMS | 横向 RMS / P95 | 总优化 P95 |
|---|---:|---:|---:|
| position-only | 1.66 mm | 1.15 / 2.56 mm | 4.86 ms |
| 可行速度前馈 + contour/lag | 2.02 mm | 0.78 / 1.69 mm | 9.78 ms |

总点误差因允许沿线滞后而略增,但决定“直线是否绕线”的横向 RMS 降低约 32%。
这是离线结构 A/B,仍须以新的 `whole` 真机 CSV 验收。

录屏或桌面渲染抢占 CPU/GPU 时,ROS timer、反馈和求解到发布延迟都可能超过名义 20 ms。
当前生产默认由 `state_safe_filter` 从每拍新鲜状态重新建立安全集合,避免旧接受命令锚点造成
连续 hold/fault。下面的状态/参考外推保留为显式实验功能,默认关闭:

```text
d_app = EWMA(solve_to_publish) + actuator_delay
t_app = t_solve + d_app
tau   = age(joint_state at solve start) + d_app
q_app  = q_meas + dq_meas*tau + 0.5*ddq_prev*tau^2
dq_app = dq_meas + ddq_prev*tau
```

启用后,它把上式作为二阶 NMPC 的初始状态,并把整个 world/base 参考预测域前移 `d_app`。安全过滤器比较
`(q_app,t_app)` 到下一拍候选的 world 运动;若发生投影、backup、hold 或求解失败,会同时清除
旧的 `q/dq/ddq` 热启动计划,避免下一拍重新生成同一个异常跳变。轨迹直线段、抬笔和五面转场
使用五次 smoothstep,端点速度和加速度均为零,用来降低拐角/转场激励。

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
| `max_actual_points` | `12000` | RViz 最多保留的实际轨迹采样点数;默认可完整保留至少一轮 `whole`,避免早期红色笔画在任务结束前被滚动删除 |
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
