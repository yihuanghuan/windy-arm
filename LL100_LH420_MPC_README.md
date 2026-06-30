# LL100-LH420 ACADOS NMPC 使用说明

本文档集中说明 LL100-LH420 整机模型的六轴 ACADOS NMPC demo。LL100 相关启动、避障、轨迹演示和常见问题以本文档为准。

## 1. 适用范围

- 控制对象:LL100-LH420 整机模型中的六轴机械臂 `A_Joint` ~ `F_Joint`。
- 可视化对象:整机 `ll100_lh420.urdf`,包含机体、4 个旋翼和机械臂。
- NMPC 模型:`ll100_lh420_nmpc.urdf`,只保留 A-F 六轴机械臂用于求解。
- 求解器:默认 ACADOS 后端。
- 避障:前桨圆盘 SDF + 机身/安装架简化 box + 连杆 capsule 采样点,默认使用 ACADOS 非线性硬约束。
- 当前用途:仿真/RViz 演示和算法验证,不直接控制真实 LL100 硬件。

## 2. 环境准备

每个新终端先 source ROS 和工作空间:

```bash
cd ~/code/windylab_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

运行 ACADOS NMPC 前还需要 source ACADOS 环境:

```bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
```

验证 ACADOS Python 接口:

```bash
python3 -c "import acados_template; print('acados ok')"
```

如果尚未安装 ACADOS:

```bash
cd ~/code/windylab_ws/src/arm-platform
scripts/install_acados.sh
source scripts/setup_acados_env.sh
```

ACADOS 求解器缓存规则:

- 第一次运行某个配置时会 `generate=True, build=True`,生成并编译 C 求解器。
- 后续同配置运行时,如果 `libacados_ocp_solver_*.so`、`acados_ocp.json` 和 `acados_solver_metadata.json` 都存在且配置指纹一致,会直接复用。
- `horizon_steps`、权重、姿态开关、前桨/机身 soft-hard 约束模式、前桨圆盘数量/几何、机身 box 数量、capsule 半径或采样点数量变化时,会自动重新生成并编译。
- 如果怀疑缓存损坏,删除对应 `/tmp/windylab_acados_*` 目录后重新运行。

## 3. 启动 LL100 控制版模型

启动控制版:

```bash
cd ~/code/windylab_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch manipulator ll100_lh420_control.launch.py
```

控制版 launch 会启动:

- `student_arm_node` 的 `ll100_sim` 六轴虚拟臂。
- LL100 整机 `robot_state_publisher`。
- RViz。
- rotor joint state publisher,只给 `rot1_Joint` ~ `rot4_Joint` 发布零位。

NMPC demo 发布的是六维 `/student/joint_command`,关节名为:

```text
A_Joint, B_Joint, C_Joint, D_Joint, E_Joint, F_Joint
```

检查控制链:

```bash
ros2 topic info /student/joint_command
```

NMPC demo 启动后应看到:

```text
Publisher count: 1
Subscription count: 1
```

除此之外还有 `ll100_lh420_display.launch.py`文件，这个是单纯的显示文件。不要用这个来启动rivz，因为显示版不订阅 `/student/joint_command`,机械臂不会跟随命令运动。


## 4. 避障 NMPC

`ll100_traj_pub.py`：生成期望轨迹
`move_ll100_lh420_nmpc_demo.py` ：核心启动demo
`MpcPinocchioIK` ：NMPC公式与求解器调用

话题的使用：
MPC接受的当前状态话题：使用 `/joint_states` 中的 `A_Joint`~`F_Joint` 作为 NMPC 当前状态和 capsule marker 姿态来源。
MPC接收的期望状态话题：无，调用`ll100_traj_pub.py`直接生成轨迹
MPC输出的控制量话题：发布 `/student/joint_command（JointState）`

### 4.1 基础模式：单目标模式

末端到达一个固定目标点后保持。先启动控制版模型,再另开终端:

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_ll100_lh420_nmpc_demo.py --ros-args \
  -p path_mode:=single_target \
  -p single_target_position:="[0.50, 0.0, 0.10]"
```

单目标模式复用轨迹模式的全部参数和默认值(见 4.2),只额外增加一个参数:

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `single_target_position` | `[0.50, 0.0, 0.10]` | `base_link` 下的固定目标点,仅 `path_mode:=single_target` 时生效 |

### 4.2 进阶模式：轨迹模式

路径版 demo 用于展示“轨迹规划 + 避障”的效果。默认轨迹为 `lower_obstacle_circle`:末端 x 轴竖直朝下,在前桨/机身避障区域下方慢速画圆。

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_ll100_lh420_nmpc_demo.py
```

路径版关键参数:

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `period_sec` | `60.0` | 轨迹周期,s;默认很慢,方便实际模型跟上 |
| `path_mode` | `60.0` | 不同轨迹模式切换 |
| `max_joint_step` | `0.01` | 每周期最大关节步长,rad |
| `path_preview_stride` | `4` | 预测时域内相邻目标点间隔,避免时域只盯住当前点 |
| `position_weight` | `120.0` | 位置跟踪权重 |
| `terminal_position_weight` | `1200.0` | 终端位置跟踪权重 |
| `lock_orientation` | `True` | 默认锁定末端姿态 |
| `orientation_target` | `vertical_down` | 末端 x 轴朝 `-Z` |
| `orientation_weight` | `0.3` | 姿态软约束权重 |
| `terminal_orientation_weight` | `3.0` | 终端姿态软约束权重 |
| `obstacle_avoidance_enabled` | `True` | 是否启用前桨圆盘避障(固定为 ACADOS 非线性硬约束,无软约束开关) |
| `obstacle_safety_margin` | `0.04` | 前桨圆盘避障的 NMPC 硬约束安全余量(从 SDF 中减去),m |
| `advance_on_feedback` | `False` | `False` 时按 `period_sec` 保持期望速度;`True` 时实际末端接近当前路径点后再推进 |
| `start_from_nearest_path_point` | `True` | 启动时从距离当前末端最近的路径点并入 |
| `hold_start_until_reached` | `True` | 从稳定弧线并入点进入容差后再启动时间轴 |
| `target_switch_tolerance` | `0.025` | 路径点切换距离阈值,m |
| `stuck_steps_before_advance` | `120` | 当前点误差长时间不下降时自动推进 |
| `stuck_advance_points` | `10` | 卡住时一次跳过的路径点数 |
| `body_obstacle_margin` | `0.015` | 机身/安装架 box 避障的 NMPC 硬约束余量(额外安全距离),m |
| `capsule_radius` | `0.018` | 连杆 capsule 碰撞半径,m;参与机身 box 的 NMPC 硬约束 |
| `capsule_samples_per_segment` | `1` | 每段连杆中间采样点数量 |
| `current_posture_weight` | `0.005` | 抑制相对当前姿态突变 |
| `center_posture_weight` | `0.01` | 抑制机械臂拉到近奇异伸直姿态 |

`path_mode` 支持:

| 模式 | 说明 |
|---|---|
| `lower_obstacle_circle` | 默认模式,末端朝下,在避障区域下方慢速画圆 |
| `avoidance_sweep` | 靠近机身/安装架约束的演示轨迹 |
| `front_wing_arc` | 前端左右机翼/前桨下方两点之间的余弦往返弧线|
| `front_propeller_line` | 前桨外侧直线演示 |
| `circle` | 在 `base_link` 的 y-z 平面画圆 |
| `windylab` | 在 `base_link` 的 y-z 平面写简化 `windylab` |
| `single_target` | 单目标模式 |

`max_joint_step` 应与底层限速匹配。`ll100_lh420_control.launch.py` 默认 `max_velocity=0.5 rad/s`,50 Hz 控制周期下推荐:

```text
max_joint_step ~= max_velocity * 0.02
```

例如 launch 使用 `max_velocity:=1.0` 时,路径 demo 可设置:

```bash
python3 move_ll100_lh420_nmpc_demo.py --ros-args -p max_joint_step:=0.02
```

### 4.3 安全与精度要求

安全第一:必须保证机身和桨叶与机械臂**完全不可能发生碰撞**;在不碰撞的前提下,再追求末端误差低、跟踪精度高。

为此,桨叶和机身避障只使用 ACADOS 非线性硬约束(已移除软约束选项)。硬约束在简化模型范围内不可违反,是"不可能碰撞"的来源;软约束只是代价惩罚、可被突破,因此不再保留。

日志指标(用于验证安全和精度):

- `terminal_error`:预测时域末端位置误差,越低跟踪越准。
- `min_obstacle_margin`:前桨和机身两类避障 margin 的最小值。
- `min_body_obstacle_margin`:只看机身/安装架 box 避障。
- `obstacle_violation_max`:最大安全裕度侵入量;硬约束下应**恒为 `0.0`**,一旦 `> 0` 说明发生侵入,优先级高于跟踪精度。

`min_obstacle_margin < 0` 表示预测轨迹已经进入安全裕度内。ACADOS hard constraint 只约束当前简化模型和采样点,不等于真机安全证明;真机前仍需要增大 margin、降低速度并保留硬件安全措施。

## 5. LL100 轨迹可视化节点

`ll100_trajectory_visualizer_demo.py` 专门用于记录和绘制 LL100 六轴末端轨迹。它不发控制命令,只监听:

- `/joint_states`: 用 A-F 六轴反馈 FK 计算实际末端轨迹。
- `/student/joint_command`: 用命令关节角 FK 计算 command FK 轨迹。

启动:

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
python3 ll100_trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=lower_obstacle_circle \
  -p period_sec:=60.0
```

这个合并话题同时包含期望轨迹、实际轨迹、command FK、当前点、前桨圆盘和机身 box。

仍然保留以下单独 topic,方便调试:

| 类型 | Topic | 内容 |
|---|---|---|
| `MarkerArray` | `/ll100_trajectory/markers` | 合并显示全部 LL100 可视化 marker |
| `Marker` | `/ll100_trajectory/expected` | 绿色完整期望轨迹 |
| `Marker` | `/ll100_trajectory/actual` | 红色实际 EE 轨迹 |
| `Marker` | `/ll100_trajectory/command_fk` | 橙色命令 FK 轨迹 |
| `Marker` | `/ll100_trajectory/current_target` | 当前期望点 |
| `Marker` | `/ll100_trajectory/current_actual` | 当前实际点 |
| `MarkerArray` | `/ll100_trajectory/obstacles` | 前桨圆盘和机身 box |

Ctrl+C 退出时会把 CSV 和 PNG 保存到:

```text
src/arm-platform/demo/data
```

参数:

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `trajectory_mode` | `lower_obstacle_circle` | 期望轨迹模式,共 6 种，与demo相同，但single_target不计入，因为单目标点没有连续路径可绘制;必须和路径 demo 的 `path_mode` 一致 |
| `period_sec` | `60.0` | 绘图期望轨迹的时间周期 |
| `expected_path_points` | `240` | RViz/Matplotlib 中期望路径采样点数 |
| `max_actual_points` | `2000` | RViz 中保留的实际轨迹点数 |
| `show_matplotlib` | `True` | 是否打开实时 Matplotlib 窗口 |
| `start_on_command` | `True` | 收到 `/student/joint_command` 后开始记录 |
| `use_message_stamp` | `False` | 默认用可视化节点接收时钟统一 `/student/joint_command` 和 `/joint_states` 时间 |
| `start_when_on_expected_path` | `True` | 实际末端进入期望轨迹入口点容差后才把记录时间置零 |
| `start_path_tolerance` | `0.025` | 判定“已经到达期望轨迹入口点”的距离阈值,m |
| `use_stable_front_wing_arc_start` | `True` | `front_wing_arc` 远离轨迹启动时等待左右稳定弧线中点之一,避免从端点或直线段并入 |
| `reset_gap_sec` | `2.0` | 命令中断超过该间隔后重置记录,开始新一段轨迹 |
| `stale_command_timeout_sec` | `0.2` | 超过该时间没收到新命令则认为命令过期/已停 |
| `ignore_initial_sec` | `1.0` | 记录开始后忽略前几秒 |
| `sample_only_on_actual_change` | `True` | 仅当实际末端发生变化时才采样,避免静止时堆积重复点 |
| `actual_change_epsilon` | `1e-9` | 判定实际末端“发生变化”的位移阈值,m |
| `record_command_fk` | `True` | 是否额外记录命令关节角 FK 得到的 command 轨迹 |
| `obstacle_safety_margin` | `0.04` | 仅可视化:前桨圆盘障碍的安全余量,m(不参与 NMPC 求解) |
| `obstacle_disk_half_thickness` | `0.02` | 仅可视化:障碍圆盘半厚度,m |
| `body_obstacle_margin` | `0.015` | 仅可视化:机身/安装架 box 障碍的额外安全距离,m |
| `capsule_radius` | `0.018` | 仅可视化:连杆 capsule 半径,m |
| `data_dir` | `.../arm-platform/demo/data` | 记录数据(轨迹/图)的保存目录 |


## 6. 避障模型

前桨避障:

- 从 `ll100_lh420.urdf` 的 `rot1_Joint` 和 `rot3_Joint` 推导前桨位置。
- 每个前桨建模为一个圆盘,圆盘行向量为 `[cx, cy, cz, nx, ny, nz, radius]`。
- ACADOS 中使用圆盘 SDF,再减去 `obstacle_safety_margin` 得到安全裕度;默认作为 nonlinear hard constraint。
- 默认前桨扫掠半径 `0.27 m`,圆盘 marker 厚度由 `obstacle_disk_half_thickness` 控制。

机身/安装架避障:

- 在 `base_link` 坐标系下用局部 box 覆盖底板、机身后部和机械臂安装架。
- 连杆避障不是完整 mesh 碰撞,而是把 `A_Link`~`F_Link` 相邻 frame 形成 capsule。
- 每段 capsule 取固定采样点,默认作为 ACADOS nonlinear hard constraint;关闭 hard constraint 时才退回 soft cost。

姿态目标:

`vertical_down` 定义为末端坐标系 x 轴朝 `base_link` 的 `-Z`,采用完整旋转矩阵:

```text
末端 x 轴 -> [0, 0, -1]
末端 y 轴 -> [0, 1, 0]
末端 z 轴 -> [1, 0, 0]
```

## 7. 常见问题

### `/student/joint_command` 没有订阅者

通常是启动了显示版 launch。请使用:

```bash
ros2 launch manipulator ll100_lh420_control.launch.py
```

不要用 `ll100_lh420_display.launch.py` 跑 NMPC 控制。

### 找不到 `/ll100_nmpc/markers`

`/ll100_nmpc/markers` 是早期单目标脚本的旧话题,已随脚本合并移除。现在 `move_ll100_lh420_nmpc_demo.py`(含 `single_target` 模式)统一发布:

```text
/ll100_nmpc_path/markers
```

RViz 里需要手动添加 `MarkerArray`。

### ACADOS 每次都重新 build

求解器配置指纹变化会触发重新生成。常见触发项包括 `horizon_steps`、姿态权重、前桨/机身 soft/hard 约束模式、机身 box/capsule 参数等。同配置下应复用 `/tmp/windylab_acados_*` 中的已编译求解器。

### 轨迹跟踪滞后或实际轨迹绕圈

优先检查 `max_joint_step` 是否大于底层 `max_velocity * 0.02`。上层 NMPC 走得比底层限速快时,实际机械臂会滞后。路径版默认先进入稳定弧线并入点,进入容差后按 `period_sec` 推进目标;如果要严格等实际末端接近每个路径点后再推进,设置 `advance_on_feedback:=true`。

如果 command FK 也明显落后或变形,优先调大 `path_preview_stride` 到 `6` 或 `8`,并保持 `max_joint_step` 不超过底层实际速度能力。`circle` 模式的期望轨迹是半径 `0.08 m` 的真圆;如果图上不是圆,先确认控制节点和 `ll100_trajectory_visualizer_demo.py` 使用相同的 `trajectory_mode` 和 `period_sec`。

如果只有作图结果显示一个固定相位差,先检查 `ll100_trajectory_visualizer_demo.py` 是否仍用 `use_message_stamp:=true`。LL100 path 控制命令的 stamp 是内部轨迹时间,而 `/joint_states` 是真实时间;默认 `use_message_stamp:=false` 可以避免两套时间基准混在同一张误差图里。

### 进入近奇异姿态后不动

说明目标轨迹把机械臂推到伸直或可操作性很差的区域。可尝试:

- 使用默认 `lower_obstacle_circle`。
- 增大 `center_posture_weight`。
- 降低 `max_joint_step`。
- 调大 `target_switch_tolerance`。
- 将目标轨迹移回更接近机械臂中间工作区的位置。

### 只想查看 LL100 整机模型

```bash
cd ~/code/windylab_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch manipulator ll100_lh420_display.launch.py
```

显示版只用于 RViz 查看,不参与 NMPC 控制。
