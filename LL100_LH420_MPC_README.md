# LL100-LH420 ACADOS NMPC 使用说明

本文档集中说明 LL100-LH420 整机模型的六轴 ACADOS NMPC demo。LL100 相关启动、避障、轨迹演示和常见问题以本文档为准。

## 1. 适用范围

- 控制对象:LL100-LH420 整机模型中的六轴机械臂 `A_Joint` ~ `F_Joint`。
- 可视化对象:整机 `ll100_lh420.urdf`,包含机体、4 个旋翼和机械臂。
- NMPC 模型:`ll100_lh420_nmpc.urdf`,只保留 A-F 六轴机械臂用于求解。
- 求解器:默认 ACADOS 后端。
- 避障:前桨圆盘 SDF + 机身/安装架简化 box + 连杆 capsule 采样点软约束。
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
- `horizon_steps`、权重、姿态开关、避障开关、前桨圆盘数量/几何、机身 box 数量、capsule 半径或采样点数量变化时,会自动重新生成并编译。
- 如果怀疑缓存损坏,删除对应 `/tmp/windylab_acados_*` 目录后重新运行。

## 3. 启动 LL100 控制版模型

不要用只显示模型的 `ll100_lh420_display.launch.py` 来跑 NMPC。显示版不订阅 `/student/joint_command`,机械臂不会跟随命令运动。

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

## 4. 单目标避障 NMPC

另开终端:

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_ll100_lh420_nmpc_demo.py
```

默认目标点 `[0.50, 0.0, 0.10]` 位于 LL100 `base_link` 坐标系。

RViz 中添加 `MarkerArray`,topic 选择:

```text
/ll100_nmpc/markers
```

该 marker 会显示前桨圆盘、机身 box、连杆关键点和 capsule 采样点。

常用参数:

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `target_position` | `[0.50, 0.0, 0.10]` | `base_link` 坐标系下的末端目标点,m |
| `terminal_tolerance` | `0.02` | 终端位置误差 warning 阈值,m |
| `horizon_steps` | `8` | NMPC 预测步数 |
| `max_joint_step` | `0.08` | 每周期单关节最大变化量,rad |
| `lock_orientation` | `False` | 是否启用末端姿态软约束 |
| `orientation_target` | `initial` | `initial` 或 `vertical_down` |
| `obstacle_weight` | `20000.0` | 前桨避障软惩罚权重 |
| `obstacle_safety_margin` | `0.04` | 前桨圆盘外的额外安全距离,m |
| `body_obstacle_enabled` | `True` | 是否启用机身/安装架 box 避障 |
| `body_obstacle_weight` | `30000.0` | capsule 侵入机身 box 的软惩罚权重 |
| `body_obstacle_margin` | `0.03` | 机身 box 额外安全距离,m |
| `capsule_radius` | `0.045` | 连杆 capsule 半径,m |
| `capsule_samples_per_segment` | `3` | 每段连杆内部采样点数量,实际还包含两个端点 |

示例:

```bash
python3 move_ll100_lh420_nmpc_demo.py --ros-args \
  -p target_position:="[0.50, 0.0, 0.10]" \
  -p obstacle_weight:=20000.0 \
  -p obstacle_safety_margin:=0.04 \
  -p body_obstacle_enabled:=true \
  -p body_obstacle_weight:=30000.0 \
  -p body_obstacle_margin:=0.03 \
  -p capsule_radius:=0.045 \
  -p capsule_samples_per_segment:=3
```

日志指标:

- `terminal_error`:预测时域末端位置误差。
- `min_obstacle_margin`:前桨和机身两类避障 margin 的最小值。
- `min_body_obstacle_margin`:只看机身/安装架 box 避障。
- `obstacle_violation_max`:最大软约束侵入量。

`min_obstacle_margin < 0` 表示软约束已经出现侵入。该避障不是硬安全证明,真机前需要增大 margin、降低速度并保留硬件安全措施。

## 5. 轨迹避障 NMPC

路径版 demo 用于展示“轨迹规划 + 避障”的效果。默认轨迹为 `lower_obstacle_circle`:末端 x 轴竖直朝下,在前桨/机身避障区域下方慢速画圆。

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_ll100_lh420_nmpc_path_demo.py
```

RViz 中添加 `MarkerArray`,topic 选择:

```text
/ll100_nmpc_path/markers
```

绿色线是期望轨迹,蓝色线是 `/joint_states` 反馈计算得到的实际末端轨迹。

`path_mode` 支持:

| 模式 | 说明 |
|---|---|
| `lower_obstacle_circle` | 默认模式,末端朝下,在避障区域下方慢速画圆 |
| `avoidance_sweep` | 靠近机身/安装架约束的演示轨迹 |
| `front_wing_arc` | 前端左右机翼/前桨下方两点之间的宽松避障弧线 |
| `front_propeller_line` | 前桨外侧直线演示 |
| `circle` | 在 `base_link` 的 y-z 平面画圆 |
| `windylab` | 在 `base_link` 的 y-z 平面写简化 `windylab` |

示例:

```bash
python3 move_ll100_lh420_nmpc_path_demo.py
```

路径版关键参数:

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `period_sec` | `60.0` | 轨迹周期,s;默认很慢,方便实际模型跟上 |
| `max_joint_step` | `0.01` | 每周期最大关节步长,rad |
| `path_preview_stride` | `4` | 预测时域内相邻目标点间隔,避免时域只盯住当前点 |
| `position_weight` | `120.0` | 位置跟踪权重 |
| `terminal_position_weight` | `1200.0` | 终端位置跟踪权重 |
| `lock_orientation` | `True` | 默认锁定末端姿态 |
| `orientation_target` | `vertical_down` | 末端 x 轴朝 `-Z` |
| `orientation_weight` | `0.3` | 姿态软约束权重 |
| `terminal_orientation_weight` | `3.0` | 终端姿态软约束权重 |
| `obstacle_avoidance_enabled` | `True` | 是否启用前桨圆盘避障代价 |
| `advance_on_feedback` | `True` | 实际末端接近当前路径点后再推进 |
| `start_from_nearest_path_point` | `True` | 启动时从距离当前末端最近的路径点并入 |
| `target_switch_tolerance` | `0.025` | 路径点切换距离阈值,m |
| `stuck_steps_before_advance` | `120` | 当前点误差长时间不下降时自动推进 |
| `stuck_advance_points` | `10` | 卡住时一次跳过的路径点数 |
| `body_obstacle_weight` | `10000.0` | 机身/安装架 box 避障权重 |
| `body_obstacle_margin` | `0.015` | 机身 box 额外安全距离,m |
| `capsule_radius` | `0.018` | 连杆 capsule 半径,m |
| `capsule_samples_per_segment` | `1` | 每段连杆中间采样点数量 |
| `current_posture_weight` | `0.005` | 抑制相对当前姿态突变 |
| `center_posture_weight` | `0.01` | 抑制机械臂拉到近奇异伸直姿态 |

路径版 demo 使用 `/joint_states` 中的 `A_Joint`~`F_Joint` 作为 NMPC 当前状态和 capsule marker 姿态来源。不要用 `/student/joint_command` 假装实际反馈。

`max_joint_step` 应与底层限速匹配。`ll100_lh420_control.launch.py` 默认 `max_velocity=0.5 rad/s`,50 Hz 控制周期下推荐:

```text
max_joint_step ~= max_velocity * 0.02
```

例如 launch 使用 `max_velocity:=1.0` 时,路径 demo 可设置:

```bash
python3 move_ll100_lh420_nmpc_path_demo.py --ros-args -p max_joint_step:=0.02
```

纯跟踪调试时,可以先关闭前桨和机身避障,确认 command FK 自身能画圆:

```bash
python3 move_ll100_lh420_nmpc_path_demo.py --ros-args \
  -p path_mode:=circle \
  -p obstacle_avoidance_enabled:=false \
  -p body_obstacle_enabled:=false
```

## 6. 解耦目标跟随 NMPC

这套 demo 把“目标生成”和“NMPC 控制”拆成两个节点:

```text
ll100_traj_pub.py -> /goal_pose -> move_ll100_lh420_nmpc_goal_demo.py -> /student/joint_command
                  <- /ll100_nmpc_goal/reached <-
```

这样控制节点只负责追踪 `/goal_pose`,后续可以接手动目标、预设轨迹、视觉跟踪目标或上层规划器。

### 6.1 启动目标跟随控制节点

先启动 LL100 控制版模型,再另开终端:

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_ll100_lh420_nmpc_goal_demo.py
```

RViz 中添加 `MarkerArray`,topic 选择:

```text
/ll100_nmpc_goal/markers
```

该 topic 会显示前桨圆盘、机身 box、连杆关键点、当前目标点和蓝色实际末端轨迹。

该节点:

- 订阅 `/goal_pose` (`geometry_msgs/PoseStamped`),目标默认按 `base_link` 坐标系解释。
- 订阅 `/joint_states`,只使用其中的 `A_Joint`~`F_Joint` 反馈。
- 发布 `/student/joint_command`,命令字段为 `position` 关节角,`velocity` 为相邻命令差分得到的前馈速度。
- 到达目标后发布 `/ll100_nmpc_goal/reached` (`std_msgs/Bool`)。

常用参数:

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `horizon_steps` | `6` | NMPC 预测步数,比路径一体版更轻 |
| `max_joint_step` | `0.08` | 每周期单关节最大变化量,rad |
| `success_tolerance` | `0.015` | 实际末端到目标的到达阈值,m |
| `lock_orientation` | `False` | 是否使用 `/goal_pose` 中的姿态作为软约束 |
| `orientation_weight` | `1.0` | 姿态软约束过程权重,仅锁姿态时生效 |
| `terminal_orientation_weight` | `10.0` | 姿态软约束终端权重,仅锁姿态时生效 |
| `obstacle_weight` | `20000.0` | 前桨圆盘避障软惩罚权重 |
| `obstacle_safety_margin` | `0.04` | 前桨圆盘外额外安全距离,m |
| `body_obstacle_enabled` | `True` | 是否启用机身/安装架 box 避障 |
| `body_obstacle_weight` | `30000.0` | 机身 box 避障软惩罚权重 |
| `capsule_samples_per_segment` | `1` | 每段连杆内部采样点数量,实际还包含两个端点 |

手动发布单点目标:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "base_link"}, pose: {position: {x: 0.50, y: 0.0, z: 0.10}, orientation: {w: 1.0}}}'
```

验收现象:

- 控制节点日志出现 `收到新目标`。
- 机械臂向目标滚动靠近。
- 到达后日志出现 `已到达目标`,并发布 `/ll100_nmpc_goal/reached: true`。

### 6.2 启动轨迹发布节点

轨迹发布节点只负责发布 `/goal_pose`,收到 reached 后推进到下一个路径点:

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
python3 ll100_traj_pub.py --ros-args \
  -p path_mode:=lower_obstacle_circle \
  -p waypoint_count:=60 \
  -p period_sec:=60.0
```

RViz 中再添加一个 `MarkerArray`,topic 选择:

```text
/ll100_traj_pub/markers
```

该 topic 会显示绿色完整期望路径和黄色当前 waypoint。

常用参数:

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `path_mode` | `lower_obstacle_circle` | 轨迹模式 |
| `waypoint_count` | `60` | 路径离散点数量,默认圆轨迹相邻点约 8 mm |
| `period_sec` | `60.0` | 轨迹语义参数和日志提示,不再用于计算 waypoint 数量 |
| `waypoint_timeout_sec` | `10.0` | 长时间没收到 reached 时自动推进,防止卡死 |
| `frame_id` | `base_link` | 发布到 `/goal_pose` 的坐标系 |

`path_mode` 支持:

| 模式 | 说明 |
|---|---|
| `lower_obstacle_circle` | 默认模式,在避障区域下方慢速画圆 |
| `avoidance_sweep` | 靠近机身/安装架约束的演示轨迹 |
| `front_wing_arc` | 前端左右机翼/前桨下方两点之间的宽松避障弧线 |
| `front_propeller_line` | 前桨外侧直线演示 |
| `circle` | 在 `base_link` 的 y-z 平面画圆 |
| `windylab` | 在 `base_link` 的 y-z 平面写简化 `windylab` |

### 6.3 新旧路径 demo 的区别

| 项目 | `move_ll100_lh420_nmpc_path_demo.py` | `move_ll100_lh420_nmpc_goal_demo.py` + `ll100_traj_pub.py` |
|---|---|---|
| 控制和轨迹 | 同一个节点里完成 | 拆成两个节点 |
| 目标来源 | 内置路径 | 任意 `/goal_pose` 发布者 |
| 推进方式 | 节点内部根据反馈推进 | 控制节点到达后发 reached,轨迹节点推进 |
| 适合用途 | 快速演示固定路径 | 手动目标、视觉目标、上层规划器接入 |
| 默认预测步数 | `8` | `6` |
| 默认 capsule 采样 | `3` 个内部点 | `1` 个内部点 |

### 6.4 LL100 轨迹可视化节点

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

RViz 会通过 `student_arm.rviz` 自动预置合并显示项 `LL100TrajectoryMarkers`。如果已经打开的是旧 RViz 配置,手动添加一个 `MarkerArray` 并选择:

```text
/ll100_trajectory/markers
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
| `trajectory_mode` | `lower_obstacle_circle` | 必须和轨迹发布节点的 `path_mode` 一致 |
| `period_sec` | `60.0` | 绘图期望轨迹的时间周期 |
| `expected_path_points` | `240` | RViz/Matplotlib 中期望路径采样点数 |
| `max_actual_points` | `2000` | RViz 中保留的实际轨迹点数 |
| `show_matplotlib` | `True` | 是否打开实时 Matplotlib 窗口 |
| `start_on_command` | `True` | 收到 `/student/joint_command` 后开始记录 |
| `ignore_initial_sec` | `1.0` | 记录开始后忽略前几秒 |

## 7. 避障模型

前桨避障:

- 从 `ll100_lh420.urdf` 的 `rot1_Joint` 和 `rot3_Joint` 推导前桨位置。
- 每个前桨建模为一个圆盘,圆盘行向量为 `[cx, cy, cz, nx, ny, nz, radius]`。
- ACADOS 代价中使用圆盘 SDF,再减去 `obstacle_safety_margin` 得到安全裕度。
- 默认前桨扫掠半径 `0.27 m`,圆盘 marker 厚度由 `obstacle_disk_half_thickness` 控制。

机身/安装架避障:

- 在 `base_link` 坐标系下用局部 box 覆盖底板、机身后部和机械臂安装架。
- 连杆避障不是完整 mesh 碰撞,而是把 `A_Link`~`F_Link` 相邻 frame 形成 capsule。
- 每段 capsule 取固定采样点,采样点对机身 box 的距离进入 ACADOS nonlinear least-squares cost。

姿态目标:

`vertical_down` 定义为末端坐标系 x 轴朝 `base_link` 的 `-Z`,采用完整旋转矩阵:

```text
末端 x 轴 -> [0, 0, -1]
末端 y 轴 -> [0, 1, 0]
末端 z 轴 -> [1, 0, 0]
```

## 8. 常见问题

### `/student/joint_command` 没有订阅者

通常是启动了显示版 launch。请使用:

```bash
ros2 launch manipulator ll100_lh420_control.launch.py
```

不要用 `ll100_lh420_display.launch.py` 跑 NMPC 控制。

### 找不到 `/ll100_nmpc/markers`

`/ll100_nmpc/markers` 只由单目标 demo 发布。路径版 demo 使用:

```text
/ll100_nmpc_path/markers
```

RViz 里需要手动添加 `MarkerArray`。

### ACADOS 每次都重新 build

求解器配置指纹变化会触发重新生成。常见触发项包括 `horizon_steps`、姿态权重、避障开关、机身 box/capsule 参数等。同配置下应复用 `/tmp/windylab_acados_*` 中的已编译求解器。

### 轨迹跟踪滞后或实际轨迹绕圈

优先检查 `max_joint_step` 是否大于底层 `max_velocity * 0.02`。上层 NMPC 走得比底层限速快时,实际机械臂会滞后。路径版默认启用 `advance_on_feedback`,可以避免目标按时间跑太快。

如果 command FK 也明显落后或变形,优先调大 `path_preview_stride` 到 `6` 或 `8`,并保持 `max_joint_step` 不超过底层实际速度能力。`circle` 模式的期望轨迹是半径 `0.08 m` 的真圆;如果图上不是圆,先确认控制节点和 `ll100_trajectory_visualizer_demo.py` 使用相同的 `trajectory_mode` 和 `period_sec`。

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
