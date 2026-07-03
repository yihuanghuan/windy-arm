# LL100-LH420 ACADOS NMPC 使用说明

本文档集中说明 LL100-LH420 整机模型的六轴 ACADOS NMPC demo。LL100 相关启动、避障、轨迹演示和常见问题以本文档为准。

## 1. 适用范围

- 控制对象:LL100-LH420 整机模型中的六轴机械臂 `A_Joint` ~ `F_Joint`。
- 可视化对象:整机 `ll100_lh420.urdf`,包含机体、4 个旋翼和机械臂。
- NMPC 模型:`ll100_lh420_nmpc.urdf`,只保留 A-F 六轴机械臂用于求解。
- 求解器:默认 acados(SQP_RTI + HPIPM);demo 可切 qpOASES / GRAMPC。
  **求解器选型/对比/安装/切换的完整说明,以 [LL100_LH420_NMPC_Solver_README.md](LL100_LH420_NMPC_Solver_README.md) 为准。**
  合格解:acados-HPIPM(默认)/ acados-qpOASES / GRAMPC;CasADi-IPOPT、SciPy-SLSQP 已弃置(慢,保留作离线参照)。
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

核心文件介绍：
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
| `period_sec` | `20.0` | 一圈时长,s(完成时间;越长越精准) |
| `path_mode` | `lower_obstacle_circle` | 不同轨迹模式切换(见下方 `path_mode` 支持表) |
| `max_joint_step` | `0.010` | 每周期最大关节步长,rad;**硬件派生 = `max_velocity`(0.5)·`control_dt`(0.02),非自由调参项** |
| `path_preview_stride` | `4` | 预测时域内相邻目标点间隔,避免时域只盯住当前点 |
| `horizon_steps` | `0(自动=按场景预设)` | 预测时域步数;见下方各场景调参推荐参数表 |
| `position_weight` | `-1.0(自动=按场景预设)` | 位置跟踪权重;见下方各场景调参推荐参数表 |
| `terminal_position_weight` | `-1.0(自动=按场景预设)` | 终端位置跟踪权重;见下方各场景调参推荐参数表 |
| `lock_orientation` | `True` | 默认锁定末端姿态 |
| `orientation_target` | `vertical_down` | 末端 x 轴朝 `-Z` |
| `orientation_weight` | `-1.0(自动=按场景预设)` | 姿态软约束权重;见下方各场景调参推荐参数表 |
| `terminal_orientation_weight` | `-1.0(自动=按场景预设)` | 终端姿态软约束权重;见下方各场景调参推荐参数表 |
| `obstacle_avoidance_enabled` | `True` | 是否启用前桨圆盘避障(固定为 ACADOS 非线性硬约束,无软约束开关) |
| `obstacle_safety_margin` | `0.005` | 前桨圆盘避障的 NMPC 硬约束安全余量(从 SDF 中减去),m;小正余量保证实际不接触 |
| `advance_on_feedback` | `False` | `False` 时按 `period_sec` 保持期望速度;`True` 时实际末端接近当前路径点后再推进 |
| `start_from_nearest_path_point` | `True` | 启动时从距离当前末端最近的路径点并入 |
| `hold_start_until_reached` | `True` | 从稳定弧线并入点进入容差后再启动时间轴 |
| `target_switch_tolerance` | `0.005` | 启动贴紧首点/路径点切换阈值,m(收紧→启动残差小、max 误差小) |
| `stuck_steps_before_advance` | `120` | 当前点误差长时间不下降时自动推进 |
| `stuck_advance_points` | `10` | 卡住时一次跳过的路径点数 |
| `body_obstacle_margin` | `0.005` | 机身/安装架 box 避障的 NMPC 硬约束余量(额外安全距离),m;小正余量保证实际不接触 |
| `capsule_radius` | `0.018` | 回退标量碰撞半径,m(仅在中心线模型与分段半径都为空时用) |
| `capsule_joint_radii` | `[0.052,0.062,0.058,0.050,0.049,0.040]` | 回退-各关节(电机)半径,m,A~F 帧(**当前被代码常量 `capsule_points` 中心线模型取代**,见下) |
| `capsule_link_radii` | `[0.045,0.035,0.043,0.016,0.035]` | 回退-各段连杆(杆身)半径,m,A→B~E→F 段(**当前被 `capsule_points` 取代**) |
| `capsule_samples_per_segment` | `3` | 每段连杆中间采样点数量;每段总采样点=此值+2(现为 5 点/段,减小长杆球间缝隙) |
| `current_posture_weight` | `-1.0(自动=按场景预设)` | 抑制相对当前姿态突变;见下方各场景调参推荐参数表 |
| `center_posture_weight` | `-1.0(自动=按场景预设)` | 抑制机械臂拉到近奇异伸直姿态;见下方各场景调参推荐参数表 |
| `joint_delta_weight` | `-1.0(自动=按场景预设)` | 一阶平滑权重(惩罚每步 Δq);见下方各场景调参推荐参数表 |
| `joint_delta_change_weight` | `-1.0(自动=按场景预设)` | 二阶平滑权重(惩罚 ΔΔq,抑抖);见下方各场景调参推荐参数表 |
| `front_obstacle_capsule` | `True` | 前桨按连杆 capsule 建模(含粗细),硬约束保证连杆表面离桨;`False`=仅关键点(旧,漏连杆粗细→真臂会擦桨) |
| `obstacle_repulsion_margin` | `0.03` | 障碍斥力场阈值,m;>0 启用,手臂进此距离内被斥力主动推离障碍(留净空);`0`=关 |
| `obstacle_repulsion_weight` | `1e4` | 斥力强度;默认开(NMPC 主动留净空、不贴硬约束墙),硬约束始终兜底。运行时可扫、不重编 |
| `gentle_start` | `True` | 启动阶段用 approach carrot;`False` 时保持旧行为 |
| `approach_speed` | `0.05` | carrot 沿当前末端→起点直线的移动速度,m/s |
| `approach_lead` | `0.04` | carrot 最多领先实际末端的距离,m;末端没跟上则暂停推进 |
| `start_reach_timeout_sec` | `8.0` | 启动到位判定超时,s |
| `start_relaxed_tolerance` | `0.02` | 超时后的放宽阈值,m;误差进入后启动滚动跟踪 |
| `min_manip_warn` | `0.012` | 可操作度低于此值打 WARN |

`path_mode` 支持:

| 模式 | 说明 |
|---|---|
| `lower_obstacle_circle` | 默认模式,末端朝下,在避障区域下方慢速画圆 |
| `avoidance_sweep` | 靠近机身/安装架约束的演示轨迹 |
| `front_wing_arc` | 右桨盘下方扫掠弧避障演示:两端点都在桨盘下方(z≈−0.22),中段升向桨盘(z≈−0.14、进斥力避障区)→ 避障把手臂顶在离桨 ~30mm 处,升段出现可见跟踪偏移(~11mm)= 避障在起作用;全程 `viol=0`、真臂不碰桨 |
| `circle` | 在 `base_link` 的 y-z 平面画圆 |
| `single_target` | 单目标模式 |

#### 各场景调参推荐参数(离线闭环坐标下降·反抖动版,2026-07-02)

**这些预设已烧录为各场景默认——裸跑 `-p path_mode:=<场景>` 即生效;显式 `-p` 传正值可逐项覆盖。**

下表为自动预设来源;无预设场景(`single_target` 等)仍回退到原全局默认。评分 = 安全硬门 + **高频震荡 hf(主目标,权重 2e4)** + 命令抖动 acc + 沿路径滞后 lag + 求解速度 + 精度 + 远奇异(实测数据发现旧评分无平滑度项,调出的参数末端误差有 ~4Hz 震荡,故重调)。CSV 明细在 `demo/data/tune_ll100_<场景>_*.csv`。

| 参数 | lower_obstacle_circle | avoidance_sweep | circle | front_wing_arc |
|---|---:|---:|---:|---:|
| `horizon_steps` | 6 | 8 | 4 | 6 |
| `position_weight` | 180 | 252 | 120 | 180 |
| `terminal_position_weight` | 1400 | 2800 | 1020 | 1200 |
| `orientation_weight` | 0.105 | 0.105 | 0.30 | 0.30 |
| `terminal_orientation_weight` | 3.0 | 1.05 | 2.1 | 3.0 |
| `joint_delta_weight` | 0.028 | 0.224 | 0.224 | 0.16 |
| `joint_delta_change_weight` | 0.175 | 0.70 | 0.70 | 0.50 |
| `current_posture_weight` | 0.014 | 0.0085 | 0.007 | 0.002 |
| `center_posture_weight` | 0.0043 | 0.0043 | 0.0035 | 0.0085 |
| 效果 rms/max/hf/求解 | 0.4/1.7mm/**0.00**/1.6ms | 5.3/14.1mm/0.33/2.0ms | 2.5/5.7mm/0.05/**1.0ms** | 3.0/7.4mm/0.11/1.5ms |

hf = 稳态误差去 0.5s 趋势后的 RMS(高频震荡量,mm)。相比旧版(无平滑度项):高频震荡从 0.4-0.9mm 压到 0-0.33mm;circle/front_wing_arc 用少量 rms 换平滑(用户要求"误差不能有高频震荡"优先)。front_wing_arc 的命令抖动 acc≈7 rad/s² 仍偏高(该路径 min-manip ~0.003 近奇异所致,物理上限)。可视化验证看 visualizer 新增的 `path_distance_error` 曲线(到路径曲线的几何距离,不受期望时间轴相位伪影影响;旧的 expected−actual 误差曲线含 100mm 级相位伪影,勿用来判跟踪精度)。

`max_joint_step` 是硬件速度约束在 OCP 里的镜像,不是自由调参项:它必须等于底层单周期能走的关节量 = `max_velocity · control_dt`。`ll100_lh420_control.launch.py` 默认 `max_velocity=0.5 rad/s`、50 Hz 控制周期,故默认:

```text
max_joint_step = max_velocity * 0.02 = 0.5 * 0.02 = 0.010
```

设得比它**大**,MPC 会规划底层跟不上的速度(规划/执行不匹配、实际末端滞后,还会侵蚀避障余量);设得比它**小**则人为限速、跟踪滞后。**要更快只能提 `max_velocity`**,再按上式同步放大 `max_joint_step`。例如 launch 用 `max_velocity:=1.0` 时:

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

### 4.4 可达范围探测工具（安全+无奇异）

`ll100_workspace_probe.py`：离线扫描机械臂在**避障约束下**的"安全可达区"——即末端**竖直朝下**、**远离奇异**、且臂与**桨盘/机身无碰撞**的所有可达末端位置,在 RViz 里用**体素方块**画出来、按**可操作度**上色(绿=远离奇异 → 黄 → 红=接近奇异阈值)。碰撞用与主 demo 完全相同的中心线胶囊模型(复用同一 `CAPSULE_CENTERLINE_POINTS`),所以能吃到你微调过的小球。

原理(两阶段):
1. **蒙特卡洛粗扫**:关节空间随机采样 → 逐构型过滤(①末端 **x 轴与 −Z 夹角** ≤ 容差,绕竖直轴 yaw 自由——与 demo 软姿态锁语义一致 ②可操作度 `sqrt(det(J·Jᵀ)) ≥` 阈值 ③所有碰撞球对桨盘/机身 `margin ≥ extra_clearance`)→ 体素合并、取体素内最大可操作度上色。
2. **前方 ROI 逐体素 IK 精查**:蒙特卡洛在笛卡尔空间不均匀,"折叠着往前下方伸+朝下"的构型是 6 维里极薄的流形,随机采样几乎砸不中——**MC 图上空 ≠ 不可达**。对 ROI 里 MC 没填上的体素,逐个用 DLS-IK(多 yaw 目标 + 上一体素解 warm start + 最近 MC 种子)求解,成功再过奇异+碰撞筛;多种子全失败才保持空(此时"大概率不可达"才成立)。

先起控制版 launch(§3,提供 RViz + robot_state_publisher),再另开终端:

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
python3 ll100_workspace_probe.py
```

RViz 里 Add → By topic → `/ll100_workspace/reachable`(可达体素)和 `/ll100_workspace/obstacles`(桨盘+机身,便于对照)。启动时算一次(MC 几秒 + IK 精查几分钟,看日志进度),之后 latched 重发。

| 参数 | 默认 | 说明 |
|---|---:|---|
| `num_samples` | `300000` | MC 采样数(轴锥判据命中率 ~3%,全局粗扫够用;局部细节靠 IK 精查,不靠堆采样) |
| `voxel_size` | `0.02` | 体素边长,m |
| `manipulability_threshold` | `0.012` | 可操作度低于此判为接近奇异,剔除(= 调参器 `SINGULAR_SOFT_TARGET`) |
| `orientation_tol_deg` | `20.0` | 末端 x 轴与 −Z 的最大夹角,°(yaw 自由) |
| `require_vertical_down` | `True` | 设 `false` = 不限姿态,得到完整可达体积(范围明显更大) |
| `extra_clearance` | `0.0` | 在内置 margin 之外再加的安全净空,m(调大→区域缩小、更远离障碍) |
| `grid_ik_enabled` | `True` | 前方 ROI 逐体素 IK 精查开关 |
| `grid_ik_region` | `[0.40,0.78,-0.35,0.35,-0.32,0.08]` | 精查 ROI `[xmin,xmax,ymin,ymax,zmin,zmax]`,base_link 系,默认盖住前方工作区(含 front_wing_arc) |
| `grid_ik_yaw_count` | `3` | 每体素尝试的竖直朝下 yaw 变体数(覆盖不同腕部取向分支) |
| `seed` | `0` | 采样随机种子,可复现 |

用途:放目标点前先在这张图上确认"在不在绿区",避免像 front_wing_arc 那样盲试不可行目标。日志会打印 MC 计数、IK 精查(尝试/新增可达/预筛跳过)、可达 bbox、可操作度范围。

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
  -p period_sec:=20.0
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
| `trajectory_mode` | `lower_obstacle_circle` | 期望轨迹模式,共 4 种,与 demo 相同，但single_target不计入，因为单目标点没有连续路径可绘制;必须和路径 demo 的 `path_mode` 一致 |
| `period_sec` | `60.0` | 绘图期望轨迹的时间周期 |
| `expected_path_points` | `240` | RViz/Matplotlib 中期望路径采样点数 |
| `max_actual_points` | `2000` | RViz 中保留的实际轨迹点数 |
| `show_matplotlib` | `True` | 是否打开实时 Matplotlib 窗口 |
| `start_on_command` | `True` | 收到 `/student/joint_command` 后开始记录 |
| `use_message_stamp` | `False` | 默认用可视化节点接收时钟统一 `/student/joint_command` 和 `/joint_states` 时间 |
| `start_when_on_expected_path` | `True` | 实际末端进入期望轨迹入口点容差后才把记录时间置零 |
| `start_path_tolerance` | `0.025` | 判定“已经到达期望轨迹入口点”的距离阈值,m |
| `use_stable_front_wing_arc_start` | `True` | `front_wing_arc` 远离轨迹启动时从弧线中点之一并入,避免从端点并入 |
| `reset_gap_sec` | `2.0` | 命令中断超过该间隔后重置记录,开始新一段轨迹 |
| `stale_command_timeout_sec` | `0.2` | 超过该时间没收到新命令则认为命令过期/已停 |
| `ignore_initial_sec` | `1.0` | 记录开始后忽略前几秒 |
| `sample_only_on_actual_change` | `True` | 仅当实际末端发生变化时才采样,避免静止时堆积重复点 |
| `actual_change_epsilon` | `1e-9` | 判定实际末端“发生变化”的位移阈值,m |
| `record_command_fk` | `True` | 是否额外记录命令关节角 FK 得到的 command 轨迹 |
| `obstacle_safety_margin` | `0.04` | 仅可视化:前桨圆盘障碍的安全余量,m(不参与 NMPC 求解) |
| `obstacle_disk_half_thickness` | `0.02` | 仅可视化:障碍圆盘半厚度,m |
| `body_obstacle_margin` | `0.015` | 仅可视化:机身/安装架 box 障碍的额外安全距离,m |
| `capsule_radius` | `0.018` | 仅回退:分段半径留空时的标量;RViz 逐采样点按各自半径(关节粗/连杆细)画青色球 |
| `data_dir` | `.../arm-platform/demo/data` | 记录数据(轨迹/图)的保存目录 |


## 6. 避障模型

前桨避障:

- 从 `ll100_lh420.urdf` 的 `rot1_Joint` 和 `rot3_Joint` 推导前桨位置。
- 每个前桨建模为一个圆盘,圆盘行向量为 `[cx, cy, cz, nx, ny, nz, radius]`。
- ACADOS 中使用圆盘 SDF,再减去 `obstacle_safety_margin` 得到安全裕度;默认作为 nonlinear hard constraint。
- 默认前桨扫掠半径 `0.27 m`,圆盘 marker 厚度由 `obstacle_disk_half_thickness` 控制。

机身/安装架避障:

- 在 `base_link` 坐标系下用局部 box 覆盖底板、机身后部和机械臂安装架。
- 连杆避障不是完整 mesh 碰撞,而是把臂离散成一串带半径的球(球的并集≈胶囊/圆柱)。
- **中心线模型(默认,最精确)**:`capsule_points` 给出 37 个球,球心=各连杆 STL 网格**沿关节轴的横截面质心**(贴住真实杆中心线,不再落在关节连线上),半径=该处真实粗细(细杆 ~25–31mm、电机 ~56–78mm,含无缝覆盖膨胀)。采样间距 30mm、`R=√(局部粗细²+15²)+6mm`,**全程无缝**(球采样对半径=杆粗的圆柱永远有缝,故半径必须略大于杆粗)。由网格离线生成。
- **回退:分段半径**(`capsule_points` 为空时):沿 `A_Link`~`F_Link` 关节连线采样,端点取 `capsule_joint_radii`、段内取 `capsule_link_radii`。缺点是圆心在关节连线上、偏离真实杆中心线,半径被撑大偏保守。
- 采样点默认作为 ACADOS nonlinear hard constraint;斥力代价复用同一组 margin。

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
