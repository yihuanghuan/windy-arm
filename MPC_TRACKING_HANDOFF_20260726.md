# 7-DOF 浮动基座 NMPC 跟踪问题交接（2026-07-26）

> **✅ 已解决（2026-07-26 晚，见本节；以下原文保留作历史记录）**
>
> ## 0. 解决方案与结果
>
> 用户放开了原"不可更改条件"：控制频率、|dq|/|ddq| 硬约束、
> `velocity_feedforward_gain` 均可修改，但关节约束须符合真实硬件
> （达妙 DM4340-48V：3.0 rad/s；RS05：5.0 rad/s，见 `config/motor_config.yaml`）。
> 控制频率保持 50 Hz 未动。
>
> ### 根因（实测证实，非推测）
>
> 螺旋不是权重问题，也不是"硬约束下不可行"（§5.2 的推断不成立——ILC 实验
> 本身就证明了 0.6mm 的 2Hz 可达）。实测执行链 2 Hz 传递函数 |H|≈0.94、
> 滞后≈40ms（7 关节完全一致；成分：反馈龄期 8ms + 发布 9ms + plant 100Hz
> 采样保持 ~5ms + 加速度限幅动态），叠加两类结构性失配：
>
> 1. **时序失配**：MPC 假设命令在求解瞬间生效、x0 为当前状态；实际 x0 陈旧
>    8ms、命令 ~14ms 后才生效。发布"区间末速度 dq_{k+1}"带来的半拍超前只
>    部分掩盖该滞后。确定性 2Hz 残差 ≈ 该相位差 × 10mm 基座补偿幅值。
> 2. **饱和失真**：|ddq|≤5 使 plant 侧加速度限幅在 83% 子步上活跃（每拍速度
>    阶跃 ≤0.1 rad/s > 单子步限幅 0.05），产生 ~2.4mm 宽带残差（基线分解中
>    非 2Hz 的部分）。电机真实能力远高于 5 rad/s²，此约束是教学保守值。
>
> ### 修改（代码默认值即新行为，三终端命令不变）
>
> - `launch/student_arm.launch.py`：`max_velocity` 1.0→3.0、`max_acceleration`
>   5.0→20.0（硬件真实规格）。
> - `demo/move_base_circle_mpc_ik_demo.py`：
>   - `second_order_max_velocity` 1.0→2.0、`second_order_max_acceleration`
>     5.0→15.0、`max_joint_step` 0.02→0.04（留 plant 执行/逆变换余量）；
>   - **Smith 预测器**（`smith_predictor_enabled`，默认开）：用已知 plant 执行律
>     （`smooth_position_controller.cc` 前馈分支）+ 在途命令队列把 x0 前推到
>     命令生效时刻。注意旧 `second_order_state_prediction`（ddq0 外推）与
>     plant 实际执行不一致，离线闭环证实会正反馈发散，保持关闭；
>   - **命令逆变换**（`command_inversion_enabled`，默认开）：发布
>     `v_c = Δq·rⁿ/(τ(1−rⁿ))`，r=1−h/τ（h=10ms, τ=0.15, n=2），使 plant 两个
>     子步后精确落在计划点，替代直接发布 dq_{k+1}（消除半拍超前 + 位置反馈项
>     ~13% 增益）；
>   - `latency_compensation_enabled` 默认开（nominal 9ms + actuator 5ms）；
>   - 首个 track tick 发布后把轨迹原点重锚到发布时刻——visualizer 以第一条
>     track 命令 stamp 为原点，原先两节点时钟差首拍发布延迟（6~17ms，逐次
>     不同），会在定时误差 KPI 中伪造 1~2mm 滞后分量。
> - 新增 `demo/hifi_offline_loop.py`：事件时序高保真离线闭环（100Hz plant 网格、
>   发布延迟、反馈龄期），离线即可复现基线 3.61mm 并迭代方案，无需 3 终端。
>
> ### 实测结果（与 §4 基线同口径，visualizer actual CSV）
>
> whole（`moving_base_circle_whole_trajectory_20260726_021228.csv`，
> loop `moving_base_mpc_loop_20260726_021045.csv`）：
>
> | 任务 | RMS (mm) | P95 (mm) | Max (mm) | 2Hz RMS (mm) |
> |---|---:|---:|---:|---:|
> | circle | 4.35→**0.37** | 6.55→**0.62** | 7.70→**0.82** | 1.34→**0.28** |
> | triangle | 3.46→**0.28** | 5.59→**0.45** | 7.82→**0.67** | 2.05→**0.25** |
> | windylab | 3.22→**0.25** | 5.33→**0.36** | 7.86→**0.80** | 1.98→**0.21** |
> | sine | 2.81→**0.28** | 4.44→**0.42** | 5.97→**0.56** | 1.30→**0.26** |
> | square | 3.57→**0.26** | 5.59→**0.37** | 6.77→**0.61** | 2.90→**0.22** |
> | **whole 合计** | 3.43→**0.27** | 5.68→**0.43** | 7.86→**0.82** | 1.50→**0.22** |
>
> 实时性与硬门（whole，5133 tick）：timer P95/P99/max = 20.13/23.63/32.45 ms
> （基线 20.14/22.88/32.58）；guard block/projection = 0；command_usable
> 5133/5133；late recovery = 3；加速度约束活跃率 0.62–0.73→**0.084**；实测
> |dq| 峰值 1.91 rad/s（硬件限 3.0）。实际 world 姿态误差（§7 缺口已补，
> 由 loop CSV q_meas FK 计算）：mean/P95/max = 2.05°/4.02°/6.33°
> （基线 2.83°/6.12°/13.97°），位置精度不以姿态劣化为代价。
>
> 单独 circle（`moving_base_circle_circle_trajectory_20260726_021006.csv`）：
> 稳态各圈 RMS 0.28–0.65mm、2Hz 0.24–0.48mm（基线 3.44/2.37）。
>
> 回退/A-B：`smith_predictor_enabled:=false command_inversion_enabled:=false
> latency_compensation_enabled:=false second_order_max_velocity:=1.0
> second_order_max_acceleration:=5.0 max_joint_step:=0.02` 加 launch
> `max_velocity:=1.0 max_acceleration:=5.0` 即恢复原基线行为。
>
> ### 两个运行陷阱（都遇到过，务必注意）
>
> 1. **启动顺序必须 终端1(plant) → 终端3(visualizer) → 终端2(MPC)**。
>    visualizer 以第一条 `moving_base_track` 命令的 stamp 为时间原点；
>    MPC 已在跑时才启动它，KPI 因时钟错位全是伪影（可到 2R=160mm），
>    但画出的轨迹形状不受影响。
> 2. **残留 FastDDS 共享内存会打爆跟踪**（20260726 02:29 实测复现：
>    `moving_base_mpc_loop_20260726_022959.csv`，whole RMS 20.7mm、
>    2Hz 17.6mm）。被 kill 的节点在 `/dev/shm` 留下 `fastrtps_*` 段/端口
>    文件后，新节点 SHM 传输失效回落 UDP 心跳修复路径，/joint_states
>    投递恒定 ~93ms 延迟 → 反馈陈旧 → state_safe_filter 投影 48% 命令 →
>    基座补偿失效。症状：loop CSV 中 `js_recv_wall_sec - js_header_stamp_sec`
>    P50 从 <1ms 变成 ~93ms。处置：停所有 ROS 节点后
>    `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*` 再重启。
>    demo 已加自检：该间隔持续 >30ms 达 1 秒即打 ERROR 日志提示清理。

> ## 0.5 不确定性实验台（第1/2级台阶，2026-07-26 深夜新增）
>
> 面向"无人机=有不确定性的合作者"研究方向（uncertainty-aware cooperation），
> 在真值/信念分离的架构上实现了两级实验：
>
> - **真值通道**：基座真实位姿 = 名义解析轨迹 + 有界多谐波位置扰动
>   （`moving_base_trajectory_config.base_disturbance_offset`，每轴
>   |Δp|≤amplitude 的 ∞-范数界，频带 0.15–2.5Hz 模拟无人机漂移+抖动，
>   seed 确定性可复算）。visualizer 用真值算 actual/KPI。
> - **信念通道**：MPC 端 `base_belief_mode` 三档——`nominal`（只知名义，
>   第1级"无知下界"）/ `truth`（全知对照，默认，amp=0 时与现状完全一致）/
>   `estimated`（第2级：逐拍模拟状态估计 = 真值(t−延迟)+高斯噪声 经一阶
>   低通，预测域内常值保持）。
>
> 关键参数（demo 与 visualizer 的 amplitude/seed **必须一致**）：
> `base_disturbance_amplitude_m / base_disturbance_seed / base_belief_mode /
> base_estimator_noise_std_m / base_estimator_delay_ms / base_estimator_lpf_hz /
> base_estimator_seed`
>
> 离线（`hifi_offline_loop.py --set uncertainty`）与 ROS 实机互相印证
> （circle, 扰动 ∞-界 10mm/每轴 RMS≈3mm，向量 RMS≈5mm）：
>
> | 信念 | 离线 RMS | ROS 实测 RMS | 含义 |
> |---|---:|---:|---|
> | truth | 0.495 | 0.79 | 全知上界：限幅余量足以完全补偿 |
> | nominal | 4.78 | 5.07 | 无知下界：误差≈扰动本身 |
> | estimated σ=2mm,40ms | 4.26 | **4.26** | 第2级中间点，离线/实机完全一致 |
>
> 离线退化网格（amp=10mm, estimated, lpf=10Hz）RMS mm：
>
> | σ\延迟 | 0ms | 40ms | 100ms |
> |---|---:|---:|---:|
> | 0mm | 1.88 | 3.25 | 5.05 |
> | 2mm | 3.23 | 4.26 | 5.81 |
> | 5mm | 6.63 | 7.24 | 8.22 |
>
> 两个对论文有价值的观察：
> 1. 即使 σ=0、零延迟，仅"低通+预测域内常值保持"就把 0.5mm 变 1.9mm——
>    **预测域内扰动外推模型是主要瓶颈**，谐波/振荡器模型的预测估计器
>    （如 Kalman+振荡器状态）是 uncertainty-aware 方法的直接切入点；
> 2. σ≥2mm 的估计噪声让 plant 加速度限幅重新活跃（~40% 子步）——
>    估计噪声直接消耗执行预算，鲁棒方法需显式权衡"跟估计"与"省加速度"。
>
> 第3级（双向协作）依赖把基座状态改走 topic（本实验台已把真值/信念
> 分离，是其前置），尚未实现。

> ## 0.6 不确定性感知 P0–P3 工作流（2026-07-26 白天新增，指向独立报告）
>
> §0.5 之上的第 3 级台阶已落地：`base_trust_estimator.py` 三层信任估计器
> （谱辨识谐波 RLS + β 信任标量 + 保形 δ/ACI，Mondrian 分组），接入 demo
> `base_belief_mode:=model`；扰动契约扩展 `base_disturbance_profile`
> （gust/fault/wideband）；一级信任降速阶梯 `trust_ladder_enabled`；
> 离线 48 跑 + ROS whole 12+2 跑实验矩阵（四线阶梯 nominal 5.16 >
> estimated 4.05 > model 3.57 > truth 0.48 mm 稳态成立）；model 档重辨识
> 实时性违规（timer P99 34.7ms）已修复为分摊状态机 + BLAS 单线程
> （复验 P99 23.4ms、稳态 0.94mm）。完整交付清单、论文逐节映射、阶梯
> fault 对比的如实结论、复现命令与数据文件清单见仓库根
> **`UNCERTAINTY_P0P3_REPORT.md`**；离线矩阵表见 `data_uncertainty_matrix.md`。

> ## 0.7 后续增强：β 门控与一级阶梯闭环（2026-07-26 晚）
>
> §0.6 初版遗留的一级阶梯标定问题已解决。β 信任不再使用
> `E[β]/β_max`（该定义把健康 β≈1 映成 b≈0.1），改为在
> `β_degraded=0.6`、`β_healthy=0.9` 两个物理精度锚点间连续标定：
> 健康 b=1.000，10mm 故障后 0.06s 内跌破 0.3，恢复后 3.06s 内回到 0.7。
>
> model 前馈现在按 `Δ̂_ff=g(b)Δ̂` 平滑门控。离线 3 seeds 正式 A/B：
> harmonic 未门控/门控 1.645/1.638mm（收益保留），fault
> 2.875/**2.279mm**（消除倒挂，−20.7%）。ROS harmonic/model 三条 whole
> 稳态 mean 为 0.964/0.924/0.961mm，均值 **0.950mm**，guard block/fault
> 全零；相对修复前 3.57mm 改善 73.4%。
>
> 一级阶梯改为四态状态机（warming/healthy/degraded/recovering），δ 门使用
> 在线健康中位基线的 1.35/1.2 倍双阈值，b 门使用 0.3/0.7 双阈值，并加
> 1s trust commission 与 2s 恢复防抖。fault whole 正式行为：
> 4.01s 基线就绪，6.15s 降级，scale 最低 0.692，23.51s 恢复，25s 后恒为 1；
> guard block/fault=0/0、solver success=100%。因此“β 加权前馈”和
> “一级阶梯重标定”均从未做/待标定清单移除；尚未做的只剩 §4.4 二、三级
> （姿态切换、抬笔退避）。
>
> ROS 验收期间宿主机的 Fast-DDS 域 0 出现外部 `/joint_states` participant；
> 正式 ON 行使用独立 `ROS_DOMAIN_ID=78`，并让三个终端统一加载
> `demo/fastdds_udp_only.xml` 禁用 SHM。参数快照 `publishers=1, valid=true`；
> 受重复发布者/反馈陈旧影响的过程 run 已明确作废。完整数值和数据 stamp
> 见 `UNCERTAINTY_P0P3_REPORT.md` §2.7。

## 1. 目标与不可更改条件

目标是在解析浮动基座运动下，让 7-DOF 机械臂末端在 `world` 系稳定绘制：

- 单任务：圆、三角、正方形、正弦、`windylab`
- `whole`：正面圆 → 左侧三角 → 背面 `windylab` → 右侧正弦 → 顶部正方形
- 抬笔段不能在可视化中连线；`windylab` 字母之间有空隙

用户明确要求不能放宽硬件相关约束：

- 控制频率：50 Hz，`dt=0.02 s`
- 关节速度硬约束：`|dq| <= 1 rad/s`
- 关节加速度硬约束：`|ddq| <= 5 rad/s²`
- 启动 plant 时 `velocity_feedforward_gain:=1.0`

当前问题不是守卫崩溃，而是落笔时实际轨迹围绕目标线周期摆动，直线呈“螺旋/绕线”，末端视觉上不稳定。

## 2. 仓库与当前代码状态

- 工作区：`/home/windylab/code/windylab_ws`
- 主要文件：
  - `src/arm-platform/demo/move_base_circle_mpc_ik_demo.py`
  - `src/arm-platform/demo/moving_base_trajectory_config.py`
  - `src/arm-platform/demo/second_order_mpc_ik.py`
  - `src/arm-platform/demo/trajectory_visualizer_demo.py`
  - `src/arm-platform/demo/tune_unified_mpc_params.py`
  - `src/arm-platform/demo/analyze_moving_base_run.py`
- 当前分支：`position-only-baseline`
- 当前 HEAD：`06873d4`
- 工作树有大量未提交修改，不要 `reset --hard`，不要覆盖用户改动。先看：

```bash
cd /home/windylab/code/windylab_ws
git status --short
git diff --stat
```

当前生产默认仍是 position-only 二阶 NMPC：

```text
second_order_horizon_steps = 10
second_order_position_weight = 5000
second_order_terminal_position_weight = 500
second_order_cartesian_velocity_weight = 0
second_order_cartesian_acceleration_weight = 0
second_order_velocity_weight = 0.02
second_order_acceleration_weight = 0
orientation_weight = 0.09375
terminal_orientation_weight = 1.25
reference_lead_steps = 1
base_reference_lead_steps = -1  # 继承 1
second_order_contour_tracking_enabled = false
online_time_scaling_enabled = false
```

本轮加入了两个**默认关闭/不改变现有启动行为**的实验接口：

- 周期误差学习前馈：`periodic_error_learning_enabled:=false`
- 基座与末端路径分离预瞄：`base_reference_lead_steps:=-1`

它们只用于 A/B，目前不能作为 `whole` 默认方案。

## 3. 基座与轨迹工况

`moving_base_trajectory_config.py` 当前默认：

```text
BASE_CIRCLE_RADIUS = 0.01 m
BASE_Z_AMPLITUDE = 0.01 m
BASE_PERIOD_SEC = 0.5 s  # 2 Hz
BASE_ATTITUDE_AMPLITUDE_DEG = 0.1 deg
EE draw circle radius = 0.08 m
```

2 Hz、1 cm 平移对应的峰值基座加速度约为：

```math
a_\text{peak} = A(2\pi f)^2
              = 0.01(4\pi)^2
              \approx 1.58\ \mathrm{m/s^2}
```

这不是很小的扰动；在当前姿态下，为维持 world 末端静止/绘图，关节加速度预算长期紧张。

## 4. 可靠基线

可靠的 whole N=10 基线：

- 控制环：`src/arm-platform/demo/data/moving_base_mpc_loop_20260726_000558.csv`
- 实际轨迹：`src/arm-platform/demo/data/moving_base_circle_whole_trajectory_20260726_000731.csv`
- 图片：同名前缀 `.png`

第一轮 whole 落笔统计：

| 任务 | RMS (mm) | P95 (mm) | Max (mm) | 2 Hz 分量 RMS (mm) |
|---|---:|---:|---:|---:|
| circle（记录到 t=3–4 s） | 4.427 | 6.078 | 6.186 | 3.252 |
| triangle | 3.483 | 5.689 | 7.818 | 2.260 |
| windylab | 3.223 | 5.325 | 7.863 | 1.967 |
| sine | 2.814 | 4.438 | 5.974 | 1.300 |
| square | 3.571 | 5.593 | 6.768 | 2.906 |
| whole 落笔合计 | **3.329** | **5.444** | **7.863** | — |

基线实时性：

```text
timer interval P95 = 20.144 ms
timer interval P99 = 22.883 ms
max = 32.579 ms
late recovery = 1
guard block = 0
dq_prev feedback fallback ≈ 2.22%
```

单独 circle、正确 `reference_lead_steps=1` 的基线：

- `moving_base_circle_circle_trajectory_20260726_001602.csv`
- 稳态约 RMS 3.437 mm、P95 5.519 mm、Max 7.324 mm、2 Hz 分量 2.372 mm

## 5. 已确认的主要问题

### 5.1 “螺旋”主要是确定性的 2 Hz 残差

误差频谱/正余弦回归显示，明显摆动与基座 2 Hz 同频，不是普通随机噪声，也不是轨迹没有收敛。N=10 circle/whole 中，2 Hz 分量通常约 1.3–3.3 mm。

更准确地说，这是在固定基座周期、执行链相位/幅值响应和关节硬约束共同作用下形成的周期极限环。

### 5.2 加速度硬约束长期活跃

在可靠 whole 基线中，每个落笔 tick 的预测域里都至少有一些 `ddq` 贴 ±5 rad/s²。所有预测加速度变量中，贴上限的平均比例约：

| 任务 | 基线加速度 active ratio |
|---|---:|
| circle | 0.640 |
| triangle | 0.678 |
| windylab | 0.731 |
| sine | 0.675 |
| square | 0.621 |

这说明误差不只是“权重没调好”。当前 NMPC 经常处于受限跟踪状态；简单提高位置权重或直接叠加补偿会把更多变量推到硬上限，并可能让别的构型变差。

这还不是严格的不可行性证明。下一步应做 acceleration-level feasibility/QP 分析，计算在每个任务、每个姿态下满足 `dq/ddq` 硬约束时的最小笛卡尔位置残差。

### 5.3 录屏/资源抖动是次要问题，不是螺旋根因

早期 N=20 加可视化/录屏时：

- 50 Hz deadline 经常失守
- `/joint_states` 旧样本排队
- 守卫可能连续拦截并 fault

已经实施：

- `/joint_states` QoS depth=1
- 单线程 executor，避免 Python 多线程 GIL 抢占
- 超过 `1.5*dt=30 ms` 时丢弃过期 warm start/commanded velocity
- 控制阶段禁用 Python GC

这些修改把严重资源型失败修复为可恢复的 deadline miss；可靠 N=10 whole 已能 `guard=0`。但是即使实时性正常，2 Hz 螺旋仍在，所以资源不是几何误差的根因。

## 6. 已测试并否决的方案

### 6.1 N=20 / 更长时域

- N=20 whole 的完整控制周期 P95 曾达到约 26.7 ms，超过 20 ms deadline。
- 非线性笛卡尔残差 N=20 circle 实测 RMS 9.114 mm，求解/反馈调度明显恶化。
- N=12 离线看起来比 N=10 好，circle 实测 2 Hz 分量约降低 28%，但 whole：
  - RMS 4.239 mm（基线 3.329）
  - P95 6.955 mm
  - timer P99 27.127 ms
- 因此生产默认保持 N=10。

### 6.2 显式速度/加速度残差

测试过：

```math
J(q)\dot q-v_{\mathrm{ref},B}
```

```math
J(q)\ddot q+\dot J(q,\dot q)\dot q-a_{\mathrm{ref},B}
```

以及 world 末端速度、基座线/角速度、基座加速度的参考导数。

结果：

- 与 stage-wise 可行投影、原始定时位置参考发生冲突
- 加速度约束更频繁活跃
- 曾出现求解 status=4、守卫连续拦截
- nonlinear 模式又破坏实时性

所以当前默认权重均为 0，只保留代码作显式实验。

### 6.3 轮廓/lag 代价

`contour_weight=5000, lag_weight=2500, cart_velocity_weight=0.5` 的 N=10 live A/B：

- 基线 whole 周期落笔 RMS 3.329 mm
- 候选约 3.807 mm，后续周期也更差
- timer P99 上升

默认保持 `second_order_contour_tracking_enabled=false`。

### 6.4 常规权重自动调参

自动/手工测试过：

- terminal position weight 500 → 1000
- joint velocity weight 0.02 → 0
- position/terminal/速度/加速度/posture 局部扫描
- posture current/center 权重 0.5×、2×

结论：

- 某些候选在重复 circle 或离线指标上略好
- 到 whole 后变差、加速度 active ratio 上升，或出现 solver status=4
- 不应只依据离线 circle 或 solver `first_error` 选参数

### 6.5 周期误差学习前馈

新增了按基座相位学习 `sin/cos` world 补偿的实验实现，硬约束不变。

逐 tick LMS 会把非周期误差调制进参考，已改成每个完整基座周期批量更新。对**重复 circle** 很有效：

- 数据：`moving_base_circle_circle_trajectory_20260726_005304.csv`
- RMS：约 3.44 → 2.97–3.05 mm
- P95：约 5.2–5.9 → 4.7–4.9 mm
- 2 Hz：约 2.24–2.56 → 0.56–0.69 mm（约降低 74%）
- guard/late/constraint violation 均为 0

但在 `whole` 单次路径上不通用：

- 数据：`moving_base_circle_whole_trajectory_20260726_005523.csv`
- whole RMS：3.329 → 3.344 mm（无改善）
- triangle 改善，circle/sine/square 变差
- 预测加速度 active ratio 进一步上升

固定一组从 circle 标定的系数也失败：

- 数据：`moving_base_circle_whole_trajectory_20260726_005903.csv`
- circle 2 Hz 降到约 0.98 mm
- triangle RMS 却从 3.48 恶化到 4.60 mm

结论：周期 ILC 适合同一路径重复运行，或以后做按任务/路径相位持久化标定；不适合直接作为 whole 第一次运行的统一补偿。默认保持关闭。

### 6.6 分离基座/末端路径预瞄

当前默认二者均为 1 拍。做了：

| 基座预瞄 | 末端路径预瞄 | circle RMS | 2 Hz RMS |
|---:|---:|---:|---:|
| 0 拍 | 1 拍 | 5.308 mm | 4.512 mm |
| 1 拍（默认） | 1 拍 | 3.437 mm | 2.372 mm |
| 2 拍 | 1 拍 | 4.373 mm | 3.124 mm |

数据：

- 0 拍：`moving_base_circle_circle_trajectory_20260726_010221.csv`
- 2 拍：`moving_base_circle_circle_trajectory_20260726_010112.csv`

结论：当前 1 拍正确；螺旋不能用一个额外时延参数解决。

## 7. 尚未完成、但初步有希望的实验

将姿态软代价缩到 1/4：

```text
orientation_weight = 0.0234375
terminal_orientation_weight = 0.3125
```

注意：这是命令行覆盖，**代码默认值没有改**。

数据：

- 参数：`moving_base_mpc_params_20260726_010410.json`
- 控制环：`moving_base_mpc_loop_20260726_010410.csv`
- 实际轨迹：`moving_base_circle_whole_trajectory_20260726_010517.csv`

运行到 t=64.15 s 后因会话交接停止；guard=0、late recovery=0、timer P99=22.30 ms。

与基线对比：

| 任务 | 基线 RMS | 低姿态权重 RMS | 结论 |
|---|---:|---:|---|
| circle | 需要用第二周期或重跑公平比较 | 4.233 mm | 暂不能定论 |
| triangle | 3.483 | 3.503 mm | 基本不变 |
| windylab | 3.223 | 2.865 mm | 改善约 11% |
| sine | 2.814 | 2.679 mm | 改善约 5% |
| square 前 2.71 s | 3.781 | 3.744 mm | 略好，未完成 |

重要缺口：目前 loop CSV 没有完整记录实际 world 姿态误差。不能只看位置变好就接受，需要：

1. 完成至少一个完整 whole 周期；
2. 记录实际末端 world orientation error（而非只看 solver terminal shadow）；
3. 确认侧面/顶部笔尖方向仍满足绘图要求；
4. 再做原权重/1/2/1/4 的同负载 A/B。

## 8. 建议下一个 AI 优先处理的研究问题

### 优先级 A：先做严格可行性分析

不要继续盲调 scalar weight。对整条 whole 参考，在每个采样点求解 acceleration-level QP：

```math
\min_{\dot q,\ddot q,s_p,s_R}
\|s_p\|^2 + \lambda_R\|s_R\|^2
 \lambda_q\|\ddot q\|^2
 \lambda_n\|N(q)(q-q_\mathrm{pref})\|^2
```

subject to:

```math
J_v(q)\dot q = v_{\mathrm{ref},B} + s_v
```

```math
J_v(q)\ddot q + \dot J_v(q,\dot q)\dot q
= a_{\mathrm{ref},B} + s_p
```

```math
|\dot q|\le1,\qquad |\ddot q|\le5
```

姿态任务使用较低优先级 slack。输出每个任务的最小可实现位置残差、最坏关节、饱和时间段和可改善的冗余姿态。

如果最小残差本身就是数毫米，则需明确告诉用户：在不改硬约束和基座 2 Hz/1 cm 工况时，零螺旋不可达；算法只能优化最小残差或通过任务优先级牺牲姿态。

### 优先级 B：利用 7-DOF 冗余，而不是统一增大参考

候选方向：

- task-priority / lexicographic MPC：world 位置第一优先，姿态第二优先，posture 第三优先
- acceleration manipulability / null-space posture scheduling
- 每个工作面独立的冗余姿态参考，而不是统一 `reset_nominal`
- 在 NMPC 中加入对未来 `J_v(q)` 条件数、所需 `qddot_ff` 峰值的软代价
- 对不可行参考做 constraint-aware reference governor，最小化法向误差，避免围绕直线来回穿越

这类方案不需要改 `±1/±5` 硬约束，但可能显著降低它们的活跃率。

### 优先级 C：完成姿态权重消融

低姿态权重初测对 `windylab`/sine 有改善。建议先完成：

```bash
python3 move_base_circle_mpc_ik_demo.py --ros-args \
  -p trajectory_shape:=whole \
  -p orientation_weight:=0.0234375 \
  -p terminal_orientation_weight:=0.3125 \
  -p record_loop_csv:=true
```

必须配套记录实际 world 姿态误差，再判断是否可接受。

### 优先级 D：若任务允许重复，再做 task/path-conditioned ILC

不要再用每个工作面一组常数谐波。应按：

```text
(task_id, path_progress_bin, base_phase)
```

持久化学习补偿，并用前一整轮数据更新下一轮。更新前先投影到 acceleration-level 可行域。该方法适合重复生产轨迹，不适合第一次 whole 运行。

## 9. 数据与分析注意事项

- `trajectory_visualizer_demo.py` 保存的 actual world CSV 是当前可信的实际位置 KPI 来源。
- `analyze_moving_base_run.py` 对 `whole` 的目标重建仍有问题，曾对已知良好数据算出约 117 mm 的假误差；不要用其 whole actual KPI 下结论，除非先修复其五工作面/抬笔时序重建。
- 自动调参必须同时硬门：
  - actual world RMS/P95/Max
  - 2 Hz 残差
  - timer/solve-to-publish P99
  - feedback timestamp fallback
  - guard/solver status
  - 实际 world 姿态误差
- 周期补偿实验中，solver `first_error` 是相对“被补偿后的参考”，可能变差，即使 nominal actual error 变好；所以不能用它替代 visualizer actual error。

## 10. 复现命令

终端 1：

```bash
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
ros2 launch manipulator student_arm.launch.py \
  arm_type:=sim use_rviz:=False velocity_feedforward_gain:=1.0
```

终端 2：

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_base_circle_mpc_ik_demo.py --ros-args \
  -p trajectory_shape:=whole \
  -p record_loop_csv:=true
```

终端 3：

```bash
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=moving_base_circle \
  -p trajectory_shape:=whole \
  -p frame_id:=world \
  -p record_control_signal:=true \
  -p ignore_initial_sec:=3.0 \
  -p show_matplotlib:=false
```

## 11. 当前进程状态

交接时已经停止所有短实验的 MPC 和 visualizer。原仿真 plant 仍在：

```text
/student_arm_node
/robot_state_publisher
```

即当前没有 MPC 节点继续发布命令。
