# MPC 统一参数闭环调参说明（固定臂 + 浮动基座臂）

本文说明 `tune_unified_mpc_params.py` 的用途、运行方法、参数含义和结果解读。该脚本用于
离线搜出一套统一参数，同时覆盖固定基座 7 自由度臂和浮动基座 7 自由度臂的所有轨迹场景
（不含 LL100 空中操作臂，后者用独立的 `tune_ll100_nmpc.py`，见第 8 节）。

它不启动 ROS、不画图，而是直接调用 `MpcPinocchioIK.step()` 做闭环 roll-out：把 MPC
命令喂给一个复刻底层 `SmoothPositionController` 速度前馈控制器的 plant，用 plant 的真实
`q_actual` 作为下一拍 MPC 的求解起点，误差也在 `q_actual` 上统计。把多组参数在全部场景跑完
后排序，输出 CSV 和终端 Top 列表。

> 旧的单场景开环调参器 `tune_mpc_params.py` 已被本脚本取代并删除。

运行前请先按 `NMPC_README.md` 完成 ACADOS 安装，并配置环境：

```bash
cd ~/code/windylab_ws/src/arm-platform
source scripts/setup_acados_env.sh
scripts/check_nmpc_env.sh
```

## 1. 快速运行

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh

# smoke test：少量组合验证闭环+多场景跑通
python3 tune_unified_mpc_params.py --skip-fine --max-combinations 3

# 完整粗到细搜索（首次每个参数组合都要编译 ACADOS 求解器，较慢）
python3 tune_unified_mpc_params.py                 # 默认 unified 模式
python3 tune_unified_mpc_params.py --mode separate # 每个场景单独调参
```

ACADOS 求解器按 `{horizon_steps, max_joint_step, orientation_weight,
terminal_orientation_weight}` 签名缓存到 `/tmp/windylab_acados_unified_tune/`，重复参数组合
会直接复用，不再重新编译。

### 两种模式（`--mode`）

| 模式 | 行为 | 输出 |
|---|---|---|
| `unified`（默认） | 搜出**一套统一参数**，同时满足全部场景；聚合分 = `mean + max(各场景分)`，强调最差场景 | 一份 CSV + 一套推荐参数（每个场景对应一条运行命令） |
| `separate` | **每个场景各自独立**跑一遍粗到细，得到各自最优参数 | 一份 CSV（含 `scenario_set` 列区分）+ 末尾各场景最优参数汇总 |

两种模式共用同一套场景定义、闭环 plant、评分和硬约束，只是搜索的“目标集合”不同。CSV 文件名
带模式后缀：`unified_mpc_tuning_<mode>_<时间戳>.csv`。

## 2. 评估的场景

一套参数会在以下 4 个场景上逐一闭环 roll-out（在脚本顶部 `build_scenarios` 可调整）：

| 场景 | demo | 锁姿态 | 说明 |
|---|---|---|---|
| `circle` | `move_arm_mpc_ik_demo.py` | 否 | 固定臂竖直圆 |
| `horizontal_circle` | `move_arm_horizontal_circle_mpc_ik_demo.py` | 是（锁起点姿态） | 固定臂水平圆 |
| `line` | `move_arm_mpc_ik_demo.py` | 否 | 固定臂直线往返 |
| `moving_base_circle` | `move_base_circle_mpc_ik_demo.py` | 是（base 系目标姿态） | 浮动基座臂 |

固定臂 circle/line 不锁姿态 → 姿态权重为 0；horizontal_circle / moving_base 锁姿态 → 使用
搜索到的 `orientation_weight` / `terminal_orientation_weight`。两类共享同一套 `horizon_steps`
和 `max_joint_step`。

## 3. 闭环 plant 模型

复刻 `smooth_position_controller.cc` 的**速度前馈分支**（角速度控制默认开启后的唯一模式），
逐关节、每个 50Hz MPC 周期跑 2 个 10ms 控制子步：

```text
v_ff  = clip(dq_cmd, ±max_velocity)
v_fb  = clip((q_cmd - q_actual)/position_feedback_tau, ±feedback_velocity_limit)
v_raw = clip(gain*v_ff + v_fb, ±max_velocity)
v_cmd = clip(v_raw, v_last ± max_acceleration*dt_ctrl)
q_actual = clip_to_limits(q_actual + v_cmd*dt_ctrl)
```

plant 参数取 `student_arm.launch.py` 默认：`gain=0.7, position_feedback_tau=0.15,
feedback_velocity_limit=0.4, max_acceleration=5.0, max_velocity=1.0`。

> **重要约束**：`max_joint_step <= max_velocity * control_dt = 1.0 * 0.02 = 0.02`。
> 否则 MPC 命令超出控制器可执行速度，闭环必然滞后。脚本会自动过滤越界的 `max_joint_step`。

## 4. 命令行参数

所有命令行参数（`python3 tune_unified_mpc_params.py -h` 可查）：

| 参数 | 类型/取值 | 默认值 | 说明 |
|---|---|---|---|
| `--mode` | `unified` / `separate` | `unified` | `unified` 搜一套参数满足所有(选中)场景；`separate` 每个场景各自单独调参 |
| `--scenario` | 逗号分隔场景名 | 全部 | 只调指定场景，如 `--scenario circle` 或 `--scenario circle,line`。配合 `--mode separate` 即“手动单独调某个场景” |
| `--list-scenarios` | flag | `False` | 打印所有可选场景名后退出（不调参） |
| `--periods` | float | `1.0` | 每个场景闭环 roll-out 的周期数（时长 = `periods × 场景 period_sec`） |
| `--top-k` | int | `10` | 终端每个 set 打印排名前 K 的参数组 |
| `--output-dir` | 路径 | `demo/data` | 调参 CSV 保存目录 |
| `--skip-fine` | flag | `False` | 只跑粗搜索，不做细搜索 |
| `--fine-top` | int | `5` | 取粗搜索 Top 几名生成细搜索邻域 |
| `--quiet-build` / `--no-quiet-build` | flag | `True`（静音） | 是否静音 ACADOS 代码生成/编译输出；排查构建问题用 `--no-quiet-build` |
| `--max-combinations` | int | `None`（不限） | 每阶段最多评估多少组，用于 smoke test |

常用组合示例：

```bash
python3 tune_unified_mpc_params.py --list-scenarios                      # 先看有哪些场景
python3 tune_unified_mpc_params.py                                       # unified: 一套参数满足所有场景
python3 tune_unified_mpc_params.py --mode separate                      # separate: 所有场景各自调
python3 tune_unified_mpc_params.py --mode separate --scenario circle    # 只单独调 circle 场景
python3 tune_unified_mpc_params.py --scenario circle,line --skip-fine   # 只粗调 circle 和 line
```

内部搜索维度（不在命令行，在脚本顶部常量里改）：

```text
COARSE_HORIZON_STEPS               = [4, 8, 12]
COARSE_MAX_JOINT_STEP              = [0.012, 0.016, 0.020]   # 受 0.02 上限约束
COARSE_ORIENTATION_WEIGHT          = [0.5, 1.0, 2.0]
COARSE_TERMINAL_ORIENTATION_WEIGHT = [5.0, 10.0, 20.0]
```

细搜索在粗搜索 Top 邻域内细化 horizon ±2、step ±0.004、姿态权重 ×[0.75, 1.0, 1.25]。
其它可在脚本顶部常量调整的量：闭环 plant 参数（`PLANT_*`，见第 3 节）、奇异阈值
`SINGULAR_SIGMA_THRESHOLD=0.05`、求解时间硬上限 `MAX_SOLVE_TIME_MS_HARD=15`（按 p95 判）、
求解时间预热步数 `SOLVE_TIME_WARMUP_STEPS=5`。

## 5. 评分与硬约束

每个场景在真实 `q_actual` 上统计，按以下口径打分，并设硬约束：

- **硬判废（score=inf）**：任一场景出现 `fallback>0`（命令不可用回退）、`solver_failure>0`
  （求解状态失败）、`singular_count>0`（平移雅可比最小奇异值 < 0.05）或 `p95 求解时间 > 15ms`。
  这保证“无奇异位姿、无失败解算、解算快”。
- **聚合分** = `mean(各场景分) + max(各场景分)`，强调最差场景，确保一套参数在所有场景都好。
- 单场景分 = 位置 RMS/最大误差 + 姿态 RMS/最大误差 + 控制平滑 + 求解时间，越低越好。

> 奇异性用**平移雅可比 (3×N) 最小奇异值**衡量末端定位能力（健康值约 0.14~0.17），不用全
> 6×N：这些 demo 轨迹锁姿态时本就靠近腕部/姿态奇异（全 6D σ≈1e-3，理想 IK 跟随也无法避免），
> 但平移雅可比始终健康。位置奇异才是会导致关节速度发散、定位失效的“奇异位姿”。

CSV 字段：`aggregate_score`、`scenario`、`scenario_score`、`rms/max_position_error`、
`rms/max_orientation_error`、`rms_control_delta`、`min_sigma`、`singular_count`、
`fallback_count`、`solver_failure_count`、`mean/p95/max_solve_time_ms` 等。

## 6. 当前已测推荐（统一参数）

2026-06-30 的一轮完整粗到细 ACADOS 闭环搜索（四场景，305/305 组均满足硬约束），冠军：

```text
horizon_steps               = 2
max_joint_step              = 0.020
orientation_weight          = 0.375
terminal_orientation_weight = 5.0
```

各场景离线评估（plant max_velocity=1.0，角速度控制开启）：

```text
scenario           rms_pos  max_pos  rms_ori  min_sigma  fallback solver_fail mean_ms
circle             0.5 mm   0.9 mm   -        0.155      0         0           0.33
horizontal_circle  1.0 mm   1.7 mm   0.024    0.140      0         0           0.16
line               0.9 mm   1.3 mm   -        0.169      0         0           0.16
moving_base_circle 1.2 mm   3.8 mm   0.008    0.155      0         0           0.35
```

这套参数已写入三个 demo 的参数默认值，直接运行即可：

```bash
# 固定臂竖直圆 / 直线
python3 move_arm_mpc_ik_demo.py --ros-args -p trajectory_mode:=circle   # 或 line

# 固定臂水平圆（锁姿态）
python3 move_arm_horizontal_circle_mpc_ik_demo.py --ros-args -p lock_orientation:=true

# 浮动基座臂
python3 move_base_circle_mpc_ik_demo.py
```

> 注：搜索最优是短时域 `horizon_steps=2`。若真机上希望更强的预测平滑裕度，`horizon_steps=4`
> 在本搜索中同样可行（聚合分略高，各项指标仍达标），可作为更保守的备选。

## 7. 调参经验

- 位置误差大：先确认 `max_joint_step` 是否顶满 `max_velocity*dt` 上限（闭环下越大跟踪越紧）。
- 闭环滞后明显：检查 `max_joint_step <= max_velocity*dt` 是否被违反。
- 姿态漂移：提高 `terminal_orientation_weight`；姿态太僵导致位置误差大：降低姿态权重。
- 控制抖动变大：降低 `max_joint_step` 或提高平滑代价项（脚本默认沿用 `MpcIkConfig` 平滑权重）。
- 求解时间偏高：降低 `horizon_steps`。
- 出现奇异判废：该轨迹几何在当前姿态约束下接近位置奇异，需调整轨迹或姿态目标，而非调权重。

## 8. 扩展：增加运动场景 / 更换求解算法

脚本预留了两个扩展点，顶部 docstring 也有对应注释（“扩展点①/②”）。

### 8.1 增加一个运动场景

在 `tune_unified_mpc_params.py` 的 `build_scenarios()` 里的 `SCENARIO` 列表加一条
`Scenario(...)`，并把轨迹写成模块级普通函数（参考 `_vertical_circle` /
`_moving_base_position`）。`Scenario` 字段：

| 字段 | 含义 |
|---|---|
| `name` | 场景名（也是 `separate` 模式的 CSV/输出标识） |
| `demo` | 该场景对应的 demo 文件名（仅用于打印可直接运行的推荐命令） |
| `period_sec` | 轨迹一圈/一程周期(秒)，roll-out 时长 = `periods * period_sec` |
| `lock_orientation` | 是否锁末端姿态（锁则用 `orientation_weight`/`terminal_orientation_weight`） |
| `position_fn(t)` | 返回 `(3,)` 目标末端位置（base 系） |
| `rotation_fn(t)` | 返回 `(3,3)` 目标姿态；不锁姿态填 `None`；想“锁起点姿态”填 `None` 且 `lock_orientation=True`（自动锁起点 IK 解的姿态） |
| `demo_extra_args` | 推荐命令里额外的 `-p` 参数，如 `["trajectory_mode:=circle"]` |

加完即可，`unified` 和 `separate` 模式都会自动带上新场景，无需改其它任何地方。例如加一个
固定臂方形轨迹：

```python
def _square(t: float) -> np.ndarray:
    ...  # 返回 base 系目标位置 (3,)

# build_scenarios() 的 SCENARIO 列表里追加：
Scenario("square", "move_arm_mpc_ik_demo.py", 8.0, False, _square, None,
         ["trajectory_mode:=square"]),
```

### 8.2 更换求解算法（把 MPC 换成别的 IK/规划器）

只改 `build_solver(params, lock_orientation)` 这一个工厂函数，让它返回你的求解器对象。该对象
只需实现 `step()` 合约：

```text
step(targets(H,3), q_current(N,), warm_start=None, target_rotation=None) -> (q_next(N,), ok, info)
  targets         : (horizon, 3) 目标位置序列(base 系)
  q_current       : (N,) 当前关节角（闭环下为实测反馈）
  target_rotation : None / (3,3) / (horizon,3,3)
  返回 q_next(N,) 下一拍关节命令; ok(bool); info(dict)
  info 尽量含: "trajectory"(下一拍 warm start)、"solve_time"(秒,可选)、
              "status"(int,0=正常)、"success"(bool)、"command_usable"(bool)、"tracking_ok"(bool)
```

评估侧用的正运动学/雅可比/奇异性都走独立的 pinocchio 模型（`Context`），与求解器内部无关，
所以换算法**不必再提供 FK**。若新算法的可调参数不同（不再是 horizon/step/姿态权重），同时改
`ParamSet` 与 `coarse_candidates` / `fine_candidates` 即可，其余评分/模式/输出逻辑不用动。

> 若只是想换 ACADOS/CasADi/SciPy 求解后端（同一套 MPC），改 `_build_config` 里的
> `solver_backend` 即可，不必动 `build_solver` 结构。

## 9. LL100 避障调参(`tune_ll100_nmpc.py`)

LL100 整机避障用**独立脚本 `tune_ll100_nmpc.py`**(与独立臂的 `tune_unified_mpc_params.py` 分开,互不影响)。它复用 `move_ll100_lh420_nmpc_demo.py` 的同一套 `MpcIkConfig` + `ll100_traj_pub` 的路径模式,离线 roll 出**闭环**轨迹,用避障 `violation` 做安全硬门。

### 9.1 闭环模型(复刻 SmoothPositionController)

被控对象复刻 LL100 底层的 `SmoothPositionController`。LL100 **固定开启速度前馈(velocity feedforward),无开关**:速度前馈 `v_ff` + 位置反馈 `v_fb`(`τ=0.15`、限速 `0.4`)+ 加速度限(`5.0`),再积分。`ll100_lh420_control.launch.py` 已设 `{'enable_velocity_feedforward': True}`,与调参器 plant 一致(LL100 与独立臂相互独立)。

MPC 每拍从 `q_plant` 反馈重解;`violation` / `margin` 在被控对象**实际经过的状态**上量(不是 MPC 预测 plan)。`--max-velocity` 应与 `student_arm` 的 `max_velocity` 一致(默认 `0.5`)。

### 9.2 运行

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
# 需先 source ROS + 工作空间 + ACADOS(导入 demo 模块和求解器);不起 ROS、不起仿真(被控对象在脚本里建模)
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 tune_ll100_nmpc.py --path-mode avoidance_sweep
# 快速冒烟:--max-combinations 2 --periods 0.5
```

### 9.3 搜索的参数 + 安全门

| 参数 | 粗搜索候选 | 说明 |
|---|---|---|
| `capsule_samples_per_segment` | `[1, 2, 3]` | 连杆胶囊采样密度,越密避障越保守 |
| `max_joint_step` | `[0.008, 0.01, 0.015, 0.02]` | MPC 每周期关节步长;> `max_velocity*dt` 会滞后 |
| `path_preview_stride` | `[4, 6]` | 预测时域内目标点间隔 |
| `body_obstacle_margin` | `[0.015, 0.02]` | 机身 box 的 NMPC 硬约束余量 |

打分先按**安全分层**:`violation_max ≤ 0.1mm 且 fallback=0` 才 `safe=True`,不安全的直接排到所有安全候选之后;安全候选之间再按 `violation + 跟踪误差` 排序。

### 9.4 输出

- CSV:`demo/data/mpc_tuning_ll100_<path_mode>_<时间戳>.csv`
- 终端打印 Top-K(先看 `safe`,再看 `rms_pos`)+ 一条可直接跑的 `move_ll100_lh420_nmpc_demo.py` 命令

`--path-mode` 支持 `ll100_traj_pub` 的模式(`avoidance_sweep` / `lower_obstacle_circle` / `circle` / `windylab` / `front_wing_arc` / `front_propeller_line`);`single_target` 不支持(没有连续路径)。
