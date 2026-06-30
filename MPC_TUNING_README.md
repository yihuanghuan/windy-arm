# MPC 统一参数闭环调参说明（固定臂 + 浮动基座臂）

本文说明 `tune_unified_mpc_params.py` 的用途、运行方法、参数含义和结果解读。该脚本用于
离线搜出一套统一参数，同时覆盖固定基座 7 自由度臂和浮动基座 7 自由度臂的所有轨迹场景
（不含 LL100 空中操作臂，后者用独立的 `tune_ll100_nmpc.py`，见第 8 节）。

它不启动 ROS、不画图，而是直接调用 `MpcPinocchioIK.step()` 做闭环 roll-out：把 MPC
命令喂给一个复刻底层 `SmoothPositionController` 速度前馈控制器的 plant，用 plant 的真实
`q_actual` 作为下一拍 MPC 的求解起点，误差也在 `q_actual` 上统计。把多组参数在全部场景跑完
后排序，输出 CSV 和终端 Top 列表。

> 旧的单场景开环调参器 `tune_mpc_params.py` 已被本脚本取代并删除。

运行前请先按 `7DOF_ARM_NMPC_README.md` 完成 ACADOS 安装，并配置环境：

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

## 2. 评估的场景（可自行扩展）

| 场景 | demo | 锁姿态 | 说明 |
|---|---|---|---|
| `circle` | `move_arm_mpc_ik_demo.py` | 否 | 固定臂竖直圆 |
| `horizontal_circle` | `move_arm_horizontal_circle_mpc_ik_demo.py` | 是（锁起点姿态） | 固定臂水平圆 |
| `line` | `move_arm_mpc_ik_demo.py` | 否 | 固定臂直线往返 |
| `moving_base_circle` | `move_base_circle_mpc_ik_demo.py` | 是（base 系目标姿态） | 浮动基座臂 |

## 3. 命令行参数

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

两种模式：

| 模式 | 行为 | 输出 |
|---|---|---|
| `unified`（默认） | 搜出一套统一参数，同时满足全部场景；聚合分 = `mean + max(各场景分)`，强调最差场景 | 一份 CSV + 一套推荐参数（每个场景对应一条运行命令） |
| `separate` | 每个场景各自独立跑一遍粗到细，得到各自最优参数 | 一份 CSV（含 `scenario_set` 列区分）+ 末尾各场景最优参数汇总 |

两种模式共用同一套场景定义、闭环 plant、评分和硬约束，只是搜索的“目标集合”不同。CSV 文件名
带模式后缀：`unified_mpc_tuning_<mode>_<时间戳>.csv`。

常用组合示例：

```bash
python3 tune_unified_mpc_params.py --list-scenarios                      # 先看有哪些场景
python3 tune_unified_mpc_params.py                                       # unified: 一套参数满足所有场景
python3 tune_unified_mpc_params.py --mode separate                      # separate: 所有场景各自调
python3 tune_unified_mpc_params.py --mode separate --scenario circle    # 只单独调 circle 场景
python3 tune_unified_mpc_params.py --scenario circle,line --skip-fine   # 只粗调 circle 和 line
```

被搜索的参数本身见第 5 节。

## 4. 评分与硬约束

每个场景在真实 `q_actual` 上统计，按以下口径打分，并设硬约束：

- **硬判废（score=inf）**：任一场景出现 `fallback>0`（命令不可用回退）、`solver_failure>0`
  （求解状态失败）、`singular_count>0`（平移雅可比最小奇异值 < 0.05）或 `p95 求解时间 > 15ms`。
  这保证“无奇异位姿、无失败解算、解算快”。
- **聚合分** = `mean(各场景分) + max(各场景分)`，强调最差场景，确保一套参数在所有场景都好。
- 单场景分 = 位置 RMS/最大误差 + 姿态 RMS/最大误差 + 控制平滑 + 求解时间，越低越好。

> 奇异性用平移雅可比 (3×N) 最小奇异值衡量末端定位能力（健康值约 0.14~0.17），不用全
> 6×N：这些 demo 轨迹锁姿态时本就靠近腕部/姿态奇异（全 6D σ≈1e-3，理想 IK 跟随也无法避免），
> 但平移雅可比始终健康。位置奇异才是会导致关节速度发散、定位失效的“奇异位姿”。

CSV 字段：`aggregate_score`、`scenario`、`scenario_score`、`rms/max_position_error`、
`rms/max_orientation_error`、`rms_control_delta`、`min_sigma`、`singular_count`、
`fallback_count`、`solver_failure_count`、`mean/p95/max_solve_time_ms` 等。

## 5. 被调的参数（搜索维度）

调参器实际**搜索**的，只有 `ParamSet`（`tune_unified_mpc_params.py` 顶部）的 4 个字段；其余
权重/阈值都固定为 `MpcIkConfig` 默认值或脚本常量，不参与搜索。

| 被调参数 | 含义 | 粗搜索候选 | 约束 / 备注 |
|---|---|---|---|
| `horizon_steps` | MPC 预测步数 | `[4, 8, 12]` | 越大看得越远但越慢；闭环下短时域往往更稳 |
| `max_joint_step` | 每控制周期最大关节步长 (rad) | `[0.012, 0.016, 0.020]` | 必须 `≤ max_velocity × control_dt = 0.02`，越界自动过滤 |
| `orientation_weight` | 时域内姿态软约束权重 | `[0.5, 1.0, 2.0]` | 仅锁姿态场景（horizontal_circle / moving_base）生效 |
| `terminal_orientation_weight` | 终端姿态软约束权重 | `[5.0, 10.0, 20.0]` | 仅锁姿态场景生效 |

> **重要约束**：`max_joint_step <= max_velocity × control_dt = 1.0 × 0.02 = 0.02`。否则 MPC
> 命令超出控制器可执行速度，闭环必然滞后；脚本会自动过滤越界候选。

**两阶段搜索**：先按上表粗网格做笛卡尔积（`coarse_candidates`）；再取每个 set 评分 Top
`--fine-top` 的参数，在其邻域细化（`fine_candidates`）：

```text
horizon_steps               : 当前值 ± 2（最小 1）
max_joint_step              : 当前值 ± 0.004（不超过 0.02 上限）
orientation_weight          : 当前值 × [0.75, 1.0, 1.25]
terminal_orientation_weight : 当前值 × [0.75, 1.0, 1.25]
```

粗/细网格都在脚本顶部常量里（`COARSE_*` / `FINE_*`），直接改数值即可。

**不参与搜索、但可在脚本顶部常量调的量**（改了对所有候选统一生效）：闭环 plant 参数
（`PLANT_*`：`max_velocity` / `gain` / `tau` / 限速 / 加速度限）、奇异阈值
`SINGULAR_SIGMA_THRESHOLD=0.05`、求解时间硬上限 `MAX_SOLVE_TIME_MS_HARD=15`（按 p95 判）、
求解时间预热步数 `SOLVE_TIME_WARMUP_STEPS=5`。

> 想把这些固定量也变成"被搜索"的维度，或增删上面 4 个被调参数，见第 8 节的增删教程（§8.3）。

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

## 8. 扩展教程

### 8.1 增加一个运动场景（让调参器多评估一种轨迹）

"场景"就是让机械臂末端走的一条轨迹（圆、直线等）。新增一种,一共 **3 步**,都在
`tune_unified_mpc_params.py` 这一个文件里改。

**第 1 步:写这条轨迹的"位置函数"。**
加一个普通函数:输入时间 `t`(秒),输出末端此刻应该到的位置 `[x, y, z]`(米,基座坐标系)。
照着文件里已有的 `_vertical_circle` / `_line` 抄改最快。
*效果:* 这就定义了"末端在每个时刻该到哪",是新轨迹的本体。

```python
def _square(t):
    ...            # 算出这一时刻的目标位置
    return [x, y, z]   # 米,基座坐标系
```

**第 2 步:在场景清单里登记这条轨迹。**
在 `build_scenarios()` 的 `SCENARIO` 列表里加一行 `Scenario(...)`,按顺序填这几样:

| 要填的 | 怎么填 |
|---|---|
| 名字 | 起个英文名,如 `"square"`(也是结果/CSV 里这条场景的标识) |
| 对应 demo | 跑这条轨迹的 demo 文件名,只用于最后打印"推荐运行命令" |
| 周期(秒) | 轨迹走一圈/一趟的时间,决定这条场景仿真跑多久 |
| 是否锁姿态 | 末端要不要保持朝向:不锁填 `False`,锁填 `True` |
| 位置函数 | 第 1 步写的函数名 |
| 姿态函数 | 不锁就填 `None`(锁的话填一个返回朝向的函数) |
| 额外命令参数 | 推荐命令里要带的参数,如 `["trajectory_mode:=square"]` |

```python
# 在 build_scenarios() 的 SCENARIO 列表里追加一行:
Scenario("square", "move_arm_mpc_ik_demo.py", 8.0, False, _square, None,
         ["trajectory_mode:=square"]),
```
*效果:* 调参器从此就知道有这条场景了。

**第 3 步:直接跑,别的不用动。**
`unified`(一套参数管所有场景)和 `separate`(每条单独调)两种模式都会自动把新场景算进去。
*效果:* 新场景立刻参与评分和搜索,其它代码一行都不用改。

### 8.2 更换求解算法（把内置 MPC 换成你自己的求解器）

想用自己写的逆解/规划算法替换内置 MPC,一共 **3 步**(第 3 步可选)。

**第 1 步:把你的算法包成一个"求解器对象",只需实现一个 `step()` 方法。**
不用懂 MPC 内部,只按下面这个输入/输出约定写:
- **输入**:未来一小段时间的目标位置序列、当前关节角(闭环时是真实反馈回来的角度)、(可选)目标朝向。
- **输出**:三样东西 —— ①下一拍要发的关节角;②本次有没有算成功(`True`/`False`);③一个信息字典。
- **信息字典**里尽量带上几项给调参器用:本次求解耗时、是否成功、命令是否可用。

*效果:* 只要满足这个约定,调参器就能像调用内置 MPC 一样调用你的算法,不用关心它内部怎么算。

**第 2 步:把"造求解器"的入口 `build_solver()` 改成返回你的对象。**
脚本里所有需要求解器的地方都只从 `build_solver()` 这一个函数拿,所以只改这一处。

*效果:* 粗搜索、细搜索、两种模式全自动换成你的算法,其它代码不用动。

**第 3 步(可选):如果你的算法可调的旋钮变了,改一下被调参数清单。**
内置 MPC 调的是 4 个量(预测步数、关节步长、两个姿态权重)。如果你的算法要调的是别的(比如增益、迭代次数),
按 §8.3 的方法把"被调参数"换成你的。

*效果:* 调参器就改成搜索你算法的参数了。

> 两个省事提示:
> - 评估用的正运动学、奇异性判断都用一套**独立的**机器人模型算,跟你的算法内部无关 ——
>   所以换算法**不用**自己再提供正运动学。
> - 如果你只是想在内置 MPC 自带的三种后端(acados / casadi / scipy)之间切换,**不用**动
>   `build_solver`,改 `_build_config()` 里的 `solver_backend` 一个字段就行。

### 8.3 增加 / 减少"被调的参数"

"被调参数"就是调参器会去帮你搜最优值的那几个量(默认是预测步数、关节步长、两个姿态权重,见第 5 节)。
下面以改其中一个为例,所有改动都在 `tune_unified_mpc_params.py` 顶部那几个函数里。

#### A. 增加一个被调参数（示例:让"平滑权重 `joint_delta_weight`"也参与搜索),共 4 步

**第 1 步:把它登记成一个被调参数。**
在参数集合 `ParamSet` 里加上 `joint_delta_weight` 这一项。
*效果:* 它现在被算作"待搜索的旋钮"之一。

**第 2 步:让这个值真正传进求解器。**
在 `_build_config()` 里把它填进求解器配置(写 `joint_delta_weight=params.joint_delta_weight`)。
*效果:* 关键的一步 —— 不做的话,值变了但求解器收不到,等于白搜。

**第 3 步:给它几个候选值。**
在 `coarse_candidates()` 里列出想试的值(比如 `[0.05, 0.1, 0.2]`);想再精细就在 `fine_candidates()` 里给它一个"在最优值附近微调"的规则。
*效果:* 调参器会把这些值和别的参数排列组合,逐一评估。

**第 4 步:给它在缓存目录名里留个位置。**
在 `_build_dir()` 拼出的目录名里,把这个参数的值也带上。
*效果:* 让每个不同取值有自己的编译缓存目录,避免"不同取值用了同一份旧缓存"导致结果对不上。

#### B. 减少一个被调参数（示例:固定终端姿态权重、不再搜它),共 3 步

**第 1 步:从被调清单里删掉它。**
在 `ParamSet` 里去掉这一项。
*效果:* 它不再是一个搜索维度。

**第 2 步:给它定一个固定值。**
在 `_build_config()` 里写死(比如 `terminal_orientation_weight=5.0`)。
*效果:* 求解器照常拿到这个量,只是它从此不再变化。

**第 3 步:清掉它的候选和命名。**
从 `coarse_candidates()` / `fine_candidates()` 里删掉它,再把 `_build_dir()` 目录名里相关部分去掉。
*效果:* 参数组合变少,整轮搜索更快。

> **为什么增减参数会影响速度:** 每多一个被调参数、或每个参数多几个候选值,要试的组合数就成倍变多,
> 而每个新组合(第一次跑时)都要现编译一次求解器。所以**减参数 = 更快**,**加参数 = 更慢但搜得更全**。

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
