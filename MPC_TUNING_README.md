# 自动调参算法

本仓库有两套相互独立的离线闭环自动调参器,分别针对两类机械臂:

- **一、7 自由度机械臂自动调参** —— 固定基座 + 浮动基座臂,搜一套统一参数,脚本 `tune_unified_mpc_params.py`。
- **二、LL100-LH420 自动调参** —— 空中操作臂避障,脚本 `tune_ll100_nmpc.py`。

两者完全独立、互不影响(求解器、场景、参数都分开)。下面分别说明。共同前置:先按总 [README.md](README.md) 的**《六、部署步骤》**(区分一般电脑 / Jetson)装好环境与 ACADOS,再激活:

```bash
cd ~/code/windylab_ws/src/arm-platform
source scripts/setup_acados_env.sh
scripts/check_nmpc_env.sh
```

## 一、7自由度机械臂自动调参

> 旧的单场景开环调参器 `tune_mpc_params.py` 已被 `tune_unified_mpc_params.py` 取代并删除。本节说明 `tune_unified_mpc_params.py`,统一覆盖固定基座与浮动基座 7 自由度臂的所有轨迹场景。

### 1. 快速运行

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh

# smoke test：少量组合验证闭环+多场景跑通
python3 tune_unified_mpc_params.py --skip-fine --max-combinations 3

# 完整粗到细搜索（首次遇到新的 (horizon_steps, max_joint_step) 才编译 ACADOS 求解器，较慢）
python3 tune_unified_mpc_params.py                 # 默认 unified 模式
python3 tune_unified_mpc_params.py --mode separate # 每个场景单独调参
```

ACADOS 求解器只按 `{horizon_steps, max_joint_step}` 签名缓存到 `/tmp/windylab_acados_unified_tune/`。
姿态权重（`orientation_weight` / `terminal_orientation_weight`）不进签名、在运行时设置，所以**只改权重
不会重新编译**；只有换 `horizon_steps` 或 `max_joint_step` 才会触发一次新编译，其余组合直接复用。

### 2. 评估的场景（可自行扩展）

| 场景 | demo | 锁姿态 | 说明 |
|---|---|---|---|
| `circle` | `move_arm_mpc_ik_demo.py` | 否 | 固定臂竖直圆 |
| `horizontal_circle` | `move_arm_horizontal_circle_mpc_ik_demo.py` | 是（锁起点姿态） | 固定臂水平圆 |
| `line` | `move_arm_mpc_ik_demo.py` | 否 | 固定臂直线往返 |
| `moving_base_circle` | `move_base_circle_mpc_ik_demo.py` | 是（base 系目标姿态） | 浮动基座臂 |

### 3. 命令行参数

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

### 4. 评分与硬约束

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

### 5. 被调的参数（搜索维度）

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

### 6. 当前已测推荐（统一参数）

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

### 7. 调参经验

- 位置误差大：先确认 `max_joint_step` 是否顶满 `max_velocity*dt` 上限（闭环下越大跟踪越紧）。
- 闭环滞后明显：检查 `max_joint_step <= max_velocity*dt` 是否被违反。
- 姿态漂移：提高 `terminal_orientation_weight`；姿态太僵导致位置误差大：降低姿态权重。
- 控制抖动变大：降低 `max_joint_step` 或提高平滑代价项（脚本默认沿用 `MpcIkConfig` 平滑权重）。
- 求解时间偏高：降低 `horizon_steps`。
- 出现奇异判废：该轨迹几何在当前姿态约束下接近位置奇异，需调整轨迹或姿态目标，而非调权重。

### 8. 扩展教程

#### 8.1 增加一个运动场景（让调参器多评估一种轨迹）

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

#### 8.2 更换求解算法（把内置 MPC 换成你自己的求解器）

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
> - 如果你只是想在内置 MPC 自带的后端(acados / casadi / scipy;casadi/scipy 已弃置为回退,另有 GRAMPC,
>   详见 [LL100_LH420_NMPC_Solver_README.md](LL100_LH420_NMPC_Solver_README.md))之间切换,**不用**动
>   `build_solver`,改 `_build_config()` 里的 `solver_backend` 一个字段就行。

#### 8.3 增加 / 减少"被调的参数"

"被调参数"就是调参器会去帮你搜最优值的那几个量(默认是预测步数、关节步长、两个姿态权重,见第 5 节)。
下面以改其中一个为例,所有改动都在 `tune_unified_mpc_params.py` 顶部那几个函数里。

##### A. 增加一个被调参数（示例:让"平滑权重 `joint_delta_weight`"也参与搜索),共 4 步

**第 1 步:把它登记成一个被调参数。**
在参数集合 `ParamSet` 里加上 `joint_delta_weight` 这一项。
*效果:* 它现在被算作"待搜索的旋钮"之一。

**第 2 步:让这个值真正传进求解器。**
在 `_build_config()` 里把它填进求解器配置(写 `joint_delta_weight=params.joint_delta_weight`)。
*效果:* 关键的一步 —— 不做的话,值变了但求解器收不到,等于白搜。

**第 3 步:给它几个候选值。**
在 `coarse_candidates()` 里列出想试的值(比如 `[0.05, 0.1, 0.2]`);想再精细就在 `fine_candidates()` 里给它一个"在最优值附近微调"的规则。
*效果:* 调参器会把这些值和别的参数排列组合,逐一评估。

**第 4 步(只有"改变求解器结构"的参数才需要):给它在缓存目录名里留个位置。**
如果这个参数会改变求解器的编译产物(比如像 `horizon_steps` / `max_joint_step` 那样改变问题规模),
就在 `_build_dir()` 的目录名里把它的值也带上,让每个取值有自己的编译缓存目录,避免"不同取值用了同一份旧缓存"。
*效果:* 结果不会串味。
反之,如果它只是**代价权重**(像姿态权重那样,运行时才写进求解器、不改结构),就**不用**动 `_build_dir` ——
这类参数不进编译缓存签名,不同取值本来就复用同一份编译产物,改了不会重新编译。

##### B. 减少一个被调参数（示例:固定终端姿态权重、不再搜它),共 3 步

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
> 逐一评估自然更慢。至于**编译**开销要分两种参数看:改变求解器结构的参数(`horizon_steps` /
> `max_joint_step`)每遇到一个新取值才现编译一次;而**代价权重**(如姿态权重)运行时才设置、不进编译缓存,
> 改它**不会**触发重新编译。所以:减参数 = 更快;加**结构类**参数 = 更慢(评估+编译都涨),加**权重类**参数
> = 只多了评估、不多编译。

## 二、LL100_LH420自动调参

LL100-LH420 空中操作臂的避障调参用独立脚本 `tune_ll100_nmpc.py`(与一里的 `tune_unified_mpc_params.py` 完全分开)。它复用 `move_ll100_lh420_nmpc_demo.py` 的同一套 `MpcIkConfig` + `ll100_traj_pub` 的路径模式,离线 roll 出**闭环**轨迹:被控对象复刻 LL100 底层 `student_arm_node` 的 `SmoothPositionController`,MPC 每拍从被控对象真实状态反馈重解,避障 `violation` 在**实际经过的状态**上量、作安全硬门。不起 ROS、不起仿真(被控对象在脚本里建模)。

### 1. 快速运行

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
# 需先 source ROS + 工作空间 + ACADOS(导入 demo 模块和求解器)
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 tune_ll100_nmpc.py --path-mode avoidance_sweep
# 快速冒烟(只评 2 组、跑半圈):
python3 tune_ll100_nmpc.py --path-mode avoidance_sweep --max-combinations 2 --periods 0.5
```

### 2. 评估的场景(path_mode)

| 模式 | 说明 |
|---|---|
| `lower_obstacle_circle` | 默认,末端朝下在避障区下方画圆 |
| `avoidance_sweep` | 机身/安装架前方扫掠 |
| `front_wing_arc` | 右桨盘下方扫掠弧避障演示 |
| `circle` | `base_link` y-z 平面画圆 |

`--path-mode` 选一个评估(`single_target` 等无连续路径的模式不参与调参)。

### 3. 命令行参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--path-mode` | `avoidance_sweep` | 评估的路径模式 |
| `--max-velocity` | `0.5` | 闭环 plant 底层关节限速,应与 `student_arm` 一致;**`max_joint_step` 由它派生 = `max_velocity·control_dt`,不搜索** |
| `--period` | `20.0` | 一圈时长 s(完成时间;越长越精准) |
| `--periods` | `1.0` | 计时跟踪圈数 |
| `--coarse-passes` | `2` | coarse 坐标下降扫描遍数(每遍逐维扫一次) |
| `--skip-fine` | 关 | 只跑 coarse,不做局部细化 |
| `--max-combinations` | 全部 | 限制最多评估多少组(冒烟用) |
| `--top-k` / `--backend` / `--output-dir` / `--quiet-build` | 同一 | 打印数 / 后端 / CSV 目录 / 静音编译 |

### 4. 评分与硬约束(优先级 安全 > 求解时间 > 精度 > 完成时间)

**硬门**(不满足直接排到所有 safe 候选之后):

1. **安全无碰撞** —— `violation_max ≤ 0.1mm`(在实际经过的状态上量)
2. **可行** —— 无 fallback(求解器不可行)

**软项**(过硬门后按此加权排序):求解时间 `mean_ms`(轻)→ 末端误差 `rms/max`(权重最大)→ 启动直达首点 `reached_first`(先 hold 到首点 `5mm` 内再计时)→ 远离奇异 `manip`(可操作度,越大越好)。

### 5. 被调的参数(9 维 NMPC,坐标下降 coarse→fine)

**只调 NMPC 参数**(时域 + 代价权重);硬件/避障参数一律固定不搜。9 维全网格积(3⁹≈2 万)组合过多,故用**坐标下降**:从基线逐维扫描候选、采纳更优(coarse),再在最优附近逐维局部细化(fine)。维度顺序把结构量 `horizon_steps` 放最前、先定结构,后面 8 个权重维在该 horizon 下复用同一求解器。

| 参数 | coarse 候选 | 含义 |
|---|---|---|
| `horizon_steps` | `[6, 8, 12]` | 预测时域步数(时域 = steps·0.02s) |
| `position_weight` | `[80, 120, 180]` | 位置跟踪 Q |
| `terminal_position_weight` | `[800, 1200, 2000]` | 终端位置 Q_N |
| `orientation_weight` | `[0.15, 0.3, 0.6]` | 姿态跟踪 |
| `terminal_orientation_weight` | `[1.5, 3, 6]` | 终端姿态 |
| `joint_delta_weight` | `[0.04, 0.08, 0.16]` | 一阶平滑 R(惩罚 Δq) |
| `joint_delta_change_weight` | `[0.12, 0.25, 0.5]` | 二阶平滑 R_Δ(抑抖) |
| `current_posture_weight` | `[0.002, 0.005, 0.01]` | 贴近当前姿态 |
| `center_posture_weight` | `[0.005, 0.01, 0.03]` | 贴关节中位,抗奇异 |

**固定·不搜**:`max_joint_step = max_velocity·control_dt`(硬件速度约束的 OCP 镜像,派生量);避障 `obstacle_safety_margin=0.005`(桨叶)/`body_obstacle_margin=0.005`(机身)/`capsule_joint_radii`·`capsule_link_radii`(分段避障半径:关节粗/连杆细,几何固定)/`capsule_samples_per_segment`(安全);`path_preview_stride=4`(追踪层)。

### 6. 当前默认与各模式结果

| 模式 | 20s/圈 max 误差(9 维调后) | 最优要点(相对基线) |
|---|---|---|
| `avoidance_sweep` | 3.3 mm(rms 1.7) | `horizon_steps=4`、`position_weight=252`、`terminal_position_weight=1680`|
| `lower_obstacle_circle` | 1.9 mm(rms 0.5) | `h=6`、`pw=180`、`tpw=800`|
| `circle` | 2.3 mm(rms 0.9) | `h=4`、`pw=216`、`tpw=1200`|

> 每模式的完整最优 `-p …` 命令见调参器终端输出末行,或 `demo/data/tune_ll100_<mode>_<时间戳>.csv` 的 `recommended_command` 列。demo 默认是一套通用参数;要拿到某模式的最优,用该模式的推荐命令覆盖。

### 7. 调参经验

- **max 误差卡住、放慢也不降?** 多半是**启动残差**(开始跟踪时离首点还差一点)—— 收紧 `target_switch_tolerance` 比放慢周期更有效。
- **某段误差特别大 + manip 很低?** 路径经过**奇异区**,调参降不动,要**改路径几何**绕开(如 `avoidance_sweep` 把前中心外推到高 manip 区)。
- **要更小误差?** 在不破坏安全的前提下**放慢 `period_sec`**,误差随周期单调下降到各自地板。
- **跟不上 / 想更快?** `max_joint_step` 已锁死 = `max_velocity·control_dt`,不再单独调;要更快只能(真机允许时)提 `max_velocity`,它会自动放大 `max_joint_step`。
- **输出**:CSV 在 `demo/data/tune_ll100_<path_mode>_<时间戳>.csv`;终端 Top-K(先看 `safe`,再看 `max/rms`)+ 一条可直接跑的 `move_ll100_lh420_nmpc_demo.py` 命令。

### 8. 扩展教程

#### 8.1 增加一个路径模式（让调参器多评估一种轨迹）

LL100 的"场景"就是一条 `path_mode` 轨迹。轨迹本体定义在 `ll100_traj_pub.py`,调参器和 demo 都从这里取。新增一种,一共 **2 步**。

**第 1 步:登记模式名。**
在 `ll100_traj_pub.py` 顶部的 `SUPPORTED_PATH_MODES` 元组里加上你的名字,如 `"my_sweep"`。
*效果:* 调参器/发布节点从此认得这个模式,`--path-mode my_sweep` 不再报错。

**第 2 步:写这条轨迹的点序列。**
在同文件的 `_build_path(mode, count)` 里加一个分支:返回一个 `(count, 3)` 的数组,每行是末端在 `base_link` 系里该到的 `[x, y, z]`(米)。照着已有的 `avoidance_sweep`(折线重采样)或 `lower_obstacle_circle`(参数化圆)抄改最快。

```python
if mode == "my_sweep":
    points = [[0.50, -0.18, 0.15], ...]   # base_link 系,米
    return _resample_polyline(points, count)
```
*效果:* 调参器 `--path-mode my_sweep` 立刻能离线闭环评估它。若还想在 demo 里可视化/运行,需在 `move_ll100_lh420_nmpc_demo.py` 的 `_build_path` 里同样加这个分支。

> 提示:求解器与路径无关(路径只是在线目标参数),所以加路径**不触发**求解器重编译,新模式复用已编译的求解器。

#### 8.2 切换 / 更换求解后端

> 求解器的完整选型/对比/安装/筛选规则,以 **[LL100_LH420_NMPC_Solver_README.md](LL100_LH420_NMPC_Solver_README.md)** 为准。
> 结论:合格解为 acados-HPIPM(默认)/ acados-qpOASES / GRAMPC;CasADi-IPOPT、SciPy-SLSQP 已弃置(慢,保留作离线参照)。

**本调参器**的 `--backend` 仍支持 acados / casadi / scipy(后两者为弃置回退,慢):

```bash
python3 tune_ll100_nmpc.py --path-mode avoidance_sweep --backend casadi   # 或 scipy(均为弃置回退)
```
*效果:* 调参器与 demo 都会换用该后端求解,别的不用动。
> 注:qpOASES 用 `-p acados_qp_solver:=FULL_CONDENSING_QPOASES`、GRAMPC 用 `-p solver_backend:=grampc`
> 在 demo 里切换(见 Solver README §7);本调参器暂只覆盖 acados/casadi/scipy 三种 `--backend`。

想把内置 MPC 换成**你自己写的算法**:LL100 的求解器统一由共享的 `MpcPinocchioIK` 构造(与独立臂同一套),做法与「一、§8.2」一致 —— 把你的算法包成一个实现 `step()` 的对象(输入:未来目标序列 + 当前关节角 + 可选目标朝向;输出:下一拍关节角、成功标志、信息字典)。这属于共享求解器改动,改前建议先与独立臂那边协调。

#### 8.3 增加 / 减少被调的 NMPC 参数

被调参数就是坐标下降会去搜的那几个量(默认 9 个 NMPC 时域/权重,见第 5 节)。所有改动都在 `tune_ll100_nmpc.py` 顶部。

##### A. 增加一个被调参数（示例:让某个新权重也参与搜索),共 4 步

**第 1 步:登记成被调项。** 在 `ParamSet` 里加上这个字段。
*效果:* 它成为坐标下降的一个搜索维度。

**第 2 步:让它真正传进求解器。** 在 `build_config()` 里把它填进 `MpcIkConfig`(写 `xxx=p.xxx`)。
*效果:* 关键一步 —— 不做的话值变了但求解器收不到,等于白搜。

**第 3 步:给它候选值。** 在 `COARSE` 字典里加一行 `"xxx": [...]`。fine 阶段会自动在最优值附近按 `FINE_SCALE`(权重)或 `FINE_HORIZON_DELTA`(步数)细化,无需单独写。
*效果:* 坐标下降会逐维扫描这些候选。

**第 4 步(只有结构量才做):判断要不要进缓存目录名。** 如果新参数是**代价权重**(由求解器运行时 `cost_set` 写入、不影响生成代码),**不要**加进 `_dir_tag()` —— 加了反而把本可复用的权重组合拆成各自重编、白白变慢。只有**结构量**(改了必须重编,如时域步数、步长边界)才加进 `_dir_tag()`。
*效果:* 权重维复用同一编译求解器;只有结构量各自独立缓存。

> 若还想让 demo 能跑出这个搜索结果,别忘了在 `move_ll100_lh420_nmpc_demo.py` 里 `declare_parameter` 并接进 `MpcIkConfig`(参考本轮新加的 `joint_delta_weight`)。

##### B. 减少一个被调参数（示例:固定某权重、不再搜),共 3 步

**第 1 步:从被调清单删掉。** 在 `ParamSet` 里去掉这一项。
*效果:* 它不再是搜索维度。

**第 2 步:给它写死固定值。** 在 `build_config()` 里直接赋常量(如 `terminal_orientation_weight=3.0`)。
*效果:* 求解器照常拿到它,只是从此不变。

**第 3 步:清候选(必要时清命名)。** 从 `COARSE` 字典删掉那一行;若它当初是结构量、进过 `_dir_tag()`,一并去掉(权重维本就不在 `_dir_tag` 里,无需动)。
*效果:* 搜索维度变少,整轮更快。

> **为什么增减参数会影响速度:** 坐标下降的评估次数 ≈ `遍数 × Σ(每维候选数)`,加维/加候选 = 更多次评估。但**这 9 维里 8 个是代价权重,已运行时 `cost_set`、扫它们只复用不重编**,所以加/减权重维影响的是**评估次数**(每次评估的 rollout 时间),不是编译时间。真正贵的是**结构量** `horizon_steps` —— 每多一个取值就多一次 ~15s 首编。
