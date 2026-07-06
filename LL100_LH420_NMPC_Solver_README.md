# LL100-LH420 NMPC 求解器对比说明(Solver README)

本文是 LL100-LH420 空中操作臂 NMPC 求解器选型/对比的单一事实源,覆盖:各求解器安装部署、
对比程序用法、启动参数、筛选规则与最终结论、分阶段验证计划、增删求解器教程,以及如何在 LL100
实机/仿真 demo 里切换求解器。

相关文件:
- 对比程序:[benchmark_nmpc_solvers.py](src/arm-platform/demo/benchmark_nmpc_solvers.py)
- NMPC 核心:[mpc_pinocchio_ik.py](src/arm-platform/demo/mpc_pinocchio_ik.py)
- GRAMPC 运行器/构建/代码生成:[grampc_ll100.py](src/arm-platform/demo/grampc_ll100.py) /
  [grampc_build.py](src/arm-platform/demo/grampc_build.py) / [grampc_codegen.py](src/arm-platform/demo/grampc_codegen.py)
- 实机/仿真 demo:[move_ll100_lh420_nmpc_demo.py](src/arm-platform/demo/move_ll100_lh420_nmpc_demo.py)

---

## 1. 各求解器的安装与部署

> **共用环境**(系统依赖、编译工作空间、安装 acados 主力求解器)与 **Jetson(ARM64)部署差异**已统一到总 [README.md](README.md) 的**《六、部署步骤》**(区分一般电脑 / Jetson)。请先在那里完成基础部署;本节只讲**各求解器自身**的安装细节与选型状态。GRAMPC 的简要安装 README 也有,完整说明仍以本节 1.4 为准。

所有求解器都在 `src/arm-platform/demo/` 下运行,先激活环境(见 1.7)。

### 1.1 acados(主力,只部署 HPIPM / qpOASES 两个 QP 后端)
acados 是一个 NMPC/OCP 求解框架:它做 SQP/RTI 迭代,每次迭代把问题线性化成一个 QP,交给一个 **QP 后端**求解。
所以 "HPIPM / qpOASES" 不是两个独立求解器,而是 acados 内部可换的 QP 后端。

- 安装位置:`~/code/acados`(源码 + 编译产物 `lib/`)。
- QP 后端由编译开关决定。查当前开关:
  ```bash
  grep -iE "ACADOS_WITH_(QPOASES|OSQP|DAQP):" ~/code/acados/build/CMakeCache.txt
  ```
- 本仓库部署目标:**HPIPM + qpOASES**。安装脚本会显式设置 `ACADOS_WITH_QPOASES=ON`,
  `ACADOS_WITH_OSQP=OFF`, `ACADOS_WITH_DAQP=OFF`。
- 安装/重编:
  ```bash
  cd ~/code/windylab_ws/src/arm-platform
  scripts/install_acados.sh
  source scripts/setup_acados_env.sh
  ```

### 1.2 GRAMPC(增广拉格朗日 + 梯度法,C++ 编译问题描述)
GRAMPC 通过 Python 绑定 **pygrampc** 使用,问题描述用 CasADi codegen 出 C 导数、编译成 C++ 模块(测速真实)。

前置:**Eigen 3.4**(`pkg-config --modversion eigen3` 应 ≥ 3.4)、C++ 编译器。安装:
```bash
cd ~/code/windylab_ws/src/arm-platform
scripts/install_grampc.sh
source scripts/setup_acados_env.sh
python3 -c "import pygrampc; print('pygrampc ok')"
```
- 首次运行对比程序时,`grampc_build.ensure_module()` 会自动用 CasADi codegen + scikit-build 编译
  LL100 的 GRAMPC 问题模块到 `src/arm-platform/demo/grampc_problem/`(生成 `.c/.h/.so`),之后复用。
  现已加过期检测:约束源(`mpc_pinocchio_ik.py`/`grampc_codegen.py`)改动后会自动重建,避免跑到旧约束。
- ⚠️ 部署注意:`grampc_build` 需要 pygrampc 源码树来取 GRAMPC/ProblemDescription 头文件,默认路径来自
  环境变量 `PYGRAMPC_ROOT`。请把它指向持久的 pygrampc 目录:
  ```bash
  export PYGRAMPC_ROOT=~/code/pygrampc
  ```
  (放进 `setup_acados_env.sh` 或你的 shell profile,避免每次手动设。)

### 1.3 各求解器速查

| 求解器 | 载体 | 安装 | 状态 |
|---|---|---|---|
| acados + qpOASES | acados(`ACADOS_WITH_QPOASES=ON`) | 编译 acados | **合格(默认)** |
| acados + HPIPM | acados 内置 | 编译 acados | **合格** |
| GRAMPC | pygrampc + C++ codegen | 见 1.2 | **合格** |

### 1.4 激活环境(每次跑之前)
```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
```

---

## 2. 对比程序快速启动

```bash
# 默认:front_wing_arc 场景,跑所有 active 求解器(periods=0.3 快速冒烟)
python3 benchmark_nmpc_solvers.py

# 正式对比:完整轨迹周期 + 稳态精度口径 + 每个求解器重复 3 次
python3 benchmark_nmpc_solvers.py --periods 1.0 --repeats 3

# 只跑三个合格解
python3 benchmark_nmpc_solvers.py --only acados_sqp_rti_hpipm,acados_sqp_rti_qpoases,grampc --periods 1.0
```
输出:终端汇总表 + `demo/data/benchmark_nmpc_<mode>_<时间戳>.csv`(+ 可选柱状图 `.png`)。

---

## 3. 启动命令参数详解

| 参数 | 默认 | 含义 |
|---|---|---|
| `--path-mode` | `front_wing_arc` | 测试套件场景:`front_wing_arc` / `lower_obstacle_circle` / `circle`|
| `--periods` | `0.3` | 跑几个轨迹周期。|
| `--repeats` | `1` | 每个求解器重复跑几次。|
| `--warmup-frac` | `0.5` | 稳态口径:跳过前 `frac×步数` 步排除启动暂态,让 RMS 反映真实稳态精度(需 `periods≥~0.7`) |
| `--tier` | `all` | 只跑某一档:`acados` / `grampc` / `all` |
| `--only` | 无 | 逗号分隔的求解器 key,只跑指定的 |
| `--max-velocity` | `0.5` | 闭环 plant 的底层关节限速 rad/s(与 launch 一致) |
| `--plot` / `--no-plot` | 开 | 是否出 P95 速度 + RMS 精度柱状图(需 matplotlib) |
| `--quiet-build` / `--no-quiet-build` | 开 | 是否静音 acados/GRAMPC 的代码生成/编译输出 |
| `--output-dir` | `demo/data` | CSV/图输出目录 |

求解器 key:`acados_sqp_rti_hpipm` / `acados_sqp_rti_qpoases` / `grampc`。

---

## 4. 求解器筛选规则

核心原则:安全第一,再谈精度和速度。

### 4.1 先看懂 5 个指标

| 指标 | 含义 | 怎么看 |
|---|---|---|
| `mean_ms` / `P95` | 每步求解耗时的均值 / 第95百分位(ms) | 控制回路 50Hz = 每 **20ms** 必须出命令。看 **P95<20ms** 判能否实时 |
| `realtime_rate`(rt%) | 求解时间 < 20ms 的步数占比 | 越接近 100% 越好 |
| `稳态 RMS` | 排除启动暂态后,末端实际位置 vs 目标的均方根误差 | **越小越准**。必须是稳态口径,否则被启动暂态淹没(见 4.4) |
| `σ_min`(sigmin) | 末端雅可比最小奇异值(稳态窗口内) | 越大越远离奇异;**< 1e-3 视为近奇异**(危险) |
| `violation_max`(viol) | 避障硬约束最大侵入量(全程) | **必须恒为 0**,>0 = 撞了障碍 |

### 4.2 硬门槛(任一不过 → 淘汰)

1. **避障(碰撞)**:`violation_max == 0` —— 全程(含从 home 姿态接近轨迹的阶段)任何时刻都不能撞障碍。
2. **奇异**:**稳态** `σ_min ≥ 1e-3` —— 求解器不能把机械臂开进/卡在近奇异构型(会失去可控性)。
3. **命令可用**:`fallback == 0` —— 每步都要解出能用的命令(求解器失败会回退保持,fallback>0 说明它经常解不出)。

> 为什么奇异按"稳态"判?机械臂的 **home(neutral)姿态本身就近奇异**(σ_min≈8e-6),离开 home 去追轨迹的
> 启动段会短暂近奇异——这是初始条件伪信号,不是求解器的错。**稳态窗口**只评价追踪稳定后的状态;
> 避障(碰撞)则必须覆盖全程,因为接近段撞障碍是真风险。

### 4.3 决策优先级(合格者内部排序)

**优先级链:安全 ＞ 求解时间 ＞ 精度 ＞ demo 完成时间。** 逐级比较,前一级分不出胜负才看下一级。

1. **安全(最高)**:先过 4.2 的硬门槛(避障 viol=0、稳态 σ_min≥1e-3、fallback=0)。不过 → 直接淘汰,
   后面的求解时间/精度再好也不看。
2. **求解时间**:合格者里,**P95(最坏单步耗时)越小越优**(P95 反映能否稳稳卡进 20ms 实时预算)。
3. **精度**:求解时间分不出高下时,再看**稳态 RMS 越小越优**。
4. **demo 完成时间(最低)**:仍并列时,选让 demo 更快跑完任务的(如支持更高限速/更短周期)。

> 注意:这与"精度优先"的直觉相反——**只要安全达标,求解时间比精度更重要**。因为快而稳的求解器留出更多
> 实时裕度、支持更高控制带宽;而三个合格解的精度已都是亚毫米级(0.26~0.74mm),差异对任务影响很小。

对比程序 [benchmark_nmpc_solvers.py](src/arm-platform/demo/benchmark_nmpc_solvers.py) 的 `rank_key()` 已按此
优先级排序(`(safe, p95_solve_ms, rms_position_error)`),终端"推荐"项即此优先级下的最优合格解。

### 4.4 测试结果（纯求解器版本）

**测试条件**:Jetson `nvpmodel` **40W** 模式(mode id `4`, 2026-07-03 实测)、部署避障几何(中心线胶囊 `capsule_points` + 斥力,前桨/机身共约 50 硬约束)、`periods=1.0`、`repeats=3`、稳态 RMS 口径。列三类场景(不含 `avoidance_sweep`;它每周期短暂穿近奇异,单列见下方备注)。"最大求解"= 三次重复中的最坏单步(含启动/重建尖峰);实时性主要看 P95 与 20ms 控制周期的裕度。原始数据在 `src/arm-platform/demo/data/jetson_40w_20260703/benchmark_nmpc_*.csv`。

#### lower_obstacle_circle
| 求解器配置 | 平均求解(ms) | P95求解(ms) | 最大求解(ms) | 稳态RMS(mm) | safe |
|---|---:|---:|---:|---:|:---:|
| acados SQP_RTI + qpOASES | 4.92 | 12.00 | 22.42 | 0.69 | ✅ |
| acados SQP_RTI + HPIPM | 15.98 | 35.65 | 84.99 | 0.69 | ✅ |
| GRAMPC | 19.56 | 20.30 | 21.65 | 0.59 | ✅ |

#### circle
| 求解器配置 | 平均求解(ms) | P95求解(ms) | 最大求解(ms) | 稳态RMS(mm) | safe |
|---|---:|---:|---:|---:|:---:|
| acados SQP_RTI + qpOASES | 3.98 | 4.27 | 13.60 | 2.41 | ✅ |
| acados SQP_RTI + HPIPM | 14.10 | 14.39 | 85.45 | 2.41 | ✅ |
| GRAMPC | 19.49 | 19.97 | 20.48 | 0.96 | ✅ |

#### front_wing_arc
| 求解器配置 | 平均求解(ms) | P95求解(ms) | 最大求解(ms) | 稳态RMS(mm) | safe |
|---|---:|---:|---:|---:|:---:|
| acados SQP_RTI + qpOASES | 5.30 | 12.76 | 20.48 | 2.37 | ✅ |
| acados SQP_RTI + HPIPM | 17.76 | 36.04 | 85.80 | 2.37 | ✅ |
| GRAMPC | 19.68 | 20.18 | 21.37 | 1.67 | ✅ |

**结论(40W 纯求解器)**:qpOASES / HPIPM / GRAMPC 三者全 safe;按优先级(安全>求解时间>精度)**首选 acados qpOASES**。qpOASES 在三场景 P95 均低于 13ms,实时裕度最好;HPIPM 在 lower/front 场景有明显启动/重建尖峰(P95 35ms+、最大约 85ms);GRAMPC 的 RMS 往往最好,但 P95 接近或略高于 20ms,适合作为精度/鲁棒性参考而非默认最快后端。

> **备注 `avoidance_sweep`**:该场景每周期短暂穿近奇异。新中心线胶囊几何下 qpOASES/HPIPM 已能保持 σ_min≈2.6e-3 过门槛(safe),但 **GRAMPC 在此场景擦障(viol≈14mm)+ 近奇异** → GRAMPC 的增广拉格朗日是软约束、苛刻场景可被突破,而 acados 是真硬约束。对"绝不碰撞"要求,acados 更可靠。

### 4.5 测试结果（接入 ROS 版本:软件栈 + 计算延迟）

4.4 是**纯求解器**闭环(Python 内部仿真 plant)的求解耗时;本节补充**接入 ROS 后的真实延迟**——把 demo 节点 [move_ll100_lh420_nmpc_demo.py](src/arm-platform/demo/move_ll100_lh420_nmpc_demo.py) 起在 ROS 2 里,用零反馈发布器 [ll100_lh420_zero_joint_state_publisher.py](src/arm-platform/demo/ll100_lh420_zero_joint_state_publisher.py) 喂 `/joint_states`,节点带 `latency_profiling:=true` 逐 tick 打点。它测的是节点里 **rclpy 回调调度 + FK/雅可比 + 求解 + marker 发布 + DDS** 的端到端软件栈延迟,不只是求解耗时。

**三个延迟量**(单位 ms):
- **`feedback_age`**:反馈到达 → tick 消费(发布频率伪影,非求解器性能)。
- **`compute`**:tick 入口 → publish(回调内全部计算:FK+雅可比+求解+marker+rclpy)。
- **`end_to_end`** = feedback_age + compute:传感器 → 命令,真实控制延迟。

**测试条件**:Jetson `nvpmodel` **40W** 模式(mode id `4`, 2026-07-03 实测)、RMW=Fast-DDS、`taskset -c 4-7` 固定 CPU、默认调度(`nice=0`)、独立 `ROS_DOMAIN_ID=81..89`、50Hz 反馈、跳过 5s 暖机、每格 1503-1520 个样本;三后端各在三场景跑一次。GRAMPC 已按当前约束重建问题模块。原始数据在 `src/arm-platform/demo/data/jetson_40w_20260703/latency_*.csv`,汇总见 `latency_summary_40w.csv`。

#### lower_obstacle_circle
| 求解器配置 | compute 均值(ms) | compute P95(ms) | end_to_end 均值(ms) | end_to_end P95(ms) |
|---|---:|---:|---:|---:|
| acados SQP_RTI + qpOASES | 17.80 | 18.24 | 18.26 | 18.74 |
| acados SQP_RTI + HPIPM | 22.50 | 22.85 | 22.98 | 23.34 |
| GRAMPC | 24.28 | 24.73 | 24.73 | 25.20 |

#### circle
| 求解器配置 | compute 均值(ms) | compute P95(ms) | end_to_end 均值(ms) | end_to_end P95(ms) |
|---|---:|---:|---:|---:|
| acados SQP_RTI + qpOASES | 16.74 | 17.21 | 17.20 | 17.70 |
| acados SQP_RTI + HPIPM | 28.44 | 29.03 | 28.93 | 29.54 |
| GRAMPC | 24.27 | 24.69 | 24.72 | 25.17 |

#### front_wing_arc
| 求解器配置 | compute 均值(ms) | compute P95(ms) | end_to_end 均值(ms) | end_to_end P95(ms) |
|---|---:|---:|---:|---:|
| acados SQP_RTI + qpOASES | 24.37 | 25.07 | 24.84 | 25.56 |
| acados SQP_RTI + HPIPM | 40.72 | 41.29 | 41.22 | 41.79 |
| GRAMPC | 24.38 | 24.89 | 24.84 | 25.38 |

**结论(接入 ROS)**:
1. **只有 lower/circle 的 qpOASES 保持在 20ms 周期内**(end_to_end P95 18.74/17.70ms);front_wing_arc + qpOASES 因 horizon=6、约束更重,端到端 P95 25.56ms。
2. **HPIPM 上机后一致最慢**,尤其 front_wing_arc 的 compute P95 41.29ms、end_to_end P95 41.79ms,不适合作为当前 50Hz 默认后端。
3. **GRAMPC 延迟稳定但约 25ms**,三场景 end_to_end P95 25.17-25.38ms,安全/精度可作为参考,但当前 50Hz 下实时裕度不足。若坚持 50Hz,优先用 qpOASES 并降低 front_wing_arc horizon/marker 开销;若使用 HPIPM/GRAMPC,建议降频到 25-40Hz 或进一步优化节点计算。

> **口径**:零反馈源 + 本机 ROS 回环、Fast-DDS 默认 QoS。此回环下臂不动,故**只测延迟、不测 RMS/safe**(要测精度/避障需把真实 plant 仿真接进 ROS);**不含真机传感器/总线延迟**,真机会更大。插桩藏在 `latency_profiling` 参数后(默认关),不影响正常运行。

**复现**(先按 §1.7 source 环境):
```bash
python3 ll100_lh420_zero_joint_state_publisher.py &     # 反馈源(50Hz)
python3 move_ll100_lh420_nmpc_demo.py --ros-args -p latency_profiling:=true \
  -p path_mode:=circle -p acados_qp_solver:=FULL_CONDENSING_QPOASES   # 换后端/场景改这两个参数
```

### 4.6 测试结果（Jetson MAXN_SUPER 锁频,部署定稿基线）

§4.5 的 40W 数据促成了一轮系统调优(过程记录见 [JETSON_DEPLOY_LOG.md](JETSON_DEPLOY_LOG.md)):
功耗档切 **MAXN_SUPER**(CPU 1497.6→1984 MHz,+32%)、`jetson_clocks` 锁频(已做成开机 systemd 服务)、
NMPC 节点以 **SCHED_FIFO 80** 实时优先级运行(`chrt -f 80 taskset -c 4-7`)。
2026-07-06 复测,其余口径与 §4.5 完全一致;原始数据在
`src/arm-platform/demo/data/jetson_maxn_super_20260706/`,汇总见 `latency_summary_maxn_super.csv`。

**end_to_end P95(ms),40W → MAXN_SUPER:**

| 场景 | qpOASES | GRAMPC | HPIPM |
|---|---|---|---|
| circle | 17.70 → **13.70** ✅ | 25.17 → 19.90 | 29.54 → (无效,见下) |
| lower_obstacle_circle | 18.74 → **14.54** ✅ | 25.20 → 20.13 | 23.34 → (无效,见下) |
| front_wing_arc | 25.56 → **18.28** ✅ | 25.37 → 20.06 | 41.79 → (无效,见下) |

**结论(部署定稿)**:
1. **qpOASES 三场景全部进入 20ms 周期**,提速比 1.29~1.40× 与主频提升(×1.325)吻合——纯频率收益,零代码改动。50Hz 部署推荐配置:**MAXN_SUPER + jetson_clocks + qpOASES + `chrt -f 80 taskset -c 4-7`**。
2. **GRAMPC** 压在 20ms 线上(19.9~20.1ms),50Hz 下仍不作默认后端,≤40Hz 可用。
3. **HPIPM 本轮数据无效,真因是 `acados_qp_solver_iter_max=500`**(2026-07-06 下午复测定论,推翻最初的"RT 节流"归因):部署调参把 QP 迭代上限默认设为 500,而 HPIPM 在本问题上**每步都跑满迭代上限**(500 次≈173ms、50 次≈31.6ms,耗时呈"恒定值"特征即为铁证;31.6ms 恰等于 40W 的 41.3ms ÷ 主频比 1.325)。qpOASES 靠热启动早收敛,不受该默认值影响。→ **若用 HPIPM,须显式传 `-p acados_qp_solver_iter_max:=50`**。原始数据见 `data/jetson_maxn_super_opt_20260706/latency_front_wing_arc_hpipm_iter{50,500}.csv`。

> **RT 节流**:已在 Jetson 上永久关闭(`sched_rt_runtime_us=-1`,见部署日志,含复位方法)。关闭后 qpOASES 的最坏单步从 67ms 收敛到 18.9ms——节流确实影响长尾,但不是 HPIPM 异常的主因。

### 4.7 节点热路径优化(诊断去重 + marker 降频,2026-07-06)

cProfile 剖析(front_wing_arc, 720 tick)发现两处大头:①`step()` 里避障诊断被 `_plan_feasible` 和
`_make_info` **各算一遍**(全时域 FK+数百次符号距离,约占 tick 的 1/4);②`_publish_markers` 每 tick
重建整套 MarkerArray(含最多 2500 点尾迹,约占 40%,虽在延迟打点之后、不计入 compute,但挤占 20ms 周期)。
两项修复:诊断算一遍两处复用([mpc_pinocchio_ik.py](src/arm-platform/demo/mpc_pinocchio_ik.py) 的
`step/solve`,语义等价);新增 `marker_publish_hz` 参数(默认 10Hz,≤0 关闭,≥50 恢复逐 tick)。

**优化效果(MAXN_SUPER + chrt -f 80,compute P95,ms):**

| 场景 | 优化前 | 优化后 | 有效控制频率 |
|---|---:|---:|---|
| circle | 13.29 | **9.33** | 49.7Hz |
| lower_obstacle_circle | 14.06 | **8.69** | 49.7Hz |
| front_wing_arc | 17.86 | **12.34** | **34.7 → 49.7Hz** |

**重要发现**:优化前 front_wing_arc 的 tick 总耗时(compute+marker)>20ms,节点实际只跑出 **~35Hz**
——此前各基线的 end_to_end"达标"是在欠频饱和状态下测得的(饱和时 tick 紧跟反馈消息执行,
feedback_age 假性≈0.4ms)。优化后恢复真 50Hz,feedback_age 变为 0~20ms 锯齿(50Hz 轮询回环的
相位伪影,非软件栈变慢),故新旧 end_to_end 不可直接比,**软件栈横向对比一律看 compute**。
原始数据与汇总:`data/jetson_maxn_super_opt_20260706/`(`latency_summary_opt.csv`)。

## 5. 教程:如何自己增 / 减求解器

**唯一事实源**是 [benchmark_nmpc_solvers.py](src/arm-platform/demo/benchmark_nmpc_solvers.py) 里的
`solver_matrix()` 函数——它返回一个 `SolverConfig` 列表,每个就是一个待比较的求解器配置。

一个 `SolverConfig` 长这样:
```python
SolverConfig(
    key="acados_sqp_rti_qpoases",          # 唯一名字(--only 用它)
    backend="acados",                      # 载体:acados / grampc
    nlp_solver_type="SQP_RTI",             # 本仓库只部署 SQP_RTI
    qp_solver="FULL_CONDENSING_QPOASES",   # acados 专用 QP 后端
    tiers=("acados",),                     # 用于 --tier 过滤
    label="acados SQP_RTI + qpOASES",      # 显示名
    status="active",                       # active=参与对比
    note="...",                            # 备注
    extra_overrides=(("acados_qp_solver_iter_max", 500),  # 覆盖 MpcIkConfig 的任意字段
                     ("acados_qp_warm_start", 1)),
)
```

### 5.1 调整 acados QP 后端参数
当前只保留 HPIPM 与 qpOASES。若要比较 qpOASES 在不同迭代上限下的表现,复制
`acados_sqp_rti_qpoases` 那一行,修改 `key` 和 `extra_overrides` 即可。

### 5.2 增加一个"同求解器不同参数"的变体(用 extra_overrides)
例:比较 qpOASES 在 iter_max=100 vs 1000。复制两行,`extra_overrides` 分别设不同 `acados_qp_solver_iter_max`,
`key` 取不同名即可。`extra_overrides` 能覆盖 `MpcIkConfig` 的任意字段(权重、horizon、margin、
`acados_hessian_approx` 等),改完自动进 build_dir 后缀,不同变体各自编译。

### 5.3 删除一个求解器
直接从 `solver_matrix()` 删除对应配置,并同步移除 demo 参数校验和部署脚本中的相关依赖。当前仓库已经
按部署目标裁剪,不再保留弃置求解器的运行路径。

### 5.4 增加一个全新求解器载体(工作量大,参考 GRAMPC)
如果不是 acados 的 QP 变体,而是一个全新求解器(像 GRAMPC),要:
1. 写一个运行器类,接口对齐 `MpcPinocchioIK.step(targets, q0, warm_start, target_rotation) -> (q_next, ok, info)`,
   `info` 要含 `solve_time/terminal_error/trajectory/status/success/command_usable/nit`;并代理
   `forward/model/data/frame_id/q_lower/q_upper/_obstacle_diagnostics/obstacle_disks/body_obstacle_boxes/
   capsule_sample_positions`(供 benchmark 与 demo 复用)。参考 [grampc_ll100.py](src/arm-platform/demo/grampc_ll100.py)。
2. 在 `evaluate()` 里加一个 `sc.backend == "你的名字"` 分支,构造你的运行器(参考 GRAMPC 分支)。
3. 在 `solver_matrix()` 加对应 `SolverConfig(backend="你的名字", ...)`。
4. 若要在实机 demo 也能用,在 [move_ll100_lh420_nmpc_demo.py](src/arm-platform/demo/move_ll100_lh420_nmpc_demo.py)
   的 backend 分支里也加一路(参考 grampc 分支)。

### 5.5 GRAMPC 专有调参
GRAMPC 的两个关键旋钮由环境变量控制(便于扫最优,默认已是最优值):
```bash
GRAMPC_TARGET_INDEX=2      python3 benchmark_nmpc_solvers.py --only grampc --periods 1.0   # 前瞻目标索引(默认2)
GRAMPC_MAX_GRAD_ITER=3     ...                                                             # 内层梯度迭代
GRAMPC_MAX_MULT_ITER=2     ...                                                             # 乘子迭代
```
> `GRAMPC_TARGET_INDEX` 从 -1(瞄最远)改成 2(瞄前方约一个时域)是 GRAMPC 从 8.34mm 提到 0.26mm 的关键;
> 该最优值与 `horizon_steps/preview_stride` 绑定,换基线要重扫。

---

## 7. 在 LL100 demo 里运行不同的求解器

实机/仿真 demo [move_ll100_lh420_nmpc_demo.py](src/arm-platform/demo/move_ll100_lh420_nmpc_demo.py)
已暴露求解器参数(默认 = acados SQP_RTI + qpOASES)。切换只需传 ROS 参数:

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
# (先 source 环境,见 1.6)

# ① 默认:acados qpOASES
python3 move_ll100_lh420_nmpc_demo.py

# ② acados HPIPM
python3 move_ll100_lh420_nmpc_demo.py --ros-args -p acados_qp_solver:=PARTIAL_CONDENSING_HPIPM

# ③ GRAMPC(精度第一)
python3 move_ll100_lh420_nmpc_demo.py --ros-args -p solver_backend:=grampc

```

三个求解器选择参数:

| ROS 参数 | 默认 | 取值 |
|---|---|---|
| `solver_backend` | `acados` | `acados` / `grampc` |
| `acados_nlp_solver_type` | `SQP_RTI` | `SQP_RTI` |
| `acados_qp_solver` | `FULL_CONDENSING_QPOASES` | `FULL_CONDENSING_QPOASES` / `PARTIAL_CONDENSING_HPIPM` |
| `acados_qp_solver_iter_max` | `500` | 正整数 |
| `acados_qp_warm_start` | `1` | `0` / `1` / `2` |

> - GRAMPC 需先按 1.2 装好 pygrampc 且设 `PYGRAMPC_ROOT`。
> - 组合示例:同时改 backend 和 qp,多个 `-p` 并列即可。

---

## 附:与其他 README 的关系

- 本 README 是求解器选型的单一事实源。
- [LL100_LH420_MPC_README.md](LL100_LH420_MPC_README.md)(LL100 demo 说明)与
  [MPC_TUNING_README.md](MPC_TUNING_README.md)(调参说明)应指向本文的部署口径。
- [7DOF_ARM_NMPC_README.md](7DOF_ARM_NMPC_README.md):7 自由度臂 demo 只保留 acados 路径;GRAMPC 是 LL100 专属。
