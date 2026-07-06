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

### 4.1 指标定义

求解器筛选分两层口径:**纯求解器**闭环(Python 内部仿真 plant,只测求解耗时与精度)用下面第一组指标;**接入 ROS** 的节点延迟(见 §4.5)用第二组延迟指标。

**纯求解器指标:**

| 指标 | 含义 | 怎么看 |
|---|---|---|
| `mean_ms` / `P95` | 每步求解耗时的均值 / 第95百分位(ms) | 控制回路 50Hz = 每 **20ms** 必须出命令。看 **P95<20ms** 判能否实时 |
| `realtime_rate`(rt%) | 求解时间 < 20ms 的步数占比 | 越接近 100% 越好 |
| `稳态 RMS` | 排除启动暂态后,末端实际位置 vs 目标的均方根误差 | **越小越准**。必须是稳态口径,否则被启动暂态淹没 |
| `σ_min`(sigmin) | 末端雅可比最小奇异值(稳态窗口内) | 越大越远离奇异;**< 1e-3 视为近奇异**(危险) |
| `violation_max`(viol) | 避障硬约束最大侵入量(全程) | **必须恒为 0**,>0 = 撞了障碍 |

**接入 ROS 的延迟指标**(单位 ms;由 demo 节点带 `latency_profiling:=true` 逐 tick 打点):

| 指标 | 含义 | 怎么看 |
|---|---|---|
| `compute` | 一个控制周期(tick)回调内的真实计算耗时:正运动学 FK + 雅可比 + 求解 QP + 避障诊断 + rclpy 开销 | **横向比较求解器快慢看这一列**,越小越好 |
| `feedback_age` | 本次 tick 所用的那帧反馈数据在缓存里已停留的时长 | 取决于节点是否真跑满 50Hz,是零反馈回环的相位伪影,**不代表求解器性能** |
| `end_to_end` | = `feedback_age` + `compute`,即传感器 → 命令的真实控制延迟 | **反直觉,不能用来横向比谁快**:算得快的求解器(qpOASES)提前算完、要空等固定节奏的下一个 tick 才发命令,`feedback_age` 反而大,导致 e2e 偏高;算得慢的求解器 tick 紧贴反馈饱和执行,`feedback_age`≈0,e2e 反而低。**选型只看 `compute` 和有效频率** |
| `有效频率` | 节点实测控制频率(Hz) | 越接近 50Hz 裕度越足 |

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

### 4.4 测试环境与场景

**测试环境**(既定部署前提):
- 板卡:Jetson Orin NX,`nvpmodel` 功耗档 **MAXN_SUPER**(mode id `0`,CPU 上限 1984 MHz)。
- **`jetson_clocks` 锁频**:CPU 全核 Min=Max=1984000、EMC/GPU 拉满(已做成开机 systemd 服务)。
- **关闭 RT 节流**:`kernel.sched_rt_runtime_us=-1`(已持久化),SCHED_FIFO 任务不受内核默认 95% CPU 上限限制。
- **NMPC 节点实时优先级绑核运行**:`chrt -f 80 taskset -c 4-7`(SCHED_FIFO 80,绑定 4-7 核)。
- RMW=Fast-DDS(默认 QoS)、50Hz 反馈、跳过 5s 暖机;接入 ROS 延迟由节点带 `latency_profiling:=true` 逐 tick 打点。
- GRAMPC 已按当前约束重建问题模块;HPIPM 一律 `acados_qp_solver_iter_max=50`(见本节参数注意)。

**三个测试场景**(`--path-mode` / demo 的 `path_mode`):
- `lower_obstacle_circle`:低位绕障圆轨迹,末端在障碍下方绕圆,避障约束持续激活。
- `circle`:标准圆轨迹,无避障几何介入,作为轻载基准场景。
- `front_wing_arc`:前桨弧线轨迹,horizon=6、约束最重(前桨/机身共约 50 硬约束),是三场景中计算最重的。

**参数使用注意**:
- HPIPM 后端必须显式设 `acados_qp_solver_iter_max=50`。用默认 500 会每步跑满迭代上限、单步约 170ms。
- 当前延迟数据用零反馈发布器(机械臂不动)测得,只测延迟、不含跟踪精度(不测 RMS/safe)。

**复现**(先按 §1.4 source 环境;纯求解器对比见 §2):
```bash
# 接入 ROS 延迟(逐 tick 打点)
python3 ll100_lh420_zero_joint_state_publisher.py &     # 反馈源(50Hz)
chrt -f 80 taskset -c 4-7 python3 move_ll100_lh420_nmpc_demo.py --ros-args \
  -p latency_profiling:=true \
  -p path_mode:=circle \
  -p acados_qp_solver:=FULL_CONDENSING_QPOASES   # 换后端/场景改这两个参数
# 用 HPIPM 时追加:-p acados_qp_solver:=PARTIAL_CONDENSING_HPIPM -p acados_qp_solver_iter_max:=50
```

### 4.5 接入 ROS 延迟结果

把 demo 节点 [move_ll100_lh420_nmpc_demo.py](src/arm-platform/demo/move_ll100_lh420_nmpc_demo.py) 起在 ROS 2 里、用零反馈发布器 [ll100_lh420_zero_joint_state_publisher.py](src/arm-platform/demo/ll100_lh420_zero_joint_state_publisher.py) 喂 `/joint_states`,在 §4.4 环境下逐 tick 打点。三后端各在三场景各跑一次;指标定义见 §4.1。
**"有效频率"列 = 节点实测控制频率**,是判断是否真正实时的关键;`e2e` 两列的读法见表后说明。
原始数据与汇总:`src/arm-platform/demo/data/jetson_maxn_super_opt_20260706/`(`latency_summary_opt.csv`)。

**lower_obstacle_circle**
| 求解器配置 | compute 均值 | compute P95 | e2e 均值 | e2e P95 | 有效频率 |
|---|---:|---:|---:|---:|---:|
| acados SQP_RTI + qpOASES | 8.22 | **8.69** | 21.92 | 44.05 | **49.7Hz** |
| acados SQP_RTI + HPIPM | 13.03 | 13.67 | 23.25 | 42.84 | 50.0Hz |
| GRAMPC | 18.12 | 18.80 | 19.57 | 19.43 | 43.9Hz |

**circle**
| 求解器配置 | compute 均值 | compute P95 | e2e 均值 | e2e P95 | 有效频率 |
|---|---:|---:|---:|---:|---:|
| acados SQP_RTI + qpOASES | 8.60 | **9.33** | 22.18 | 44.88 | **49.7Hz** |
| acados SQP_RTI + HPIPM | 18.28 | 18.97 | 19.92 | 20.24 | 47.2Hz† |
| GRAMPC | 18.11 | 18.75 | 19.73 | 19.54 | 43.9Hz |

**front_wing_arc**
| 求解器配置 | compute 均值 | compute P95 | e2e 均值 | e2e P95 | 有效频率 |
|---|---:|---:|---:|---:|---:|
| acados SQP_RTI + qpOASES | 11.67 | **12.34** | 23.47 | 47.94 | **49.7Hz** |
| acados SQP_RTI + HPIPM | 30.69 | 31.64 | 31.07 | 32.05 | 20.9Hz |
| GRAMPC | 18.16 | 18.82 | 19.59 | 19.49 | 43.5Hz |

> † circle HPIPM 样本较少(n=326),P95 供参考。单位均为 ms。

**怎么读这张表(重要,别被 e2e 误导)**:
- **选型只看 `compute` 和有效频率**。`compute`=回调内真实计算耗时(FK+雅可比+求解+诊断),越小越好;
  有效频率越接近 50Hz 说明实时裕度越足。结论一目了然:**qpOASES compute 全场景 <13ms、稳稳真 50Hz,
  裕度最好**;GRAMPC 恒定 ~18ms、只能跑到 ~44Hz;HPIPM 随场景波动,front_wing_arc 达 31ms、掉到 20.9Hz。
- **`e2e` 两列反直觉、不能横向比谁快**。e2e = feedback_age + compute,而 feedback_age 取决于"节点是否
  真跑满 50Hz":qpOASES 算得快、轻松跑满,每个 tick 要等下一帧反馈,feedback_age 出现 0~20ms 锯齿,
  于是 e2e P95 反而"最高"(~44ms);GRAMPC/HPIPM 算得慢、tick 紧贴反馈饱和执行,feedback_age≈0,e2e 看着"最低"。
  **务必以 compute 为准。**

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
