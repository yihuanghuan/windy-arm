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

### 1.1 acados(主力,提供 HPIPM / qpOASES / OSQP 三个 QP 后端)
acados 是一个 NMPC/OCP 求解框架:它做 SQP/RTI 迭代,每次迭代把问题线性化成一个 QP,交给一个 **QP 后端**求解。
所以 "HPIPM / qpOASES / OSQP" 不是三个独立求解器,而是 acados 内部可换的 QP 后端。

- 安装位置:`~/code/acados`(源码 + 编译产物 `lib/`)。
- QP 后端由编译开关决定。查当前开关:
  ```bash
  grep -iE "ACADOS_WITH_(HPIPM|QPOASES|OSQP|DAQP):" ~/code/acados/build/CMakeCache.txt
  ```
- 本环境已编译:**HPIPM + qpOASES + OSQP**(`ACADOS_WITH_OSQP=ON`)。DAQP 未编译。
- **OSQP 是后加的**:acados 默认不含 OSQP,需重编。源码已在 `~/code/acados/external/osqp`,重编步骤:
  ```bash
  cp -a ~/code/acados/lib ~/code/acados/lib_backup           # 先备份
  cd ~/code/acados/build
  /usr/bin/cmake -DACADOS_WITH_OSQP=ON .                     # 用系统 cmake(3.x),别用 pip 装的 cmake4
  /usr/bin/cmake --build . --target install -j
  # 验证:nm -D ~/code/acados/lib/libacados.so | grep ocp_qp_osqp 应有输出
  ```
  ⚠️ 重编会替换共享的 `libacados.so`,若有别的进程在用 acados,先协调停手。

### 1.2 CasADi + IPOPT(弃置回退,保留作离线高精度参照)
- IPOPT 通过 CasADi 调用。安装:`pip install casadi`(或系统包 `python3-casadi`)。
- 定位:太慢(~1100ms/步、实时率 0%),不进实时回路,**已弃置**;但完整收敛、精度高,留作离线 ground-truth。

### 1.3 SciPy + SLSQP(弃置回退)
- 用 `scipy.optimize.minimize(method="SLSQP")`,`pip install scipy` 即可。
- 定位:~45ms/步、实时率低,**已弃置**。

### 1.4 GRAMPC(增广拉格朗日 + 梯度法,C++ 编译问题描述)
GRAMPC 通过 Python 绑定 **pygrampc** 使用,问题描述用 CasADi codegen 出 C 导数、编译成 C++ 模块(测速真实)。

前置:**Eigen 3.4**(`pkg-config --modversion eigen3` 应 ≥ 3.4)、C++ 编译器。安装:
```bash
# 1) 拉源码(pygrampc + grampc 子模块)。建议放持久目录,别放 /tmp
git clone https://github.com/grampc/pygrampc.git ~/code/pygrampc
cd ~/code/pygrampc && git submodule update --init --recursive
# 2) 修构建依赖(旧 packaging 会报 InvalidName)
pip install --user --upgrade "packaging>=24" scikit-build-core pybind11 ninja cmake
# 3) 免构建隔离编译安装
pip install . --no-build-isolation --user
python3 -c "import pygrampc; print(pygrampc.__version__)"   # 应打印 2.2.2
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

### 1.5 Crocoddyl(DDP + mim_solvers CSQP 硬约束,源码编译;已实测淘汰)
Crocoddyl 是 DDP/FDDP 最优控制框架,配 mim_solvers 的 CSQP 可做硬约束。已接入对比
([crocoddyl_ll100.py](src/arm-platform/demo/crocoddyl_ll100.py)),但**实测非实时**(~100–250ms/步、fallback 上千、过不了安全门槛,见 §4.4 备注),仅作 DDP 路线对照,不进实时回路。

- **务必 ABI 兼容**:本机 Pinocchio 是 apt 包 `ros-humble-pinocchio` 3.9.0(`/opt/ros/humble`)。
  **绝不能 `pip install crocoddyl/pinocchio`**——会拉进第二个 Pinocchio 与 ROS 版冲突。apt 源无现成
  `ros-humble-crocoddyl`,故针对 ROS 的 Pinocchio 从源码编译。
- 依赖:`sudo apt-get install -y ros-humble-jrl-cmakemodules`。
- 源码 + 版本(装到独立前缀,不污染系统):
  ```bash
  ROOT=~/.local/src/crocoddyl_ros_humble
  PREFIX=~/.local/opt/crocoddyl_ros_humble_pinocchio39
  git clone --depth 1 -b v2.0.0 https://github.com/jrl-umi3218/jrl-cmakemodules.git $ROOT/jrl-cmakemodules
  git clone --depth 1 -b v3.2.1 https://github.com/loco-3d/crocoddyl.git             $ROOT/crocoddyl
  git clone --depth 1 -b v0.3.0 https://github.com/machines-in-motion/mim_solvers.git $ROOT/mim_solvers
  # 依次 cmake(-G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX)构建 + install
  # 提速:crocoddyl 的 CMakeLists 里 SCALAR_TYPES 只留 double(删 float),省一半绑定编译量
  ```
- 用前把前缀加进 `PYTHONPATH`,并验证 Pinocchio 仍是 ROS 版(没被换掉):
  ```bash
  export PYTHONPATH=$PREFIX/lib/python3.10/site-packages:$PYTHONPATH
  python3 -c "import pinocchio,crocoddyl,mim_solvers; print(pinocchio.__version__)"   # 应仍是 3.9.0
  ```
- 跑对比:`python3 benchmark_nmpc_solvers.py --only crocoddyl --path-mode <场景> --periods 1.0`
  (它 `status=pending`、单独 tier,不进默认扫描;单步超时上限由 `CROCODDYL_MAX_SOLVE_TIME` 控制,默认 0.25s)。

### 1.6 各求解器速查

| 求解器 | 载体 | 安装 | 状态 |
|---|---|---|---|
| acados + HPIPM | acados 内置 | 编译 acados | **合格(默认)** |
| acados + qpOASES | acados(`ACADOS_WITH_QPOASES=ON`) | 编译 acados | **合格** |
| acados + OSQP | acados(需 `-DACADOS_WITH_OSQP=ON` 重编) | 见 1.1 | 不合格(卡奇异) |
| acados 全SQP | acados(`nlp_solver_type=SQP`) | 编译 acados | 不合格(卡奇异) |
| GRAMPC | pygrampc + C++ codegen | 见 1.4 | **合格** |
| Crocoddyl | crocoddyl + mim_solvers(源码编译) | 见 1.5 | **淘汰(非实时,~100–250ms/步)** |
| CasADi + IPOPT | casadi | `pip install casadi` | 弃置回退 |
| SciPy + SLSQP | scipy | `pip install scipy` | 弃置回退 |

### 1.7 激活环境(每次跑之前)
```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
export PYGRAMPC_ROOT=~/code/pygrampc        # GRAMPC 用
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
| `--tier` | `all` | 只跑某一档:`A`=完整求解器 / `B`=固定 acados 换 QP 后端 / `all` |
| `--only` | 无 | 逗号分隔的求解器 key,只跑指定的(无视 status,可复跑淘汰项) |
| `--include-eliminated` | 关 | 连同弃置/待启用的一起跑(默认只跑 `status=active`) |
| `--max-velocity` | `0.5` | 闭环 plant 的底层关节限速 rad/s(与 launch 一致) |
| `--plot` / `--no-plot` | 开 | 是否出 P95 速度 + RMS 精度柱状图(需 matplotlib) |
| `--quiet-build` / `--no-quiet-build` | 开 | 是否静音 acados/GRAMPC 的代码生成/编译输出 |
| `--output-dir` | `demo/data` | CSV/图输出目录 |

求解器 key:`acados_sqp_rti_hpipm` / `acados_sqp_rti_qpoases` / `acados_sqp_rti_osqp` /
`acados_sqp_hpipm`(全SQP)/ `grampc` / `casadi_ipopt` / `scipy_slsqp`。

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
> 启动段会短暂近奇异——这是初始条件伪信号,不是求解器的错。**稳态窗口**能正确区分:三合格解稳态
> σ_min≈0.03–0.04(安全),而全SQP/OSQP 是**全程卡在奇异**(σ_min≈3e-5,真不合格)。避障(碰撞)则必须
> 覆盖全程,因为接近段撞障碍是真风险。

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

**测试条件**:部署避障几何(中心线胶囊 `capsule_points` + 斥力,前桨/机身共约 50 硬约束)、`periods=1.0`、稳态 RMS 口径。列三类场景(不含 `avoidance_sweep`;它每周期短暂穿近奇异,单列见下方备注)。"最大求解"= 单轮最坏单步(含启动尖峰,仍应 <20ms 实时预算)。

#### lower_obstacle_circle
| 求解器配置 | 平均求解(ms) | 最大求解(ms) | 稳态RMS(mm) | safe |
|---|---:|---:|---:|:---:|
| acados SQP_RTI + qpOASES | 1.00 | 11.14 | 0.69 | ✅ |
| acados SQP_RTI + HPIPM | 2.53 | 15.92 | 0.69 | ✅ |
| GRAMPC | 4.23 | 8.79 | 0.59 | ✅ |
| acados 全SQP + HPIPM | 43.72 | 55.37 | 198.87 | ❌ |
| acados SQP_RTI + OSQP | 92.53 | 169.70 | 337.59 | ❌ |
| CasADi + IPOPT | ~1100 | — | — | 弃置 |
| SciPy + SLSQP | ~45 | — | — | 弃置 |

#### circle
| 求解器配置 | 平均求解(ms) | 最大求解(ms) | 稳态RMS(mm) | safe |
|---|---:|---:|---:|:---:|
| acados SQP_RTI + qpOASES | 0.88 | 4.96 | 2.41 | ✅ |
| acados SQP_RTI + HPIPM | 2.36 | 16.02 | 2.41 | ✅ |
| GRAMPC | 4.28 | 6.33 | 0.96 | ✅ |
| acados 全SQP + HPIPM | 3.40 | 55.75 | 2.41 | ✅ |
| acados SQP_RTI + OSQP | 4.52 | 94.50 | 2.37 | ❌ |
| CasADi + IPOPT | ~1100 | — | — | 弃置 |
| SciPy + SLSQP | ~45 | — | — | 弃置 |

#### front_wing_arc
| 求解器配置 | 平均求解(ms) | 最大求解(ms) | 稳态RMS(mm) | safe |
|---|---:|---:|---:|:---:|
| acados SQP_RTI + qpOASES | 1.12 | 9.17 | 2.37 | ✅ |
| acados SQP_RTI + HPIPM | 2.87 | 13.04 | 2.38 | ✅ |
| GRAMPC | 4.70 | 6.59 | 1.67 | ✅ |
| acados 全SQP + HPIPM | 37.31 | 49.66 | 138.21 | ❌ |
| acados SQP_RTI + OSQP | 118.89 | 180.19 | 234.37 | ❌ |
| CasADi + IPOPT | ~1100 | — | — | 弃置 |
| SciPy + SLSQP | ~45 | — | — | 弃置 |

> **弃置两项(CasADi-IPOPT / SciPy-SLSQP)**:实时率 0%/低,不进实时回路,故不在新几何逐场景重跑;
> 平均求解为旧几何/离线实测量级(新几何约束更多只会更慢),仅作对照。IPOPT 完整收敛可作离线精度 ground-truth。

**结论(三场景一致)**:qpOASES / HPIPM / GRAMPC 三者全 safe;按优先级(安全>求解时间>精度)**首选 acados qpOASES**(平均 0.9–1.1ms、最快),HPIPM 次之(均衡),GRAMPC 精度常最好(0.59–1.67mm)且最大求解最稳(无启动尖峰)。全SQP/OSQP 除 circle 外基本不合格。

> **备注 `avoidance_sweep`**:该场景每周期短暂穿近奇异。新中心线胶囊几何下 qpOASES/HPIPM 已能保持 σ_min≈2.6e-3 过门槛(safe),但 **GRAMPC 在此场景擦障(viol≈14mm)+ 近奇异** → GRAMPC 的增广拉格朗日是软约束、苛刻场景可被突破,而 acados 是真硬约束。对"绝不碰撞"要求,acados 更可靠。

### 4.5 测试结果（接入 ROS 版本:软件栈 + 计算延迟）

4.4 是**纯求解器**闭环(Python 内部仿真 plant)的求解耗时;本节补充**接入 ROS 后的真实延迟**——把 demo 节点 [move_ll100_lh420_nmpc_demo.py](src/arm-platform/demo/move_ll100_lh420_nmpc_demo.py) 起在 ROS 2 里,用零反馈发布器 [ll100_lh420_zero_joint_state_publisher.py](src/arm-platform/demo/ll100_lh420_zero_joint_state_publisher.py) 喂 `/joint_states`,节点带 `latency_profiling:=true` 逐 tick 打点。它测的是节点里 **rclpy 回调调度 + FK/雅可比 + 求解 + marker 发布 + DDS** 的端到端软件栈延迟,不只是求解耗时。

**三个延迟量**(单位 ms):
- **`feedback_age`**:反馈到达 → tick 消费(发布频率伪影,非求解器性能)。
- **`compute`**:tick 入口 → publish(回调内全部计算:FK+雅可比+求解+marker+rclpy)。
- **`end_to_end`** = feedback_age + compute:传感器 → 命令,真实控制延迟。

**测试条件**:RMW=Fast-DDS、`taskset` 核隔离 + `nice` 降扰、独立 `ROS_DOMAIN_ID`、50Hz 反馈、跳过 5s 暖机、每格约 1500 样本;三后端各在三场景跑一次。GRAMPC 需先把问题模块重建到当前约束(见 §1.4)才能上机。

#### lower_obstacle_circle
| 求解器配置 | compute 均值(ms) | compute P95(ms) | end_to_end 均值(ms) | end_to_end P95(ms) |
|---|---:|---:|---:|---:|
| acados SQP_RTI + qpOASES | 7.06 | 7.44 | 12.90 | 13.08 |
| acados SQP_RTI + HPIPM | 7.97 | 8.28 | 15.43 | 16.41 |
| GRAMPC | 7.12 | 7.45 | 15.83 | 16.59 |

#### circle
| 求解器配置 | compute 均值(ms) | compute P95(ms) | end_to_end 均值(ms) | end_to_end P95(ms) |
|---|---:|---:|---:|---:|
| acados SQP_RTI + qpOASES | 9.25 | 9.72 | 15.80 | 16.85 |
| acados SQP_RTI + HPIPM | 12.35 | 12.70 | 15.95 | 16.81 |
| GRAMPC | 6.87 | 7.08 | 15.68 | 16.84 |

#### front_wing_arc
| 求解器配置 | compute 均值(ms) | compute P95(ms) | end_to_end 均值(ms) | end_to_end P95(ms) |
|---|---:|---:|---:|---:|
| acados SQP_RTI + qpOASES | 6.48 | 6.89 | 15.85 | 16.47 |
| acados SQP_RTI + HPIPM | 9.57 | 10.18 | 15.32 | 15.65 |
| GRAMPC | 7.01 | 7.50 | 15.81 | 16.75 |

**结论(接入 ROS)**:
1. **三后端 compute P95 全 ≤ 12.7ms、稳稳 <20ms** —— 接入 ROS 后仍全部实时可行。
2. **约 6ms 的节点固定开销(FK+雅可比+marker+rclpy)压平了求解器差异**:纯求解 4.7 倍差距(qpOASES ~1ms / HPIPM ~2.5ms / GRAMPC ~4.7ms)上机后压缩到 <2 倍;唯一仍可见的规律是 **HPIPM 上机后一致最慢**(partial-condensing 每次调用开销更大,circle 场景 P95 12.7ms)。
3. **`end_to_end` P95 稳定在 ~13–17ms,与后端/场景基本无关** —— compute 与 feedback_age 此消彼长、填满约一个 tick 周期。**上机后端到端延迟的瓶颈是控制周期时序,不是求解器速度。**

> **口径**:零反馈源 + 本机 ROS 回环、Fast-DDS 默认 QoS。此回环下臂不动,故**只测延迟、不测 RMS/safe**(要测精度/避障需把真实 plant 仿真接进 ROS);**不含真机传感器/总线延迟**,真机会更大。插桩藏在 `latency_profiling` 参数后(默认关),不影响正常运行。

**复现**(先按 §1.7 source 环境):
```bash
python3 ll100_lh420_zero_joint_state_publisher.py &     # 反馈源(50Hz)
python3 move_ll100_lh420_nmpc_demo.py --ros-args -p latency_profiling:=true \
  -p path_mode:=circle -p acados_qp_solver:=FULL_CONDENSING_QPOASES   # 换后端/场景改这两个参数
```

## 5. 教程:如何自己增 / 减求解器

**唯一事实源**是 [benchmark_nmpc_solvers.py](src/arm-platform/demo/benchmark_nmpc_solvers.py) 里的
`solver_matrix()` 函数——它返回一个 `SolverConfig` 列表,每个就是一个待比较的求解器配置。

一个 `SolverConfig` 长这样:
```python
SolverConfig(
    key="acados_sqp_rti_qpoases",          # 唯一名字(--only 用它)
    backend="acados",                      # 载体:acados / casadi / scipy / grampc
    nlp_solver_type="SQP_RTI",             # acados 专用:SQP_RTI / SQP
    qp_solver="FULL_CONDENSING_QPOASES",   # acados 专用 QP 后端
    tiers=("B",),                          # 属于哪档(A/B),仅用于 --tier 过滤
    label="acados SQP_RTI + qpOASES",      # 显示名
    status="active",                       # active=参与对比 / eliminated=弃置只记录 / pending=待启用
    note="...",                            # 备注
    extra_overrides=(("acados_qp_solver_iter_max", 500),  # 覆盖 MpcIkConfig 的任意字段
                     ("acados_qp_warm_start", 1)),
)
```

### 5.1 增加一个 acados QP 后端变体(最简单,零重构)
例:加 acados 的 DAQP 后端。
1. 先确认 acados 编了 DAQP:`grep ACADOS_WITH_DAQP ~/code/acados/build/CMakeCache.txt`。没有就先重编(见 1.1)。
2. 在 `solver_matrix()` 列表里加一行:
   ```python
   SolverConfig("acados_sqp_rti_daqp", "acados", "SQP_RTI",
                "FULL_CONDENSING_DAQP", ("B",), "acados SQP_RTI + DAQP", status="active"),
   ```
3. 跑:`python3 benchmark_nmpc_solvers.py --only acados_sqp_rti_daqp --periods 1.0`。
> 原理:不同 QP 后端会各自生成独立的 acados 编译目录(build_dir 后缀含 nlp/qp/iter/ws),互不污染。

### 5.2 增加一个"同求解器不同参数"的变体(用 extra_overrides)
例:比较 qpOASES 在 iter_max=100 vs 1000。复制两行,`extra_overrides` 分别设不同 `acados_qp_solver_iter_max`,
`key` 取不同名即可。`extra_overrides` 能覆盖 `MpcIkConfig` 的任意字段(权重、horizon、margin、
`acados_hessian_approx` 等),改完自动进 build_dir 后缀,不同变体各自编译。

### 5.3 弃置/淘汰一个求解器(不删除)
把它那行的 `status` 从 `"active"` 改成 `"eliminated"`,补 `note` 说明原因。它就不再跑、只作为记录行
出现在汇总表和 CSV 里(保留可追溯,想复跑用 `--only <key>`)。IPOPT/SLSQP 就是这么处理的。

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
已暴露求解器参数(默认 = acados SQP_RTI + HPIPM,与原行为一致)。切换只需传 ROS 参数:

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
# (先 source 环境,见 1.6)

# ① 默认:acados HPIPM
python3 move_ll100_lh420_nmpc_demo.py

# ② acados qpOASES(速度第一)
python3 move_ll100_lh420_nmpc_demo.py --ros-args -p acados_qp_solver:=FULL_CONDENSING_QPOASES

# ③ GRAMPC(精度第一)
python3 move_ll100_lh420_nmpc_demo.py --ros-args -p solver_backend:=grampc

# ④ acados 全 SQP(不合格,仅试)
python3 move_ll100_lh420_nmpc_demo.py --ros-args -p acados_nlp_solver_type:=SQP

# ⑤ 弃置回退 IPOPT(会打印弃置警告,慢)
python3 move_ll100_lh420_nmpc_demo.py --ros-args -p solver_backend:=casadi
```

三个求解器选择参数:

| ROS 参数 | 默认 | 取值 |
|---|---|---|
| `solver_backend` | `acados` | `acados` / `grampc` / `casadi`(IPOPT,弃置)/ `scipy`(SLSQP,弃置) |
| `acados_nlp_solver_type` | `SQP_RTI` | `SQP_RTI` / `SQP`(仅 acados) |
| `acados_qp_solver` | `PARTIAL_CONDENSING_HPIPM` | `..._HPIPM` / `FULL_CONDENSING_QPOASES` / `PARTIAL_CONDENSING_OSQP`(需重编 acados) |

> - OSQP 需先按 1.1 重编 acados 才能选。
> - GRAMPC 需先按 1.4 装好 pygrampc 且设 `PYGRAMPC_ROOT`。
> - 组合示例:同时改 backend 和 qp,多个 `-p` 并列即可。
> - 选 `casadi`/`scipy` 时节点会打印"弃置回退"警告——它们能跑但慢,不建议实时用。

---

## 附:与其他 README 的关系

- 本 README 是求解器选型的单一事实源。
- [LL100_LH420_MPC_README.md](LL100_LH420_MPC_README.md)(LL100 demo 说明)与
  [MPC_TUNING_README.md](MPC_TUNING_README.md)(调参说明 §8.2"三种后端")建议各加一处指向本文的链接,
  并把"三种后端 acados/casadi/scipy"更新为"三合格解 + 两弃置回退"。这两个文件由调参侧维护,改动需协调。
- [7DOF_ARM_NMPC_README.md](7DOF_ARM_NMPC_README.md):7 自由度臂 demo 仍只支持 acados/casadi/scipy
  (GRAMPC 是 LL100 专属),基本无需改;顶多注明 casadi/scipy 为弃置回退。
