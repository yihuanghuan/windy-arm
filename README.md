# windylab_ws 文档索引

本仓库是机械臂 NMPC(非线性模型预测控制)平台,涉及**两套机械臂系统**和**四类主题**的说明文档。
本文件是**导航总入口**——先在这里找到该看哪个文档,再点进去。

---

## 一、两套机械臂系统(先分清,别混)

| 系统 | 是什么 | 自由度 | 相关文档 |
|---|---|---|---|
| **A. 固定基 7 自由度臂**(A-L1-GAMMA) | 桌面固定基座机械臂,学生实验用 | 7 | STUDENT_GUIDE、7DOF_ARM_NMPC_README |
| **B. LL100-LH420 空中操作臂** | 浮动基座(整机含机体+旋翼)上的六轴机械臂 | 6 | LL100_LH420_MPC_README、Solver README |

> **自动调参**(MPC_TUNING_README)与 **NMPC 核心求解器**(mpc_pinocchio_ik.py)两套系统**共用**。

---

## 二、文档地图

| 文档 | 面向谁 | 讲什么 | 什么时候看 |
|---|---|---|---|
| **[STUDENT_GUIDE.md](STUDENT_GUIDE.md)** | 学生 / 新手 | 启动仿真/真机、ROS 2 话题控制关节、IK 控末端 | **第一次上手**(系统 A) |
| **[7DOF_ARM_NMPC_README.md](7DOF_ARM_NMPC_README.md)** | 用 7 自由度臂做 NMPC 的人 | 固定基 7 自由度臂的 ACADOS NMPC IK 运行说明 | 跑/改系统 A 的 NMPC demo |
| **[LL100_LH420_MPC_README.md](LL100_LH420_MPC_README.md)** | 用 LL100 的人 | LL100-LH420 六轴 NMPC demo:启动、避障、轨迹、常见问题 | 跑/改系统 B 的 NMPC demo |
| **[MPC_TUNING_README.md](MPC_TUNING_README.md)** | 要调参的人 | 两套离线闭环自动调参器(系统 A + 系统 B),用法/参数/结果解读 | 想自动搜一套好参数时 |
| **[LL100_LH420_NMPC_Solver_README.md](LL100_LH420_NMPC_Solver_README.md)** | 要选/比/换求解器的人 | 求解器安装部署、对比程序、筛选规则与结论、增删求解器教程、demo 里切换求解器 | 关心"用哪个求解器/怎么比"时 |

---

## 三、按"我想做什么"导航

- **我要在新机器上从零部署(一般电脑 / Jetson)** → [六、部署步骤](#六部署步骤一般电脑--jetson)
- **我第一次用机械臂,想让它动起来** → [STUDENT_GUIDE.md](STUDENT_GUIDE.md)
- **我要跑固定基 7 自由度臂的 NMPC** → [7DOF_ARM_NMPC_README.md](7DOF_ARM_NMPC_README.md)
- **我要跑 LL100 的 NMPC demo(避障/画圆/贴桨盘)** → [LL100_LH420_MPC_README.md](LL100_LH420_MPC_README.md)
- **我的跟踪误差大/想自动找好参数** → [MPC_TUNING_README.md](MPC_TUNING_README.md)
- **我想知道用哪个求解器最好 / 怎么加一个新求解器 / 怎么在 demo 里换求解器** → [LL100_LH420_NMPC_Solver_README.md](LL100_LH420_NMPC_Solver_README.md)
- **acados 装不上 / 缓存问题** → 各 NMPC README 的"环境准备/缓存"节 + Solver README §1

---

## 四、文档间关系(依赖 / 引用)

```
                    ┌──────────────────────┐
                    │  STUDENT_GUIDE.md    │  入门(系统 A:话题/IK)
                    └──────────┬───────────┘
                               │ 进阶到 NMPC
              系统 A ┌─────────┴─────────┐ 系统 B
   ┌───────────────────────────┐   ┌───────────────────────────┐
   │ 7DOF_ARM_NMPC_README.md   │   │ LL100_LH420_MPC_README.md │  (NMPC demo 说明)
   └─────────────┬─────────────┘   └───────┬───────────────────┘
                 │  两系统共用            │  └── 求解器选型引用 ──┐
                 ▼                        ▼                      ▼
        ┌────────────────────┐   ┌──────────────────────────────────────┐
        │ MPC_TUNING_README  │   │ LL100_LH420_NMPC_Solver_README.md    │
        │ (自动调参,两系统) │   │ (求解器对比/选型,单一事实源)        │
        └────────────────────┘   └──────────────────────────────────────┘
```

- **调参 vs 求解器**:调参(MPC_TUNING)是"固定求解器,搜代价权重等参数";求解器对比(Solver README)是"固定参数,比不同求解器"。两者正交、互补。
- **求解器结论的单一事实源**是 [LL100_LH420_NMPC_Solver_README.md](LL100_LH420_NMPC_Solver_README.md);
  LL100 与调参两个 README 里凡涉及"用哪个后端/求解器",都指回它。
- **NMPC 核心代码** `src/arm-platform/demo/mpc_pinocchio_ik.py`(`MpcPinocchioIK`)被上述所有 NMPC/调参/对比复用。

---

## 五、建议阅读顺序

0. **新机器先部署**:[六、部署步骤](#六部署步骤一般电脑--jetson)(区分一般电脑 / Jetson)。
1. **新手**:STUDENT_GUIDE → (选系统)7DOF_ARM_NMPC_README 或 LL100_LH420_MPC_README。
2. **要精度**:上面之后 → MPC_TUNING_README(调参)。
3. **要选/换求解器**:LL100_LH420_NMPC_Solver_README。

---

## 六、部署步骤(一般电脑 / Jetson)

> 本章是**唯一的部署入口**。各分册 README 不再重复部署步骤,统一指回这里。
> - **一般电脑** = x86_64 台式/笔记本,Ubuntu 22.04 + ROS 2 Humble。
> - **Jetson** = NVIDIA Jetson(ARM64/aarch64,如 Orin NX,JetPack 6.x / Ubuntu 22.04)。
>
> 两类主体步骤相同,**只有 acados 编译和求解器取舍不同**;Jetson 的差异一律用 ⚠️ 标出。

### 6.0 通用前提

- 操作系统 Ubuntu 22.04,已装 ROS 2 Humble(`source /opt/ros/humble/setup.bash` 可用)。
- 克隆到**固定路径** `~/code/windylab_ws`(脚本与文档默认此路径,别改):
  ```bash
  mkdir -p ~/code && git clone <本仓库地址> ~/code/windylab_ws
  ```
- ⚠️ Jetson 用户名通常是 `nvidia` 而非 `windylab`,命令里的 `~` 会自动展开成 `/home/nvidia`,没问题;**唯独**作图程序 `data_dir` 默认写死了 `/home/windylab/...`,Jetson 上跑作图时加参数覆盖:
  `-p data_dir:=$HOME/code/windylab_ws/src/arm-platform/demo/data`。

### 6.1 基础依赖(两类相同)

```bash
sudo apt update
sudo apt install -y \
  git cmake build-essential python3-pip pkg-config \
  ros-humble-pinocchio ros-humble-robot-state-publisher ros-humble-rviz2 \
  libeigen3-dev
python3 -m pip install --user numpy casadi
```

验证 Eigen ≥ 3.4(GRAMPC 需要):`pkg-config --modversion eigen3`。

### 6.2 编译工作空间(两类相同)

```bash
cd ~/code/windylab_ws/src
[ -d serial ] || git clone https://github.com/ZhaoXiangBox/serial.git serial

cd ~/code/windylab_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select serial dummy_interface manipulator
source install/setup.bash
```

### 6.3 安装 acados(主力求解器)—— **这里区分两类**

**一般电脑(x86_64):**

```bash
cd ~/code/windylab_ws/src/arm-platform
scripts/install_acados.sh
source scripts/setup_acados_env.sh
python3 -c "import acados_template; print('acados ok')"
```

**⚠️ Jetson(ARM64):必须给 blasfeo 指定 ARM target**,否则默认按 x86 编,会编译失败或结果不对。安装脚本已支持用 `ACADOS_CMAKE_ARGS` 传入:

```bash
cd ~/code/windylab_ws/src/arm-platform
export ACADOS_CMAKE_ARGS="-DBLASFEO_TARGET=ARMV8A_ARM_CORTEX_A57"
scripts/install_acados.sh
source scripts/setup_acados_env.sh
python3 -c "import acados_template; print('acados ok')"
```

> Orin 是 Cortex-A78,blasfeo 无 A78 专项 target,用 `ARMV8A_ARM_CORTEX_A57` 覆盖全 ARMv8-A 即可;若编译报指令不支持,退到 `-DBLASFEO_TARGET=GENERIC`。首次编译在 Jetson 上明显更慢,耐心等。

### 6.4 安装 GRAMPC

本仓库只部署三类求解器:acados + HPIPM、acados + qpOASES、GRAMPC。GRAMPC 通过
`pygrampc` 源码树提供头文件,并由 demo 首次运行时自动生成/编译 LL100 问题模块。

```bash
cd ~/code/windylab_ws/src/arm-platform
scripts/install_grampc.sh
source scripts/setup_acados_env.sh
python3 -c "import pygrampc; print('pygrampc ok')"
```

### 6.5 环境自检 + 每次运行前激活

一次性自检(只读,不改系统):

```bash
cd ~/code/windylab_ws/src/arm-platform
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
source scripts/setup_acados_env.sh
scripts/check_nmpc_env.sh
```

**每个新终端**跑 demo / 对比前,先激活环境:

```bash
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
export PYGRAMPC_ROOT=~/code/pygrampc      # 用到 GRAMPC 时
```

### 6.6 ⚠️ Jetson 专属注意

1. **native 产物不能从 x86 搬**:别把 x86 机器上的 `build/`、`install/`、`~/code/acados/lib/`、acados 的 `c_generated_code/`、GRAMPC 的 `grampc_problem/*.so` 拷过来——架构不符。一律在 Jetson 上重新编。
2. **acados target**:见 6.3(最常见的坑)。
3. **编译慢**:acados、GRAMPC、colcon 首次编译比 x86 慢很多;acados 首次跑某配置还会现场生成 + 编译 C 求解器,属正常。
4. **路径**:见 6.0 的 `data_dir` 提醒。

### 6.7 部署完成后去哪

- 跑 LL100 demo(避障/画圆/贴桨盘) → [LL100_LH420_MPC_README.md](LL100_LH420_MPC_README.md)
- 跑固定基 7 自由度臂 demo → [7DOF_ARM_NMPC_README.md](7DOF_ARM_NMPC_README.md)
- 跑求解器对比 / 选型 → [LL100_LH420_NMPC_Solver_README.md](LL100_LH420_NMPC_Solver_README.md)
- 自动调参 → [MPC_TUNING_README.md](MPC_TUNING_README.md)
