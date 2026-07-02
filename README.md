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

1. **新手**:STUDENT_GUIDE → (选系统)7DOF_ARM_NMPC_README 或 LL100_LH420_MPC_README。
2. **要精度**:上面之后 → MPC_TUNING_README(调参)。
3. **要选/换求解器**:LL100_LH420_NMPC_Solver_README。
