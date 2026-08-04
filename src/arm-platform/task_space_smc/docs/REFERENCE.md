# 参考手册：设计、标定、实测特性与排错

本文件是 [README](../README.md) 的深入篇。README 只讲**怎么跑**（目录结构、部署、
仿真启动、实物启动）；本文件讲**为什么这么设计、实测到了什么、出了问题怎么查**。

⚠️ 章节编号沿用 2026-08-04 拆分前的原编号，文内交叉引用（如 [8.4](#84-读哪个姿态误差指标)）
   因此保持有效。编号不连续是拆分的结果，不是缺页。

---

## 1. 这个模块是什么

一条**独立于现有 NMPC 的**控制链，不复用也不修改 NMPC 代码：

- 用 Pinocchio `JointModelFreeFlyer` 建浮动基座机械臂模型
- 世界系末端位姿轨迹跟踪
- 任务空间拉格朗日滑模控制
- 严格四元数 / SO(3) 对数姿态误差
- 基座运动补偿
- 动态一致广义逆 + 零空间投影
- 完整自由基座 RNEA 输出关节力矩

> **数学定位**：这是"完整自由基座建模 + 规定基座运动下的机械臂约化控制"。基座动力学行
> `M_BB a_B + M_Ba q̈ + h_B = F_B` **没有求解**，基座轨迹是外部规定量，机械臂反作用不会
> 反过来改变基座。`whole` 只是绘图轨迹的名称，不代表已实现无人机-机械臂联合 whole-body 控制。

### 1.1 控制律结构

```
参考轨迹 ──► 位姿误差 e, ė ──► 滑模面 s = ė + Λe
                                   │
                                   ▼
                        ẍ_des = ẍ_r + Λė + K·s + η·tanh(s/φ)
                                   │
                                   ▼
              q̈ = J̄·(ẍ_des − J̇q̇) + N·q̈_null      ← redundancy_resolution.py
                                   │
                                   ▼
              τ = RNEA(q, q̇, q̈)                    ← 完整自由基座
                                   │
                                   ▼
              限幅 → 0x12 协议帧                     ← actuator_adapter.py
```

`J̄ = M⁻¹Jᵀ(JM⁻¹Jᵀ)⁻¹` 是动态一致广义逆，`N = I − J̄₀J` 是零空间投影。

> **`J̄` 与 `N` 的优化含义**：这两者不是启发式构造，而是下述等式约束二次规划的闭式解：
> ```
> min_q̈  ½(q̈ − q̈₀)ᵀ M (q̈ − q̈₀)     s.t.  J q̈ = ẍ_des − J̇q̇
> ```
> 解即 `q̈ = J̄b + N q̈₀`。加权矩阵取质量矩阵 `M` 就是"动态一致"的来源（Gauss 最小拘束
> 原理下的加速度能量最小）。
>
> 理解这一点对判断能力边界很重要：**它只能处理等式约束**。关节限位、力矩上限、速度包线
> 这类**不等式**在这个框架里表达不了，只能退化成软势场或事后限幅 —— 而事后限幅会破坏
> 已解出的最优解（见 [13.2](#132-已知的结构性限制)）。

> ⚠️ **投影必须用无阻尼的广义逆构造。** `N = I − J̄·J` 只有当 `J̄` 是伪逆时才是投影算子；
> 用**阻尼**逆构造时 `N² ≠ N`，零空间运动会**漏进任务空间**。实测泄漏在近奇异处从
> 3.5e-09 涨到 2.4e-03（6 个数量级），触发正反馈：σ 下滑 → 阻尼升 → 泄漏涨 → 被当作任务
> 扰动 → 六轴力矩饱和 → 位置误差 41 mm。
> 现用截断伪逆 `_undamped_inverse()` 专供投影，任务项仍用阻尼逆（奇异保护不变），
> 泄漏回到 3.3e-14。回归测试 `test/test_nullspace_projector_is_exact.py`。

### 1.2 源文件导航

| 文件 | 职责 |
|---|---|
| `model.py` | `FloatingBaseArmModel`：URDF 加载、FK/雅可比/CRBA/RNEA、标定值注入 |
| `task_space_smc.py` | 滑模控制律本体 |
| `redundancy_resolution.py` | 广义逆、零空间投影、5D 笔轴降维、限位规避、可行性盒 |
| `pose_error.py` / `quaternion_math.py` | SO(3) 对数姿态误差、四元数运算 |
| `trajectory_reference.py` | 参考轨迹采样，底层几何在 `demo/moving_base_trajectory_config.py` |
| `base_state.py` | 基座状态估计与延迟/噪声注入 |
| `actuator_adapter.py` | 力矩限幅、速率限制、单位换算、超时兜底、急停 |
| `whole_task_smc_ros_runner.py` | ROS 节点：订阅反馈、跑控制律、发布力矩与诊断、写 CSV/JSON/PNG |
| `offline_simulation.py` | 不依赖 ROS 的自检入口 + 控制器装配工厂 |
| `torque_controller_node.py` | 独立力矩控制节点（7 关节） |
| `pose_check_readonly.py` | **只读**位形核对，直接读串口不经 ROS，见 [7.4](../README.md#41-上机前检查) |
| `serial_tap.py` | **双向**串口抓包（下行也能抓），含假板子模式，见 [7.5](#75-双向抓包-serial_tappy) |
| `phase2..phase5_*.py` | 实机标定脚本，见第 9 节 |
| `tune_whole_task_smc.py` | 自动增益扫描（⚠️ 离线，见 [11.1](#111-调参规则)） |
| `plot_whole_task_smc.py` | 轨迹报告绘图 |

---

## 2. 三种机械臂构型

| | 7 关节 | 6 关节 | **6 关节 + 笔架** |
|---|---|---|---|
| URDF | `config/arm.urdf` | `config/arm6.urdf` | `config/arm6_tool.urdf` |
| 末端 frame | `link7` | `link6` | `link7`（笔尖） |
| 驱动关节 | 7 | 6 | 6（`joint7` 是 `fixed`） |
| `nq` / `nv` | 14 / 13 | 13 / 12 | 13 / 12 |
| 惯性参数 | CAD 名义值 | **已按实机标定修正** | **已按实机标定修正** |
| 任务维度 | 6D | 5D | 5D |
| 工作面布局 | 竖直板 | 水平面 | 水平面 |
| 用途 | 历史基线 | 过渡，已被右列取代 | **实机构型，当前主线** |

> **实物只有 6 个关节。** 末端没有电机，`joint7` 物理不存在。但固件仍按 7 通道上报，
> 第 7 通道是幽灵数据（见 [10.4](#104-第-7-通道是幽灵数据)）。屏蔽靠 `arm_version:=gamma6`。

### 2.1 为什么要加笔架

末端取 `link6` 原点时，它正落在腕转轴上：**转 `q6` 完全不移动末端**，只贡献 5D 任务的
2 行姿态行，位置行拿不到它，近奇异时掉秩。取笔尖 `link7`（外移 91.00 mm）后 `q6` 有力臂，
位置行也贡献，退化消失。

沿轨迹 100 点实测：

| | 位置可达 | 姿态可达 | σ 最差 |
|---|---|---|---|
| 无笔架，6D | 99 % | **43 %** | 0.00104 |
| 无笔架，5D | 100 % | 87 % | 0.00816 |
| **有笔架，5D** | **100 %** | **100 %** | **0.01942** |

闭环 RMS 从 10.35 mm 降到 3.95 mm。**写字本来就要装笔，这个工具无论如何都要有** ——
它的几何不是负担，是条件数的来源。

`arm6_tool.urdf` 由 `arm.urdf` 把 `joint7` 从 `revolute` 改成 `fixed` 生成，
偏移 `(0.084, 0, −0.035)`。笔架 `link7` 质量 0.060 kg，惯量会正确进入动力学。

> ⚠️ 标定是在**没装笔架**时做的，那 0.060 kg 当时不存在。装上真实笔架后应复测一阶质量矩。

---

---

## 4. 离线仿真

```bash
export PYTHONPATH="$PWD/src/arm-platform:$PYTHONPATH"
python3 src/arm-platform/task_space_smc/sim/offline_simulation.py \
  --profile sixdof --reference-mode hold --duration 1.0
```

| 参数 | 可选值 | 说明 |
|---|---|---|
| `--profile` | `none` / `translation` / `rotation` / `sixdof` | 注入的基座运动类型 |
| `--reference-mode` | `hold` / `circle` | 末端参考轨迹 |
| `--duration` | 秒，默认 `1.0` | 仿真时长 |
| `--output-dir` | 默认 `/tmp/task_space_smc_offline` | 输出目录 |

输出含位置误差、姿态测地误差、滑模面、奇异值、阻尼、零空间泄漏、力矩饱和等诊断量。
5D 模式下另有 `pen_axis_error` 一列，见 [8.4](#84-读哪个姿态误差指标)。

> ⚠️ **离线结果只能用于"是否可行"这类判断，不得用来给参数排序。** 理由见 [11.1](#111-调参规则)。

---

---

## 7. 实机接口细节

> 上机的**操作步骤**在 [README §4](../README.md#4-实物启动)。这里只讲接口本身的机理，
> 以及绕过 SMC 直接操作底层时要知道的事。

### 7.3 直接力矩接口

```bash
ros2 run manipulator student_arm_node --ros-args \
  -p arm_type:=a_l1 -p arm_version:=gamma6 \
  -p enable_direct_effort_command:=true -p effort_command_timeout_sec:=0.05
```

`/student/joint_effort_command` 直达 `ExecuteDirectMotorCommand → DMMotor::UpdateCommand
→ ProtocolV1::SetCurrent → 0x12`。底层是 DM MIT 模式：
`τ = kp·(p_des − p) + kd·(v_des − v) + t_ff`，位置环与力矩前馈**同时生效**，
调 `kp` 可在"纯位置"和"纯力矩"之间连续过渡 —— 这是各标定阶段的共同手段。

> **上机前必读的三条安全约束：**
>
> 1. **超时兜底行为决定机械臂会不会垮下来。** 力矩模式若发 `kp=kd=current=0`，
>    在 MIT 模式下是彻底失力。兜底必须是**带默认增益的位置保持**。
> 2. **协议里没有使能/失能帧。** 软件层没有真正的急停通道，**急停只能靠物理断电**。
> 3. **单关节测试时其余关节保持位置锁定。** `DMMotor::UpdateCommand` 有个陷阱：
>    关节接近限位 0.05 rad 以内且发 `kp=0` 时，驱动层会**静默把 kp 改回默认值**并强制
>    位置保持，力矩指令被无声吞掉。

### 7.5 双向抓包 `serial_tap.py`

`pose_check_readonly.py` 只能抓上行。**下行用被动读法原理上抓不到** —— UART 不会把 TX
回环到 RX，打开同一个 `/dev/ttyUSB0` 去 `read()` 只看得到板子发来的字节；何况串口被控制
节点独占。所以本工具走**伪终端透明中继**：节点开一个 PTY，工具把字节原样转发到真串口，
两个方向同时打时间戳落盘并在线解码。方向靠地址字节区分（`0xAA` 下行 / `0x55` 上行）。

```bash
python3 task_space_smc/hw/serial_tap.py --selftest                    # 10 项自检，无硬件
python3 task_space_smc/hw/serial_tap.py --fake-board --link /tmp/ttyFAKE --seconds 20
python3 task_space_smc/hw/serial_tap.py --port /dev/ttyUSB0 --link /tmp/ttyTAP \
        --seconds 60 --capture /tmp/tap.bin                        # 上机中继
python3 task_space_smc/hw/serial_tap.py --decode /tmp/tap.bin         # 离线复解
```

`--fake-board` 冒充控制板按 200 Hz 回放上行帧（含 20 Hz 心跳 —— 少了它节点会封锁力矩
前馈），**不碰实物、不用通电**就能让真正的 `student_arm_node` 接上来验证驱动层。

判读结论工具会直接给：

| 现象 | 结论 |
|---|---|
| 下行 `0x12` 缺失 / 帧率低于 `0x10` / 校验错 / 短写 | 驱动层 |
| 下行 `0x12` 正确，上行 `0x22` 不跟随 | 电机固件对 `kp=kd=0` 的处理 |
| 下行 `0x12` 正确，上行 `0x22` 跟随 | 力矩确实产生了，转查机械/摩擦 |
| **力矩恒 0 而 kp 恒非 0** | **指令被静默降级成位置保持**，见下 |

> ⚠️ **`MotorControl.enable = 0` 会被静默降级。** `StudentEffortCommandCallback` 在
> `enable == 0` 时把力矩指令换成带默认增益的位置保持并 return，**不打任何日志**；
> 上游 `hardware_enabled` 在 runner 与 launch **两处默认都是 false**。表象是控制器
> 日志里满幅力矩、线上一个字节力矩都没有。纯力矩模式的定义就是 `kp=kd=0`，
> 所以"力矩恒 0 而 kp 是默认增益"是这个降级的确定性指纹，工具会自动报出来。

> **与 `pose_check_readonly.py` 的安全边界不同：中继模式【会】向真串口发送字节**
> —— 它本来就是链路的一段。但它只转发不生成，自检 `test_relay_is_byte_exact`
> 用 2 万随机字节验证"进什么出什么"。

---

---

## 8. 任务定义：5D 笔轴投影

### 8.1 为什么降维

6 关节机械臂**没有球腕**：腕部只有 `joint5` 绕 z、`joint6` 绕 x。6D 位姿任务对它是过约束的。
绕笔轴的自转对写字没有意义 —— 笔转多少度不影响笔迹。剔除它把零空间秩从 0 提到 1，
那个自由度用于限位规避。

### 8.2 数学形式

设 `a` 是末端笔轴在世界系下的方向，`P = I − a·aᵀ`：

```
J₅ = [ J_pos(3行) ; B · J_rot(2行) ]
y₅ = [ y_pos       ; B · y_rot      ]
```

`B` 取 `P` 的两个非零奇异方向（`U[:, :2].T`）。

> **实现要点**：`P` 的秩是 2，直接把 `[J_pos; P·J_rot]` 堆成 6 行会得到一个**秩 5 的
> 奇异矩阵**。必须用 SVD 取基降成真正的 5 行。

### 8.3 笔轴是末端 x 轴

依据是**物理笔架的安装方向** —— 笔沿 `link7` 的 x 轴延伸（RViz 里的红轴），
所以要让笔垂直于纸面，就得让 x 轴垂直于纸面。

配置项 `task.pen_axis_local: [1.0, 0.0, 0.0]`。

约定分两套，由 `MB_STATION_LAYOUT` 决定（`_station_geometry` 的 `top` 站两种布局共用，
所以它也随之切换）：

| 布局 | 笔轴 | 用途 |
|---|---|---|
| `vertical` | 末端 **y** 轴 | 7 关节历史基线，沿用 `arm.urdf` 的原约定 |
| `horizontal` | 末端 **x** 轴 | **6 关节实机构型** |

> ⚠️ 取错的后果不是精度下降，而是**约束方向反了**：放开该锁死的笔尖指向、锁死无所谓的
> 绕笔自转。而且 `pen_axis_error()` 用的是同一个轴，**自己验自己不会报警**。
> 实测把它从 y 改成 x：`hwtest` RMS 0.0256 → **0.0041 mm**，
> `whole` RMS 2.42 → **0.061 mm（40 倍）**。
>
> 回归测试 `test/test_pen_axis_convention.py` 用笔画点云的 SVD 法向做独立基准，
> 并额外核对两份 6 关节配置一致、**代码默认值与配置一致**（防止配置漏写时静默用回旧约定）。

### 8.4 读哪个姿态误差指标

pen_axis 模式下，**完整 3D 姿态误差是误导性指标** —— 它包含故意不控制的绕笔轴自转，
那一项自由漂移，报出 0.5 rad 以上是正常的。要看的是 `pen_axis_error`。

### 8.5 为什么 6 关节用水平面而 7 关节用竖直板

竖直板要求腕部产生**水平**笔轴，而 6 关节没有球腕；7 关节多一个自由度不受此限。
站心附近 20 点实测：

| | 6 关节竖直板 | 6 关节水平面 | 7 关节竖直板 | 7 关节水平面 |
|---|---|---|---|---|
| 限位余量 | 0.126 | **0.47~0.52** | 达标 | — |
| σ_min | 0.0075 | **0.081~0.143** | 达标 | — |
| 闭环 RMS | — | 2.42 mm | 达标 | **353.9 mm，96.4% 力矩饱和** |

**同一几何对两种腕部结构好坏相反**，所以做成了可选：

```python
# demo/moving_base_trajectory_config.py
_STATION_LAYOUT = os.environ.get('MB_STATION_LAYOUT', 'vertical')
```

> ⚠️ **默认 `vertical` 不要改**。该文件被 7 关节通路、`pinocchio_ik`、MPC demo、visualizer
> 和 `demo/test_moving_base_trajectory_shapes.py` 共同 import。改默认值等于单方面废掉
> 7 关节基线。`horizontal` 由 6 关节 launch 用 `os.environ.setdefault` 显式声明。

### 8.6 零空间限位规避

```yaml
task:
  limit_avoidance:
    gain: 0.10
    activate_rad: 0.5
```

排斥梯度**只在距限位小于 `activate_rad` 时激活**，中段恒为 0。

> **全局拉向中位是错的**：那样零空间项会全程与任务项抢自由度，实测把位置可达率从
> 100 % 拉到 81 %。只在临近限位时激活则相反：触限次数 19 → 3，笔轴可达率 88 % → 100 %。

---

## 9. 实机标定体系

### 9.1 标定文件

所有实机标定值集中在 `config/arm6_calibration.yaml`。

| 字段 | 含义 | 如何生效 |
|---|---|---|
| `zero_offset_rad` | URDF 建模零位与编码器零位之差 | 仅标定脚本使用 |
| `first_moment_ratio` | 一阶质量矩实测/名义比值 | **已烘进 URDF**，见下 |
| `armature_kgm2` | 转子惯量经减速比反射的等效值 | `model.armature[6:]`，自动加载 |
| `stiction_nm` | 静摩擦（启动力矩） | 死区估计、精度预算 |
| `kinetic_friction_nm` | 动摩擦 | 加速阶段补偿 |
| `inertia_kgm2` | 有效惯量（含转子） | 校验用 |
| `torque_scale_alpha` | `0x12` 命令标度 | 力矩命令换算 |

> ⚠️ **`first_moment_ratio` 已烘进 `arm6.urdf` 和 `arm6_tool.urdf` 的连杆惯性参数**
> （质量标度 `[1.005, 1.336, 1.418, 1.220, 1.0, 1.361]`）。这两个 URDF 算出的重力
> **就是**实机重力，**不要再乘一次** —— 曾在测试里出现，把 2.83 Nm 夸大成 4.09 Nm。
> `config/arm.urdf`（7 关节）**未修正**，对它这个系数仍然适用。

> ⚠️ **改了连杆惯量必须同步重算 armature**，使 `M_连杆 + armature = 实测总惯量`。
> 连杆变重则 armature 必须变小，否则总惯量被重复计入一次。

> **静摩擦与动摩擦不能混用**，两者相差 0.8~32 倍（Stribeck）。起步用静摩擦，加速用动摩擦。

> **`armature` 占总惯量的 60~95 %。** URDF 只有连杆惯量，实测有效惯量是它的 2.5~20.5 倍
> （joint4 的 98 % 来自转子）。不加它质量矩阵会小一个数量级，动力学前馈几乎不起作用。

### 9.2 标定失效条件

以下任一情况发生后**必须重测**：

- 拆装关节、更换电机、调整预紧
- 基座**倾角**改变（纯平移不影响 —— 重力是均匀场）
- 手工掰动关节
- **装上或更换笔架**（标定是在无笔架时做的）

### 9.3 五个阶段

| 阶段 | 脚本 | 解决什么 | 指标 |
|---|---|---|---|
| 2 | `phase2_torque_probe.py` | `0x12` 通路是否通、标度多少；各关节静摩擦 | 标度 α 及不确定度 |
| 3 | `phase3_gravity_calibration.py` | 零位偏差、一阶质量矩 | 正弦拟合的相位与幅值 |
| 4 | `phase4_gravity_compensation.py` | 整臂重力前馈是否托得住 | 分级降 kp 到 0 仍不下坠 |
| 5 | `phase5_inertia_id.py` | 有效惯量与动摩擦 | `qdd` 对 `τ` 的线性度 R² |

阶段 2/3/4 已注册为 ROS 可执行：

```bash
ros2 run manipulator phase2_torque_probe.py --help
```

阶段 5 与 `pose_check_readonly.py` **未注册**到 `install(PROGRAMS)`，用解释器直接跑：

```bash
export PYTHONPATH="$PWD/src/arm-platform:$PYTHONPATH"
python3 src/arm-platform/task_space_smc/hw/phase5_inertia_id.py --help
```

### 9.4 方法学要点

**斜率法免疫摩擦。** 阶段 2 测标度和阶段 5 测惯量是同一手法 —— 摩擦进截距不进斜率：

```
阶段 2:  d(Δq)/d(t_ff) = α / kp
阶段 5:  d(qdd)/d(τ)   = 1 / M
```

**相位精度由拟合条件数决定，不是死区/√N。** 扫描区间覆盖的周期比例太小会使幅值与相位
强耦合 —— 实测用 0.8 rad 区间时 `ratio` 错了 59 %。

**`resid/A < 0.05` 只在 `A/f > 5` 时才指示几何错误。** 重力信号幅值与静摩擦同量级的关节
（本机 joint4/5），残差比 0.29~0.35 是摩擦主导的正常现象。

**判断"关节不动"的三种原因：**

| 现象 | 原因 |
|---|---|
| `0x22` 力矩跟随命令变化，位置不动 | 静摩擦未克服 |
| `0x22` 力矩锁定在某个常数 | 通道死了 |
| 跟踪误差等于命令、力矩 ≈ 0 | 节点模式不对 |

详见 [PHASE1_FINDINGS.md](PHASE1_FINDINGS.md)。

---

## 10. 真机接口实测特性

数据来自 `/dev/ttyUSB0 @ 921600` 的被动抓包（只读），2480 帧**零校验错误**。

### 10.1 帧格式

```
0xFE 0xAA | cmd | len | payload | sumCheck | addCheck    PC → 板
0xFE 0x55 | cmd | len | payload | sumCheck | addCheck    板 → PC
```

| 方向 | cmd | 内容 |
|---|---|---|
| 下行 | `0x10`/`0x11`/`0x12`/`0x13`/`0x14` | position / velocity / **力矩** / kp / kd，各 7×float32 |
| 上行 | `0x20`/`0x21`/`0x22`/`0x23`/`0x24` | position / velocity / current / voltage / temperature |
| 上行 | `0x01` | 心跳 20 Hz |
| 上行 | `0x25` | 未知，2 Hz，payload `[1,1,1,0,0,0,0]`。**含义未确认** |

`sumCheck` 是累加和，`addCheck` 是累加和的累加和，都取低 8 位。

### 10.2 上报速率 200 Hz

```
0x20/0x21/0x22/0x24 : 200.6 Hz     0x23 voltage : 2.0 Hz（全通道恒为 0，固件未实现）
0x01 heartbeat      :  20.0 Hz
```

上行带宽只用了 26.8 %，所以**不是带宽限制，是固件把速率定死在 200 Hz**。

### 10.3 `0x12` / `0x22` 的单位是 Nm

由量化步长反解，四条全部落在干净整数上：

| 量 | 实测 LSB | 反解 | 比值 |
|---|---|---|---|
| position | 3.833771e−04 | 16 bit / ±4π rad | 0.99969 |
| velocity | 2.441788e−02 | 12 bit / ±50 rad/s | 1.00016 |
| **力矩 J1–3** | 1.367378e−02 | 12 bit / **±28.00 Nm** | 0.999996 |
| **力矩 J4–6** | 2.930164e−03 | 12 bit / **±6.00 Nm** | 1.00016 |

两种电机的标度**不同**且分别对应各自峰值力矩。若单位是安培，没有理由让两个不同品牌电机的
量程恰好落在各自峰值力矩附近。

```
硬件饱和上限:  joint1-3 ±28 Nm    joint4-6 ±6 Nm
额定（连续热）: joint1-3  ±9 Nm    joint4-6 ±1.6 Nm
```

命令方向 `0x12` 的标度由阶段 2 实测确认，六关节合并 **α = 1.033 ± 0.024**。

### 10.4 第 7 通道是幽灵数据

| 流 | joint1–6 范围 | 通道 7 |
|---|---|---|
| position | −1.381 … 1.324 | **−6.5231 rad**（超出 ±4 限位） |
| velocity | −0.208 … 0.159 | **+27.9731 rad/s**（超 RS05 上限 5.6 倍） |
| temperature | 27 … 31 °C | **1** |

温度 = 1 是铁证。屏蔽靠 `arm_version:=gamma6`。回归测试 `test/test_arm6_config.cc`。

### 10.5 传感器分辨率

编码器 LSB = 3.834e−4 rad，折算到末端约 **0.13 mm**（六关节 RSS）。

| | 数值 |
|---|---|
| 仿真指标 | RMS < 1 mm，peak < 2 mm |
| 实机放宽指标 | 1 cm |
| **传感器分辨率下限** | **≈ 0.13 mm** |

**任何低于 0.13 mm 的仿真结果物理上无法在真机复现**，只能作为算法自洽性检查。

速度 LSB = 0.0244 rad/s 相当粗，折到末端约 10 mm/s。滑模面 `s = ė + Λe` 直接吃速度，
边界层厚度 `phi` 必须大于该噪声底。

### 10.6 电机包线尚未实测

`config/motor_config.yaml` 的 `max_velocity`（`[3,3,3,5,5,5]` rad/s）是**厂商标称值**，
未台架实测。**转矩-转速特性完全没有** —— 当前模型假设力矩上限与转速无关，而真实 BLDC
在高速时可用力矩会下降。上机前建议单独标定（可沿用 [9.4](#94-方法学要点) 的斜率法）。

---

## 11. 参数整定

### 11.1 调参规则

> **调参与验证必须跑真实 ROS 节点链**（`rviz:=false` 可关 GUI）。
> **离线仿真只能用于"是否可行"这类判断，不得用离线数值给参数排序。**

这条规则是有代价换来的：曾用离线闭环 harness 跑 240 次 Optuna 搜索，最优报
**RMS 0.168 mm 且完全确定性**；同一份配置在真实 ROS 里跑出 **116 mm**。
机制是优化器找到了 harness 的保真度缺口并加以利用。

更早还有一个被忽略的信号：同一 harness 与 ROS 的**分窗 RMS 秩相关只有 0.475**。
结构性指标（触限 24.5 % vs 23.9 %）能对上，但那不足以支撑参数排序 —— 而排序正是调参
唯一需要的东西。

⚠️ `tune_whole_task_smc.py` 是**离线**扫描器，受此规则约束。

### 11.2 真实 ROS 的噪声底

同一配置重复 8 次实测：均值 2.42 mm、标准差 0.668 mm、**变异系数 27.6 %**。
**分辨不出比噪声小的差异** —— 调参前先量噪声底，否则会调到噪声上。

并行靠 `ROS_DOMAIN_ID` 隔离，每个 worker 一个域，DDS 不串扰。单次约 82 秒。

### 11.3 手动整定顺序

1. 先把 `eta` 置零，验证模型维度、雅可比分块和 RNEA
2. 用小的 `position.lambda` 和 `orientation.lambda`
3. 增大 `k` 直到滑模面平滑收敛
4. **只有在扰动和模型误差被表征之后**才增大 `eta`
5. 调 `phi`（下限受速度量化噪声约束，见 [10.5](#105-传感器分辨率)）
6. 任务跟踪稳定后再启用零空间项
7. 最后测试奇异位形附近和力矩饱和

| 参数 | 含义 |
|---|---|
| `lambda` | 沿滑模流形的收敛速度 |
| `k` | 状态回到滑模流形的速度 |
| `eta` | 鲁棒匹配扰动项 |
| `phi` | tanh 平滑过渡宽度 |

> 理想模型下平滑 SMC 律仍可收敛到零。存在扰动、饱和、噪声或延迟时，实际保证通常是
> **一致最终有界（UUB）**。

### 11.4 增益与惯量的耦合

**改了 `armature` 或惯量标定值，增益必须重新扫描。** 实测惯量（含转子反射项）比 URDF 大
2.5~20 倍，同样的加速度需求就要 2.5~20 倍力矩。未缩放的增益下力矩饱和率达 50.2 %。

### 11.5 不要靠放松物理限制取巧

放松电机速度上限的真实 ROS 剂量-响应实验：

| `max_velocity` | RMS | 备注 |
|---|---|---|
| ×0.5 | 11.13 mm | 更差 |
| ×1.0 | 2.49 mm | 基线 |
| ×1.5 | 2.51 mm | 与基线统计不可分 |
| ×3 | 3.01 mm | 峰值 41→79.8 mm，**安静时段误差地板从 0.0196 塌到 0.188 mm** |
| ×30 | 49.99 mm | 发散 |

那个 0.02 mm 的误差地板正是 1 mm 目标所依赖的东西。力矩上限同理：`tau_scale=2` 时
RMS 反而 2.018 → 4.949，速度夹翻倍。

---

## 12. 测试

```bash
cd /home/windylab/code/windylab_ws
python3 -m pytest -q src/arm-platform/test        # 161 passed

./build/manipulator/test_torque_sim_arm           # 1
./build/manipulator/test_arm6_config              # 4
./build/manipulator/test_protocol_heartbeat       # 6
./build/manipulator/test_effort_safety            # 6
```

几个守住关键不变量的测试：

| 测试 | 守什么 |
|---|---|
| `test_nullspace_projector_is_exact.py` | `N² = N` 且泄漏在机器精度量级，**即使重阻尼** |
| `test_pen_axis_convention.py` | 笔轴约定与轨迹几何一致（用点云 SVD 法向做独立基准） |
| `test_feasibility_scaling.py` | 可行性盒与饱和感知解算（对应实现**当前停用**，见 [13.2](#132-已知的结构性限制)） |
| `test_arm6_model.py` | 6 关节模型维度、标定值加载、armature 注入 |
| `test_dynamic_consistent_inverse.py` | 广义逆的动态一致性、5D/6D 任务维度自适应 |
| `test_arm6_config.cc` | `gamma6` 只创建 6 个电机，不读第 7 通道 |
| `test_effort_safety.cc` | 力矩模式的超时兜底、限幅、急停 |

> ⚠️ **单元测试全绿不等于闭环稳定。** 实施"饱和感知解算"时 161 个测试全过，真实 ROS
> 却跑出 245.7 mm（对照 2.43 mm）。单元测试只验**孤立时刻**的约束满足，验不出闭环稳定性。
> 这就是 [11.1](#111-调参规则) 那条规则的由来。

> 全包 `colcon test` **不是绿色**：仓库现有 lint 会递归扫描 `dist/`、`build/` 等生成目录，
> 产生数千条既有问题。不要为本模块批量格式化整个仓库。

---

## 13. 当前能力边界

区分"已验证"和"未验证"，避免把仿真数字当作真机保证。

| 项目 | 状态 |
|---|---|
| 6 关节 `hwtest` 轨迹，ROS 仿真 | ✅ **RMS 0.0041 mm / 峰值 0.029 mm** |
| 6 关节模型、标定、armature 注入 | ✅ 已验证 |
| 6 关节 `0x12` 力矩通路（实机） | ✅ α = 1.033 ± 0.024 |
| 6 关节整臂重力补偿（实机） | ✅ kp = 0 下不下坠 |
| 6 关节惯量辨识（实机） | ✅ joint1–5（R² 0.981~0.9997）；**joint6 未辨识** |
| 6 关节 `whole` 任务，ROS 仿真 | ✅ **RMS 0.061 mm / 峰值 1.13 mm** |
| 6 关节任意轨迹，实机 | ❌ 未运行 |
| 7 关节 `whole` 任务，ROS 仿真 | ⚠️ **基线已失效**，见 13.3 |

### 13.1 两条轨迹都已达标

| 轨迹 | RMS | 峰值 | 指标 |
|---|---|---|---|
| `hwtest` | 0.0041 mm | 0.029 mm | RMS<1mm / peak<2mm ✅ |
| `whole` | 0.061 mm | 1.13 mm | 同上 ✅ |

`whole` 此前长期卡在 **2.42 mm**，误差 >1 mm 的时间只占 2.68 % 却贡献绝大部分 RMS。
当时定位到的根因链是"基座补偿的关节速度代价随位形爆炸 → 撞速度硬夹 → 力矩饱和 → 41 mm 尖峰"。

后来发现**上游是笔轴约定错了**（见 [8.3](#83-笔轴是末端-x-轴)）：约束的是错误的旋转自由度，
机械臂被逼进本不必要的位形，补偿代价才高 6~40 倍。改成末端 x 轴后 RMS 直接降到 0.061 mm，
那条链的诱因随之消失。

**仍然有效的排除结论**（都做过真实 ROS 反事实，不要重做）：放松速度上限、放松力矩上限、
TOPP 时间参数化、分段边界平滑、零空间参数调整 —— 五条全部无效或更差。详见
[HANDOFF_2026_08_02.md](HANDOFF_2026_08_02.md) 第 6.4 节。

### 13.2 已知的结构性限制

**① 事后限幅破坏已解出的最优解。** `actuator_adapter.py` 在 RNEA 之后对力矩逐分量
`np.clip`。多轴同时饱和时，实际执行的加速度 `M⁻¹(τ_clip − h)` 在 `M` 度量下不是任何意义上
的投影，与任务加速度已无关系。

**② 局部方法跨不过解分支。** 自运动流形可能有多个不连通分支，微分求解只能在起步位形所在
的分支上滑动。独立的多起点 IK 显示解存在，但微分控制器到不了。

**③ 饱和感知解算（SNS）已实现但停用。** `FeasibilityLimits` / `_saturation_aware_task` /
`_feasible_acceleration_box` 保留在 `redundancy_resolution.py`，runner 里设了
`feasibility=None`。原因：单元测试全绿但真实 ROS 坏 100 倍。**重新启用前必须先在真实 ROS
上验证。**

### 13.3 7 关节基线已失效

它此前"已验证达标"的数字是在改 `torque_sim_arm.cc` 限幅逻辑（额定→峰值）和 armature
跳过逻辑**之前**测的，现在数字未知，**需要重新存档**。

另外它的配置 `smc_params_whole.yaml` 仍写着 500 Hz，而实机固件只有 200 Hz。

---

## 14. 常见问题

**Q: 改了 `task_space_smc/` 下的 Python 文件，行为没变化？**
`install(DIRECTORY ...)` 装的是副本不是符号链接。重新 `colcon build --packages-select manipulator`。
用 [3.1](../README.md#23-什么时候必须重编) 的 `diff` 确认。

**Q: 启动 6 关节 launch 后 `/joint_states` 里出现了 `joint7`？**
`arm_version` 没生效。确认用的是 `student_arm6.launch.py`（硬编码 `arm_version=gamma6`）。

**Q: 节点崩溃，提示 `Control cycle dt must be greater than 0.01`？**
位置模式下 `control_rate_hz` 超过 100。降到 `100.0` 或更低。

**Q: 发了力矩指令但机械臂完全不动？**
五个可能，**按实际踩到的频次排序**：

① **`MotorControl.enable` 是 0** —— `StudentEffortCommandCallback` 会把它**静默**换成带
默认增益的位置保持，不打任何日志。上游 `hardware_enabled` 在 runner 与 launch **两处默认
都是 false**，不显式传 `hardware_enabled:=true` 就会发 `enable=0`。用
[7.5](#75-双向抓包-serial_tappy) 抓一下：力矩恒 0 而 kp 是默认增益，就是它。
② `enable_direct_effort_command` 没开。
③ `arm_type` 用了 `a_l1_gamma` 而不是 `a_l1` —— 前者的 `Init()` 从不调用
`SetRateTorque()`，`SetCurrent` 会把所有力矩**静默归零**。
④ 关节离限位不足 0.05 rad，驱动层把 `kp` 改回默认值并强制位置保持。
⑤ 静摩擦未克服 —— 查 `0x22` 是否跟随命令变化，见 [9.4](#94-方法学要点)。

前四条都可以**不碰实物**用 `serial_tap.py --fake-board` 当场判定。

**Q: 位置模式下某个关节死活到不了目标？**
多半是 joint2。它静摩擦 1.775 Nm（其余关节的 2.4~18 倍）且承载最重，位置误差乘 kp 顶不过
静摩擦，会卡在约 10° 的偏差上不动。见 [7.2](../README.md#42-起驱动)。

**Q: RViz 里机械臂整条躺在书写平面里，看着像解错了？**
正常。笔轴垂直纸面时 `link6 → link7` 那一节必然平行于纸面，两者是同一件事的两面。
见 [5.2](../README.md#35-rviz-里看什么)。

**Q: 姿态误差报了 0.5 rad 以上，但笔迹看起来是对的？**
pen_axis 模式下完整 3D 姿态误差包含故意不控制的绕笔自转。看 `pen_axis_error`，
见 [8.4](#84-读哪个姿态误差指标)。

**Q: 关节角在 5 ms 内跳了几个 rad？**
先查数据来源，不要查动力学。这类物理上不可能的跳变通常意味着有多个节点实例同时在发布同一
话题。`ps -eo pid,args | grep student_arm_node` 确认只有一个实例。读实物真值用
[7.4](../README.md#41-上机前检查) 的只读脚本。

**Q: 串口打不开？**
确认 `ls /dev/ttyUSB*` 有设备、已加入 `dialout` 组（`sudo usermod -aG dialout $USER` 后需
重新登录）、`port_name` 正确、且没有其他进程占用（`fuser -v /dev/ttyUSB0`）。

**Q: CP2102 需要装驱动吗？**
Linux 不需要，内核自带 `cp210x`。`lsmod | grep cp210` 应能看到。网上下载的
`CP210x_Universal_Windows_Driver.zip` 是 Windows 驱动，用不上。

**Q: 提示 `Direct effort command timeout`？**
runner 超过 `effort_command_timeout_sec`（默认 0.05 s）没发新指令。确认兜底行为是带默认
增益的位置保持，而不是零力矩 —— 见 [7.3](#73-直接力矩接口)。

**Q: `ros2 topic list` 报 `!rclpy.ok()`？**
ROS 2 daemon 处于坏状态。`ros2 daemon stop` 后重试，或用 `--no-daemon`。

**Q: `ros2 node list` 看得到节点，`ros2 topic list` 却列不出它的话题？**
同样是 daemon 缓存。实测出现过 `ros2 topic info` 报 `Unknown topic`、而加 `--no-daemon`
后订阅赫然在列的情况 —— 差点被误判成"订阅没建起来"。**排查订阅/发布关系时一律加
`--no-daemon`**，这和 [7.4](../README.md#41-上机前检查) 那条"物理上不可能的现象先查数据源"是同一类
教训：先怀疑观测工具，再怀疑被观测对象。

**Q: 仿真精度做到 0.01 mm 有意义吗？**
没有。编码器分辨率折到末端约 0.13 mm，低于这个数的仿真结果在真机上不可复现。
见 [10.5](#105-传感器分辨率)。
