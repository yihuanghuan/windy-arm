# Gravity compensation / manual guidance 控制前准备

日期2026-09-17；仅仓库审计、离线数值计算和单元测试。当前新固件只验证RX，尚未验证TX；本报告与代码变更不构成主动控制授权。

## 1. 完整控制链与字段

```text
MotorControl (/student/joint_effort_command)
  → StudentArmNode::ValidateEffortCommand / StudentEffortCommandCallback
  → ControlLoop → ExecuteEffortGuarded
  → ArmPlatform::ExecuteDirectMotorCommand
  → AbsArm::SetMotorCommand
  → 六个DMMotor::UpdateCommand(id=0..5)
  → ProtocolV1 SetPosition/Velocity/Current/Kp/Kd
  → AbsBus::Send → ProtocolV1::Pop（0x10→11→12→13→14）
  → SerialBus::SendCore → serial_.write
  → FE AA cmd 1c [7×float32 payload] sum add → 控制板内部（不可见）
```

RX的FE55与TX的FEAA不同；每类TX帧34字节，常规完整周期五帧共170字节。checksum覆盖帧头至payload，sum累加、add累计sum。当前主机memcpy/native float假设小端IEEE754；preview明确转float32。fresh ProtocolV1的第7槽初始0，gamma6只更新0..5，故五帧仍携带slot6=0，**不是只发6槽**。

| MotorControl字段 | 处理与协议 |
|---|---|
| header | 不进入ProtocolV1；节点超时使用ROS接收时刻，不按消息header age拒收 |
| position | 六轴目标rad；后三轴乘-1→0x10；位置反馈距限位0.05rad以内时存在目标/增益覆盖，见下 |
| velocity | 目标rad/s；后三轴乘-1→0x11；直驱路径不经过SmoothPositionController限速 |
| current | 后三轴乘-1，再按rated_torque夹限→0x12。字段名current不证明物理单位是A |
| p | 原值→0x13；不取负；无一般增益幅值/非负上限检查 |
| d | 原值→0x14；不取负；无一般增益幅值/非负上限检查 |
| enable | 没有对应串口字段，DMMotor不检查；student节点把0转换为主动保位 |

源文件：`src/arm-platform/src/ros_nodes/student_arm_node.cc`、`include/manipulator/ros_nodes/student_arm_node.h`、`src/robotics/{arm_platform.cc,arm/abs_arm.cc,motor/dm_motor.cc,protocol/protocol_v1.cc,bus/abs_bus.cc,bus/serial_bus.cc}`。

### 校验、补缺和隐含行为

非零enable时current必须等于joint_count=6，所有提供数组要求finite。position若长度不是6，整体替换为缓存当前位置；velocity/p/d长度不是6会整体置0。这会隐式改变命令，首次实验必须六个数组全部明确提供。非零enable不是只接受1，任意非零走同一路。

`DMMotor::UpdateCommand`先写Kp/Kd/current，再处理position。若实际位置距上下限<0.05rad且p=0，则自动恢复motor_config默认p/d，目标夹到限位内；若p非零但目标不在内部安全区，也会覆盖目标。这意味着“纯力矩”在限位附近不再纯力矩。current只有前馈夹限，PD总力矩不受该夹限保障。

### enable=0、启动、timeout、Ctrl-C

- **enable=0不是电机失能/急停/零力矩。** 回调忽略传入数值，MakeHoldPositionCommand取缓存q，velocity=0，p/d为保位增益（缺/非正值fallback5/0.1），current取可选重力前馈。常规MakeHoldPositionCommand还将enable重写为1。硬件根本没有收到enable字段。
- 直驱默认参数effort timeout=0.05s、心跳timeout=0.2s。外部指令超时后锁存当时q并主动PD(+可选g)保位，继续发送五帧。不是“停发=安全断力”。
- 心跳过期只阻止非零current，改为current=0的主动PD保持；不会阻止所有position/Kp/Kd输出。若命令本来current=0，门控不拦截。
- **直驱正常定时模式即使未收到外部MotorControl，启动后也自动保位。** 它先PublishCurrentState等待心跳、再等3次joint_feedback_generation递增，然后一次锁存hold，持续TX。因此启动student_arm_node hardware本身就是主动控制。
- 这“3次反馈”是状态发布次数，不是三个经验证的新position帧；ProtocolV1全局心跳也不能证明每轴有效。回调的step_on_effort_command分支可直接ExecuteEffortGuarded，并不保证走完定时器启动门控；该参数设计给仿真，不能拿来当硬件安全门。
- 非直驱默认位置路径：无setpoint时SmoothPositionController返回空command，DMMotor因数组缺项不更新，fresh ProtocolV1无新控制payload，不能概括为也自动保位。收到position目标后正常控制，1秒timeout时锁存当前位置继续保位。不能据此把hardware启动视为已审计的纯接收：它打开可写串口、保持命令订阅且进入发送路径。
- student_arm6现有launch有真实启动参数p=[35,35,35,5,5,5]、d=[1.5,1.5,1.5,1,1,1]、gravity_hold_enabled=True；不同于节点本身默认，也不同于本计划候选Kp=30。**本轮已把该launch历史gravity_hold_zero_offset_rad改成全零**，但没有消除自动保位，仍禁止直接启动。
- StudentArmNode析构只取消timer，SerialBus析构只close。Ctrl-C/进程崩溃不会发已定义的失能或归零帧；控制板保持、失能或其他反应不可见。只有控制发布器退出、driver仍活着时才可能触发上述timeout fallback。
- 这条AL1/学生节点链没有显式硬件reset/回零指令。软件`ResetVelocityState`只重置内部平滑状态；不能与电机reset混淆。串口打开的电气控制线行为及板端上电默认动作另属未知。
- Phase4自身退出finally还会尝试主动send保位；abort中也有约300周期回退。因此Ctrl-C Phase4也不是无输出停止。本轮没有运行它。

## 2. 新固件兼容性与0x12标度

源码确定**主机仍发送0x10–0x14**，不能证明新固件仍按旧含义接收。反馈恢复仅验证FE55上行，不能证明FEAA下行旋向、MIT增益单位、模式、总限幅和watchdog。

作者历史Phase2、calibration metadata、PHASE1_FINDINGS/REFERENCE支持旧平台0x12近似为关节Nm：位移斜率法`alpha = Kp * d(q)/d(tau_command)`；单joint1曾测0.9932±0.0943，标定文件保存六轴合并1.033±0.024。历史量化分析0x22约DM±28、RS05±6，与 torque-like 解释相容；仅0x22跟随命令不足以独立证明真实力矩，因为可能反馈内部命令估计。

**alpha是实验的实际输出/命令比值，不是帧编码常数。** 它依赖当时电机、固件缩放、增益及测法，不能跨固件自动移植。若以后独立确认该比值适用，期望Nm→命令应为tau/alpha，而不是tau*alpha。本次preview用alpha=1作为待验证假设，不应用1.033。仓库Python torque_nm adapter也直接传tau，无统一自动alpha修正；ampere分支需要Kt×gear×efficiency，不能凭字段名切换。

## 3. 重力重复补偿：已确认并修复

`arm6.urdf`与`arm6_tool.urdf`头部明确记载质量/惯量已由历史first_moment_ratio拟合修正。逐项XML比较arm.urdf原CAD与arm6得到link1..6质量倍率：

`[1.0049703,1.3363357,1.4182605,1.2199444,1.0,1.3612618]`

这不是每关节ratio的简单一一乘法，而是连杆质量拟合；某关节的g依赖后续连杆。修正与标定元数据在ef84a9d提交加入；不能再把[1.265,1.349,1.228,0.972,1.021,1]乘一次。

```text
当前arm6/arm6_tool已拟合惯性
 → Pinocchio g(q_model) = 该URDF的势能梯度
 → GravityController: 直接g（原本就没有ratio）
 → Phase4/Phase5: 直接g（本轮移除重复ratio）
 → SMC RNEA: 自带一个g（本轮移除额外g替换/缩放）
```

“已修正”只代表模型包含历史拟合，不意味着本机精确。arm6_tool另含固定link7工具，真实没有该工具时不能选它；预览用arm6。惯量armature只影响加速项，不影响静态g，手推初期不自动增加摩擦/惯性补偿。

SMC仍保留load_arm6_calibration用于历史armature/provenance；默认apply_armature=True的既有动态行为未全盘改写，**未来运行SMC前仍必须审核本机惯量**。本轮preview使用固定基座纯URDF，未用armature。Phase2/3属于历史标定工作流，某些ratio用于辨识而非运行前馈；未将整个归档/校准库一概改写，禁止直接拿来上机。

## 4. zero_offset真实语义

历史量是`q_model = q_encoder + offset`：由重力正弦相位拟合编码器坐标与URDF建模坐标之间的差（实现zero_offset=-wrap(phi_measured−phi_model)），不是固件机械归零寄存器，也不是一个应加到所有command.position的通用控制偏置。它影响FK/Jacobian与g计算；期望模型角若转回编码器控制目标，应做逆变换，不能两边都加。

测量受当时姿态、摩擦、负载、基座朝向和参考模型影响；历史J4/J5未充分收敛、J6零偏不可由无重力信号识别。用户已确认当前raw→URDF→RViz姿态一致，所以**本机默认offset=[0,0,0,0,0,0]**，不自动应用历史六值。视觉一致是强基线，仍不等于亚角度动力学标定已完成。

本轮SMC model新增显式zero_offset_rad参数 / config model_zero_offset_rad；Phase4/5新增显式--model-zero-offset-rad。没有显式输入就零，环境ARM6_CALIBRATION_YAML不能再悄悄注入这三条路径的历史offset。student_arm6 fallback也改为零。

未来重新标定方案仅设计：先固定底座与工具，独立测base朝向/连杆几何；在另行批准的多个已知静态姿态用外部角度/位姿测量拟合encoder-to-model offset，独立留出姿态验证，给不确定度。与质量、摩擦、力矩标度分开辨识；J6用可观测的末端方向标记而非仅末端位置或g。当前不执行任何扫动。

## 5. 手推模式定义

| 结构 | 释放后的表现/局限 |
|---|---|
| 纯g | 理想模型重力平衡，但无耗散；惯性可使其继续移动，误差可漂移 |
| g−D*dq | 能耗散速度，适合引导；模型误差仍可造成慢漂，无特定保持点 |
| g+低Kp(q_ref−q)−D*dq | 可稳定，但若q_ref一直固定在旧姿态，松手会回到旧位置 |
| g+阻尼+release position latch | 引导时低刚度，释放后锁存新位置并平滑增加刚度，最符合目标 |

推荐最后一种，初期由操作者明确给GUIDE/RELEASE信号。仅靠速度小无法区分“松手”“仍握住静止”“卡住”；没有外力传感器和经验证的观测器，不做自动意图判定。

两种等价实现只能选一：板端MIT PD时`current=g, position=q_hold, velocity=0, p=Kp,d=Kd`；主机全力矩实现则`current=g+Kp*error-D*dq,p=d=0`。不能两边重复阻尼/刚度。GUIDE默认板端p≈0、d>0，近限位会触发现有DMMotor gain override，必须先设计边界行为。

可追溯历史范围：Phase4 L0 Kp30/5，Kd1/0.1；L1 Kp15/2.5、Kd0.5/0.05；L2 Kp6/1、Kd0.3/0.03；L3 Kp1.5/0.25、Kd0.2/0.02；L4 Kp0、Kd0.2/0.02（前三/后三轴）。motor_config保位Kd1.5/1更高。这些只是旧平台实验范围，新固件不得直接照搬。低阻尼与速度量化/摩擦相互影响，需要新机验证。不得把motors disabled/zero torque称为重力补偿。

## 6. 当前姿态dry-run

输入为用户提供保存角度，未订阅实时数据、未重开串口：

`[-0.031854,1.356337,-1.334973,0.001726,0.007481,0.001726]`

| joint | g(q) Nm（模型） | abs(g)/rated | 最近限位余量rad |
|---|---:|---:|---:|
| 1 | 0.243443630 | 2.705% | 3.968146 |
| 2 | -2.204040600 | 24.489% | 0.643663 |
| 3 | 0.995140384 | 11.057% | 0.235827 |
| 4 | 0.002196898 | 0.137% | 3.998274 |
| 5 | -0.005219531 | 0.326% | 1.992519 |
| 6 | 0.000110836 | 0.0069% | 3.998274 |

全部finite；未超rated，未触及0.05rad驱动覆盖区，也未进入预览0.15rad近限位提示区；J3余量最小，不代表工作空间碰撞已检查。J6无工具时仅link6自身参与，其质心相对x转轴径向约0.113mm，因此重力接近0但不要求精确0。关节重力符号随姿态/轴定义变化，不能以“正负不统一”判异常；数值势能梯度一致，实际控制旋向仍未验证。

preview输出完整MotorControl字典（stamp=0明确为离线占位）、Kp/Kd、g、软件包络、五个float32七槽理论数组。**只有打印，不构造ROS消息对象、不初始化ROS、不导入serial/receiver、没有publish/open-device路径。** 可输入--q或保存的--joint-state-json；后者按joint name重排，拒绝缺项/重复/非有限值。

运行本轮使用的离线命令：

```bash
PYTHONPATH="$PWD/src/arm-platform:$PYTHONPATH" python3 -m task_space_smc.core.manual_guidance_preview --q -0.031854 1.356337 -1.334973 0.001726 0.007481 0.001726
```

预览代码绝不提供--execute/硬件模式。新工具不使用会产生MotorControl的旧ActuatorAdapter或Phase4入口。

## 7. 代码变更、测试和局限

实际修改（均未上机）：

- core/gravity_semantics.py：URDF单一重力来源与显式offset校验。
- core/model.py：SMC移除重复重力倍率，默认offset0，保留显式偏移入口及历史provenance。
- hw/phase4_gravity_compensation.py：前馈改为URDF g，默认offset0；Phase5发现同类问题一并修复。
- launch/student_arm6.launch.py：fallback历史offset改0；**仍为危险hardware launch，未运行**。
- config/arm6_calibration.yaml：澄清历史ratio/offset/标定适用范围，不修改历史数值。
- src/controller/gravity_controller.cc：只补充语义注释，原数值本就直接使用g。
- core/manual_guidance_preview.py：仅离线打印工具；test/core/test_gravity_semantics.py：数值回归。

测试86项通过：重力势能有限差分与Pinocchio一致、arm6/arm6_tool固定基座与浮动基座SMC静止RNEA一致、g下正向动力学加速度0、Phase4/5真实feedforward方法对非单位历史ratio不再重复补偿、显式offset一致作用于运动学/动力学、六槽旋向/第七槽0/float32、NaN与限位拒绝，以及现有模型/重力辨识离线回归。测试只提取Phase4/5数值方法，不构建硬件Node；C++仅注释变更，未运行硬件可执行文件，也未声称完成C++实机验证。

风险仍包括：新固件TX未知；自动启动保位、enable=0非失能、五帧非原子、current仅前馈夹限、每轴反馈新鲜度不足、Ctrl-C后状态不明；URDF历史拟合与本机负载不确定；GUIDE意图/释放检测不可仅靠速度。修改语义将改变既有SMC/Phase4数值，旧实验结果不能直接沿用，任何主动试验前须重新审阅。

第一实验具体候选、NO-GO门槛、持续时间、abort及退出状态见 [FIRST_ACTIVE_CONTROL_PLAN.md](FIRST_ACTIVE_CONTROL_PLAN.md)。本轮停止于准备。
