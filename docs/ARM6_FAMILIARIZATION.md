# REAL ARM HARDWARE UNDERSTANDING

日常终端操作见 [零重力 ↔ HOLD 操作手册](ARM6_ZERO_GRAVITY_OPERATOR.md)，
包含当前故障重启、正常开关和断电冷启动。当前稳定重复启动问题尚未修复。

**已完成的原手推档结论（2026-09-18）：ARM6 GRAVITY / MANUAL GUIDANCE: PASS，限本次测试姿态
和保留保护的手推档。** 三次无接触接管、六轴独立手推及整臂联动已完成。
[最终验收报告与曲线](arm6_familiarization_validation/restart_after_power_20260918/ACCEPTANCE_REPORT.md)。
结束时为带重力支撑的 HOLD，guidance 已退出；没有取消任何速度/限位保护。
随后扩大速度范围的整臂试验在人工推动 J4 达 −2.014652 rad/s 时触发瞬时保护，
操作者确认故障后无异常；该轮不能标为新档位验收 PASS。
[1.8/2.0 档位报告](arm6_familiarization_validation/speed_18_20260918/ACCEPTANCE_REPORT.md)。
按操作者后续明确要求，当前配置将 J4–J6 瞬时上限小幅改到 2.2 rad/s，J1–J3
仍为 2.0；六轴持续阈值均为 1.8 rad/s / 120 ms。新配置验证进度见
[腕部 2.2 档位报告](arm6_familiarization_validation/wrist_22_20260918/ACCEPTANCE_REPORT.md)。
不要把此前的 PASS 当作此新档位已完成实机验收。
腕部新档位本轮最终为 NOT YET VERIFIED：第一次启动反馈故障；人工要求的一次重试
完成两次接管，第三次在 JointState 时间戳检查中止，随后接管超时锁存。当前为
保留有界 FF 的故障 HOLD，guidance 已退出，RViz 保留。

以下包含历史调查与失败试验，不代表当前最终验收结果。

最新接管/手推档说明见
[bumpless 验收](arm6_familiarization_validation/bumpless_20260918/ACCEPTANCE_REPORT.md)。
本文下面保留此前调查与历史试验，不能将旧版启动命令当作当前接管流程。

当前 guidance 必须从驱动实际已发送的健康 HOLD 指令接管；无法取得该指令
时拒绝启动，不能重置 q_ref 并从零 FF 猜测承重状态。驱动重启的显式
`hold_command_file` 仅接受同一串口、120 秒内采集且位置/速度校验通过的
指令快照，不能复用过期验收文件。常规重复试验保持同一个驱动运行，调用
`/arm6_manual_guidance/stop` 平滑回 HOLD 后才退出/重启 guidance 节点。

`manual_push_test:=true` 只是允许申请手推档，并不会直接放宽速度限制。
同一个驱动需完成至少三次接管及各 5 秒静置；当前 GUIDE 静置通过，人工
明确就绪后，调用 `/arm6_manual_guidance/begin_manual_push`，等待
`ACTIVE MANUAL_PUSH`。普通 GUIDE 保留原阈值；所有接管阶段仍为
0.5 rad/s 立即中止。发生故障不能用正常停止/准备服务清除锁存。

断电后不能复用断电前的 HOLD 文件。新的受支撑初始化入口是
`supported_startup_file`，仅允许 `mode:=guidance start_guidance:=false`
且 `external_support_confirmed:=true`。先用
`task_space_smc/hw/capture_supported_startup.py` 只读采集连续静止反馈；
15 秒内启动，驱动再检查实测姿态与样本相差不超过 0.002 rad。
此入口使用静止力矩反馈中位数估计首帧支撑，FF=0、原 HOLD 增益不变，
通过 q_ref 产生该估计支撑。它**不是旧命令读回，也不保证冷启动无扰**，
必须在外部可靠承重支撑下建立并验证新 HOLD。正常 guidance 随后仍从驱动
实际已发送的 HOLD 快照接管，所有限位、速度与故障锁存保护保留。

2026-09-18，修改控制代码前完成调查。工作分支 `lagrange-smc`，HEAD
`b496a4f`。当前本地/远程引用均无 `merge/zzq-v2`，没有切分支、reset、commit
或 push。开始时已有六个修改文件及实机审计/工具等未跟踪文件，保留原工作。

## Mechanical model

- DOF = 6；joint names = joint1, joint2, joint3, joint4, joint5, joint6。
- URDF = `src/arm-platform/config/arm6.urdf`；base = `base_link`；EE = `link6`。
- `a_l1` + `gamma6` 由 YAML 创建六个 DMMotor；不使用 `a_l1_gamma` 七轴工厂。
- 固件 V1 固定七槽帧属于传输格式；第七槽为 ghost，不能创建 joint7。
  ROS JointState、MotorState、MotorControl 及手推计算均为六维。

## Motors

以下数值来自当前 motor_config.yaml，不是本轮实测额定能力。

| joint | id/数组槽 | motor | raw→ROS 符号 | rated/max Nm | 默认 MIT Kp/Kd |
|---|---:|---|---:|---|---|
| joint1 | 0 | DM4340-48V | +（右手） | 9/27 | 30/1.5 |
| joint2 | 1 | DM4340-48V | +（右手） | 9/27 | 30/1.5 |
| joint3 | 2 | DM4340-48V | +（右手） | 9/27 | 30/1.5 |
| joint4 | 3 | RS05 | −（左手） | 1.6/5.5 | 5/1 |
| joint5 | 4 | RS05 | −（左手） | 1.6/5.5 | 5/1 |
| joint6 | 5 | RS05 | −（左手） | 1.6/5.5 | 5/1 |

实际旋向取 motor_config 的型号字段，arm_config 中逐关节 coord_system 不被解析。
DMMotor 在反馈与命令边界转换 q/dq/effort 的符号，并连续展开角度；不加模型零偏。
额定力矩用于前馈 clamp；max_torque 和 max_velocity 配置尚未作为驱动总输出保护。

## Hardware communication / ROS

ProtocolV1，921600 baud。现场发现 CP2102：
`/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`
→ `/dev/ttyUSB0`；fuser/进程检查未发现占用。没有凭默认串口名判断设备。
TX `FE AA`，RX `FE 55`，双累加校验。0x10–14 为 q/dq/FF/Kp/Kd，
0x20–24 为反馈，0x01 心跳。0x12 字段名 current，现有驱动按 nominal Nm
使用；精确固件标度、下游 CAN ID、断流行为没有本轮认证证据。

| topic | type | 用途 |
|---|---|---|
| /joint_states | sensor_msgs/msg/JointState | 实测编码器 q/dq/effort，给 RSP |
| /student/joint_command | sensor_msgs/msg/JointState | 位置目标，不是反馈 |
| /student/joint_effort_command | dummy_interface/msg/MotorControl | MIT q/dq/p/d/current |
| /student/joint_feedback | dummy_interface/msg/MotorState | 六电机位置、速度、current、温度 |

MotorControl.enable=0 在 student 中表示请求保位，不是硬件失能；协议不编码 enable。
仅 student 管串口，gravity 节点只通过 ROS 发布。

## Gravity convention

```text
raw encoder → DMMotor sign/unwrap → q_encoder (/joint_states)
q_model = q_encoder + explicit model_zero_offset_rad  [一次]
tau_gravity = Pinocchio(arm6.urdf).gravity(q_model)    [不乘 ratio]
六维 MotorControl → DMMotor inverse sign → V1 serial
```

arm6.urdf 文件头与 arm6_calibration.yaml 均说明质量/惯量已按连杆尺度
`[1.0050,1.3363,1.4183,1.2199,1,1.3613]` 拟合。历史 first_moment_ratio
`[1.265,1.349,1.228,.972,1.021,1]` 是相对 CAD 的记录，不能再次乘在 g 上。
GravityModel 本身不加偏置；FloatingBaseArmModel 在 make_q_full 中加显式偏置。
现有未提交的 gravity_semantics/model/phase4/phase5 修改已去除重复缩放，保留。
默认 offset=0 是尚未采用历史标定，不是确认模型与实机零位一致。
历史 joint4/5 未收敛，joint6 重力信号很弱；不能用手推效果倒推其零偏准确。
本阶段假定 base 固定且模型 +Z 向上，无工具/相机附加载荷。

## Safety review before changes

- 已有：位置命令维数/NaN/限位检查；1 s 命令超时；50 ms effort 超时保位；
  200 ms heartbeat；9/1.6 Nm 前馈限幅；近限位恢复默认增益。
- 缺口：startup 三帧数的是 ROS 回调，不能证明三个真实帧；heartbeat
  不能代替 q/dq 新鲜度；解码未校验各类型长度；未知帧也 Notify。
- effort 仅 current 严格定长，其他数组会补齐；底层 q/dq 索引无完整保护。
- 没有真实 feedback 的 NaN/速度统一门；析构仅取消 timer；保位并非急停。
- 模型限位 j2=[−3.14,1.57]，配置为 [−3.1415,2]，手推应取两者交集；
  URDF effort/velocity=0 不代表真实额定力矩/速度。
- 总 MIT 输出含 PD，前馈限幅不能保证硬件总输出限幅。串口断开、进程被杀后
  板端行为未知；软件退出不能保证断能或永久保持。

## References inspected

阅读用户指定 active 文件及 `.Trash/.../hw/manual_guidance.py`。归档手推代码
包含重复 ratio，以及 FF 减 damping*dq 同时设置 MIT d 的重复阻尼；不照搬。
外部参考实际 package root 为
`~/westlake/windylab-arm-real/windylab-arm/windylab_ws/src/arm-platform`，
已阅读 gravity_controller、gravity_drag_config、master/student 节点与 launch、
arm_platform。采用渐入、限幅、限位与退出设计思路，不移植 gripper/joint7。
本次不涉及 SLAM、D435i、基座扰动、末端稳像或 SMC 控制。

现场既有记录 `docs/real_arm_audit_20260917/gravity_tuning/GRAVITY_TUNING_RESULT.md`
记载 Run06 J5 HARD_ABORT，重力补偿尚未验收。此为已有证据，不是本轮试验。
不能把软件实现、离线测试通过写成“已验证松手不下坠”。

## Implemented interface

入口 `arm6_hardware_familiarization.launch.py`：固定 a_l1/gamma6/arm6/link6；
串口必须显式传入。默认 `mode:=readonly`，只用已有 student_arm_node 读真实反馈，
配套 RSP/RViz，不启动任何模拟反馈、SMC、轨迹或第二个串口节点。

`mode:=hold` 是有动力的 PD 保位；`mode:=guidance` 启动手推节点，先等真实完整
q/dq/effort 三组帧，再从当前反馈锁存起点：3 s 渐入 g(q)，再用 3 s 平滑撤去
Kp，持续 GUIDE。GUIDE 为 p=0、dq_ref=0、current=g(q)、d=配置阻尼；当前位置
字段只为完整 MIT 消息，不产生固定姿态恢复力。正常手推不自动锁存释放位姿。
唯一阻尼在 MIT d，无历史 ratio、alpha 或摩擦 boost。不能保证模型误差为零。

数学核心 `task_space_smc/core/manual_guidance.py` 为严格六维，限制反馈年龄、
周期间隔、速度、关节限位余量、重力预算、前馈变化率。限位取 YAML 和 URDF
（扣除显式模型零偏）的交集。重力+阻尼预算不够时拒绝进入，不静默假装补偿足够。
当前限位保护是停止 GUIDE 并保位，不是软力矩墙；不提供碰撞/桌面安全认证。

速度保护（2026-09-18 更新）：共享 YAML 中的 `transition_*` / `guide_*` 参数
同时传入 Python 和 C++。RAMP/Kp withdrawal 单帧超过 0.5 rad/s 仍中止；
GUIDE 超过 0.5 只告警，超过 1.0 连续 80 ms 或单帧超过 1.2 才中止。
初始持续阈值为0.8；现场J2正常手推达0.982触发后，用户明确要求扩大范围，
因此仅将持续阈值改为1.0，其他阈值与增益不变。
六轴分别使用 monotonic/steady clock 计时，回到阈值或以下即复位；故障不复位。
驱动仅在实际下发的六轴 Kp 全为零时认定 GUIDE。故障或 operator stop 后恢复
PD 保持，因此此时的阻力和回位手感不能作为 GUIDE 的低阻抗验收结果。
两层的告警/计时诊断分别发布于 `/student/speed_safety` 和
`/arm6_manual_guidance/speed_safety`。详见
[本轮变更与验证](arm6_familiarization_validation/speed_phase_20260918/SPEED_SAFETY_CHANGE_SUMMARY.md)。

驱动补齐 q/dq/effort 帧长度、有限值与单独新鲜度检查，未知帧/heartbeat 不更新
电机状态；无真实完整反馈时不发布初始化零缓存。相同帧不重复盖新 ROS 时间戳。
`/joint_states` 永远是编码器坐标；若显式提供非零模型偏置，launch 只在内存中
将偏置写入 RSP 描述的关节 origin，使可视化与动力学一致，磁盘 URDF 不修改。

MotorControl 启用消息五组数组严格定长，拒绝 NaN、负增益、过旧消息和越界目标。
familiarization 关闭位置指令；effort 超时/反馈丢失/超速锁存故障，重启前不能
恢复手推。有效消息时间按 steady clock 计时，消息 stamp 额外检查排队旧命令。

### Stop semantics

优先在硬件节点继续运行时停止手推：

```bash
ros2 service call /arm6_manual_guidance/stop std_srvs/srv/Trigger '{}'
```

它请求驱动锁存当前位置并转 PD+最近已发送的有界重力前馈保位，正常 GUIDE 不保位；
停止后不自动重启。渐入期间退出不突然跳到完整 g(q)。节点丢失由驱动 50 ms
watchdog 保位；反馈失效切零前馈 PD 保位，保位目标只锁存一次。若仅某轴超速而
其他反馈仍有效，仅清除超速轴的前馈，保留其他轴已发送的支撑前馈；已经撤掉的
前馈不会在故障锁存期间自动恢复。这是本轮实测中止之后的离线修正，未再实测。
完整 launch 的 Ctrl-C 会尝试发送一次退出保位，**不是硬件失能**，进程关闭
后没有持续 watchdog；硬断链、断电、SIGKILL 和固件断流行为仍不能由此保证。
托举/实体急停由现场人员负责，不能以“关闭串口”当作动力切断。

## Build and run

本轮隔离构建在 `/tmp/windy-familiarization-{build,install,log}`，没有覆盖其他
workspace。普通构建可使用：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to manipulator dummy_description
source install/setup.bash
```

直接使用本轮已构建 overlay：

```bash
source /opt/ros/humble/setup.bash
source /tmp/windy-familiarization-install/setup.bash
export ROS_DOMAIN_ID=95
export ROS_LOCALHOST_ONLY=1
ros2 launch manipulator arm6_hardware_familiarization.launch.py \
  port_name:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

先确认该设备仍是实机、串口无其他持有者。同一个串口不能同时运行此 launch、
旧 readonly 包、旧 bringup/tuning tools 或参考工程驱动。readonly 表示没有电机
数据写入，不表示 open/termios 操作不会改变串口控制线。

现场排查最近 J5 HARD_ABORT 并准备托举/急停后，才将上面命令改为
`mode:=guidance`。这是明确的主动模式，启动将施力；前 6 秒尚未完全低阻抗。
首轮禁止自动运动到旧姿态、用旧脚本代替此入口或绕过故障自动重试。
本轮经用户现场就绪确认后执行过一次主动模式，但在 GUIDE 前保护中止，见下文。

验收过程：先逐关节慢推确认实际方向与 RViz 相同，再在远离关节限位与桌面的
姿态下观察 GUIDE 的松手漂移、速度和温度，记录当前负载/零偏/模型。持续漂移、
明显下坠或主动运动时停止并核对 g(q)、坐标与固件 torque 标度；不能用提高 Kp
掩盖问题并宣称零重力完成。不能从当前近静止截图推断人工推动验收通过。

## Verification, 2026-09-18

- 隔离构建成功；4 个 C++ 测试套件、19 项 Python 重力/手推测试通过。
- 5 项伪串口端到端测试通过：启动不发假零反馈、readonly 零写入、六轴反馈与
  双向符号、拒绝非六维/NaN 命令、ghost 不进入 ROS、停止位置帧而心跳继续、
  超时锁存保持不自动重启、安装后手推节点完成渐入/零 Kp/停止服务，以及单轴
  超速不撤去其他轴支撑、随后反馈失效撤去全部前馈且不自动恢复。
- 第一轮真机 **readonly**：8 秒 804 条 JointState，100.30 Hz，最大接收间隔
  17.29 ms；511 组同时间戳完整 TF 与 Pinocchio FK 一致，位置误差
  6.98e-17 m、转角误差 7.82e-17 rad。数值一致性不是实机几何标定精度。
- graph：唯一 JointState 发布者 student_arm_node，MotorControl 发布者 0；
  RViz RobotModel/Global Status 均 OK。strace 对实机 tty 的 write/writev/pwrite64
  调用数 **0**；40 s 后只读 launch 正常关闭，fuser 未发现残留占用。
- 人工推动同步尚未现场确认；真实非零重力有过尝试，松手漂移和低阻抗感受
  **未验收通过**，不能将软件/伪串口结果替代实机验收。

证据：[反馈/TF](arm6_familiarization_validation/feedback_live.json)、
[串口零写入](arm6_familiarization_validation/serial_audit.json)、
[RViz 截图](arm6_familiarization_validation/rviz_live.png)、
[launch 日志](arm6_familiarization_validation/launch_readonly.txt)、
[伪串口测试](arm6_familiarization_validation/tests_pty.txt)。

### Authorized active trial: FAILED BEFORE GUIDE

用户确认“现场已就绪，可以进行上述手推验收”后，使用同一 launch 的
`mode:=guidance`，无第二串口连接。3 s 渐入后撤去刚度，在首条手推命令后
**5.583 s**，J5 反馈 **0.500610 rad/s** 超过 0.5 阈值，驱动锁存故障。
随后停止服务终止 guidance 发布，记录故障后约 2 s 状态，再关闭 launch。
**没有进入 GUIDE、没有重试、没有放宽阈值、没有翻转符号或套用历史标定。**

中止前最后一条 ROS 请求：Kp=`[1.5994,1.5994,1.5994,.26657,.26657,.26657]`，
Kd=`[.24265,.24265,.24265,.05267,.05267,.05267]`，
FF=`[.24242,-2.22053,1.00933,.023857,-.002980,.000115]` nominal Nm。
这些是请求值，不是板端 ACK 或实测总力矩。

| 指标 | J2 | J5 |
|---|---:|---:|
| 起始 q (rad) | 1.362059 | .102233 |
| 故障附近 q (rad) | 1.404783 | .145197 |
| 故障后约 2 s 的 q (rad) | 1.458953 | .163226 |
| 全记录最大 abs(dq) (rad/s) | .444445 | .720390 |
| 故障后观测位移 (rad) | +.054170 | +.018029 |

J5 的 .720390 rad/s 峰值出现在故障附近/之后，不能只报告触发帧 .500610。
位置窗口也显示运动，不是单纯以速度量化噪声判定。记录的最大 JointState 接收
间隔为 28.84 ms，小于 50 ms 门限；不能据此证明所有底层通信时刻，但当前证据
与速度保护吻合。

用户现场反馈：“**没有主动推动，但是机械臂确实碰到了我的手，这可能产生了力的
作用**”。因此外力不为零，不能将 J5 的运动独立归因于符号、质量模型或控制器。
本轮不能认证无接触漂移或零重力效果。

实测版本在任一轴超速后撤去全部 FF，J2 随后偏移约 3.10°，暴露承重轴失去支撑
的退出风险（接触因素也可能参与，不能作唯一因果归因）。随后仅在离线修改退出
逻辑：有效反馈下保留未超速轴最后已发送的有界 FF；故障轴清零；反馈失效全部
清零；故障锁存不恢复。新增伪串口回归通过。**该退出修正版未再施加到真机。**

最终程序全部关闭、串口无占用。最后的保位命令不等于硬件失能，关闭后没有连续
反馈，不能声称关闭后机械臂停止或永久保持。后续需先在可区分托举接触的条件下
复核 J5 与承重轴模型/力矩接口，才能继续验收。

证据：[逐时事件](arm6_familiarization_validation/active/events.jsonl)、
[数值分析与现场反馈](arm6_familiarization_validation/active/analysis.json)、
[时序图](arm6_familiarization_validation/active/trial.png)、
[主动 launch 日志](arm6_familiarization_validation/active/launch.txt)。
