# Real Arm Hardware Understanding Report

> 后续更新：用户已确认真实硬件为 6DOF A-L1 Gamma / gamma6 / arm6，身份不再待确认。本文保留初次调查时的证据边界；最新只读实现、测试和运行状态见 [READONLY_FOLLOWUP.md](READONLY_FOLLOWUP.md)。历史零偏/控制标定仍不能直接沿用。

日期：2026-09-17（Asia/Shanghai）。仓库：`/home/yihuang/westlake/windy-arm`；分支 `lagrange-smc`；HEAD `b496a4f`。

**结论：已完成源码和现场静态调查，未建立真机反馈链，未验证 RViz 同步。没有启动驱动、打开串口、发布控制消息、修改运行参数或进入重力补偿。**

证据分级：S=本次读到的源码；H=本次硬件/进程观测；R=本次 ROS 图观测；D=仓库历史说明/标定记录。D 不等于当前真机验证。当前只掌握 USB 桥身份，不能把候选机械臂型号写成已确认事实。

## 1. Hardware

| 项目 | 结论与证据 |
|---|---|
| 型号 | **候选 A-L1 Gamma 六轴构型，未完成真机身份确认**。S：`ArmFactory`→`a_l1`→`AL1::InitFromConfig`→`a_l1_gamma6`。H 只能识别 CP2102，不能识别桥后机械臂型号。 |
| DOF | 候选 6；当前没有编码器反馈/铭牌/实体关节清点证据，不能排除配置与实际设备不符。 |
| joints / 顺序 | 候选 `joint1, joint2, joint3, joint4, joint5, joint6`；按 YAML 插入顺序发布，命令数组按电机 id 索引。 |
| motor IDs | 候选协议数组下标 0..5；**不等于已确认的下游 CAN ID**。前 3 个模型 DM4340-48V，后 3 个 RS05。 |
| communication | H：USB→CP2102 USB-UART（10c4:ea60，cp210x）。S：SerialBus→ProtocolV1→控制板。板内到电机的物理总线、固件版本、CAN ID 未见可验证证据；不能断言 RS485/CAN/EtherCAT。 |
| serial device | H：`/dev/ttyUSB0`；稳定链接 `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`。 |
| permissions | H：root:dialout，0660；当前 yihuang 已在 dialout 组；没有修改权限。`fuser` 未显示占用者（受进程可见权限限制）。 |
| baudrate | S：节点硬编码 921600；当前未打开设备、未测得实际正在使用的波特率。 |
| cameras / CAN | H：USB 清单无 D435i；未发现 ttyACM/ttyTHS 或 SocketCAN 网络接口。这不排除控制板内部 CAN。 |
| 当前使用 URDF | R：没有可见机械臂节点/robot_state_publisher，因此**没有观测到当前使用的 URDF**。源码六轴候选为 `config/arm6.urdf`。 |

源码证据：`src/arm-platform/src/robotics/arm/{arm_factory,a_l1,a_l1_gamma,abs_arm}.cc`、`src/arm-platform/config/{arm_config,motor_config}.yaml`、`src/arm-platform/src/robotics/config/arm_config.cc`。

### 1.1 零位、旋向、限位与模型关系

驱动编码器坐标：j1..3 正号；j4..6 负号。`DMMotor::UpdateState` 首次用 remainder 归到 ±π，随后连续展开；没有减速比/角度单位转换，也没有把标定零偏加到 `/joint_states`。这不是物理回零或机械零位的确认。

| joint | id | 轴（局部 URDF） | 驱动限位 rad | arm6 URDF 限位 rad | 历史模型零偏 rad |
|---|---:|---|---|---|---:|
| joint1 | 0 | +X | [-4,4] | [-4,4] | +0.039576 |
| joint2 | 1 | -Y | [-3.1415,2] | [-3.14,1.57] | -0.297745 |
| joint3 | 2 | -Y | [-1.5708,1.5708] | [-1.57,1.57] | -0.020218 |
| joint4 | 3 | +X | [-4,4] | [-4,4] | -0.192988 |
| joint5 | 4 | +Z | [-2,2] | [-2,2] | -0.111667 |
| joint6 | 5 | +X | [-4,4] | [-4,4] | 0（未辨识） |

模型角定义为 `q_model = q_encoder_ros + zero_offset_rad`。URDF 零位是各 joint 旋转为零、使用其 origin 固定变换的姿态；六关节 origin rpy 均为 0，xyz 依次为 `[.0613,0,0]`、`[.042986,.03395,.064067]`、`[0,-.057,.24873]`、`[.015,.025,.063014]`、`[.15,0,.0335]`、`[.084,0,-.035]`（米）。**没有本次实测的编码器零位对应机械姿态**。j4/j5 标定未收敛，j6 无充分重力信号，不能把表中偏置当作本机认证值。

- `arm.urdf`：7 个 revolute joints，末端 link7，CAD 名义惯性；没有单独命名 gripper joint。joint7 限位 ±3.15，但七轴 YAML 为 ±4。
- `arm6.urdf`：移除 link7/joint7，末端 link6；连杆质量/惯性已改为历史标定值，不能理解成仅截掉一节的完全相同动力学模型。
- `arm6_tool.urdf`：仍为 6 个活动关节，`joint7` 为固定工具连接，末端 link7；固定 joint7 不是第七电机。
- 六轴 YAML 注释记载“第七通道幽灵数据”，不是本轮观察；协议固定七通道不证明物理 7DOF。
- `a_l1_gamma` 工厂分支为另一硬编码七电机路径；`a_l1 + arm_version=gamma6` 才是 YAML 六电机路径。硬编码 joint7 为右手系，YAML gamma 的 RS05 joint7 为左手系，两条路径也不等价。
- YAML 关节项中的 `coord_system` 不被 LoadArmConfig 读取；实际符号取 motor model。
- URDF effort/velocity limit 均为 0，不能用作真实电机额定能力。驱动限位与模型限位不一致，后续必须统一到已确认机械限位和坐标系。

### 1.2 双向协议与数据流

```text
真实编码器/控制板（当前未采样）
  ↓ USB-UART /dev/ttyUSB0，源码 921600
SerialBus::ReadCore → ProtocolV1::Feed/DecodeFrame
  ↓ DMMotor::UpdateState（符号、位置展开）
AbsArm::GetJointStates → ArmPlatform::Notify*
  ↓ student_arm_node
/joint_states + /student/joint_feedback
```

```text
/student/joint_command (JointState)
  → StudentCommandCallback → SmoothPositionController
/student/joint_effort_command (MotorControl)
  → StudentEffortCommandCallback → startup/timeout hold
  ↓ ArmPlatform::ExecuteControlCycle / ExecuteDirectMotorCommand
AbsArm::SetMotorCommand → DMMotor::UpdateCommand
  ↓ ProtocolV1::Pop → SerialBus::SendCore → serial.write
USB-UART → 控制板 → 电机（板内路径待确认）
```

接收帧 `FE 55 cmd len payload sum add`；发送帧 `FE AA cmd len payload sum add`。校验为字节累加 sum 与累计和 add（mod 256）。数值使用 memcpy 原生 float/uint16；常见小端主机等价小端编码，协议本身没有端序协商。

| 方向 | cmd | 内容 |
|---|---|---|
| 反馈 | 0x01 | heartbeat；源码历史注释约 20 Hz，非本次测量 |
| 反馈 | 0x20 / 0x21 / 0x22 / 0x23 | 位置/速度/current/电压，均按 7×float32 解码（28 B） |
| 反馈 | 0x24 | 温度，按 7×uint16 解码（14 B） |
| 反馈 | 0x25 | 未解释；只统计未知帧，不能猜测意义 |
| 命令 | 0x10 / 0x11 / 0x12 / 0x13 / 0x14 | 位置/速度/current/Kp/Kd，每帧 7×float32（28 B） |

即使只创建六个 DMMotor，**发送协议仍打包七通道**；第七通道的值及固件含义必须在主动控制前确认。`MotorControl.enable` 不被 DMMotor/ProtocolV1 编码为硬件断能命令。

`current` 单位有冲突：头文件称 A，DMMotor 用 rated_torque 限幅，历史标定称 0x12 接近 Nm（alpha=1.033）。本轮不能认证它是电流接口或确定换算。也未发现明确的 enable/disable/MIT 模式切换帧实现。实际固件控制模式需要独立确认。

## 2. ROS2

H：ROS2 Humble，环境仅 `/opt/ros/humble` overlay；`ros2 pkg prefix manipulator`、`dummy_description` 都未找到。进程只发现 domain 0 的 ros2 daemon；未见硬件节点、控制 runner、RViz/RSP；robot.service inactive。

R：对当前 domain 0，以及仓库启动脚本/交接文档提到的 domain 5、77，分别运行无 daemon 缓存的 node/topic/service 查询（各 3 秒发现窗口）。三个域都没有可见节点或服务；只出现 `/parameter_events`、`/rosout`。这不是所有域/所有网络 DDS 参与者的不存在证明。

### Feedback

下表是源码定义，**本次均未观测到**；Rate 标注 N/A 不是测得 0 Hz。

| Topic | Type | Rate | Source | 是否安全只读 |
|---|---|---:|---|---|
| /joint_states | sensor_msgs/msg/JointState | N/A；student timer 默认 100 Hz | student_arm_node 或仿真/其他发布者 | 订阅可只读；必须先确认发布者，且不得发布到这里 |
| /student/joint_feedback | dummy_interface/msg/MotorState | N/A；随状态发布，受参数控制 | student_arm_node | 订阅只读 |
| /arm/joint_feedback | dummy_interface/msg/MotorState | N/A | robot_arm_node（arm_hardware_node） | 订阅只读 |
| /master/joint_states、/slave/joint_states | sensor_msgs/msg/JointState | N/A | master/slave 节点及 namespace | 订阅只读；它们也参与遥操作控制链 |
| /master/joint_feedback、/slave/joint_feedback | dummy_interface/msg/MotorState | N/A | master/slave，发布参数决定是否有消息 | 订阅只读 |
| /health/master_arm、/health/slave_arm、/health/robot | std_msgs/msg/String | N/A；health 定时器约 1 Hz | 对应节点、聚合器 | 订阅只读，不能代替编码器新鲜度检测 |
| /tf、/tf_static | tf2_msgs/msg/TFMessage | N/A | robot_state_publisher / TF broadcaster | 订阅只读 |

源码期望位置 rad、速度 rad/s；JointState.effort 直接复制 current，Nm/A 仍待确认。MotorState 无 joint names，需要与配置顺序绑定。DMMotor 电压缓存未从 protocol 更新，NotifyMotorFeedback 也未填 voltage，因此不能把该字段当成已验证电压反馈。

`/joint_states` 数量、当前角度、更新频率、稳定性、实际 URDF/TF tree：**全部未验证**。没有为了填表生成假状态或启动 simulation。

### Command

| Topic | Type | 作用 | 风险 |
|---|---|---|---|
| /student/joint_command | sensor_msgs/msg/JointState | q/dq 目标，按数组顺序接受 | 位置运动；名称字段不负责重排 |
| /student/joint_effort_command | dummy_interface/msg/MotorControl | p,d,position,velocity,current,enable | MIT 风格 PD+前馈；可施力/运动/切入保位 |
| /joint_states | sensor_msgs/msg/JointState | arm_hardware_node 把它当 MoveIt 目标 | 发布可直接驱动机械臂，不是全仓库统一反馈通道 |
| /master/joint_states | sensor_msgs/msg/JointState | slave 的目标输入 | 遥操作运动 |
| /slave/joint_states | sensor_msgs/msg/JointState | master 读取 effort/velocity 作为力反馈数据 | 当前回调只缓存，未见连接 SetForceFeedback；仍禁止注入 |
| /goal_pose | geometry_msgs/msg/PoseStamped | LL100 控制 demo 的目标链 | 控制器存在时可能间接运动 |

未找到独立命名的 torque/current/velocity 命令 topic；相关字段复用 MotorControl。未找到项目自定义电机控制 service server；当前服务清单为空。任何运行节点的 `set_parameters*`、生命周期或外部驱动服务仍需逐项审查，不能由此认定安全。

## 3. Control capabilities

- **position**：有，SmoothPositionController + MIT Kp/Kd；不是已验证的固件独立 position mode。
- **velocity**：有 dq feedforward 和协议 0x11，但未确认独立 velocity mode；不应宣传为已验证速度闭环接口。
- **torque/current**：有 0x12 前馈通路和 direct-effort 接口；历史 YAML 支持按 Nm 使用，当前真机/固件/单位未认证。
- **gravity compensation**：C++ GravityController 用 Pinocchio `computeGeneralizedGravity`；默认 p/d 为零；软限位与力矩 clamp 调用在该控制器中被注释。DMMotor 仍会限幅并在接近限位时强制恢复默认 Kp/Kd。
- **impedance/compliance**：可组合 `tau_ff + Kp(q_ref-q) + Kd(dq_ref-dq)`（以固件采用此语义为前提）。已有分级低刚度试验，未找到可直接部署且经过本次验证的持续手推/松手锁存模式。
- **SMC**：`core/model.py` 的 RNEA/重力/armature、`core/task_space_smc.py`、`core/friction_feedforward.py` 可复用计算；SMC runner 是运动控制器，不能为调查而启动。
- **actuator_adapter**：有 Nm/A 路径、力矩和变化率限制、超时/estop 标志；它返回 disabled/zero 不意味着整个硬件链断能。学生节点 enable=0 会改发保位。

源码：`src/arm-platform/src/controller/{gravity_controller,smooth_position_controller}.cc`、`src/arm-platform/src/robotics/motor/dm_motor.cc`、`src/arm-platform/task_space_smc/core/{model,actuator_adapter,task_space_smc,friction_feedforward}.py`。

## 4. RViz

目标链：真实 encoder → 经确认的只读反馈源 → 编码器/模型坐标映射 → JointState → robot_state_publisher → TF → RViz。

已有未跟踪文件（本轮开始前存在，保留原内容）：

- `src/arm-platform/launch/real_arm_readonly_rviz.launch.py`
- `src/arm-platform/config/real_arm_readonly.rviz`

审查结论：launch 只创建 RSP 和 RViz，不创建驱动、joint_state_publisher、controller 或轨迹节点。RViz 配置只有 Grid/RobotModel/TF 和相机/选择工具。launch 校验 URDF 活动关节为 joint1..6，默认订阅 `/joint_states`，Fixed Frame 为 base_link。

静态验证：Python AST、RViz YAML、URDF XML 通过；URDF 引用的 mesh 在仓库中全部存在。未导入/执行 launch。当前 ament 找不到 dummy_description，运行前需构建/加载正确 overlay。

**同步未完成：没有安全反馈源，也没有实际角度与模型匹配证据。没有启动 RViz 用静态模型冒充真机同步。** 直接把当前学生反馈送 RSP 只会显示编码器空间；历史零偏未应用，尤其 j2 约 -17°。确认标定有效后，应增加仅订阅原始反馈、向专用 `/real_arm_readonly/model_joint_states` 发布的映射节点，RSP 重映射到该话题；该专用话题不得接任何硬件控制订阅者。不要把校正角回发 `/joint_states`，避免 arm_hardware_node 把它当命令。

后续验收：确认真实发布者和配置→按 name 重排六关节、检查有限值/长度/rad→统计数秒频率/年龄/角度范围→核实 TF→在用户自行确认可安全手动移动的状态下观察模型是否对应。此报告不请求用户现在推动未确认状态的机械臂。

## 5. Gravity compensation

### 5.1 需求分类

以 `M(q)qdd + C(q,qd)qd + g(q) = tau + tau_ext` 为符号约定：

- 电机 disabled/zero torque：不抵消重力，可能下落，不能实现目标。
- `tau=g(q)`：理想模型中静止可平衡，但没有主动阻尼或位置恢复；受到推动后不会保证停住，实际误差/摩擦会导致漂移、卡滞或下坠。
- `tau=g(q)-D qd`：更接近“手推可移、松手渐停”；但残余模型误差仍可造成漂移，不保证精确保位。
- 固定参考的低刚度阻抗：松手回到原参考附近，不一定停在新位置。
- **更匹配完整需求：拖动时重力补偿+阻尼，释放后锁存新位置并平滑加入适度刚度。** 需要可靠释放判据（优先人工手推开关/接触信号；仅速度低不能区分被人静态压住和已释放）。
- admittance 需要可信外力测量/估计，再生成位置/速度目标，当前不是最小修改方案。

以上控制分类参照 [MIT Manipulator Control](https://manipulation.mit.edu/force.html) 的重力补偿与刚度/阻尼控制；“释放锁存”的选择是针对本项目需求的设计判断。

### 5.2 已有实现与证据局限

`phase4_gravity_compensation.py` 每级稳定 3s、记录 4s，100 Hz，Kp 比例 1/.5/.2/.05/0；L4 Kd=[.2,.2,.2,.02,.02,.02]。默认捕获当前姿态；显式 `--posture` 会主动就位。漂移阈值 .2rad、速度 .5rad/s、桌面/围栏与限位保护是代码已有值，**并非本机批准值**。

历史标定 YAML 和交接文档声称有 torque 标度、重力、惯量实机实验；本 checkout 未找到所引用的 phase2/3/4/5 原始 JSON 结果，也未找到足以认证当前配置 L4 成功的运行证据。交接文档提供的是流程和判据。不能结论“当前真机已经验证重力补偿”。

发现的阻塞问题：

1. 正式 YAML 和 arm6 URDF 明确说明质量矩修正已经烘入 URDF；Phase4 `feedforward()` 又乘 `first_moment_ratio`，SMC `model.py::_apply_gravity_ratio` 同样再乘。按正式配置会重复修正（前三轴倍率 1.265/1.349/1.228）。candidate 与正式值不同，不能盲目换一个文件解决。
2. C++ fallback 使用偏置但未乘 ratio，与 Phase4/SMC 不同；进入/退出时重力前馈可能跳变。
3. Phase4 的“现在可以手推”输出之后立即销毁节点。常规 student 50ms 超时后变为 PD+重力保位，不会持续维持 L4。
4. Phase4 无独立 feedback timestamp 新鲜度闸门，循环可用缓存继续发命令；`wait_state` 不保证所有状态同帧/新鲜。driver heartbeat 存活也不能证明各通道位置新鲜。
5. direct startup 的“3 feedback frames”实际依据 ROS 发布回调计数，不能证明收到三帧独立新编码器帧。协议每类反馈都 Notify，且解码没有按 cmd 严格校验 payload 长度。
6. 近限位时 DMMotor 在 Kp=0 命令下自动恢复默认刚度并牵引到边界内，不是全域无刚度。
7. `enable=0`、消息超时、`ActuatorAdapter.estop` 与真实断能语义不一致；Ctrl-C 不能视为可靠急停，master 析构甚至会 Reset。
8. Phase4 dry-run 打印代码一部分误置于 `parse_posture` 的 return 后，不可依赖它生成完整审批计划。

### 5.3 最小改动设计（未执行控制）

先统一模型/标定版本和单位：给校准配置明确的 URDF hash、是否已修正的标志，单一实现计算 `g(q_encoder+offset)`，杜绝重复缩放。复用已审核重力计算与限幅组件，新建持续运行的手推控制状态机，默认仅计算/记录、不发布。状态为 WAIT_FEEDBACK → 人工批准后的 HOLD → 人工拖动开关下 GUIDE → RELEASE_HOLD；q_hold 只在释放时锁存，禁止每周期跟随下垂位置重新锁存。

GUIDE 使用 Kp=0、适度 Kd；RELEASE_HOLD 平滑提高 Kp，保持阻尼和一致重力前馈。限幅、力矩变化率、真实反馈年龄、全关节完整性、速度、限位、碰撞余量必须在发送前校验。失效策略需结合托举/制动和硬件断能能力设计；不能一概“零力矩最安全”。首阶段不添加强静摩擦补偿，避免起动冲击/误判外力。精确 damping/stiffness 需在身份、单位、姿态和负载确认后给出，不能从历史数组直接批准。

参数/标定：实际安装姿态与重力方向；六轴符号、零偏与展开规则；URDF 质量/质心/工具负载；0x12 单位/比例/关节扭矩限制；速度滤波和延迟；摩擦、温度、有效惯量/armature；反馈频率；释放判据；真实急停/制动能力。历史 stiction=[.725,1.775,.625,.125,.100,.100] Nm；j6 惯量未辨识，armature 借用 j5；这些是待复核历史参数。

### 5.4 人工审批步骤（未执行；当前不具备放行条件）

下一步应先补齐身份、反馈和串口控制线无副作用证据，再完成上述修正。当前**不建议批准任何重力动作**。以下是现有命令的影响说明，不是运行指令或已批准实验：

Step 1：若启动 `ros2 launch manipulator student_arm6.launch.py arm_type:=a_l1 port_name:=/dev/ttyUSB0 enable_direct_effort_command:=true use_rviz:=false`，将开启六轴硬件驱动；反馈就绪后即主动发送锁存保位，不会等待 Phase4 才施力。

Step 2：若随后运行 `python3 src/arm-platform/task_space_smc/hw/phase4_gravity_compensation.py --max-level 4`，将向 `/student/joint_effort_command` 发布 `dummy_interface/msg/MotorControl`，包含六轴 q_ref、零 dq_ref、Kp/Kd、重力前馈及 enable=1。无 `--posture` 时捕获当前 q；带该参数会额外就位，必须单独批准。

Step 3：影响 joint1..6；通过协议仍发送 7 通道帧。所谓模式是 MIT 风格 PD+前馈增益组合，不能声称已经识别固件模式切换。逐级把六轴 Kp 降到零，仅留 Kd 和重力前馈。

Step 4：驱动初始 torque 来源是 arm6 URDF 的 g(q+硬编码 offset)，保位 Kp=[35,35,35,5,5,5]、Kd=[1.5,1.5,1.5,1,1,1]；Phase4 来源是 GravityModel 的 g(q+YAML offset)×ratio。两个来源现不一致，因此上述流程必须先修正再申请审批。

预期机械表现：启动保位即可能纠偏施力；低刚度时可被外力推动；误差可能导致抬升、下坠、漂移，结束后恢复高刚度保位。主要风险：重复补偿、坐标/单位错误、负载变化、饱和、末端掉落/碰撞、近限位强制增益、失联保持旧命令、停止时状态跳变。

紧急停止：**当前未确认硬件急停或制动方法，软件 stop/enable=0/Ctrl-C 不是已验证急停**。试验前必须由现场人员识别并确认实体急停/驱动断能路径，并有承重支撑以防失电下落；不要靠人把手伸入运动范围接住。没有这一条件不进入试验。

最后审批必须针对已修复版本、确切关节/姿态、增益/力矩包络、持续时间和退出方式；“继续执行”不构成运动授权。本轮在这里停止主动硬件动作。

## 6. Safety audit

本清单覆盖本 checkout 发现的直接硬件写路径、控制发布入口及主要包装器；静态搜索不是任意动态加载/外部固件的形式化无遗漏证明。未列出的程序不自动获得安全资格；归档副本与仿真名称也不豁免。

### 6.1 危险 launch / executable

- `student_arm.launch.py`、`student_arm6.launch.py` → student_arm_node。默认 sim 不代表与当前 ROS 域隔离；硬件 arm_type+direct 模式会自动保位。
- `master_arm.launch.py` → master_arm_node：持续重力输出；节点 auto_reset 默认 true，析构可执行回零。launch 也明确设置 auto_reset=True，退出同样危险。
- `slave_arm.launch.py` → slave_arm_node：跟随 master 目标。
- `moveit2_arm.launch.py` → arm_hardware_node：订阅 `/joint_states` 为控制。
- `demo.launch.py` → slave 等节点；`scripts/start.sh`、`robot.service` 会拉起它。
- `task_space_smc_whole.launch.py`、`task_space_smc_whole6.launch.py`、`task_space_smc_whole6_sim.launch.py` → student/SMC runner；共享 topic 可能连上其他硬件订阅者。
- `vicon_hold_demo.launch.py`、`vicon_moving_base_candidate_test.launch.py` → whole_task_smc_ros_runner.py，hardware_enabled 可开启硬件输出。
- `moving_base_uncertainty.launch.py` → student+运动 demo。
- `ll100_lh420_control.launch.py` → student/状态发布者；`ll100_lh420_display.launch.py` → joint_state_publisher_gui 或零状态发布者，遇到 arm_hardware_node 会成为命令源。
- `scripts/run_task_space_smc_whole.sh`、`demo/run_ab_experiment.sh`、`demo/run_ros_closed_loop_bench.py`、`demo/sweep_envelope_2d.py`：包装启动/实验脚本，需追到子进程，默认禁跑。
- `localization.launch.py` 会启动 MAVROS 并打开飞控串口；不属于批准的机械臂只读可视化链，禁止顺带启动。camera/lidar 不是机械臂反馈源。

四个核心硬件 executable：`student_arm_node`、`master_arm_node`、`slave_arm_node`、`arm_hardware_node`；底层出口 `AbsArm::SetMotorCommand → SerialBus::SendCore`。独立执行程序即使当前无目标，也具有收消息后下发能力；本轮没有启动。

### 6.2 危险 Python 入口

路径前缀 `src/arm-platform/task_space_smc/hw/`：

- `whole_task_smc_ros_runner.py`
- `torque_interface_probe.py`、`phase2_torque_probe.py`
- `phase3_gravity_calibration.py`、`phase4_gravity_compensation.py`、`phase5_inertia_id.py`
- `fk_gravity_verify.py`（名称 verify 但能发布位置命令）
- `full_prepose.py`、`wrist_prepose.py`
- `vicon_base_extrinsic_calibrate.py`（标定会发控制命令）
- `serial_tap.py`：存在串口双向转发/写路径，不是全模式只读。
- `pose_check_readonly.py`：不发控制字节，但 Serial() 打开及配置早于 `_forbid_writes`，DTR/RTS/开关端口的板端影响未确认；本轮未运行。不能靠名称判安全。

路径前缀 `src/arm-platform/demo/`：

- `move_arm_demo.py`、`move_arm_line_demo.py`、`move_arm_ik_demo.py`、`move_arm_mpc_ik_demo.py`
- `move_base_circle_visual_demo.py`、`move_base_circle_mpc_ik_demo.py`、`move_ll100_lh420_nmpc_demo.py`
- `ll100_traj_pub.py` 发布 goal_pose，可能间接产生运动。
- `ll100_lh420_zero_joint_state_publisher.py`、`ll100_lh420_rotor_joint_state_publisher.py`、`ll100_lh420_plant_node.py` 发布 `/joint_states`，当前仓库的硬件节点可能消费为命令。

归档 `docs/arm_calibration_handoff_20260814/code/task_space_smc/hw/` 中同名控制脚本也危险，另有 `phase3_j5_leso_observer_tune.py`；归档 launch 中 `student_arm6.launch.py`、`task_space_smc_whole.launch.py`、`vicon_hold_demo.launch.py` 同样不得直接运行。

### 6.3 Topic / service / 停止语义

危险 topic 列于第 2 节；另外，任意重映射到同一输入的名称也同样危险。不能使用 joint_state_publisher GUI、ros2 bag play 或任何假关节状态向真实控制输入注入消息。

未发现自定义控制 service server；发现的 get_parameters 是只读查询。修改参数不在本轮范围。不要调用未审查服务。

**停止进程也可能有机械副作用**：master 析构回零；Phase4 退出后 driver 超时保位；step_on_effort_command 分支还需单独审核其超时行为，不能假定所有模式有定时兜底。本轮没有 kill/重启任何已有进程。

## 7. 本轮变更与未完成项

仅新增本报告及静态审计索引/现场记录；保留原有未跟踪 RViz launch/config。没有改控制源码、YAML 标定、系统硬件配置或服务；未编译/运行任何 hardware demo。

完成：源码反馈/命令路径、模型关系、协议、控制能力与缺陷、现场 USB/ROS/进程静态检查、可视化文件静态验证、重力设计与审批模板。

未完成且明确阻塞：真实型号/电机 ID/零位/工具核实；固件和板内总线；真实反馈采样；RViz 真机同步；当前硬件重力验收。需要实体身份资料及可证明无输出/无控制线副作用的反馈链，不能通过擅自启用控制来补齐。

附：现场命令记录见 [OBSERVATIONS.txt](OBSERVATIONS.txt)；带行号的源码路径索引见 [SOURCE_INDEX.txt](SOURCE_INDEX.txt)。
