# FIRST ACTIVE CONTROL PLAN — Revision 3: durable pre-write gate

本版替代上一版“带重力前馈的静止接管候选”。**第一次不使用重力前馈，不启动student_arm_node、Phase4、SMC或任何旧hardware launch。** Gate A/B/C分别审批，不能合并。

状态：**本轮仅离线准备，修订版等待下一次人工批准，未执行 Gate A**。用户此前已确认独立承重和实体动力隔离；本轮没有重新验证现场，也没有打开串口。UNKNOWN 固件信息作为 residual risk，不再自动阻塞代码准备。程序与完整参数依据见 [FIRST_ACTIVE_TAKEOVER_REVIEW.md](FIRST_ACTIVE_TAKEOVER_REVIEW.md)。

## Gate A — Static takeover

唯一目标：把真实、当前反馈位置作为恒定目标，观察两秒内是否跳变。没有预姿态、轨迹、回零、手推或重力试验。静态零误差/零速度目标无法独立识别position sign、velocity sign、Kp/Kd数值标度；成功仅说明该姿态下未出现明显接管异常。

专用程序：`src/arm-platform/tools/first_active_takeover.py`。没有ROS依赖，没有MotorControl publisher，不使用任何旧controller。离线入口为 `--replay` 或 `--dry-run`；硬件入口需要完整review文件、当前会话动作授权记录和现场终端键入确认。批准文件模板全部false，不能自动补齐；须明确接受未确认板端行为的风险，并绑定被审阅程序的SHA-256。不得把风险接受伪造成固件认证。

### 实验前硬条件

- 用户现场、明确批准这一次Gate A动作及六轴命令、持续时间、退出步骤；Codex不可自行开始。
- **全部活动连杆/末端已被可靠承重支撑**，支撑能在电机失力或短暂反向力矩时保持安全，不是只固定底座、也不是让人手托。工作区无障碍、无人处在夹点/坠落范围；必须允许观察小位移，不能用强硬夹死掩盖巨大的控制力。
- 现场已确认可立即切断电机动力的实体急停/电源隔离位置和步骤；急停是否切动力、是否另需电源隔离必须明确。断电后的臂由支撑承载。
- 已确认槽位/旋向、反馈兼容性及作者主机帧格式；新板命令实际语义、精确版本、内部总限幅和状态保持仍有未知。必须人工审阅下述风险；不能要求虚构“已知版本”作为联锁。
- 当前锁定模式/旧目标/旧增益切换风险已审阅。五帧并非原子事务；未知时不得从“目标相同”推导“安全”。
- 对关闭串口/crash的板端行为若仍无法确认，最终边界明确为现场动力隔离；不能以2秒时长替代物理措施。
- 独占/dev/ttyUSB0；关闭只读receiver及其他端口持有者。TTY打开本身的控制线影响也属于本次主动授权范围。

### 时序

1. 读取完整review JSON、要求现场确认，尚未打开设备。日志文件必须能独占创建。
2. 打开指定`/dev/ttyUSB0`，921600/raw、O_RDWR、TIOCEXCL；**仍无TX**，仅被动读主动FE55，不请求/唤醒。
3. 连续约1秒确认0x20/21/22/24，每种至少180帧，六轴有限、温度非零、50ms新鲜度、位置窗口覆盖≥0.9s且至少180样本；位置静止范围≤4个encoder量化步长，速度/限位检查通过。启动2秒无完整主动反馈即退出。任何实际反馈中断不自动重连。
4. 锁存`q_hold=当前六轴实际反馈`，不加zero_offset，不读nominal pose。使用protocol raw角×已确认旋向；不先模2π再发送，以免改变绕圈表示。
5. 进入 `PRE_SEND`，尚非 ACTIVE：解码实际待发字节，严格检查首个 position 与最新已读 q / q_hold 完全相等（同一float32坐标，容差0）；计算 P/D/FF/total proxy、限位余量和反馈年龄。独占创建 `.pre_send.json`，flush、fsync、读回核对并打印；主日志同步保存快照与摘要且 fsync。任一步异常或无法确认落盘，ABORT BEFORE FIRST WRITE。之后重新检查年龄、10ms延迟、实际packet及guards。第一批完整write被内核接受后才进入ACTIVE。最多两秒Gate A；最多100Hz，每次必须有新position且年龄≤10ms。所有候选packet逐字节相同，target从不改变、不追随漂移。
6. 两秒到期NORMAL EXIT：**停止TX、关闭串口**，不发送退出/disable/零增益帧。现场执行约定的动力隔离，确认支撑承重。板端可能保留最后目标，不能把close当实验已安全解除。

### 初始命令（启动时实际数值取新鲜反馈）

已保存姿态只是审批参考：
`[-0.031854,1.356337,-1.334973,0.001726,0.007481,0.001726] rad`。

- position = 锁存q_hold。
- velocity = `[0,0,0,0,0,0]`。
- Kp = `[6,6,6,1,1,1]`。
- Kd = `[0.3,0.3,0.3,0.03,0.03,0.03]`。
- tau_ff = `[0,0,0,0,0,0]`，**不计算、不发送g(q)**。
- 没有enable字段，串口协议没有可靠的enable=0失能命令。

每批明确顺序：0x10 position → 0x11 velocity0 → **0x12全零** → 0x13 Kp → 0x14 Kd。后三轴position取反回wire坐标；所有帧7×float32，slot6=0。0x12的用途仅为避免残留旧前馈，绝不据此宣称通过Gate C；在新板上零值清前馈的实际效果尚未主动验证，必须作为风险接受，不能声称已证实。常规批170B，8N1/921600理想线时约1.845ms，USB/调度额外延迟及板端逐帧生效未界定。

### 明确保护包络（均为host观测/估计，不是板端总力矩认证）

`tau_proxy[i] = Kp[i]*(q_hold[i]-q[i]) + Kd[i]*(0-dq[i]) + 0`

| 参数 | J1–J3 | J4–J6 |
|---|---:|---:|
| abs(position error)上限 | 0.006134033rad | 0.006134033rad |
| abs(raw dq)上限 | 0.293060303rad/s | 0.293060303rad/s |
| abs(tau_proxy)最坏包络 | 0.124722290 | 0.014925842 |
| 50ms均值proxy变化率上限 | 2.494445801/s | 0.298516846/s |

Nm/Nm/s解释以板端MIT物理单位假设为条件。proxy不是传感器实测力矩；尤其旧命令到首批生效前的瞬态不受这些界限证明。

额外abort：50ms位置窗口（至少覆盖40ms）持续位移>4量化步长=0.001533508rad；相邻位置跳变>0.006134033rad；任一joint距限位<0.15rad；checksum/malformed/nonfinite/温度零；四种反馈任一年龄>50ms；控制线程停顿>50ms；非单调时钟；串口EOF/写错误/短写；TX队列未清空；写前额外延迟>10ms。速度超过包络立即abort；单帧0.1587或0.232rad/s尖峰不会单凭数值触发，因为已在已有静态反馈出现。全部异常退出都不发送“修正”动作、不重试、不重新打开。

人工看到非预期运动、受力、异响、支撑偏移、板卡/电机异常时，**直接执行实体急停/动力隔离**，不等待软件量化门槛或终端响应。

### NORMAL EXIT / ABORT EXIT / POWER-OFF FALLBACK

- NORMAL EXIT：正常情况下最后完整帧是最终批次的0x14 Kd，最后已请求状态仍是固定q_hold+PD+零前馈。之后主机停发并close；用户按约定切动力。没有“最后一帧失能”。
- ABORT EXIT：立即停止新增TX并close，不试图改变q_hold或重发g/PD。不宣称最后批完整：短写、崩溃或USB错误时可能只到某个帧/字节；内核accepted日志不是控制板ACK，已经在途的数据不能撤回。现场立即动力隔离。
- POWER-OFF FALLBACK：进程卡死/SIGKILL/主机掉电、串口持续打开但无命令、ROS退出、USB异常等均可能使最后控制状态持续或触发未知板端watchdog。唯一已安排的最终边界是**人在场+承重支撑+实体动力隔离**。未确认这些条件则不能启动。

本程序与ROS无关，ROS停止并不会使它退出；应针对该进程/物理动力停止。SIGINT/SIGTERM被当作abort处理，但不是电机失能，不能替代POWER-OFF FALLBACK。

## Gate B — Tiny joint command validation（仅在Gate A成功后另行设计/审批）

不在本轮程序中实现。届时依据实际支撑、工具和碰撞空间选择低重力影响轴；J6无工具模型下g很小，只是候选，末端附加物可改变风险。必须对位置符号、小目标响应和增益作用作独立验证，不能因A成功就自动发送小动作。幅度、方向、增益、次数、退出路径需重新审批。

## Gate C — Torque/feedforward validation（仅在A/B成功后）

另行验证非零0x12的sign、actual torque scale和alpha。不得自动继承作者1.033；此后才决定重力补偿/手推是否可进入准备验证。

## Revision 3 证据与发送顺序决定

当前程序 SHA256：`cc9f022002a2232fa15340d5f55125527369947dfa41bd8cbcd9236047e695cd`。

详见 [GATE_A_PREWRITE_CHECKLIST.md](GATE_A_PREWRITE_CHECKLIST.md)。本轮不改变 Kp/Kd、任何 guard、目标或2秒上限；只增加写前门控、快照及纯离线入口。此前旧版本的执行授权不适用于本修订版本。

`ProtocolV1::Pop` 源码依次组合 0x10→0x11→0x12→0x13→0x14；独立程序同序一次提交170B。一次write不代表板端原子生效。position先更新时，q_cmd=q_measured降低旧Kp乘位置误差这一项；采样后位移、旧velocity目标、旧Kd、旧FF仍可能产生力矩。随后vd=0时，旧Kd仍乘当前非零dq；直到0x12才请求清FF，直到0x13/14才请求更新增益。未知旧增益不能由新增益proxy限制。

**拟保留现有顺序，不新增Stage1零增益和Stage2爬坡。** 零增益在当前五帧顺序中也是最后才到，并不能消除前面旧状态窗口；若改为先发零增益，则额外依赖未验证的逐帧生效/模式语义，还会主动撤去原有支撑力。爬坡增加中间状态、持续时间与实验变量，板端无已知原子提交/ACK，无法证明整体更安全。可靠机械承重与实体动力隔离是边界；此决定不等于证明原顺序无瞬态。未来有固件证据后可另行审阅替代时序。

首帧proxy是新命令的估算，**首帧相对旧板端状态的真实力矩slew为UNKNOWN**。首帧后开始累积已审阅的50ms均值slew历史，不伪造旧力矩=0或预先声称slew通过。

## 唯一候选主动命令（本轮未执行）

从仓库根目录；review文件必须由实际现场审阅形成，本轮只有全false模板，不存在获准的review：

```bash
python3 src/arm-platform/tools/first_active_takeover.py --execute-gate-a --review-file docs/real_arm_audit_20260917/control_preparation/takeover/gate_a_review.approved.json --port /dev/ttyUSB0 --log docs/real_arm_audit_20260917/control_preparation/takeover/gate_a_active.jsonl
```

不能从本文件的候选命令推断已获得执行授权。首次静止接管、Gate B、Gate C各自必须收到用户明确批准。

READY FOR HUMAN REVIEW — GATE A NOT EXECUTED
