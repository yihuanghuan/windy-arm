# First active takeover — code and experiment review

## 审阅结论

专用Gate A程序已准备，**只完成离线回放和故障测试，未打开串口、未运行硬件入口、未发送命令**。用户此前已确认独立承重与可用动力隔离；固件内部行为仍为UNKNOWN。本轮只准备修订版，等待下一次明确批准，不继承旧版执行授权。

程序：[first_active_takeover.py](../../../src/arm-platform/tools/first_active_takeover.py)。测试：[test_first_active_takeover.py](../../../src/arm-platform/test/core/test_first_active_takeover.py)。实验顺序、唯一候选主动命令和三层退出见 [FIRST_ACTIVE_CONTROL_PLAN.md](FIRST_ACTIVE_CONTROL_PLAN.md)。

## 范围与可审计性

- 单文件、标准库+已审计的纯Decoder；不导入ROS/SMC/Phase4/GravityController，也不构建MotorControl对象。
- GateA是纯状态机；candidate packet由tick返回而不发送。`--replay`只读保存的chunks日志、输出JSON；`--dry-run`回放至PRE_SEND并保存真实待发快照，永不进入live。
- 独立live入口有且仅有一个`os.write(fd, data)`位置。没有隐式send、自动重连、serial请求、reset、enable、trajectory或改变目标接口。
- 启动先验证review，现场必须TTY键入确认；再打开可写串口、纯读qualify窗口，最后才可能有第一笔TX。不是“启动就自动保位”的student节点。
- 写前核查反馈年龄/状态机、新position、调度延迟和内核TX队列；短写直接abort，不把剩余bytes当下一批续写。
- 正常/异常结束均没有追加“退出控制命令”；不伪造disable。日志区分`TX_KERNEL_ACCEPTED`与板端ACK（本协议没有已知ACK）。进程崩溃可能来不及写最后日志，日志不能证明板端最终状态。
- q_hold从启动后的反馈一次锁存，用gamma6 sign [+,+,+,-,-,-]；为保留wire winding，不先对原始角模2π。超既有限位余量时拒绝而非改目标。
- q保持固定不是证明六轴不会运动：Kp/Kd激活、旧前馈、逐帧更新和模式切换仍可能改变受力。

## Kp/Kd候选与依据

采用仓库 `task_space_smc/hw/phase4_gravity_compensation.py` 的历史L2：BASE_P=[30,30,30,5,5,5]×0.20，BASE_D=[1,1,1,0.1,0.1,0.1]×0.30，得到：

`Kp=[6,6,6,1,1,1]，Kd=[0.3,0.3,0.3,0.03,0.03,0.03]`。

不是运行Phase4，只引用其增益范围。相对于motor_config默认Kp30/5、Kd1.5/1，这减少位置误差和速度噪声对应的预期PD力矩。较低增益不保证新固件安全：若单位/语义不同，上述物理解释不成立。

**不使用gravity feedforward、alpha或历史offset。** 0x12仍发送七个0，理由是省略更新可能保留未知旧前馈；它只清零，不验证非零力矩标度。新固件“零0x12”的实际作用仍未主动验证，明确作为风险，而非伪造成已知或自动阻塞准备。

低增益＋零g不能承担完整重力。例如当前模型J2的|g|约2.204Nm，以Kp6仅靠PD平衡需约0.367rad误差，远超本实验0.006134rad边界。静止时允许误差对应J2的弹性力矩仅0.036804Nm。**支撑必须承担几乎全部负载；绝不能以2秒很短、current=0或PD存在替代承重支撑。** 不以模型g作为本轮发送内容。

## Torque safety envelope及推导

proxy按当前反馈计算：

`tau_proxy = Kp*(q_hold-q) - Kd*dq + 0`。

所用参数来自本仓库/本机已有数据：

- 位置量化步长δq=0.000383377075rad，本机静态腕部窗口跨度约2δq；取允许位置误差16δq=0.006134033rad，是观察噪声跨度的8倍，远小于作者Phase2的0.15rad/Phase4的0.20rad abort。这是保守的可观测性/位移预算，不是机械安全极限。
- 50ms内连续位移上限4δq=0.001533508rad，约为已有静态跨度2倍；不把一个encoder量化跳变当持续运动。
- RS05速度量化δv=0.024421692rad/s；新固件静态捕获出现约0.23199rad/s，取向上整数12δv=0.293060303rad/s（约再留2个量化步长），同时用更紧的位置变化门槛。该限值小于作者Phase4的0.5rad/s。相同上限用于六轴，前三轴真实运动仍受位置判据约束。
- 最坏abs(proxy) = Kp*16δq + Kd*12δv。不是使用rated_torque当总力矩证明。
- proxy变化率在**相邻约50ms窗口均值**之间计算：abs(Δmean_proxy)/Δt ≤ proxy_bound/0.05。对应从零到包络边缘不快于50ms；50ms取自仓库direct-effort timeout。用窗口均值避免把已知速度量化瞬变当真实力矩slew；原始瞬时proxy仍逐帧受幅值约束。
- joint limit margin=0.15rad，来源历史FK/Phase4姿态检查，严于DMMotor近限位0.05rad分支。
- 四类feedback stale=50ms，约10个200.9Hz反馈周期；100Hz目标发送周期，发送前position age≤10ms且必须有新样本。

| Joint | Kp | Kd | worst-case abs(proxy) | 50ms均值slew上限 | 参考q(rad) | 最近限位余量(rad) |
|---|---:|---:|---:|---:|---:|---:|
| J1 | 6 | 0.3 | 0.124722290 | 2.494445801 | -0.031854 | 3.968146 |
| J2 | 6 | 0.3 | 0.124722290 | 2.494445801 | 1.356337 | 0.643663 |
| J3 | 6 | 0.3 | 0.124722290 | 2.494445801 | -1.334973 | 0.235827 |
| J4 | 1 | 0.03 | 0.014925842 | 0.298516846 | 0.001726 | 3.998274 |
| J5 | 1 | 0.03 | 0.014925842 | 0.298516846 | 0.007481 | 1.992519 |
| J6 | 1 | 0.03 | 0.014925842 | 0.298516846 | 0.001726 | 3.998274 |

Nm/Nm/s均是MIT单位假设下的proxy，不是实测真实力矩。程序没有声称限制板内瞬时力矩slew，不能凭50ms均值排除更快峰值。固定packet令新的期望目标/增益/前馈在第一批之后不再跳变，但第一批与旧模式之间的跳变量未知，板端限流仍未知，必须依赖切换风险审阅、物理支撑和现场动力隔离。

原current rated clamp只涉及feedforward；本程序tau_ff=0仍可产生非零PD。旧固件历史rated=[9,9,9,1.6,1.6,1.6]仅作参照，不是本轮已配置或已验证的板端总限制。

## 有效性窗口、异常与退出

完整条件见计划及代码常量。连续qualify约1秒、每类至少180样本；四类合法FE55、slot0..5有限、temperature非零。温度零导致保守拒绝，不将它推广成通用固件故障码。按host read时间衡量新鲜度，不假称已知道板端采样时间/内部总线延迟。

checksum/长度/非有限、反馈缺失、位置漂移/跳变、原始速度越界、proxy幅值或均值slew越界、关节限位、调度停顿、端口错误/部分TX均停止新增输出，无fallback controller。ABORT的关闭串口不使机械臂安全；现场人立即按支撑条件下的动力隔离方案处置。

正常最后完整控制状态：q_hold、vd=0、ff=0、上述Kp/Kd，最后完整帧通常0x14。关闭后的保持/失能/故障行为无法由仓库确定。崩溃或短写可能只发到部分帧；内核队列已接收的数据、USB硬件在途数据不能撤回。

ROS不参与，ROS停止而串口仍开着时程序可继续到2秒；程序卡死时软件deadline不起作用。SIGINT/SIGTERM只进行软件abort，SIGKILL不能保证finally。必须具备可用的实体急停/断电最终边界，人员不得离场。

NORMAL EXIT：停发/close→人切动力→确认支撑承重。
ABORT EXIT：停发/close（若程序仍可运行）→人立即切动力；不重试/不发保位修正。
POWER-OFF FALLBACK：程序无响应、USB/ROS/主机异常、无法确定最后状态时，人绕过软件切电机动力；设备已由支撑承载。

## Revision 3：首帧落盘门控与离线验证

程序 SHA256：`cc9f022002a2232fa15340d5f55125527369947dfa41bd8cbcd9236047e695cd`。

`QUALIFY → PRE_SEND → 快照校验/落盘/打印 → 写前复查 → 首批完整内核接受 → ACTIVE`。

- snapshot直接解码待发packet，核对五帧checksum、顺序、长度、slot6=0、固定增益、全零速度/FF、q_hold一致性。
- 首帧position与当前已读measured q严格相等。双方来自同一float32协议坐标；没有offset、重采样或角度取模，无需容差。即使偏差小于既有position guard，首帧不相等仍拒绝。
- measured q/dq、各类反馈接收时间/年龄、命令、误差、每轴margin/guard、P/D/FF/proxy及其最大占比全部写入PRE_SEND_SNAPSHOT并打印。时间是host monotonic接收时间，不是板内采样时间。
- 独占创建JSON，flush→fsync→读回一致；主JSONL也保存快照并fsync。路径冲突、写入失败、fsync失败、读回不一致、终端打印异常均不能进入第一次write。此前某次留下的snapshot不能授权本次发送。
- 落盘后再检查新鲜度及10ms写前延迟；慢磁盘/终端会安全拒绝，不放宽阈值。系统调用之间仍有主机调度窗口，非硬实时安全控制器。
- 首帧相对于未知旧板端力矩的slew无法计算，明确UNKNOWN；不把它伪造为0。首帧proxy建立后，沿用50ms均值slew检查。
- exact firmware version / board total torque limit / 0x12 scale / disconnect / persistent state 均为UNKNOWN。review需要风险接受，不能填写alpha=1.033或假认证以越过联锁。批准模板仍为全false。

**31项测试通过**，包含用户要求的10类fail-closed场景。proxy独立越界测试临时注入更紧测试边界，仅在测试进程内：因为原proxy限值本身由position/velocity最坏界推导，合法分量不会数学上单独越界。不修改运行常量。

两套保存真实反馈完整离线回放均DONE、所有候选packet恒定；这只是模拟状态结束，**不是Gate A实验PASS**。`--dry-run`停于PRE_SEND，输出READY TO WRITE；系统调用核查无串口设备open/write。测试额外将os.open/os.write/live/select替换为失败哨兵，dry-run仍通过。

证据：[测试](takeover/prewrite_v3/tests.txt)、[最终快照](takeover/prewrite_v3/pre_send_snapshot_final.json)、[dry-run输出](takeover/prewrite_v3/dry_run_final.txt)、[syscalls审计](takeover/prewrite_v3/syscall_audit.json)、[固件capture回放](takeover/prewrite_v3/replay_firmware.json)、[ROS capture回放](takeover/prewrite_v3/replay_ros.json)。旧takeover/tests.txt是此前17项测试的历史证据，不代表本版本。

## 首帧顺序及残余风险

保留0x10→0x11→0x12→0x13→0x14，固定低增益，不新增零增益或爬坡阶段。详见计划中的逐帧分析：q_cmd=q只压低旧Kp的误差项，不能消除旧Kd×dq、旧速度目标、旧FF、采样延迟及模式切换风险。零增益按原顺序依然晚到；调整顺序/渐变又依赖未知板端语义，当前证据无法证明更安全。

未知固件信息不是代码准备阻塞项，但也没有消失。物理承重必须能应对失力或暂态；人可绕过软件立即动力隔离。50ms均值proxy绝不是板内总力矩限幅或瞬态峰值保证。

## Gate A无法证明的事

- 固定qd=q、vd=0不能独立确认position/velocity符号或Kp/Kd实际标度；Gate A仅检查此姿态下基本静止兼容性及明显接管跳变。方向验证仍属Gate B。
- 0x12全零不能验证torque sign/scale/alpha；留给Gate C。
- 静止可由旧控制器、制动或支撑维持；没有跳变不证明新命令已生效。本轮不得用主动运动来排除这些解释。
- close(serial)、Ctrl-C、enable=0均不证明失能。正常/abort/crash最终状态未知，现场动力隔离是最终边界。
- review布尔项不是传感器；不能由Codex自动改为true。程序hash变化后必须重新审批。

READY FOR HUMAN REVIEW — GATE A NOT EXECUTED
