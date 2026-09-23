# Gate A pre-write checklist — Revision 3

READY FOR HUMAN REVIEW — GATE A NOT EXECUTED

本轮仅修改代码、离线测试和保存反馈回放。没有打开真实串口、没有发送任何0x10–0x14、没有启动hardware controller。旧版GATE_A_RESULT.md保留上轮ABORT BEFORE FIRST COMMAND历史，不改写成成功。新版本需要下一次人工批准。

## CONFIRMED（注明证据边界）

| 项目 | 证据与边界 |
|---|---|
| feedback mapping | 新硬件已保存15秒反馈，physical joint1–6 = slot0–5；slot6不用 |
| q sign | gamma6 [+,+,+,-,-,-]；反馈旋向与现有模型链确认，不等于主动方向试验通过 |
| ROS/RViz | 约200.8Hz、TF完整、用户现场确认姿态一致；未使用历史zero offset |
| host protocol format | ProtocolV1::Pop源码顺序10/11/12/13/14，每帧7×float32；这是主机格式证据，不是新固件内部实现证据 |
| code hash | `cc9f022002a2232fa15340d5f55125527369947dfa41bd8cbcd9236047e695cd`；与最终dry-run快照及review模板一致 |
| physical support | 用户此前明确确认独立可靠承重；助手未重新现场验证，未来执行前由用户确认仍有效 |
| physical power isolation | 用户此前明确确认可操作实体动力隔离且本人在场；本轮未测试断电，未来执行需约定具体步骤 |

## UNKNOWN（不得改写为known）

- exact board firmware version = UNKNOWN
- board internal total torque limit = UNKNOWN
- 0x12 actual torque scale = UNKNOWN；不使用作者alpha=1.033
- disconnect behavior = UNKNOWN；close(serial)不等于disable
- persistent board state = UNKNOWN，包括旧模式、旧目标、旧Kp/Kd、旧FF、逐帧何时生效

这些不再自动阻塞Gate A代码准备；它们仍是主动接管的residual risk。风险接受必须在未来批准中明确，不能伪造固件证据。五帧非原子、旧Kd×dq/旧FF和首帧真实torque slew不受host proxy证明。

## 第一笔write之前（代码强制）

1. 批准文件绑定当前SHA；当前会话Gate A批准、现场人、独立承重、可切动力、端口独占、未知板端风险接受全部满足；当前模板全false，不是批准文件。
2. 被动FE55 qualify≥1秒、每类≥180帧、position窗口≥0.9秒；六轴finite、温度非零、checksum/length正常、非stale、静止和限位检查通过。
3. latch q_hold=current q；不套历史姿态、offset或角度取模。PRE_SEND尚不是ACTIVE。
4. 解码实际待发170B，核对顺序/checksum/slot6、固定Kp/Kd、vd=ff=0。首帧qd=q_hold=q_measured严格相等（同float32，容差0）。任何非零首帧误差即拒绝，比已有position guard更严格。
5. 计算每轴P、D、FF、total proxy，与原guard比较；检查关节margin、位置/速度guard、新鲜度。旧状态到首帧的真实slew无法计算，记录UNKNOWN；后续按原50ms均值slew判据。
6. 输出PRE_SEND_SNAPSHOT到独占JSON：write→flush→fsync→read-back→terminal print；live主日志追加该记录并fsync。任何失败，ABORT BEFORE FIRST WRITE，TX=0。
7. 完成落盘后重新检查packet、feedback age及10ms主机延迟；内核TX队列非空拒绝。慢磁盘/终端导致拒绝，不放宽阈值。
8. 只有上述条件通过才可调用唯一write；完整批次被内核接受才进入ACTIVE。accepted不等于板端ACK。两秒到期或guard失败停止新增TX，不发退出帧。

首帧snapshot的timestamp是host monotonic相对capture/start时间；新鲜度只反映host接收，无法证明板端采样或内部总线没有延迟。持久化使用文件fsync和读回；不宣称在磁盘/操作系统故障或掉电下具备审计完整性保证。

## 最终dry-run快照（保存的真实反馈，不是本轮现场读取）

来源：`ros_rviz_readonly/capture.chunks.jsonl`。t=1.000952736秒，最大feedback age=0.976580ms；position age同值。qualified position=204帧。

| Joint | measured q = commanded q (rad) | measured dq (rad/s) | P | D = total proxy | FF | margin (rad) |
|---|---:|---:|---:|---:|---:|---:|
| J1 | -0.031853675842 | -0.004884719849 | 0 | 0.001465415955 | 0 | 3.968146324158 |
| J2 | 1.356336593628 | -0.004884719849 | 0 | 0.001465415955 | 0 | 0.643663406372 |
| J3 | -1.334973335266 | -0.004884719849 | 0 | 0.001465415955 | 0 | 0.235826664734 |
| J4 | 0.001726150513 | 0.036628723145 | 0 | -0.001098861694 | 0 | 3.998273849487 |
| J5 | 0.007480621338 | 0.085468292236 | 0 | -0.002564048767 | 0 | 1.992519378662 |
| J6 | 0.001726150513 | -0.036628723145 | 0 | 0.001098861694 | 0 | 3.998273849487 |

position_command−measured_q = [0,0,0,0,0,0]。
velocity command = current/FF = [0,0,0,0,0,0]。
Kp=[6,6,6,1,1,1]；Kd=[0.3,0.3,0.3,0.03,0.03,0.03]，未变更。
max_abs_tau_proxy=0.002564048767；max_tau_proxy_ratio_to_guard=0.171785867632（17.1786%）。这些是新命令MIT语义假设下的proxy，不是板端真实力矩。

position-error guard=0.006134033203rad；velocity guard=0.293060302734rad/s；minimum margin=0.15rad。J3 margin=0.235826664734rad，超过最小值0.085826664734rad。

理论0x10（wire旋向）=[-0.031853675842,1.356336593628,-1.334973335266,-0.001726150513,-0.007480621338,-0.001726150513,0]。
理论0x11、0x12全7槽0；0x13=Kp+[0]、0x14=Kd+[0]。wire Kd有正常float32表示差异（0.3→0.300000011920929；0.03→0.029999999329448），不是临时改增益。完整bytes、时间戳、差分和数组见[最终快照](takeover/prewrite_v3/pre_send_snapshot_final.json)。

## 拟采用的接管顺序

capture当前q → qualify/门控/持久化 → 一批0x10→0x11→0x12→0x13→0x14 → 固定同一目标最多2秒 → 停发/close → 现场约定动力隔离。

不引入零增益stage或增益爬坡。理由：原顺序中零增益同样晚到，改序又依赖未知板端逐帧/模式语义，爬坡增加额外状态，不能从“平滑”推出更安全。qd=q降低旧Kp误差但无法保证无瞬态。详细分析见[计划](FIRST_ACTIVE_CONTROL_PLAN.md)。独立承重不能省略。

## Offline测试及系统调用证据

31项测试通过，包括：首帧q不一致（即使在原guard内）、stale、NaN、velocity、torque proxy、joint margin、UNKNOWN只记录风险、dry-run禁止open/write、pre-write异常TX=0、snapshot/fsync失败TX=0；另有packet校验、无反馈、连续窗口、恒定目标、原slew等回归。

固件capture模拟：185个候选批；ROS capture模拟：184个候选批；均DONE且所有候选packet恒定。无实际发送。

最终dry-run：serial open syscall=0，serial write syscall=0，socket/connect=0；0x10–0x14实际TX=0。见[系统调用核查](takeover/prewrite_v3/syscall_audit.json)、[trace](takeover/prewrite_v3/dry_run_final.syscalls.log)、[测试](takeover/prewrite_v3/tests.txt)。打印理论hex和写入本地审计文件不是串口TX。此计数范围是本次dry-run进程，不伪称系统全局不存在其他进程。

本轮可重跑的唯一类型命令（输出路径须新建）：

```bash
python3 src/arm-platform/tools/first_active_takeover.py --dry-run --capture docs/real_arm_audit_20260917/ros_rviz_readonly/capture.chunks.jsonl --snapshot /tmp/gate_a_review_fresh_snapshot.json
```

## Gate A范围与退出

验证：静止接管有无明显跳变；观察0x10/11/13/14在该姿态下的基本兼容性。静止试验不能独立证明符号/增益标度或命令已生效。
不验证：0x12 scale、gravity compensation、free guidance、Gate B/C。

NORMAL：2秒停发/close，用户按约定动力隔离。ABORT：停发/close、不补发、不reset、不改变增益，用户及时动力隔离。POWER-OFF FALLBACK：进程crash/卡死、USB异常等由人绕过软件切动力。最后命令仍可能在板上保持；关闭串口不证明电机失能。

READY FOR HUMAN REVIEW — GATE A NOT EXECUTED
