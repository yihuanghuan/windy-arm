# 最新补充：针对 J5 的一次人工批准重测 — FAIL

保留原报告及 SUPPORT_A_B_COMPARISON 于下方。本次重新授权的唯一一次重测在约 **40.6ms** 触发 **J5** 位移guard，未完成2秒。J5变化 **0.00153446rad（约0.088°）**，J2/J3在该窗口无可分辨位置变化。已停发、关闭串口，不自动重试。

程序SHA256未变，所有控制参数/guard未变。0x10–0x14各4帧；checksum/malformed/stale=0。

本次J3限位余量 **0.154953rad**，仅比原最小余量多 **0.004953rad**。起始姿态与此前不同，需保留为风险/比较限制。现场已确认J5真实小范围移动；支撑实际仅架住J2–J3之间连杆，未确认J5及下游独立承重。不能把J5变化仅解释为反馈量化，重力/接管受力变化仍是待验证假设。实体动力隔离完成情况尚未确认。

[本次详细结果](takeover/j5_observed_retry_20260917/RESULT.md) · [本次pre-write snapshot](takeover/j5_observed_retry_20260917/run1.jsonl.pre_send.json) · [本次位移图](gate_a_plots/j5_observed_retry_displacement.png)

顶层JSON和CSV已追加 `external_support_j5_observed_retry`，此前两组数据完整保留。

---

# GATE A RESULT — WITH EXTERNAL SUPPORT

**FAIL — Run 1 guard abort；Run 2/3未执行。**

用户纠正了上一轮现场前提：此前并无独立机械承重，主要依靠机械臂自身保持姿态。本轮用户明确确认已有稳定外部支撑承托远端/前臂。控制程序与上轮逐字节一致，SHA256：`cc9f022002a2232fa15340d5f55125527369947dfa41bd8cbcd9236047e695cd`。本轮未改增益、目标逻辑、guard、频率、超时、mapping或发送顺序。

## 执行与首帧

有支撑Run 1计划2秒；首次TX调度后约36.539ms，joint5触发50ms连续位移guard，立即停止新增发送并关闭串口。没有自动重试。
q_hold（本轮现场重新测量）=[-0.031853675842285156, 1.4929046630859375, -1.3586254119873047, 0.0024938583374023438, 0.08496952056884766, 0.0024938583374023438] rad。
command q=measured q=q_hold，六轴初始差值均为0；velocity command=current/FF=[0,0,0,0,0,0]；Kp=[6,6,6,1,1,1]，Kd=[0.3,0.3,0.3,0.03,0.03,0.03]。
首帧P=FF=0；D=total proxy=[0.0014654159545898437, 0.0014654159545898437, 0.0014654159545898437, 0.0010988616943359375, -0.00036632537841796875, 0.00036632537841796875]。最大proxy=0.001465415955，最大proxy guard占比7.3621%；首帧最大feedback age=3.431287ms，J3 margin=0.212174588013rad。
[PRE_SEND_SNAPSHOT](takeover/external_support_20260917/run1.jsonl.pre_send.json)已在第一次write之前持久化。资格窗口通过，六轴四类必需反馈有效且无NaN/Inf/校验/长度/stale错误。

## SUPPORT_A_B_COMPARISON

**核心回答：J2在已观察到的约30ms短时范围内位移显著减小，但未消失；本轮因J5 guard中止，不能声称有支撑时可稳定完成2秒。**
在同一30ms截止时间，J2 support/no-support位移比=0.333333，减小66.67%。在上次abort时间30.695669ms截止，比例=0.200000，减小80.00%。两种比例都使用各自最后一个已收到position样本，不插值，不把不同终止时刻直接当成同一时刻。
这支持外部承重对J2短时表现有帮助的解释；没有足够证据认定上次失败主要原因已被确定，更不能证明position/Kp/Kd方向正确或错误。

**严格A/B的限制：初始姿态并不相同。** 虽未改任何控制参数且每次都按授权实时锁存q_hold，以下姿态变化改变了重力/接触条件。该差异是在接管前反馈中发现，不是本程序施加的位置偏移；未执行恢复姿态动作。未知persistent board state同样未被控制。

| Joint | 无支撑q_hold | 有支撑q_hold | 差值rad | 差值deg |
|---|---:|---:|---:|---:|
| J1 | -0.031853676 | -0.031853676 | 0.000000000 | 0.000000 |
| J2 | 1.356336594 | 1.492904663 | 0.136568069 | 7.824774 |
| J3 | -1.334973335 | -1.358625412 | -0.023652077 | -1.355164 |
| J4 | 0.001726151 | 0.002493858 | 0.000767708 | 0.043986 |
| J5 | 0.008631706 | 0.084969521 | 0.076337814 | 4.373835 |
| J6 | 0.001726151 | 0.002493858 | 0.000767708 | 0.043986 |

### 同一时间尺度的J2/J3

时间基准=每次首笔TX记录的主机调度时间，包含写前工作，不是实测板端命令生效时间。取截止时刻之前最新已接收q/dq（零阶保持），其接收年龄见JSON。50/100/500/2000ms两组均已停止观测，必须记为N/A，不外推为稳定值。

| cutoff ms | joint | 无支撑 Δq / dq / proxy | 有支撑 Δq / dq / proxy |
|---:|---|---|---|
| 10 | J2 | 0.000000000 / -0.004884720 / 0.001465416 | 0.000000000 / -0.004884720 / 0.001465416 |
| 10 | J3 | 0.000000000 / -0.004884720 / 0.001465416 | 0.000000000 / -0.004884720 / 0.001465416 |
| 20 | J2 | 0.000000000 / 0.063491821 / -0.019047546 | 0.000000000 / 0.004884720 / -0.001465416 |
| 20 | J3 | 0.000000000 / -0.014652252 / 0.004395676 | 0.000000000 / -0.004884720 / 0.001465416 |
| 30 | J2 | 0.001144409 / 0.161172867 / -0.055218315 | 0.000381470 / 0.014652252 / -0.006684494 |
| 30 | J3 | 0.000000000 / -0.014652252 / 0.004395676 | 0.000000000 / -0.004884720 / 0.001465416 |
| 50 | J2 | N/A（已abort） | N/A（已abort） |
| 50 | J3 | N/A（已abort） | N/A（已abort） |
| 100 | J2 | N/A（已abort） | N/A（已abort） |
| 100 | J3 | N/A（已abort） | N/A（已abort） |
| 500 | J2 | N/A（已abort） | N/A（已abort） |
| 500 | J3 | N/A（已abort） | N/A（已abort） |
| 2000 | J2 | N/A（已abort） | N/A（已abort） |
| 2000 | J3 | N/A（已abort） | N/A（已abort） |

单位依次rad、rad/s、名义Nm proxy。J3两组已观测position都未偏离各自q_hold；这不是更长时间静止保证。

### Whole observed window对比（终止时间不同）

| Joint/condition | max abs Δq rad | peak-to-peak rad | final Δq rad | max abs dq rad/s | max abs proxy | position guard占比 | velocity guard占比 | proxy guard占比 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| J2 no support | 0.001907349 | 0.001907349 | 0.001907349 | 0.161172867 | 0.059795952 | 31.09% | 55.00% | 47.94% |
| J2 support | 0.000381470 | 0.000381470 | 0.000381470 | 0.024419785 | 0.009614754 | 6.22% | 8.33% | 7.71% |
| J3 no support | 0.000000000 | 0.000000000 | 0.000000000 | 0.014652252 | 0.004395676 | 0.00% | 5.00% | 3.52% |
| J3 support | 0.000000000 | 0.000000000 | 0.000000000 | 0.004884720 | 0.001465416 | 0.00% | 1.67% | 1.17% |
| J5 no support | 0.000766754 | 0.001151085 | 0.000766754 | 0.134311676 | 0.004063454 | 12.50% | 45.83% | 27.22% |
| J5 support | 0.001534462 | 0.001534462 | 0.001534462 | 0.183151245 | 0.006645622 | 25.02% | 62.50% | 44.52% |

### 本轮六轴

| Joint | max abs Δq | max abs dq | max abs proxy | minimum joint-limit margin |
|---|---:|---:|---:|---:|
| joint1 | 0.000000000 | 0.004884720 | 0.001465416 | 3.968146324 |
| joint2 | 0.000381470 | 0.024419785 | 0.009614754 | 0.506713867 |
| joint3 | 0.000000000 | 0.004884720 | 0.001465416 | 0.212174588 |
| joint4 | 0.000000000 | 0.061050415 | 0.001831512 | 3.997506142 |
| joint5 | 0.001534462 | 0.183151245 | 0.006645622 | 1.913496017 |
| joint6 | 0.000384331 | 0.085468292 | 0.002564049 | 3.997506142 |

### Guard与transient

本轮joint5在约49.941914ms窗口变化0.001534461975rad，原阈值0.001533508301rad，使用率100.062189%。超出量仅0.000000953674rad，但原规则已触发；不能事后放宽或以量化解释为由重试。
J5 position在约16.5/26.5/31.6/36.5ms分别相对锁存值增加约0.000384/0.000768/0.001151/0.001534rad，呈数帧同向变化。不能仅凭极小越界量判定为数值误报，也不能在无现场反馈时断言真实J5运动原因。
本轮J2在约21.8ms变化0.000381470rad，此后至36.5ms未继续增大；J3未见可分辨位置变化。已观察片段没有建立持续振荡证据，但时长太短，无法排除未来漂移/振荡或证明稳定。
首20ms覆盖完整；50/100ms窗口均截断。两组都不足50ms主动观测，已审阅相邻50ms均值torque-proxy slew无法计算（null）；raw event有限差分不是同一指标，详见summary，不能混用。

## 通信、TX、退出与现场

本轮0x10–0x14各4帧，共20帧、680字节、4批，batch逐字节相同。顺序仍为0x10→0x11→0x12→0x13→0x14；间隔推算约94.663Hz（仅4批，不代表2秒稳态）。
主机kernel accepted不是板端ACK。最后请求仍为本次q_hold、固定PD、零速度/FF，最后完整帧0x14；没有补发失能、reset或恢复姿态。
checksum=0、malformed=0、nonfinite=0，四类必需反馈stale=0。反馈频率（包含qualify）：0x24=203.275Hz, 0x20=202.550Hz, 0x21=202.541Hz, 0x22=202.493Hz, 0x23=2.008Hz。
post-TX解码事件处最大feedback age=5.230406ms；这是可重建事件的最大值，不是所有主循环时刻的完整观测。
正常2秒：未完成。2/2/5秒重复：未进行，Run1失败即停。原程序仍固定2秒，本轮未修改duration来准备Run3。
程序ABORT EXIT，close在EXIT日志和进程结束前完成；原程序没有精确close时间戳，不能编造。fuser复查未见端口占用者。close(serial)不代表motor disabled，已明确提示现场动力隔离。
本轮现场观察：UNKNOWN — no user report for this supported run received yet。动力隔离完成确认：UNKNOWN — user instructed; assistant cannot verify physical isolation。
上次现场观察为J2轻微向下、无其他异常；不能把上次观察用于本次。Firmware version、board total torque limit、0x12 scale、disconnect behavior、persistent state仍UNKNOWN。

## 交付

- [完整JSON，包括两组数据及SUPPORT_A_B_COMPARISON](gate_a_summary.json)
- [带condition列的合并timeseries CSV](gate_a_timeseries.csv)
- [A/B对比图](gate_a_plots/SUPPORT_A_B_COMPARISON.png)、[本轮六轴位移](gate_a_plots/displacement.png)、[速度](gate_a_plots/dq.png)、[proxy](gate_a_plots/proxy.png)
- [本轮原始RX/TX日志](takeover/external_support_20260917/run1.jsonl)、[review记录](takeover/external_support_20260917/run1.review.json)
- [上轮归档报告](takeover/external_support_20260917/no_support_previous_artifacts/GATE_A_RESULT.md)
- [离线分析验证](takeover/external_support_20260917/comparison_validation.json)

**FAIL**：本轮J2短时改善，但J5触发原guard，未完成静止接管。停止后续硬件实验，不修改控制参数，不进入Gate B/C或gravity compensation。

GATE A WITH EXTERNAL SUPPORT COMPLETE — DO NOT EXECUTE INTENTIONAL MOTION WITHOUT HUMAN AUTHORIZATION
