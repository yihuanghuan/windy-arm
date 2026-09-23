# GATE A RESULT — FAIL

2026-09-17。本轮实际执行一次，Run 1 在首次TX调度后约30.696ms触发持续位移guard并退出，未完成计划2秒。用户现场确认：“看起来像是第二个关节向下运动了一点，没有其他异常现象”。这是实体位移伴随guard abort，不能按普通日志问题自主重试。Run 2/3未执行，未进入Gate B/C或重力补偿。

## Runs

| Run | 计划 | 实际 | 结果 |
|---|---|---|---|
| 1 | 2秒固定q_hold | 3批TX、约30.696ms后停止；非完整2秒 | FAIL：joint2持续位移guard |
| 2 | 2秒、重新锁存 | 未执行 | 前次非PASS，禁止重试 |
| 3 | 5秒、重新锁存 | 未执行 | 前次非PASS，禁止重试 |

实际程序SHA256：`cc9f022002a2232fa15340d5f55125527369947dfa41bd8cbcd9236047e695cd`，与批准版本完全一致。本轮未修改硬件程序、增益、guard、mapping、目标、频率、超时或发送顺序。独立分析工具的修改只作用于已保存文件。原程序固定DURATION=2秒，不具备5秒入口；本轮在Run 1已触发停止条件，未尝试更改时长或实现Run 3。

## 首帧与命令

实时锁存q_hold = [-0.031853675842285156, 1.3563365936279297, -1.3349733352661133, 0.0017261505126953125, 0.008631706237792969, 0.0017261505126953125] rad。J5与历史参考不同，证明没有硬编码上次目标。
第一帧measured q=command q=q_hold；六轴初始差值[0,0,0,0,0,0]。完整[pre-write snapshot](takeover/authorized_gate_a_20260917/run1.jsonl.pre_send.json)在第一笔write之前持久化。
Kp=[6,6,6,1,1,1]；Kd=[0.3,0.3,0.3,0.03,0.03,0.03]；velocity command=current/FF=[0,0,0,0,0,0]。无历史zero offset、gravity或alpha。
首帧P=FF=0；D=total proxy = [0.0014654159545898437, 0.0014654159545898437, 0.0014654159545898437, -0.00036632537841796875, -0.004029350280761719, -0.0010988616943359375]。max abs=0.004029350281；最大包络占比26.9958%。首帧最大feedback age=4.174141ms，J3 margin=0.235826664734rad。

## Takeover transient

时间基准是TX日志的主机调度时间，早于首笔实际write（包含快照/检查时间）；没有板端命令生效或ACK时间戳。因此不能把以下ms值当作精确板端响应时延。

| Joint | 首20ms max abs(q−hold) rad | 首20ms max abs(dq) rad/s | 首50ms/100ms已观察片段 max abs(q−hold) | 同片段 max abs(dq) |
|---|---:|---:|---:|---:|
| J1 | 0.000000000 | 0.004884720 | 0.000000000 | 0.014652252 |
| J2 | 0.000000000 | 0.063491821 | 0.001907349 | 0.161172867 |
| J3 | 0.000000000 | 0.014652252 | 0.000000000 | 0.014652252 |
| J4 | 0.000000000 | 0.036628723 | 0.000000000 | 0.085468292 |
| J5 | 0.000384331 | 0.134311676 | 0.000766754 | 0.134311676 |
| J6 | 0.000383377 | 0.036628723 | 0.000383377 | 0.036628723 |

首20ms有4个position/4个velocity反馈。50ms和100ms窗口**未完成**，仅有30.696ms片段（7个position/6个velocity）；不能声称通过这些完整窗口。最后RX余26字节是停止时截断的尚未完整反馈帧，不等于checksum/malformed错误。

## 全部已观察主动窗口

| Joint | max abs displacement rad | position peak-to-peak rad | final displacement rad | max abs velocity rad/s | max abs proxy | min limit margin rad |
|---|---:|---:|---:|---:|---:|---:|
| joint1 | 0.000000000 | 0.000000000 | 0.000000000 | 0.014652252 | 0.004395676 | 3.968146324 |
| joint2 | 0.001907349 | 0.001907349 | 0.001907349 | 0.161172867 | 0.059795952 | 0.641756058 |
| joint3 | 0.000000000 | 0.000000000 | 0.000000000 | 0.014652252 | 0.004395676 | 0.235826665 |
| joint4 | 0.000000000 | 0.000000000 | 0.000000000 | 0.085468292 | 0.002564049 | 3.998273849 |
| joint5 | 0.000766754 | 0.001151085 | 0.000766754 | 0.134311676 | 0.004063454 | 1.990601540 |
| joint6 | 0.000383377 | 0.000383377 | 0.000000000 | 0.036628723 | 0.001098862 | 3.997890472 |

position统计含锁存时刻基线；velocity统计含首帧测量。proxy由最近收到的q/dq作零阶保持重算；不是实测力矩。完整RX块已先写日志，abort在该块处理途中抛出，因此离线重算可能包含active guard尚未处理完的数据；不把它当成程序在线已检查到的值。

| Joint | position error guard占比 | velocity guard占比 | proxy guard占比 | raw proxy差分峰值（非guard slew） |
|---|---:|---:|---:|---:|
| joint1 | 0.00% | 5.00% | 3.52% | 18.478699 |
| joint2 | 31.09% | 55.00% | 47.94% | 122.507869 |
| joint3 | 0.00% | 5.00% | 3.52% | 17.500148 |
| joint4 | 0.00% | 29.16% | 17.18% | 23.098915 |
| joint5 | 12.50% | 45.83% | 27.22% | 9.239710 |
| joint6 | 6.25% | 12.50% | 7.36% | 8.540875 |

已审阅slew guard基于相邻约50ms均值；此次主动片段不足一个完整50ms窗口，更没有两个均值窗口，故六轴该指标均为**不可用（JSON null）**，不能填写0或声称通过。表中raw差分受异步position/velocity事件及量化影响，不能直接与50ms均值guard比较。

## 触发原因与运动判断

- joint2从q_hold单向增加：约20.805ms时+0.000381470rad，25.830ms时+0.001144409rad，30.696ms时+0.001907349rad（0.1093°）；用户观察为轻微向下运动。
- 最后约49.731644ms窗口（包含接管前约19ms反馈）joint2变化0.001907348633rad，超过0.001533508301rad阈值，使用率124.38%。它没有超过较宽的总position-error guard，但确实超过已有连续位移判据。
- joint2协议速度在约16.115/20.977/25.989ms为0.0634918/0.1123314/0.1611729rad/s，与位移发展方向一致；短片段提示速度上升，不足以判断长期持续加速。
- 没有证据证明“大幅瞬间跳变”；但已存在真实小幅非预期位移，静止接管目标未达成。不能因角度小或支撑可靠改判PASS。
- 未建立持续振荡证据；窗口太短，无法排除后续振荡或长期漂移。J5/J6也出现少量量化级变化，不能仅据此断定额外实体异常。
- 原因尚未定位。低增益零前馈切换导致受力变化、支撑微小让位、板端状态/命令语义均是待查假设；本数据不能区分，也不能推导应提高Kp、加g或改发送顺序。

## 通信与TX

0x10/0x11/0x12/0x13/0x14各3帧，共15帧、510字节、3批；每批170B，所有packet逐字节相同且checksum独立复核通过。批次间隔推算95.268Hz，仅3批，不能代表2秒稳态频率。
串口驱动返回完整accepted bytes只证明主机内核接受，不是板端ACK或实际接收数。最后完整帧0x14，后续没有退出/失能/重置帧。
反馈率（包含约1秒qualify）：0x20=203.520Hz, 0x21=203.479Hz, 0x22=203.298Hz, 0x24=203.298Hz, 0x23=2.008Hz。
checksum=0、malformed=0、nonfinite=0；所需0x20/21/22/24无stale，最大帧间隔约9.183ms。0x23约2Hz，不属于50ms freshness约束的四类必需反馈。
已解码post-TX事件处最大feedback age=5.865784ms；不是连续时间最大值，日志没有逐次记录select循环的所有age。未将缺失时间点伪造为已测量。

## Exit、现场与边界

ABORT EXIT。程序finally关闭串口，进程exit code=1；只读fuser复查没有端口占用者。原日志未记录独立的close时间戳，故只能证明close发生于EXIT日志与进程退出前，不能给出虚构精确时刻。
最后请求状态仍为本次q_hold、零velocity/FF及固定Kp/Kd；旧板端持久状态、断流/关闭后的行为为UNKNOWN。关闭串口不等于disable。已立即提示现场用户执行实体动力隔离；用户尚未明确回报隔离已完成，报告保留UNKNOWN。
现场确认joint2轻微向下运动、没有其他异常现象。不将“没有其他异常”写成“没有发生运动”。
无Run 2/3，因此run-to-run reproducibility不可评估。未生成Gate B执行程序或继续主动试验。固件版本、内部总力矩限幅、0x12标度、disconnect behavior、persistent state继续UNKNOWN。

## 证据

- [原始RX/TX/EXIT日志](takeover/authorized_gate_a_20260917/run1.jsonl)
- [批准与版本记录](takeover/authorized_gate_a_20260917/run1.review.json)
- [完整summary JSON](gate_a_summary.json)
- [逐事件六轴CSV](gate_a_timeseries.csv)
- [位移图](gate_a_plots/displacement.png)、[速度图](gate_a_plots/dq.png)、[proxy图](gate_a_plots/proxy.png)
- [独立原始字节交叉核验](takeover/authorized_gate_a_20260917/analysis_validation.json)
- [仅分析代码修改记录](takeover/authorized_gate_a_20260917/analysis_changes.json)

**FAIL — Gate A requires investigation.** 不自主重试，不放宽guard，不执行任何计划运动。

GATE A COMPLETE — WAITING FOR HUMAN AUTHORIZATION FOR ANY INTENTIONAL MOTION
