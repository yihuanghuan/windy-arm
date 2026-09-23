# GATE_A_RESULT

## Verdict

**INCONCLUSIVE**

执行状态：**ABORT BEFORE FIRST COMMAND — preflight未通过，串口未打开，硬件入口未调用。**

用户已明确授权本次Gate A，并确认独立承重、现场人员与可用实体动力隔离。未执行不是因缺少Gate A动作授权；当前已审阅程序不能原样满足此次指定的首帧检查/记录要求，且其必要固件证据项仍缺失。遵守“不临时修改程序、不绕过preflight”的要求，在设备访问前停止。

## 版本与preflight证据

程序：`src/arm-platform/tools/first_active_takeover.py`。

SHA256：`68c682381dc69ab7d7e4ba03944025d1900ac235b61cd060e3f1f7be58ca5691`。

与上次审阅报告引用的 `takeover/gate_a_review.example.json` 中记录的hash完全一致。程序为Git未跟踪文件，不能仅凭`git diff`认定它没有变化；本次采用记录hash核对。`git diff --check`通过，已有其它未提交修改已检查并保存为preflight证据，本轮未修改程序、参数或控制逻辑。

实际阻塞：

1. `live()`第256行直接`os.write`；第258行才写`TX_KERNEL_ACCEPTED`。此前只有原始RX日志，没有用户要求的**发送前打印并记录measured q、command q、六轴差值、velocity/current/Kp/Kd**。事后从RX/TX解码不能替代这一明确时序要求。
2. `tick()`第149行首次锁存q_hold并构造packet，同次调用可返回首批。`ingest()`第111行的proxy计算以q_hold非空为条件，所以首次返回时`proxy`仍未显式计算/记录，slew基线也尚未形成。q误差按构造为零、速度已受限，理论上可推导首帧proxy幅值界，但这不等于已完成此次要求的显式首帧pre-send检查与记录。
3. `validate_review()`第193–207行要求固件版本、固件语义依据、板端总限幅依据、旧模式切换依据等。现有模板这些字段为空、相应联锁为false。用户本次确认承重、现场动力隔离与动作授权，不等于提供了这些固件事实；不能伪填或删除检查。

本次没有调用live()来“试试看是否会拒绝”，没有通过wrapper/monkeypatch绕过检查，也没有补发任何其它命令。

## A. Takeover

- 第一帧measured q：N/A，未进行本轮实时读取。
- 第一帧commanded q：N/A，没有生成或发送本轮实际控制帧。
- 六轴初始误差：N/A，不使用历史姿态冒充当前测量。
- 是否观察到明显跳变：N/A，未发生本次接管试验；没有据此宣称现场设备没有运动。

## B. Motion

| Joint | max abs(q−q_hold) | peak-to-peak position | max abs(dq) | final displacement |
|---|---|---|---|---|
| joint1 | N/A | N/A | N/A | N/A |
| joint2 | N/A | N/A | N/A | N/A |
| joint3 | N/A | N/A | N/A | N/A |
| joint4 | N/A | N/A | N/A | N/A |
| joint5 | N/A | N/A | N/A | N/A |
| joint6 | N/A | N/A | N/A | N/A |

未采集不能记作“位移为零”。joint3当前限位余量也未实测，不沿用旧的0.235827rad作为preflight结果。

## C. Commands

审阅程序常量与本次要求一致：Kp=[6,6,6,1,1,1]，Kd=[0.3,0.3,0.3,0.03,0.03,0.03]，velocity/current全零，持续时间2.0s。**实际未应用任何这些参数。**

| cmd | 本轮发送数 |
|---|---:|
| 0x10 | 0 |
| 0x11 | 0 |
| 0x12 | 0 |
| 0x13 | 0 |
| 0x14 | 0 |

实际发送频率：N/A（未开启实验，而非测得一个有效0Hz控制窗口）。本轮串口open=0，serial write=0。

## D. Guards

position最大占比、velocity最大占比、torque proxy最大值、torque slew最大值、最小限位余量、最大feedback age：全部N/A，没有本轮运行样本。

触发的是**执行前代码/证据核查停止**，不是一次真实反馈guard失败。没有放宽任何阈值。

## E. Exit

- preflight中止，尚未打开串口；关闭时间N/A。
- 最后一帧发送内容：无。
- 本轮未改变执行器状态，也未创建需要退出的控制会话。
- 是否现场动力隔离：没有本次TX导致的新增处置要求；现场人员仍根据机械臂既有状态决定是否隔离，本报告不宣称电机已失能。

## 后续边界

先报告本次阻塞；本轮不修改/运行主动程序。要满足本次执行要求，需要对首帧检查及发送前日志做明确修订并重新审阅，同时如实处理固件证据条件；不得临时绕过联锁。不得把这份未执行结果解释为Gate A PASS或机械控制故障。

证据：[execution_preflight.json](takeover/execution_preflight.json)、[execution_preflight_git.diff](takeover/execution_preflight_git.diff)、[execution_preflight_git_status.txt](takeover/execution_preflight_git_status.txt)。

DO NOT PROCEED TO GATE B WITHOUT HUMAN REVIEW.
