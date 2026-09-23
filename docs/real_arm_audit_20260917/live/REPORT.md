# Read-only 真机反馈测试报告

2026-09-17，用户已确认 A-L1 Gamma 6DOF，并明确授权默认 UART 接线下打开 `/dev/ttyUSB0`、921600、raw、只接收主动反馈。

**串口成功，收到持续合法反馈，但测试于 3.806s 提前停止并关闭串口。未完成要求的至少10s统计，也未进入 ROS JointState 发布阶段。** 原因是 joint5 速度读数达到 +0.15872955rad/s，触发本轮保守0.15rad/s异常阈值。该阈值不是平台认证运动阈值；目前不能据此确认机械臂真的运动，也不能擅自判定为噪声继续采集。

本轮没有重开串口、发送查询/唤醒/控制帧，没有 MotorControl、student/master/slave、重力、SMC、prepose/reset/demo。历史 zero_offset 未应用。

## 1. 接收统计（3.806033908s，不是10s稳态测量）

| cmd | 类型 | 合法帧数 | 帧数/实际采集时长 Hz | payload B |
|---|---|---:|---:|---:|
| 0x01 | heartbeat | 76 | 19.968 | 1 |
| 0x20 | position | 755 | 198.369 | 28 |
| 0x21 | velocity | 755 | 198.369 | 28 |
| 0x22 | current/effort（单位未确认） | 754 | 198.106 | 28 |
| 0x23 | voltage | 7 | 1.839 | 28 |
| 0x24 | temperature | 754 | 198.106 | 14 |
| 0x25 | 未解释反馈 | 7 | 1.839 | 28 |

共接收93094字节。Checksum error=0；malformed length=0；前六通道非有限帧=0；stale interval=0（阈值250ms）；位置帧最大主机接收间隔6.102ms。

起始丢弃6字节非帧头内容，停止时剩24字节未完成帧。这两者单独记录，不冒充已判定坏帧。频率包含打开后首包/缓冲边界与 strace 开销，短窗口中的低频0x23/0x25不能精确推定周期；它们约2Hz。

0x23前六通道均0，不能当成可信实际电压。0x24前五通道为28/28/28/31~32/32，第六通道0。收到有效FE55只证明主机链路和帧解码正常，不证明每台电机在线。

## 2. 最后收到的关节角和速度

坐标为与 gamma6 驱动相同的符号 [+,+,+,-,-,-]、首次±π归一及后续展开，无历史零偏。位置和速度是两个独立帧，不保证同时采样。

| joint | position rad | velocity rad/s |
|---|---:|---:|
| joint1 | 0.01010895 | -0.00488472 |
| joint2 | 1.32543659 | -0.00488472 |
| joint3 | -1.34947014 | -0.00488472 |
| joint4 | 0.00326061 | 0.01221085 |
| joint5 | 0.00287724 | 0.15872955 |
| joint6 | 0.00000000 | -0.00000000 |

前六个位置值全部有限。j1/j2/j3/j6在755帧中不变；j4/j5峰峰值分别均为0.000383377rad（0.021966°）。j4/j5速度有明显波动，与位置微小变化不完全一致，可能与反馈噪声/量化有关，但本轮没有证据排除真实瞬时运动。

**第六通道 position、velocity、current、temperature 全窗口始终为0**。用户确认的物理6DOF不变；此处待确认的是第六轴遥测有效性/上电/连接状态，不是重新质疑机械臂DOF。不能用有限的全零缓存证明第六轴有效在线。

## 3. ROS 与 RViz

计划为先12s纯接收验收，再发布35s至 `/real_arm/raw_joint_states`，最后关闭。本次异常发生在纯接收阶段，实际 published=0。

- `/real_arm/raw_joint_states`：publisher=0、subscriber=1（RSP）；**实测 ROS 更新频率不可得**，不能拿串口198.37Hz冒充 topic hz。
- 已执行有界 `ros2 topic hz` 和 `ros2 topic echo --once`，均4s超时、没有样本；详见同目录日志（空日志是没数据，不是成功）。
- 已有 RSP/RViz 仍保持纯显示状态，没有启动反馈节点之外的控制器，也没有把离线保存角度作为“实时”消息回放。
- 私有链为 `/real_arm/raw_joint_states → RSP → /real_arm/tf → RViz`。该段之前用隔离合成数据验证过；**本轮未完成真实 encoder 贯通验证**。
- RViz窗口此时缺少动态TF，显示的初始模型不能代表真机姿态；本次无法记录可信的 raw encoder 模型与实体偏差。未改 offset、URDF joint origin 或模型姿态去“对齐”。

## 4. 安全证据与异常处置

`strace -f -yy` 实际记录 `/dev/ttyUSB0` 以 O_RDONLY|O_NOCTTY|O_NONBLOCK|O_CLOEXEC 打开；设置 TIOCEXCL、921600/raw；随后只有 read，最终 close成功。

对该串口fd记录到的 write/writev系统调用数=0；没有DTR/RTS/BREAK ioctl，没有ProtocolV1发送器。程序没有发送0x10..0x14帧的路径。证据只能认证本进程的用户态调用，不能替代硬件逻辑分析仪证明内核/适配器线缆电气行为。

旧 tty 属性为9600/canonical/echo，已按本次明确授权改为921600/raw/no echo，没有修改udev、权限或服务。末尾 fuser未显示串口占用者。

异常触发后立即关闭Receiver并退出，无复位、恢复模式、兜底保位或“发零”动作。现场物理运动、控制板复位、电机状态变化没有摄像/状态位可供独立观测，已询问用户，尚未收到现场确认。

## 5. 后续边界

等待现场确认是否出现动作/复位/电机状态变化，以及第六轴当前状态。不能仅因位置波动小就自动调大速度阈值、重开串口或忽略第六轴。

若现场确认全程静止且允许继续只读调查，可先对速度反馈语义/噪声做离线分析，再给出明确的只读监测标准并重新采集不少于10s；不需要也不允许通过控制请求唤醒电机。本次已授权只读范围仍保留，当前停止是对异常的处置，不是把读操作误当运动授权。

## 文件

- [capture.json](capture.json)：现场接收统计与停止原因。
- [capture.rx.bin](capture.rx.bin)：实际收到的字节，仅用于离线分析，不可回写串口。
- [offline_channels.json](offline_channels.json)：逐通道原始值范围。
- [syscall_audit.json](syscall_audit.json)、[serial_trace_excerpt.txt](serial_trace_excerpt.txt)：串口系统调用审计。
- 原完整trace位于 `/tmp/real_arm_readonly_syscalls.log`。

本轮生产代码新增有界 `live_audit.py` 与独立 UART 授权参数，并增加逐cmd计数；6项协议/只读访问离线测试通过。未改动任何控制源码或标定参数。
