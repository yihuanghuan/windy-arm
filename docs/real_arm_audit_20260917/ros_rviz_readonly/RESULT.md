# 真实 A-L1 Gamma 6DOF → ROS2 → RViz 只读贯通报告

日期：2026-09-17。用户确认采用更新固件后的平台。此次只授权接收、JointState、RSP 和 RViz；没有主动控制。

## 结果

**真实反馈链已贯通。用户现场确认：“现在rviz中机械臂和真实机械臂的姿态是一样的了”。**

`/dev/ttyUSB0 → real_arm_readonly_live_audit → /real_arm/raw_joint_states → real_arm_readonly_state_publisher → /real_arm/tf → RViz`

mapping joint1..6 ← slot0..5，sign [+1,+1,+1,-1,-1,-1]；slot6未进入模型；未应用作者历史zero_offset_rad，未修改URDF。

完成10秒纯观察后发布约25秒，共发布5020条JointState，独立订阅器收到5020条。首末接收间隔24.995094秒，实测200.799409 Hz；`ros2 topic hz`最终窗口200.801 Hz。接收间隔最大14.591ms，标准差0.316ms。stamp严格递增；5020条均有正确的六个joint name和六项有限position/velocity。effort为空（没有将未经确认单位的current冒充Nm）。

串口观察+发布总耗时35.056520秒（包括关闭）；checksum=0、malformed=0、nonfinite=0、stale事件=0；位置反馈最大接收间隔9.030ms。J6映射后position全窗口ROS范围0.0013427734375…0.002109527587890625 rad，持续非零有效。发布结束后串口关闭且无占用；未留下持续硬件读取任务。

## 一条完整实测JointState

```yaml
header:
  stamp:
    sec: 1789641453
    nanosec: 660497909
  frame_id: base_link
name:
- joint1
- joint2
- joint3
- joint4
- joint5
- joint6
position:
- -0.031853675842285156
- 1.3563365936279297
- -1.3349733352661133
- 0.0017261505126953125
- 0.007480621337890625
- 0.0017261505126953125
velocity:
- -0.0048847198486328125
- -0.0048847198486328125
- -0.0048847198486328125
- -0.03662872314453125
- 0.012210845947265625
- -0.03662872314453125
effort: []
---
```

## 最后一次观测角度

停止后下表是最后观测值，不宣称是继续实时读取的值。

| joint | rad | deg |
|---|---:|---:|
| joint1 | -0.031853675842 | -1.825081 |
| joint2 | 1.356336593628 | 77.712362 |
| joint3 | -1.334973335266 | -76.488338 |
| joint4 | 0.001726150513 | 0.098901 |
| joint5 | 0.007480621338 | 0.428608 |
| joint6 | 0.001726150513 | 0.098901 |

## TF与RViz

六个活动关节TF全部存在：`base_link → link1 → link2 → link3 → link4 → link5 → link6`。每个子link收到470次变换，约18.79Hz；JointState约200.8Hz与RSP降频后的TF发布率是两回事。模型恰好六个revolute joint，没有fixed joint，因此本模型未收到tf_static变换不代表缺失。

从ROS订阅得到的robot_description与仓库 `src/arm-platform/config/arm6.urdf` 完全文本一致。使用既有只读RSP/RViz进程，未重复启动控制驱动。仅使用隔离的 `/real_arm/raw_joint_states`、`/real_arm/robot_description`、`/real_arm/tf`、`/real_arm/tf_static`，未向全局/joint_states发布。

采集期间截图 [rviz_window_live.png](rviz_window_live.png) 显示Global Status: Ok、RobotModel Status: Ok、URDF parsed OK，以及base_link/link1..6全部Transform OK。此前缺动态JointState导致的RobotModel错误已解除。左下属性帮助区保留的“Status: Error”字样不是当前树状状态；当前状态树均为绿色OK。

现场对照：用户在本次观察后明确确认模型与实体姿态相同，因此没有可据实指出的具体joint错位；当前静止姿态没有观察到明显encoder zero offset、joint direction、URDF geometry或base frame orientation错误。末端/tool精确尺寸未进行量测，不能据整体视觉一致确认几何精度；静态一致也不等于全运动范围方向/零偏/标定均已验证。未要求用户推动或解锁机械臂。

按照“完成后停止”，实时输入已经结束。RViz/RSP可保留最后显示，但后续不会继续跟随实体，TF显示可能随数据变旧出现超时；这与采集期间已验证的同步状态区分。

## 本轮发现并修复的接收工具问题

第一轮纯观察10秒反馈正常，随后live_audit在接收循环里初始化DDS/ROS，主线程约0.31秒未读串口，触发0.25秒stale条件，发布0条后立即关闭。证据保存在相邻 `../ros_rviz_attempt1/`；这个事件不能证明硬件断流。

修复 `src/real_arm_readonly/real_arm_readonly/live_audit.py`：将ROS初始化、反馈Node和JointState publisher创建提前到Receiver打开之前；纯观察完成后才允许发布，原停止阈值、映射、只读Receiver完全保留。没有增加控制topic或串口TX路径。协议/读取保护离线测试10项通过，修复后的第二轮完成25秒发布且无stale。首次事件未从记录中删去。

## Safety audit

- 两轮strace均记录 `/dev/ttyUSB0` 以O_RDONLY打开并成功关闭；对该串口fd的write/writev/pwrite64系统调用合计 **0**。
- 0x10–0x14串口发送 **0**，未发送请求/唤醒帧。普通JSON、原始抓包、ROS网络和日志写入不等于serial write。
- 监测到的本ROS图MotorControl publisher **0**（整个观察窗口无MotorControl类型topic）。
- 本ROS图controller/student/master/slave/gravity/SMC节点 **0**；仅有只读反馈、RSP、RViz及诊断/CLI节点。该结论限定本机本ROS域可观察图，不宣称不可见域的全局事实。
- 没有进入gravity compensation、SMC、prepose/reset/demo；没有修改URDF或应用历史零偏。

## 证据

- [capture.json](capture.json)：本轮读取统计；[capture.rx.bin](capture.rx.bin)：原始串口字节；[capture.chunks.jsonl](capture.chunks.jsonl)：主机接收时间。
- [serial_syscalls.log](serial_syscalls.log)：串口fd系统调用。
- [ros_monitor.json](ros_monitor.json)：5020条完整JointState、TF及周期ROS图快照。
- [topic_hz.txt](topic_hz.txt)、[joint_state_once.yaml](joint_state_once.yaml)、[topic_info.txt](topic_info.txt)、[nodes_during.txt](nodes_during.txt)。
- [rviz_window_live.png](rviz_window_live.png)：采集期间RViz窗口。
