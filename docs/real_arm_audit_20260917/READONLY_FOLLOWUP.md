# 六轴只读链后续实施记录

2026-09-17。用户已明确确认真实平台为 **6DOF / A-L1 Gamma / gamma6 / arm6**，身份不再是待确认项。作者实验标定参数不因此变成本机有效标定。本记录更新 REPORT.md 中的身份待确认状态；原始现场空 ROS 图记录保留作为当时事实。

## 已完成

- 新建 `src/real_arm_readonly/` 独立包，生产代码仅接收协议数据和发布反馈；无 MotorControl / dummy_interface / 控制器依赖。
- 正确校验 FE55 接收帧、sum/add、各 cmd payload 长度、前六通道有限值；忽略第七物理不存在通道，不用 heartbeat 冒充新位置。
- `/real_arm/raw_joint_states`：六轴名称、符号、连续位置；不加载零偏和任何作者标定，effort 留空。
- 接收超时输出 STALE，停止重发位置；接收错误关闭本节点 fd，不重连、不 reset、不发送兜底命令。
- 专用 launch 的硬件源默认关闭。RSP 输入固定为 `/real_arm/raw_joint_states`，TF/robot_description 隔离到 `/real_arm/`。RViz 配置跟随该路径，无控制工具。
- 构建成功：只构建 dummy_description + real_arm_readonly，build/install/log 均在 `/tmp/windy-readonly-*`。
- 6 个测试通过：分包/重同步、坏帧/NaN、第七通道、符号/展开、无确认不 open、PTY 的只读 FD 拒写与无 echo、隔离域的反馈→TF 与断流检查（部分测试涵盖多项断言）。
- 实际启动了 **RSP + RViz**（没有反馈节点），窗口与资源已检查。启动的 ROS graph：real_arm_readonly_state_publisher、real_arm_readonly_rviz、RViz 自带 transform_listener；没有 MotorControl 节点。

## 当前真实状态

`/real_arm/raw_joint_states`：publisher=0，subscriber=1（RSP）。`/real_arm/tf`：publisher=1（RSP），subscriber=1（RViz TF listener）。显示窗口的初始模型没有实测角度，出现 TF 状态错误符合缺少反馈的现状，**不是已同步模型**。

真机串口从未打开；`fuser /dev/ttyUSB0` 未显示持有者。所有合成反馈只在 ROS_DOMAIN_ID=173、ROS_LOCALHOST_ONLY=1 的测试域运行，测试结束已清理；没有把合成角度送进实际显示端/domain0。当前显示端继续等待真实反馈。

## 静态安全审计

生产模块：

- `protocol.py`：无 I/O，只解析和坐标映射。
- `receiver.py`：仅 O_RDONLY 打开、os.read、tty 属性设置、TIOCEXCL 和 close；无 write、send、DTR/RTS/BREAK、控制帧生成。未接线确认时在 os.open 前拒绝。
- `feedback_node.py`：只发布 JointState 到 `/real_arm/raw_joint_states` 和诊断 String；无订阅/控制 service，参数 services 关闭；没有当前值/零力矩的 MotorControl 兜底。
- launch：仅允许 dedicated feedback、RSP、RViz；无 student、master、slave、hardware、SMC、prepose 或 demo executable。
- RSP/RViz 的标准参数服务存在，但不连接执行器；没有调用任何参数修改服务。

**软件只读不等于物理隔离**。Linux/驱动可在 open/configure/close 时切换控制线；原先 termios 的 echo/软流控还可能生成串口输出。依据 [pySerial 官方关于 open 的说明](https://pyserial.readthedocs.io/en/latest/pyserial_api.html)，仅禁止 write 不能消除开口控制线副作用。

因此硬件启动必须先核实 TX/DTR/RTS 等输出与板端关系。实现采用保守的物理 RX 隔离前提；`rx_isolated` 是人工声明，不是软件自动检测或运动授权。未获得接线证据前没有打开串口，未实际配置 921600 或 termios。接收器后续需设置波特率/raw，这属于明确披露的必要端口配置，并非静默修改系统配置。

## 剩余阻塞与下一步

唯一当前接入阻塞是“现有 USB-UART 打开/配置/关闭不能影响板端”的证据，**不是 6/7 自由度身份**。已向用户询问接线/物理隔离，尚未收到回复。

接线确认后仍需采集真实帧、测频率/角度/连续性并验收 RViz；若固件不主动吐反馈，不能自动发送请求或 enable。若无法证明现有桥安全，可准备物理被动 RX 监听方案，由用户决定现场接线；不自行改线或进入力控。

本轮不进入重力补偿、不启动 MotorControl 节点、不借作者参数修正显示姿态。

文件与详细契约见 [real_arm_readonly README](../../src/real_arm_readonly/README.md)。


后续：用户已明确授权默认UART接线下只读开口，实际测试及提前停止记录见 [live/REPORT.md](live/REPORT.md)。原“等待接线授权”状态已由该次授权取代；当前等待的是速度异常与第六轴反馈的现场澄清。
