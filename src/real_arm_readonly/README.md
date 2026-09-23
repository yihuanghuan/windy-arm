# real_arm_readonly

独立的六轴 A-L1 Gamma / gamma6 / arm6 **接收与显示**包。用户已确认硬件身份，不复用作者零偏或动力学标定。不依赖 `manipulator` executable、`dummy_interface`、MotorControl、SMC、重力控制器或 PySerial。

## 当前状态

代码和离线测试已完成。后续用户已明确批准默认 UART 接线下的 open/raw/921600；本轮通过独立 `uart_open_authorized` 路径执行了 3.806s 真实接收，因速度反馈异常提前关闭，未进入 ROS 发布阶段。详见 `docs/real_arm_audit_20260917/live/REPORT.md`。
RViz 可视化端已经启动，等待 `/real_arm/raw_joint_states`。不能把窗口中的初始模型当作真实姿态。

## 安全边界

应用以 `O_RDONLY|O_NOCTTY|O_NONBLOCK|O_CLOEXEC` 打开串口，没有串口 write/send、电机控制发布器、ROS 订阅或控制 service。断开时不重连、不发送 reset、零力矩或保位。

**O_RDONLY 不是物理单向隔离。** tty open/close/configure 可能改变 modem lines，旧 tty 配置还可能产生 echo/XON/XOFF。因此硬件模式默认关闭，默认要求先由现场人员确认适配器 TX/DTR/RTS 等所有输出无法影响控制板（RX/GND 接收链，注意电平/接地安全；不要根据本 README 盲目改线）。USB 内置桥如无法隔离，需要原理图/固件证明，或独立被动监听方案；本实现不绕过该要求。`rx_isolated=true` 只是操作者声明，软件无法测量接线，绝不能为了消除错误而随手开启。

确认后，接收器会对该 tty 设置 921600、8N1、raw、无 echo/软硬流控、无 HUPCL，并申请 TIOCEXCL。没有 udev/chmod/systemd 修改，没有 DTR/RTS ioctl 或 BREAK，没有发送协议初始化帧。TIOCEXCL 不会驱逐已有打开者，也不能阻止有特权程序；使用前须确认没有其它串口持有者/控制节点。这些 tty 设置在关闭后不主动恢复旧配置，以免重新打开 flow/echo；它们仍是配置变化，运行前需要知情确认。

## 话题契约

| Topic | 内容 |
|---|---|
| `/real_arm/raw_joint_states` | JointState，joint1..6，rad / rad/s；gamma6 符号 [+,+,+,-,-,-]，位置展开，与现有 DMMotor 约定一致；**不加 zero offset** |
| `/real_arm/feedback_status` | JSON String：收包率、位置帧年龄、校验/长度/非有限值计数、OK/STALE/ERROR |
| `/real_arm/robot_description` | arm6 URDF |
| `/real_arm/tf`、`/real_arm/tf_static` | 专用可视化 TF，仅 RSP 和 RViz 连接 |

raw 指未经标定的关节坐标，不是未取符号的线缆字节；原始七通道不伪装成七个物理关节。仅 checksum+长度正确且前六通道有限的 0x20 帧触发 JointState。0x21 速度单独到达，接收年龄不超过 25ms 才附带；不是与位置严格同时采样。电流单位未确认，effort 留空。温度/电压/current 当前不发布，不影响位置链。

时间戳是主机接收时间，不能证明固件采样时间；启动串口缓存及 USB 缓冲延迟仍需现场评估。0.25s 无位置帧即 STALE，不重发缓存 q；断流恢复后重新归一到 ±π，不跨断流猜测圈数。超过 ±π 的首次绝对圈数未确认。发布可有高频帧，RViz 默认30fps，RSP默认20Hz节流；不是每个编码器帧都渲染。

只有已确认的原始话题作为 RSP 输入；**不发布 `/joint_states` 或任何 `/student/*`**。RViz 无交互控制工具。不得将此话题重映射到命令输入。未校准时各轴变化可以对应编码器，但几何绝对零姿态不能保证与实物一致；后续需独立被动测量确认。

## 构建与安全的显示端启动

```bash
colcon --log-base /tmp/windy-readonly-log build \
  --packages-select dummy_description real_arm_readonly \
  --build-base /tmp/windy-readonly-build \
  --install-base /tmp/windy-readonly-install
source /tmp/windy-readonly-install/setup.bash
ros2 launch real_arm_readonly real_arm_readonly_rviz.launch.py start_feedback:=false
```

这只启动 RSP+RViz。`start_feedback` 默认 false。设 true 但未声明 `rx_isolated` 时，launch 在创建任何节点前拒绝串口接入；直接启动 feedback executable 也在 os.open 前拒绝。硬件启用命令留待接线确认，本轮没有执行。

launch/config 复用 `src/arm-platform/` 下的专用只读文件，由本包单独安装；URDF 使用 arm6，meshes 来自 dummy_description。不需要构建控制驱动。修改资源后需重建（本次使用非 symlink install）。

## 验证

纯离线测试：

```bash
PYTHONPATH="src/real_arm_readonly:${PYTHONPATH}" python3 -m pytest -q src/real_arm_readonly/test/test_protocol.py
```

隔离 ROS 集成测试（先核对 domain 173 无其它节点）：

```bash
source /tmp/windy-readonly-install/setup.bash
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 REAL_ARM_OFFLINE_TEST=1 \
PYTHONPATH="src/real_arm_readonly:${PYTHONPATH}" \
python3 -m pytest -q src/real_arm_readonly/test
```

测试以 FakeReceiver 注入合成字节，只发布反馈到本机测试域；启动的唯一子进程是 `start_feedback=false,use_rviz=false` 的 RSP。伪终端测试只访问 `/dev/pts`，验证 FD 写入返回 EBADF、二进制接收无 echo。测试中的 os.write 仅针对人工创建的 PTY，不属于生产模块。6 项测试通过；这不是硬件验收。

硬件验收待做：确认接线/独占所有权→只启动接收器→确认发布者、六个名称、频率、年龄、角度连续性→检查私有 TF→在现场明确安全条件下由用户决定是否手动移动，核对各轴方向与模型。不得为了反馈而发送 poll、enable、MIT、reset 或 MotorControl。

## 本次明确 UART 授权下的有界审计

`python3 -m real_arm_readonly.live_audit` 复用同一 Receiver 和 Decoder，提供独立的 `--uart-open-authorized` 明示入口；不伪造 `rx_isolated`。用户本次已批准默认接线下打开 `/dev/ttyUSB0`、raw、921600，但没有批准任何 UART 输出。正常路径先观察12s，再发布35s，最后关闭；只读 ROS 节点名称为 real_arm_readonly_live_audit。

审计额外设置保守停止条件：无主动位置反馈2s、位置断流0.25s、任意前六通道非有限值、坏校验/长度、位置相对首次偏移超过0.02rad、报告速度超过0.15rad/s。上述阈值是本轮异常筛查，不是平台认证阈值。当前因 j5 的速度尖峰停止，**不自动放宽阈值或重试**，等待现场澄清。历史校准仍不应用。

证据文件为主机本地 `.rx.bin`、JSON 和 strace 摘要；写入这些本地文件不涉及串口写。日志时间戳/频率为主机接收测量，附有提前停止原因。

## 七槽位复查更新

用户确认上一轮无实体运动后，已离线检查全部七槽位、完成新只读规则并重采16.00054s。旧速度单帧硬停已由 `ReadGuard` 的Warning/持续性与位移一致性规则替代。全部原始七槽位、chunk主机时间戳均保留。index5持续四项全零，index6固定异常值与历史记载吻合；没有改映射。为避免把缺失joint6当真实零角，审计入口在这种情况下完成纯接收后关闭，不进入ROS发布。`channel5_nonzero_observed` 只是否决全零占位的最低检查，非充分的电机在线认证。详见 `docs/real_arm_audit_20260917/RECHECK_REPORT.md`。
