# SPEED SAFETY CHANGE SUMMARY

原逻辑：max_guidance_velocity=0.5，Python/C++两层、全部phase共用单帧中止，无duration、无独立instant上限。

首次验证配置唯一数值来源：config/arm6_familiarization.yaml。transition warning/abort=0.5；GUIDE warning=0.5、sustained abort=0.8/0.08s、instant abort=1.2（rad/s）。每轴独立计时，速度<=0.8即清零，phase切换也清零；Python time.monotonic与C++steady_clock，不使用ROS墙钟做持续计时。

Python由过渡进度判定phase；C++只在上一次实际发送的六轴Kp精确为零且命令enable有效时使用GUIDE阈值，其他phase保持严格。纯保护类各自锁存故障。Driver新增只允许停机的speed_fault_axes位掩码通知，解决两进程采样顺序不同导致Python先fault、驱动尚未辨识轴的问题；不改变MotorControl格式，不新增串口所有者。

GUIDE阻尼、重力模型、offset、ratio约定、FF/slew限制均不变。重力+阻尼预算以新的instant上限计算可达阻尼力矩，较旧预算更保守，拒绝不够预算的姿态。NaN、心跳、新鲜度、维度、ghost过滤、超时、故障锁存和按轴保留FF均保留。operator stop现在也在驱动层锁存，不能被后续命令悄悄恢复。

文件：config/arm6_familiarization.yaml；launch/arm6_hardware_familiarization.launch.py；core/manual_guidance.py和新增core/speed_safety.py；hw/manual_guidance.py；student_arm_node.h/.cc和新增speed_safety.h；test_manual_guidance.py、test_familiarization_pty.py、test_effort_safety.cc。

验证：24项相关Python通过；9项PTY通过（包含真实已安装Python+驱动在0.6速度告警而不退GUIDE、持续/瞬时/transition中止、FF保留、计时复位、fault通知竞态）；4组C++通过；独立overlay构建、py_compile、launch --show-args和git diff --check通过。

额外全量Python：653通过，2项既有失败：Vicon外参断言和SMC hwtest gain冻结断言。通过git archive HEAD在/tmp独立副本运行相同两项，复现同样失败（2 failed）。没有checkout，未更改其测试或配置；它们不属于familiarization launch，也不作为本轮安全检查通过的证据。

实际阈值、warning、各轴持续时间、fault mask通过/student/speed_safety和/arm6_manual_guidance/speed_safety记录；真实wire FF使用单一hardware node串口write跟踪解码，非第二次打开串口。

实机readonly：100.336 Hz，403帧，259组TF匹配，六轴，无MotorControl发布者，RViz两项Status OK。随后已执行两次实机，详见ACCEPTANCE_REPORT.md。

## 现场明确授权的参数更新

初始0.8阈值下，J1在GUIDE完成18.43°移动，峰速0.601仅告警；操作者确认容易推动、松手保持。J2正常手推达0.972，超过0.8持续80.033ms而中止。用户明确要求扩大范围后，仅将GUIDE持续阈值改为1.0；其他阈值、80ms、模型和阻尼不变。保留旧0.8配置测试，增加当前1.0配置边界测试：25项Python及9项PTY通过，构建通过。

新参数实机启动约115ms时，J2在RAMP触发0.5严格保护，未到GUIDE；因此1.0持续阈值的GUIDE手推尚无真机通过证据。未再次提高阈值或自动重试。
