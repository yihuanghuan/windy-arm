# ARM6 零重力与 HOLD 切换：终端操作

适用当前 `lagrange-smc`、六轴 gamma6、`arm6.urdf`、ROS 2 Humble。

**最新核查：已重新建立驱动并成功启用 MANUAL_PUSH，但随后因 gravity slew lag 中止。**
当前硬件驱动 PID 58752 保留故障 HOLD，guidance 已退出；没有自动重启。
J4–J6 告警阈值 1.5 rad/s 已在源码准备，尚未安装/加载到实机，当前运行参数仍为 1.0。
[本轮告警与新故障核查](arm6_familiarization_validation/wrist_warning_15_20260918/REPORT.md)。

先前零重力与六轴手推已成立；最新腕部速度档尚未完成验收。
最近第三次启动被 `old/future JointState stamp` 拒绝，随后触发接管超时。
另一次驱动冷重启出现过 339 ms 命令间隔。这两个启动问题尚未修复，
不能把下面的操作说明当作“任意重复启动均已可靠通过”的证明。

这里“锁死”指 **HOLD：电机通电、主动位置保持并保留有界支撑**，不是机械刹车。
停止零重力时保留硬件驱动；关闭驱动/拔电源不是 HOLD 操作。

## 1. 环境：每个终端都先执行

```bash
cd /home/yihuang/westlake/windy-arm
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=95
export ROS_LOCALHOST_ONLY=1
export ARM6_PORT="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
```

`install/` 是本仓库编译产物。重新编译只在需要更新软件时执行，不是每次切换都执行：

```bash
cd /home/yihuang/westlake/windy-arm
source /opt/ros/humble/setup.bash
colcon build --packages-up-to manipulator --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

初次连接/换 USB 口时检查：

```bash
ls -l /dev/serial/by-id/
readlink -f "$ARM6_PORT"
```

不要把其他机械臂/串口替换进这个命令。只能有一个 `student_arm_node` 打开该串口。
不要同时运行其他 effort/position 控制器。

## 2. 当前故障后的恢复：保留实际 HOLD 指令重启驱动

这是故障恢复，不是日常零重力开关。操作者在场，机械臂静止，停止手推。
只适用于**未断电、当前驱动仍在发送实际 HOLD**。如果已断电，跳到第 7 节。

终端 A（先执行第 1 节环境）：

```bash
ARM6_RUN="$PWD/log/arm6-operator-$(date +%Y%m%d-%H%M%S)"
/usr/bin/python3 src/arm-platform/tools/arm6_operator_prepare.py \
  --output-dir "$ARM6_RUN" --capture-hold --port "$ARM6_PORT"
```

若当前没有硬件驱动（例如 17:07:54 已关闭的这次），不要执行 warm restart；按第 7 节支撑初始化。

这个工具**只订阅 ROS、读取参数并写文件**，不发电机命令，不打开串口，不清故障。
它要求六轴连续新鲜反馈、静止、位于原限位安全边距内、HOLD 指令稳定、支撑力矩有界。
只有打印 `HOLD captured` 才继续。失败时不要用旧快照替代。

保存 guidance 参数到固定位置，便于其他终端使用：

```bash
cp "$ARM6_RUN/guidance_params.yaml" log/arm6_guidance_params.yaml
```

然后停止**旧硬件 launch**。如果它就在你的终端里，按 Ctrl+C，并等进程退出。
当前由 Codex 启动的这一个旧 launch，在撰写本文时 PID 为 **51172**；它不是永久固定 PID。
本次先核对：

```bash
ps -p 51172 -o pid,args
```

只有输出确实是本机械臂的 `ros2 launch manipulator arm6_hardware_familiarization.launch.py`
时，才执行：

```bash
kill -INT 51172
```

后续会话使用自己的硬件终端 Ctrl+C，不能沿用这个 PID，也不要 `pkill` 全部 ROS 进程。
确认旧驱动已退出，串口没有其他占用：

```bash
fuser -v "$ARM6_PORT"
```

保持姿态不变，在采集快照后 **120 秒内**，在同一个终端 A 执行：

```bash
ros2 launch manipulator arm6_hardware_familiarization.launch.py \
  port_name:="$ARM6_PORT" \
  mode:=guidance \
  start_guidance:=false \
  manual_push_test:=true \
  use_rviz:=false \
  hold_command_file:="$ARM6_RUN/hold_seed.yaml"
```

这个 launch 建立可接管的 **HOLD**；`start_guidance:=false` 表示此时不会进入零重力。
串口只由它管理。终端 A 此后一直保留。
等待 heartbeat + 3 feedback frames ready，保持无接触至少 5 秒。
若出现 `fault latched`、`hold seed invalid` 或反馈超时，**停止本轮，不自动重启**。
快照过期或姿态改变时不要改时间戳/容差来绕过验证。

当前已存在独立 RViz 窗口，可继续使用。若没有，在另一个已初始化环境的终端启动：

```bash
rviz2 -d "$(ros2 pkg prefix --share manipulator)/arm6_familiarization.rviz"
```

## 3. HOLD → 零重力

终端 B 执行第 1 节环境，然后运行：

```bash
ros2 run manipulator manual_guidance.py --ros-args \
  --params-file "$PWD/log/arm6_guidance_params.yaml"
```

这是真实施力。开始时不要触碰；依次观察：

```text
ACTIVE HANDOVER_START
ACTIVE HANDOVER_GRAVITY_MATCH
ACTIVE KP_WITHDRAWAL
ACTIVE GUIDE
```

**`ACTIVE GUIDE` 仍是无接触验收档，此时不要手推。只有 `ACTIVE MANUAL_PUSH` 才允许按手推档运动。**
`ACTIVE GUIDE` 表示六轴 Kp=0，重力补偿与原 Kd 阻尼生效，但普通 GUIDE 的瞬时上限仍是 1.2 rad/s。
进入 GUIDE 后先无接触静置至少 5 秒。没有 `ACTIVE GUIDE` 就不能当作已开启成功。
`GUIDANCE STOPPED`、时间戳异常或 HARD_ABORT 都意味着本轮失败，不能继续手推。

**每次新启动硬件驱动，手推扩展档要求先完成三次接管和静置。**
第 1、2 次在静置通过后按第 4 节回 HOLD、退出终端 B 中的 guidance，再重复本节启动；
第 3 次静置通过后保持 GUIDE。终端 A 应累计出现：

```text
NO_TOUCH_PASS: completed handovers=1
NO_TOUCH_PASS: completed handovers=2
NO_TOUCH_PASS: completed handovers=3
```

计数属于同一个驱动进程；正常开关 guidance 不清零，重启驱动会清零。
三个 trial 不是自动循环重试；任何一次故障就停。

## 4. 零重力 → HOLD（正常关闭零重力）

终端 C 执行第 1 节环境，然后：

```bash
ros2 service call /arm6_manual_guidance/stop std_srvs/srv/Trigger '{}'
```

服务返回只是“停止请求已提交”。必须再看到终端 B：

```text
HOLD: operator stop acknowledged
```

此时已平滑回到位置保持，可在终端 B 按 **Ctrl+C** 退出 guidance。
**不要关闭终端 A 的硬件驱动。**

可在终端 C 核对：

```bash
ros2 topic echo --once /student/applied_motor_command
ros2 topic echo --once /student/speed_safety
```

正常 HOLD 的 `p` 为 `[30,30,30,5,5,5]`，`d` 为 `[1,1,1,0.1,0.1,0.1]`。
`current` 字段是驱动约定的 nominal torque FF，不要求全零。
没有 ack 或已为 FAULT 时，不把 `success=True` 当作成功解锁/保持。

**以后再次开启：重复第 3 节的 `ros2 run`。再次关闭：重复本节 stop → 等 ack → Ctrl+C。**
无需关闭/重启硬件驱动，也不需要断电。

## 5. 开启已配置的人工手推速度档

必须是同一驱动已累计三次静置通过、当前 GUIDE 又静置通过、操作者明确准备手推。
终端 C：

```bash
ros2 param set /student_arm_node manual_push_test true
ros2 param set /arm6_manual_guidance manual_push_test true
ros2 service call /arm6_manual_guidance/begin_manual_push std_srvs/srv/Trigger '{}'
```

直到终端 B 出现 **`ACTIVE MANUAL_PUSH`** 才开始推动。服务返回 `request submitted`
不等于已经启用；若拒绝，保持等待，不降低检查门槛。

当前已安装/实机配置（源码中的腕部告警 1.5 变更尚未部署）：

| 模式 | 告警 rad/s | 持续超速中止 | 瞬时中止 rad/s |
|---|---:|---|---:|
| 接管/HOLD/撤 Kp | 0.5 | 不使用持续计时 | 0.5 |
| 普通 GUIDE | 0.5 | >1.0，持续 ≥80 ms | 1.2 |
| MANUAL_PUSH J1–J3 | 1.0 | >1.8，持续 ≥120 ms | 2.0 |
| MANUAL_PUSH J4–J6 | 1.0 | >1.8，持续 ≥120 ms | 2.2 |

`manual_push_test=true` 只是开放申请能力，不直接跳过静置/接管门槛。
角度限位和 0.10 rad 安全边距、力矩限制、NaN、反馈新鲜度、心跳、故障锁存均保留。
HOLD 的“位置保持”会产生恢复力；GUIDE/MANUAL_PUSH 的 Kp=0。
两者是你要切换的状态，并不需要另一套“真正零重力”控制器。

## 6. 故障时怎么处理

看到 `HARD_ABORT`、`GUIDANCE STOPPED`、`fault latched` 或 `HANDOVER command timeout`：

1. 停止手推，保留驱动运行，记录当时是否有人接触。
2. 不反复发 begin/stop 试图清锁存；这些不是 fault reset 服务。
3. 确认反馈/限位/支撑和故障原因，再决定是否按第 2 节做**显式**重启。
4. 如果关节越界、反馈不稳定、曾断电或指令不可知，不使用 warm HOLD 快照流程。

普通 stop 服务用于健康状态切换。故障不会自动恢复 GUIDE。
超速轴 FF 清零、其他反馈有效轴保留最后有界支撑；反馈整体失效可能 all FF→0 + PD HOLD。
最近启动时序问题尚未修复；出现同类报错时不能靠多次启动证明系统已可靠。

## 7. 断电/冷启动：与当前 warm restart 分开

整机断电后旧 HOLD 文件失效。先可靠支撑机械臂，再上电并仅启动 readonly；
支撑保持到新 HOLD 现场确认稳定。这里不能保证机械臂无需人工支撑就自动安全启动。

终端 A（无其他串口驱动）：

```bash
ros2 launch manipulator arm6_hardware_familiarization.launch.py \
  port_name:="$ARM6_PORT" mode:=readonly use_rviz:=false
```

终端 C（执行第 1 节环境，机械臂已可靠支撑且静止）：

```bash
ARM6_COLD="$PWD/log/arm6-cold-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$ARM6_COLD"
PYTHONPATH="$PWD/src/arm-platform:$PYTHONPATH" /usr/bin/python3 \
  src/arm-platform/task_space_smc/hw/capture_supported_startup.py \
  --output "$ARM6_COLD/supported.yaml" --port "$ARM6_PORT"
```

采集成功后在终端 A Ctrl+C 关闭 readonly；在终端 C、采样文件 **15 秒内**执行：

```bash
ros2 launch manipulator arm6_hardware_familiarization.launch.py \
  port_name:="$ARM6_PORT" mode:=guidance start_guidance:=false \
  manual_push_test:=true use_rviz:=false \
  external_support_confirmed:=true \
  supported_startup_file:="$ARM6_COLD/supported.yaml"
```

这个入口根据受支撑静止力矩反馈估计初始支撑，不是旧命令读回，不保证冷启动无扰。
超过 15 秒就重新只读采集，不能改文件时间。
确认新 HOLD 能自行保持后才移开支撑，再开始无接触接管。

若本地尚无 `log/arm6_guidance_params.yaml`，在另一环境终端生成（不发指令）：

```bash
ARM6_PREP="$PWD/log/arm6-params-$(date +%Y%m%d-%H%M%S)"
/usr/bin/python3 src/arm-platform/tools/arm6_operator_prepare.py --output-dir "$ARM6_PREP"
cp "$ARM6_PREP/guidance_params.yaml" log/arm6_guidance_params.yaml
```

然后按第 3–5 节操作。需要整机下电/关闭硬件驱动时，先回 HOLD、可靠承重支撑，
再关闭驱动和电源。不要把“退出所有进程”当作锁死命令。

## 验收界限

本阶段的操作验收可定义为：同一个硬件驱动运行期间，能够重复完成
`HOLD → GUIDE/手推 → stop acknowledgement → HOLD`，反馈/支撑连续，
不产生非预期 fault。故障恢复和断电冷启动分别验收。
操作命令已经提供；最新构建的稳定重复切换尚未通过，不标为 PASS。

## 本轮交付核对

- 本仓库 `colcon build --packages-up-to manipulator`：4 个包构建成功；`install/setup.bash` 不依赖此前 `/tmp` 构建链。
- 新准备工具 Python 语法检查通过；对当前真实 HOLD 成功进行 ROS-only 快照采集；错误串口匹配被拒绝且未写入 seed。
- 已生成 `log/arm6_guidance_params.yaml`，六轴、新腕部档位和 URDF 路径校验通过。
- launch `--show-args` 通过，`git diff --check` 通过。
- 本轮没有发电机命令、清锁存、重启实机或进行 GUIDE 试验；新增的是操作手册与只读准备工具，控制逻辑未修改。
