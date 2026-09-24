# 2026-09-24 link6 world hold：真实小幅扰动 demo

已完成真实视觉反馈 → 原 P 控制 → 驱动位置指令 → 实际关节运动。
用户观察：首轮“有比较轻微反方向运动趋势”，第二轮“有一定的补偿效果”。
这证明闭环接通并有反向补偿，尚未完成六方向稳定性能验收，也没有匹配扰动的无控制对照。

## 本次状态与结果

用户调整了整个 base 的朝向以改善视野，安装外参未改变。重新启动 SLAM、做整体支撑件小幅运动初始化后，
10 秒静止视觉位置跨度各轴小于 0.4 mm；本会话未出现此前米级位姿跳变。
几何仍为 `W_E = W_C * inverse(H_C) * H_E(q6)`，控制完整 link6，q6 没有置零。
H=joint6_housing，C=camera_link；安装数值见 FIXED_MOUNT_RESULT_20260923.md。

| 实际接入窗口 | 时长 | 位置误差最大值 | SO(3) 姿态误差最大值 | 退出原因 |
|---|---:|---:|---:|---|
| 固定 base | 15.00 s | 0.231 mm | 0.0353° | 预定时长完成 |
| 第一次水平扰动 | 31.61 s | 30.58 mm | 2.435° | joint1 目标相对初始保位命令达到 3° |
| 第二次更慢、更小扰动 | 49.41 s | 4.027 mm | 1.386° | joint4 目标相对初始保位命令达到 3° |

第二轮最后 3 秒误差中位数：位置 3.57 mm，姿态 1.125°。这些是视觉反馈误差，不是外部计量精度。
两次扰动前都有较长的等待时间，因此不能用全段误差中位数代表运动阶段表现。
第二轮实际关节变化约 `[2.23, 0, 0, -1.12, -1.41, -0.90]°`，q6 确实参与。
第一轮 joint4 速度峰值 0.4518 rad/s，第二轮各轴峰值均不超过 0.1832 rad/s；无驱动速度故障。
两轮退出后 `operator_hold` 均成功，补偿节点已 disable；驱动继续主动保位，未断电。

离线由 SLAM 与 FK 分解第二轮运动：若关节固定，末端位移估计最大 7.12 mm，实测重建末端位移最大 4.03 mm。
这是同一数据的运动学反事实估计，**不是独立测得的无控制基线**，不能据此宣称正式稳定性能提升百分比。
末段关节运动贡献与反向扰动方向的余弦约 0.932，与现场观察一致。

当前姿态仍接近腕部奇异位形。第二轮末端 Jacobian rank=6，奇异值约
`[1.734238, 1.432129, 1.001735, 0.174844, 0.142301, 0.00011775]`，条件数约 14728。
这与较弱的六维补偿能力有关，不能以名义 rank=6 代替良好可控性。
joint2/3 对本次小位置命令没有可分辨的位移，关节位置跟随仍需验证；未添加高级控制算法。

## 最小接口修正

冷启动后原驱动使用 PD 位置偏差承重。直接以每周期实测 q 重建命令会抹掉该承重偏差。
本次保留已有保位驱动，新增 `camera_world_hold_position_bridge.py`：

- 控制器原有 P / SO(3) / 完整 Jacobian 计算不变，额外发布 `~/joint_velocity`（供位置积分）。
- 接入层复用 `camera_fk_motion_probe.py` 的实际保位快照、精确首帧交接、100 Hz 发布、反馈检查和退出保位。
- 从原保位 reference 积分 dq；电机 velocity 字段始终为 0，原 Kp/Kd/current 不变。
- 继续使用原驱动 watchdog、速度保护和故障锁存。试验另有 3° 行程及 15/45/60 秒时长边界。
- 未接入 camera twist、加速度、SMC、NMPC、LADRC 或在线 base estimator。

**本次 controller 的 `publish_command=false` 不等于整套系统无动作：显式启动 `position_bridge --execute` 后，
bridge 将积分位置发布到原 `/student/joint_effort_command` 的 MIT position 字段。**
不同时使用 `publish_command=true` 或另一个命令发布者。桥接器会拒绝竞争发布者。
正常 `/student/joint_command` 路径仍保持原样；本次没有切换到 student_arm6 普通位置驱动。
实际积分位置另记 `/camera_world_hold/actuator_joint_command`，真实驱动输出记 `/student/applied_motor_command`。
controller CSV 的 `command_published=false` 仅表示未直发 `/student/joint_command`；实际闭环窗口由 bridge 日志裁剪。

## 数据与验证

目录：`log/link6_demo_20260924/view2/`。

- `closed_loop_bag/`：固定 base 和首轮扰动；`slow_bag/`：第二轮扰动，均已正常关闭。
- `moving_active.csv` / `moving_error.png`、`slow_active.csv` / `slow_error.png`：仅真实接入窗口的误差曲线。
- `static_trial/`、`moving_trial/`、`slow_trial/`：实际关节、相机和积分位置 JSONL。
- `trial_metrics.json`、`moving_analysis.json`、`slow_analysis.json`：本报告指标。
- Python 编译、ROS 节点 preview 启动、构建通过；接入层/原交接/隔离 ROS 测试 **13 passed**，见 `bridge_tests.log`。
- 先前完整 link6 重建/有限差分/六列 Jacobian 软件测试仍适用，本次未改运动学。

## 复现本次接入

所有终端：

```bash
cd ~/westlake/windy-arm
source /opt/ros/humble/setup.bash
source ~/westlake/ORB-SLAM3/ros2_install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=95 ROS_LOCALHOST_ONLY=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
```

相机/SLAM 没有运行时才启动，使用新的结果目录：

```bash
ros2 launch orb_slam3_ros2 live.launch.py \
  orb_slam3_root:=/home/yihuang/westlake/ORB-SLAM3 \
  serial_no:=_244222077008 use_viewer:=false \
  result_dir:=$PWD/log/demo_reproduce_slam
```

本次驱动保持运行即可，无需再次接管或重启。若冷启动，必须先有可靠外部支撑，按已有 supported startup 流程取得新鲜反馈：

```bash
# 终端 A，只读驱动。
ros2 launch manipulator arm6_hardware_familiarization.launch.py \
  port_name:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  mode:=readonly start_guidance:=false use_rviz:=false

# 终端 B，输出文件须不存在。
export PYTHONPATH="$PWD/src/arm-platform:$PYTHONPATH"
python3 -m task_space_smc.hw.capture_supported_startup \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --output /tmp/link6_supported_start.yaml

# 停止终端 A 的只读 launch，确认串口释放，采样后 15 秒内启动下列保位。
ros2 launch manipulator arm6_hardware_familiarization.launch.py \
  port_name:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  mode:=guidance start_guidance:=false use_rviz:=false \
  supported_startup_file:=/tmp/link6_supported_start.yaml external_support_confirmed:=true
```

保持外部支撑直到保位验证通过。等待 `/slam/state_valid=true` 且停稳，再启动 P 节点（CSV 文件须不存在）：

```bash
ros2 run manipulator camera_world_hold_controller.py --ros-args \
  --params-file src/arm-platform/config/camera_world_hold.yaml \
  -p publish_command:=false -p kp_position:=0.7 -p kp_rotation:=0.7 \
  -p max_linear_velocity:=0.02 -p max_angular_velocity:=0.08 \
  -p max_joint_velocity:=0.03 -p log_path:=$PWD/log/demo_reproduce.csv
```

另一个终端先录包：

```bash
ros2 bag record -o log/demo_reproduce_bag \
  /slam/camera_pose /slam/state_valid /joint_states \
  /student/applied_motor_command /student/speed_safety \
  /camera_world_hold/desired_pose /camera_world_hold/measured_link6_pose \
  /camera_world_hold/pose_error /camera_world_hold/joint_velocity \
  /camera_world_hold/actuator_joint_command
```

另一个终端准备接入（输出目录须不存在）：

```bash
# controller 必须先处于 READY，不能已经 HOLD。
ros2 run manipulator camera_world_hold_position_bridge.py \
  --execute --duration 60 --speed .03 --max-offset-deg 3 \
  --output log/demo_reproduce_trial
```

看到 `BRIDGE READY` 后，在 30 秒内 enable；此时捕获新 link6 目标：

```bash
ros2 service call /camera_world_hold/enable std_srvs/srv/SetBool '{data: true}'
# 退出补偿；bridge 在接收 dq 中断后调用原驱动 operator_hold。
ros2 service call /camera_world_hold/enable std_srvs/srv/SetBool '{data: false}'
```

只移动与 base 刚性锁紧的整个支撑件。demo 按上述小范围运行；本次没有完成 ±X/±Y/±Z、roll/pitch/yaw 全套验收。
下一步扩大实验范围前的实际问题是近奇异姿态、小命令关节跟随及行程边界；本次新视野下 SLAM 与驱动故障没有再次阻塞。
