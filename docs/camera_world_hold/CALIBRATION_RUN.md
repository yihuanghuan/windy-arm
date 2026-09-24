# 安装旋转标定与实机操作命令

当前受控对象为 **link6**。保留 `/camera_world_hold` 节点名、enable 服务及配置文件名。
`T_W_link6 = T_W_camera * inverse(T_housing_camera) * T_housing_link6(q6)`。
使用 link6 LOCAL Jacobian，将 world twist 旋转到 link6 轴后求阻尼伪逆，与 world Jacobian 方法等价；不使用在线 base observer。

所有终端使用同一环境：

```bash
cd ~/westlake/windy-arm
source /opt/ros/humble/setup.bash
source ~/westlake/ORB-SLAM3/ros2_install/setup.bash
source ~/westlake/windy-arm/install/setup.bash
export ROS_DOMAIN_ID=95 ROS_LOCALHOST_ONLY=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
```

## D435i / ORB-SLAM3

已有相机节点时不要重复启动。新会话：

```bash
ros2 launch orb_slam3_ros2 live.launch.py \
  orb_slam3_root:=/home/yihuang/westlake/ORB-SLAM3 \
  serial_no:=_244222077008 \
  result_dir:=/home/yihuang/westlake/windy-arm/log/camera_hold_live \
  use_viewer:=false
```

若仅重启 SLAM，保留已经运行的 RealSense 节点，并先停止旧 SLAM 进程：

```bash
ros2 run orb_slam3_ros2 stereo_inertial_node --ros-args \
  --params-file ~/westlake/ORB-SLAM3/ros2_install/orb_slam3_ros2/share/orb_slam3_ros2/config/d435i.yaml \
  -p use_viewer:=false -p max_pending_frames:=2 -p max_path_poses:=300 \
  -p trajectory_path:="$PWD/log/camera_hold_live/trajectory.tum"
```

等待 `/slam/state_valid=true`。SLAM 重启、地图重置或 base 改变固定位置后，应新建标定会话；不要把这些事件两侧的姿态直接作为同一固定 base 数据集。

## 固定 base，复用已验证 guidance 采样

base 必须锁紧，相机支架固定；继续使用当前已验证 guidance。不要为了采样切到位置模式。
新输出目录必须不存在已有 samples.json。

```bash
ros2 run manipulator calibrate_camera_mount_rotation.py \
  --config "$PWD/src/arm-platform/config/camera_world_hold.yaml" \
  --output "$PWD/log/mount_calibration_$(date +%Y%m%d_%H%M%S)" \
  --target-poses 6 --settle-seconds 2 --average-seconds 0.75 --exclude-joint5
```

每个姿态停稳至少 3 秒后按 ENTER；或者在另一终端执行：

```bash
ros2 service call /camera_mount_calibration/capture std_srvs/srv/Trigger '{}'
# 6–10 个姿态，用 joint1..4 覆盖至少两个旋转轴，joint5 保持不变后：
ros2 service call /camera_mount_calibration/solve std_srvs/srv/Trigger '{}'
```

不加 `--auto`，避免途中短暂停顿被采入。工具检查最近 2 秒关节静止，并平均末尾约 0.5 秒的同步视觉/FK 数据。
实测 SLAM 降到约 15–18 Hz 时，使用 `--average-seconds 0.75` 收集足够帧。
中断后仅在 base 与 SLAM 地图都未变化时，使用原输出目录并加 `--resume` 续采。
仅通过残差与已知平移检查的结果自动更新配置，同时保存原配置副本。
`samples.json` 保留原始平均姿态、时间戳、URDF 哈希和 TF；`result.json` 保存全部相对转角、残差及接受/拒绝原因。

现场确认 D435i 安装在 joint6 电机壳体上，不随输出端转动。
控制器使用新增 Pinocchio 固定参考系 `joint6_housing`（URDF joint6 原点、位于 joint6 旋转之前），其父关节为 joint5。
壳体相对 link5 的平移 `[0.084, 0, -0.035]` m、旋转 I；用户确认测量轴与 joint6=0 时一致。
壳体到左 IR 使用原平移 `[-0.0042, 0.018, 0.037]` m；等价 link5 到左 IR 平移为 `[0.0798, 0.018, 0.002]` m。
不存在适用于所有 joint6 角度的固定 `R_link6_camera`。
视觉测量对象为 camera_link，控制对象为 link6。配置字段 `joint6_to_left_optical_xyzw` 对应左 IR optical；工具利用实测 RealSense TF 完成旋转换算。
该字段的 joint6 明确指壳体固定参考系；输出结果使用 `R_joint6_housing_camera`。
相机 Jacobian 第 6 列为零，但它已不作为控制 Jacobian。当前使用 link6 Jacobian，joint6 的平移列为零、角速度列沿 housing x 轴，末端恢复完整六轴控制。

## Shadow

标定通过后，用正式配置启动；enable 前先准备录包。

```bash
ros2 run manipulator camera_world_hold_controller.py --ros-args \
  --params-file src/arm-platform/config/camera_world_hold.yaml \
  -p publish_command:=false \
  -p log_path:="$PWD/log/shadow_$(date +%Y%m%d_%H%M%S).csv"
```

```bash
ros2 bag record -o "$PWD/log/camera_hold_bag_$(date +%Y%m%d_%H%M%S)" \
  /slam/camera_pose /slam/state_valid /joint_states /student/joint_command \
  /camera_world_hold/desired_pose /camera_world_hold/pose_error \
  /camera_world_hold/measured_link6_pose \
  /camera_world_hold/command_twist /camera_world_hold/shadow_joint_command \
  /camera_world_hold/status
```

```bash
ros2 service call /camera_world_hold/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera_world_hold/enable std_srvs/srv/SetBool '{data: false}'
```

旧的 `log/camera_mount_calibration/session03/provisional_shadow.yaml` 是未通过验收的诊断配置，不用于真机 HOLD。
当前 watchdog 使用 steady-clock 接收间隔 `pose_receive_timeout=0.20 s`。
`pose_sensor_latency` 单独记录；header 仅有 `pose_header_max_age=1.0 s` 的宽松合理性检查。
`state_valid=false` 仍立即退出 HOLD，不自动恢复。没有实现延迟补偿。
CSV 中 `camera_*` 为原始视觉位姿，`measured_*`/`desired_*` 为 link6 世界位姿，另存 Jacobian 奇异值、rank、condition number、传感器延迟与接收间隔。

## 真机 HOLD（满足前提后执行；本次尚未执行）

前提：标定验收通过、shadow 方向正确，以及 base 有能承载当前重心且允许受控移动的支撑。
本次松开 base 固定时发生晃动，因此保持固定，不执行松开固定后的扰动。

机械臂静止且有可靠支撑时，停止 guidance，并等待其报告保位：

```bash
ros2 service call /arm6_manual_guidance/stop std_srvs/srv/Trigger '{}'
```

停止旧 guidance 节点和旧机械臂 launch，确认串口释放；不要同时启动两个串口驱动。随后：

```bash
ros2 launch manipulator student_arm6.launch.py \
  arm_type:=a_l1 \
  port_name:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  max_velocity:=0.5 control_rate_hz:=100.0 \
  enable_direct_effort_command:=false use_rviz:=true
```

退出 shadow 节点后另开终端：

```bash
ros2 run manipulator camera_world_hold_controller.py --ros-args \
  --params-file src/arm-platform/config/camera_world_hold.yaml \
  -p publish_command:=true \
  -p log_path:="$PWD/log/hold_$(date +%Y%m%d_%H%M%S).csv"
```

录包后，在 READY 状态调用 enable。先固定 base 观察 10–20 秒，再在可靠移动支撑条件下依次测试小幅平移、转动。

```bash
ros2 service call /camera_world_hold/enable std_srvs/srv/SetBool '{data: true}'
# 结束或出现异常：
ros2 service call /camera_world_hold/enable std_srvs/srv/SetBool '{data: false}'
# CSV 换成本次文件：
ros2 run manipulator plot_camera_world_hold.py log/hold_TIMESTAMP.csv
```

绘图仅呈现实际记录。shadow 的误差曲线不能解释为实机闭环稳定性能。
录包运行时不要直接用长事务查询 SQLite 文件；停止录包后再离线分析。
