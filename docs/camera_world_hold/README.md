# D435i world pose hold — Phase 1

> 当前实机扩大扰动记录和命令见 [EXTENDED_RESULT_20260924.md](EXTENDED_RESULT_20260924.md)。
> 冻结后的可选速度前馈见 [VELOCITY_FEEDFORWARD.md](VELOCITY_FEEDFORWARD.md)，默认关闭，尚未实机接入。
> 下文保留早期审计；其“外参未定/尚未闭环”状态和默认增益已被后续实验更新。

> 现场更正（2026-09-23）：D435i 安装在 joint6 **电机壳体**，不随 joint6 输出端转动。
> 当前配置已改为 `joint6_housing`，不能再使用下文早期审计中的固定 `link6 → camera` 假设。
> 最新控制对象已改为 **link6 完整 6D pose**：`T_W_E = T_W_C * inverse(T_H_C) * T_H_E(q6)`。
> 使用 link6 的完整六列 Jacobian，q6 参与姿态补偿。节点/文件名保留以兼容命令。
> 以下正文是早期 camera hold 记录；当前标定与复现命令见 [CALIBRATION_RUN.md](CALIBRATION_RUN.md)。

2026-09-23：实现/构建/软件测试完成；真实相机和机械臂只读联合测试通过。
**尚未进行真实补偿运动：缺少 joint6/link6 → 相机安装旋转。不要把默认零四元数改成随意的单位四元数。**

## A. 实际审计

| 项目 | 代码与现场结论 |
|---|---|
| active DOF | **6**，驱动 `a_l1/gamma6`，配置创建六个电机；固件第七通道不参与 |
| active joints | `joint1, joint2, joint3, joint4, joint5, joint6` |
| EE / reference | `link6`；Pinocchio joint6 的随动原点/坐标轴与其 child `link6` 相同 |
| command | `/student/joint_command`, `sensor_msgs/msg/JointState`，六个 `position`，按 joint1…joint6 排序。驱动不按 name 重排，因此 controller 明确以该顺序输出 |
| feedback ordering | 实测 `/joint_states` 是 joint1…joint6。controller 按 name 重排输入，维数/关节集合不匹配则拒绝 |
| model | `src/arm-platform/config/arm6.urdf`，安装后 `install/manipulator/share/manipulator/arm6.urdf`，模型角度直接使用编码器 q，与当前 guidance 的零 offset 一致 |
| SLAM pose | `/slam/camera_pose` 是 **orb_map → camera_link**，不是原始左 IR optical 位姿 |
| TF | 实测 `base_link → link1 → … → link6`；另一个树为 `orb_map → camera_link → RealSense sensor frames`；没有 link6 → camera 安装边，也没有 world → base |

审计依据：`student_arm6.launch.py`、`arm6_hardware_familiarization.launch.py`、
`student_arm_node.cc::StudentCommandCallback`、`arm_config.yaml::a_l1_gamma6`、
`arm_platform.cc::NotifyJointState`，以及 ORB-SLAM3
`ros2/orb_slam3_ros2/src/stereo_inertial_node.cpp::ensureCameraLinkTransform/publishPose`
（查找 `left_optical_to_camera_link_` 与 `inertial_state.Twc`）。
Wrapper 执行 `T_map_left * T_left_camera_link`，再应用固定的初始输出原点变换。

现场 ROS 使用 **ROS_DOMAIN_ID=95、ROS_LOCALHOST_ONLY=1**。不要混入默认 domain 0。
现场原有 launch 是 `arm6_hardware_familiarization ... mode:=guidance`，它显式禁用位置命令。
相机 controller 的真实输出模式会检查驱动参数并拒绝在 guidance/direct effort 模式 enable。

## B. 几何与计算

控制 frame：`camera_link`；Pinocchio 内部固定 frame：`d435_camera_control`。
不向全局 TF 发布 link6→camera_link（否则会给现有 camera_link 增加第二个父节点）。

记 `T_A_B` 为把 B 中坐标转换到 A 的变换：

```text
T_link6_camera = T_link6_left_optical * T_left_optical_camera
p_link6_left   = [-0.0042, +0.018, +0.037] m
R_link6_left_optical = 待提供，参数 joint6_to_left_optical_xyzw
```

本次真实 D435i 的 TF 查询 `tf2_echo camera_infra1_optical_frame camera_link`：

```text
p_left_camera = [0, 0, 0]
q_left_camera (xyzw) = [0.5, -0.5, 0.5, 0.5]
R_left_camera = [[0,-1,0], [0,0,-1], [1,0,0]]
```

因此本次 `p_link6_camera=[-0.0042,0.018,0.037] m`，旋转为未知安装旋转乘上述已知旋转。
代码始终从 RealSense TF 读取内部变换，不硬编码以上测量。
右 IR/RGB 原点数据不用于 stereo camera_link 的杆臂。
三个给定原点共线，只能约束一根安装轴，不能恢复完整安装旋转。
相邻 `windylab-arm-for6` 的仿真默认安装位置 `[0.06,0,0.04]` 与本次不同，其 rpy=0 不作为实机外参。
Vicon 的 `arm_ee` 刚体旋转也不是 D435i 安装旋转。

```text
e_p = p_des - p_cur
e_R = log(R_des R_cur^T)
V_world = clamp([kp_position e_p, kp_rotation e_R])
V_camera = diag(R_cur^T, R_cur^T) V_world
J = Pinocchio computeFrameJacobian(camera fixed frame, LOCAL)
dq = J^T (J J^T + lambda^2 I)^-1 V_camera
q_cmd = clip_joint_limits(q_current + clamp_joint_speed(dq) dt)
```

LOCAL Jacobian 含 fixed frame 杆臂，twist 的原点一直是相机原点，所以只转坐标轴，
不再用世界平移做第二次 adjoint。没有 base pose 重建，也没有 twist/accel 前馈。
本次默认 25 Hz，Kp=1.5/s，线速度 0.05 m/s，角速度 0.2 rad/s，关节速度 0.3 rad/s，
低于现有 position driver 默认 0.5 rad/s。参数启动后只读，修改后重启并重新 enable。

WAITING 等待新鲜且有效的三个输入；READY 不运动；SetBool enable 才捕获当前目标进入 HOLD。
重复 enable 不覆盖旧目标。disable/SLAM false/超时退出 HOLD；有新鲜关节反馈时发送一次当前 q 保位，
否则交给原驱动 watchdog。恢复数据后保持 READY，必须重新 enable，重新捕获目标。
shadow 模式根本不创建 `/student/joint_command` publisher。

## C. 运行命令

每个终端先执行：

```bash
cd ~/westlake/windy-arm
source /opt/ros/humble/setup.bash
source ~/westlake/ORB-SLAM3/ros2_install/setup.bash
source ~/westlake/windy-arm/install/setup.bash
export ROS_DOMAIN_ID=95 ROS_LOCALHOST_ONLY=1
```

构建（代码已在本机完成构建）：

```bash
colcon build --packages-select manipulator
source install/setup.bash
```

1. D435i/ORB-SLAM3：确认没有另一个 RealSense 节点占用设备，再启动。

```bash
mkdir -p ~/westlake/windy-arm/log/camera_hold_live
cd ~/westlake/windy-arm/log/camera_hold_live
ros2 launch orb_slam3_ros2 live.launch.py \
  orb_slam3_root:=/home/yihuang/westlake/ORB-SLAM3 \
  serial_no:=_244222077008 \
  result_dir:=/home/yihuang/westlake/windy-arm/log/camera_hold_live \
  use_viewer:=false
```

缓慢移动/转动相机完成初始化，直到 `/slam/state_valid=true`。
本次首次 IMU 没有消息，通过只复位序列号 244222077008 的 D435i 后恢复；没有修改 SLAM。

2. 真实机械臂：若还在现有 guidance 会话中，先请求已有保位，再在原终端 Ctrl-C 停止 guidance 与旧 arm launch，释放串口。不要同时启动两个串口 owner。模式切换时支撑机械臂；此切换尚未在本任务实测。

```bash
ros2 service call /arm6_manual_guidance/stop std_srvs/srv/Trigger '{}'
# 确认返回成功，在原 guidance 和 arm launch 终端 Ctrl-C 后：
ros2 launch manipulator student_arm6.launch.py \
  arm_type:=a_l1 \
  port_name:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  max_velocity:=0.5 control_rate_hz:=100.0 \
  enable_direct_effort_command:=false use_rviz:=true
```

3. 只读联合检查（现有 guidance 模式就能执行）：

```bash
ros2 run manipulator camera_world_hold_monitor.py
```

4. **补齐可信安装旋转**：编辑
`~/westlake/windy-arm/src/arm-platform/config/camera_world_hold.yaml` 中
`joint6_to_left_optical_xyzw`，填写 `R_link6_left_optical` 的单位四元数 `[x,y,z,w]`。
它将左红外 optical 向量映射到 link6。若已知的是 `R_link6_camera_link`，先乘 RealSense 的
`R_camera_link_left_optical` 再转四元数。不要用 `R_left_camera` 代替未知安装旋转。

5. Shadow：允许执行完整计算和记录，但不发实机指令。

```bash
cd ~/westlake/windy-arm
ros2 run manipulator camera_world_hold_controller.py --ros-args \
  --params-file src/arm-platform/config/camera_world_hold.yaml \
  -p publish_command:=false \
  -p log_path:="$PWD/log/shadow_$(date +%Y%m%d_%H%M%S).csv"
# 另一个终端，READY 后：
ros2 service call /camera_world_hold/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /camera_world_hold/status
# 小幅人工移动 base，检查 error、twist、dq、q_cmd；结束后：
ros2 service call /camera_world_hold/enable std_srvs/srv/SetBool '{data: false}'
```

6. 实机闭环：退出 shadow 节点，确认机械臂已经启动位置接口。

```bash
cd ~/westlake/windy-arm
ros2 run manipulator camera_world_hold_controller.py --ros-args \
  --params-file src/arm-platform/config/camera_world_hold.yaml \
  -p publish_command:=true \
  -p log_path:="$PWD/log/hold_$(date +%Y%m%d_%H%M%S).csv"
# READY 后（另一终端）：
ros2 service call /camera_world_hold/enable std_srvs/srv/SetBool '{data: true}'
# 静止观察，再依次小幅 ±X/±Y/±Z、roll/pitch/yaw、组合扰动。
# 停止：
ros2 service call /camera_world_hold/enable std_srvs/srv/SetBool '{data: false}'
```

7. 录制与绘图（录制先于 enable）：

```bash
ros2 bag record -o "$PWD/log/hold_bag_$(date +%Y%m%d_%H%M%S)" \
  /camera_world_hold/desired_pose /slam/camera_pose \
  /camera_world_hold/pose_error /camera_world_hold/command_twist \
  /joint_states /student/joint_command /slam/state_valid \
  /camera_world_hold/status /camera_world_hold/shadow_joint_command
# 停止控制后，以本次 CSV 路径替换下面参数：
ros2 run manipulator plot_camera_world_hold.py log/hold_YYYYMMDD_HHMMSS.csv
```

CSV 记录 measured/desired 位姿、xyz 误差、SO(3) log 误差、`R_des R_cur^T` 的 RPY、
q、dq、q_cmd、消息年龄、valid、状态和是否实际发布命令。RPY 图用于显示，控制律使用 SO(3) log。
CSV 路径必须不存在，防止覆盖上次实验；没有 HOLD 样本时绘图会明确拒绝生成性能图。

## D. 本次测试证据与结果

- syntax/import、安装后 `ros2 run` 启动：通过。
- 17 项数学测试 + 3 项 ROS 状态/接口测试：20 passed；另有真实 C++ driver 的 sim backend 接口测试：1 passed。
- 相机 LOCAL Jacobian 数值有限差分最大误差 `2.20e-9`；测试含非零安装旋转和杆臂。
- 12 个 ±translation/rotation 软件扰动，80 次小步 P 控制后误差均降到初始的 30% 以下。
- ROS 测试含 shadow 零实机输出、READY 零输出、enable 捕获、重复 enable、失效保位、超时、恢复不追旧目标、错误 frame/NaN 拒绝。
- 现有 `student_arm6.launch.py arm_type:=sim` 的 C++ `/student/joint_command` 通路实际接收六轴命令并改变反馈；未触碰串口。
- 真机 read-only：6 秒内 pose 180 条（29.99 Hz）、valid 180 条、关节 600 条（99.98 Hz）、gyro 1200 条、accel 608 条。
- 真机 controller 达到 READY，明确报告 mounting rotation missing。**没有启用真机 HOLD，没有实机扰动验收结果。**
- 真机图上的 enable 服务实测返回 `success=False, joint6 -> camera mounting rotation missing`。
- 本次启动的相机/SLAM 与测试节点已退出；原机械臂 guidance 会话仍在运行。
- 真实采集文件在 `log/camera_world_hold/`：`hardware_audit.json`、`readonly_after_reset.log`、
  `installed_startup.log`、`hardware_readonly.csv`、`readonly_bag/`、构建与测试日志。
- `software_interface.csv/.png` 是软件接口测试产物，不能作为实机效果证据。

复现软件测试（**独立域 196，不能在真机域 95 运行合成发布器**）：

```bash
source /opt/ros/humble/setup.bash
source ~/westlake/windy-arm/install/setup.bash
cd ~/westlake/windy-arm
ROS_DOMAIN_ID=196 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 -m pytest -q \
  src/arm-platform/test/camera_world_hold
```

下一步仅需补齐可信的 **joint6/link6 → camera mounting rotation**，随后执行上面的
shadow、位置模式切换和真机 HOLD 验收。现有 guidance 的位置接口禁用是明确的模式设置，并非待开发功能。
