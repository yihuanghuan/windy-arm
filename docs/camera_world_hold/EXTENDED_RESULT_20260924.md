# 2026-09-24 link6 实机抗扰：扩大幅度实验

Phase 1 冻结记录。目标为用户指定的 **10 cm / 5 s** 水平 base 整体扰动；手动距离/时间是现场近似值，没有外部计量设备。
已完成约 10 cm / 5 s 的一次真实补偿并停稳；双向复核尚未完成：用户离开用餐，主动停止本轮并要求冻结；后续不得补记为通过。一次提高增益/限速没有证实更好。
较早的 `DEMO_RESULT_20260924.md` 是近奇异姿态下的首个小幅 demo，不能代表当前结果。

## 已验证结果

D435i 仍固定在 joint6 housing。外参及 `T_W_E = T_W_C * inv(T_H_C) * T_H_E(q6)` 未变；六个关节都参与 P 控制。
自主慢速调整腕部后，Jacobian 条件数从约 1.5 万降到约 15–24；没有更改 URDF、编码器比例或加入高级控制。

| 实际接入试验 | 参数/退出情况 | 峰值位置/姿态误差 | 末段结果 |
|---|---|---|---|
| `bent_3cm`，约 3 cm / 5 s | Kp .7/.7，关节限速 .08 rad/s；人工停止前无保护退出 | 20.7 mm / 3.05° | 用户：调整几次后稳定 |
| `bent_5cm`，约 5 cm / 5 s | Kp 1.2/1.0，.08 rad/s；人工停止前无保护退出 | 35.4 mm / 5.47° | 最后 10 s 均值 2.63 mm / .84°；用户：能跟上并稳定 |
| `thread1_rate12_5cm`，约 5 cm / 5 s | Kp 1.2/1.0，.12 rad/s；人工停止前无保护退出 | 19.67 mm / 3.33° | 最后 10 s 均值 2.62 mm / .94°；用户确认反向补偿且稳定 |

| `thread1_rate12_75mm`，约 7.5 cm / 5 s | Kp 1.2/1.0，.12 rad/s；人工停止前无保护退出 | 23.95 mm / 4.44° | 最后 10 s 均值 2.18 mm / 0.82°；用户确认补偿且停稳 |
| `thread1_rate12_10cm`，约 10 cm / 5 s | Kp 1.2/1.0，.12 rad/s；人工停止前无保护退出 | 34.18 mm / 3.99° | 最后 10 s 均值 2.51 mm / 0.82°；用户确认补偿且停稳 |

以上误差来自相机与编码器重建的 link6，不是独立真值。各次手动扰动不同，不能仅凭表格宣称参数提升百分比。
`rate12_5cm` 因用户意外大角度晃动而触发 joint1 速度保护；`rate12_5cm_retry` 因视觉接收间隔 > .20 s 退出。
`thread1_rate16_10cm` 尝试 Kp=1.8、关节上限 .16，触发 40° 试验行程边界，退出时仍约 15 mm / 7.5° 误差，也不能算收敛成功；随后恢复 1.2/.12 参数。
这几次保护退出不能算收敛成功。故障确认停稳后才恢复，没有放宽驱动保护。

实际数据：`log/link6_demo_20260924/extended/`。各成功试验目录包含 `streams.jsonl`、`active.csv`、`metrics.json` 和误差图。
10 cm 预检查预计最大约 28° 关节运动；实际最大约 32.2°。该轮 bridge 行程边界设为 40°，原关节限位裕量和驱动保护不变。
`active.csv` 只裁剪真实 bridge 接入窗口，不混入 shadow 或退出后的保位数据。全程 ROS bag 在 `extended/bag/`。

## 本轮修正及限制

- 保留原驱动承重位置偏差，从 authoritative hold reference 积分 P 控制 dq，电机 velocity 字段为零。
- 使用已有 student_arm6 的位置 PD：P=[35,35,35,5,5,5]，D=[1.5,1.5,1.5,1,1,1]。
  诊断确认旧增益下 joint2 ±1° 指令不产生可分辨位移，±2° 出现运动和滞后；没有猜测模型比例。
- 3 mm / 1° 连续死区，避免静止噪声导致反复小调整；日志仍记录原始误差。
- bridge 可 `--duration 0` 持续工作；退出时同时 disable 控制器并交回驱动保位，避免 UI HOLD 与实际执行不一致。
- 桥接器 .2 rad/s 只是软件允许上界；已成功扰动测试到 .12，不代表 .2 已验证。
- SLAM 老会话出现 22 Hz、264 ms 偶发处理耗时，触发 .20 s 接收看门狗。只给 wrapper 添加 `opencv_threads` 参数，默认 0 保持原行为。
  本轮以 1 重启后短时 30 Hz、约 20–30 ms 接收延迟；约 10 分钟后仍出现少量队列丢帧，线程限制没有彻底解决长期积压；本轮成功扰动窗口未因此退出。
- 没有放宽 .20 s 接收看门狗，也没有添加延迟补偿、camera twist/accel 或 base estimator。

软件验证：全套 49 项中 48 项首次通过；另一项为标定测试硬编码旧默认 gain，已改为检查所有非外参字段不变，相关 6 项重测全部通过。
机械臂 package 构建与 ORB wrapper 编译通过。

## 复现当前配置

每个终端：

```bash
cd ~/westlake/windy-arm
source /opt/ros/humble/setup.bash
source ~/westlake/ORB-SLAM3/ros2_install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=95 ROS_LOCALHOST_ONLY=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
```

仅在相机和 SLAM 未运行时启动（不要重复占用设备）；先固定 base，完成静止 IMU 窗口和小幅初始化运动：

```bash
mkdir -p log/hold_reproduce_slam
ros2 launch orb_slam3_ros2 live.launch.py \
  serial_no:=_244222077008 use_viewer:=false opencv_threads:=1 \
  result_dir:=$PWD/log/hold_reproduce_slam
```

运行中的机械臂保位驱动可直接复用。冷启动仍按 `DEMO_RESULT_20260924.md` 的 supported startup 流程获取新鲜反馈并验证支撑，
保位 launch 额外指定 `config:=$PWD/src/arm-platform/config/arm6_world_hold_position.yaml`。
**不要复用实验目录旧 `*_seed.yaml`，不要在已有驱动占用串口时再启动驱动。**

控制器先 READY（仓库默认 .12 为本轮已验证限速）：

```bash
ros2 run manipulator camera_world_hold_controller.py --ros-args \
  --params-file src/arm-platform/config/camera_world_hold.yaml \
  -p max_joint_velocity:=0.12 -p log_path:=$PWD/log/hold_reproduce.csv
```

录包使用 `DEMO_RESULT_20260924.md` 的 topic 清单。另一个终端准备真实位置接入：

```bash
ros2 run manipulator camera_world_hold_position_bridge.py \
  --execute --duration 0 --speed .12 --max-offset-deg 40 \
  --output log/hold_reproduce_trial
```

看到 `BRIDGE READY` 后 30 秒内开启，抓取当前真实 link6 目标：

```bash
ros2 service call /camera_world_hold/enable std_srvs/srv/SetBool '{data: true}'
# 退出补偿，交回原驱动保位：
ros2 service call /camera_world_hold/enable std_srvs/srv/SetBool '{data: false}'
```

`publish_command=false` 只表示 controller 不直发 `/student/joint_command`；`bridge --execute` 仍会真实驱动。
有效 bridge 与 HOLD 都在线才执行人工扰动。SLAM 或驱动保护退出后必须重新检查、人工 enable；不自动追旧目标。

冻结时真实驱动为 operator HOLD，无补偿执行节点。不得自动恢复此前目标。
基线对应的 ORB 工作区补丁、上游 SHA 和可执行文件哈希保存在 `log/link6_demo_20260924/extended/phase1_freeze/`；未提交 ORB 仓库已有的无关修改。
