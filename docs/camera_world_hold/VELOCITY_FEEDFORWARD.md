# Phase 2：可关闭的速度前馈

Phase 1 已冻结：提交 `1e9ae76`，标签 `link6-world-hold-phase1-20260924`。
该基线完成过约 **10 cm / 5 s** 手动水平 base 扰动：峰值误差 34.18 mm / 3.99°，末段 10 s 均值 2.51 mm / .82°。
这些是 SLAM+编码器重建的误差，非独立计量。扩大实验记录、误差图及复现命令见 [EXTENDED_RESULT_20260924.md](EXTENDED_RESULT_20260924.md)。

开发分支：`agent/link6-velocity-feedforward`。**新前馈尚未真实接入电机。** 用户离开期间所有运动接入已停止，驱动继续保位。
默认两个 feedforward gain 均为 0，原 Phase 1 参数和行为保留。没有加入加速度、SMC/NMPC、base 位姿估计器或状态预测。

## 计算

`/slam/camera_twist` 的源码确认：线速度在 camera_link 原点，角速度和线速度均以 orb_map 表示，时间戳与 camera_pose 相同。
相机固定在 housing，所以不能把 camera twist 直接当作 link6 twist，也不能把机械臂主动补偿产生的速度再次当成外部扰动。

在同一图像时刻 `s`，用已标定的相机 frame 和编码器计算：

```text
V_external_C = V_camera_measured - J_camera_world(q_s) * dq_s
r = p_world_link6(s) - p_world_camera(s)
V_external_E.linear  = V_external_C.linear + V_external_C.angular × r
V_external_E.angular = V_external_C.angular

V_E_measured = J_link6_world(q_s) * dq_s + V_external_E
V_feedforward = -G * deadzone(V_external_E)
V_command = clamp(Kp * pose_error + clamp(V_feedforward))
```

随后沿用原六列 link6 Jacobian、阻尼伪逆、关节限速及已有位置积分接口。
q6 的相机 Jacobian 列为零；q6 的 link6 角速度贡献保留。这个前馈是直接运动学速度分离，不积分/估计 world→base pose。
它与 `-Kd * V_E_measured` 速度反馈阻尼不同，本次没有把两种项混在一起。

编码器位置在图像时间戳插值，关节速度用同一时刻附近默认 100 ms 的短窗直线拟合，降低量化噪声。
窗口必须包含该图像时刻，有至少 4 个样本、跨度至少半个窗口，相邻反馈间隔不超过 40 ms；不外推到未来。
Pose/Twist 必须同 stamp，且不能比最新 Pose 落后超过 80 ms。
这只是输入对齐，没有把滞后的 SLAM 状态预测到当前时刻。

## 参数与退出

所有参数在 `camera_world_hold.yaml`，启动时读取：

| 参数 | 默认 | 含义 |
|---|---:|---|
| velocity_feedforward_linear_gain / angular_gain | 0 / 0 | 范围 0–1；0 关闭该部分 |
| feedforward_max_linear_velocity | .03 m/s | 前馈自身限幅，叠加后仍受原总限幅 |
| feedforward_max_angular_velocity | .10 rad/s | 同上 |
| feedforward_linear_deadband | .003 m/s | 静止速度噪声死区 |
| feedforward_angular_deadband | .01 rad/s | 静止角速度噪声死区 |
| joint_velocity_window | .10 s | 编码器拟合窗口 |
| twist_receive_timeout | .20 s | Twist 接收看门狗 |
| feedforward_max_sample_lag | .08 s | 与最新 Pose 的最大时间戳落后 |

Twist 缺失、错 frame、非有限值、过期或无法匹配时，**当周期前馈清零，继续原 P 控制**，不会保留旧速度。
Pose 接收中断、SLAM invalid、关节反馈中断仍按 Phase 1 退出 HOLD，恢复后须重新 enable。
`ff_valid/ff_reason` 明确区分前馈是否生效。

新增诊断：`~/feedforward_twist`、`~/external_link6_twist`、`~/measured_link6_twist`。
后两个 topic 使用匹配的图像时间戳；命令 topic 使用命令时刻。
CSV 同时记录 camera velocity、自身运动速度、外部扰动速度、前馈、匹配时间戳、延迟及有效性。
电机命令中的 velocity 字段仍为 0，所有补偿最终都通过位置指令执行。

## 已完成验证

- 完整测试 **65 passed**：原 P 回归、所有六列 Jacobian、混合 base/关节运动有限差分、q6、杆臂、对齐、限幅、丢失/错 frame/NaN 回退、隔离 ROS 通信。
- 理想运动学闭环中，前馈能在 pose error 尚为零时产生反向补偿；降低运动误差。这不是动态仿真或硬件性能证明。
- Python syntax/import、ROS package build 通过。
- 静止真机 **仅 shadow**：61 个控制样本（约 2.4 s），全部成功对齐；线速度最大约 1.66 mm/s，角速度最大约 .0075 rad/s，前馈均为零。
- 现场两个运动命令 topic publisher count 均为 0；没有启动 bridge。
- shadow 因老 SLAM 会话 > .20 s 接收间隔退出，正确回到 READY。长期视觉积压仍存在，OpenCV 单线程设置未彻底解决，不能宣称长时 shadow 或前馈真实闭环通过。

数据：`log/link6_demo_20260924/extended/phase2_tests.log`、`phase2_shadow.csv`、`phase2_shadow_metrics.json`。

## 下次只读 shadow 命令

先复用既有相机、SLAM、机械臂保位会话，确认只有一个 controller，没有运动 bridge。**这些命令本身不接入电机。**

```bash
cd ~/westlake/windy-arm
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=95 ROS_LOCALHOST_ONLY=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
ros2 run manipulator camera_world_hold_controller.py --ros-args \
  --params-file src/arm-platform/config/camera_world_hold.yaml \
  -p publish_command:=false \
  -p velocity_feedforward_linear_gain:=0.5 \
  -p velocity_feedforward_angular_gain:=0.5 \
  -p log_path:=$PWD/log/phase2_next_shadow.csv
```

另一个同环境终端，在 READY 后抓取当前 link6 目标，进行 shadow 计算：

```bash
ros2 service call /camera_world_hold/enable std_srvs/srv/SetBool '{data: true}'
# 结束：
ros2 service call /camera_world_hold/enable std_srvs/srv/SetBool '{data: false}'
```

真实前馈测试待用户回到现场、视觉连续性恢复并完成运动 shadow 后进行。不要把 Phase 1 的成功当成 Phase 2 实机验证。
