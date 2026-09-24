# 2026-09-23 实测结果

## Calibration

现场确认：D435i 固定在 joint6 **电机壳体**，不随 joint6 输出端旋转。
用户确认测量坐标轴与 URDF joint6=0 时一致。
因此不存在与 q6 无关的固定 `T_link6_camera`；当前模型使用 `joint6_housing`。

```text
T_link5_housing: R=I, p=[0.084, 0, -0.035] m
p_housing_left_optical=[-0.0042, 0.018, 0.037] m
p_link5_left_optical=[0.0798, 0.018, 0.002] m
T_left_optical_camera_link: p=0, q_xyzw=[0.5,-0.5,0.5,0.5]
```

最终固定 base 会话有 8 个静止姿态、28 个可用相对运动对。候选解（**未验收**）：

```text
R_joint6_housing_camera_link =
[[ 0.997568433,  0.047589309,  0.050916399],
 [-0.042488331,  0.994380908, -0.096960569],
 [-0.055244581,  0.094561450,  0.993984994]]

q_joint6_housing_camera_link_xyzw =
[0.047964911, 0.026587032, -0.022559109, 0.998240243]

候选 joint6_to_left_optical_xyzw（壳体→左 optical）=
[-0.477151628, 0.547675647, -0.473123705, 0.498529507]
```

AX=XB 旋转残差平均 3.089°、最大 6.862°，未通过当前平均≤2°、最大≤5°门限。
已知平移相对运动检查：平均 3.492 mm、最大 7.777 mm。
没有删除残差大的样本，没有将候选旋转写入正式配置；正式四元数仍为明确无效值。

短时间动作回查仍有转角幅度不一致：主要 joint5 动作中，FK/SLAM 分别约 18.41°/15.97°、18.83°/15.52°；主要 joint4 动作约 32.50°/31.88°、25.41°/25.25°。
固定安装旋转无法改变这些相对转角幅度，暂未确定原因属于关节测量/模型还是视觉估计。

原始关节角、静止平均姿态、逐对残差、TF 和 bag：
`log/camera_mount_calibration/session03/`。
`samples_original_link6.json` 保存纠正壳体安装事实之前的 FK；当前样本按保存的 q 重算 FK，原始值也保留在样本字段中。

## Shadow result

在真实 ROS 输入上使用**候选外参、publish_command=false**运行了壳体模型。
相机 Jacobian 第六列、dq6 均为零；没有 `/student/joint_command` 发布者。
现有 P 算法未改变。相机实际仅受前五个关节控制，Jacobian 最大秩 5，不能独立补偿任意 6D 扰动。

最终静止 shadow 记录 199 个 HOLD 周期、约 7.92 秒，随后因 pose 超时退出。
消息年龄中位数 160 ms、99% 分位 295 ms、最大 437 ms，超过该诊断会话的 350 ms 超时阈值。
没有自动恢复 HOLD。未完成任何轴的受控 base 扰动方向验收。

这段**无闭环动作的静止测量波动**：位置误差范数中位数/最大值 0.233/0.420 mm；姿态误差范数 0.049/0.074°。
这些数值不是机械臂补偿精度。曲线：`log/camera_mount_calibration/session03/shadow.png`。

## Hardware HOLD

**未执行真实 camera pose HOLD，未发布补偿位置指令，未验证反向补偿或闭环振荡。**
用户报告松开 base 固定时因偏心重心出现晃动，随后确认已重新锁紧、可靠支撑并停稳。
已取消 base 移动提示。guidance 经过静止检查后 operator-stop，原驱动仍保位；最终无 fault_axes。
标定节点、shadow 节点和本轮 bag 已停止；相机/SLAM及原机械臂驱动保留。

## Software tests

构建、Python 导入/语法、真实节点启动与采样接口通过。
完整测试 28 passed，包括壳体 FK、Jacobian 有限差分、q6 不影响相机、ROS 状态/接口及 C++ 驱动 sim 后端；最后采样窗口修改后的 6 项标定测试再次通过。
所有合成 ROS 发布器均使用隔离域 196；真机域为 95。

## Commands

完整命令见 [CALIBRATION_RUN.md](CALIBRATION_RUN.md)。其中真机位置模式/HOLD命令尚未在本轮执行。

## Remaining blocker

1. 固定 base 下的 FK/视觉相对转角仍不一致，外参拟合未验收。
2. SLAM 消息延迟会触发 HOLD 退出，连续静止 HOLD 尚未通过。
3. 当前 base 松开固定即晃动，没有可支撑偏心重心并允许受控扰动的条件。
