# Link6 Phase 1 实机集成结果（2026-09-23，本轮）

本轮未完成真实 link6 HOLD。机械臂最后已从 guidance 切回驱动保位；shadow、标定和录包已停止。未发布任何本轮 link6 补偿命令。检查时 SLAM 进程已自行退出，日志报 Segmentation fault；RealSense 保留运行。

## 控制与软件验证

受控对象已改为 link6。相机固定在 joint6_housing，模型使用 joint6 旋转前原点，父 link5，平移 [0.084, 0, -0.035] m、旋转 I。
用户确认的壳体到左 optical 平移仍为 [-0.0042, 0.018, 0.037] m。

T_W_E = T_W_C * inverse(T_H_C) * T_H_E(q6)。
使用 link6 LOCAL Jacobian；将 world 线速度和角速度各自旋转进 link6 坐标，等价于使用完整 world Jacobian。没有添加高级控制或前馈。

33 项软件测试通过：q6 不改变 camera FK、会改变 link6 orientation；全部六列 world Jacobian 有限差分最大误差 3.659e-9；world Jacobian 与旋转后的 LWA Jacobian 一致；软件六方向补偿符号通过。
连续收到带 400 ms header 延迟的 Pose 不会退出 HOLD；停止接收会退出，恢复不自动重新 enable。watchdog 使用 steady-clock gap 0.20 s，header sanity 上限 1.0 s。

一般测试姿态 rank=6，singular values=[1.743608,1.555216,0.836741,0.251441,0.176853,0.049900]，未做量纲缩放的 condition=34.94。
最后采样姿态 rank=6，singular values=[1.715371, 1.537639, 0.903443, 0.22969, 0.175413, 0.004932]，condition=347.840。
q6 LOCAL Jacobian 列=[0,0,0,1,0,0]。不能把 rank=6 理解为远离奇异位形。

## 新标定：未接受

共 7 个静止姿态，21 个有效相对运动对；joint5 范围仅 0.2467 deg。候选 H→camera_link：

R_H_C =
[[ 9.972217479e-01  7.329569362e-02 -1.328634164e-02]
 [-7.448503423e-02  9.832579087e-01 -1.663005249e-01]
 [ 8.747881712e-04  1.668281337e-01  9.859856026e-01]]

q_H_C xyzw = [0.0836334809083147, -0.0035552167275946134, -0.03710103097949881, 0.9957993346078494]

对应配置字段 H→left optical 的候选 xyzw = [-0.43575480299621977, 0.5564893148840343, -0.4764110507033137, 0.5229435006321302]

平均/最大 AX=XB 旋转残差：2.986027/5.741744 deg。
平均/最大相对平移一致性误差：24.441/46.292 mm。
工具当前检查为旋转平均≤2°、最大≤5°，平移平均≤20 mm、最大≤50 mm；这组结果未通过，没有写入正式 camera_world_hold.yaml。
这些检查是工具阈值，不是声称存在通用硬件安全标准。

原 6 姿态结果平均2.612°、最大5.428°；加入第7姿态后没有改善。joint1 为主的第4→5姿态 FK/SLAM 为11.462°/14.768°；joint2 为主的第6→7为29.139°/33.217°。固定 mounting rotation 无法解释转角幅值差异。不能只归因于 joint5，也没有修改已确认的平移或编码器比例来拟合数据。

`UNACCEPTED_shadow_only.yaml` 只用于诊断，不可用于真实 HOLD。正式配置仍保留无效安装四元数。

## 实际 shadow / HOLD

shadow 节点启动，publish_command=false，并验证 /student/joint_command 发布者数量为0。随后 SLAM 跟踪丢失，地图 id/epoch 从0/1变为1/2，wrapper 停止正式 Pose 发布。enable 返回 pose receive gap exceeded，未进入 HOLD。
真实 q6 单轴静止/运动对比未完成。base 的三平移及三旋转方向 shadow 未完成；软件方向测试不能替代它们。
没有运行真实位置闭环，没有可报告的闭环位置/姿态误差或振荡结果。

## 数据与真正 blocker

- 新样本/完整残差：log/link6_world_hold/calibration_no_joint5/samples.json、result.json；保留6姿态及7姿态结果。
- 软件诊断：software_jacobian.json、final_sample_jacobian.json；测试日志 log/camera_mount_calibration/link6_all_tests.log。
- 原始录包：log/link6_world_hold/experiment_bag；shadow CSV：q6_shadow.csv（没有 HOLD 数据）。
- 当前需要解决 FK 与 SLAM 相对运动不一致，才能接受固定外参；单纯继续放宽残差阈值无法验证其正确性。
- 当前 SLAM 会话已经换图并停止，继续真机测试前必须恢复有效状态；旧采样不能与重启后地图混用。
- base 扰动前还需确认 base 与刚性支撑件整体可控移动。用户已收拢姿态，但本轮没有实际验证移动支撑条件。

复现命令见 CALIBRATION_RUN.md；其中真实 HOLD 命令仍是未满足前提的后续步骤，不能当作本轮已测试通过。
