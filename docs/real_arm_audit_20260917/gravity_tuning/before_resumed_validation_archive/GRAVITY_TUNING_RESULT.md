# GRAVITY TUNING RESULT

2026-09-18，更新通信分级后的实际执行结果。**最终 INCONCLUSIVE。已完成 8 秒纯读取复检和 3 轮主动 tuning；第 4 次尝试首写前因当前姿态超出计划几何包络停止。** 没有完成两次 1.2 s 重复，不能宣布 GRAVITY BRING-UP PASS。

## 1. 最终停止的具体原因

Run03 最后收到 J5=**0.044307709 rad**；Attempt04 的新鲜反馈为 **0.073844910 rad**，相差 **+0.029537201 rad（约 1.69°）**。Attempt04 计划以 Run03 末姿态为几何中心，J5 几何 envelope 小于此变化，首写前 `live pose outside geometry box` 阻止执行。该次 **write attempts=0、TX=0**，未锁存可发送目标。

用户随后明确补充：**“我稍微碰了下末端关节”**。这是两次记录间发生人工接触的现场信息，因此不能把0.02954rad差异用作“自行漂移”或板端异常的证据。两次会话之间没有连续反馈，精确接触时刻、力度和全部位移归因仍未知；实体动力隔离是否实施也未知。记录见 [operator_contact_observation.json](comm_recheck_20260918/operator_contact_observation.json)。

这次停止不是单个 0x22 坏帧，也不是旧的 0.0015 rad 位移 guard。新的通信策略确实通过后继续执行了多轮；最终已有安全计划无法覆盖新姿态，按 hard abort 边界关闭串口、不恢复、不自动重试。**关闭串口不代表电机失能；实体动力隔离仍是最终安全边界。**

## 2. 通信基线与新策略

汇总 10 份历史传输正常记录（含旧平台反馈的通信时序，不等于旧硬件状态全部有效），独立 position/velocity gap 样本分别 16452/16448；损坏的单块捕获被排除。

| 流 | max gap ms | p99 ms | p99.9 ms |
| --- | ---: | ---: | ---: |
| 0x20 | 9.030 | 5.778 | 6.200 |
| 0x21 | 15.021 | 7.743 | 9.794 |

历史典型间隔 4.996369 ms。使用 `ceil_ms(max(max_gap+2*典型间隔, 2*p99.9+典型间隔))`，得到 **26 ms**。15.021+2×4.996≈25.014 ms，向上取整为26，而非最初口头估计25。不是声称 26 ms 已通过系统稳定性证明；它是根据观测推导的失去可信状态前停止界限。超过30ms不静默放宽。

COMM_WARNING：单个 checksum/malformed 或诊断损失，丢弃坏帧，继续使用新鲜的合法 q/dq；坏帧绝不更新状态。

COMM_SOFT_FAIL：1秒内≥3个非连续错误、关键流仍新鲜，或只读预检频率显著偏离历史带。首 TX 前最多两次自动 close/0.3s等待/reopen；本次无需重试。

COMM_HARD_ABORT：q/dq age≥26ms、连续2个关键坏帧、250ms内≥5坏帧且错误率≥2%、关键数据非有限、串口异常/断开，或通信损坏与扩大方向矛盾同时发生。active hard abort 不恢复、不重试。低频 0x23/25/01 不套用26ms状态期限。

8.000秒只读实测：

| cmd | 合法帧数 | Hz（去初始50ms缓存） | max gap ms |
| --- | ---: | ---: | ---: |
| 0x20 | 1610 | 200.806 | 5.363 |
| 0x21 | 1610 | 200.806 | 9.239 |
| 0x22 | 1610 | 200.811 | 9.881 |
| 0x24 | 1609 | 200.896 | 9.688 |
| 0x23 | 16 | 2.008 | 498.019 |
| 0x25 | 16 | 2.008 | 498.086 |
| 0x1 | 161 | 20.081 | 50.114 |

checksum/malformed/nonfinite=0；consecutive/burst=0；最大关键 age **8.983 ms**。O_RDONLY，serial write=0。三轮主动以及第4次预检通信也均无解析错误或 stale，关键 age 最大约9ms。不能将 J5 动态归因于本次通信噪声。

证据：[baseline.json](comm_recheck_20260918/baseline.json)、[preflight01.summary.json](comm_recheck_20260918/receive/preflight01.summary.json)。

## 3. 实际参数与完整搜索过程

所有主动 run：Kp=[30,30,30,5,5,5]，Kd=[1,1,1,0.1,0.1,0.1]；velocity_command=0；q_command 恒为本轮实时锁存 q_start。FF=λ*g(q_start)，冻结 g。未改变任何 joint/gravity sign、mapping、offset、协议顺序，未乘历史 alpha/first_moment_ratio。

增益依据 Phase2 HOLD_P/HOLD_D、Phase4 BASE_P/BASE_D；不把 Phase2 Kd=10 脱困流程加入补偿搜索。选 λ=.95 的静态误差通过几何筛选，再只改变 λ 至1.0；之后只延长观察，不以增益掩盖异常。

| Attempt | λ | 计划短测 s | 首 TX 后反馈覆盖 s | 结果 | TX批数 |
| --- | ---: | ---: | --- | --- | ---: |
| run01 | 0.95 | 0.2 | 0.7006473780002125 | SOFT_FAIL | 65 |
| run02 | 1.00 | 0.2 | 0.21848345799980962 | SOFT_FAIL | 20 |
| run03 | 1.00 | 0.5 | 0.4987673340001493 | NORMAL | 46 |
| run04 | 1.00（未发送） | 1.2 | 无主动阶段 | HARD_ABORT_BEFORE_FIRST_WRITE | 0 |

Run01：0.2s短测后保留同一 q_start 最多另0.5s观察回稳，约0.70s结束。J5先动后稳，最终误差 **0.016878 rad**，略超 wrist恢复容差0.015，因此 SOFT_FAIL。没有改目标去“纠正”误差。

Run02：λ=1.0。J5最大位移 **0.014194 rad**，最终误差 **0.013810 rad**，在授权恢复容差内且尾窗稳定。因启动阶段仍有短暂 DIRECTION_MISMATCH，保留 SOFT_FAIL 标记，允许分析后继续。

Run03：同参数延长0.5s，**NORMAL**。J5最大位移 **0.004603 rad**，最终误差 **0.004220 rad**，没有方向矛盾标记。J2/J3无可分辨位移。

Run04：只计划延长1.2s，首写前几何检查失败，无主动阶段；未进行下一轮。其 planned λ/Kp/Kd 不能计入实际发送。

前3轮 q_start 分别见各 [PRE_SEND_SNAPSHOT](comm_recheck_20260918/)；JSON保存 measured q/dq、g、P/D/FF/proxy、全部wire数组、age、限位及文件hash。每轮均重新读取，不复用旧 q_hold。

## 4. 六轴动态与 guard

以下由完整原始 FE55 反馈独立重解析；位置/速度以接收顺序最新值对齐。尾漂移用末≤200ms独立位置帧 OLS；短窗值不是已建立的长期 steady drift。零值表示编码器未分辨出变化。

| Run / joint | max Δq rad | max abs dq rad/s | tail drift rad/s | max abs proxy nominal Nm | 80% hard位移界占用 | min joint margin rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| run01 / joint1 | 0.0000000 | 0.004885 | 0.000000 | 0.217331 | 0.0% | 3.975777 |
| run01 / joint2 | 0.0000000 | 0.004885 | 0.000000 | 2.072023 | 0.0% | 0.207941 |
| run01 / joint3 | 0.0000000 | 0.014652 | 0.000000 | 0.991125 | 0.0% | 0.164072 |
| run01 / joint4 | 0.0003834 | 0.134312 | -0.000483 | 0.012932 | 0.8% | 3.999042 |
| run01 / joint5 | 0.0187960 | 0.402931 | -0.001155 | 0.120989 | 91.5% | 1.970654 |
| run01 / joint6 | 0.0003843 | 0.109890 | 0.000000 | 0.010882 | 2.6% | 3.997123 |
| run02 / joint1 | 0.0000000 | 0.004885 | 0.000000 | 0.231286 | 0.0% | 3.975777 |
| run02 / joint2 | 0.0000000 | 0.004885 | 0.000000 | 2.171104 | 0.0% | 0.207941 |
| run02 / joint3 | 0.0000000 | 0.014652 | 0.000000 | 1.042464 | 0.0% | 0.164072 |
| run02 / joint4 | 0.0003834 | 0.109890 | -0.000669 | 0.015833 | 0.8% | 3.999042 |
| run02 / joint5 | 0.0141945 | 0.427349 | 0.049070 | 0.104690 | 71.8% | 1.959145 |
| run02 / joint6 | 0.0003834 | 0.109890 | -0.001176 | 0.012793 | 2.5% | 3.997123 |
| run03 / joint1 | 0.0000000 | 0.004885 | 0.000000 | 0.233598 | 0.0% | 3.975777 |
| run03 / joint2 | 0.0000000 | 0.004885 | 0.000000 | 2.171183 | 0.0% | 0.207941 |
| run03 / joint3 | 0.0000000 | 0.014652 | 0.000000 | 1.042385 | 0.0% | 0.164072 |
| run03 / joint4 | 0.0003834 | 0.158730 | -0.000375 | 0.018142 | 0.8% | 3.999042 |
| run03 / joint5 | 0.0046034 | 0.109890 | -0.000081 | 0.036049 | 23.1% | 1.955309 |
| run03 / joint6 | 0.0003834 | 0.109890 | 0.000188 | 0.013019 | 2.6% | 3.997123 |

Run01/02/03 J5 方向矛盾最长连续判定约 **20.08 / 30.08 / 0 ms**；均未达到50ms并持续扩大。单帧速度与持续运动分开判定。前两轮启动有加速，随后趋稳；不能把启动 proxy 与速度异向简单等同于符号错误。没有观察到本次窗口内增长振荡；窗口有限，不推广到长期。

Run01 J5接近约 **91.5% 的实际hard位移边界**，余量较小，已记录，不能写成“所有参数充分安全”。后两轮启动位移降低，但 q_start/板端持续状态也不同，不能把改善全部归因于λ从.95到1.0。

J2/J3 在各有效窗口内没有可分辨位移，λ=.95和1.0之间缺少可识别差异；高Kp与摩擦也可能贡献保持效果。**0x12 sign/精确Nm标度、effective_scale_estimate 仍 UNKNOWN**。没有拟合或应用历史alpha。

## 5. 几何、restore、退出与会话间变化

各plan用URDF/STL连续关节箱位移界、桌面半空间裁剪、非相邻mesh距离；约束来自作者桌面 z=0/x≤0.02 及用户J3距桌约10cm。每轮模型全包络桌面下界约 **0.053122m**，self-distance下界约 **0.00452m**；都是条件预测，不是动态实测或完整场景标定。

Run01实际 envelope约 [0.01364,0.01180,0.01180,0.06318,0.02567,0.01887] rad；80%为hard位移界。J2/J3因几何约束仍比用户绝对上限小。随q_start变化重新计算；未为了通过任意扩大。

RESTORE采用保持已经发送的 q_start 连续参考来观察是否回到容差内，不增加位置目标轨迹。Run01未达到容差；Run02/03在容差内近静止，不需要额外quintic目标运动。**这些“近q_start”并非严格回到同一个量化值**，因此各轮起点逐渐变化被完整记录。

Run03最后至Attempt04反馈之间J5有约0.02954rad差异。该变化大于第4次计划J5几何box，意味着旧plan不适用；用户补充的人工接触说明这不是一次可用于判断自主漂移的对照。原日志的首写前中止结果保留，不改写为实机通过。获知人工接触后已仅离线按最后新姿态重新做FK/几何检查，不是扩大旧box来覆盖变化；见 [manual_contact_replan.json](comm_recheck_20260918/manual_contact_replan.json) 和 [预览](comm_recheck_20260918/manual_contact_replan_preview.json)。λ=1、Kp/Kd不变、候选1.2s，仍须live重新读取q_start。此次未重开或重试，等待hard abort后的人工恢复授权。会话间应避免接触机械臂；close≠disable这一限制仍存在。

## 6. TX、版本、测试与数据文件

共 **131批、22270 B、655帧** 内核接受数据；0x10/11/12/13/14 各 **131帧**。逐帧校验长度、checksum、次序及与快照完全相同；无目标变化、reset、homing、enable=0或其它退出帧。内核接受不等同于板端ACK。Run04发送0帧。

所有实际尝试 runner SHA256：`8fded896c1dc9baaf4897f685c76196e249c1cf84b84079e3e425360e96acd12`。通信模块 `015052fd5393a6e98a0edab9405b7069e4ec9c500422e2077184d7ac48d67f0c`；策略模块 `c046050fc96471b38b8330bf2be06e5447ea7d0de652cbbb9b813ac236931c9b`。每轮plan/review绑定这些依赖和URDF。原 Gate A/bring-up代码保持未改。

**44项离线/模拟串口测试通过**：坏帧drop、孤立关键流恢复、soft/hard分级、stale、非有限、快照失败TX=0、部分write、write异常、J5加速、hash拒绝、固定packet、无退出帧等。首次dry-run和后续参数预览device open/write均0。实机参数变更前完成几何、proxy和静态可行性检查。期间一次纯分析脚本对list使用abs导致异常，改用np.abs后继续；控制代码与参数未受影响，未因此重复实机run。

- [gravity_tuning_runs.csv](gravity_tuning_runs.csv)：attempt×joint一行，未执行字段留空。
- [gravity_tuning_timeseries.csv](gravity_tuning_timeseries.csv)：合法位置/速度事件×joint，明确区分原生事件与保持值。
- [gravity_tuning_summary.json](gravity_tuning_summary.json)：完整snapshot、plan、exit、通信、瞬态、方向一致性和指标。
- [本次J5动态](gravity_tuning_plots/session_j5_motion.png)、[J2/J3与J5 P/D/FF](gravity_tuning_plots/session_j23_and_j5_proxy.png)、[会话间J5记录](gravity_tuning_plots/j5_between_sessions.png)。会话间图只画离散点，不伪造连续轨迹。

早先通信故障停止版报告已归档在 `before_comm_recheck_archive/`。旧离线候选/历史回放图保留并有明确标签，不作为本轮实机结果。

**INCONCLUSIVE — STOPPED BEFORE RUN04 FIRST WRITE; NO RETRY, NO RESTORE.**

**GRAVITY COMPENSATION NOT YET VALIDATED FOR REPEATED 1–2s HOLD. MANUAL GUIDANCE NOT EXECUTED.**
