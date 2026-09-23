# ARM6 GRAVITY / MANUAL GUIDANCE: PASS

2026-09-18，`lagrange-smc`，当前真实 gamma6、校准 arm6.urdf、joint1–joint6。
结论适用于本次测试姿态与保留安全保护的手推档。完成后已停止 guidance，
平滑切回带重力支撑的 HOLD，RViz 保持运行；HOLD 不等于电机失能。

## 1. Startup discontinuity root cause

旧接管将 q_ref 重置为实测 q，FF 从零开始，抹掉原 PD 承重力矩。此前已修复为
继承实际已发送 HOLD 命令，并通过调整 q_ref 将 PD 支撑连续转移给重力 FF。
本轮新增问题是整机断电后旧命令快照失效，不能将旧文件当作当前板端状态。

## 2. Handover and supported initialization

新增 `supported_startup.py` / `capture_supported_startup.py`、launch 的
`supported_startup_file` / `external_support_confirmed`，以及驱动启动姿态
容差参数。正常无扰接管算法、gravity、URDF、zero offset、GUIDE Kp/Kd 未改。

用户确认有可靠支撑后，先采集连续两秒静止反馈。以 nominal motor effort
中位数估计旧支撑，构造 FF=0、原 HOLD Kp/Kd 的 q_ref：
`q_ref = q + (median_effort + Kd*dq)/Kp`。
这只是支撑估计，**不是板端旧命令读回，也不证明冷启动严格无扰**。
样本须新鲜、六维、有限、静止、力矩有界；新启动实测姿态与采样相差不超过
0.002 rad。必须保持外部支撑，先建立并验证新 HOLD，不能直接自动启动 GUIDE。

首帧估计支撑为
`[-0.690598, -1.620512, 0.799999, 0.021978, 0.032234, 0.019048]` nominal Nm。
建立 HOLD 后 J2/J3 无可分辨位移、无故障。用户随后确认：
“能自行保持，手和支撑都已离开”。此后才进行三次无接触接管。

正常接管第一帧仍严格匹配驱动实际 HOLD；三秒 gravity match 同步补偿 q_ref，
随后三秒撤去 Kp，GUIDE Kp=0、Kd=`[.2,.2,.2,.05,.05,.05]`。
重力仍为 `g_arm6(q + explicit offset)`，不乘 historical first_moment_ratio。

## 3. Transition protection

HOLD/HANDOVER/RAMP/KP_WITHDRAWAL 继续使用 0.5 rad/s 立即中止。
关节原限位与 0.1 rad 余量、反馈新鲜度、心跳、NaN、力矩预算、六轴检查、
ghost 过滤、故障锁存与 fallback 均保留。

## 4. Manual-push profile

三次静置检查及人工明确就绪后才开放 MANUAL_PUSH：六轴 warning=0.8、
sustained=1.5 rad/s/120 ms、instant=2.0 rad/s。普通 GUIDE 保留
0.5 warning / 1.0 sustained 80 ms / 1.2 instant。所有计时按关节独立、单调时钟。
用户在六轴完成后要求取消所有速度限制；**未执行该部分**。后续整臂测试
使用原有有界手推档，全部保护保持有效，没有提高阈值或去掉近限位余量。

## 5. Offline validation

- Python：40 passed，包括新增受支撑初始化验证及原重力/手推测试。
- 伪串口：13 passed，包括初始估计支撑的实际 wire 命令、姿态偏移拒绝且锁存。
- C++ 功能测试：4 suites passed；构建、launch 参数及语法检查通过。
- 额外运行的全仓 CTest 中 flake8、pep257、uncrustify 格式/风格检查未通过，
  完整输出保留在 `cpp_tests.txt`。不能把结果描述为全仓所有检查全绿。
  功能测试输出另存 `cpp_functional_tests.txt`。

## 6. Three no-touch handovers after power restoration

| Trial | J2 max abs(dq), rad/s | J2 max command step, nominal Nm | J2/J3 position span |
|---|---:|---:|---:|
| 1 | 0.014652 | 0.002267 | 0 / 0 rad |
| 2 | 0.004885 | 0.0000000500 | 0 / 0 rad |
| 3 | 0.004885 | 0.0000000529 | 0 / 0 rad |

三次均进入 GUIDE 并完成至少五秒静置，无故障。第一轮全轴最大单周期指令
变化约 0.003091 Nm，来自连续 Kp 撤出；第二、三轮均小于 0.000083 Nm。
指令变化按同一当前 q/dq 评估前后命令，避免把速度量化噪声算成指令阶跃。
每轮静置 J1–J3 无可分辨位移；腕部位置跨度不超过 0.000768 rad。

## 7. Per-joint and whole-arm acceptance

| Joint | Observed excursion | Peak abs(dq) | Operator result |
|---|---:|---:|---|
| J1 | 18.95°（前一轮同一实机） | 0.777 rad/s | 容易推动、松手保持、无回位或异常 |
| J2 | 20.24° | 0.474 rad/s | 容易推动且松手保持 |
| J3 | 26.75° | 0.444 rad/s | 容易推动且松手保持 |
| J4 | 42.46° | 1.233 rad/s | 可以推动且松手保持 |
| J5 | 63.21° | 1.282 rad/s | 可以推动且松手保持 |
| J6 | 38.35° | 1.209 rad/s | 可以推动并且松手保持 |

幅度为整个单轴验收窗口的最大角度跨度，包含人工往返，不是净位移；实际
部分动作超过建议的 10–20°，但未触及限位/中止门槛。J2/J3 松手最后三秒
位置跨度为零；J4–J6 不超过 0.0003834 rad。全过程 GUIDE/手推 Kp 均为零。
没有观察到持续回位、下坠、自行加速或异常高频振荡。

J1 的独立手推证据见上一轮
[`bumpless_20260918`](../bumpless_20260918/ACCEPTANCE_REPORT.md)，此次没有声称
重新完成其 19° 单轴试验；本轮整臂联动又实际移动了 J1。

随后用户明确要求并完成整臂联动，确认“整臂容易推动且松手保持”。
J1–J6 角度跨度分别为 `[6.32,11.15,4.52,0.31,23.63,0.044]` 度，
最高速度为 J5 的 1.013 rad/s。松手后三秒 J1–J3 位置跨度为零，腕部各轴
不超过 0.0003834 rad；没有故障。不是每个关节在这次联动里都有大幅运动，
所以该记录补充而不替代前面的六轴独立测试。

## 8. HARD_ABORT

本轮受支撑初始化、三次无接触接管、J2–J6 手推和整臂联动 **均无 HARD_ABORT**。
有正常速度告警，记录于 events.jsonl；warning 未中断 MANUAL_PUSH。
前一轮推反 J2 触发限位仍保留为历史事件，不被本轮 PASS 覆盖或删除。

## 9. Fault fallback

没有为验证 fallback 故意触发新故障。超速轴 FF 清零、其他轴保留最后有界
支撑的既有实现未改；之前已有实机验证，伪串口回归仍通过。
早前整机断电/反馈失效后全 FF 清零的记录另存，不与本轮正常退出混淆。
本轮退出调用健康 operator HOLD，保持重力 FF、恢复原 HOLD 增益，并确认
`HOLD: operator stop acknowledged`；guidance 节点随后退出，没有恢复手推。

## 10. Final status and artifacts

**ARM6 GRAVITY / MANUAL GUIDANCE: PASS（本次姿态与有界手推档）。**

当前只有硬件 HOLD、robot_state_publisher、RViz 运行。本阶段到此结束。
没有启动 D435i、SLAM、基座扰动补偿、SMC 或末端稳定。

记录：`events.jsonl`、`serial_tx.strace`、`serial_tx_decoded.jsonl`、
`analysis.json`、`per_joint_results.json`、`whole_arm_result.json`、`final_hold.json`。
分析器校验了 68865 组实际串口命令的长度、checksum、有限值及第七槽为零。
图：`overview.png`、`trial1_j2_handover.png` 至 `trial3_j2_handover.png`、
`j2_push_release.png` 至 `j6_push_release.png`、`whole_arm_push_release.png`。
图中包含 q/dq、实际 wire q_ref/FF/Kp/Kd、模型重力、估计 PD/总力矩、
MotorState nominal effort 及 phase/warning 标记。估计力矩不等于实测总输出。

## 11. Git

详见 `git_diff_stat.txt`，包含之前的本地修改，不包含未跟踪文件。
没有 commit、push、reset 或 checkout；未覆盖无关本地工作。
