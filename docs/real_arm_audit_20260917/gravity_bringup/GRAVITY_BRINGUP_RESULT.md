# GRAVITY BRING-UP RESULT

离线报告更新：2026-09-18。硬件数据来自 2026-09-17 保存的两次尝试；本次续办仅完成离线解码、核查和报告，没有重开串口或发送控制。

**总体结论：INCONCLUSIVE。实际主动运行 Run02：FAIL — HARD_ABORT。**

J5 在首个 TX 时间戳后约 **62.829 ms** 的可观测窗口内持续正向位移 **0.007671356 rad（约 0.440°）**；完整协议速度帧峰值 **0.354091644 rad/s**。程序因 `sustained position-consistent velocity` 中止。J2/J3 同窗口没有可分辨的位置变化，但不能据此宣布重力补偿已验证。没有完成计划的 0.25 s 短测，没有多 λ 比较、重复稳定保持或 3 s hold。

## 1. 授权边界与当前状态

用户实验前确认已撤支撑、人员退出、无额外工具/负载、J3 距桌面约 10 cm；此前确认实体动力隔离可操作。此次授权允许范围内选取 λ 与历史增益，但 hard abort 后明确禁止自动重试、恢复、翻转符号。该边界已生效。

Run02 已停止新增 TX 并关闭串口；未恢复姿态、未 reset/homing、未发送 enable=0、未执行 Gate B/C、SMC 或手推。关闭串口不等于电机失能。**本次运行后的肉眼观察、异常声音和是否已实施实体动力隔离均 UNKNOWN，尚无用户回报。若尚未隔离，应由现场人员按已确认的方式完成动力隔离。** 不接触可能仍带力矩的机械臂。

## 2. 参数准备与变更依据

原离线方案 Kp=[6,6,6,1,1,1]、Kd=[0.3,0.3,0.3,0.03,0.03,0.03]、从 λ=0.1 开始，低 λ 无法在授权小包络内承担重力。旧版“NOT EXECUTED”报告已归档到 [pre_adaptive_offline_archive](pre_adaptive_offline_archive/)，不是本次运行结论。

本次选取固定 Kp=[30,30,30,5,5,5]、Kd=[1,1,1,0.1,0.1,0.1]。依据仓库 `phase2_torque_probe.py:51–52` HOLD_P/HOLD_D 与 `phase4_gravity_compensation.py:47–48` BASE_P/BASE_D，以及历史标定说明。这里只以作者仓库使用范围作为选参依据，不把它等同于新控制板已通过验证。

λ 选择公式：estimated_static_error=abs((1−λ)g)/Kp；要求小于 min(包络/4, RESTORE_TOL/2)，其中 RESTORE_TOL=4×0.000383377075 rad。按 0.005 网格向上取整，当前姿态得到 **λ=0.99**。这比“仅不越位移包络”更严格，目的是在正常结束时接近 q_start，并非证明 0.99 的真实标度安全。J2 模型静态残差约 0.000720095 rad；λ=0.9 虽可落在位移包络内，仍不满足此回稳容差。此前低 λ 方案被离线跳过，未逐级发送。

重力为当前 arm6.urdf 的 Pinocchio g(q_start)，冻结在本次实时 q_start；未应用历史 zero_offset、alpha=1.033、额外 first_moment_ratio，未改变 mapping/sign。六轴 q_target 恒等于新鲜锁存 q_start，velocity_command=0。slot6 命令为零。顺序保持 0x10→0x11→0x12→0x13→0x14，程序周期下限 10 ms。

计划短测 0.25 s，只有正常完成才允许最多另 0.75 s 用同一已发送 q_start 观察回稳。没有生成目标位置移动轨迹；因为参考本来就是 q_start，准备方案保持参考连续。**本次在短测内 hard abort，未进入此回稳阶段。**

## 3. 两次尝试及程序版本

| Run | 结果 | λ | 实际 TX | 首 TX 后反馈观察长度 |
| --- | --- | ---: | ---: | ---: |
| Run01 | ABORT_BEFORE_FIRST_WRITE：prewrite host delay | 0.99（仅准备） | 0 B / 0 帧 | 无主动阶段 |
| Run02 | HARD_ABORT：sustained position-consistent velocity | 0.99 | 1020 B / 30 帧 | 0.062828754 s |

Run01 SHA256：`7a7cffcd7c71050a453b8adac2987207ebc6c64cadea6dfeb1802ea3c9e82505`。

Run02 SHA256：`99ca10f23315ac7504296550979543303ee6b499c1877b8f3120ff4a626dd84f`。离线续办核对当前控制文件仍为此值，没有修改控制程序。

Run01 无 TX 后仅修改日志：保留完整持久化 snapshot，终端由大段 JSON 改为紧凑输出，增加日志耗时记录；未修改控制参数、guard、周期、协议顺序。原延迟原因没有足够时间分段记录，不能断言是终端唯一导致。变更详见 [logging_only_change.json](adaptive_20260917/logging_only_change.json)。Run02 保存快照与 fsync 耗时约 1.387 ms，终端约 0.022 ms。

两个版本分别通过 17 项离线测试；dry-run 记录串口 open/write=0。此次分析另有 7 项测试通过，包括独立 RX 解码与现有纯解码器逐帧一致、分块、尾部半帧、checksum、无 TX 与中止样本处理、变更命令拒绝。见 [offline_analysis_tests.txt](adaptive_20260917/offline_analysis_tests.txt)。没有再运行 hardware 程序。

## 4. 初始状态、命令与预写记录

Run01 与 Run02 分别重新锁存了各自反馈，不复用旧姿态。完整 q、dq、P/D/FF、age、joint margin、理论协议数组及 packet hex 分别见：

- [Run01 PRE_SEND_SNAPSHOT](adaptive_20260917/run01.jsonl.pre_send.json)
- [Run02 PRE_SEND_SNAPSHOT](adaptive_20260917/run02.jsonl.pre_send.json)

Run02 q_start：`[-0.031853675842285156, 1.3601512908935547, -1.434920310974121, 0.009781837463378906, 0.008631706237792969, 0.0017261505126953125]`。

g(q_start)：`[0.23993978668515104, -2.1602837138681865, 1.0415318600294794, 0.002200891647942701, -0.003686862052070297, 0.00011287937930031872]`（模型 Nm）。

tau_gc=0.99g：`[0.23754038881829953, -2.1386808767295045, 1.0311165414291845, 0.002178882731463274, -0.0036499934315495936, 0.00011175058550731553]`（主机期望的 nominal Nm；真实板端物理标度未验证）。

第一帧 q_command−measured_q=[0,0,0,0,0,0]；快照已先成功落盘。首帧 torque proxy：`[0.24242510866693234, -2.1337961568808717, 1.0360012612778173, 0.008283924235369524, 1.2878882903531556e-05, -0.0011093340092192471]`。q 与快照值在 float32 线上完全对应；前馈/阻尼有常规 float32 表示舍入，保存了精确 wire 数组。

## 5. 全观察窗口运动统计

角度 rad，速度 rad/s；包含最后一个触发 guard 的原始位置帧。q_end 只是最后收到的状态，**不是串口关闭后最终机械姿态**。

| Joint | q_start | q_end | max abs Δq | position p-p（含 q_start） | max abs dq | final Δq |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| joint1 | -0.031853676 | -0.031853676 | 0.000000000 | 0.000000000 | 0.004884720 | 0.000000000 |
| joint2 | 1.360151291 | 1.360151291 | 0.000000000 | 0.000000000 | 0.004884720 | 0.000000000 |
| joint3 | -1.434920311 | -1.434920311 | 0.000000000 | 0.000000000 | 0.014652252 | 0.000000000 |
| joint4 | 0.009781837 | 0.009781837 | 0.000383377 | 0.000383377 | 0.061050415 | 0.000000000 |
| joint5 | 0.008631706 | 0.016303062 | 0.007671356 | 0.007671356 | 0.354091644 | 0.007671356 |
| joint6 | 0.001726151 | 0.002109528 | 0.000383377 | 0.000383377 | 0.085468292 | 0.000383377 |

J2/J3 基于独立位置帧的全窗口 OLS 斜率均为 0 rad/s，即未分辨出位移；协议速度非零偏置/量化不等于确定运动。J5 全窗口位置斜率约 0.109976 rad/s，并在末段增大；不是只有一帧速度尖峰。J4/J6 变化为约一个位置量化单位，不能凭这次短记录判定其实际微动成因。

首 TX 后约 0.072 ms 就收到 J5 一个量化单位偏差，可能涉及采样/缓存时延，不把该第一点单独归因为命令瞬态；后续连续增长是本次保护判断的主要依据。

| 观察截止 ms | 最后完整事件 ms | J5 Δq | J5 最新完整 dq | J5 proxy | J2/J3 Δq |
| --- | ---: | ---: | ---: | ---: | --- |
| 10 | 8.947 | 0.000383377 | 0.085468 | -0.014114 | 0 / 0 |
| 20 | 18.164 | 0.000383377 | 0.061050 | -0.011672 | 0 / 0 |
| 30 | 28.129 | 0.001150131 | 0.061050 | -0.015506 | 0 / 0 |
| 50 | 48.132 | 0.003835678 | 0.231991 | -0.046027 | 0 / 0 |

100/250/500/2000 ms 无数据，不做外推。位置与速度按主机接收顺序采用最新值保持对齐，不声称具有控制板同步采样时间。完整 transient max 指标见 JSON。

## 6. Guard、几何与桌面间隙

有效 displacement envelope=`[0.0128, 0.0128, 0.0128, 0.01881662292480469, 0.01881662292480469, 0.01881662292480469]` rad。限位取 URDF/硬件 config 交集，最低保留 0.05 rad。nominal total proxy=Kp(q_start−q)−Kd·dq+tau_gc；界限 J1–3=4.5、J4–6=0.8 nominal Nm。它不是控制板实际总力矩饱和保证。

| Joint | displacement guard占用 | max abs proxy | proxy guard占用 | 相邻位置事件 proxy slew（nominal Nm/s） | 最小限位余量 rad |
| --- | ---: | ---: | ---: | ---: | ---: |
| joint1 | 0.00% | 0.242425 | 5.39% | 0.000000 | 3.968146 |
| joint2 | 0.00% | 2.143566 | 47.63% | 1.957624 | 0.209849 |
| joint3 | 0.00% | 1.045769 | 23.24% | 1.947005 | 0.135080 |
| joint4 | 2.04% | 0.010201 | 1.28% | 1.486062 | 3.990218 |
| joint5 | 40.77% | 0.077416 | 9.68% | 2.926056 | 1.983697 |
| joint6 | 2.04% | 0.008659 | 1.08% | 2.446609 | 3.997890 |

**实际触发 sustained position-consistent velocity，而非位移上限或单帧速度。** 离线以最后约 49.842 ms 位置窗重建：J5 位移 0.007287979 rad，均速 0.146220722 rad/s；本轮阈值 min(0.20, envelope/0.15)=0.125444153 rad/s，约 **116.56%**，且超过两个位置量化单位。此重建用 RX 时间；抛异常的精确 check 时间未记录，因此不伪称逐时钟完全相同。

J5 有 2 个完整速度帧超过 warning 阈值 0.293060303 rad/s；在线 STATE 中 9 个循环标为 warning，循环数不是独立速度样本数。三个 50 ms 分块的持续加速、振荡和平均 proxy slew 判据尚未获得足够主动窗口；不能报告“已排除加速/振荡”。上表相邻事件 slew 是描述性离线指标，不是那个在线分块 guard。

几何计划见 [plan_v2_logging_only.json](adaptive_20260917/plan_v2_logging_only.json)：STL 三角形与桌面半空间裁剪、关节箱内保守点运动界、非相邻 link 距离。使用作者场景桌面 z=0、x≤0.02 和现场 J3≈10 cm，再保留 1 cm 测量预留。取更保守结果，整包络桌面间隙下界 **0.053027387 m**，非相邻 mesh 间距下界 **0.005451328 m**，现场 J3 约束推得下界 **0.078669568 m**。

这些是依赖 URDF、base/table 位置、mesh 覆盖和清空场景假设的**预测界**，不是实测动态间隙；没有经过本机完整桌面坐标标定。本次 q 在该箱内，但不能将模型界作为现场几何已完全验证的结论。用户“J3 最低约10 cm”与模型另有 link 较低的差异保留，已使用较小一侧。

## 7. 通信与 TX 审计

独立 FE55 解码校验全部完整帧；对 0x20/21/22/23 用 7 float，对 0x24 用 7 uint16。Run02 统计（频率去掉开串口最初 50 ms 缓冲窗口）：

| cmd | 全记录合法帧数 | RX Hz | 最大帧间隔 ms |
| --- | ---: | ---: | ---: |
| 0x20 | 217 | 200.818 | 6.340 |
| 0x21 | 216 | 200.797 | 9.237 |
| 0x22 | 217 | 200.792 | 9.237 |
| 0x24 | 217 | 200.792 | 9.026 |

另有 0x01=21、0x23=2、0x25=2 合法帧；不推断未知 cmd 含义。checksum/malformed/nonfinite=0。已完整建立四流后，离线逐帧最大 age=9.237 ms，在线成功 guard 最大 age=4.998 ms，均低于 50 ms；记录窗口内未 stale。开头有 9 B 不完整流前缀；末尾 0x21 未收全，未被解析为有效速度或误报 malformed。关闭后的反馈不可用，不延长 stale/运动观察结论。

6 批 TX，每批 170 B，0x10/11/12/13/14 各 6 帧。独立验证各帧 checksum、长度、顺序和 float32 数组，所有批完全等于 PRE_SEND_SNAPSHOT，q_target、Kp/Kd、FF 从未改变。相邻批间隔 10.126–11.218 ms，五个间隔平均频率 **92.531 Hz**，不是严格 100 Hz。内核接受字节不等于板端确认执行。

first_TX=1.002130136 s，last_TX=1.056166185 s，last_RX=1.064958890 s，serial_close完成=1.120977298 s（均相对本轮 host origin）。即末次 TX 在首 TX 后 54.036 ms，记录观察至 62.829 ms，关闭完成至 118.847 ms；**不能把 118.847 ms 当作有效运动观测时长**。最后一批仍是同一静态 q_start、零目标速度和 λ=0.99 前馈；无退出控制帧。

只读预采集另有一次“关闭串口后才检查 freshness”的脚本断言错误；[preflight_quality.json](adaptive_20260917/preflight_quality.json) 单独检查了接收期间 gap/age，未发现通信异常。这是工具问题，未发送主动命令；保留原记录，不将其改写为现场故障。

## 8. 能回答与不能回答的问题

支持的结论：本次 J5 正向位移持续增长且与速度反馈一致，保护中止正确；J2/J3 在约 63 ms 内没有编码器可分辨位移；整批命令没有位置目标变化。

不足以回答：gravity feedforward 的 sign、真实 Nm scale、稳定 λ 区域。只执行 λ=0.99 一点，同时 holding stiffness 相对 Gate A 改变，且窗口很短，无法从 J2/J3 不动分离 PD 与重力前馈的作用。`effective_scale_estimate=UNKNOWN`，没有置信区间可据实给出。

J5 FF 仅约 −0.00365 nominal Nm；末态 P≈−0.03836、D≈−0.03541，proxy≈−0.07742，与测得正向运动相反。**这是主机模型公式结果，不证明控制板实际施加同方向/同标度力矩，也不证明 joint sign 错误。** 固件实际 0x12 语义、持久旧状态、实际驱动力/负载与模型均需进一步区分；现有数据不能指定唯一原因，更不能自行翻符号或增益再试。

UNKNOWN 保留：精确固件版本、板端总 torque saturation、0x12 实际 sign/scale、断流/关闭后状态、persistent board state；本次现场运动观察和实体隔离执行情况也未知。

## 9. 交付与下一步边界

- [gravity_bringup_summary.json](gravity_bringup_summary.json)：两次尝试、完整快照、TX/通信、六轴指标、瞬态与结论；未观测字段用 null。
- [gravity_bringup_all_runs.csv](gravity_bringup_all_runs.csv)：每 run × joint 一行，Run01 主动指标留空。
- [gravity_bringup_timeseries.csv](gravity_bringup_timeseries.csv)：每完整位置/速度事件 × joint 一行，携带最新另一流值；事件 cmd 显式标记，不把保持值当独立采样。
- [J5 中止图](gravity_bringup_plots/j5_abort.png)、[J2/J3 图](gravity_bringup_plots/j2_j3_position_velocity.png)、[λ/FF 图](gravity_bringup_plots/lambda_and_feedforward.png)、[单点 displacement vs λ](gravity_bringup_plots/displacement_vs_lambda.png)。

此次新增离线 `analyze_gravity_bringup.py` 与 7 项分析测试；没有更改 hardware controller。下一步可以继续离线审查 J5 相关语义与现有数据，但任何重新打开串口开展主动实验都须针对这次 hard abort 重新人工审阅与授权。不得将此次“继续完成任务”解释为故障后重试授权。

**GRAVITY BRING-UP STOPPED — HARD ABORT; VALIDATION INCONCLUSIVE.**

**NO RESTORE, NO AUTOMATIC RETRY, NO MANUAL GUIDANCE. WAITING FOR HUMAN REVIEW.**
