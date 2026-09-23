# ARM6 manual-guidance 实机验收（分阶段速度保护）

## 1. Current speed safety logic

原有全phase单帧0.5保护已改为分阶段、独立逐轴monotonic计时。当前配置：transition warning/abort=0.5；GUIDE warning=0.5，sustained=1.0持续80ms，instant=1.2。0.8为首次试验配置；改为1.0有用户在Run01故障后的明确授权，不是自动升阈值。fault latch不自动复位。

## 2. Code changes

见[SPEED_SAFETY_CHANGE_SUMMARY](SPEED_SAFETY_CHANGE_SUMMARY.md)。本轮只修改速度保护、诊断、配置和测试。模型、URDF、zero offset、gravity ratio、Kp/Kd结构及阻尼均未改变；硬件链路仍为单一student_arm_node、六轴gamma6。

## 3. Test results

当前参数25项相关Python通过；9项真实驱动PTY通过；4组C++通过（本次仅参数更新后由PTY再次覆盖C++读入的新阈值）；构建/launch检查通过。原0.8参数测试保留。额外全量Python653通过、2项原HEAD已存在的Vicon/SMC断言失败，在临时HEAD副本中复现，未改动它们。

## 4. Real-arm GUIDE result

Run01（sustained=0.8）：readonly100.26Hz；成功进入GUIDE，六轴Kp=0。5秒无接触静置最大净变化0.000383 rad，未见持续漂移。

Run02（sustained=1.0）：readonly100.46Hz，实时dry-run PASS；启动约0.115s的RAMP阶段中止，未进入GUIDE。

## 5. Per-joint manual push result

| Joint | Result |
|---|---|
| J1 | GUIDE内约18.43°运动范围，峰速0.601rad/s仅告警；操作者确认容易推动，松手保持。通过本轴手推观察。 |
| J2 | Run01约9.05°后持续超速中止，未完成10–20°及松手观察；操作者确认当时是正常手推。Run02启动即中止。 |
| J3 | 未完成GUIDE手推验收。 |
| J4 | 未完成GUIDE手推验收。 |
| J5 | 未完成GUIDE手推验收。 |
| J6 | 未完成GUIDE手推验收。 |

## 6. HARD_ABORT

| Run | phase/type | joint | q rad | dq rad/s | sustained duration | Kp/Kd | actual pre-fault wire FF Nm | measured effort Nm |
|---|---|---|---|---|---|---|---|---|
| 01 | GUIDE / sustained | J2 | 1.244946 | -0.971916 | 0.080033 s | 0/0.2 | -2.053959 | -1.825642 |
| 02 | RAMP_GRAVITY / transition | J2 | 1.320859 | 0.503052 | 0 s (single frame) | 30/1 | -0.070384 | -1.333334 |

Run01实际串口4955批全部核对长度/校验和/六轴映射。J2故障FF清零，其他轴保留此前有界FF（J3=1.0613Nm）。J2故障后2秒净变化+0.08316rad，故障后的运动不能算GUIDE手推结果。

Run02实际串口463批核对。故障前J2 FF仅-0.070384Nm；故障后J2 FF清零，其余保留当时渐入的FF（J3=0.034691Nm）。并非恢复到完整g(q)。启动前J2 effort=-1.48376Nm，首帧q_ref=q_measured且FF=0，提示原PD承重支撑在接管时骤减；这是启动接管不连续的证据，尚不能仅靠这一条记录证明全部动力学因果。整段J2从1.29110到1.39105rad，峰速0.57143rad/s。操作者明确确认本次启动完全无人接触，保护后没有其他异常。故障前速度不能归因于人工推动。

[Run01全程](active_overview.png) · [Run01故障前后2s](fault_window.png) · [Run01持续速度计时](sustained_speed_timer.png)

[Run02全程](run02/active_overview.png) · [Run02故障窗口](run02/fault_window.png)。Run02启动仅115ms即故障，记录中的故障前反馈不足2秒，曲线未补造缺失数据；故障后记录超过4秒。

当前保持故障锁存PD hold，RViz继续显示真实反馈；不是GUIDE，也不是电机失能。未在Run02后自动重试，未扩大transition阈值。

## 7. Final status

**NOT YET VERIFIED**。J1已有GUIDE手推通过证据；J2–J6未完成，且无人接触的启动接管支撑需调查，主动测试已停止。没有开始D435i、SLAM、SMC或末端稳定。

## 8. Git diff --stat

见[完整输出](git_diff_stat.txt)。包含本轮之前已存在的本地修改，不含未跟踪文件。未commit/push/reset/checkout。
