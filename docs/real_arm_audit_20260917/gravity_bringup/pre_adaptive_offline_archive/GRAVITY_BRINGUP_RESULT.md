# GRAVITY BRINGUP RESULT

**INCONCLUSIVE — PRE-FLIGHT ONLY; NO HARDWARE EXPERIMENT EXECUTED.**

用户已经授权有界重力补偿及正常结束后的有界回到本轮起点。本轮不是因缺少笼统执行授权停止，而是在执行前发现：低lambda阶段的慢速、近起点稳定恢复与固定低Kp/无额外承重之间存在名义力矩可行性冲突；现场完整workspace/catch信息也未建立。原Gate A未再运行，新gravity命令未发送。

## 现场与证据边界

- 用户最新确认最低点J3距桌面约10cm。记录为现场估计，不是模型测量，也不能直接套用作者DESK_PLANE_Z=0、DESK_EDGE_X=0.02等坐标。
- 全部连杆/末端到障碍距离、被动兜底间距、刚性支撑已撤出的实际状态尚未完整确认。没有基于端点路径冒充全部连杆碰撞检查。
- 上次保存q是离线参考，不是当前实时位置；没有重开串口获取新的q。下次若实验成立，必须现场重新读取/锁存。
- 该保存姿态J3限位余量0.154953175354rad。若保留先前0.15rad限位余量并将剩余空间的一半保留作不确定性预算，J3候选位移包络仅0.002476587677rad。此为离线提案，未安装/运行新guard。

## Gravity model

固定基座Pinocchio直接读取当前arm6.urdf，无历史zero_offset，无运行时first_moment_ratio，无alpha=1.033，无历史摩擦/惯量标定注入。并未导入或运行Phase4/SMC/旧hardware节点。

URDF SHA256：`60cbb80ad234f5bb5d55c53d0d77853b04ab33f8a4c1bb3d79889586201b1166`。
q_saved=[-0.031853675842285156, 1.494049072265625, -1.415846824645996, 0.0024938583374023438, 0.16706371307373047, 0.002109527587890625] rad。
g(q_saved)=[0.2583339385303244, -2.303601500310789, 0.960947922021869, 0.02943664912663042, -0.007233618207626143, 0.00010753843208119264] Nm（模型）。
假设URDF基座重力方向和负载与当前实体一致；0x12实际标度/方向未知，此处只是名义Nm计算，不是新固件力矩测量。

## 低lambda后的RESTORE可行性

参考轨迹回到q_start并稳定时，dq约0、q_ref=q_start。忽略未知静摩擦且假设0x12的名义标度成立，关节静平衡需要：
`Kp*(q_start-q) + lambda*g(q) = g(q)`。
因此在本姿态附近，所需静态位置偏差约为`|(1-lambda)*g|/Kp`。这不是动态仿真，也不是真实机械臂必然位移预测；它指出仅靠已批准项能否提供静态恢复所需力矩。未知摩擦或错误标度可能改变结果，但不能将其默认为可靠支撑。

| lambda | J2 tau_gc Nm | J2未补偿量abs Nm | J2名义静态误差rad | J3 tau_gc Nm | J3名义静态误差rad | 实机状态 |
|---:|---:|---:|---:|---:|---:|---|
| 0.10 | -0.230360150 | 2.073241350 | 0.345540225 | 0.096094792 | 0.144142188 | NOT EXECUTED |
| 0.20 | -0.460720300 | 1.842881200 | 0.307146867 | 0.192189584 | 0.128126390 | NOT EXECUTED |
| 0.35 | -0.806260525 | 1.497340975 | 0.249556829 | 0.336331773 | 0.104102692 | NOT EXECUTED |
| 0.50 | -1.151800750 | 1.151800750 | 0.191966792 | 0.480473961 | 0.080078994 | NOT EXECUTED |
| 0.75 | -1.727701125 | 0.575900375 | 0.095983396 | 0.720710942 | 0.040039497 | NOT EXECUTED |
| 1.00 | -2.303601500 | 0.000000000 | 0.000000000 | 0.960947922 | 0.000000000 | NOT EXECUTED |

J2在lambda=0.10时未补偿约2.073241Nm，最大0.03rad位置误差对应P项仅0.18Nm；静态误差约0.345540rad，远超授权包络。即使lambda=0.50，名义误差仍约0.191967rad。Kd在静止时不提供支撑力；动态阻尼也不能使低lambda在零误差静止处产生缺失重力力矩。
如果将“回到附近”候选容差定为4个已观察encoder步长=0.001533508301rad，则名义静态恢复所需lambda至少J2约0.9960、J3约0.9904。此计算不允许跳级，也不是要求直接采用这些比例。
quintic只能让reference平滑，不能保证physical q跟踪。低lambda阶段结束若有下沉，生成q_now→q_start的平滑参考并保留同一lambda，仍可能因力矩不足继续下沉/超限。仅宣布RESTORE不解决这个问题。
正常阶段还要求“无持续加速、没有sign疑点、workspace可接受”。如果第一次低lambda观察本身触发hard abort，就绝不能执行restore或再试同级。本轮没有为了得到真实abort数据而先执行一个退出/恢复可行性未解决的实验。

## 已完成的离线代码

- `src/arm-platform/tools/gravity_bringup_preflight.py`：仅保存snapshot输入；直接URDF重力；六个lambda理论数组；授权上限与限位缩小包络；名义静态恢复检查；已验证core quintic的返回参考及速度/加速度计算。没有execute入口、ROS publisher或串口接口。
- 恢复参考候选速度0.03rad/s，是作者Phase4参考0.18rad/s的1/6；候选加速度0.06rad/s²对应至少0.5s达到此速度。这只是提案，不是已验证硬件能力。T=max(0.5,1.875*|dq|/vmax,sqrt((10/sqrt(3))*|dq|/amax))，保证quintic参考端点速度/加速度0和峰值界；并不保证实体速度/加速度。
- 原`first_active_takeover.py`保持原SHA256 `cc9f022002a2232fa15340d5f55125527369947dfa41bd8cbcd9236047e695cd`，未扩大原程序权限或放宽其guard。
- 完成16项离线测试：纯URDF语义/固定lambda/符号数组、非法输入、包络和近限位拒绝、低lambda静态恢复不足、quintic端点/范围/速度/加速度、任一恢复前提失败不得恢复。

## 尚未实施的live设计

未声称已完成可上机的自动分级控制器。现场几何、低lambda正常退出/恢复策略未解决前，不实现一个会被误认为可安全执行的live入口。单帧velocity spike应Warning而非hard abort、持续速度/位置/加速度/发散的联合判据、总proxy及slew边界等仍需在可行实验结构上完成并冻结，不能沿用0.0015rad Gate A位移门槛充当新包络。
若改动lambda序列、增益、目标策略、恢复/abort语义、包络或力矩限制，属于用户第13节规定的控制逻辑变更，必须先停下审阅；本轮未自行采取任何这些变更。
可讨论的后续方向是另行确定低lambda阶段的独立安全终止/恢复办法，或不同的已审阅验证结构。不能自动使用尚未验证的高lambda恢复，也不能依靠close(serial)使机械臂停住。

## 按lambda的实际实验结果

所有lambda均NOT EXECUTED：q_start_live/q_end/Δq/dq/sag_rate/guard utilization均null；执行时长0，TX=0。effective_scale_estimate=UNKNOWN；sign=UNKNOWN。未发生hardware abort，也未进行RESTORE或3秒hold。没有用离线模型值冒充实测值。
`gravity_bringup_timeseries.csv`仅表头，因为没有真实实验样本。`gravity_bringup_plots/`仅有明确标注OFFLINE MODEL ONLY的恢复可行性图；没有伪造position/velocity/lambda实测曲线。

## 交付与结论

- [完整离线结果JSON](gravity_bringup_summary.json)
- [实测时序CSV（无样本）](gravity_bringup_timeseries.csv)
- [离线恢复可行性图](gravity_bringup_plots/offline_restore_feasibility.png)
- [测试结果](tests.txt)
- [离线预览完整输出](offline_preview.txt)

**INCONCLUSIVE — bring-up尚未开始。没有机械臂控制命令、没有serial open/write；没有声称实验通过。** 需要先解决低lambda恢复策略与完整现场workspace条件；现有授权不是这些物理可行性事实的替代。

GRAVITY COMPENSATION BRING-UP PREPARATION STOPPED — NO HARDWARE EXECUTION
WAITING FOR HUMAN REVIEW OF LOW-LAMBDA RESTORE STRATEGY; NO MANUAL GUIDANCE AUTHORIZATION
