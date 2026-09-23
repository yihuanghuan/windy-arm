"""Generate offline tuning report from saved evidence. No hardware access."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[3]
BASE=ROOT/'docs/real_arm_audit_20260917/gravity_tuning'
SESSION=BASE/'session_20260918'


def read(name):return json.loads((SESSION/name).read_text())


def main():
    fault=read('communication_fault.json');geom=read('offline_geometry_candidates.json')
    replay=read('prior_j5_policy_replay.json');state=read('read_only_state.json')
    policy=ROOT/'src/arm-platform/tools/gravity_tuning_policy.py'
    prior_control=ROOT/'src/arm-platform/tools/bounded_gravity_bringup.py'
    summary=dict(status='INCONCLUSIVE',session='20260918',active_tuning_runs=0,
        read_only_attempts=1,attempt_result='HARD_ABORT_BEFORE_CONTROL',
        reason='FE55 0x22 checksum error; independent decoder confirms',
        hardware_session=state,communication_fault=fault,
        tx_frames={hex(c):0 for c in range(0x10,0x15)},
        no_reopen_no_retry_after_fault=True,restore_result='NOT_ATTEMPTED',
        motor_disabled='UNKNOWN; closing UART is not disabling power',
        physical_power_isolation_performed='UNKNOWN',
        offline_geometry=geom,prior_replay=replay,
        policy_sha256=hashlib.sha256(policy.read_bytes()).hexdigest(),
        prior_controller_sha256=hashlib.sha256(prior_control.read_bytes()).hexdigest(),
        policy_is_offline_only=True,policy_tests=19,policy_tests_passed=19,
        final_validated_parameters=None,gravity_sign='UNKNOWN',effective_torque_scale='UNKNOWN',
        dry_run_serial_open=0,dry_run_serial_write=0,
        dry_run_evidence='policy imports only numpy/collections; synthetic and saved-data replay, no live entry point',
        future_live_readiness=False)
    (BASE/'gravity_tuning_summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    fields=['run_id','phase','result','lambda','Kp','Kd','q_start','q_end','duration',
        'max_displacement','velocity','drift','oscillation','gravity_torque','P','D','FF',
        'direction_consistency','desk_margin','joint_limit_margin','restore_result',
        'serial_tx_frames','checksum_errors','reason']
    with (BASE/'gravity_tuning_runs.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
        w.writerow(dict(run_id='preflight01',phase='READ_ONLY_NOT_ACTIVE_RUN',
            result='HARD_ABORT',restore_result='NOT_ATTEMPTED',serial_tx_frames=0,
            checksum_errors=1,reason='0x22 checksum mismatch before control qualification'))
    plots=BASE/'gravity_tuning_plots';plots.mkdir(exist_ok=True)
    rr=replay['rows'];t=np.array([r['t']*1000 for r in rr])
    fig,ax=plt.subplots(3,1,figsize=(10,8),sharex=True)
    ax[0].plot(t,[r['delta'][4]*1000 for r in rr],'.-');ax[0].set(ylabel='J5 displacement [mrad]',title='HISTORICAL Run02 replay — not a new physical tuning run')
    ax[1].plot(t,[r['dq'][4] for r in rr],'.-',label='Protocol velocity');ax[1].set(ylabel='Velocity [rad/s]');ax[1].legend()
    ax[2].plot(t,[r['proxy'][4] for r in rr],label='Nominal proxy')
    ax[2].plot(t,[r['nominal_proxy_minus_gravity'][4] for r in rr],label='Proxy - model g',ls='--')
    mismatch=[i for i,r in enumerate(rr) if r['direction_mismatch'][4]]
    ax[2].scatter(t[mismatch],[rr[i]['proxy'][4] for i in mismatch],color='red',label='DIRECTION_MISMATCH')
    ax[2].set(xlabel='Host time since historical first TX [ms]',ylabel='Nominal Nm');ax[2].legend()
    for a in ax:a.grid()
    fig.tight_layout();fig.savefig(plots/'historical_j5_direction_replay.png',dpi=160);plt.close(fig)
    cand=geom['candidates'];fig,ax=plt.subplots(figsize=(9,5))
    for j in (1,2):
        ax.plot([c['lambda_value'] for c in cand],[c['static_error'][j] for c in cand],'.-',label=f'J{j+1} static estimate')
        ax.axhline(.4*geom['envelope_preview'][j],ls='--',label=f'J{j+1} screening limit')
    ax.set(xlabel='lambda',ylabel='Estimated static error [rad]',title='OFFLINE candidates only — no new lambda was sent');ax.grid();ax.legend()
    fig.tight_layout();fig.savefig(plots/'offline_lambda_screen.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,5));x=np.arange(6)
    ax.bar(x-.2,[.04,.05,.05,.08,.08,.08],.4,label='User absolute maxima')
    ax.bar(x+.2,geom['envelope_preview'],.4,label='Offline geometry preview (unqualified pose)')
    ax.set(xticks=x,xticklabels=[f'J{i}' for i in range(1,7)],ylabel='Envelope [rad]',title='Geometry still constrains the authorized maxima');ax.legend()
    fig.tight_layout();fig.savefig(plots/'offline_envelope.png',dpi=160);plt.close(fig)
    (plots/'README.md').write_text('No active run occurred in this session. Historical replay and offline predictions are labeled explicitly. There is no current-run motion/drift plot because qualification stopped at the first received chunk.\n')
    candidates='\n'.join(f"| {c['lambda_value']:.2f} | {c['static_error'][1]:.6f} | {c['static_error'][2]:.6f} | {'候选，未验证' if c['feasible'] else 'SKIP'} |" for c in cand)
    envtable='\n'.join(f"| J{j+1} | {maxenv:.3f} | {geom['envelope_preview'][j]:.6f} | {geom['hard_abort_at_80_percent'][j]:.6f} |" for j,maxenv in enumerate([.04,.05,.05,.08,.08,.08]))
    text=f'''# GRAVITY TUNING RESULT

日期：2026-09-18。**最终状态 INCONCLUSIVE。主动 tuning run 数=0；只读预检尝试=1，结果 HARD_ABORT_BEFORE_CONTROL。**

本轮已接受新的 bounded tuning 授权，包括允许受限自然运动和范围内自主选参。但第一次纯读取就出现真实 FE55 checksum 错误，因此在第一条控制命令之前关闭串口；没有忽略错误、重开、清缓存后重试或开展主动参数搜索。这遵守本轮第 3/F、14 条通信异常 hard abort 后停止的要求，而非沿用旧的极窄位移判据。

## 1. 新收到的通信故障证据

通过已经审计的 Receiver 以 **O_RDONLY** 打开 `/dev/ttyUSB0`，921600/raw/no echo。只执行接收，未建立写命令路径。原计划约 1.4 s 的反馈资格检查，在第一个 **241 B** RX chunk 就中止；该 chunk 内完整有效 position 仅两帧，不能给出可信的实时频率、稳定性或当前动态范围。

原始记录：[preflight_rx.jsonl](session_20260918/preflight_rx.jsonl)。错误详情：[communication_fault.json](session_20260918/communication_fault.json)。

| 项目 | 证据 |
| --- | --- |
| 出错 cmd | 0x22，current/effort feedback |
| header 在 chunk 的偏移 | 70 B |
| 声明 payload length | 28 B |
| 应有总帧长 | 34 B |
| 下一条有效 FE55 header 的偏移 | 103 B，即间隔 33 B |
| 按声明长度提取的末两字节 | AD FE |
| 独立算得 sum/add | 11 AD |
| checksum error | 1 |
| bad length / nonfinite | 0 / 0 |
| 0x10–0x14 TX | 全部 0 |

候选错误帧原始字节：

```text
{fault['candidate_frame_hex']}
```

预期的 `11` 校验字节未出现在该边界，下一帧的 FE 被声明长度覆盖。它与一个 checksum 字节缺失的形式吻合，但不能确定丢失发生于控制板、UART/USB、开口状态切换或其它路径。现有只读解码器与独立解码器都报同一错误，不能通过放宽解析规则将该帧视为有效。正常开流的两字节前缀、末尾不完整帧与此中间错误分开记录，不混作 checksum 故障。

本轮未执行 lambda/Kp/Kd，也没有 q_start 锁存用于控制、PRE_SEND_SNAPSHOT、restore 或补偿效果统计。不能把下面离线计算写成实机成果。

## 2. 历史 Kd 与参数边界核查

`phase4_gravity_compensation.py:47–56`：BASE_P=[30,30,30,5,5,5]，BASE_D=[1,1,1,0.1,0.1,0.1]；L0–L4 的 kd_frac=[1,0.5,0.3,0.2,0.2]。

因此选用的常规补偿历史代码包络为 J1–3 Kd **0.2–1.0**，J4–6 **0.02–0.1**。`phase2_torque_probe.py:51–52` 的 HOLD_D 与上界相同。其 RAMP_KD/ESCAPE_KD=10 属于另一种斜坡/脱困流程，明确排除。motor_config 的默认 Kd=1.5/1 也不作为本次常规补偿真机已验证上界来扩大搜索。

作者交接文档第 8 节描述了 Phase4 L0–L4 的实机验收；仓库缺少每个 level 对应的完整原始运行数据，故这里区分“历史实现/交接记载”与“本机固件实证”。新硬件上的阻尼语义仍未验证。本次没有实际改变 Kd。

Kp 严格限制为 J1–3 6–30、J4–6 1–5；λ 限制为 0.4–1.0，相邻候选差≤0.15。保留符号、mapping、offset=0、协议顺序，不使用历史 alpha/重复 ratio。

## 3. 几何与静态候选：仅离线预览

预检虽未合格，两个合法 position 帧仍可用于标明不确定性的离线 FK 预览。最后一帧六轴 signed q 为：

`{geom['q_preview']}`

g(q) 预览：`{geom['g_preview']}` nominal Nm。

沿用作者桌面 z=0、x≤0.02、URDF/STL 和现场约 J3=10 cm 约束；使用完整关节箱的点位移界和非相邻 mesh 距离界。当前场景未完成精确桌面坐标标定，因此**不是可直接执行的 plan**。先缩小全箱，再逐轴寻找较大的几何可行箱；控制实际工作区域仍应小于 80% 边界。

| Joint | 用户最大授权 envelope | 几何候选 envelope | 80% hard 边界 |
| --- | ---: | ---: | ---: |
{envtable}

候选全箱桌面下界 {geom['certificate']['minimum_desk_clearance']:.9f} m，自碰撞距离下界 {geom['certificate']['self_distance_lower']:.9f} m。它们接近筛选边界，只是有模型假设的搜索结果，不能作为额外实体间隙。实际重新规划应优先保留更小包络与更多余量。不能因为用户授权上限较大，就绕过几何约束放大 J2/J3。

离线暂以 Kp=[30,30,30,5,5,5] 筛选残差；筛选线为 `abs((1−λ)g)/Kp <= 0.4*envelope`，在 80% hard 位移界之前保留一半余量。没有用成本函数覆盖安全约束。

| λ | J2 预测静态误差 rad | J3 预测静态误差 rad | 筛选 |
| --- | ---: | ---: | --- |
{candidates}

0.95/0.99/1.0 只是此假设下的候选，**没有任一个被本轮发送或验证**。更低 Kp 的可行性须重新计算，不能靠预期效果直接选择。完整离线依据见 [offline_geometry_candidates.json](session_20260918/offline_geometry_candidates.json)。

## 4. 新动态策略与旧 J5 数据回放

新增 `src/arm-platform/tools/gravity_tuning_policy.py`，**仅离线策略，没有 serial/ROS/执行入口**。SHA256：`{summary['policy_sha256']}`。它不替换原实机程序；原 bring-up 文件 SHA 保持 `{summary['prior_controller_sha256']}`。

提议逻辑：

- 80% 实际几何包络为 hard 位移界；不再把 0.005/0.01/0.02 rad 本身设为 hard 条件。
- J1–3/J4–6 raw velocity ceiling=0.6/0.8 rad/s，要求≥35 ms 同向超限，且位置斜率同向、超过相应 ceiling 的一半、位移超过两量化单位。单帧 spike 只警告；nominal total proxy 越界仍保护。
- 以三段 30 ms 位置趋势检测持续加速或增长振荡。使用位置量化推导的噪声界，不以单个速度差分判断加速。
- J5 及其它轴记录 P/D/FF/proxy；两个 20 ms 位置斜率估计加速度，计算 proxy 与加速度符号得分。proxy、速度和加速度持续矛盾、增长且超过 50 ms 时 hard abort，绝不以“先调下一组再确认”作为忽略已危险趋势的理由。
- 同时记录 nominal `proxy−g`，因为 proxy 本身不是包括外载和重力后的净力矩。符号矛盾是诊断警报，不等于证明方向配置错误。
- 完整测试后安全但未稳为 SOFT_FAIL；观察不足也不是 PASS。硬通信、geometry、限位、非有限值异常不降级为 SOFT_FAIL。

对旧 Run02 的完整 62.829 ms 数据离线回放：新的点级 guard 在已记录窗口内未触发 hard abort，运行分类为 **SOFT_FAIL（观察不足）**；J5 末段 direction_consistency_score=−1，标记 DIRECTION_MISMATCH，连续可判断时间约 19.872 ms，尚不到 50 ms。旧数据既不能证明可以继续，也不能预测 63 ms 后会否触发新保护；没有把历史实验重新标为成功。

回放详情：[prior_j5_policy_replay.json](session_20260918/prior_j5_policy_replay.json)。P5/D5/FF5 末值约 −0.038357/−0.035409/−0.003650，proxy≈−0.077416，位置正向增长。这一不确定性保留，不以 envelope 增大解释掉。

**19 项离线测试通过**，覆盖参数范围、通信/几何/stale、非有限值、80% 边界、允许慢速漂移、单帧 spike、持续超速、加速保护、观察不足分类及静态筛选。策略模块只导入 numpy/collections；测试确认无 transport/IO 导入，回放 serial open/write=0。见 [policy_tests.txt](session_20260918/policy_tests.txt)。

这是策略准备，不是完整在线安全 runner：尚需完整 TX 前屏障、每循环反馈新鲜度检查、状态机、参数锁存、snapshot 持久化、正常/soft restore 和自动搜索调度的集成测试。通信 hard abort 后未继续构建可直接运行的主动入口，也未绕过该停止条件。

## 5. 后续搜索结构（未执行）

先确保通信资格稳定；每 run 锁存新鲜 q_start 并重算几何与 g。每次只改变 λ，步长≤0.15，静态不可行点跳过；其次只调整历史范围内 Kd，再考虑 Kp。新点 0.15–0.3 s，正常后 0.5 s，稳定后 1–2 s；至少两次稳定复现后才可能 PASS。最多 8–12 run。

J2/J3 分别以尾窗位置斜率和位移判断 sag/drift；短窗不足时 steady drift=null。成本可用“按包络/速度阈值归一化后的 J2/J3 drift、总位移、速度 RMS、振荡和 mismatch”逐项比较，安全筛选先于排序；不使用黑盒随机搜索。当前没有实机候选效果，未虚构排名或最终最优参数。

NORMAL/SOFT_FAIL 后 restore 只能使用已真实通过的 λ/Kp/Kd 回本轮 q_start，quintic 的时间由位移与已确认速度/加速度界推导，误差允许 J1–3≤0.01、J4–6≤0.015 rad 且近静止。尚无新验证参数，不生成可执行 restore。HARD_ABORT 永远不 restore。

## 6. 实验汇总与退出

| run_id | 阶段 | 结果 | λ/Kp/Kd | TX | restore |
| --- | --- | --- | --- | ---: | --- |
| preflight01 | 只读，尚未构成主动 run | HARD_ABORT：checksum | 未应用 | 0 | 未尝试 |

当前 CSV 将未测 q_start/q_end/duration/drift/direction 等主动指标留空；JSON 用 null/UNKNOWN，而非补零冒充成功。两帧 q_preview 不用于任何控制目标。

串口已关闭，没有自动恢复、reset/homing、重开或重试；当前 MotorControl 发布/硬件 controller 启动次数均为 0。只读 open 不等于硬件 TX 线物理隔离，close 也不等于失能。板端持久状态、确切固件、真实 0x12 scale/sign、断流后状态以及现场是否已实施动力隔离仍 UNKNOWN；现场人员负责既定实体安全边界。

交付：[gravity_tuning_runs.csv](gravity_tuning_runs.csv)、[gravity_tuning_summary.json](gravity_tuning_summary.json)、[图表](gravity_tuning_plots/)。图表分别为明确标注的历史 J5 回放和离线 λ/几何预测；本轮没有主动时间序列，不能画出不存在的运动响应。

**INCONCLUSIVE — COMMUNICATION HARD ABORT BEFORE CONTROL.**

**NO AUTOMATIC RETRY / NO RESTORE / MANUAL GUIDANCE NOT VALIDATED.**
'''
    (BASE/'GRAVITY_TUNING_RESULT.md').write_text(text)
    print(json.dumps({k:summary[k] for k in ('status','active_tuning_runs','attempt_result','tx_frames','policy_tests_passed','future_live_readiness')},indent=2))


if __name__=='__main__':main()
