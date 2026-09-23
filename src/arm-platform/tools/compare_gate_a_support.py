"""Offline support/no-support comparison. Never imports a hardware execution entry."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil


def load(directory):
    s=json.loads((directory/'gate_a_summary.json').read_text())
    with (directory/'gate_a_timeseries.csv').open() as f:r=list(csv.DictReader(f))
    return s,r


def state_at(summary,rows,joint,t):
    # No extrapolation after stream termination; timestamps reflect host receive.
    if t>summary['post_command_observation_s']+1e-12:return None
    selected=[r for r in rows if int(r['joint'])==joint and 0<=float(r['t_from_first_tx_schedule'])<=t]
    if not selected:return None
    r=selected[-1]
    event=float(r['t_from_first_tx_schedule'])
    return dict(delta_q=float(r['q_minus_q_hold']),dq=float(r['dq']),torque_proxy=float(r['total_proxy']),latest_event_ms=event*1000,position_age_at_cut_ms=(t-event+float(r['position_feedback_age']))*1000,velocity_age_at_cut_ms=(t-event+float(r['velocity_feedback_age']))*1000)


def compare(base):
    exp=base/'takeover/external_support_20260917'
    old,oldrows=load(exp/'no_support_previous_artifacts');new,newrows=load(exp/'run1_analysis')
    times=(10,20,30,50,100,500,2000)
    table={str(ms):{str(j):{'no_support':state_at(old,oldrows,j,ms/1000),'support':state_at(new,newrows,j,ms/1000)} for j in (2,3)} for ms in times}
    tmatch=old['post_command_observation_s']
    match={str(j):{'no_support':state_at(old,oldrows,j,tmatch),'support':state_at(new,newrows,j,tmatch)} for j in (2,3)}
    ratios={}
    for label,data in [('30ms',table['30']),('previous_abort_time',match)]:
        a=data['2']['no_support']['delta_q'];b=data['2']['support']['delta_q']
        ratios[label]=dict(support_over_no_support=abs(b/a) if a else None,reduction_percent=(1-abs(b/a))*100 if a else None)
    delta=[n-o for n,o in zip(new['q_hold'],old['q_hold'])]
    trigger=new['displacement_guard_trigger'];trigger['max_utilization']=max(map(abs,trigger['delta']))/trigger['threshold']
    new['onsite_observation']='UNKNOWN — no user report for this supported run received yet'
    onsite=exp/'run1.onsite.json'
    if onsite.exists():new.update(json.loads(onsite.read_text()))
    info=dict(verdict='FAIL',scope='External-support Gate A stopped after Run 1 guard abort; no retries',hardware_program_sha256=new['program_sha256'],control_program_identical=old['program_sha256']==new['program_sha256'],runs={'no_support_previous_run1':old,'external_support_run1':new,'external_support_run2':{'state':'NOT EXECUTED','reason':'Run 1 guard abort'},'external_support_run3':{'state':'NOT EXECUTED','reason':'Run 1 guard abort; approved program remains fixed at 2s, no 5s change made'}},SUPPORT_A_B_COMPARISON=dict(timepoint_method='last received q/dq at or before requested host-relative cutoff, no forward extrapolation; null means observation terminated',timepoints_ms=table,previous_abort_time_ms=tmatch*1000,at_previous_abort_time=match,joint2_displacement_ratios=ratios,initial_q_difference_support_minus_no_support=delta,causal_conclusion='J2 short-transient displacement and velocity reduced, consistent with support helping load bearing. Neither run completed 2s; supported run aborted on J5. Different initial q and unknown persistent board state confound strict support-only causality; does not prove cause or validate command directions.'),active_code_modified=False,hardware_retry_count=0)
    (base/'gate_a_summary.json').write_text(json.dumps(info,indent=2)+'\n')
    fields=['condition']+list(oldrows[0].keys())
    with (base/'gate_a_timeseries.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
        for condition,rows in [('no_support_previous',oldrows),('external_support',newrows)]:
            for row in rows:w.writerow(dict(condition=condition,**row))
    plots=base/'gate_a_plots';plots.mkdir(exist_ok=True)
    for p in (exp/'run1_analysis/gate_a_plots').glob('*.png'):shutil.copy2(p,plots/p.name)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,2,figsize=(12,9),sharex=True)
    for col,j in enumerate((2,3)):
        for ri,(key,ylabel) in enumerate((('q_minus_q_hold','Displacement (rad)'),('dq','Velocity (rad/s)'),('total_proxy','New-command proxy (nominal Nm)'))):
            ax=axes[ri,col]
            for label,rows,color in [('No support',oldrows,'tab:orange'),('External support',newrows,'tab:blue')]:
                selected=[r for r in rows if int(r['joint'])==j and -.015<=float(r['t_from_first_tx_schedule'])]
                ax.step([float(r['t_from_first_tx_schedule'])*1000 for r in selected],[float(r[key]) for r in selected],where='post',label=label,color=color)
                ax.plot(float(selected[-1]['t_from_first_tx_schedule'])*1000,float(selected[-1][key]),'x',color=color)
            ax.axvline(0,color='black',ls='--',alpha=.5);ax.set_title(f'Joint {j}');ax.set_ylabel(ylabel);ax.grid(alpha=.3)
    for ax in axes[-1]:ax.set_xlabel('ms from first TX scheduling timestamp')
    axes[0,0].legend();fig.suptitle('Support comparison: both runs aborted; initial poses differ; no data beyond endpoints')
    fig.tight_layout();fig.savefig(plots/'SUPPORT_A_B_COMPARISON.png',dpi=150);plt.close(fig)
    def fmt(x):return f'{x:.9f}'
    lines=['# GATE A RESULT — WITH EXTERNAL SUPPORT','', '**FAIL — Run 1 guard abort；Run 2/3未执行。**','',
      '用户纠正了上一轮现场前提：此前并无独立机械承重，主要依靠机械臂自身保持姿态。本轮用户明确确认已有稳定外部支撑承托远端/前臂。控制程序与上轮逐字节一致，SHA256：`'+new['program_sha256']+'`。本轮未改增益、目标逻辑、guard、频率、超时、mapping或发送顺序。','',
      '## 执行与首帧','',
      f"有支撑Run 1计划2秒；首次TX调度后约{new['post_command_observation_s']*1000:.3f}ms，joint5触发50ms连续位移guard，立即停止新增发送并关闭串口。没有自动重试。",
      'q_hold（本轮现场重新测量）='+str(new['q_hold'])+' rad。',
      'command q=measured q=q_hold，六轴初始差值均为0；velocity command=current/FF=[0,0,0,0,0,0]；Kp=[6,6,6,1,1,1]，Kd=[0.3,0.3,0.3,0.03,0.03,0.03]。',
      '首帧P=FF=0；D=total proxy='+str(new['snapshot']['torque_proxy']['total'])+'。最大proxy=0.001465415955，最大proxy guard占比7.3621%；首帧最大feedback age=3.431287ms，J3 margin=0.212174588013rad。',
      '[PRE_SEND_SNAPSHOT](takeover/external_support_20260917/run1.jsonl.pre_send.json)已在第一次write之前持久化。资格窗口通过，六轴四类必需反馈有效且无NaN/Inf/校验/长度/stale错误。','',
      '## SUPPORT_A_B_COMPARISON','',
      '**核心回答：J2在已观察到的约30ms短时范围内位移显著减小，但未消失；本轮因J5 guard中止，不能声称有支撑时可稳定完成2秒。**',
      f"在同一30ms截止时间，J2 support/no-support位移比={ratios['30ms']['support_over_no_support']:.6f}，减小{ratios['30ms']['reduction_percent']:.2f}%。在上次abort时间{tmatch*1000:.6f}ms截止，比例={ratios['previous_abort_time']['support_over_no_support']:.6f}，减小{ratios['previous_abort_time']['reduction_percent']:.2f}%。两种比例都使用各自最后一个已收到position样本，不插值，不把不同终止时刻直接当成同一时刻。",
      '这支持外部承重对J2短时表现有帮助的解释；没有足够证据认定上次失败主要原因已被确定，更不能证明position/Kp/Kd方向正确或错误。','',
      '**严格A/B的限制：初始姿态并不相同。** 虽未改任何控制参数且每次都按授权实时锁存q_hold，以下姿态变化改变了重力/接触条件。该差异是在接管前反馈中发现，不是本程序施加的位置偏移；未执行恢复姿态动作。未知persistent board state同样未被控制。','',
      '| Joint | 无支撑q_hold | 有支撑q_hold | 差值rad | 差值deg |','|---|---:|---:|---:|---:|']
    for j in range(6):lines.append(f'| J{j+1} | {fmt(old["q_hold"][j])} | {fmt(new["q_hold"][j])} | {fmt(delta[j])} | {math.degrees(delta[j]):.6f} |')
    lines+=['','### 同一时间尺度的J2/J3','',
      '时间基准=每次首笔TX记录的主机调度时间，包含写前工作，不是实测板端命令生效时间。取截止时刻之前最新已接收q/dq（零阶保持），其接收年龄见JSON。50/100/500/2000ms两组均已停止观测，必须记为N/A，不外推为稳定值。','',
      '| cutoff ms | joint | 无支撑 Δq / dq / proxy | 有支撑 Δq / dq / proxy |','|---:|---|---|---|']
    for ms in times:
        for j in (2,3):
            a=table[str(ms)][str(j)]['no_support'];b=table[str(ms)][str(j)]['support']
            def entry(z):return 'N/A（已abort）' if z is None else ' / '.join(fmt(z[k]) for k in ('delta_q','dq','torque_proxy'))
            lines.append(f'| {ms} | J{j} | {entry(a)} | {entry(b)} |')
    lines+=['','单位依次rad、rad/s、名义Nm proxy。J3两组已观测position都未偏离各自q_hold；这不是更长时间静止保证。','',
      '### Whole observed window对比（终止时间不同）','',
      '| Joint/condition | max abs Δq rad | peak-to-peak rad | final Δq rad | max abs dq rad/s | max abs proxy | position guard占比 | velocity guard占比 | proxy guard占比 |','|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for j in (1,2,4):
        for label,ss in [('no support',old),('support',new)]:
            r=ss['whole_run'][j];lines.append(f'| J{j+1} {label} | '+ ' | '.join(fmt(r[k]) for k in ('max_displacement','peak_to_peak','final_displacement','max_velocity','max_torque_proxy'))+f" | {r['position_guard_utilization']:.2%} | {r['velocity_guard_utilization']:.2%} | {r['proxy_guard_utilization']:.2%} |")
    lines+=['','### 本轮六轴','',
      '| Joint | max abs Δq | max abs dq | max abs proxy | minimum joint-limit margin |','|---|---:|---:|---:|---:|']
    for r in new['whole_run']:lines.append('| '+r['joint']+' | '+' | '.join(fmt(r[k]) for k in ('max_displacement','max_velocity','max_torque_proxy','minimum_joint_limit_margin'))+' |')
    lines+=['','### Guard与transient','',
      f"本轮joint5在约{trigger['window_span']*1000:.6f}ms窗口变化{trigger['delta'][4]:.12f}rad，原阈值{trigger['threshold']:.12f}rad，使用率{trigger['max_utilization']:.6%}。超出量仅{trigger['delta'][4]-trigger['threshold']:.12f}rad，但原规则已触发；不能事后放宽或以量化解释为由重试。",
      'J5 position在约16.5/26.5/31.6/36.5ms分别相对锁存值增加约0.000384/0.000768/0.001151/0.001534rad，呈数帧同向变化。不能仅凭极小越界量判定为数值误报，也不能在无现场反馈时断言真实J5运动原因。',
      '本轮J2在约21.8ms变化0.000381470rad，此后至36.5ms未继续增大；J3未见可分辨位置变化。已观察片段没有建立持续振荡证据，但时长太短，无法排除未来漂移/振荡或证明稳定。',
      '首20ms覆盖完整；50/100ms窗口均截断。两组都不足50ms主动观测，已审阅相邻50ms均值torque-proxy slew无法计算（null）；raw event有限差分不是同一指标，详见summary，不能混用。','',
      '## 通信、TX、退出与现场','',
      f"本轮0x10–0x14各{new['tx_counts']['0x10']}帧，共20帧、{new['accepted_bytes']}字节、4批，batch逐字节相同。顺序仍为0x10→0x11→0x12→0x13→0x14；间隔推算约{new['tx_batch_interval_rate_hz']:.3f}Hz（仅4批，不代表2秒稳态）。",
      '主机kernel accepted不是板端ACK。最后请求仍为本次q_hold、固定PD、零速度/FF，最后完整帧0x14；没有补发失能、reset或恢复姿态。',
      'checksum=0、malformed=0、nonfinite=0，四类必需反馈stale=0。反馈频率（包含qualify）：'+', '.join(f'{k}={v:.3f}Hz' for k,v in new['feedback_rates_hz'].items())+'。',
      f"post-TX解码事件处最大feedback age={new['max_feedback_age_at_decoded_events']*1000:.6f}ms；这是可重建事件的最大值，不是所有主循环时刻的完整观测。",
      '正常2秒：未完成。2/2/5秒重复：未进行，Run1失败即停。原程序仍固定2秒，本轮未修改duration来准备Run3。',
      '程序ABORT EXIT，close在EXIT日志和进程结束前完成；原程序没有精确close时间戳，不能编造。fuser复查未见端口占用者。close(serial)不代表motor disabled，已明确提示现场动力隔离。',
      '本轮现场观察：'+new['onsite_observation']+'。动力隔离完成确认：'+new['power_isolation_confirmation']+'。',
      '上次现场观察为J2轻微向下、无其他异常；不能把上次观察用于本次。Firmware version、board total torque limit、0x12 scale、disconnect behavior、persistent state仍UNKNOWN。','',
      '## 交付','',
      '- [完整JSON，包括两组数据及SUPPORT_A_B_COMPARISON](gate_a_summary.json)',
      '- [带condition列的合并timeseries CSV](gate_a_timeseries.csv)',
      '- [A/B对比图](gate_a_plots/SUPPORT_A_B_COMPARISON.png)、[本轮六轴位移](gate_a_plots/displacement.png)、[速度](gate_a_plots/dq.png)、[proxy](gate_a_plots/proxy.png)',
      '- [本轮原始RX/TX日志](takeover/external_support_20260917/run1.jsonl)、[review记录](takeover/external_support_20260917/run1.review.json)',
      '- [上轮归档报告](takeover/external_support_20260917/no_support_previous_artifacts/GATE_A_RESULT.md)',
      '- [离线分析验证](takeover/external_support_20260917/comparison_validation.json)','',
      '**FAIL**：本轮J2短时改善，但J5触发原guard，未完成静止接管。停止后续硬件实验，不修改控制参数，不进入Gate B/C或gravity compensation。','',
      'GATE A WITH EXTERNAL SUPPORT COMPLETE — DO NOT EXECUTE INTENTIONAL MOTION WITHOUT HUMAN AUTHORIZATION']
    (base/'GATE_A_RESULT.md').write_text('\n'.join(lines)+'\n')
    return info

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('base',type=Path);args=p.parse_args()
    s=compare(args.base);print(json.dumps(s['SUPPORT_A_B_COMPARISON'],indent=2))
