"""Offline analysis of the renewed communication-tier tuning session."""
import csv
import hashlib
import json
from pathlib import Path
import shutil
import struct
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_gravity_bringup import decode_rx, checksum

ROOT=Path(__file__).resolve().parents[3]
BASE=ROOT/'docs/real_arm_audit_20260917/gravity_tuning'
SESSION=BASE/'comm_recheck_20260918'
SIGNS=np.array([1,1,1,-1,-1,-1])


def analyze(n):
    path=SESSION/f'run{n:02}.jsonl';rows=[json.loads(x) for x in path.read_text().splitlines()]
    end=rows[-1];assert end['kind']=='EXIT'
    plan=json.loads((SESSION/f'plan{n+1:02}.json').read_text())
    frames,errors,tail=decode_rx(rows)
    last_pos=[f for f in frames if f['cmd']==0x20][-1]
    last_q=(np.array(last_pos['values'][:6])*SIGNS).tolist()
    tx=[r for r in rows if r['kind']=='TX_KERNEL_ACCEPTED']
    snap=next((r['snapshot'] for r in rows if r['kind']=='PRE_SEND_SNAPSHOT'),None)
    for r in tx:
        assert snap is not None and r['hex']==snap['packet_hex']
        packet=bytes.fromhex(r['hex']);assert len(packet)==170==r['accepted_bytes']
        for i,c in enumerate(range(0x10,0x15)):
            f=packet[i*34:(i+1)*34];assert f[:4]==bytes([254,170,c,28]) and f[-2:]==checksum(f[:-2])
            assert list(struct.unpack('<7f',f[4:-2]))==snap['theoretical_arrays'][hex(c)]
    assert sum(r['accepted_bytes'] for r in tx)==end['accepted_bytes']
    result=dict(run_id=f'run{n:02}',result=end['state'],reason=end['reason'],plan=plan,
        source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),snapshot=snap,exit=end,
        q_last_observed=last_q,independent_decode_errors=errors,
        partial_tail_hex=tail,tx_counts={hex(c):len(tx) for c in range(0x10,0x15)},
        all_packets_identical=True,tx_mean_hz=float(1/np.mean(np.diff([r['t'] for r in tx]))) if len(tx)>1 else None,
        metrics=None,transients={},on_site_observation='UNKNOWN: no current operator reply recorded')
    if not tx:
        result['preflight_q_minus_geometry_center']=(np.array(last_q)-plan['q_geometry_center']).tolist()
        result['remaining_geometry_envelope']=(np.array(plan['envelope'])-abs(np.array(last_q)-plan['q_geometry_center'])).tolist()
        result['feedback_observation_s']=None
        return result,[]
    first=end['first_tx_time'];latest={};at={};events=[]
    q0=np.array(snap['q_start']);kp=np.array(snap['kp']);kd=np.array(snap['kd']);ff=np.array(snap['current_command'])
    for f in frames:
        if f['values'] is None:continue
        c=f['cmd'];latest[c]=np.array(f['values'][:6])*SIGNS;at[c]=f['t']
        if c not in (0x20,0x21) or f['t']<first or not all(c in latest for c in (0x20,0x21)):continue
        q,v=latest[0x20],latest[0x21];p=kp*(q0-q);d=-kd*v
        events.append(dict(t=f['t'],elapsed=f['t']-first,cmd=c,q=q.copy(),v=v.copy(),delta=q-q0,p=p,d=d,ff=ff,proxy=p+d+ff,
            age=max(f['t']-at[c] for c in (0x20,0x21))))
    pos=[e for e in events if e['cmd']==0x20];q=np.array([e['q'] for e in pos]);t=np.array([e['elapsed'] for e in pos])
    v=np.array([e['v'] for e in events]);proxy=np.array([e['proxy'] for e in events]);allq=np.array([e['q'] for e in events])
    states=[r for r in rows if r['kind']=='STATE' and r['t']>=first]
    tailmask=t>=max(t[0],t[-1]-.2);tt=t[tailmask]-t[tailmask].mean()
    slope=np.sum(tt[:,None]*(q[tailmask]-q[tailmask].mean(axis=0)),axis=0)/np.sum(tt**2)
    margin=np.minimum(allq-plan['lower'],np.array(plan['upper'])-allq)
    mismatch=np.max([r['mismatch_duration_s'] for r in states],axis=0)
    mean_slew=[r['mean_proxy_slew'] for r in states if 'mean_proxy_slew' in r]
    result['metrics']=[dict(joint=f'joint{j+1}',q_start=float(q0[j]),q_end=float(q[-1,j]),
        max_displacement=float(np.max(abs(q[:,j]-q0[j]))),final_delta=float(q[-1,j]-q0[j]),
        max_abs_velocity=float(np.max(abs(v[:,j]))),velocity_rms=float(np.sqrt(np.mean(v[:,j]**2))),
        tail_drift_rad_s=float(slope[j]),tail_ptp_rad=float(np.ptp(q[tailmask,j])),
        max_abs_proxy=float(np.max(abs(proxy[:,j]))),max_proxy_guard_ratio=float(np.max(abs(proxy[:,j]))/([4.5]*3+[.8]*3)[j]),
        minimum_joint_margin=float(np.min(margin[:,j])),
        hard_displacement_guard_utilization=float(np.max(abs(q[:,j]-q0[j]))/(.8*snap['effective_envelope'][j])),
        max_direction_mismatch_s=float(mismatch[j]),direction_mismatch_samples=sum(bool(r['direction_mismatch'][j]) for r in states),
        max_mean_proxy_slew=float(np.max(np.array(mean_slew)[:,j])) if mean_slew else None,
        growing_oscillation_detected=any(r.get('reason')=='growing oscillation' for r in states)) for j in range(6)]
    result['feedback_observation_s']=end['last_receive_t']-first
    result['position_observation_s']=float(t[-1]);result['maximum_logged_critical_age_s']=max(max(r['feedback_ages'].values()) for r in states)
    result['direction_score_time_series']=[dict(t=r['t']-first,score=r['direction_consistency_score'],mismatch=r['direction_mismatch'],duration=r['mismatch_duration_s']) for r in states]
    for ms in (20,50,100,200,300,500,1000,1200):
        sub=[e for e in events if e['elapsed']<=ms/1000]
        result['transients'][str(ms)]=dict(delta=sub[-1]['delta'].tolist(),dq=sub[-1]['v'].tolist(),proxy=sub[-1]['proxy'].tolist()) if events[-1]['elapsed']>=ms/1000 else None
    return result,events


def main():
    archive=BASE/'before_resumed_validation_archive';archive.mkdir(exist_ok=True)
    for name in ('GRAVITY_TUNING_RESULT.md','gravity_tuning_summary.json','gravity_tuning_runs.csv','gravity_tuning_plots'):
        src,dst=BASE/name,archive/name
        if src.exists() and not dst.exists():shutil.copytree(src,dst) if src.is_dir() else shutil.copy2(src,dst)
    data=[analyze(i) for i in range(1,7)];runs=[r for r,_ in data]
    baseline=json.loads((SESSION/'baseline.json').read_text());preflight=json.loads((SESSION/'receive/preflight01.summary.json').read_text())
    gap=(np.array(runs[3]['q_last_observed'])-runs[2]['q_last_observed']).tolist()
    summary=dict(status='INCONCLUSIVE',active_runs=5,attempts=6,preflight=preflight,baseline=baseline,
        runs=runs,between_run03_and_attempt04_q_delta=gap,
        interval_motion_timing_and_cause='UNKNOWN: no continuous feedback between sessions',
        final_stop='run06 HARD_ABORT: continued position-derived acceleration; no restore or retry',
        final_validated_gravity_parameters=None,effective_scale_estimate='UNKNOWN',
        manual_guidance_executed=False,automatic_retry_after_hard_abort=False,
        physical_power_isolation_performed='UNKNOWN',tests_passed=44,
        timestamp_meaning='host RX completion / pre-write host timestamps, no board sample timestamps')
    observation=SESSION/'operator_contact_observation.json'
    if observation.exists():
        summary['operator_observation']=json.loads(observation.read_text())
        summary['interval_motion_timing_and_cause']='Operator confirms touching the end joint; exact timing/force/full attribution unknown. Do not use interval displacement as spontaneous-motion evidence.'
        runs[3]['on_site_observation']=summary['operator_observation']
    summary['resumed_preflight']=json.loads((SESSION/'resume_01/receive/preflight01.summary.json').read_text())
    summary['run06_guard_reconstruction']=json.loads((SESSION/'run06_guard_reconstruction.json').read_text())
    summary['run06_operator_observation']=json.loads((SESSION/'run06_operator_observation.json').read_text())
    runs[5]['on_site_observation']=summary['run06_operator_observation']
    summary['human_authorized_resume_after_run04']='恢复，继续工作; resumed run05/run06; no retry after run06'
    summary['total_tx_batches']=sum(r['exit']['tx_batches'] for r in runs)
    summary['total_tx_bytes']=sum(r['exit']['accepted_bytes'] for r in runs)
    summary['no_hardware_access_during_final_analysis']=True
    (BASE/'gravity_tuning_summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    csvrows=[];series=[]
    for run,events in data:
        p=run['plan'];s=run['snapshot']
        for j in range(6):
            row=dict(run_id=run['run_id'],joint=f'joint{j+1}',result=run['result'],
                lambda_planned=p['lambda_value'],lambda_executed=p['lambda_value'] if s else None,
                kp=p['kp'][j],kd=p['kd'][j],planned_duration_s=p['duration_s'],
                feedback_observation_s=run['feedback_observation_s'],restore_result=run['exit']['restore_result'],
                gravity_torque=s['g_hold'][j] if s else None,ff=s['current_command'][j] if s else None,
                desk_lower_bound_m=p['certificate']['minimum_desk_clearance'],
                self_distance_lower_m=p['certificate']['self_distance_lower'],
                tx_frames_per_command=run['tx_counts']['0x10'],tx_bytes=run['exit']['accepted_bytes'],
                runner_sha256=run['exit']['hashes']['gravity_tuning_runner.py'],q_observed=run['q_last_observed'][j])
            if run['metrics']:row.update(run['metrics'][j])
            csvrows.append(row)
        for e in events:
            for j in range(6):series.append(dict(run_id=run['run_id'],joint=f'joint{j+1}',elapsed_s=e['elapsed'],host_t=e['t'],event_cmd=hex(e['cmd']),
                q=e['q'][j],dq=e['v'][j],delta=e['delta'][j],q_command=s['q_start'][j],velocity_command=0,
                lambda_value=s['lambda_value'],kp=s['kp'][j],kd=s['kd'][j],p=e['p'][j],d=e['d'][j],ff=e['ff'][j],proxy=e['proxy'][j],critical_age=e['age']))
    for filename,rs in (('gravity_tuning_runs.csv',csvrows),('gravity_tuning_timeseries.csv',series)):
        fields=list(dict.fromkeys(k for r in rs for k in r))
        with (BASE/filename).open('w') as f:w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rs)
    plots=BASE/'gravity_tuning_plots';plots.mkdir(exist_ok=True)
    active=[d for d in data if d[1]]
    fig,axes=plt.subplots(len(active),2,figsize=(12,3*len(active)))
    for k,(r,events) in enumerate(active):
        pos=[e for e in events if e['cmd']==0x20];vel=[e for e in events if e['cmd']==0x21]
        t=[e['elapsed'] for e in pos];tv=[e['elapsed'] for e in vel]
        axes[k,0].plot(t,[e['delta'][4]*1000 for e in pos],'.-',markersize=2,label='J5')
        axes[k,0].axhline(.8*r['snapshot']['effective_envelope'][4]*1000,ls='--',color='red',label='Hard displacement boundary')
        axes[k,1].plot(tv,[e['v'][4] for e in vel],'.-',markersize=2,label='J5 protocol velocity')
        axes[k,0].set(title=f"{r['run_id']} lambda={r['snapshot']['lambda_value']} {r['result']}",ylabel='Displacement [mrad]')
        axes[k,1].set(ylabel='Velocity [rad/s]')
        for a in axes[k]:a.grid();a.legend(fontsize=8);a.set_xlabel('Time since first TX [s]')
    fig.tight_layout();fig.savefig(plots/'session_j5_motion.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(len(active),2,figsize=(12,3*len(active)))
    for k,(r,events) in enumerate(active):
        pos=[e for e in events if e['cmd']==0x20];t=[e['elapsed'] for e in pos]
        for j in (1,2):axes[k,0].plot(t,[e['delta'][j] for e in pos],label=f'J{j+1}')
        for key in ('p','d','ff','proxy'):axes[k,1].plot([e['elapsed'] for e in events],[e[key][4] for e in events],label=key)
        axes[k,0].set(title=r['run_id']+' J2/J3 (overlapping traces)',ylabel='Displacement [rad]')
        axes[k,1].set(title=r['run_id']+' J5 P/D/FF',ylabel='Nominal Nm')
        for a in axes[k]:a.grid();a.legend(fontsize=8);a.set_xlabel('Time since first TX [s]')
    fig.tight_layout();fig.savefig(plots/'session_j23_and_j5_proxy.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,4))
    x=[];y=[];labels=[]
    for r in runs:
        if not r['snapshot']:
            x.append(len(x));y.append(r['q_last_observed'][4]);labels.append(r['run_id']+' preflight');continue
        x.extend([len(x),len(x)+1]);y.extend([r['snapshot']['q_start'][4],r['q_last_observed'][4]]);labels.extend([r['run_id']+' start',r['run_id']+' last RX'])
    ax.scatter(x,y);ax.set(xticks=x,xticklabels=labels,ylabel='J5 position [rad]',title='Separate observation epochs — NO continuous trace between runs')
    ax.tick_params(axis='x',labelrotation=20);ax.grid();fig.tight_layout();fig.savefig(plots/'j5_between_sessions.png',dpi=160);plt.close(fig)
    write_report(summary)
    print(json.dumps(dict(status=summary['status'],runs=[dict(id=r['run_id'],result=r['result'],duration=r['feedback_observation_s'],j5=r['metrics'][4] if r['metrics'] else None) for r in runs],between_session_delta=gap),indent=2))


def write_report(summary):
    runs=summary['runs']; table=[]; metrics=[]; detail=[]
    for r in runs:
        m=r['metrics'];p=r['plan'];s=r['snapshot'];e=r['exit']
        table.append(f"| {r['run_id']} | {p['lambda_value']} | {p['duration_s']} | {r['feedback_observation_s']} | {r['result']} | {e['tx_batches']} | {e['restore_result']} |")
        if not m:continue
        for v in m:
            metrics.append(f"| {r['run_id']}/{v['joint']} | {v['max_displacement']:.7f} | {v['final_delta']:.7f} | {v['max_abs_velocity']:.6f} | {v['tail_drift_rad_s']:.6f} | {v['max_abs_proxy']:.6f} | {100*v['hard_displacement_guard_utilization']:.1f}% | {v['minimum_joint_margin']:.6f} |")
        detail.append(f"### {r['run_id']}\n\nq_start = `{s['q_start']}`\n\ng(q_start) = `{s['g_hold']}`；tau_ff=λg。\n\nq_end = `{r['q_last_observed']}`\n\nTX平均频率 {r['tx_mean_hz']:.3f} Hz；0x10–14 各 {e['tx_batches']} 帧；关键状态最大age {e['communication']['max_critical_age_s']*1000:.3f} ms。几何整箱桌面下界 {p['certificate']['minimum_desk_clearance']:.6f} m；self-distance下界 {p['certificate']['self_distance_lower']:.6f} m。最后TX时刻 {e['last_tx']['t']:.9f}s，串口关闭 {e['serial_closed_t']:.9f}s（均相对本run打开时钟）。\n\n[首写快照](comm_recheck_20260918/{r['run_id']}.jsonl.pre_send.json) · [完整日志](comm_recheck_20260918/{r['run_id']}.jsonl)")
    text=f"""# GRAVITY TUNING RESULT

2026-09-18。**最终 INCONCLUSIVE。Run06 触发 HARD_ABORT，已停止TX并关闭串口；没有恢复、没有重试。重力补偿未通过重复验证，未执行 manual guidance。**

## 本次恢复后的结果

用户“恢复，继续工作”之后，先完成8秒纯读取通信复检，再执行 Run05/06。全部使用 λ=1、Kp=[30,30,30,5,5,5]、Kd=[1,1,1,0.1,0.1,0.1]。每轮独立锁存实时q_start，q_target全程固定，无position step；FF冻结为g(q_start)。控制代码没有修改。

Run05：计划1.2秒，延长0.5秒检查回稳，共反馈覆盖1.700157秒。J5最大位移0.01726246 rad，最终+0.01649475 rad，峰值速度0.50061035 rad/s；尾200ms漂移仅0.00032283 rad/s、峰峰值一个编码量化步0.00038338 rad。尾部接近静止，但最终误差略超0.015rad恢复容差，故SOFT_FAIL。实际hard位移界占用91.36%，余量偏小；不能称为全部充分安全。

Run06：重新锁存q_start，计划1.2秒；首TX后约82.83ms的反馈、83.01ms的guard决策触发 `continued position-derived acceleration`。J5从0.087271690升至0.104535103 rad，Δ=+0.017263412 rad（约0.989°），峰值速度0.50061035 rad/s。其他关节只有零或一个量化步变化。J2/J3均没有编码器可分辨位移。只发送8批，随后停止，无退出控制帧。

用户现场补充：**“我没太注意，但感觉机械臂没有明显的运动”**。该观察与约1°的编码器变化并不矛盾；不是“确认完全没动”。是否动力隔离、Run06是否有接触仍UNKNOWN。关闭串口不代表失能，板端持久状态及断流行为UNKNOWN；请现场按情况实体动力隔离。

## 触发判据的离线复核：不能过度归因

Run06的三个30ms窗口位置OLS速度为J5 **0.023446 → 0.201016 → 0.342449 rad/s**；相邻增幅均超过两量化步/30ms=0.025558rad/s，且末值>0.08，符合当前已部署加速guard。第一窗含首写前快照，实际样本覆盖20.89ms；第二/三窗覆盖29.83/25.04ms，因此可在首TX后约83ms触发。

但更近的两个20ms窗口速度是 **0.365986 → 0.305589 rad/s**，短窗加速度约 **−3.01987 rad/s²**；末协议速度0.207569rad/s也低于此前峰值。这说明长窗口guard保留了启动加速的影响，**触发时已有减速迹象，不能把日志reason写成已证明持续发散或没有趋稳迹象。** 本轮没有修改guard，也没有以这一离线发现绕过HARD_ABORT重试。

J5最长连续 DIRECTION_MISMATCH约35.14ms，未达50ms持续矛盾阈值；最终短窗proxy与减速度同向。最终P=-0.086317、D=-0.020757、FF=-0.003526，proxy=-0.110600 nominal Nm，速度仍正向。proxy不是实测净力矩，不能据此直接证明sign错误。触发来源是独立的三窗口加速判据，不是50ms方向矛盾、单帧速度、checksum或位移硬界。

[逐窗重建](comm_recheck_20260918/run06_guard_reconstruction.json)。本次仍无法区分启动瞬态、板端保留状态、控制语义/标度和模型未覆盖力矩的贡献；不自动翻转符号、不改mapping、不调高增益。未来任何重试须经过此次hard abort的人工审阅。

## 全部实验与参数搜索

最初λ=.95通过静态/几何筛选，随后λ=1；增益始终相同，之后仅延长窗口。作者历史依据Phase2 HOLD_P/HOLD_D、Phase4 BASE_P/BASE_D；未使用Phase2脱困Kd=10。新参数均先离线可行性、几何、快照/dry-run；未使用历史offset、alpha=1.033或first_moment_ratio。

| Run | λ计划值 | 计划s | 首TX后反馈覆盖s | 结果 | TX批数 | restore |
| --- | ---: | ---: | --- | --- | ---: | --- |
{chr(10).join(table)}

Run01/02启动有短暂方向矛盾但随后趋稳；Run03短窗NORMAL。Run04首写前几何中止，TX=0。用户已说明Run03/04间碰过末端，不能将该会话间位移当成自主漂移证据；恢复授权后重新做几何计划与preflight才执行05/06。Run05可近稳，Run06提前停止，**两次一致的1–2秒保持成功标准未达到**。

J2/J3保持不变是观测事实，不能仅据此识别0x12精确sign/scale或排除高Kp/摩擦贡献。effective_scale_estimate=UNKNOWN，未拟合或采用历史alpha。

## 通信分级与检查

10份历史传输正常记录：0x20 maxgap9.030ms、p99 5.778、p99.9 6.200；0x21 maxgap15.021ms、p99 7.743、p99.9 9.794。典型周期4.996369ms。`ceil_ms(max(maxgap+2period,2*p99.9+period))`给出关键状态age硬界26ms；这是观测推导界限，不是完整闭环稳定性证明。

孤立checksum/malformed丢弃并warning，坏数据不更新状态；1s≥3非连续错误为soft，首TX前允许最多2次重开。2连续关键坏帧、250ms≥5坏帧且≥2%错误率、关键非有限、age≥26ms或I/O故障为hard。0x22和诊断单帧异常不令q/dq失效。

恢复前8.000063秒只读检查：0x20/21/22/24约200.8Hz，错误0，最大关键age8.933ms。Run05/06解析错误、stale均0，最大关键age8.989/8.634ms。独立解析全部6次原始RX并核验TX。首尾截断片段不当作合法坏帧；保存在JSON。**没有证据将J5动态解释为本次通信故障。**

[历史基线](comm_recheck_20260918/baseline.json) · [恢复只读检查](comm_recheck_20260918/resume_01/receive/preflight01.summary.json)

## 六轴指标

位置指标来自独立解码的全部合法FE55帧（含abort前最后帧）。速度/位置按接收时序保持最近值，非板端同步采样；尾漂移为末≤200ms位置OLS，Run06短窗不代表steady drift。零变化表示编码器未分辨变化。proxy为主机模型量，不是总实际力矩。

| Run/joint | max Δq rad | final Δq rad | peak abs dq rad/s | tail drift rad/s | peak abs proxy nominal Nm | hard位移界占用 | min限位余量rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
{chr(10).join(metrics)}

Run06 J5位移尚未到hard界（占88.69%），nominal proxy峰值0.121939，限额占15.24%；没有触发速度硬界。缺少长时段的50ms均值slew标为null，不能写成0。完整方向一致性、瞬态、均值slew见summary。

## 每轮快照、几何及退出

几何使用URDF/STL、作者桌面半空间及现场J3约10cm信息，整箱预测桌面间隙下界约5.31cm。它是依赖场景假设的保守计算，不是在线实测桌距；实际box比授权绝对上限小。

RESTORE本次仅保留原先q_start参考观察是否达到误差/静止容差，没有额外轨迹。Run01/05未达恢复容差；Run02/03已在容差内。Run06 hard abort禁止恢复。每轮关闭后无连续反馈，不能声称关闭后保持/失能/停止位移。

{chr(10).join(detail)}

## 安全审计与交付

所有run合计 **{summary['total_tx_batches']}批、{summary['total_tx_bytes']} bytes、{summary['total_tx_batches']*5}控制帧**；0x10/11/12/13/14各{summary['total_tx_batches']}帧。每批长度170B、checksum/次序检查通过，各run内与预写快照完全一致；slot6为0。内核接受不是板端ACK。未发reset/home/enable=0或退出补偿帧；未启动ROS MotorControl/controller。Run06停止后fuser未发现串口占用，也没有runner进程；此后只读文件分析，未重开串口。

runner SHA256 `8fded896c1dc9baaf4897f685c76196e249c1cf84b84079e3e425360e96acd12`；comm `015052fd5393a6e98a0edab9405b7069e4ec9c500422e2077184d7ac48d67f0c`；policy `c046050fc96471b38b8330bf2be06e5447ea7d0de652cbbb9b813ac236931c9b`。全部run hash一致，控制实现未因本轮分析变化。此前44项离线测试通过，恢复dry-run open/write=0。最终分析只更新分析脚本及报告。

UNKNOWN：精确固件版本、0x12实际标度/符号、板端总力矩饱和、持久状态、断流后行为、实际动力隔离状态。本次不声称控制系统已经安全认证。

- [全部摘要和快照](gravity_tuning_summary.json)
- [逐run逐joint指标](gravity_tuning_runs.csv)
- [原始事件对齐时间序列](gravity_tuning_timeseries.csv)
- [J5运动图](gravity_tuning_plots/session_j5_motion.png)
- [J2/J3和J5 P/D/FF](gravity_tuning_plots/session_j23_and_j5_proxy.png)
- [离散会话姿态](gravity_tuning_plots/j5_between_sessions.png)

先前报告已归档至before_resumed_validation_archive；更早通信中止报告在before_comm_recheck_archive。

**INCONCLUSIVE — RUN06 HARD_ABORT; NO RESTORE, NO RETRY.**

**GRAVITY COMPENSATION NOT VALIDATED — MANUAL GUIDANCE NOT EXECUTED.**
"""
    (BASE/'GRAVITY_TUNING_RESULT.md').write_text(text)


if __name__=='__main__':main()
