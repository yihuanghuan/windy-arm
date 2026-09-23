#!/usr/bin/env python3
"""Offline report from saved JSONL only. No driver, serial, ROS or control imports.

RX timestamps are host chunk completion times, not encoder sample timestamps.
Incomplete trailing frames are retained and never interpreted as telemetry.
"""
import argparse
from collections import Counter
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

SIGNS = np.array([1, 1, 1, -1, -1, -1])
LENGTHS = {0x20: 28, 0x21: 28, 0x22: 28, 0x23: 28, 0x24: 14}
REQUIRED = (0x20, 0x21, 0x22, 0x24)
PROXY_LIMIT = np.array([4.5, 4.5, 4.5, .8, .8, .8])
QUANTUM = .0003833770751953125


def checksum(data):
    return bytes((sum(data) % 256, sum((len(data)-i)*b for i, b in enumerate(data)) % 256))


def decode_rx(rows):
    buf = b''
    errors = Counter(checksum=0, malformed=0, nonfinite=0, discarded_bytes=0)
    frames = []
    for row in rows:
        if row['kind'] != 'RX':
            continue
        buf += bytes.fromhex(row['hex'])
        while buf:
            start = buf.find(b'\xfe\x55')
            if start < 0:
                keep = int(buf.endswith(b'\xfe'))
                errors['discarded_bytes'] += len(buf)-keep
                buf = buf[-1:] if keep else b''
                break
            errors['discarded_bytes'] += start
            buf = buf[start:]
            if len(buf) < 4:
                break
            cmd, length = buf[2:4]
            if cmd in LENGTHS and length != LENGTHS[cmd]:
                errors['malformed'] += 1
                buf = buf[1:]
                continue
            if len(buf) < length+6:
                break
            frame = buf[:length+6]
            if checksum(frame[:-2]) != frame[-2:]:
                errors['checksum'] += 1
                buf = buf[1:]
                continue
            buf = buf[length+6:]
            values = None
            if cmd in LENGTHS:
                values = struct.unpack('<7H' if cmd == 0x24 else '<7f', frame[4:-2])
                if not np.isfinite(values).all():
                    errors['nonfinite'] += 1
            frames.append(dict(t=row['t'], cmd=cmd, values=values, hex=frame.hex()))
    return frames, dict(errors), buf.hex()


def check_tx(rows, snapshot):
    tx = [r for r in rows if r['kind'] == 'TX_KERNEL_ACCEPTED']
    counts = Counter({hex(c): 0 for c in range(0x10, 0x15)})
    for row in tx:
        packet = bytes.fromhex(row['hex'])
        assert len(packet) == row['accepted_bytes'] == 170
        assert row['hex'] == snapshot['packet_hex'], 'changed target/command packet'
        for index, cmd in enumerate(range(0x10, 0x15)):
            frame = packet[index*34:(index+1)*34]
            assert frame[:4] == bytes((254, 170, cmd, 28))
            assert checksum(frame[:-2]) == frame[-2:]
            values = struct.unpack('<7f', frame[4:-2])
            assert values[-1] == 0
            assert list(values) == snapshot['theoretical_arrays'][hex(cmd)]
            counts[hex(cmd)] += 1
    gaps = np.diff([r['t'] for r in tx])
    return dict(counts=dict(counts), batches=len(tx), bytes=sum(r['accepted_bytes'] for r in tx),
                inter_batch_mean_hz=float(1/np.mean(gaps)) if len(gaps) else None,
                inter_batch_gap_s=gaps.tolist(), all_packets_equal_snapshot=True,
                last=tx[-1] if tx else None,
                meaning='kernel-accepted bytes; board receipt/execution not acknowledged')


def analyze(path, plan):
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    snapshot = json.loads(Path(str(path)+'.pre_send.json').read_text())
    assert next(r['snapshot'] for r in rows if r['kind']=='PRE_SEND_SNAPSHOT') == snapshot
    assert snapshot['position_command'] == snapshot['q_start'] == snapshot['q']
    assert snapshot['q_command_minus_measured'] == [0]*6
    exit_row = rows[-1]
    assert exit_row['kind'] == 'EXIT'
    frames, errors, tail = decode_rx(rows)
    tx = check_tx(rows, snapshot)
    assert tx['bytes'] == exit_row['accepted_bytes']
    first = exit_row['first_tx_time']
    last_rx = max(r['t'] for r in rows if r['kind']=='RX')
    q0 = np.array(snapshot['q_start'])
    kp, kd, ff = (np.array(snapshot[k]) for k in ('kp', 'kd', 'current_command'))
    env = np.array(snapshot['effective_envelope'])
    latest, arrivals, history = {}, {}, []
    max_age = 0.
    stale = 0
    for frame in frames:
        cmd, t = frame['cmd'], frame['t']
        arrivals[cmd] = t
        if frame['values'] is not None:
            latest[cmd] = np.array(frame['values'][:6])*SIGNS
        if not all(c in latest for c in REQUIRED):
            continue
        age = max(t-arrivals[c] for c in REQUIRED)
        max_age = max(max_age, age)
        stale += int(age > .05)
        if first is None or t < first or cmd not in (0x20, 0x21):
            continue
        q, v = latest[0x20], latest[0x21]
        p, d = kp*(q0-q), -kd*v
        history.append(dict(t=t, elapsed=t-first, cmd=cmd, q=q.copy(), dq=v.copy(),
                            p=p, d=d, ff=ff, proxy=p+d+ff,
                            feedback_age=age,
                            margin=np.minimum(q-np.array(plan['lower']), np.array(plan['upper'])-q)))
    states = [r for r in rows if r['kind']=='STATE']
    comm = dict(errors=errors, incomplete_tail_hex=tail,
                frame_counts=dict(Counter(hex(f['cmd']) for f in frames)),
                max_required_age_at_frame_event_s=max_age, stale_frame_events=stale,
                max_logged_guard_age_s=max(max(r['feedback_age'].values()) for r in states),
                streams={})
    for cmd in REQUIRED:
        times = np.array([f['t'] for f in frames if f['cmd']==cmd])
        # Avoid serial bytes already queued on open; host rates over settled receive window.
        steady = times[times >= .05]
        comm['streams'][hex(cmd)] = dict(count=len(times),
            receive_hz_after_first_50ms=float((len(steady)-1)/(steady[-1]-steady[0])),
            max_gap_s=float(np.max(np.diff(times))),
            final_age_at_last_rx_s=float(last_rx-times[-1]))
    result = dict(run_id=path.stem, program_sha256=exit_row['program_sha256'],
        source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        result=exit_row['state'], reason=exit_row['reason'], snapshot=snapshot,
        exit=exit_row, tx=tx, communication=comm,
        observation_duration_after_first_tx_s=last_rx-first if first is not None else 0.,
        physical_observation='UNKNOWN: no post-run operator observation received',
        power_isolation_performed='UNKNOWN',
        online_warning_iterations=sum(r['velocity_spike_warning'] for r in states),
        joint_metrics=None, transients={}, guard_reconstruction=None)
    if not history:
        return result, history
    q = np.array([h['q'] for h in history]); v = np.array([h['dq'] for h in history])
    proxy = np.array([h['proxy'] for h in history]); delta = q-q0
    pos = [h for h in history if h['cmd']==0x20]
    vel = [h for h in history if h['cmd']==0x21]
    # Position updates only: no duplicated zero-order-held values in OLS.
    times = np.array([h['elapsed'] for h in pos])
    positions = np.array([h['q'] for h in pos])
    slope = np.sum((times-times.mean())[:,None]*(positions-positions.mean(axis=0)),axis=0)/np.sum((times-times.mean())**2)
    # Observed adjacent position-event proxy differences, NOT the 3-block online guard.
    pp = np.array([h['proxy'] for h in pos]); dt = np.diff(times)
    slew = np.max(abs(np.diff(pp,axis=0)[dt>0]/dt[dt>0,None]),axis=0)
    margins = np.array([h['margin'] for h in history])
    result['joint_metrics'] = [dict(joint=f'joint{i+1}', q_start=float(q0[i]),
        q_end=float(q[-1,i]), final_displacement=float(delta[-1,i]),
        max_abs_displacement=float(abs(delta[:,i]).max()),
        position_peak_to_peak=float(np.ptp(np.r_[q0[i],positions[:,i]])),
        max_abs_protocol_velocity=float(abs(v[:,i]).max()),
        position_ols_rate_rad_s=float(slope[i]),
        max_abs_torque_proxy=float(abs(proxy[:,i]).max()),
        max_adjacent_position_event_proxy_slew=float(slew[i]),
        min_joint_limit_margin=float(margins[:,i].min()),
        max_displacement_guard_ratio=float(abs(delta[:,i]).max()/env[i]),
        max_proxy_guard_ratio=float(abs(proxy[:,i]).max()/PROXY_LIMIT[i]),
        velocity_spike_complete_frames=int(sum(abs(h['dq'][i])>.293060302734375 for h in vel))) for i in range(6)]
    for ms in (10,20,30,50,100,250,500,2000):
        if result['observation_duration_after_first_tx_s'] < ms/1000:
            result['transients'][str(ms)] = None
            continue
        sub = [h for h in history if h['elapsed']<=ms/1000]
        result['transients'][str(ms)] = dict(last_sample_elapsed_s=sub[-1]['elapsed'],
            delta=(sub[-1]['q']-q0).tolist(), dq=sub[-1]['dq'].tolist(), proxy=sub[-1]['proxy'].tolist(),
            max_abs_delta=np.max(abs(np.array([h['q'] for h in sub])-q0),axis=0).tolist(),
            max_abs_dq=np.max(abs(np.array([h['dq'] for h in sub])),axis=0).tolist())
    recent = [h for h in pos if pos[-1]['t']-h['t']<=.05]
    span = recent[-1]['t']-recent[0]['t']
    speed = (recent[-1]['q']-recent[0]['q'])/span
    vmax = np.minimum([.15]*3+[.20]*3,env/.15)
    result['guard_reconstruction'] = dict(method='offline host RX position times; exact raising-check timestamp not logged',
        span_s=span, delta=(recent[-1]['q']-recent[0]['q']).tolist(), speed=speed.tolist(),
        limits=vmax.tolist(), utilization=(abs(speed)/vmax).tolist(),
        three_50ms_block_acceleration_oscillation_slew='insufficient active duration; NOT established safe',
        abort_sample_in_successful_STATE_log=False)
    return result, history


def export(base, runs, histories):
    archive = base/'pre_adaptive_offline_archive'
    archive.mkdir(exist_ok=True)
    for name in ('GRAVITY_BRINGUP_RESULT.md','gravity_bringup_summary.json','gravity_bringup_timeseries.csv','gravity_bringup_plots'):
        src, dst = base/name, archive/name
        if src.exists() and not dst.exists():
            shutil.copytree(src,dst) if src.is_dir() else shutil.copy2(src,dst)
    summary = dict(verdict='INCONCLUSIVE', active_run_verdict='FAIL — HARD_ABORT',
        gravity_sign='UNKNOWN', effective_scale_estimate='UNKNOWN', validated_lambda=[],
        no_retry_no_restore_after_hard_abort=True, runs=runs,
        timestamp_semantics='host RX completion and pre-write host TX timestamps; no board sample clock',
        source='adaptive_20260917; analysis is offline only')
    (base/'gravity_bringup_summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    compact=[]
    for run in runs:
        snap=run['snapshot']
        for i in range(6):
            row=dict(run_id=run['run_id'], joint=f'joint{i+1}', result=run['result'],
                program_sha256=run['program_sha256'], lambda_value=snap['lambda_value'],
                kp=snap['kp'][i], kd=snap['kd'][i], q_start=snap['q_start'][i],
                gravity=snap['g_hold'][i], tau_gc=snap['current_command'][i],
                observation_s=run['observation_duration_after_first_tx_s'],
                tx_batches=run['tx']['batches'], tx_bytes=run['tx']['bytes'],
                desk_lower_bound_m=snap['desk_clearance_certified'])
            if run['joint_metrics']:
                row.update(run['joint_metrics'][i])
            compact.append(row)
    columns=list(dict.fromkeys(k for r in compact for k in r))
    with (base/'gravity_bringup_all_runs.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=columns);w.writeheader();w.writerows(compact)
    timeseries=[]
    for run, hist in zip(runs,histories):
        for h in hist:
            for i in range(6):
                timeseries.append(dict(run_id=run['run_id'],event_cmd=hex(h['cmd']),joint=f'joint{i+1}',
                    host_t_s=h['t'],elapsed_s=h['elapsed'],q=h['q'][i],dq=h['dq'][i],
                    delta_q=h['q'][i]-run['snapshot']['q_start'][i],
                    q_command=run['snapshot']['q_start'][i], velocity_command=0,
                    kp=run['snapshot']['kp'][i], kd=run['snapshot']['kd'][i],
                    lambda_value=run['snapshot']['lambda_value'],p_term=h['p'][i],d_term=h['d'][i],
                    ff_term=h['ff'][i],torque_proxy=h['proxy'][i],feedback_age_s=h['feedback_age'],
                    joint_limit_margin=h['margin'][i]))
    with (base/'gravity_bringup_timeseries.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(timeseries[0]));w.writeheader();w.writerows(timeseries)
    plots=base/'gravity_bringup_plots';plots.mkdir(exist_ok=True)
    run=runs[-1];hist=histories[-1];snap=run['snapshot']
    pos=[h for h in hist if h['cmd']==0x20];vel=[h for h in hist if h['cmd']==0x21]
    t=np.array([h['elapsed']*1000 for h in pos]);tv=np.array([h['elapsed']*1000 for h in vel])
    q=np.array([h['q'] for h in pos])-snap['q_start'];v=np.array([h['dq'] for h in vel])
    def save(fig,name):
        fig.tight_layout();fig.savefig(plots/name,dpi=160);plt.close(fig)
    fig,ax=plt.subplots(2,1,figsize=(9,6),sharex=True)
    for j in (1,2):
        ax[0].plot(t,q[:,j],'.-',label=f'J{j+1}')
        ax[1].plot(tv,v[:,j],'.-',label=f'J{j+1}')
    ax[0].set(ylabel='Displacement [rad]',title='Run02: J2/J3, only 62.83 ms observed; overlapping traces')
    ax[1].set(ylabel='Protocol velocity [rad/s]',xlabel='Host time since first TX [ms]')
    for a in ax:a.grid();a.legend()
    save(fig,'j2_j3_position_velocity.png')
    fig,ax=plt.subplots(3,1,figsize=(9,8),sharex=True)
    ax[0].plot(t,q[:,4]*1000,'.-');ax[0].set(ylabel='J5 displacement [mrad]',title='Run02 HARD_ABORT: sustained position-consistent velocity')
    ax[1].plot(tv,v[:,4],'.-');ax[1].set(ylabel='J5 protocol velocity [rad/s]')
    for k in ('p','d','ff','proxy'):
        ax[2].plot([h['elapsed']*1000 for h in hist],[h[k][4] for h in hist],label=k)
    ax[2].set(ylabel='Nominal proxy [Nm]',xlabel='Host time since first TX [ms]');ax[2].legend()
    for a in ax:a.grid();a.axvline(t[-1],color='red',ls='--',alpha=.5)
    save(fig,'j5_abort.png')
    fig,ax=plt.subplots(2,1,figsize=(9,6),sharex=True)
    ax[0].plot(t,np.full(len(t),snap['lambda_value']));ax[0].set(ylabel='lambda',ylim=(0,1.05),title='One executed lambda only; commands remained constant')
    for j in range(6):ax[1].plot(t,np.full(len(t),snap['current_command'][j]),label=f'J{j+1}')
    ax[1].set(ylabel='Feedforward command [nominal Nm]',xlabel='Host time since first TX [ms]');ax[1].legend(ncol=3)
    save(fig,'lambda_and_feedforward.png')
    fig,ax=plt.subplots(figsize=(8,4))
    for j,m in enumerate(run['joint_metrics']):ax.scatter([.99],[m['max_abs_displacement']*1000],label=f'J{j+1}',marker=['o','s','^','v','*','x'][j])
    ax.set(xlabel='lambda',ylabel='Max displacement [mrad]',title='Single aborted run: no lambda response trend can be inferred',xlim=(.94,1.01));ax.legend(ncol=3);ax.grid()
    save(fig,'displacement_vs_lambda.png')
    (plots/'README.md').write_text('Plots use complete RX frames, host receipt times, and no interpolation of missing tail frames.\nOnly Run02 sent commands; 0.99 was the only executed lambda. No post-close motion was observed in this capture.\nThe older offline_restore_feasibility.png is a historical prediction, not this experiment.\n')
    write_report(base, runs)


def write_report(base, runs):
    run = runs[1]; snap = run['snapshot']; metrics = run['joint_metrics']
    table = '\n'.join('| '+m['joint']+' | '+' | '.join(f"{m[k]:.9f}" for k in
        ('q_start','q_end','max_abs_displacement','position_peak_to_peak','max_abs_protocol_velocity','final_displacement'))+' |'
        for m in metrics)
    guards = '\n'.join(f"| {m['joint']} | {100*m['max_displacement_guard_ratio']:.2f}% | {m['max_abs_torque_proxy']:.6f} | {100*m['max_proxy_guard_ratio']:.2f}% | {m['max_adjacent_position_event_proxy_slew']:.6f} | {m['min_joint_limit_margin']:.6f} |" for m in metrics)
    transients = '\n'.join(f"| {ms} | {x['last_sample_elapsed_s']*1000:.3f} | {x['delta'][4]:.9f} | {x['dq'][4]:.6f} | {x['proxy'][4]:.6f} | 0 / 0 |" for ms,x in run['transients'].items() if x is not None)
    streams = '\n'.join(f"| {cmd} | {d['count']} | {d['receive_hz_after_first_50ms']:.3f} | {d['max_gap_s']*1000:.3f} |" for cmd,d in run['communication']['streams'].items())
    report = f'''# GRAVITY BRING-UP RESULT

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

Run01 SHA256：`{runs[0]['program_sha256']}`。

Run02 SHA256：`{run['program_sha256']}`。离线续办核对当前控制文件仍为此值，没有修改控制程序。

Run01 无 TX 后仅修改日志：保留完整持久化 snapshot，终端由大段 JSON 改为紧凑输出，增加日志耗时记录；未修改控制参数、guard、周期、协议顺序。原延迟原因没有足够时间分段记录，不能断言是终端唯一导致。变更详见 [logging_only_change.json](adaptive_20260917/logging_only_change.json)。Run02 保存快照与 fsync 耗时约 1.387 ms，终端约 0.022 ms。

两个版本分别通过 17 项离线测试；dry-run 记录串口 open/write=0。此次分析另有 7 项测试通过，包括独立 RX 解码与现有纯解码器逐帧一致、分块、尾部半帧、checksum、无 TX 与中止样本处理、变更命令拒绝。见 [offline_analysis_tests.txt](adaptive_20260917/offline_analysis_tests.txt)。没有再运行 hardware 程序。

## 4. 初始状态、命令与预写记录

Run01 与 Run02 分别重新锁存了各自反馈，不复用旧姿态。完整 q、dq、P/D/FF、age、joint margin、理论协议数组及 packet hex 分别见：

- [Run01 PRE_SEND_SNAPSHOT](adaptive_20260917/run01.jsonl.pre_send.json)
- [Run02 PRE_SEND_SNAPSHOT](adaptive_20260917/run02.jsonl.pre_send.json)

Run02 q_start：`{snap['q_start']}`。

g(q_start)：`{snap['g_hold']}`（模型 Nm）。

tau_gc=0.99g：`{snap['current_command']}`（主机期望的 nominal Nm；真实板端物理标度未验证）。

第一帧 q_command−measured_q=[0,0,0,0,0,0]；快照已先成功落盘。首帧 torque proxy：`{snap['torque_proxy']}`。q 与快照值在 float32 线上完全对应；前馈/阻尼有常规 float32 表示舍入，保存了精确 wire 数组。

## 5. 全观察窗口运动统计

角度 rad，速度 rad/s；包含最后一个触发 guard 的原始位置帧。q_end 只是最后收到的状态，**不是串口关闭后最终机械姿态**。

| Joint | q_start | q_end | max abs Δq | position p-p（含 q_start） | max abs dq | final Δq |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
{table}

J2/J3 基于独立位置帧的全窗口 OLS 斜率均为 0 rad/s，即未分辨出位移；协议速度非零偏置/量化不等于确定运动。J5 全窗口位置斜率约 0.109976 rad/s，并在末段增大；不是只有一帧速度尖峰。J4/J6 变化为约一个位置量化单位，不能凭这次短记录判定其实际微动成因。

首 TX 后约 0.072 ms 就收到 J5 一个量化单位偏差，可能涉及采样/缓存时延，不把该第一点单独归因为命令瞬态；后续连续增长是本次保护判断的主要依据。

| 观察截止 ms | 最后完整事件 ms | J5 Δq | J5 最新完整 dq | J5 proxy | J2/J3 Δq |
| --- | ---: | ---: | ---: | ---: | --- |
{transients}

100/250/500/2000 ms 无数据，不做外推。位置与速度按主机接收顺序采用最新值保持对齐，不声称具有控制板同步采样时间。完整 transient max 指标见 JSON。

## 6. Guard、几何与桌面间隙

有效 displacement envelope=`{snap['effective_envelope']}` rad。限位取 URDF/硬件 config 交集，最低保留 0.05 rad。nominal total proxy=Kp(q_start−q)−Kd·dq+tau_gc；界限 J1–3=4.5、J4–6=0.8 nominal Nm。它不是控制板实际总力矩饱和保证。

| Joint | displacement guard占用 | max abs proxy | proxy guard占用 | 相邻位置事件 proxy slew（nominal Nm/s） | 最小限位余量 rad |
| --- | ---: | ---: | ---: | ---: | ---: |
{guards}

**实际触发 sustained position-consistent velocity，而非位移上限或单帧速度。** 离线以最后约 49.842 ms 位置窗重建：J5 位移 0.007287979 rad，均速 0.146220722 rad/s；本轮阈值 min(0.20, envelope/0.15)=0.125444153 rad/s，约 **116.56%**，且超过两个位置量化单位。此重建用 RX 时间；抛异常的精确 check 时间未记录，因此不伪称逐时钟完全相同。

J5 有 2 个完整速度帧超过 warning 阈值 0.293060303 rad/s；在线 STATE 中 9 个循环标为 warning，循环数不是独立速度样本数。三个 50 ms 分块的持续加速、振荡和平均 proxy slew 判据尚未获得足够主动窗口；不能报告“已排除加速/振荡”。上表相邻事件 slew 是描述性离线指标，不是那个在线分块 guard。

几何计划见 [plan_v2_logging_only.json](adaptive_20260917/plan_v2_logging_only.json)：STL 三角形与桌面半空间裁剪、关节箱内保守点运动界、非相邻 link 距离。使用作者场景桌面 z=0、x≤0.02 和现场 J3≈10 cm，再保留 1 cm 测量预留。取更保守结果，整包络桌面间隙下界 **0.053027387 m**，非相邻 mesh 间距下界 **0.005451328 m**，现场 J3 约束推得下界 **0.078669568 m**。

这些是依赖 URDF、base/table 位置、mesh 覆盖和清空场景假设的**预测界**，不是实测动态间隙；没有经过本机完整桌面坐标标定。本次 q 在该箱内，但不能将模型界作为现场几何已完全验证的结论。用户“J3 最低约10 cm”与模型另有 link 较低的差异保留，已使用较小一侧。

## 7. 通信与 TX 审计

独立 FE55 解码校验全部完整帧；对 0x20/21/22/23 用 7 float，对 0x24 用 7 uint16。Run02 统计（频率去掉开串口最初 50 ms 缓冲窗口）：

| cmd | 全记录合法帧数 | RX Hz | 最大帧间隔 ms |
| --- | ---: | ---: | ---: |
{streams}

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
'''
    (base/'GRAVITY_BRINGUP_RESULT.md').write_text(report)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--base',type=Path,default=Path('docs/real_arm_audit_20260917/gravity_bringup'))
    args=ap.parse_args();saved=args.base/'adaptive_20260917'
    plan=json.loads((saved/'plan_v2_logging_only.json').read_text())
    analyzed=[analyze(saved/f'run{i:02}.jsonl',plan) for i in (1,2)]
    export(args.base,[x[0] for x in analyzed],[x[1] for x in analyzed])
    print(json.dumps({r['run_id']:dict(result=r['result'],metrics=r['joint_metrics'],guard=r['guard_reconstruction'],tx=r['tx']) for r,_ in analyzed},indent=2))


if __name__=='__main__':
    main()
