"""Offline Gate A JSONL analysis only; no device/ROS/transport capability."""
import argparse
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import statistics
import struct
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/real_arm_readonly'))
from real_arm_readonly.protocol import Decoder,SIGNS

def analyze(log_path,out):
    records=[json.loads(line) for line in log_path.read_text().splitlines()]
    snap=next(r['snapshot'] for r in records if r['kind']=='PRE_SEND_SNAPSHOT')
    tx=[r for r in records if r['kind']=='TX_KERNEL_ACCEPTED']
    exit_record=next(r for r in records if r['kind']=='EXIT')
    qh=snap['command']['position'];kp=snap['command']['kp'];kd=snap['command']['kd']
    t0=tx[0]['t'] if tx else None
    decoder=Decoder();latest={};ages={};events=[];position=[];velocity=[];times={}
    for r in records:
        if r['kind']!='RX':continue
        t=r['t'];decoder.feed(bytes.fromhex(r['hex']))
        for cmd,raw in decoder.raw_frames:
            latest[cmd]=list(raw);ages[cmd]=t;times.setdefault(cmd,[]).append(t)
            if cmd==0x20:position.append((t,[s*x for s,x in zip(SIGNS,raw)]))
            if cmd==0x21:velocity.append((t,[s*x for s,x in zip(SIGNS,raw)]))
            if cmd not in (0x20,0x21) or not all(c in latest for c in (0x20,0x21,0x22,0x24)):continue
            q=[s*x for s,x in zip(SIGNS,latest[0x20])];v=[s*x for s,x in zip(SIGNS,latest[0x21])]
            displacement=[x-h for x,h in zip(q,qh)];p=[-k*x for k,x in zip(kp,displacement)];d=[-k*x for k,x in zip(kd,v)]
            proxy=[x+y for x,y in zip(p,d)]
            margin=[min(x-lo,hi-x) for x,lo,hi in zip(q,(-4,-3.1415,-1.5708,-4,-2,-4),(4,2,1.5708,4,2,4))]
            events.append(dict(t=t,relative_t=t-t0 if t0 is not None else None,cmd=hex(cmd),q=q,dq=v,displacement=displacement,p=p,d=d,proxy=proxy,margin=margin,ages={hex(c):t-ages[c] for c in (0x20,0x21,0x22,0x24)}))
    active=[e for e in events if e['t']>=t0];pos=[(t,q) for t,q in position if t>=t0];vel=[(t,v) for t,v in velocity if t>=t0]
    end=max(r['t'] for r in records if r['kind']=='RX');coverage=end-t0
    # Include the locked position as the zero-displacement baseline.
    rows=[]
    for j in range(6):
        qs=[qh[j]]+[q[j] for _,q in pos];vs=[snap['measured']['dq'][j]]+[v[j] for _,v in vel]
        raw_slew=[abs(b['proxy'][j]-a['proxy'][j])/(b['t']-a['t']) for a,b in zip(active,active[1:]) if b['t']>a['t']]
        rows.append(dict(joint=f'joint{j+1}',q_hold=qh[j],max_displacement=max(abs(q-qh[j]) for q in qs),peak_to_peak=max(qs)-min(qs),final_displacement=qs[-1]-qh[j],max_velocity=max(map(abs,vs)),max_torque_proxy=max([abs(snap['torque_proxy']['total'][j])]+[abs(e['proxy'][j]) for e in active]),max_raw_proxy_finite_difference=max(raw_slew,default=None),max_guard_50ms_mean_slew=None,minimum_joint_limit_margin=min(e['margin'][j] for e in active),position_guard_utilization=max(abs(q-qh[j]) for q in qs)/snap['limits']['position_error_guard'],velocity_guard_utilization=max(map(abs,vs))/snap['limits']['velocity_guard'],proxy_guard_utilization=max(abs(e['proxy'][j]) for e in active)/snap['limits']['torque_proxy_guard'][j]))
    transient={}
    for ms in (20,50,100):
        pp=[q for t,q in pos if t-t0<=ms/1000];vv=[v for t,v in vel if t-t0<=ms/1000]
        transient[str(ms)+'ms']=dict(complete_window=coverage>=ms/1000,observed_duration_s=min(coverage,ms/1000),position_samples=len(pp),velocity_samples=len(vv),max_abs_displacement=[max([0]+[abs(q[j]-qh[j]) for q in pp]) for j in range(6)],max_abs_velocity=[max([abs(snap['measured']['dq'][j])]+[abs(v[j]) for v in vv]) for j in range(6)])
    # Reconstruct the original 40–50 ms displacement check, including qualification history.
    trigger=None
    for i,(t,q) in enumerate(position):
        candidates=[(tt,qq) for tt,qq in position[:i+1] if t-tt<=.05]
        if candidates and t-candidates[0][0]>=.04:
            delta=[a-b for a,b in zip(q,candidates[0][1])]
            if max(map(abs,delta))>.00153350830078125:
                trigger=dict(t=t,relative_t=t-t0,window_start=candidates[0][0],window_span=t-candidates[0][0],delta=delta,threshold=.00153350830078125,joints=[j+1 for j,x in enumerate(delta) if abs(x)>.00153350830078125]);break
    counts={hex(c):0 for c in range(0x10,0x15)};identical=len({r['hex'] for r in tx})==1
    for r in tx:
        b=bytes.fromhex(r['hex']);assert len(b)==r['accepted_bytes']==170
        for i,c in enumerate(range(0x10,0x15)):
            f=b[i*34:(i+1)*34];assert f[:4]==bytes((254,170,c,28))
            assert f[-2]==sum(f[:-2])%256 and f[-1]==sum(sum(f[:i+1]) for i in range(32))%256
            counts[hex(c)]+=1
    cmd_rates={hex(c):(len(ts)-1)/(ts[-1]-ts[0]) for c,ts in times.items() if len(ts)>1}
    gaps={hex(c):max(b-a for a,b in zip(ts,ts[1:])) for c,ts in times.items() if len(ts)>1}
    summary=dict(verdict='FAIL',reason=exit_record['reason'],run='run1',program_sha256=snap['program_sha256'],snapshot=snap,q_hold=qh,kp=kp,kd=kd,current=[0]*6,velocity_command=[0]*6,first_tx_schedule_t=t0,last_feedback_t=end,post_command_observation_s=coverage,whole_run=rows,transient=transient,displacement_guard_trigger=trigger,max_feedback_age_at_decoded_events=max(max(e['ages'].values()) for e in active),feedback_rates_hz=cmd_rates,max_feedback_gaps_s=gaps,checksum_errors=decoder.bad_checksum,malformed=decoder.bad_length,nonfinite=decoder.nonfinite,incomplete_tail_bytes=len(decoder.buffer),stale_detected=any(gaps.get(hex(c),0)>.05 for c in (0x20,0x21,0x22,0x24)),tx_counts=counts,accepted_bytes=sum(r['accepted_bytes'] for r in tx),command_batches_identical=identical,tx_batch_interval_rate_hz=(len(tx)-1)/(tx[-1]['t']-tx[0]['t']) if len(tx)>1 else None,tx_events=tx,exit=exit_record,onsite_observation='UNKNOWN — awaiting user report; no visual/audio telemetry available',power_isolation_confirmation='UNKNOWN — user instructed; assistant cannot verify physical isolation',close_time='close occurred before EXIT record and process completion; exact close timestamp not logged',runs_2_3='NOT EXECUTED: Run 1 guard abort prohibits retry',reproducibility='not assessable, only one aborted run',limitations=['TX timestamps are scheduling timestamps immediately before pre-write work, not measured UART/board reception times.','50/100ms windows truncated at abort.','No pair of post-takeover 50ms mean windows: reviewed mean-slew statistic unavailable; raw finite difference is separate and quantization-sensitive.','Full stored RX chunk decoded offline may include telemetry not processed by active guard after its first exception.','Trajectory duration too short to prove sustained acceleration/oscillation or long-term drift.','No active takeover retry permitted.'])
    onsite=log_path.with_suffix('.onsite.json')
    if onsite.exists():summary.update(json.loads(onsite.read_text()))
    out.mkdir(exist_ok=True,parents=True)
    (out/'gate_a_summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    fields=['run','t_host','t_from_first_tx_schedule','feedback_event','joint','q','dq','q_hold','q_minus_q_hold','position_command','velocity_command','current_command','kp','kd','p_proxy','d_proxy','ff_proxy','total_proxy','joint_limit_margin','position_feedback_age','velocity_feedback_age','max_feedback_age','tx_batches_accepted_so_far','phase','run_abort_reason']
    with (out/'gate_a_timeseries.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
        for e in events:
            for j in range(6):w.writerow(dict(zip(fields,['run1',e['t'],e['relative_t'],e['cmd'],j+1,e['q'][j],e['dq'][j],qh[j],e['displacement'][j],qh[j],0,0,kp[j],kd[j],e['p'][j],e['d'][j],0,e['proxy'][j],e['margin'][j],e['ages']['0x20'],e['ages']['0x21'],max(e['ages'].values()),sum(r['t']<=e['t'] for r in tx),'POST_FIRST_TX' if e['t']>=t0 else 'QUALIFICATION (command hypothetical)',exit_record['reason']])))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plots=out/'gate_a_plots';plots.mkdir(exist_ok=True)
    for key,label in [('displacement','q - q_hold (rad)'),('dq','velocity (rad/s)'),('proxy','new-command torque proxy (nominal Nm)')]:
        fig,axes=plt.subplots(3,2,figsize=(11,8),sharex=True)
        es=[e for e in events if e['relative_t']>=-.12]
        for j,ax in enumerate(axes.flat):
            ax.plot([e['relative_t']*1000 for e in es],[e[key][j] for e in es],'.-',ms=2)
            ax.axvline(0,color='black',ls='--',label='first TX schedule')
            ax.axvline(coverage*1000,color='red',ls=':',label='last RX / abort')
            ax.set_title(f'joint{j+1}');ax.grid(alpha=.3);ax.set_ylabel(label)
        axes[2,0].set_xlabel('ms relative to first TX schedule');axes[2,1].set_xlabel('ms relative to first TX schedule')
        axes[0,0].legend(fontsize=7);fig.suptitle('Gate A Run 1 — guard abort; no further runs')
        fig.tight_layout();fig.savefig(plots/(key+'.png'),dpi=150);plt.close(fig)
    return summary

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('log',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();result=analyze(args.log,args.output)
    print(json.dumps({k:result[k] for k in ('verdict','reason','post_command_observation_s','displacement_guard_trigger','tx_counts','checksum_errors','malformed','whole_run','transient')},indent=2))
