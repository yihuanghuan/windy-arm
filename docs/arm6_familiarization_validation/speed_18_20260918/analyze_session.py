"""Offline analysis only; never publishes commands or opens hardware."""
import json
import re
import struct
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

P = Path(__file__).resolve().parent
events = []
for line in (P/'events.jsonl').open():
    try:
        events.append(json.loads(line))
    except json.JSONDecodeError:  # A live recorder can have an incomplete last line.
        break
pol = [e for e in events if e['kind'] == 'policy']
t0 = pol[0]['wall_ns']/1e9
starts = [e['wall_ns']/1e9 for e in pol if e['policy_phase'] == 'HANDOVER_START']

def arr(rows, key):
    return np.asarray([e[key] for e in rows])

def stats(rows):
    q = arr(rows, 'q')
    return dict(duration_sec=(rows[-1]['wall_ns']-rows[0]['wall_ns'])/1e9,
                max_abs_dq=np.max(abs(arr(rows, 'dq')), axis=0).tolist(),
                q_span_rad=np.ptp(q, axis=0).tolist(),
                net_delta_q_rad=(q[-1]-q[0]).tolist(),
                max_same_feedback_command_step_Nm=np.max(abs(arr(rows,
                    'delta_total_command_from_previous_cycle')), axis=0).tolist())

summary = {'trials': [], 'faults': [e for e in events if e['kind']=='fault_snapshot']}
for i, start in enumerate(starts):
    end = starts[i+1] if i+1<len(starts) else float('inf')
    rows = [e for e in pol if start <= e['wall_ns']/1e9 < end]
    handover = [e for e in rows if e['policy_phase'] not in ('GUIDE','MANUAL_PUSH')]
    guide = [e for e in rows if e['policy_phase']=='GUIDE']
    quiet = [e for e in guide if e['wall_ns'] <= guide[0]['wall_ns']+5_010_000_000] if guide else []
    summary['trials'].append(dict(trial=i+1, first_command=rows[0],
        handover=stats(handover), first_five_seconds=stats(quiet) if quiet else None))

# Decode actual wire command batches, verify checksums, dimensions and ghost slot.
pattern = re.compile(r'^\d+\s+([\d.]+)\s+write\(\d+, "(.*?)", (\d+)\)\s+=\s+(\d+)')
tx=[]
with (P/'serial_tx_decoded.jsonl').open('w') as out:
    for line in (P/'serial_tx.strace').open():
        m=pattern.match(line)
        if not m or float(m[1])<t0-2 or float(m[1])>events[-1]['wall_ns']/1e9:
            continue
        b=bytes.fromhex(m[2].replace('\\x',''))
        assert len(b)==int(m[3])==int(m[4])==170
        row={'wall':float(m[1])}
        for k,key in enumerate(('position','velocity','ff','p','d')):
            f=b[k*34:(k+1)*34]
            assert f[:4]==bytes([0xfe,0xaa,0x10+k,28])
            total=cumulative=0
            for v in f[:32]:
                total=(total+v)%256; cumulative=(cumulative+total)%256
            assert f[32:]==bytes([total,cumulative])
            v=np.array(struct.unpack('<7f',f[4:32]))
            assert v[6]==0 and np.isfinite(v).all()
            v=v[:6]
            if key in ('position','velocity','ff'):v*= [1,1,1,-1,-1,-1]
            row[key]=v.tolist()
        tx.append(row);out.write(json.dumps(row)+'\n')
summary['validated_wire_batches']=len(tx)
summary['warnings']=[e for e in events if e['kind']=='log' and 'WARNING' in e.get('message','')]
summary['operator_results']=[json.loads(l) for l in (P/'operator_results.jsonl').open()]
manual_start=next((e['wall_ns']/1e9 for e in pol if e['policy_phase']=='MANUAL_PUSH'),None)
states=[e for e in events if e['kind']=='state' and e['wall_ns']/1e9>=t0]
support=[e for e in events if e['kind']=='support' and e['wall_ns']/1e9>=t0]
if summary['faults']:
    fault_wall=summary['faults'][0]['wall_ns']/1e9
    before=[e for e in support if e['wall_ns']/1e9<fault_wall][-1]
    after=[e for e in support if e['wall_ns']/1e9>=fault_wall]
    summary['fallback']={'before':before,'first':after[0],'settled':after[-1],
        'post_fault_q_span_rad':np.ptp(arr(after,'q'),axis=0).tolist(),
        'post_fault_max_abs_dq':np.max(abs(arr(after,'dq')),axis=0).tolist()}
    summary['fault_window']=[e for e in events if fault_wall-2<=e['wall_ns']/1e9<=fault_wall+2]
    (P/'fault_window.json').write_text(json.dumps(summary.pop('fault_window'),indent=2)+'\n')
(P/'analysis.json').write_text(json.dumps(summary,indent=2)+'\n')

logs=[e for e in events if e['kind']=='log' and
      (e.get('message','').startswith('ACTIVE ') or 'HARD_ABORT' in e.get('message','') or
       'SPEED WARNING' in e.get('message','') or 'HOLD: operator' in e.get('message',''))]
pt=arr(pol,'wall_ns')/1e9-t0
wt=arr(tx,'wall')-t0
mot=[e for e in events if e['kind']=='motor' and e['wall_ns']/1e9>=t0]
mt=arr(mot,'wall_ns')/1e9-t0
st=arr(states,'wall_ns')/1e9-t0
at=arr(support,'wall_ns')/1e9-t0
plt.rcParams.update({'font.size':8})

def plot(path, joints, bounds=None):
    fig,axes=plt.subplots(7,len(joints),figsize=(max(8,3.3*len(joints)),15),sharex=True,squeeze=False)
    for c,j in enumerate(joints):
        axes[0,c].set_title(f'J{j+1}')
        axes[0,c].plot(st,arr(states,'q')[:,j],label='q')
        axes[0,c].plot(wt,arr(tx,'position')[:,j],label='wire q_ref',ls='--',lw=.8)
        axes[1,c].plot(st,arr(states,'dq')[:,j],lw=.6)
        axes[2,c].plot(pt,arr(pol,'gravity')[:,j],label='gravity')
        axes[2,c].plot(wt,arr(tx,'ff')[:,j],label='wire FF',ls='--')
        axes[3,c].plot(wt,arr(tx,'p')[:,j],label='Kp')
        axes[4,c].plot(wt,arr(tx,'d')[:,j],label='Kd')
        axes[5,c].plot(at,arr(support,'estimated_total_torque')[:,j],label='estimated total')
        axes[5,c].plot(at,arr(support,'estimated_pd_torque')[:,j],label='estimated PD',ls='--')
        axes[6,c].plot(mt,arr(mot,'current')[:,j],lw=.6)
        for ax in axes[:,c]:
            ax.grid(alpha=.2)
            if bounds:ax.set_xlim(*bounds)
            for e in logs:
                x=e['wall_ns']/1e9-t0
                if bounds and not bounds[0]<=x<=bounds[1]:continue
                msg=e['message'];color='red' if 'HARD_ABORT' in msg else 'orange' if 'WARNING' in msg else 'gray'
                ax.axvline(x,color=color,lw=.5,alpha=.6)
        for e in logs:
            x=e['wall_ns']/1e9-t0
            if bounds and not bounds[0]<=x<=bounds[1]:continue
            if 'WARNING' not in e['message'] and 'HANDOVER_START' not in e['message']:
                label=e['message'].replace('ACTIVE ','')
                if 'HARD_ABORT' in label:label='HARD_ABORT'
                if 'HOLD: operator' in label:label='HOLD'
                if label=='HANDOVER_GRAVITY_MATCH':label='HANDOVER / RAMP'
                axes[0,c].text(x,1,label,rotation=90,fontsize=6,clip_on=True,
                               transform=axes[0,c].get_xaxis_transform(),va='top')
        for r in (0,2,5):axes[r,c].legend(fontsize=6)
        axes[-1,c].set_xlabel('Seconds from first guidance command')
    for ax,label in zip(axes[:,0],('q / q_ref (rad)','dq (rad/s)','FF (nominal Nm)','Kp','Kd',
                                  'Estimated torque (Nm)','MotorState effort (nominal Nm)')):ax.set_ylabel(label)
    fig.tight_layout();fig.savefig(P/path,dpi=140);plt.close(fig)

plot('overview.png',list(range(6)))
for i,start in enumerate(starts):plot(f'trial{i+1}_j2_handover.png',[1],(start-t0-1,start-t0+12))
for i,e in enumerate(summary['faults']):
    x=e['wall_ns']/1e9-t0;plot(f'fault{i+1}_window.png',list(range(6)),(x-2,x+2))
if manual_start is not None:
    boundaries=[manual_start]+[e['wall_ns']/1e9 for e in summary['operator_results']]
    # Each operator answer ends its joint window and authorizes the next one.
    if summary['faults']:boundaries.append(summary['faults'][0]['wall_ns']/1e9+2)
    order=json.loads((P/'manual_joint_order.json').read_text())
    for i,(a,b) in enumerate(zip(boundaries,boundaries[1:])):
        if i >= len(order):
            break
        j=order[i]-1
        plot(f'j{j+1}_push_release.png',[j],(a-t0,b-t0))
if (P/'whole_arm_test_start.json').exists():
    a=json.loads((P/'whole_arm_test_start.json').read_text())['wall_ns']/1e9-t0
    plot('whole_arm_push_release.png',list(range(6)),(a,st[-1]))
print(json.dumps({k:v for k,v in summary.items() if k not in ('warnings','faults')},indent=2))
