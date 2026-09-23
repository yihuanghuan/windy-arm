"""Read-only startup-fault analysis. No ROS or hardware access."""
import json,re,struct
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=Path(__file__).resolve().parent
fault=1789720979.511725458
rows=[json.loads(l) for l in (P/'events.jsonl').open()]
state=[r for r in rows if r['kind']=='state' and r['wall_ns']/1e9>=fault-1]
support=[r for r in rows if r['kind']=='support' and r['wall_ns']/1e9>=fault-1]
pat=re.compile(r'^\d+\s+([\d.]+)\s+write\(\d+, "(.*?)", (\d+)\)\s+=\s+(\d+)')
tx=[]
for line in (P/'serial_tx.strace').open():
 m=pat.match(line)
 if not m or float(m[1])>rows[-1]['wall_ns']/1e9:continue
 b=bytes.fromhex(m[2].replace('\\x',''));assert len(b)==int(m[3])==int(m[4])==170
 r={'wall':float(m[1])}
 for k,key in enumerate(('q_ref','velocity','ff','kp','kd')):
  f=b[k*34:(k+1)*34];assert f[:4]==bytes([254,170,16+k,28])
  a=c=0
  for v in f[:32]:a=(a+v)%256;c=(c+a)%256
  assert f[32:]==bytes([a,c]);v=np.array(struct.unpack('<7f',f[4:32]));assert v[6]==0 and np.isfinite(v).all()
  v=v[:6]
  if k<3:v*= [1,1,1,-1,-1,-1]
  r[key]=v.tolist()
 tx.append(r)
first=tx[0];pre=[r for r in tx if r['wall']<fault];post=[r for r in tx if r['wall']>=fault]
q=np.array([r['q'] for r in state]);dq=np.array([r['dq'] for r in state]);times=np.array([r['wall'] for r in tx])
summary=dict(fault_wall=fault,first_wire=first,last_prefault_wire=pre[-1],first_postfault_wire=post[0],last_wire=tx[-1],
 validated_batches=len(tx),max_wire_gap_sec=float(max(np.diff(times))),first_recorded_state_wall=state[0]['wall_ns']/1e9,
 post_fault_q_span_rad=np.ptp(q,axis=0).tolist(),post_fault_peak_abs_dq=np.max(abs(dq),axis=0).tolist(),
 final_q=q[-1].tolist(),last_support=support[-1],completed_handovers=0,guide_entered=False)
(P/'startup_analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
(P/'fault_window.json').write_text(json.dumps([r for r in rows if fault-2<=r['wall_ns']/1e9<=fault+2],indent=2)+'\n')
fig,ax=plt.subplots(5,6,figsize=(19,12),sharex=True)
st=np.array([r['wall_ns']/1e9-fault for r in state]);wt=times-fault
for j in range(6):
 ax[0,j].set_title(f'J{j+1}')
 ax[0,j].plot(st,q[:,j],label='q');ax[0,j].plot(wt,[r['q_ref'][j] for r in tx],label='wire q_ref',ls='--')
 ax[1,j].plot(st,dq[:,j]);ax[2,j].plot(wt,[r['ff'][j] for r in tx]);ax[3,j].plot(wt,[r['kp'][j] for r in tx]);ax[4,j].plot(wt,[r['kd'][j] for r in tx])
 ax[0,j].legend(fontsize=7)
 for a in ax[:,j]:a.axvline(0,color='red');a.grid(alpha=.3);a.set_xlim(-.5,3)
 ax[4,j].set_xlabel('Seconds from startup fault')
for a,label in zip(ax[:,0],['q / q_ref (rad)','dq (rad/s)','FF (nominal Nm)','Kp','Kd']):a.set_ylabel(label)
fig.suptitle('Fault in startup HOLD; guidance never started. ROS feedback record begins after fault.');fig.tight_layout();fig.savefig(P/'startup_fault.png',dpi=120)
print(json.dumps({k:v for k,v in summary.items() if k not in ('last_support','first_wire','last_prefault_wire','first_postfault_wire','last_wire')},indent=2))
