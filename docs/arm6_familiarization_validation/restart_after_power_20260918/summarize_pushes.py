"""Offline per-joint acceptance windows, bounded by operator responses."""
import json
from pathlib import Path
import numpy as np

P=Path(__file__).resolve().parent
rows=[json.loads(l) for l in (P/'events.jsonl').open()]
policy=[e for e in rows if e['kind']=='policy' and e['policy_phase']=='MANUAL_PUSH']
responses=[json.loads(l) for l in (P/'operator_results.jsonl').open()]
start=policy[0]['wall_ns'];results=[]
for response in responses:
    end=response['wall_ns'];j=response['joint']-1
    window=[e for e in policy if start<=e['wall_ns']<=end]
    if not window:continue
    release=[e for e in window if e['wall_ns']>=window[-1]['wall_ns']-3_000_000_000]
    q=np.array([e['q'] for e in window]);dq=np.array([e['dq'] for e in window])
    qr=np.array([e['q'] for e in release]);dqr=np.array([e['dq'] for e in release])
    results.append(dict(joint=j+1,operator=response['result'],start_wall_ns=start,end_wall_ns=end,
        q_initial_rad=q[0,j],q_final_rad=q[-1,j],travel_deg=float(np.rad2deg(np.ptp(q[:,j]))),
        net_travel_deg=float(np.rad2deg(q[-1,j]-q[0,j])),peak_abs_dq=float(np.max(abs(dq[:,j]))),
        last_three_seconds_q_span_rad=float(np.ptp(qr[:,j])),
        last_three_seconds_delta_q_rad=float(qr[-1,j]-qr[0,j]),
        last_three_seconds_peak_abs_dq=float(np.max(abs(dqr[:,j]))),
        kp_all_zero=bool(np.all(np.array([e['p'] for e in window])==0)),
        other_axes_span_deg=np.rad2deg(np.ptp(q,axis=0)).tolist()))
    start=end
(P/'per_joint_results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(results,ensure_ascii=False,indent=2))
