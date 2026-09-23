"""Offline acceptance analysis of ROS observations and actual serial writes."""
import json
import os
import re
import struct
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

p = Path(os.environ.get('ARM6_ACCEPTANCE_OUT', str(Path(__file__).resolve().parent)))
speed_cfg = json.loads((p/'speed_config.json').read_text())
events = [json.loads(line) for line in (p/'active_events.jsonl').read_text().splitlines()]
states = [x for x in events if x['kind'] == 'state']
commands = [x for x in events if x['kind'] == 'command' and x['enable']]
motors = [x for x in events if x['kind'] == 'motor']
faults = [x for x in events if x['kind'] == 'fault_snapshot']
fault = faults[0] if faults else next(x for x in events if x['kind'] == 'stop_request')
warnings = [x for x in events if x['kind'] == 'log' and 'SPEED WARNING' in x.get('message','')]
t0 = commands[0]['wall_ns']/1e9
tf = fault['wall_ns']/1e9
tx = []
pattern = re.compile(r'^\d+\s+([\d.]+)\s+write\(\d+, "(.*?)", (\d+)\)\s+=\s+(\d+)')
for line in (p/'serial_tx.strace').read_text().splitlines():
    m = pattern.match(line)
    if not m:
        continue
    stamp = float(m[1])
    if stamp > states[-1]['wall_ns']/1e9:
        continue
    b = bytes.fromhex(m[2].replace('\\x', ''))
    assert len(b) == int(m[3]) == int(m[4]) == 170
    result = dict(time=stamp-t0, wall=stamp)
    for index, key in enumerate(['position', 'velocity', 'ff', 'p', 'd']):
        frame = b[index*34:(index+1)*34]
        assert frame[:4] == bytes([0xfe, 0xaa, 0x10+index, 28])
        total = cumulative = 0
        for byte in frame[:32]:
            total = (total+byte) % 256
            cumulative = (cumulative+total) % 256
        assert frame[32:] == bytes([total, cumulative])
        v = np.array(struct.unpack('<7f', frame[4:32]))
        assert v[6] == 0 and np.isfinite(v).all()
        v = v[:6]
        if key in ('position', 'velocity', 'ff'):
            v *= [1, 1, 1, -1, -1, -1]
        result[key] = v.tolist()
    tx.append(result)
(p/'serial_tx_decoded.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in tx))

def array(rows, key):
    return np.asarray([x[key] for x in rows])

st = array(states, 'wall_ns')/1e9-t0
ct = array(commands, 'wall_ns')/1e9-t0
mt = array(motors, 'wall_ns')/1e9-t0
xt = array(tx, 'time')
q, dq, g = (array(states, k) for k in ('q', 'dq', 'gravity'))
fault_rel = tf-t0
idx = max(0, int(np.searchsorted(st, fault_rel, side='right')-1))
cmd_idx = max(0, int(np.searchsorted(ct, st[idx], side='right')-1))
wire_idx = max(0, int(np.searchsorted(xt, st[idx], side='right')-1))
motor_idx = int(np.argmin(abs(mt-st[idx])))
speed_events = [x for x in events if x['kind'] in ('python_speed','driver_speed')]
speed_faults = [x for x in speed_events if any(x['status'].get('fault_axes',[]))]
summary = dict(hard_abort=bool(faults), phase=fault['phase'], event_time=fault_rel,
               q=q[idx].tolist(), dq=dq[idx].tolist(), gravity=g[idx].tolist(),
               command=commands[cmd_idx], actual_wire=tx[wire_idx], motor=motors[motor_idx],
               speed_fault=speed_faults[0] if speed_faults else None,
               warnings=len(warnings), wire_batches_validated=len(tx),
               static=json.loads((p/'no_contact_static.json').read_text()) if (p/'no_contact_static.json').exists() else None)
guide_states = [x for x in states if x['phase']=='GUIDE']
if guide_states:
    guide_q = array(guide_states,'q')
    summary['guide_excursion_deg'] = np.rad2deg(np.ptp(guide_q,axis=0)).tolist()
    summary['guide_max_abs_dq'] = np.max(abs(array(guide_states,'dq')),axis=0).tolist()
    summary['guide_delta_q'] = (guide_q[-1]-guide_q[0]).tolist()
    summary['guide_duration'] = (guide_states[-1]['wall_ns']-guide_states[0]['wall_ns'])/1e9
(p/'active_analysis.json').write_text(json.dumps(summary, indent=2)+'\n')

plt.rcParams.update({'font.size': 8})
for zoom in ((False, True) if faults else (False,)):
    fig, axes = plt.subplots(5, 6, figsize=(19, 12), sharex=True)
    for j in range(6):
        axes[0,j].set_title(f'J{j+1}')
        axes[0,j].plot(st, q[:,j], lw=.9)
        axes[1,j].plot(st, dq[:,j], lw=.6)
        for bound, color, style in ((speed_cfg['guide_warning_speed'], 'orange', ':'),(speed_cfg['guide_sustained_abort_speed'], 'r', ':'),(speed_cfg['guide_instant_abort_speed'], 'r', '--')):
            for sign in (-1,1):
                axes[1,j].axhline(sign*bound, color=color, ls=style, lw=.6)
        axes[2,j].plot(st, g[:,j], label='model gravity', lw=1)
        axes[2,j].plot(ct, array(commands,'current')[:,j], label='ROS FF', lw=.8)
        axes[2,j].plot(xt, array(tx,'ff')[:,j], label='wire FF', ls='--', lw=.8)
        axes[3,j].plot(xt, array(tx,'p')[:,j], label='Kp')
        axes[3,j].plot(xt, array(tx,'d')[:,j], label='Kd', ls='--')
        axes[4,j].plot(mt, array(motors,'current')[:,j], label='MotorState current', lw=.6)
        axes[4,j].set_xlabel('Time from first ROS command (s)')
        for row in range(5):
            ax = axes[row,j]
            ax.axvline(fault_rel, color='red' if faults else 'green', lw=.9)
            for warning in warnings:
                ax.axvline(warning['wall_ns']/1e9-t0, color='orange', alpha=.25, lw=.5)
            ax.grid(alpha=.25)
            if zoom:
                ax.set_xlim(fault_rel-2, fault_rel+2)
            else:
                for edge in (3,6):
                    ax.axvline(edge, color='gray', ls=':', lw=.6)
    for ax,label in zip(axes[:,0], ['q (rad)','dq (rad/s)','Gravity / FF (Nm)','MIT gains','Measured effort (nominal Nm)']):
        ax.set_ylabel(label)
    for row in (2,3):
        axes[row,0].legend(fontsize=6)
    fig.suptitle('New ARM6: '+('fault ±2 seconds' if zoom else 'RAMP 0–3s / Kp withdrawal 3–6s / GUIDE; orange=WARNING, red=FAULT, green=STOP'))
    fig.tight_layout()
    fig.savefig(p/('fault_window.png' if zoom else 'active_overview.png'), dpi=150)
    plt.close(fig)
print(json.dumps(summary, indent=2))
