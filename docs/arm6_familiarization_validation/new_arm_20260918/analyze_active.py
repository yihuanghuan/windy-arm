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
events = [json.loads(line) for line in (p/'active_events.jsonl').read_text().splitlines()]
states = [x for x in events if x['kind'] == 'state']
commands = [x for x in events if x['kind'] == 'command' and x['enable']]
motors = [x for x in events if x['kind'] == 'motor']
fault = next(x for x in events if x['kind'] == 'fault_snapshot')
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
overspeed = np.where(np.any(abs(dq) > .5, axis=1))[0]
idx = int(overspeed[0])
axis = int(np.argmax(abs(dq[idx])))
cmd_idx = max(0, int(np.searchsorted(ct, st[idx], side='right')-1))
wire_idx = max(0, int(np.searchsorted(xt, st[idx], side='right')-1))
motor_idx = int(np.argmin(abs(mt-st[idx])))
holds = [x for x in tx if x['time'] >= fault_rel-.05 and np.allclose(x['p'], [30]*3+[5]*3)]
hold_t = holds[0]['time']
before = [x for x in tx if x['time'] < hold_t][-1]
postmask = (st >= hold_t) & (st <= hold_t+2)
start_index = int(np.searchsorted(st, hold_t))
summary = dict(
    fault_time_active_sec=fault_rel, phase_before_fault=fault['phase'],
    first_overspeed=dict(time=st[idx], joint=axis+1, q=q[idx].tolist(), dq=dq[idx].tolist(),
                         gravity=g[idx].tolist(), command=commands[cmd_idx],
                         actual_last_wire_command=tx[wire_idx], motor_feedback=motors[motor_idx]),
    peak_abs_dq=np.max(abs(dq), axis=0).tolist(),
    initial_q=q[0].tolist(), final_q=q[-1].tolist(),
    last_wire_before_hold=before, first_fault_hold=holds[0],
    last_observed_hold=holds[-1],
    fallback_2sec_delta_q=(q[np.where(postmask)[0][-1]]-q[start_index]).tolist(),
    fallback_2sec_max_abs_dq=np.max(abs(dq[postmask]), axis=0).tolist(),
    fallback_ff_range=np.ptp(array(holds, 'ff'), axis=0).tolist(),
    wire_batches_validated=len(tx),
    no_contact=json.loads((p/'no_contact_static.json').read_text()))
(p/'active_analysis.json').write_text(json.dumps(summary, indent=2)+'\n')

plt.rcParams.update({'font.size': 8})
for zoom in (False, True):
    fig, axes = plt.subplots(5, 6, figsize=(19, 12), sharex=True)
    for j in range(6):
        axes[0,j].set_title(f'J{j+1}')
        axes[0,j].plot(st, q[:,j], lw=.9)
        axes[1,j].plot(st, dq[:,j], lw=.6)
        axes[1,j].axhline(.5, color='r', ls=':', lw=.7)
        axes[1,j].axhline(-.5, color='r', ls=':', lw=.7)
        axes[2,j].plot(st, g[:,j], label='model gravity', lw=1)
        axes[2,j].plot(ct, array(commands,'current')[:,j], label='ROS FF', lw=.8)
        axes[2,j].plot(xt, array(tx,'ff')[:,j], label='wire FF', ls='--', lw=.8)
        axes[3,j].plot(xt, array(tx,'p')[:,j], label='Kp')
        axes[3,j].plot(xt, array(tx,'d')[:,j], label='Kd', ls='--')
        axes[4,j].plot(mt, array(motors,'current')[:,j], label='MotorState current', lw=.6)
        axes[4,j].set_xlabel('Time from first ROS command (s)')
        for row in range(5):
            ax = axes[row,j]
            ax.axvline(fault_rel, color='red', lw=.9)
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
    fig.suptitle('New ARM6: '+('fault ±2 seconds' if zoom else 'RAMP 0–3s / Kp withdrawal 3–6s / GUIDE / fault hold'))
    fig.tight_layout()
    fig.savefig(p/('fault_window.png' if zoom else 'active_overview.png'), dpi=150)
    plt.close(fig)
print(json.dumps(summary, indent=2))
