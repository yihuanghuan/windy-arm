"""Offline proposal for a supported cold start, NOT an old-command snapshot.

MotorState effort is in the driver's nominal torque units. Matching its median
is only an estimate of previous support; external support is required until a
new actual HOLD has been established and inspected. Gravity is not modified.
"""
import numpy as np


def propose_supported_hold(samples, policy):
    if len(samples) < 180:
        raise ValueError('need two seconds of continuous six-axis feedback')
    t = np.asarray([s['t'] for s in samples], dtype=float)
    q, dq, effort = [np.asarray([s[k] for s in samples], dtype=float)
                     for k in ('q', 'dq', 'effort')]
    if any(a.shape != (len(samples), 6) or not np.isfinite(a).all()
           for a in (q, dq, effort)) or not np.isfinite(t).all():
        raise ValueError('six finite axes required')
    if t[-1]-t[0] < 1.9 or np.any(np.diff(t) <= 0) or np.max(np.diff(t)) > policy.timeout:
        raise ValueError('feedback window is short, nonmonotonic or stale')
    if np.any(q <= policy.lower+policy.margin) or np.any(q >= policy.upper-policy.margin):
        raise ValueError('feedback outside unchanged joint margin')
    if np.max(np.ptp(q, axis=0)) > .002 or np.max(abs(dq)) > .2:
        raise ValueError('supported startup requires stationary feedback')
    if np.max(abs(dq[-1])) > .15:
        raise ValueError('last velocity exceeds startup gate')
    if np.any(abs(effort) > .5*policy.limit) or np.any(np.ptp(effort,axis=0) > .2*policy.limit):
        raise ValueError('effort estimate too large or inconsistent')
    estimated = np.median(effort, axis=0)
    reference = q[-1] + (estimated+policy.hold_d*dq[-1])/policy.hold_p
    if np.any(reference <= policy.lower+policy.margin) or np.any(reference >= policy.upper-policy.margin):
        raise ValueError('proposed reference outside unchanged joint margin')
    command = dict(position=reference.tolist(), velocity=[0.]*6, current=[0.]*6,
                   p=policy.hold_p.tolist(), d=policy.hold_d.tolist())
    policy.set_hold_command(command)
    return dict(command=command, q=q[-1].tolist(), estimated_support=estimated.tolist(),
                source='supported_effort_estimate', support_is_command_readback=False)
