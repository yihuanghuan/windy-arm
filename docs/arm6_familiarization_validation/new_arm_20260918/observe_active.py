"""Acceptance instrumentation: ROS subscriptions and authorized stop service only.

No serial access, motor command publisher, controller or parameter changes.
Stops on fault, explicit stop file, or a bounded 180-second active interval.
"""
import json
import os
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
from dummy_interface.msg import MotorControl, MotorState
from rcl_interfaces.msg import Log
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

OUT = Path(os.environ.get('ARM6_ACCEPTANCE_OUT', str(Path(__file__).resolve().parent)))
ROOT = Path(__file__).resolve().parents[3]
model = pin.buildModelFromUrdf(str(ROOT / 'src/arm-platform/config/arm6.urdf'))
data = model.createData()
rclpy.init()
node = rclpy.create_node('arm6_new_arm_acceptance_observer')
stream = (OUT / 'active_events.jsonl').open('x')
started = time.monotonic()
active = guide = fault_time = last_received = None
fault = last_state = last_cmd = None
phase = 'WAITING'
static = []
static_written = False


def record(kind, **values):
    stream.write(json.dumps(dict(t=time.monotonic()-started,
                                wall_ns=time.time_ns(), kind=kind, phase=phase,
                                **values))+'\n')
    stream.flush()


def state(msg):
    global last_state, last_received
    last_received = time.monotonic()
    q, dq = list(msg.position), list(msg.velocity)
    valid = len(q) == len(dq) == 6 and np.isfinite(q+dq).all()
    g = pin.computeGeneralizedGravity(model, data, np.asarray(q)).tolist() if valid else None
    last_state = dict(q=q, dq=dq, effort=list(msg.effort), gravity=g,
                      source_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
    record('state', **last_state)
    if guide is not None and last_received-guide <= 10 and fault is None:
        static.append(dict(t=last_received-guide, **last_state))


def motor(msg):
    record('motor', **{k: list(getattr(msg, k)) for k in
                      ('position', 'velocity', 'current', 'voltage', 'temperature')})


def command(msg):
    global active, guide, last_cmd, phase
    now = time.monotonic()
    if msg.enable and active is None:
        active = now
    if msg.enable and len(msg.p) == 6 and max(abs(x) for x in msg.p) < 1e-8:
        if guide is None:
            guide = now
        if fault is None:
            phase = 'GUIDE'
    last_cmd = {k: list(getattr(msg, k)) for k in ('position', 'velocity', 'current', 'p', 'd')}
    last_cmd['enable'] = msg.enable
    record('command', **last_cmd)


def log(msg):
    global phase, fault, fault_time
    if msg.name not in ('student_arm_node', 'arm6_manual_guidance'):
        return
    record('log', name=msg.name, level=msg.level, message=msg.msg)
    if 'ACTIVE ' in msg.msg and fault is None:
        phase = msg.msg.split('ACTIVE ', 1)[1]
    if 'GUIDANCE STOPPED: operator stop' in msg.msg:
        phase = 'OPERATOR_HOLD'
        return
    if fault is None and any(s in msg.msg for s in
                            ('GUIDANCE STOPPED', 'fault latched', 'command timeout')):
        fault, fault_time = msg.msg, time.monotonic()
        record('fault_snapshot', reason=fault, state=last_state, command=last_cmd)
        phase = 'FAULT'


node.create_subscription(JointState, '/joint_states', state, 100)
node.create_subscription(MotorState, '/student/joint_feedback', motor, 100)
node.create_subscription(MotorControl, '/student/joint_effort_command', command, 100)
node.create_subscription(Log, '/rosout', log, 100)
stop = node.create_client(Trigger, '/arm6_manual_guidance/stop')
reason = None
try:
    while True:
        rclpy.spin_once(node, timeout_sec=.01)
        now = time.monotonic()
        status = dict(phase=phase, elapsed=now-started,
                      active_sec=now-active if active else None,
                      guide_sec=now-guide if guide else None,
                      fault=fault, state=last_state, command=last_cmd)
        (OUT / 'active_status.json').write_text(json.dumps(status, indent=2)+'\n')
        if guide and now-guide >= 10 and not static_written and len(static) > 1:
            q = np.asarray([x['q'] for x in static]); dq = np.asarray([x['dq'] for x in static])
            result = dict(duration=static[-1]['t']-static[0]['t'], samples=len(static),
                          delta_q=(q[-1]-q[0]).tolist(), max_abs_dq=np.max(abs(dq), axis=0).tolist(),
                          q_range=np.ptp(q, axis=0).tolist())
            (OUT / 'no_contact_static.json').write_text(json.dumps(result, indent=2)+'\n')
            record('static_complete', result=result)
            static_written = True
        if fault_time and now-fault_time >= 2.5:
            reason = 'fault: '+fault
        elif (OUT / 'stop_active').exists():
            reason = 'operator acceptance stop'
        elif active and now-active >= 180:
            reason = '180-second active observation deadline'
        elif not active and now-started >= 60:
            reason = '60-second startup deadline'
        elif active and last_received and now-last_received > .1:
            reason = 'observer feedback stale'
        if reason:
            break
    record('stop_request', reason=reason)
    if stop.wait_for_service(timeout_sec=1.):
        future = stop.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.)
        record('stop_response', received=future.done(),
               message=future.result().message if future.done() and future.result() else None)
    until = time.monotonic()+2
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.01)
finally:
    (OUT / 'active_result.json').write_text(json.dumps(dict(
        stop_reason=reason, fault=fault, guide_reached=guide is not None,
        static_complete=static_written, last_state=last_state), indent=2)+'\n')
    stream.close()
    node.destroy_node()
    rclpy.shutdown()
