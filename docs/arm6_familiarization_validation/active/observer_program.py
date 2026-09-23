import json,time
from pathlib import Path
import numpy as np
import rclpy
from dummy_interface.msg import MotorControl, MotorState
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import Log
from std_srvs.srv import Trigger

out=Path('docs/arm6_familiarization_validation/active')
rclpy.init()
n=rclpy.create_node('arm6_guidance_acceptance_observer')
f=(out/'events.jsonl').open('w')
t0=time.monotonic(); start=None;guide=None;fault=None;q0=None;lastq=None;lastv=None;laststate=None;cmdcount=0

def record(kind,data):
    f.write(json.dumps(dict(t=time.monotonic()-t0,kind=kind,**data))+'\n');f.flush()

def state(m):
    global q0,lastq,lastv,laststate,fault
    lastq=list(m.position);lastv=list(m.velocity);laststate=time.monotonic()
    if len(lastq)!=6 or len(lastv)!=6 or not np.isfinite(lastq+lastv).all():fault='invalid feedback'
    if q0 is None:q0=lastq
    record('state',dict(q=lastq,dq=lastv,effort=list(m.effort)))

def cmd(m):
    global start,guide,cmdcount
    now=time.monotonic();cmdcount+=1
    if start is None and m.enable:start=now
    if m.enable and len(m.p)==6 and max(abs(v) for v in m.p)<1e-8 and guide is None:guide=now
    record('command',{k:list(getattr(m,k)) for k in ['position','velocity','current','p','d']}|dict(enable=m.enable))

def log(m):
    global fault
    if m.name in ('student_arm_node','arm6_manual_guidance'):
        record('log',dict(name=m.name,level=m.level,message=m.msg))
        if 'GUIDANCE STOPPED' in m.msg or 'fault latched' in m.msg or 'command timeout' in m.msg:
            fault=m.msg

n.create_subscription(JointState,'/joint_states',state,100)
n.create_subscription(MotorControl,'/student/joint_effort_command',cmd,100)
n.create_subscription(Log,'/rosout',log,100)
cli=n.create_client(Trigger,'/arm6_manual_guidance/stop')
stop_reason=None
try:
    while time.monotonic()-t0<40:
        rclpy.spin_once(n,timeout_sec=.01)
        now=time.monotonic()
        status=dict(elapsed=now-t0,command_count=cmdcount,active_sec=now-start if start else None,guide_sec=now-guide if guide else None,fault=fault,q=lastq,dq=lastv)
        (out/'status.json').write_text(json.dumps(status,indent=2)+'\n')
        if fault:stop_reason='guard: '+fault;break
        if start and (laststate is None or now-laststate>.1):stop_reason='observer feedback timeout';break
        if guide and now-guide>=14:stop_reason='bounded 14-second GUIDE observation completed';break
    if stop_reason is None:stop_reason='observer startup/transition timeout'
    record('stop_request',dict(reason=stop_reason))
    if cli.wait_for_service(timeout_sec=1.):
        future=cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(n,future,timeout_sec=1.)
        record('stop_response',dict(received=future.done(),message=future.result().message if future.done() and future.result() else None))
    end=time.monotonic()+2
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
finally:
    result=dict(stop_reason=stop_reason,fault=fault,guide_reached=guide is not None,command_count=cmdcount,q_start=q0,q_end=lastq,hardware_disable_claimed=False)
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)
    f.close();n.destroy_node();rclpy.shutdown()
