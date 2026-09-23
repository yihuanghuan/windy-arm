"""ROS-only acceptance recorder. No serial open or MotorControl publisher."""
import json
import time
from pathlib import Path
import rclpy
from sensor_msgs.msg import JointState
from dummy_interface.msg import MotorState, MotorControl
from std_msgs.msg import String
from rcl_interfaces.msg import Log

OUT=Path(__file__).resolve().parent
rclpy.init(); node=rclpy.create_node('arm6_bumpless_acceptance_recorder')
stream=(OUT/'events.jsonl').open('x')
started=time.monotonic(); phase='HOLD'; trial=0; fault=None; fault_at=None
state=command=support=policy=speed=None; completed=0; latest_write=0.

def record(kind,**data):
    stream.write(json.dumps(dict(t=time.monotonic()-started,wall_ns=time.time_ns(),
                                kind=kind,phase=phase,trial=trial,**data))+'\n')
    stream.flush()

def on_state(m):
    global state
    state=dict(q=list(m.position),dq=list(m.velocity),effort=list(m.effort))
    record('state',**state)

def on_command(m):
    global command
    command={k:list(getattr(m,k)) for k in ('position','velocity','current','p','d')}
    command['enable']=m.enable
    record('command',**command)

def on_applied(m):
    record('applied',**{k:list(getattr(m,k)) for k in ('position','velocity','current','p','d')})

def on_support(m):
    global support,completed
    support=json.loads(m.data);completed=support['completed_handovers']
    record('support',**support)

def on_policy(m):
    global policy
    policy=json.loads(m.data)
    payload={k:v for k,v in policy.items() if k!='phase'}
    record('policy',policy_phase=policy['phase'],**payload)

def on_speed(m):
    global speed
    speed=json.loads(m.data);record('speed',status=speed)

def on_log(m):
    global phase,trial,fault,fault_at
    if m.name not in ('student_arm_node','arm6_manual_guidance'):return
    record('log',name=m.name,message=m.msg,level=m.level)
    if m.name=='arm6_manual_guidance' and m.msg.startswith('ACTIVE '):
        phase=m.msg.split('ACTIVE ',1)[1]
        if phase=='HANDOVER_START':trial+=1
    if 'HOLD: operator' in m.msg:phase='HOLD'
    if fault is None and any(s in m.msg for s in ('HARD_ABORT','fault latched','GUIDANCE STOPPED','command timeout')):
        fault=m.msg;fault_at=time.monotonic()
        record('fault_snapshot',reason=fault,state=state,command=command,support=support,policy=policy,speed=speed)
        phase='FAULT'

node.create_subscription(JointState,'/joint_states',on_state,100)
node.create_subscription(MotorState,'/student/joint_feedback',lambda m:record('motor',**{
    k:list(getattr(m,k)) for k in ('position','velocity','current','temperature')}),100)
node.create_subscription(MotorControl,'/student/joint_effort_command',on_command,100)
node.create_subscription(MotorControl,'/student/applied_motor_command',on_applied,100)
node.create_subscription(String,'/student/support_diagnostics',on_support,100)
node.create_subscription(String,'/arm6_manual_guidance/support_diagnostics',on_policy,100)
node.create_subscription(String,'/student/speed_safety',on_speed,100)
node.create_subscription(Log,'/rosout',on_log,100)
try:
    while time.monotonic()-started<1800:
        rclpy.spin_once(node,timeout_sec=.01)
        now=time.monotonic()
        if now-latest_write>.05:
            status=dict(elapsed=now-started,phase=phase,trial=trial,completed=completed,
                        fault=fault,state=state,support=support,policy=policy,speed=speed)
            temp=OUT/'status.tmp';temp.write_text(json.dumps(status,indent=2)+'\n')
            temp.replace(OUT/'status.json');latest_write=now
        if (OUT/'finish_recording').exists() or (fault_at is not None and now-fault_at>=5):break
finally:
    stream.close();node.destroy_node();rclpy.shutdown()
