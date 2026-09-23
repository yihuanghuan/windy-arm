"""Read-only recovery observer. No command publisher or serial access."""
import json
import time
from pathlib import Path
import rclpy
from sensor_msgs.msg import JointState

P=Path(__file__).resolve().parent
rclpy.init();node=rclpy.create_node('arm6_supported_recovery_observer')
started=time.monotonic();last=None;state=None;count=0
out=(P/'feedback.jsonl').open('x')

def received(m):
    global last,state,count
    last=time.monotonic();count+=1
    state=dict(wall_ns=time.time_ns(),t=last-started,names=list(m.name),
               q=list(m.position),dq=list(m.velocity),effort=list(m.effort))
    out.write(json.dumps(state)+'\n');out.flush()

node.create_subscription(JointState,'/joint_states',received,100)
written=0.
try:
    while time.monotonic()-started<1800 and not (P/'finish_observer').exists():
        rclpy.spin_once(node,timeout_sec=.02)
        now=time.monotonic()
        if now-written>.2:
            s=dict(wall_ns=time.time_ns(),age_sec=None if last is None else now-last,
                   count=count,state=state)
            tmp=P/'status.tmp';tmp.write_text(json.dumps(s,indent=2)+'\n')
            tmp.replace(P/'status.json');written=now
finally:
    out.close();node.destroy_node();rclpy.shutdown()
