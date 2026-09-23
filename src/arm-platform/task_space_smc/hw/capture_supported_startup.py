"""Capture ROS feedback for supported startup; NEVER publishes motor commands.

The resulting file is a measured-effort estimate, not a previous-command dump.
An explicit supported launch is required to turn this proposal into a HOLD.
"""
import argparse
import json
from pathlib import Path
import time

import rclpy
from sensor_msgs.msg import JointState
import yaml
from task_space_smc.core.manual_guidance import Guidance, NAMES
from task_space_smc.core.supported_startup import propose_supported_hold


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--port',required=True)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    policy=Guidance(root/'config/arm6.urdf',yaml.safe_load((root/'config/arm6_familiarization.yaml').read_text()))
    rclpy.init();node=rclpy.create_node('supported_startup_feedback_capture')
    rows=[];started=time.monotonic()
    def callback(msg):
        if tuple(msg.name)!=NAMES:raise ValueError('wrong joint names')
        rows.append(dict(t=time.monotonic()-started,q=list(msg.position),
                         dq=list(msg.velocity),effort=list(msg.effort)))
    node.create_subscription(JointState,'/joint_states',callback,100)
    try:
        while time.monotonic()-started<8:
            rclpy.spin_once(node,timeout_sec=.01)
            if rows and rows[-1]['t']-rows[0]['t']>=2.05:break
        if not rows or time.monotonic()-started-rows[-1]['t']>policy.timeout:
            raise RuntimeError('no fresh feedback')
        proposal=propose_supported_hold(rows,policy)
        evidence=dict(port=args.port,captured_unix_sec=time.time(),samples=rows,proposal=proposal)
        with args.output.open('x') as out:yaml.safe_dump(evidence,out,sort_keys=False)
        print(json.dumps(proposal,indent=2))
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
