#!/usr/bin/env python3
"""Read-only JointState/TF acceptance capture. Does not open serial or publish."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


def capture(duration, urdf):
    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()
    assert model.nq == model.nv == 6
    rclpy.init()
    node = rclpy.create_node('arm6_feedback_verifier')
    samples, transforms, errors = [], {}, []
    names = [f'joint{i}' for i in range(1, 7)]

    def stamp(header):
        return header.stamp.sec * 10**9 + header.stamp.nanosec

    def state(msg):
        arrays = [msg.position, msg.velocity, msg.effort]
        if msg.name != names or any(len(v) != 6 or not np.isfinite(v).all() for v in arrays):
            errors.append('invalid six-axis feedback')
            return
        samples.append(dict(stamp=stamp(msg.header), received=time.monotonic(),
                            position=list(msg.position), velocity=list(msg.velocity)))

    def tf(msg):
        for t in msg.transforms:
            transforms.setdefault(stamp(t.header), {})[t.child_frame_id] = t

    node.create_subscription(JointState, '/joint_states', state, 100)
    node.create_subscription(TFMessage, '/tf', tf, 100)
    start = time.monotonic()
    try:
        while time.monotonic() - start < duration:
            rclpy.spin_once(node, timeout_sec=.02)
        graph = dict(nodes=node.get_node_names(), joint_state_publishers=[
            x.node_name for x in node.get_publishers_info_by_topic('/joint_states')],
            motor_control_publishers=[x.node_name for x in
                                      node.get_publishers_info_by_topic('/student/joint_effort_command')])
    finally:
        node.destroy_node()
        rclpy.shutdown()
    discrepancies = []
    for s in samples:
        chain = transforms.get(s['stamp'], {})
        if any(f'link{i}' not in chain for i in range(1, 7)):
            continue
        tf_ee = pin.SE3.Identity()
        for i in range(1, 7):
            t = chain[f'link{i}']
            expected_parent = 'base_link' if i == 1 else f'link{i-1}'
            if t.header.frame_id != expected_parent:
                errors.append('unexpected TF parent')
            v, q = t.transform.translation, t.transform.rotation
            tf_ee = tf_ee * pin.SE3(pin.Quaternion(q.w, q.x, q.y, q.z),
                                   np.array([v.x, v.y, v.z]))
        pin.forwardKinematics(model, data, np.array(s['position']))
        pin.updateFramePlacements(model, data)
        expected = data.oMf[model.getFrameId('link6')]
        discrepancies.append([float(np.linalg.norm(tf_ee.translation - expected.translation)),
                              float(np.linalg.norm(pin.log3(tf_ee.rotation.T @ expected.rotation)))])
    q = np.array([s['position'] for s in samples])
    dt = np.diff([s['received'] for s in samples])
    passed = bool(len(samples) > 10 and discrepancies and not errors and
                  graph['joint_state_publishers'] == ['student_arm_node'] and
                  np.max(discrepancies) < 1e-6)
    return dict(passed=passed, sample_count=len(samples), matched_tf_count=len(discrepancies),
                measured_rate_hz=float(1 / np.mean(dt)) if len(dt) else None,
                max_receive_gap_sec=float(np.max(dt)) if len(dt) else None,
                first_q=q[0].tolist() if len(q) else None,
                last_q=q[-1].tolist() if len(q) else None,
                observed_q_span=np.ptp(q, axis=0).tolist() if len(q) else None,
                max_fk_tf_error=np.max(discrepancies, axis=0).tolist() if discrepancies else None,
                graph=graph, errors=errors,
                manual_motion_observed_by_operator=False,
                note='TF agrees with encoder-based URDF; physical calibration and hand-guidance are not certified')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=8.)
    parser.add_argument('--urdf', type=Path, required=True,
                        help='Visualization URDF; default-offset run uses arm6.urdf')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = capture(args.duration, args.urdf)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
