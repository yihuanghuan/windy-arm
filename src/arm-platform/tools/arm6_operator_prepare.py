#!/usr/bin/env python3
"""Prepare operator files using ROS only; never publishes commands or opens serial.

Without --capture-hold, only generate a guidance parameter file. With it, capture
the driver's actual stationary HOLD for an explicitly requested warm restart.
The snapshot is invalid after power loss and expires after 120 seconds.
"""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from task_space_smc.core.manual_guidance import Guidance, NAMES


def capture(policy, port):
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node('arm6_operator_hold_capture')
    latest = {}
    rows = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

    def joint(msg):
        age = node.get_clock().now().nanoseconds * 1e-9 - (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if tuple(msg.name) != NAMES or not -.01 <= age <= policy.timeout:
            latest.pop('joint', None)
            return
        latest['joint'] = (time.monotonic(), np.asarray(msg.position))

    def support(msg):
        rows.append((time.monotonic(), json.loads(msg.data)))

    node.create_subscription(JointState, '/joint_states', joint, qos)
    node.create_subscription(String, '/student/support_diagnostics', support, qos)
    try:
        client = node.create_client(GetParameters, '/student_arm_node/get_parameters')
        if not client.wait_for_service(timeout_sec=3.):
            raise RuntimeError('hardware driver unavailable; no snapshot written')
        request = GetParameters.Request()
        request.names = ['arm_type', 'arm_version', 'port_name', 'read_only']
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
        response = future.result() if future.done() else None
        if response is None or len(response.values) != 4:
            raise RuntimeError('cannot verify driver')
        v = response.values
        if (v[0].string_value != 'a_l1' or v[1].string_value != 'gamma6' or
                Path(v[2].string_value).resolve() != Path(port).resolve() or v[3].bool_value):
            raise RuntimeError('requires the matching active gamma6 driver')
        rows.clear()
        start = time.monotonic()
        while time.monotonic() - start < 8.:
            rclpy.spin_once(node, timeout_sec=.01)
            if len(rows) >= 180 and rows[-1][0] - rows[0][0] >= 2.05:
                break
        if (len(rows) < 180 or rows[-1][0] - rows[0][0] < 1.9 or
                max(np.diff([t for t, _ in rows])) > policy.timeout or
                time.monotonic() - rows[-1][0] > policy.timeout or
                'joint' not in latest or time.monotonic() - latest['joint'][0] > policy.timeout):
            raise RuntimeError('need continuous fresh stationary feedback; no snapshot written')
        if any(node.count_publishers(topic) != 1 for topic in (
                '/joint_states', '/student/support_diagnostics', '/student/applied_motor_command')):
            raise RuntimeError('expected one real driver publisher per feedback topic')
        q, dq = [np.asarray([s[key] for _, s in rows]) for key in ('q', 'dq')]
        if any(a.shape != (len(rows), 6) or not np.isfinite(a).all() for a in (q, dq)):
            raise RuntimeError('six finite feedback axes required')
        if (np.max(np.ptp(q, axis=0)) > .002 or np.max(abs(dq)) > .2 or
                np.max(abs(dq[-1])) > .15 or
                np.any(q <= policy.lower + policy.margin) or np.any(q >= policy.upper - policy.margin)):
            raise RuntimeError('arm must be stationary inside unchanged joint margins')
        command = rows[-1][1]['command']
        if any(s['command'] != command for _, s in rows):
            raise RuntimeError('applied command changed; this is not a stationary HOLD')
        policy.set_hold_command(command)
        if (not np.array_equal(command['p'], policy.hold_p) or
                not np.array_equal(command['d'], policy.hold_d) or
                np.any(np.asarray(command['velocity']) != 0)):
            raise RuntimeError('requires actual HOLD gains, not GUIDE')
        total = (np.asarray(command['p']) * (np.asarray(command['position']) - q[-1])
                 - np.asarray(command['d']) * dq[-1] + command['current'])
        if (latest['joint'][1].shape != (6,) or
                np.max(abs(latest['joint'][1] - q[-1])) > .002 or
                np.max(abs(np.asarray(command['position']) - q[-1])) > .03 or
                np.any(abs(total) > policy.limit)):
            raise RuntimeError('HOLD snapshot position/support mismatch')
        return dict(port=str(Path(port).resolve()), captured_unix_sec=time.time(),
                    command=command, q=q[-1].tolist(),
                    source='live applied HOLD; ROS-only capture, not a fault reset')
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--capture-hold', action='store_true')
    parser.add_argument('--port')
    args = parser.parse_args()
    if args.capture_hold and (not args.port or not Path(args.port).exists()):
        parser.error('--capture-hold requires an existing verified --port')
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('manipulator'))
    cfg = yaml.safe_load((share / 'arm6_familiarization.yaml').read_text())
    policy = Guidance(share / 'arm6.urdf', cfg)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    cfg.update(urdf_path=str(share / 'arm6.urdf'), hardware_enabled=True, manual_push_test=True)
    (args.output_dir / 'guidance_params.yaml').write_text(
        yaml.safe_dump({'arm6_manual_guidance': {'ros__parameters': cfg}}))
    if args.capture_hold:
        seed = capture(policy, args.port)
        (args.output_dir / 'hold_seed.yaml').write_text(yaml.safe_dump(seed))
        print('HOLD captured. Use within 120 seconds; invalid after power loss.')
    print(args.output_dir.resolve())


if __name__ == '__main__':
    main()
