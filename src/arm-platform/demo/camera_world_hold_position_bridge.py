#!/usr/bin/env python3
"""Integrate the existing P loop's dq into the driver's authoritative MIT hold.

No kinematics or controller here. Keep the exact first command and its support
offset, gains and current. Motor velocity remains zero. Reuse the calibration
probe's handover, 100 Hz publisher, tracking checks and operator HOLD service.
Run only with the world controller's publish_command=false; --execute required.
"""
import argparse
import json
import signal
import time

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from camera_fk_motion_probe import Probe, motor_command
from camera_world_hold_core import ordered_joints

# Software ceiling; each trial selects a lower --speed. The real driver's
# existing speed/fault protections and the incremental tracking check remain.
MAX_INTEGRATION_RATE = .2


def integrate_position(reference, dq, dt, speed):
    reference, dq = np.asarray(reference, float), np.asarray(dq, float)
    if (reference.shape != (6,) or dq.shape != (6,) or
            not np.isfinite(np.r_[reference, dq, dt, speed]).all() or
            not 0 < dt <= .03 or not 0 < speed <= MAX_INTEGRATION_RATE):
        raise ValueError('invalid position integration input')
    dq = dq * min(1., speed / max(np.max(abs(dq)), 1e-15))
    return reference + dq * dt


class PositionBridge(Probe):
    def __init__(self, args):
        super().__init__(args)
        self.rate = None
        self.rate_time = -np.inf
        self.rate_error = None
        self.create_subscription(JointState, '/camera_world_hold/joint_velocity',
                                 self.on_rate, 1)
        self.target_pub = self.create_publisher(
            JointState, '/camera_world_hold/actuator_joint_command', 1)

    def on_rate(self, msg):
        try:
            age = (self.get_clock().now().nanoseconds * 1e-9 -
                   msg.header.stamp.sec - msg.header.stamp.nanosec * 1e-9)
            if not -.05 <= age <= .15:
                raise ValueError('stale integration rate')
            rate = ordered_joints(msg.name, msg.velocity, self.kin.names)
            if not np.isfinite(rate).all() or np.max(abs(rate)) > MAX_INTEGRATION_RATE+1e-6:
                raise ValueError('invalid or excessive integration rate')
            self.rate, self.rate_time = rate, time.monotonic()
        except ValueError as error:
            self.rate_error = str(error)

    def run_hold(self):
        self.spin(3.)
        self.check()
        if self.count_publishers('/student/joint_command'):
            raise RuntimeError('position command publisher already exists')
        if self.count_publishers('/camera_world_hold/joint_velocity') != 1:
            raise RuntimeError('requires exactly one P controller')
        if time.monotonic()-self.rate_time < .3:
            raise RuntimeError('disable controller HOLD before preparing bridge')
        recent = [q for t, q in self.rows if t > time.monotonic()-2.]
        if len(recent) < 100 or np.max(np.ptp(recent, axis=0)) > .003:
            raise RuntimeError('stationary joints required')
        if not self.args.execute:
            print('PREVIEW: inputs ready; no handover or motor publisher', flush=True)
            return
        if self.count_publishers('/student/joint_effort_command') != 1:
            raise RuntimeError('competing motor publisher')
        if self.applied is None:
            raise RuntimeError('authoritative hold unavailable')
        motor_command({k:list(getattr(self.applied,k))
                       for k in ('position','velocity','current','p','d')})
        self.seed = json.loads(self.call('/student_arm_node/prepare_guidance'))
        self.prepared = True
        self.initial_q = self.rows[-1][1].copy()
        self.command = motor_command(self.seed)
        self.active = True
        self.tick()  # Exact first command; no removal of existing PD support.
        self.spin(.1)
        print('BRIDGE READY: retaining authoritative hold; enable P controller now', flush=True)
        started = None
        waiting = previous = time.monotonic()
        while True:
            self.spin(.005)
            now = time.monotonic()
            self.check()
            if self.rate_error:
                raise RuntimeError(self.rate_error)
            if self.count_publishers('/student/joint_command'):
                raise RuntimeError('competing position controller')
            if started is None:
                if self.rate_time < waiting:
                    if now-waiting > 30.:
                        raise RuntimeError('enable wait expired')
                    previous = now
                    continue
                started = now
            if now-self.rate_time > .12:
                raise RuntimeError('P controller stopped/stale; driver HOLD')
            if self.args.duration > 0 and now-started >= self.args.duration:
                print('Bounded trial complete; returning to driver HOLD', flush=True)
                break
            target = integrate_position(self.command.position, self.rate,
                                        now-previous, self.args.speed)
            previous = now
            if np.max(abs(target-self.seed['position'])) > np.deg2rad(self.args.max_offset_deg):
                print('Bounded trial excursion reached; returning to driver HOLD', flush=True)
                break
            self.kin.check_q(target)
            if (np.min(target-self.kin.model.lowerPositionLimit) < .1 or
                    np.min(self.kin.model.upperPositionLimit-target) < .1):
                raise RuntimeError('target approaches existing joint margin')
            self.command.position = target.tolist()
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name, msg.position, msg.velocity = self.kin.names, target.tolist(), [0.]*6
            self.target_pub.publish(msg)
            self.record('integrated_position', q_cmd=target.tolist(), dq=self.rate.tolist())

    def close(self):
        # Keep the controller's visible state consistent with actual actuation.
        # Continue refreshing the last motor command while requesting disable;
        # Probe.close then hands it back to the driver's authoritative HOLD.
        try:
            if self.prepared:
                client = self.create_client(SetBool, '/camera_world_hold/enable')
                try:
                    if client.wait_for_service(timeout_sec=.2):
                        future = client.call_async(SetBool.Request(data=False))
                        rclpy.spin_until_future_complete(self, future, timeout_sec=.5)
                finally:
                    self.destroy_client(client)
        finally:
            # A failed UI/state service must never prevent hardware handover.
            super().close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--output', required=True)
    parser.add_argument('--speed', type=float, default=.03)
    parser.add_argument('--max-offset-deg', type=float, default=3.)
    parser.add_argument('--duration', type=float, default=20.,
                        help='trial seconds; 0 runs until explicit stop or a guard trips')
    args = parser.parse_args()
    if not (0 < args.speed <= MAX_INTEGRATION_RATE and 0 < args.max_offset_deg <= 60. and
            0 <= args.duration <= 180.):
        parser.error('speed <= .2, excursion <= 60 deg; duration 0 or up to 180 s')
    args.allow_uninitialized_slam = False
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PositionBridge(args)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_:setattr(node, 'interrupted', True))
    try:
        node.run_hold()
    finally:
        try:
            node.close()
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    main()
