#!/usr/bin/env python3
"""Small static-pose tests using the driver's existing MIT position hold.

Preserves the authoritative hold gains/current and exact first handover command.
No camera feedback control or automatic recovery. Optional slow transitions use
the gains already defined by student_arm6's position profile.
Defaults to preview. An onsite operator must clear the workspace before --execute.
"""
import argparse
from collections import deque
import json
from pathlib import Path
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from dummy_interface.msg import MotorControl
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, String, UInt64
from std_srvs.srv import Trigger

from camera_world_hold_core import CameraKinematics, default_urdf, ordered_joints


def checked_offsets(plan, maximum_deg):
    offsets = np.asarray([p['offset_deg'] for p in plan], dtype=float)
    if offsets.ndim != 2 or offsets.shape[1] != 6 or not len(offsets):
        raise ValueError('plan requires six offsets for every waypoint')
    if not np.isfinite(offsets).all() or np.max(abs(offsets)) > maximum_deg:
        raise ValueError('plan exceeds finite joint excursion bound')
    for p in plan:
        if not np.isfinite(p.get('hold_seconds', 4.)) or p.get('hold_seconds', 4.) < 3.:
            raise ValueError('settled capture requires at least three seconds')
    return np.deg2rad(offsets)


def blend(start, end, elapsed, speed):
    # Cubic position ramp, zero endpoint velocities, bounded peak slope.
    duration = max(1., 1.5 * np.max(abs(end-start)) / speed)
    u = float(np.clip(elapsed/duration, 0., 1.))
    return start+(end-start)*(3*u*u-2*u*u*u), elapsed >= duration


def motor_command(seed):
    message = MotorControl(enable=1)
    for key in ('position','velocity','current','p','d'):
        values = np.asarray(seed[key], dtype=float)
        if values.shape != (6,) or not np.isfinite(values).all():
            raise ValueError('invalid authoritative hold field: '+key)
        setattr(message, key, values.tolist())
    return message


def incremental_tracking_error(q, command, initial_q, initial_command):
    # The authoritative hold can deliberately contain a static PD offset.
    # Keep that load-supporting offset; monitor only newly introduced mismatch.
    return (np.asarray(command)-initial_command)-(np.asarray(q)-initial_q)


def preserve_support_reference(anchor, reference, old_p, new_p):
    """Keep the existing static PD effort at a fixed measured anchor."""
    anchor, reference, old_p, new_p = (
        np.asarray(x, float) for x in (anchor, reference, old_p, new_p))
    if (any(x.shape != (6,) for x in (anchor, reference, old_p, new_p)) or
            not np.isfinite(np.r_[anchor, reference, old_p, new_p]).all() or
            np.any(old_p <= 0) or np.any(new_p <= 0)):
        raise ValueError('invalid support-preserving gain transition')
    return anchor + (reference-anchor)*old_p/new_p


class Probe(Node):
    def __init__(self, args):
        super().__init__('camera_fk_motion_probe')
        self.args = args
        self.kin = CameraKinematics(default_urdf(), 'joint6_housing')
        self.rows = deque(maxlen=500)
        self.pose_time = self.valid_time = -np.inf
        self.valid = False
        self.fault = None
        self.seed = self.command = None
        self.initial_q = None
        self.active = False
        self.prepared = False
        self.interrupted = False
        self.applied = None
        self.path = Path(args.output)
        self.path.mkdir(parents=True, exist_ok=False)
        self.log = (self.path/'streams.jsonl').open('w')
        self.create_subscription(JointState, '/joint_states', self.on_joint, 10)
        self.create_subscription(PoseStamped, '/slam/camera_pose', self.on_pose, 10)
        self.create_subscription(PoseStamped, '/slam/camera_pose_raw', lambda m:self.on_pose(m, 'camera_raw'), 10)
        self.create_subscription(UInt64, '/slam/map_epoch', lambda m:self.record('map_epoch', value=m.data), 10)
        self.create_subscription(Imu, '/camera/camera/gyro/sample', self.on_gyro,
                                 rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Bool, '/slam/state_valid', self.on_valid, 10)
        self.create_subscription(String, '/student/speed_safety', self.on_safety, 10)
        self.create_subscription(MotorControl, '/student/applied_motor_command', self.on_applied, 10)
        self.pub = self.create_publisher(MotorControl, '/student/joint_effort_command', 1) if args.execute else None
        self.create_timer(.01, self.tick)

    def record(self, kind, **data):
        self.log.write(json.dumps(dict(kind=kind, receive=time.monotonic(), **data))+'\n')

    def on_joint(self, m):
        try:
            q = self.kin.check_q(ordered_joints(m.name, m.position, self.kin.names))
            self.rows.append((time.monotonic(), q))
            self.record('joint', stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                        q=q.tolist(), dq=list(m.velocity))
        except ValueError as e:
            self.fault = str(e)

    def on_pose(self, m, kind='camera'):
        if kind == 'camera':
            self.pose_time = time.monotonic()
        p, q = m.pose.position, m.pose.orientation
        self.record(kind, stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                    p=[p.x,p.y,p.z], xyzw=[q.x,q.y,q.z,q.w])

    def on_valid(self, m):
        self.valid, self.valid_time = m.data, time.monotonic()
        self.record('valid', value=m.data)

    def on_gyro(self, m):
        w=m.angular_velocity
        self.record('gyro', stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                    w=[w.x,w.y,w.z])

    def on_safety(self, m):
        s = json.loads(m.data)
        if any(s.get('fault_axes', [])) or s.get('phase') == 'FAULT':
            self.fault = 'driver fault: '+m.data

    def on_applied(self, m):
        self.applied = m

    def tick(self):
        if self.active and self.command is not None:
            self.command.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.command)

    def spin(self, duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.005)

    def check(self, require_slam=True):
        if self.interrupted:
            raise RuntimeError('operator interrupt')
        if self.fault:
            raise RuntimeError(self.fault)
        now = time.monotonic()
        if not self.rows or now-self.rows[-1][0] > .1:
            raise RuntimeError('joint feedback stale')
        if require_slam and not self.args.allow_uninitialized_slam:
            if not self.valid or now-self.pose_time > .20 or now-self.valid_time > .25:
                raise RuntimeError('SLAM unavailable; returning to driver hold')
        if self.active and self.count_publishers('/student/joint_effort_command') != 1:
            raise RuntimeError('competing motor command publisher')
        if self.active and np.max(abs(incremental_tracking_error(
                self.rows[-1][1],self.command.position,self.initial_q,np.array(self.seed['position'])))) > .08:
            self.record('tracking_error', q=self.rows[-1][1].tolist(), q_cmd=list(self.command.position))
            raise RuntimeError('joint tracking error exceeds 0.08 rad')

    def call(self, name):
        client = self.create_client(Trigger, name)
        if not client.wait_for_service(timeout_sec=2.):
            raise RuntimeError('service unavailable: '+name)
        f = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, f, timeout_sec=3.)
        self.destroy_client(client)
        if not f.done() or f.result() is None or not f.result().success:
            raise RuntimeError(name+': '+(str(f.result()) if f.done() else 'timeout'))
        return f.result().message

    def run(self, plan):
        self.spin(3.)
        self.check()
        recent = [q for t,q in self.rows if t > time.monotonic()-2.]
        if len(recent) < 100 or np.max(np.ptp(recent, axis=0)) > .003:
            raise RuntimeError('two stationary seconds required before handover')
        if self.applied is None:
            raise RuntimeError('authoritative driver hold unavailable')
        # Validate serialization and the current hold before requesting the
        # driver's timed handover. JSON may represent exact zeros as integers.
        motor_command({k:list(getattr(self.applied,k)) for k in ('position','velocity','current','p','d')})
        offsets = checked_offsets(plan, self.args.max_offset_deg)
        q0 = self.rows[-1][1].copy()
        for d in offsets:
            q = self.kin.check_q(q0+d)
            if np.min(q-self.kin.model.lowerPositionLimit) < .1 or np.min(self.kin.model.upperPositionLimit-q) < .1:
                raise ValueError('planned pose too close to joint limits')
        if not self.args.execute:
            print('PREVIEW:', json.dumps(dict(q0=q0.tolist(), offsets_deg=np.rad2deg(offsets).tolist())), flush=True)
            return
        if self.count_publishers('/student/joint_effort_command') != 1:
            raise RuntimeError('stop the old guidance publisher before executing')
        response = self.call('/student_arm_node/prepare_guidance')
        self.prepared = True
        seed = json.loads(response)
        self.seed = seed
        self.initial_q = self.rows[-1][1].copy()
        self.command = motor_command(seed)
        for d in offsets:
            self.kin.check_q(np.array(seed['position'])+d)
        self.active = True
        self.tick()  # Exact authoritative first command; no gain/current step.
        self.spin(.25)
        if getattr(self.args, 'position_profile', False):
            anchor = self.rows[-1][1].copy()
            p0, d0 = np.asarray(self.command.p), np.asarray(self.command.d)
            reference = np.asarray(self.command.position)
            p1 = np.array([35., 35., 35., 5., 5., 5.])
            d1 = np.array([1.5, 1.5, 1.5, 1., 1., 1.])
            begin = time.monotonic()
            while True:
                self.check()
                u = min(1., (time.monotonic()-begin)/5.)
                blend_u = 3*u*u-2*u*u*u
                gains = p0+(p1-p0)*blend_u
                self.command.p = gains.tolist()
                self.command.d = (d0+(d1-d0)*blend_u).tolist()
                self.command.position = preserve_support_reference(
                    anchor, reference, p0, gains).tolist()
                self.spin(.01)
                if u >= 1.:
                    break
            self.record('position_profile_set', p=list(self.command.p),
                        d=list(self.command.d), q_cmd=list(self.command.position))
        wrist_damping = getattr(self.args, 'wrist_damping', None)
        if wrist_damping is not None:
            # Optional stationary gain transition, within the existing position
            # launch's Kd. Exact authoritative first command remains unchanged.
            d0 = np.asarray(self.command.d)
            d1 = d0.copy()
            d1[3:] = wrist_damping
            begin = time.monotonic()
            while True:
                self.check()
                u = min(1., (time.monotonic()-begin)/5.)
                self.command.d = (d0+(d1-d0)*(3*u*u-2*u*u*u)).tolist()
                self.spin(.01)
                if u >= 1.:
                    break
            self.record('wrist_damping_set', d=list(self.command.d))
        # A gain transition rescaled the support offset. Never restore the old
        # reference with the new gains when beginning the motion plan.
        motion_reference = np.array(self.command.position)
        for delta in offsets:
            self.kin.check_q(motion_reference+delta)
        start = motion_reference.copy()
        for i, (waypoint, delta) in enumerate(zip(plan, offsets)):
            target = motion_reference+delta
            self.record('waypoint', index=i, target=target.tolist())
            print(f'WAYPOINT {i}: offsets_deg={np.rad2deg(delta).tolist()}', flush=True)
            begin = time.monotonic()
            while True:
                self.check()
                qcmd, done = blend(start, target, time.monotonic()-begin, self.args.speed)
                self.command.position = qcmd.tolist()
                self.spin(.01)
                if done:
                    break
            end = time.monotonic()+waypoint.get('hold_seconds',4.)
            while time.monotonic() < end:
                self.check()
                self.spin(.02)
            if waypoint.get('capture', False):
                if self.args.allow_uninitialized_slam:
                    raise ValueError('cannot capture uninitialized SLAM')
                print(self.call('/camera_mount_calibration/capture'), flush=True)
            self.record('settled', index=i, q=self.rows[-1][1].tolist())
            self.log.flush()
            print(f'SETTLED {i}: q_deg={np.rad2deg(self.rows[-1][1]).tolist()}', flush=True)
            start = target

    def close(self):
        try:
            if self.prepared:
                # A latched driver fault intentionally rejects this service;
                # never clear it here. Still flush evidence and close the node.
                try:
                    print('STOP:', self.call('/student_arm_node/operator_hold'), flush=True)
                finally:
                    self.active = False
        finally:
            self.log.close()
            self.destroy_node()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--allow-uninitialized-slam', action='store_true')
    p.add_argument('--max-offset-deg', type=float, default=5.)
    p.add_argument('--speed', type=float, default=.025)
    p.add_argument('--wrist-damping', type=float, default=None,
                   help='optional five-second stationary Kd ramp; driver must permit it')
    p.add_argument('--position-profile', action='store_true',
                   help='five-second support-preserving ramp to student_arm6 Kp/Kd; driver must permit it')
    a = p.parse_args()
    if not 0 < a.speed <= .05 or not 0 < a.max_offset_deg <= 15:
        p.error('speed must be (0,.05] rad/s; excursion must be (0,15] degrees')
    if a.wrist_damping is not None and not .1 <= a.wrist_damping <= 1.:
        p.error('wrist damping must be within [.1,1.0], the existing position profile')
    if a.position_profile and a.wrist_damping is not None:
        p.error('choose one gain transition')
    plan = json.loads(Path(a.plan).read_text())
    checked_offsets(plan,a.max_offset_deg)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Probe(a)
    def interrupt(*_):
        node.interrupted = True
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        node.run(plan)
    finally:
        node.close()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
