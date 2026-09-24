"""Timestamp matching and bounded velocity feedforward; no robot publishers."""
from collections import deque

import numpy as np

from camera_world_hold_core import clamp_norm, pose_deadzone, vector


class JointMotionHistory:
    """Fit local encoder position slopes at the image time, not receipt time.

    A short linear fit suppresses encoder quantization. It only uses samples
    already received, never predicts forward, and rejects gaps/extrapolation.
    """
    def __init__(self, window=.10):
        if not np.isfinite(window) or not .04 <= window <= .3:
            raise ValueError('joint velocity window must be within [.04, .3] s')
        self.window = window
        self.rows = deque(maxlen=400)

    def add(self, stamp, q):
        q = vector(q, 6)
        if not np.isfinite(stamp) or stamp <= 0:
            raise ValueError('invalid encoder timestamp')
        if self.rows and stamp <= self.rows[-1][0]:
            self.rows.clear()
        self.rows.append((stamp, q.copy()))

    def at(self, stamp):
        rows = [(t-stamp, q) for t, q in self.rows if abs(t-stamp) <= self.window/2]
        if len(rows) < 4:
            raise ValueError('encoder history warming up')
        t = np.array([r[0] for r in rows])
        q = np.array([r[1] for r in rows])
        if t[0] > 1e-6 or t[-1] < -1e-6 or np.ptp(t) < .5*self.window:
            raise ValueError('encoder history does not bracket image time')
        if np.max(np.diff(t)) > .04:
            raise ValueError('encoder history gap')
        # Interpolate q at the timestamp; regress dq over the same short window.
        at_stamp = np.array([np.interp(0., t, q[:, j]) for j in range(6)])
        centered = t-t.mean()
        dq = centered @ q / (centered @ centered)
        return at_stamp, dq


class VelocitySamples:
    def __init__(self, window=.10):
        self.joints = JointMotionHistory(window)
        self.poses = deque(maxlen=90)
        self.twists = deque(maxlen=90)
        self.input_reason = 'waiting for camera twist'

    def add_pose(self, stamp, received, pose):
        if self.poses and stamp <= self.poses[-1][0]:
            self.poses.clear()
            self.twists.clear()
        self.poses.append((stamp, received, pose.copy()))

    def add_twist(self, stamp, received, value):
        value = vector(value, 6)
        if not np.isfinite(stamp) or stamp <= 0:
            raise ValueError('invalid twist timestamp')
        if self.twists and stamp <= self.twists[-1][0]:
            self.twists.clear()
        self.twists.append((stamp, received, value))
        self.input_reason = 'waiting for synchronized camera pose'

    def invalidate(self, reason):
        self.twists.clear()
        self.input_reason = reason

    def evaluate(self, kin, now, ros_now, latest_pose_stamp, cfg):
        gains = np.array([cfg['velocity_feedforward_linear_gain']]*3 +
                         [cfg['velocity_feedforward_angular_gain']]*3)
        zero = np.zeros(6)
        status = dict(ff_enabled=bool(np.any(gains)), ff_valid=False, ff_reason='disabled',
                      ff_sample_stamp=None, ff_sensor_latency=None, ff_receive_gap=None)
        if not np.any(gains):
            return zero, status, {}
        try:
            if not self.twists:
                raise ValueError(self.input_reason)
            if now-self.twists[-1][1] > cfg['twist_receive_timeout']:
                raise ValueError('camera twist receive gap exceeded; P-only fallback')
            pair = None
            for stamp, received, twist in reversed(self.twists):
                if not 0 <= latest_pose_stamp-stamp <= cfg['feedforward_max_sample_lag']:
                    continue
                for ps, pr, pose in reversed(self.poses):
                    # The wrapper emits pose/twist with the exact same image stamp.
                    if abs(ps-stamp) < 1e-6:
                        pair = stamp, min(received, pr), pose, twist
                        break
                if pair is not None:
                    break
            if pair is None:
                raise ValueError('no matching pose/twist timestamps; P-only fallback')
            stamp, received, pose, twist = pair
            status.update(ff_sample_stamp=stamp, ff_sensor_latency=ros_now-stamp,
                          ff_receive_gap=now-received)
            if (now-received > cfg['twist_receive_timeout'] or
                    not -.05 <= ros_now-stamp <= cfg['pose_header_max_age']):
                raise ValueError('matched velocity sample stale; P-only fallback')
            q, dq = self.joints.at(stamp)
            details = kin.external_velocity(q, dq, pose, twist)
            external = details['external_link6_twist']
            ff = -gains * np.r_[pose_deadzone(external[:3], cfg['feedforward_linear_deadband']),
                               pose_deadzone(external[3:], cfg['feedforward_angular_deadband'])]
            ff = np.r_[clamp_norm(ff[:3], cfg['feedforward_max_linear_velocity']),
                       clamp_norm(ff[3:], cfg['feedforward_max_angular_velocity'])]
            if not np.isfinite(ff).all():
                raise ValueError('nonfinite feedforward; P-only fallback')
            status.update(ff_valid=True, ff_reason='synchronized external velocity')
            details['synchronized_joint_velocity'] = dq
            return ff, status, details
        except (ValueError, np.linalg.LinAlgError) as error:
            status['ff_reason'] = str(error)
            return zero, status, {}
