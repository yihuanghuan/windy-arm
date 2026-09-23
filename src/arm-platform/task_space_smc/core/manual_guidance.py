"""Fixed-base six-axis manual guidance mathematics; no ROS or serial I/O."""
from pathlib import Path

import numpy as np
import pinocchio as pin

from .gravity_semantics import explicit_offsets, urdf_feedforward
from .speed_safety import SpeedSafety

NAMES = tuple(f"joint{i}" for i in range(1, 7))
RATED = np.array([9., 9., 9., 1.6, 1.6, 1.6])


def vector(value, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (6,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain exactly six finite numbers")
    return result.copy()


class Guidance:
    def __init__(self, urdf, config):
        self.path = str(Path(urdf).resolve())
        self.model = pin.buildModelFromUrdf(self.path)
        if (self.model.nq != 6 or self.model.nv != 6 or
                tuple(self.model.names)[1:] != NAMES or
                not self.model.existFrame('link6')):
            raise ValueError("manual guidance requires joint1..joint6 and link6")
        self.data = self.model.createData()
        self.offset = explicit_offsets(6, config['model_zero_offset_rad'])
        self.lower = np.maximum(vector(config['joint_lower_limits'], 'lower'),
                                self.model.lowerPositionLimit - self.offset)
        self.upper = np.minimum(vector(config['joint_upper_limits'], 'upper'),
                                self.model.upperPositionLimit - self.offset)
        self.limit = vector(config['torque_limits'], 'torque_limits')
        self.slew = vector(config['torque_rate_limits'], 'torque_rate_limits')
        self.kd = vector(config['damping'], 'damping')
        self.hold_p = vector(config['hold_p'], 'hold_p')
        self.hold_d = vector(config['hold_d'], 'hold_d')
        self.speed_guard = SpeedSafety(config)
        self.speed = self.speed_guard.cfg['guide_instant_abort_speed']
        self.margin = float(config['joint_limit_margin'])
        self.ramp = float(config['ramp_duration_sec'])
        self.release = float(config['release_duration_sec'])
        self.timeout = float(config['feedback_timeout_sec'])
        for value in (self.speed, self.margin, self.ramp, self.release, self.timeout):
            if not np.isfinite(value) or value <= 0:
                raise ValueError('durations, velocity and margin must be positive/finite')
        if (np.any(self.limit <= 0) or np.any(self.limit > RATED) or
                np.any(self.slew <= 0) or np.any(self.kd <= 0) or
                np.any(self.hold_p <= 0) or np.any(self.hold_d < self.kd) or
                np.any(self.lower + 2 * self.margin >= self.upper)):
            raise ValueError('invalid gains, limits or safety interval')
        self.q_start = None
        self.started = None
        self.last_time = None
        self.last_tau = np.zeros(6)
        self.fault = None
        self.seed = None
        self.manual_active = False
        self.previous_command = None

    def set_hold_command(self, command):
        if self.started is not None:
            raise ValueError('cannot replace an active handover')
        self.seed = {k: vector(command[k], 'hold '+k) for k in
                     ('position', 'velocity', 'current', 'p', 'd')}
        c = self.seed
        if (np.any(c['p'] <= 0) or np.any(c['p'] > self.hold_p) or
                np.any(c['d'] < self.kd) or np.any(c['d'] > self.hold_d) or
                np.any(c['velocity'] != 0) or np.any(abs(c['current']) > self.limit) or
                np.any(c['position'] < self.lower) or np.any(c['position'] > self.upper)):
            raise ValueError('invalid existing hold command')
        self.last_tau = c['current'].copy()

    def enable_manual_push(self):
        if self.fault or self.previous_command is None or self.previous_command['phase'] != 'GUIDE':
            raise ValueError('manual push requires healthy GUIDE')
        self.manual_active = True

    def gravity(self, q):
        return pin.computeGeneralizedGravity(self.model, self.data, vector(q, 'q_model')).copy()

    def command(self, q, dq, now, age):
        """RAMP_GRAVITY → RELEASE_STIFFNESS → GUIDE. Any fault requires restart."""
        if self.fault:
            raise ValueError(self.fault)
        try:
            return self._command(q, dq, float(now), float(age))
        except (ValueError, RuntimeError) as exc:
            self.fault = str(exc)
            raise

    def _command(self, q, dq, now, age):
        q, dq = vector(q, 'q'), vector(dq, 'dq')
        if not np.isfinite(now) or not np.isfinite(age) or age < 0 or age > self.timeout:
            raise ValueError('stale/invalid feedback time')
        if np.any(q <= self.lower + self.margin) or np.any(q >= self.upper - self.margin):
            raise ValueError('joint limit margin reached')
        elapsed = 0. if self.started is None else now-self.started
        phase = (('MANUAL_PUSH' if self.manual_active else 'GUIDE') if elapsed >= self.ramp+self.release else
                 'KP_WITHDRAWAL' if elapsed >= self.ramp else
                 'HANDOVER_START' if self.started is None else 'HANDOVER_GRAVITY_MATCH')
        self.speed_guard.update(dq, phase, now)
        if self.speed_guard.fault:
            raise ValueError(self.speed_guard.fault)
        gravity = urdf_feedforward(self, q, self.offset)
        # Refuse an unsupported posture rather than silently accepting gravity saturation.
        budget_speed = self.speed_guard.manual['guide_manual_instant_abort_speed'] if self.manual_active else self.speed
        if np.any(np.abs(gravity) + self.kd * budget_speed > self.limit):
            raise ValueError('gravity plus damping exceeds configured torque budget')
        if self.started is None:
            if self.seed is None:
                raise ValueError('authoritative existing hold command required; no zero-FF takeover')
            if np.any(np.abs(dq) > 0.15):
                raise ValueError('arm must be nearly stationary for takeover')
            self.started = self.last_time = now
            self.q_start = q.copy()
        dt = now - self.last_time
        if dt < 0 or dt > self.timeout:
            raise ValueError('controller interval exceeded')
        elapsed = now - self.started
        blend = np.clip(elapsed / self.ramp, 0., 1.)
        blend = blend * blend * (3. - 2. * blend)
        release = np.clip((elapsed - self.ramp) / self.release, 0., 1.)
        release = release * release * (3. - 2. * release)
        target = (1.-blend) * self.seed['current'] + blend * gravity
        tau = np.clip(target, self.last_tau - self.slew * dt, self.last_tau + self.slew * dt)
        tau = np.clip(tau, -self.limit, self.limit)
        # Never drop position support while gravity slew limiting is still catching up.
        if release > 0 and np.max(np.abs(tau - gravity)) > 0.02:
            raise ValueError('gravity slew lag during stiffness release')
        self.last_tau, self.last_time = tau.copy(), now
        guiding = release >= 1.
        # H1: P*(q_ref-q)+FF is invariant at a fixed measured q. Never erase
        # the inherited spring load by resetting q_ref to q_measured.
        reference = self.seed['position'] - (tau-self.seed['current'])/self.seed['p']
        p = (1.-release)*self.seed['p']
        d = (1.-release)*self.seed['d'] + release*self.kd
        reference = q if guiding else reference
        if np.any(reference < self.lower) or np.any(reference > self.upper):
            raise ValueError('handover reference exceeds joint limits')
        pd = p*(reference-q)-d*dq
        total = pd+tau
        if np.any(abs(total) > self.limit):
            raise ValueError('estimated total support exceeds torque budget')
        previous = self.previous_command or self.seed
        previous_total = (np.asarray(previous['p'])*(np.asarray(previous['position'])-q)
                          -np.asarray(previous['d'])*dq+np.asarray(previous['current']))
        result = dict(position=reference.tolist(),
                    velocity=[0.] * 6, current=tau.tolist(),
                    p=p.tolist(), d=d.tolist(), enable=1, phase=phase,
                    q=q.tolist(), dq=dq.tolist(), gravity=gravity.tolist(),
                    estimated_pd_torque=pd.tolist(), estimated_total_torque=total.tolist(),
                    delta_total_command_from_previous_cycle=(total-previous_total).tolist())
        self.previous_command = result
        return result
