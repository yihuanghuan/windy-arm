"""Per-axis phase-dependent speed guard; caller supplies monotonic seconds."""
import numpy as np

SPEED_KEYS = ('transition_warning_speed', 'transition_abort_speed',
              'guide_warning_speed', 'guide_sustained_abort_speed',
              'guide_sustained_abort_duration', 'guide_instant_abort_speed')
MANUAL_KEYS = ('guide_manual_warning_speed', 'guide_manual_sustained_abort_speed',
               'guide_manual_instant_abort_speed')


class SpeedSafety:
    def __init__(self, config):
        self.cfg = {k: float(config[k]) for k in SPEED_KEYS}
        self.manual = {k: np.asarray(config[k], dtype=float) for k in MANUAL_KEYS}
        self.manual_duration = float(config['guide_manual_sustained_duration'])
        if (any(v.shape != (6,) or not np.isfinite(v).all() or np.any(v <= 0)
                for v in self.manual.values()) or not np.isfinite(self.manual_duration) or
                self.manual_duration <= 0 or not np.all(self.manual[MANUAL_KEYS[0]] < self.manual[MANUAL_KEYS[1]]) or
                not np.all(self.manual[MANUAL_KEYS[1]] < self.manual[MANUAL_KEYS[2]])):
            raise ValueError('invalid six-axis manual speed parameters')
        c = self.cfg
        if not all(np.isfinite(v) and v > 0 for v in c.values()):
            raise ValueError('speed safety parameters must be positive/finite')
        if not (c['transition_warning_speed'] <= c['transition_abort_speed'] <=
                c['guide_instant_abort_speed'] and c['guide_warning_speed'] <
                c['guide_sustained_abort_speed'] < c['guide_instant_abort_speed']):
            raise ValueError('invalid speed safety threshold ordering')
        self.since = [None] * 6
        self.last = None
        self.guide = None
        self.manual_phase = False
        self.status = {}
        self.fault = None

    def update(self, dq, phase, now):
        if self.fault:
            return self.status
        dq = np.asarray(dq, dtype=float)
        if dq.shape != (6,) or not np.isfinite(dq).all() or not np.isfinite(now):
            raise ValueError('invalid speed sample')
        if self.last is not None and now < self.last:
            raise ValueError('speed safety requires monotonic time')
        guide = phase in ('GUIDE', 'MANUAL_PUSH')
        manual = phase == 'MANUAL_PUSH'
        if guide != self.guide or manual != self.manual_phase:
            self.since = [None] * 6
        self.guide, self.manual_phase, self.last = guide, manual, now
        c = self.cfg
        warning, duration, kinds = [], [], []
        for i, speed in enumerate(abs(dq)):
            warning_limit = self.manual[MANUAL_KEYS[0]][i] if manual else c['guide_warning_speed' if guide else 'transition_warning_speed']
            sustained_limit = self.manual[MANUAL_KEYS[1]][i] if manual else c['guide_sustained_abort_speed']
            instant_limit = self.manual[MANUAL_KEYS[2]][i] if manual else c['guide_instant_abort_speed']
            duration_limit = self.manual_duration if manual else c['guide_sustained_abort_duration']
            warning.append(bool(speed > warning_limit))
            if guide and speed > sustained_limit:
                if self.since[i] is None:
                    self.since[i] = now
            else:
                self.since[i] = None
            elapsed = 0. if self.since[i] is None else now-self.since[i]
            duration.append(elapsed)
            kinds.append('instantaneous' if guide and speed > instant_limit else
                         'sustained' if guide and self.since[i] is not None and
                         elapsed >= duration_limit else
                         'transition' if not guide and speed > c['transition_abort_speed'] else '')
        self.status = dict(phase=phase, dq=dq.tolist(), warning=warning,
                           sustained_duration=duration, fault_types=kinds,
                           fault_axes=[bool(k) for k in kinds])
        if any(kinds):
            i = next(i for i,k in enumerate(kinds) if k)
            self.fault = (f'speed HARD_ABORT joint{i+1} phase={phase} type={kinds[i]} '
                          f'dq={dq[i]:.6f} duration={duration[i]:.6f}s')
        return self.status
