"""Decides only whether to stop READING. Never controls or makes an arm safe."""
from collections import deque
import math


class ReadGuard:
    def __init__(self):
        self.positions = deque()
        self.velocities = deque()
        self.high_runs = [0]*6
        self.high_since = [None]*6
        self.warnings = [0]*6
        self.first = None
        self.last = None
        self.latest_velocity = None

    def update(self, cmd, values, t):
        if not all(math.isfinite(x) for x in values):
            return 'nonfinite physical-channel feedback'
        if cmd == 0x21:
            self.latest_velocity = list(values)
            self.velocities.append((t,list(values)))
            for j,v in enumerate(values):
                if abs(v) > .15:
                    self.warnings[j] += 1
                    self.high_runs[j] += 1
                    if self.high_since[j] is None: self.high_since[j] = t
                    if self.high_runs[j] >= 5 and t-self.high_since[j] >= .019:
                        return f'joint{j+1}: velocity >0.15 for >=5 samples / >=19ms'
                else:
                    self.high_runs[j] = 0; self.high_since[j] = None
        if cmd == 0x20:
            if self.first is None: self.first = list(values)
            if self.last is not None and max(abs(a-b) for a,b in zip(values,self.last)) > .02:
                return 'position jump >0.02 rad between adjacent frames'
            if max(abs(a-b) for a,b in zip(values,self.first)) > .02:
                return 'position drift >0.02 rad from initial observation'
            self.last = list(values)
            self.positions.append((t,list(values)))
            while self.positions and t-self.positions[0][0] > .1: self.positions.popleft()
            if len(self.positions) >= 4:
                old_t,old=self.positions[0]
                if t-old_t >= .05:
                    for j in range(6):
                        disp=values[j]-old[j]
                        changes=[b[1][j]-a[1][j] for a,b in zip(self.positions,list(self.positions)[1:])]
                        same=sum(d*disp>0 and abs(d)>1e-4 for d in changes)
                        if abs(disp) > .01 and same >= 3:
                            return f'joint{j+1}: continuous position displacement >0.01 rad/100ms'
                        vs=[v[j] for vt,v in self.velocities if old_t <= vt <= t]
                        if vs and abs(disp)>.003:
                            mean=sum(vs)/len(vs); expected=mean*(t-old_t)
                            if mean*disp>0 and abs(mean)>.05 and .25<abs(expected/disp)<4:
                                return f'joint{j+1}: velocity and displacement consistently indicate movement'
        while self.velocities and t-self.velocities[0][0]>.12: self.velocities.popleft()
        return None
