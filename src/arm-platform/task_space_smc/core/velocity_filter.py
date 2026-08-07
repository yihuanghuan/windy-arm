from __future__ import annotations

import numpy as np

__all__ = ["JointVelocityFilter"]


# 关节速度一阶低通（指数滑动平均），逐关节独立，截止频率 f_c ≤ 0 时该关节直通：
#   τ = 1/(2π·f_c),  α = dt/(τ + dt),  y_k = y_{k−1} + α·(u_k − y_{k−1})
#   相位滞后 φ(f) = arctan(f / f_c)
class JointVelocityFilter:

    def __init__(self, cutoff_hz, n_joints: int) -> None:
        cutoff = np.asarray(cutoff_hz, dtype=float).ravel()
        if cutoff.size == 1:
            cutoff = np.full(int(n_joints), float(cutoff[0]))
        if cutoff.size != int(n_joints):
            raise ValueError(
                f"cutoff_hz 需要 {n_joints} 个值或 1 个标量，收到 {cutoff.size}")
        if not np.all(np.isfinite(cutoff)):
            raise ValueError(f"cutoff_hz 含非有限值: {cutoff}")
        if np.any(cutoff < 0.0):
            raise ValueError(f"cutoff_hz 不能为负: {cutoff}")
        self.cutoff_hz = cutoff
        self.n_joints = int(n_joints)
        self._state = np.zeros(self.n_joints)
        self._primed = False

    @property
    def enabled(self) -> bool:
        return bool(np.any(self.cutoff_hz > 0.0))

    def alpha(self, dt: float) -> np.ndarray:
        dt = float(dt)
        if dt <= 0.0:
            raise ValueError(f"dt 必须为正，收到 {dt}")
        a = np.ones(self.n_joints)
        active = self.cutoff_hz > 0.0
        tau = 1.0 / (2.0 * np.pi * self.cutoff_hz[active])
        a[active] = dt / (tau + dt)
        return a

    def phase_lag_deg(self, frequency_hz) -> np.ndarray:
        f = float(frequency_hz)
        out = np.zeros(self.n_joints)
        active = self.cutoff_hz > 0.0
        out[active] = np.degrees(np.arctan2(f, self.cutoff_hz[active]))
        return out

    def reset(self, value=None) -> None:
        self._state = (np.zeros(self.n_joints) if value is None
                       else np.asarray(value, dtype=float).reshape(self.n_joints).copy())
        self._primed = value is not None

    def __call__(self, velocity, dt: float) -> np.ndarray:
        v = np.asarray(velocity, dtype=float).reshape(self.n_joints)
        if not self.enabled:
            return v.copy()
        if not np.all(np.isfinite(v)):

            return self._state.copy()
        if not self._primed:

            self._state = v.copy()
            self._primed = True
            return self._state.copy()
        a = self.alpha(dt)
        self._state = self._state + a * (v - self._state)


        passthrough = self.cutoff_hz <= 0.0
        self._state[passthrough] = v[passthrough]
        return self._state.copy()
