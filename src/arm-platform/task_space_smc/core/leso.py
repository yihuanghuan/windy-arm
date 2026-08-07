from __future__ import annotations

import numpy as np

__all__ = ["JointLESO"]


# 关节速度的三阶线性扩张状态观测器（LESO）。把每个关节视为双积分链
#   q̈ = b0·u + f,   f 为总扰动（未建模动力学、摩擦、外力）
# 状态 z = [q̂, q̇̂, f̂]，取 q̇̂ 作为速度估计，替代对编码器角度做差分。
# 结构为「零阶保持预测 + 当前观测器校正」，每步用当前时刻的位置量测校正：
#   预测  p = Φ·z + [dt²/2, dt, 0]ᵀ·b0·u,  Φ = [[1, dt, dt²/2], [0, 1, dt], [0, 0, 1]]
#   校正  z⁺ = p + L·(q_meas − p[0])
class JointLESO:

    def __init__(
        self,
        omega_o,
        n_joints: int,
        nominal_dt: float = 0.005,   # 名义控制周期 (s)，dt 缺失或非法时的兜底值
        dt_min_ratio: float = 0.2,   # 实际 dt 相对名义值的下限比。大→抖动的 dt 被夹得更狠、增益更贴名义值；小→放任极小 dt，l3 = (1−β)³/dt² 会爆
        dt_max_ratio: float = 4.0,   # 上限比。大→放任大 dt，一拍推进过多、观测器发散；小→丢拍时推进不足，估计滞后
        resync_sec: float = 0.05,    # 超过此间隔判为断流，只对齐位置不推进状态 (s)。大→拿超长间隔硬积分；小→轻微丢拍就重置、估计被反复打断
        b0=0.0,                      # 输入增益 b0，q̈ = b0·u + f 里的 b0。为 0 即不用模型输入，三阶 LESO 退化成纯微分器
    ) -> None:
        w = np.asarray(omega_o, dtype=float).ravel()
        if w.size == 1:
            w = np.full(int(n_joints), float(w[0]))
        if w.size != int(n_joints):
            raise ValueError(
                f"omega_o 需要 {n_joints} 个值或 1 个标量，收到 {w.size}")
        if not np.all(np.isfinite(w)):
            raise ValueError(f"omega_o 含非有限值: {w}")
        if np.any(w < 0.0):
            raise ValueError(f"omega_o 不能为负: {w}")
        nominal_dt = float(nominal_dt)
        if not np.isfinite(nominal_dt) or nominal_dt <= 0.0:
            raise ValueError(f"nominal_dt 必须为正，收到 {nominal_dt}")
        if not 0.0 < float(dt_min_ratio) < float(dt_max_ratio):
            raise ValueError(
                f"需要 0 < dt_min_ratio < dt_max_ratio，收到 "
                f"{dt_min_ratio} / {dt_max_ratio}")

        self.omega_o = w
        self.n_joints = int(n_joints)
        self.nominal_dt = nominal_dt
        self.dt_lo = float(dt_min_ratio) * nominal_dt
        self.dt_hi = float(dt_max_ratio) * nominal_dt
        self.resync_sec = float(resync_sec)


        self._active = self.omega_o > 0.0
        self._z = np.zeros((3, self.n_joints))
        self._primed = False


        b = np.asarray(b0, dtype=float).ravel()
        if b.size == 1:
            b = np.full(self.n_joints, float(b[0]))
        if b.size != self.n_joints:
            raise ValueError(f"b0 需要 {n_joints} 个值或 1 个标量，收到 {b.size}")
        if not np.all(np.isfinite(b)):
            raise ValueError(f"b0 含非有限值: {b}")
        self.b0 = b
        self._has_input = bool(np.any(b != 0.0))


    @property
    def enabled(self) -> bool:
        return bool(np.any(self._active))

    @property
    def position_estimate(self) -> np.ndarray:
        return self._z[0].copy()

    @property
    def velocity_estimate(self) -> np.ndarray:
        return self._z[1].copy()

    @property
    def disturbance_estimate(self) -> np.ndarray:
        return self._z[2].copy()

    # 观测器增益 L，由估计误差动力学的三重离散极点配在 z = β = e^(−ω_o·dt) 解出：
    #   l1 = 1 − β³
    #   l2 = 1.5/dt · (1 − β)² · (1 + β)
    #   l3 = (1 − β)³ / dt²
    # 三个增益只由带宽 ω_o 一个量决定（带宽参数化）。
    def gains(self, dt: float) -> np.ndarray:
        dt = float(dt)
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError(f"dt 必须为正的有限值，收到 {dt}")
        b = np.exp(-self.omega_o * dt)
        one_b = 1.0 - b
        return np.vstack([
            1.0 - b ** 3,
            1.5 / dt * one_b ** 2 * (1.0 + b),
            one_b ** 3 / (dt * dt),
        ])

    def discrete_pole(self, dt: float) -> np.ndarray:
        return np.exp(-self.omega_o * float(dt))

    # 位置量测 → 速度估计的离散频响，并按理想微分 j2πf 归一（理想微分处结果为 1）：
    #   A = (I − L·e1ᵀ)·Φ
    #   G(z) = z · e2ᵀ · (zI − A)⁻¹ · L
    #   H(f) = G(e^(j2πf·dt)) / (j2πf)
    def response(self, frequency_hz, dt: float | None = None) -> np.ndarray:
        dt = self.nominal_dt if dt is None else float(dt)
        f = float(frequency_hz)
        if f == 0.0:
            return np.ones(self.n_joints, dtype=complex)
        z = np.exp(2j * np.pi * f * dt)
        out = np.ones(self.n_joints, dtype=complex)
        Phi = np.array([[1.0, dt, 0.5 * dt * dt], [0.0, 1.0, dt], [0.0, 0.0, 1.0]])
        L_all = self.gains(dt)
        for j in range(self.n_joints):
            if not self._active[j]:
                continue
            L = L_all[:, j]
            A = (np.eye(3) - np.outer(L, [1.0, 0.0, 0.0])) @ Phi
            gy = z * (np.linalg.solve(z * np.eye(3) - A, L)[1])
            out[j] = gy / (2j * np.pi * f)
        return out

    def phase_lag_deg(self, frequency_hz, dt: float | None = None) -> np.ndarray:
        return -np.degrees(np.angle(self.response(frequency_hz, dt)))


    def reset(self, position=None, velocity=None) -> None:
        self._z[:] = 0.0
        if position is not None:
            self._z[0] = np.asarray(position, dtype=float).reshape(self.n_joints)
        if velocity is not None:
            self._z[1] = np.asarray(velocity, dtype=float).reshape(self.n_joints)
        self._primed = position is not None

    def __call__(self, position, velocity, dt: float | None = None, u=None):
        if not self.enabled:
            return velocity
        q = np.asarray(position, dtype=float).reshape(self.n_joints)
        v_in = np.asarray(velocity, dtype=float).reshape(self.n_joints)

        if not np.all(np.isfinite(q)):

            out = self._z[1].copy()
            out[~self._active] = v_in[~self._active]
            return out

        if not self._primed:


            self._z[0] = q
            self._z[1] = np.where(np.isfinite(v_in), v_in, 0.0)
            self._z[2] = 0.0
            self._primed = True
            out = self._z[1].copy()
            out[~self._active] = v_in[~self._active]
            return out

        raw_dt = self.nominal_dt if dt is None else float(dt)
        if not np.isfinite(raw_dt) or raw_dt <= 0.0:
            raw_dt = self.nominal_dt
        if raw_dt > self.resync_sec:


            self._z[0] = q
            out = self._z[1].copy()
            out[~self._active] = v_in[~self._active]
            return out

        step = float(np.clip(raw_dt, self.dt_lo, self.dt_hi))
        z0, z1, z2 = self._z


        acc = z2
        if self._has_input and u is not None:
            acc = z2 + self.b0 * np.asarray(u, dtype=float).reshape(self.n_joints)
        p0 = z0 + step * z1 + 0.5 * step * step * acc
        p1 = z1 + step * acc
        p2 = z2

        l1, l2, l3 = self.gains(step)
        innovation = q - p0
        self._z = np.vstack([
            p0 + l1 * innovation,
            p1 + l2 * innovation,
            p2 + l3 * innovation,
        ])
        out = self._z[1].copy()


        out[~self._active] = v_in[~self._active]
        self._z[0][~self._active] = q[~self._active]
        self._z[1][~self._active] = v_in[~self._active]
        self._z[2][~self._active] = 0.0
        return out

    def describe(self) -> str:
        if not self.enabled:
            return "关节速度 LESO: 全部关闭（逐位直通）"
        f_list = (1.0, 10.0, 24.8)
        lag = {f: np.round(self.phase_lag_deg(f), 2).tolist() for f in f_list}
        return (
            "关节速度 LESO(3阶, ZOH+当前观测器): ω_o=%s rad/s (%s Hz)，"
            "离散极点 %s @%.1f ms，相位滞后 1Hz %s° / 10Hz %s° / 24.8Hz %s°"
            "（对照：电机自报的额外 15 ms 在 24.8 Hz 是 133.9°）"
            % (np.round(self.omega_o, 1).tolist(),
               np.round(self.omega_o / (2 * np.pi), 1).tolist(),
               np.round(self.discrete_pole(self.nominal_dt), 4).tolist(),
               1000.0 * self.nominal_dt,
               lag[1.0], lag[10.0], lag[24.8]))
