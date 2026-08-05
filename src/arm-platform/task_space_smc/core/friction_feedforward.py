from __future__ import annotations

import numpy as np

__all__ = [
    "FrictionFeedforwardConfig",
    "approach_torque_stiffness",
    "stiction_feedforward",
    "residual_deadband",
]


_MAX_SAFE_SCALE = 1.0


class FrictionFeedforwardConfig:

    __slots__ = ("enabled", "stiction_nm", "scale", "velocity_epsilon",
                 "error_gain")

    def __init__(
        self,
        stiction_nm,
        *,
        enabled: bool = False,
        scale: float = 0.8,
        velocity_epsilon: float = 0.02,
        error_gain: float = 1.0,
    ) -> None:
        stiction = np.asarray(stiction_nm, dtype=float).ravel()
        if stiction.size == 0:
            raise ValueError("stiction_nm 不能为空")
        if not np.all(np.isfinite(stiction)):
            raise ValueError(f"stiction_nm 含非有限值: {stiction}")
        if np.any(stiction < 0.0):
            raise ValueError(f"stiction_nm 不能为负: {stiction}")

        scale = float(scale)
        if not np.isfinite(scale) or scale < 0.0:
            raise ValueError(f"scale 必须是 [0, 1) 内的有限值，收到 {scale}")
        if scale >= _MAX_SAFE_SCALE:
            raise ValueError(
                f"scale={scale} 不允许。必须严格小于 1 —— scale >= 1 时零误差处的"
                "前馈本身就够挣脱静摩擦，会自激成振幅约等于死区的极限环。"
                "详见 friction_feedforward 模块 docstring 设计选择 3。")

        velocity_epsilon = float(velocity_epsilon)
        if not np.isfinite(velocity_epsilon) or velocity_epsilon <= 0.0:
            raise ValueError(
                f"velocity_epsilon 必须为正（tanh 的速度尺度），收到 {velocity_epsilon}")

        error_gain = float(error_gain)
        if not np.isfinite(error_gain) or error_gain < 0.0:
            raise ValueError(
                f"error_gain 必须非负（1/s，把位置误差折成等效速度），收到 {error_gain}")

        self.enabled = bool(enabled)
        self.stiction_nm = stiction
        self.scale = scale
        self.velocity_epsilon = velocity_epsilon
        self.error_gain = error_gain

    @property
    def n_joints(self) -> int:
        return int(self.stiction_nm.size)

    def __repr__(self) -> str:
        return (f"FrictionFeedforwardConfig(enabled={self.enabled}, "
                f"scale={self.scale}, velocity_epsilon={self.velocity_epsilon}, "
                f"error_gain={self.error_gain}, "
                f"stiction_nm={np.round(self.stiction_nm, 4).tolist()})")


def stiction_feedforward(
    config: FrictionFeedforwardConfig,
    velocity_reference,
    position_error,
) -> np.ndarray:
    n = config.n_joints
    qd_ref = np.asarray(velocity_reference, dtype=float).reshape(n)
    err = np.asarray(position_error, dtype=float).reshape(n)

    if not config.enabled:
        return np.zeros(n)

    if not np.all(np.isfinite(qd_ref)) or not np.all(np.isfinite(err)):

        return np.zeros(n)

    drive = qd_ref + config.error_gain * err
    return config.scale * config.stiction_nm * np.tanh(
        drive / config.velocity_epsilon)


def approach_torque_stiffness(inertia_diagonal, kp) -> np.ndarray:
    M = np.asarray(inertia_diagonal, dtype=float).ravel()
    gains = np.asarray(kp, dtype=float).ravel()
    if M.shape != gains.shape:
        raise ValueError(f"惯量与增益维度不一致: {M.shape} vs {gains.shape}")
    if np.any(M <= 0.0):
        raise ValueError(f"惯量对角元必须为正，收到 {M}")
    return M * gains


def residual_deadband(config: FrictionFeedforwardConfig, torque_stiffness) -> np.ndarray:
    n = config.n_joints
    gains = np.asarray(torque_stiffness, dtype=float).reshape(n)
    if np.any(gains <= 0.0):
        raise ValueError(f"torque_stiffness 必须为正，收到 {gains}")

    f_s = config.stiction_nm
    if not config.enabled or config.scale == 0.0:
        return f_s / gains

    knee = config.velocity_epsilon / config.error_gain if config.error_gain else np.inf

    lo = np.zeros(n)
    hi = f_s / gains
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        drive = gains * mid + config.scale * f_s * np.tanh(mid / knee)
        stuck = drive < f_s
        lo = np.where(stuck, mid, lo)
        hi = np.where(stuck, hi, mid)
    return 0.5 * (lo + hi)
