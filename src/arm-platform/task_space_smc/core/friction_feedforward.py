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
        scale: float = 0.8,           # 补偿比例，必须 < 1。大→残留死区小，≥1 时零误差处即可自行挣脱、自激成极限环；小→补不动
        velocity_epsilon: float = 0.02,  # tanh 的速度尺度 ε (rad/s)。大→过零平缓、零速附近补得少；小→接近 sign()、零速换向抖
        error_gain: float = 1.0,      # 位置误差折成等效速度的 k_e (1/s)。大→静止时也肯补、死区更窄但易越过；小→只在真动起来时才补
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
                "前馈本身就够挣脱静摩擦，会自激成振幅约等于死区的极限环。")

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


# 静摩擦前馈力矩。静摩擦在零速附近是符号函数，直接补 f_s·sign(q̇) 会在零速处
# 反复换向抖动，故用 tanh 光滑化，并把位置误差按 k_e 折成等效速度一并作为驱动量：
#   drive = q̇_ref + k_e·e_q
#   τ_ff  = scale · f_s · tanh(drive / ε)
# f_s 为静摩擦力矩，ε (velocity_epsilon) 是 tanh 的速度尺度，k_e (error_gain, 1/s)
# 把位置误差换算成速度，scale < 1 保证零误差处的前馈不足以自行挣脱静摩擦。
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


# 关节等效力矩刚度。关节空间 PD 的比例项经惯量折算成力矩：
#   τ = M·q̈_des ≈ M·kp·e_q  ⇒  K = M ⊙ kp   (N·m/rad)
def approach_torque_stiffness(inertia_diagonal, kp) -> np.ndarray:
    M = np.asarray(inertia_diagonal, dtype=float).ravel()
    gains = np.asarray(kp, dtype=float).ravel()
    if M.shape != gains.shape:
        raise ValueError(f"惯量与增益维度不一致: {M.shape} vs {gains.shape}")
    if np.any(M <= 0.0):
        raise ValueError(f"惯量对角元必须为正，收到 {M}")
    return M * gains


# 残留稳态死区 δ：卡住的位置误差恰好使总驱动力矩等于静摩擦，
#   K·δ + scale·f_s·tanh(δ / knee) = f_s,   knee = ε / k_e
# 左端对 δ 单调递增，故在 [0, f_s/K] 上二分求根；前馈关闭时退化为 δ = f_s / K。
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
