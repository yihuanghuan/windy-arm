from __future__ import annotations

from collections import deque

import numpy as np

__all__ = [
    "SmithPredictorConfig",
    "JointStatePredictor",
    "DEFAULT_MAX_VELOCITY_CORRECTION",
    "DEFAULT_MAX_POSITION_CORRECTION",
]


# 两路修正的限幅缺省值。大→敢补更多、模型不准时把错误直接灌进控制器；
# 小→补偿被截断，连续触顶还会触发整体退回原始反馈。
DEFAULT_MAX_VELOCITY_CORRECTION = (0.20, 0.20, 0.20, 0.50, 0.50, 0.50)  # Δq̇ 上限 (rad/s)，大臂 0.20、腕部 0.50
DEFAULT_MAX_POSITION_CORRECTION = 0.005                                 # Δq 上限 (rad)


class SmithPredictorConfig:

    __slots__ = ("enabled", "n_joints", "velocity_horizon_sec",
                 "position_horizon_sec", "beta_velocity", "beta_position",
                 "max_velocity_correction", "max_position_correction",
                 "fallback_after_clamped_ticks")

    def __init__(
        self,
        n_joints: int,
        *,
        enabled: bool = False,
        velocity_horizon_sec: float = 0.025,  # 速度路补偿时域 H_v (s)，应等于速度路名义延时。大→补过头、超前振荡；小→补不足、仍滞后
        position_horizon_sec: float = 0.010,  # 位置路补偿时域 H_p (s)，同理对应位置路延时
        beta_velocity: float = 0.0,   # 速度路补偿力度 β_v ∈ [0,1]，1 = 按名义延时全补。大→更接近真实当前速度、模型错时放大误差；小→退化成不补
        beta_position: float = 0.0,   # 位置路补偿力度 β_p ∈ [0,1]，同上
        max_velocity_correction=DEFAULT_MAX_VELOCITY_CORRECTION,
        max_position_correction=DEFAULT_MAX_POSITION_CORRECTION,
        fallback_after_clamped_ticks: int = 10,  # 连续触顶多少拍就整体退回原始反馈。大→容忍偶发触顶但模型跑偏时纠正慢；小→轻微触顶就放弃预测
    ) -> None:
        n = int(n_joints)
        if n <= 0:
            raise ValueError(f"n_joints 必须为正，收到 {n_joints}")
        self.n_joints = n
        self.enabled = bool(enabled)

        for name, value in (("velocity_horizon_sec", velocity_horizon_sec),
                            ("position_horizon_sec", position_horizon_sec)):
            v = float(value)
            if not np.isfinite(v) or v < 0.0:
                raise ValueError(f"{name} 必须是非负有限值，收到 {value}")
            setattr(self, name, v)

        for name, value in (("beta_velocity", beta_velocity),
                            ("beta_position", beta_position)):
            v = float(value)
            if not np.isfinite(v) or not (0.0 <= v <= 1.0):


                raise ValueError(f"{name} 必须落在 [0,1]，收到 {value}")
            setattr(self, name, v)

        self.max_velocity_correction = self._limit_vector(
            "max_velocity_correction", max_velocity_correction, n)
        self.max_position_correction = self._limit_vector(
            "max_position_correction", max_position_correction, n)

        ticks = int(fallback_after_clamped_ticks)
        if ticks < 0:
            raise ValueError(
                f"fallback_after_clamped_ticks 不能为负，收到 {fallback_after_clamped_ticks}")
        self.fallback_after_clamped_ticks = ticks

    @staticmethod
    def _limit_vector(name, value, n):
        arr = np.asarray(value, dtype=float).ravel()
        if arr.size == 1:
            arr = np.full(n, float(arr[0]))
        if arr.size != n:
            raise ValueError(f"{name} 需要 {n} 个值或 1 个标量，收到 {arr.size}")
        if not np.all(np.isfinite(arr)):
            raise ValueError(f"{name} 含非有限值: {arr}")
        if np.any(arr < 0.0):
            raise ValueError(f"{name} 不能为负: {arr}")
        return arr

    @property
    def active(self) -> bool:
        return self.enabled and (self.beta_velocity > 0.0
                                 or self.beta_position > 0.0)

    def describe(self) -> str:
        return (
            "Smith 预测: %s  速度路 β=%.2f horizon %.1f ms，位置路 β=%.2f horizon %.1f ms；"
            "限幅 Δq̇<=%s rad/s、Δq<=%s rad；连续 %d 拍触顶即退回"
            % ("启用" if self.active else "关闭（逐位直通）",
               self.beta_velocity, 1000.0 * self.velocity_horizon_sec,
               self.beta_position, 1000.0 * self.position_horizon_sec,
               np.array2string(self.max_velocity_correction, precision=2),
               np.array2string(self.max_position_correction, precision=4),
               self.fallback_after_clamped_ticks))


# Smith 预测器：反馈链路有传输与滤波延时，读回的 (q, q̇) 是过去时刻的状态。
# 用已经发出、但反馈里还没体现的指令加速度往前推，把状态补到当前时刻。
# 缓冲最近若干拍的 (q̈_cmd, dt)，从最新一拍往回累加到给定时域 H：
#   Δv(H) = Σ q̈·Δt,   Δx(H) = Σ (Δv·Δt + ½·q̈·Δt²)
#   q̇̂ = q̇ + β_v·Δv(H_v)
#   q̂  = q + β_p·(H_p·q̇̂ + Δx(H_p))
# β ∈ [0,1] 是补偿力度（1 为按名义延时全补）。两路修正各自限幅；连续
# fallback_after_clamped_ticks 拍触顶说明模型与实际已经对不上，直接退回原始反馈。
class JointStatePredictor:

    def __init__(self, config: SmithPredictorConfig) -> None:
        self.config = config
        self.n_joints = config.n_joints


        depth = int(max(config.velocity_horizon_sec,
                        config.position_horizon_sec) / 0.005 + 0.5) + 8
        self._buf: deque = deque(maxlen=max(depth, 8))
        self._clamped_ticks = 0
        self.tripped = False
        self.last_diagnostics = {}


    @property
    def active(self) -> bool:
        return self.config.active and not self.tripped

    def reset(self) -> None:
        self._buf.clear()
        self._clamped_ticks = 0
        self.tripped = False
        self.last_diagnostics = {}


    def push_command(self, qdd_cmd, dt: float) -> None:
        dt = float(dt)
        if not np.isfinite(dt) or dt <= 0.0:

            return
        a = np.asarray(qdd_cmd, dtype=float).reshape(self.n_joints)
        self._buf.append((a.copy(), dt))


    def _integrate(self, horizon: float):
        dv = np.zeros(self.n_joints)
        dx = np.zeros(self.n_joints)
        elapsed = 0.0
        for a, dt in reversed(self._buf):
            if elapsed >= horizon:
                break
            step = min(dt, horizon - elapsed)

            dx += dv * step + 0.5 * a * step * step
            dv += a * step
            elapsed += step
        return dv, dx, elapsed

    def predict(self, position, velocity):
        cfg = self.config
        if not self.active:
            self.last_diagnostics = {"active": 0.0, "tripped": float(self.tripped)}
            return position, velocity

        q = np.asarray(position, dtype=float).reshape(self.n_joints)
        qd = np.asarray(velocity, dtype=float).reshape(self.n_joints)
        if not (np.all(np.isfinite(q)) and np.all(np.isfinite(qd))):

            self.last_diagnostics = {"active": 0.0, "feedback_nonfinite": 1.0}
            return position, velocity

        dv_v, _, span_v = self._integrate(cfg.velocity_horizon_sec)
        _, dx_p, span_p = self._integrate(cfg.position_horizon_sec)
        if not (np.all(np.isfinite(dv_v)) and np.all(np.isfinite(dx_p))):
            self.last_diagnostics = {"active": 0.0, "command_nonfinite": 1.0}
            return position, velocity


        corr_v = cfg.beta_velocity * dv_v


        qd_hat_raw = qd + corr_v
        corr_p = cfg.beta_position * (cfg.position_horizon_sec * qd_hat_raw + dx_p)

        clamp_v = np.clip(corr_v, -cfg.max_velocity_correction,
                          cfg.max_velocity_correction)
        clamp_p = np.clip(corr_p, -cfg.max_position_correction,
                          cfg.max_position_correction)
        hit = bool(np.any(clamp_v != corr_v) or np.any(clamp_p != corr_p))

        if hit:
            self._clamped_ticks += 1
            if (cfg.fallback_after_clamped_ticks > 0
                    and self._clamped_ticks >= cfg.fallback_after_clamped_ticks):

                self.tripped = True
                self.last_diagnostics = {
                    "active": 0.0, "tripped": 1.0,
                    "clamped_ticks": float(self._clamped_ticks)}
                return position, velocity
        else:
            self._clamped_ticks = 0

        self.last_diagnostics = {
            "active": 1.0,
            "tripped": 0.0,
            "clamped": 1.0 if hit else 0.0,
            "clamped_ticks": float(self._clamped_ticks),
            "velocity_span_sec": float(span_v),
            "position_span_sec": float(span_p),
            "max_velocity_correction_used": float(np.max(np.abs(clamp_v))),
            "max_position_correction_used": float(np.max(np.abs(clamp_p))),
        }
        return q + clamp_p, qd + clamp_v
