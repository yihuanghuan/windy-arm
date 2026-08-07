from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .pose_error import PoseState
from .quaternion_math import rotation_to_quat


# 五次插值多项式及其对 u 的一、二阶导：
#   s(u) = 10u³ − 15u⁴ + 6u⁵
# 满足 s(0)=0、s(1)=1，且两端一阶、二阶导都为零 —— 起停时速度与加速度均为零。
def quintic(u: float) -> tuple[float, float, float]:
    u = float(min(max(u, 0.0), 1.0))
    s = 10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5
    ds = 30.0 * u ** 2 - 60.0 * u ** 3 + 30.0 * u ** 4
    dds = 60.0 * u - 180.0 * u ** 2 + 120.0 * u ** 3
    return s, ds, dds


# SO(3) 对数映射，经四元数中转：q = (w, v)，取 w ≥ 0 的短程分支，
#   ω = 2·atan2(‖v‖, w)/‖v‖ · v
def rotation_log(R: np.ndarray) -> np.ndarray:
    q = rotation_to_quat(np.asarray(R, dtype=float).reshape(3, 3))
    w = float(q[0])
    v = np.asarray(q[1:], dtype=float)
    if w < 0.0:
        w, v = -w, -v
    n = float(np.linalg.norm(v))
    if n < 1e-15:
        return np.zeros(3)

    return (2.0 * np.arctan2(n, w) / n) * v


# SO(3) 指数映射（Rodrigues）：R = I + sinθ·[n]× + (1 − cosθ)·[n]×²,  n = φ/θ
def rotation_exp(phi: np.ndarray) -> np.ndarray:
    phi = np.asarray(phi, dtype=float).reshape(3)
    theta = float(np.linalg.norm(phi))
    if theta < 1e-12:
        return np.eye(3)
    k = phi / theta
    K = np.array([[0.0, -k[2], k[1]],
                  [k[2], 0.0, -k[0]],
                  [-k[1], k[0], 0.0]])
    return (np.eye(3) + np.sin(theta) * K
            + (1.0 - np.cos(theta)) * (K @ K))


@dataclass
class ApproachPlan:
    distance_m: float
    rotation_deg: float
    duration_sec: float
    peak_linear_speed: float
    peak_angular_speed: float
    peak_linear_accel: float


# 就位段的任务空间参考轨迹：从起点位姿到目标位姿走一条五次时间律的测地线。
#   p(t) = p0 + s·Δp,                Δp = p1 − p0
#   R(t) = R0·exp(s·Δr),             Δr = log(R0ᵀ·R1)   （在起点体系下的转轴）
#   v = ṡ·Δp,   ω = ṡ·(R0·Δr),      a = s̈·Δp,   α = s̈·(R0·Δr)
# 姿态走单轴等速转，故角速度方向恒定，可直接用同一个 ṡ 缩放。
class TaskSpaceApproach:

    # s'(u) 与 s''(u) 的极值，用于事前估算峰值速度/加速度：
    #   max s'  = s'(1/2)          = 15/8 = 1.875
    #   max s'' = s''(1/2 − √3/6)  = 10/√3
    PEAK_DS = 1.875
    PEAK_DDS = 10.0 / np.sqrt(3.0)

    def __init__(self, start_position, start_rotation,
                 goal_position, goal_rotation, duration_sec: float):
        self.p0 = np.asarray(start_position, dtype=float).reshape(3)
        self.R0 = np.asarray(start_rotation, dtype=float).reshape(3, 3)
        self.p1 = np.asarray(goal_position, dtype=float).reshape(3)
        self.R1 = np.asarray(goal_rotation, dtype=float).reshape(3, 3)
        self.duration_sec = float(duration_sec)
        self.dp = self.p1 - self.p0

        self.dr = rotation_log(self.R0.T @ self.R1)
        self.axis_w = self.R0 @ self.dr

    def plan(self) -> ApproachPlan:
        T = max(self.duration_sec, 1e-9)
        d = float(np.linalg.norm(self.dp))
        a = float(np.linalg.norm(self.dr))
        return ApproachPlan(
            distance_m=d,
            rotation_deg=float(np.degrees(a)),
            duration_sec=self.duration_sec,
            peak_linear_speed=self.PEAK_DS * d / T,
            peak_angular_speed=self.PEAK_DS * a / T,
            peak_linear_accel=self.PEAK_DDS * d / (T * T),
        )

    def finished(self, elapsed: float) -> bool:
        return float(elapsed) >= self.duration_sec

    def sample(self, elapsed: float) -> PoseState:
        T = self.duration_sec
        if T <= 0.0:

            return PoseState(
                position=self.p1.copy(),
                quaternion_wxyz=rotation_to_quat(self.R1),
                linear_velocity=np.zeros(3),
                angular_velocity=np.zeros(3),
                linear_acceleration=np.zeros(3),
                angular_acceleration=np.zeros(3),
            )
        s, ds_du, dds_du = quintic(float(elapsed) / T)
        ds = ds_du / T
        dds = dds_du / (T * T)
        R = self.R0 @ rotation_exp(s * self.dr)
        return PoseState(
            position=self.p0 + s * self.dp,
            quaternion_wxyz=rotation_to_quat(R),
            linear_velocity=ds * self.dp,
            angular_velocity=ds * self.axis_w,
            linear_acceleration=dds * self.dp,
            angular_acceleration=dds * self.axis_w,
        )

    def sample_positions(self, count: int = 51) -> np.ndarray:
        n = max(2, int(count))
        s = np.array([quintic(u)[0] for u in np.linspace(0.0, 1.0, n)])
        return self.p0[None, :] + s[:, None] * self.dp[None, :]
