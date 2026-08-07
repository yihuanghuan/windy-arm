from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .quaternion_math import rotation_to_quat, skew


def _import_moving_base_trajectory_config():
    here = Path(__file__).resolve()
    candidates = [
        here.parents[2] / "demo",
        here.parents[2],
        here.parents[3] / "demo",
    ]
    for path in candidates:
        if path.exists() and str(path) not in sys.path:
            sys.path.insert(0, str(path))
    import moving_base_trajectory_config as traj

    return traj


traj = _import_moving_base_trajectory_config()


@dataclass
class ReferenceSample:
    position_w: np.ndarray
    quaternion_wxyz: np.ndarray
    rotation_w: np.ndarray
    linear_velocity_w: np.ndarray
    angular_velocity_w: np.ndarray
    linear_acceleration_w: np.ndarray
    angular_acceleration_w: np.ndarray
    pen_down: bool
    stroke_id: int


@dataclass
class AnalyticBaseSample:
    position_w: np.ndarray
    quaternion_wxyz: np.ndarray
    rotation_wb: np.ndarray
    velocity_lwa: np.ndarray
    acceleration_lwa: np.ndarray


# 反对称矩阵取回向量，[v]× 的逆运算
def _vee(S: np.ndarray) -> np.ndarray:
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


# 末端参考轨迹采样。轨迹本身只给位姿，速度与加速度用中心差分补出来：
#   ḟ = (f(t+h) − f(t−h))/(2h),    f̈ = (f(t+h) − 2f(t) + f(t−h))/h²
# 角速度由 W = Ṙ·Rᵀ 取 vee；Ẇ = R̈·Rᵀ + Ṙ·Ṙᵀ。
# 差分误差会让 W 带上一点非反对称分量，先取 (W − Wᵀ)/2 投影回 so(3) 再 vee。
# 全部量都在世界系。
def reference_sample(t: float, shape: str = "whole", derivative_dt: float = 0.005) -> ReferenceSample:
    h = float(derivative_dt)
    sample = traj.ee_trajectory_sample(float(t), shape)
    minus = traj.ee_trajectory_sample(float(t) - h, shape)
    plus = traj.ee_trajectory_sample(float(t) + h, shape)
    p = np.asarray(sample.position, dtype=float).reshape(3)
    pm = np.asarray(minus.position, dtype=float).reshape(3)
    pp = np.asarray(plus.position, dtype=float).reshape(3)
    R = np.asarray(sample.rotation, dtype=float).reshape(3, 3)
    Rm = np.asarray(minus.rotation, dtype=float).reshape(3, 3)
    Rp = np.asarray(plus.rotation, dtype=float).reshape(3, 3)
    Rd = (Rp - Rm) / (2.0 * h)
    Rdd = (Rp - 2.0 * R + Rm) / (h * h)
    W = Rd @ R.T
    W = 0.5 * (W - W.T)
    Wd = Rdd @ R.T + Rd @ Rd.T
    Wd = 0.5 * (Wd - Wd.T)
    return ReferenceSample(
        position_w=p,
        quaternion_wxyz=rotation_to_quat(R),
        rotation_w=R,
        linear_velocity_w=(pp - pm) / (2.0 * h),
        angular_velocity_w=_vee(W),
        linear_acceleration_w=(pp - 2.0 * p + pm) / (h * h),
        angular_acceleration_w=_vee(Wd),
        pen_down=bool(sample.pen_down),
        stroke_id=int(sample.stroke_id),
    )


# 解析基座轨迹采样。位姿同样用中心差分求导，但输出换算到基座体坐标系
# （pinocchio 自由飞行关节的 v/a 就要这个系）：
#   W_body = Rᵀ·Ṙ,              ω_body = vee(W_body)
#   Ẇ_body = Rᵀ·R̈ − W_body²,   α_body = vee(Ẇ_body)
#   v_body = Rᵀ·ṗ_w
#   a_body = Rᵀ·p̈_w − ω_body × v_body     （体系下线速度求导多出的科氏项）
def base_sample(t: float, shape: str = "whole", path_t: float | None = None, derivative_dt: float = 0.005) -> AnalyticBaseSample:
    h = float(derivative_dt)
    path_t = float(t) if path_t is None else float(path_t)
    p = np.asarray(traj.base_pose(float(t)), dtype=float).reshape(3)
    pm = np.asarray(traj.base_pose(float(t) - h), dtype=float).reshape(3)
    pp = np.asarray(traj.base_pose(float(t) + h), dtype=float).reshape(3)
    R = np.asarray(traj.base_rotation(float(t), shape, path_t), dtype=float).reshape(3, 3)
    Rm = np.asarray(traj.base_rotation(float(t) - h, shape, path_t - h), dtype=float).reshape(3, 3)
    Rp = np.asarray(traj.base_rotation(float(t) + h, shape, path_t + h), dtype=float).reshape(3, 3)
    Rd = (Rp - Rm) / (2.0 * h)
    Rdd = (Rp - 2.0 * R + Rm) / (h * h)
    W_body = R.T @ Rd
    W_body = 0.5 * (W_body - W_body.T)
    W_body_dot = R.T @ Rdd - W_body @ W_body
    W_body_dot = 0.5 * (W_body_dot - W_body_dot.T)
    omega_body = _vee(W_body)
    p_dot_w = (pp - pm) / (2.0 * h)
    p_ddot_w = (pp - 2.0 * p + pm) / (h * h)
    v_local = R.T @ p_dot_w
    a_local = R.T @ p_ddot_w - np.cross(omega_body, v_local)
    return AnalyticBaseSample(
        position_w=p,
        quaternion_wxyz=rotation_to_quat(R),
        rotation_wb=R,
        velocity_lwa=np.concatenate([v_local, omega_body]),
        acceleration_lwa=np.concatenate([a_local, _vee(W_body_dot)]),
    )


def whole_period() -> float:
    return float(traj.ee_trajectory_period("whole"))


def set_hold_reference(position_w, rotation_w=None) -> None:
    """把 ``hold`` 形状的常量参考位姿设为给定的世界系位姿。

    ``hold`` 的参考量与时间无关，目标位姿只能由外部给定一次。
    """
    traj.set_hold_pose(position_w, rotation_w)


def hold_reference() -> tuple[np.ndarray, np.ndarray]:
    """返回当前 ``hold`` 参考位姿 (position_w, rotation_w)。"""
    position, rotation = traj.hold_pose()
    return (np.asarray(position, dtype=float).reshape(3),
            np.asarray(rotation, dtype=float).reshape(3, 3))


def drawing_segments(shape: str = "whole", sample_count: int = 4000) -> list[np.ndarray]:
    return [np.asarray(seg, dtype=float) for seg in traj.sample_ee_drawing_segments(shape, sample_count)]
