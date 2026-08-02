"""World-frame whole-task reference helpers shared by ROS and offline runs."""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .quaternion_math import rotation_to_quat, skew


def _import_moving_base_trajectory_config():
    here = Path(__file__).resolve()
    candidates = [
        here.parent.parent / "demo",
        here.parent.parent,
        here.parent.parent.parent / "demo",
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


def _vee(S: np.ndarray) -> np.ndarray:
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


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


def drawing_segments(shape: str = "whole", sample_count: int = 4000) -> list[np.ndarray]:
    return [np.asarray(seg, dtype=float) for seg in traj.sample_ee_drawing_segments(shape, sample_count)]
