"""Strict world-frame pose error utilities."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .quaternion_math import (
    quat_to_rotation,
    quaternion_error,
    quaternion_log_error,
    so3_left_jacobian_inverse,
)


@dataclass
class PoseState:
    position: np.ndarray
    quaternion_wxyz: np.ndarray
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray
    linear_acceleration: np.ndarray | None = None
    angular_acceleration: np.ndarray | None = None


@dataclass
class PoseError:
    position: np.ndarray
    linear_velocity: np.ndarray
    orientation: np.ndarray
    orientation_rate: np.ndarray
    q_error_wxyz: np.ndarray
    R_error: np.ndarray
    omega_error: np.ndarray


def compute_pose_error(desired: PoseState, actual: PoseState) -> PoseError:
    p_d = np.asarray(desired.position, dtype=float).reshape(3)
    p = np.asarray(actual.position, dtype=float).reshape(3)
    v_d = np.asarray(desired.linear_velocity, dtype=float).reshape(3)
    v = np.asarray(actual.linear_velocity, dtype=float).reshape(3)
    w_d = np.asarray(desired.angular_velocity, dtype=float).reshape(3)
    w = np.asarray(actual.angular_velocity, dtype=float).reshape(3)

    q_err = quaternion_error(desired.quaternion_wxyz, actual.quaternion_wxyz)
    R_err = quat_to_rotation(q_err)
    e_R = quaternion_log_error(desired.quaternion_wxyz, actual.quaternion_wxyz)
    omega_error = w_d - R_err @ w
    e_R_dot = so3_left_jacobian_inverse(e_R) @ omega_error

    return PoseError(
        position=p_d - p,
        linear_velocity=v_d - v,
        orientation=e_R,
        orientation_rate=e_R_dot,
        q_error_wxyz=q_err,
        R_error=R_err,
        omega_error=omega_error,
    )


def enforce_quaternion_sign_continuity(q_ref_wxyz: np.ndarray, q_prev_wxyz: np.ndarray | None) -> np.ndarray:
    q = np.asarray(q_ref_wxyz, dtype=float).reshape(4).copy()
    if q_prev_wxyz is not None and float(np.dot(q, np.asarray(q_prev_wxyz).reshape(4))) < 0.0:
        q = -q
    return q
