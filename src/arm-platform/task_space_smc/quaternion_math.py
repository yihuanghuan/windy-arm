"""Quaternion and SO(3) helpers used by the task-space SMC controller.

Internal quaternions are always stored as [w, x, y, z].  ROS messages use
[x, y, z, w] and must be converted at the interface boundary.
"""

from __future__ import annotations

import math

import numpy as np


EPS = 1e-12


def _as_vec3(v: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
    out = np.asarray(v, dtype=float).reshape(3)
    if not np.all(np.isfinite(out)):
        raise ValueError("vector contains NaN/Inf")
    return out


def skew(v: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
    x, y, z = _as_vec3(v)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def quat_normalize(q: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(4)
    if not np.all(np.isfinite(q)):
        raise ValueError("quaternion contains NaN/Inf")
    n = np.linalg.norm(q)
    if n < EPS:
        raise ValueError("zero quaternion is invalid")
    return q / n


def ros_xyzw_to_wxyz(q_xyzw: np.ndarray | list[float]) -> np.ndarray:
    q = np.asarray(q_xyzw, dtype=float).reshape(4)
    return quat_normalize(np.array([q[3], q[0], q[1], q[2]]))


def wxyz_to_ros_xyzw(q_wxyz: np.ndarray | list[float]) -> np.ndarray:
    q = quat_normalize(q_wxyz)
    return np.array([q[1], q[2], q[3], q[0]])


def quat_conjugate(q: np.ndarray | list[float]) -> np.ndarray:
    q = quat_normalize(q)
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_multiply(q1: np.ndarray | list[float], q2: np.ndarray | list[float]) -> np.ndarray:
    w1, x1, y1, z1 = quat_normalize(q1)
    w2, x2, y2, z2 = quat_normalize(q2)
    return quat_normalize(
        np.array(
            [
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            ]
        )
    )


def quat_to_rotation(q: np.ndarray | list[float]) -> np.ndarray:
    w, x, y, z = quat_normalize(q)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def rotation_to_quat(R: np.ndarray) -> np.ndarray:
    R = np.asarray(R, dtype=float).reshape(3, 3)
    tr = float(np.trace(R))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return quat_normalize(
            np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
        )
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) * 2.0
        q = np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    elif i == 1:
        s = math.sqrt(max(0.0, 1.0 + R[1, 1] - R[0, 0] - R[2, 2])) * 2.0
        q = np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s])
    else:
        s = math.sqrt(max(0.0, 1.0 + R[2, 2] - R[0, 0] - R[1, 1])) * 2.0
        q = np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s])
    return quat_normalize(q)


def quaternion_error(q_desired_wxyz: np.ndarray | list[float], q_actual_wxyz: np.ndarray | list[float]) -> np.ndarray:
    q_err = quat_multiply(q_desired_wxyz, quat_conjugate(q_actual_wxyz))
    if q_err[0] < 0.0:
        q_err = -q_err
    return quat_normalize(q_err)


def quaternion_log_error(q_desired_wxyz: np.ndarray | list[float], q_actual_wxyz: np.ndarray | list[float]) -> np.ndarray:
    q_err = quaternion_error(q_desired_wxyz, q_actual_wxyz)
    eta = float(np.clip(q_err[0], -1.0, 1.0))
    eps = q_err[1:]
    eps_norm = float(np.linalg.norm(eps))
    if eps_norm < 1e-10:
        return 2.0 * eps
    angle = 2.0 * math.atan2(eps_norm, eta)
    return angle * eps / eps_norm


def so3_left_jacobian(phi: np.ndarray | list[float]) -> np.ndarray:
    phi = _as_vec3(phi)
    theta = float(np.linalg.norm(phi))
    K = skew(phi)
    K2 = K @ K
    if theta < 1e-8:
        return np.eye(3) + 0.5 * K + (1.0 / 6.0) * K2
    a = (1.0 - math.cos(theta)) / (theta * theta)
    b = (theta - math.sin(theta)) / (theta**3)
    return np.eye(3) + a * K + b * K2


def _left_jacobian_inverse_A(theta: float) -> float:
    if theta < 1e-6:
        return 1.0 / 12.0 + theta * theta / 720.0 + theta**4 / 30240.0
    return 1.0 / (theta * theta) - (1.0 / math.tan(0.5 * theta)) / (2.0 * theta)


def _left_jacobian_inverse_A_dot(theta: float, theta_dot: float) -> float:
    if theta < 1e-6:
        dA_dtheta = theta / 360.0 + theta**3 / 7560.0
    else:
        half = 0.5 * theta
        cot_half = 1.0 / math.tan(half)
        csc2_half = 1.0 / (math.sin(half) ** 2)
        dA_dtheta = -2.0 / (theta**3) + csc2_half / (4.0 * theta) + cot_half / (2.0 * theta * theta)
    return dA_dtheta * theta_dot


def so3_left_jacobian_inverse(phi: np.ndarray | list[float]) -> np.ndarray:
    phi = _as_vec3(phi)
    theta = float(np.linalg.norm(phi))
    K = skew(phi)
    A = _left_jacobian_inverse_A(theta)
    return np.eye(3) - 0.5 * K + A * (K @ K)


def so3_left_jacobian_inverse_dot(phi: np.ndarray | list[float], phi_dot: np.ndarray | list[float]) -> np.ndarray:
    phi = _as_vec3(phi)
    phi_dot = _as_vec3(phi_dot)
    theta = float(np.linalg.norm(phi))
    theta_dot = 0.0 if theta < EPS else float(phi @ phi_dot) / theta
    K = skew(phi)
    K_dot = skew(phi_dot)
    A = _left_jacobian_inverse_A(theta)
    A_dot = _left_jacobian_inverse_A_dot(theta, theta_dot)
    return -0.5 * K_dot + A_dot * (K @ K) + A * (K_dot @ K + K @ K_dot)


def geodesic_angle_from_error(e_R: np.ndarray | list[float]) -> float:
    return float(np.linalg.norm(_as_vec3(e_R)))
