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


# 任务空间位姿误差。位置通道取直接差，姿态通道走 SO(3) 对数映射：
#   e_p = p_d − p,                e_v = v_d − v
#   q_err = q_d ⊗ q⁻¹,            R_err = R(q_err)
#   e_R = log(q_err) ∈ R³         （对数映射，即等效轴角向量）
#   e_ω = ω_d − R_err·ω           （把实际角速度先转到期望系再作差）
#   ė_R = J_l⁻¹(e_R)·e_ω          （J_l 为 SO(3) 左雅可比，把角速度换算成 e_R 的导数）
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


# 四元数双覆盖：q 与 −q 表示同一姿态。若与上一拍点积为负则取反，
# 保证参考四元数序列连续，否则数值微分会在翻号处炸出一个尖峰。
def enforce_quaternion_sign_continuity(q_ref_wxyz: np.ndarray, q_prev_wxyz: np.ndarray | None) -> np.ndarray:
    q = np.asarray(q_ref_wxyz, dtype=float).reshape(4).copy()
    if q_prev_wxyz is not None and float(np.dot(q, np.asarray(q_prev_wxyz).reshape(4))) < 0.0:
        q = -q
    return q
