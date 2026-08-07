from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .pose_error import PoseState, compute_pose_error
from .quaternion_math import so3_left_jacobian, so3_left_jacobian_inverse_dot


def diag(values: np.ndarray | list[float]) -> np.ndarray:
    return np.diag(np.asarray(values, dtype=float).reshape(3))


@dataclass
class SMCGains:
    lambda_p: np.ndarray
    k_p: np.ndarray
    eta_p: np.ndarray
    phi_p: np.ndarray
    lambda_R: np.ndarray
    k_R: np.ndarray
    eta_R: np.ndarray
    phi_R: np.ndarray


@dataclass
class SMCCommand:
    nu: np.ndarray
    position_sliding: np.ndarray
    orientation_sliding: np.ndarray
    position_error: np.ndarray
    orientation_error: np.ndarray
    orientation_error_rate: np.ndarray
    omega_error: np.ndarray


# 任务空间滑模控制。位置与姿态两个通道各自定义滑模面，输出 6 维期望加速度
# ν = [a_cmd, α_cmd]，交给下游的动力学求逆映射成关节力矩。
#
# 位置通道（误差 e_p = p_d − p，ė_p = v_d − v）：
#   s_p   = ė_p + Λ_p·e_p
#   a_cmd = a_d + Λ_p·ė_p + K_p·s_p + η_p·tanh(s_p/φ_p)
#   代回 ṡ_p = ë_p + Λ_p·ė_p 得 ṡ_p = −K_p·s_p − η_p·tanh(s_p/φ_p)，s_p → 0 后
#   e_p 按 Λ_p 指数收敛。
#
# 姿态通道（误差 e_R = log(q_d ⊗ q⁻¹)，ė_R = J_l⁻¹(e_R)·e_ω）：
#   s_R    = ė_R + Λ_R·e_R
#   ë_R*   = −Λ_R·ė_R − K_R·s_R − η_R·tanh(s_R/φ_R)      （同样给出 ṡ_R = −K_R·s_R − …）
#   由 ė_R = J_l⁻¹·e_ω 求导得 ë_R = J̇_l⁻¹·e_ω + J_l⁻¹·ė_ω，反解出期望的误差角加速度
#   ė_ω = J_l·(ë_R* − J̇_l⁻¹·e_ω)
#   再由 e_ω = ω_d − R_err·ω 求导得 ė_ω = α_d − e_ω × (R_err·ω) − R_err·α，反解
#   α_cmd = R_errᵀ·[ α_d − e_ω × (R_err·ω) − J_l·(ë_R* − J̇_l⁻¹·e_ω) ]
#
# 切换项用 tanh(s/φ) 代替 sign(s)：φ 是边界层宽度，避免有限采样率下的抖振。
class TaskSpaceSMC:
    def __init__(self, gains: SMCGains):
        self.gains = gains

    @staticmethod
    def _sat_tanh(phi: np.ndarray, s: np.ndarray) -> np.ndarray:
        width = np.maximum(np.asarray(phi, dtype=float).reshape(3), 1e-9)
        return np.tanh(s / width)

    def compute(self, desired: PoseState, actual: PoseState) -> SMCCommand:
        err = compute_pose_error(desired, actual)
        g = self.gains
        Lp = diag(g.lambda_p)
        Kp = diag(g.k_p)
        Etap = diag(g.eta_p)
        LR = diag(g.lambda_R)
        KR = diag(g.k_R)
        EtaR = diag(g.eta_R)

        a_d = np.zeros(3) if desired.linear_acceleration is None else np.asarray(desired.linear_acceleration, dtype=float).reshape(3)
        alpha_d = np.zeros(3) if desired.angular_acceleration is None else np.asarray(desired.angular_acceleration, dtype=float).reshape(3)
        omega = np.asarray(actual.angular_velocity, dtype=float).reshape(3)

        s_p = err.linear_velocity + Lp @ err.position
        a_cmd = a_d + Lp @ err.linear_velocity + Kp @ s_p + Etap @ self._sat_tanh(g.phi_p, s_p)

        s_R = err.orientation_rate + LR @ err.orientation
        e_R_ddot_star = -LR @ err.orientation_rate - KR @ s_R - EtaR @ self._sat_tanh(g.phi_R, s_R)
        Jl = so3_left_jacobian(err.orientation)
        Jl_inv_dot = so3_left_jacobian_inverse_dot(err.orientation, err.orientation_rate)
        alpha_cmd = err.R_error.T @ (
            alpha_d
            - np.cross(err.omega_error, err.R_error @ omega)
            - Jl @ (e_R_ddot_star - Jl_inv_dot @ err.omega_error)
        )

        return SMCCommand(
            nu=np.concatenate([a_cmd, alpha_cmd]),
            position_sliding=s_p,
            orientation_sliding=s_R,
            position_error=err.position,
            orientation_error=err.orientation,
            orientation_error_rate=err.orientation_rate,
            omega_error=err.omega_error,
        )
