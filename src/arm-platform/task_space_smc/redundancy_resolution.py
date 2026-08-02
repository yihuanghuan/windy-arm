"""Dynamic-consistent inverse and nullspace acceleration resolution."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class RedundancyResult:
    qdd_cmd: np.ndarray
    Jbar: np.ndarray
    N: np.ndarray
    sigma_min: float
    condition_number: float
    damping: float
    nullspace_leakage_norm: float
    task_error_after_projection: np.ndarray


def _diag(values: np.ndarray | list[float]) -> np.ndarray:
    v = np.asarray(values, dtype=float).reshape(-1)
    return np.diag(v)


def adaptive_damping(
    singular_values: np.ndarray,
    sigma_warn: float,
    sigma_critical: float,
    damping_min: float,
    damping_max: float,
) -> float:
    sigma_min = float(np.min(singular_values)) if singular_values.size else 0.0
    if sigma_warn <= sigma_critical:
        return float(damping_max if sigma_min <= sigma_critical else damping_min)
    if sigma_min >= sigma_warn:
        return float(damping_min)
    if sigma_min <= sigma_critical:
        return float(damping_max)
    ratio = (sigma_warn - sigma_min) / (sigma_warn - sigma_critical)
    return float(damping_min + ratio * (damping_max - damping_min))


def dynamic_consistent_inverse(
    J_arm: np.ndarray,
    M_aa: np.ndarray,
    characteristic_length: float,
    damping: float = 0.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    J = np.asarray(J_arm, dtype=float).reshape(6, 7)
    M = np.asarray(M_aa, dtype=float).reshape(7, 7)
    scale = np.diag([1.0, 1.0, 1.0, characteristic_length, characteristic_length, characteristic_length])
    J_scaled = scale @ J
    X = np.linalg.solve(M, J_scaled.T)
    A = J_scaled @ X
    if damping > 0.0:
        A = A + (damping * damping) * np.eye(6)
    Jbar_scaled = X @ np.linalg.solve(A, np.eye(6))
    return Jbar_scaled, J_scaled, scale


def resolve_acceleration(
    J_arm: np.ndarray,
    M_aa: np.ndarray,
    task_rhs: np.ndarray,
    q_arm: np.ndarray,
    qd_arm: np.ndarray,
    q_nominal: np.ndarray,
    kp_null: np.ndarray,
    kd_null: np.ndarray,
    characteristic_length: float,
    sigma_warn: float,
    sigma_critical: float,
    damping_min: float,
    damping_max: float,
    joint_acceleration_weight: np.ndarray | list[float] | None = None,
) -> RedundancyResult:
    J = np.asarray(J_arm, dtype=float).reshape(6, 7)
    M = np.asarray(M_aa, dtype=float).reshape(7, 7)
    if joint_acceleration_weight is not None:
        M = M + _diag(joint_acceleration_weight)
    y = np.asarray(task_rhs, dtype=float).reshape(6)
    scale = np.diag([1.0, 1.0, 1.0, characteristic_length, characteristic_length, characteristic_length])
    J_scaled = scale @ J
    y_scaled = scale @ y
    singular_values = np.linalg.svd(J_scaled, compute_uv=False)
    sigma_min = float(np.min(singular_values)) if singular_values.size else 0.0
    sigma_max = float(np.max(singular_values)) if singular_values.size else 0.0
    condition_number = float("inf") if sigma_min <= 0.0 else sigma_max / sigma_min
    damping = adaptive_damping(singular_values, sigma_warn, sigma_critical, damping_min, damping_max)
    Jbar, _, _ = dynamic_consistent_inverse(J, M, characteristic_length, damping=damping)
    q = np.asarray(q_arm, dtype=float).reshape(7)
    qd = np.asarray(qd_arm, dtype=float).reshape(7)
    q_nom = np.asarray(q_nominal, dtype=float).reshape(7)
    qdd_null = -_diag(kp_null) @ (q - q_nom) - _diag(kd_null) @ qd
    N = np.eye(7) - Jbar @ J_scaled
    qdd_task = Jbar @ y_scaled
    qdd_cmd = qdd_task + N @ qdd_null
    leakage = J_scaled @ N
    projected = J @ qdd_cmd
    return RedundancyResult(
        qdd_cmd=qdd_cmd,
        Jbar=Jbar,
        N=N,
        sigma_min=sigma_min,
        condition_number=condition_number,
        damping=damping,
        nullspace_leakage_norm=float(np.linalg.norm(leakage, ord=2)),
        task_error_after_projection=projected - y,
    )
