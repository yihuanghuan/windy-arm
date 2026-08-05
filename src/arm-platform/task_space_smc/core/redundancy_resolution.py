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

    feasibility_scale: float = 1.0
    nullspace_scale: float = 1.0


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


    J = np.asarray(J_arm, dtype=float)
    if J.ndim != 2 or J.shape[0] not in (5, 6):
        raise ValueError(f"J_arm must be (5|6, n_arm), got {J.shape}")
    task_dim = J.shape[0]
    n = J.shape[1]
    M = np.asarray(M_aa, dtype=float).reshape(n, n)
    scale = np.diag([1.0, 1.0, 1.0] + [characteristic_length] * (task_dim - 3))
    J_scaled = scale @ J
    X = np.linalg.solve(M, J_scaled.T)
    A = J_scaled @ X
    if damping > 0.0:
        A = A + (damping * damping) * np.eye(task_dim)
    Jbar_scaled = X @ np.linalg.solve(A, np.eye(task_dim))
    return Jbar_scaled, J_scaled, scale


def pen_axis_task(J_arm, task_rhs_6, ee_rotation, axis_local=(1.0, 0.0, 0.0)):
    J = np.asarray(J_arm, dtype=float)
    if J.ndim != 2 or J.shape[0] != 6:
        raise ValueError(f"J_arm must be (6, n_arm), got {J.shape}")
    y = np.asarray(task_rhs_6, dtype=float).reshape(6)
    R = np.asarray(ee_rotation, dtype=float).reshape(3, 3)
    a = R @ np.asarray(axis_local, dtype=float).reshape(3)
    norm = np.linalg.norm(a)
    if norm < 1e-9:
        raise ValueError("pen axis is degenerate")
    a = a / norm
    P = np.eye(3) - np.outer(a, a)


    U, S, _ = np.linalg.svd(P)
    B = U[:, :2].T
    J5 = np.vstack([J[:3, :], B @ J[3:, :]])
    y5 = np.concatenate([y[:3], B @ y[3:]])
    return J5, y5, a


def pen_axis_error(R_actual, R_desired, axis_local=(1.0, 0.0, 0.0)):
    a = np.asarray(R_actual, dtype=float).reshape(3, 3) @ np.asarray(
        axis_local, dtype=float).reshape(3)
    d = np.asarray(R_desired, dtype=float).reshape(3, 3) @ np.asarray(
        axis_local, dtype=float).reshape(3)
    a = a / max(np.linalg.norm(a), 1e-12)
    d = d / max(np.linalg.norm(d), 1e-12)
    return float(np.arccos(np.clip(a @ d, -1.0, 1.0)))


def _undamped_inverse(J_arm, M_aa, characteristic_length, rcond=1e-10):
    J = np.asarray(J_arm, dtype=float)
    task_dim, n = J.shape
    M = np.asarray(M_aa, dtype=float).reshape(n, n)
    scale = np.diag([1.0, 1.0, 1.0] + [characteristic_length] * (task_dim - 3))
    J_scaled = scale @ J
    X = np.linalg.solve(M, J_scaled.T)
    A = J_scaled @ X
    u, s, vt = np.linalg.svd(A)
    keep = s > max(rcond * (s[0] if s.size else 0.0), 1e-300)
    s_inv = np.zeros_like(s)
    s_inv[keep] = 1.0 / s[keep]
    return X @ (vt.T * s_inv) @ u.T


@dataclass
class LimitAvoidance:
    lower: np.ndarray
    upper: np.ndarray
    gain: float = 0.10
    activate_rad: float = 0.5


def _limit_gradient(q, lower, upper, activate):
    q = np.asarray(q, dtype=float)
    lo = np.asarray(lower, dtype=float)
    hi = np.asarray(upper, dtype=float)
    g = np.zeros_like(q)
    dl = q - lo
    du = hi - q
    near_lo = dl < activate
    near_hi = du < activate
    g[near_lo] += (activate - dl[near_lo]) / activate
    g[near_hi] -= (activate - du[near_hi]) / activate
    return g


@dataclass
class FeasibilityLimits:

    velocity: np.ndarray
    lower: np.ndarray
    upper: np.ndarray
    dt: float
    torque: np.ndarray | None = None
    brake_safety: float = 0.5


def _feasible_acceleration_box(q, qd, M, limits: FeasibilityLimits):
    q = np.asarray(q, dtype=float).ravel()
    qd = np.asarray(qd, dtype=float).ravel()
    v = np.abs(np.asarray(limits.velocity, dtype=float).ravel())
    dt = max(float(limits.dt), 1e-6)
    v_hi = v.copy()
    v_lo = -v.copy()
    if limits.torque is not None:
        tau = np.abs(np.asarray(limits.torque, dtype=float).ravel())
        m_diag = np.maximum(np.abs(np.diag(np.asarray(M, dtype=float))), 1e-9)
        a_brake = np.maximum(float(limits.brake_safety) * tau / m_diag, 0.0)
        margin_hi = np.maximum(
            np.asarray(limits.upper, dtype=float).ravel() - q, 0.0)
        margin_lo = np.maximum(
            q - np.asarray(limits.lower, dtype=float).ravel(), 0.0)
        v_hi = np.minimum(v_hi, np.sqrt(2.0 * a_brake * margin_hi))
        v_lo = np.maximum(v_lo, -np.sqrt(2.0 * a_brake * margin_lo))
    return (v_lo - qd) / dt, (v_hi - qd) / dt


def _uniform_scale(u, lo, hi):
    u = np.asarray(u, dtype=float).ravel()
    eps = 1e-12
    ratios = np.ones_like(u)
    pos = u > eps
    neg = u < -eps
    ratios[pos] = hi[pos] / u[pos]
    ratios[neg] = lo[neg] / u[neg]
    return float(np.clip(min(1.0, float(np.min(ratios)) if u.size else 1.0),
                         0.0, 1.0))


def _damped_inverse_on_columns(J_scaled, M, columns, damping):
    Jf = J_scaled[:, columns]
    Mf = M[np.ix_(columns, columns)]
    X = np.linalg.solve(Mf, Jf.T)
    A = Jf @ X
    if damping > 0.0:
        A = A + (damping * damping) * np.eye(A.shape[0])
    return X @ np.linalg.solve(A, np.eye(A.shape[0]))


def _task_scaling_factor(a, b, lo, hi, tol=1e-9):
    s_lo, s_hi = -np.inf, np.inf
    for ai, bi, li, ui in zip(a, b, lo, hi):
        if abs(ai) < 1e-12:
            if bi < li - tol or bi > ui + tol:
                return 0.0
            continue
        l = (li - bi) / ai
        u = (ui - bi) / ai
        if ai < 0.0:
            l, u = u, l
        s_lo = max(s_lo, l)
        s_hi = min(s_hi, u)
    if s_lo > s_hi + tol:
        return 0.0
    return float(np.clip(min(s_hi, 1.0), 0.0, 1.0))


def _saturation_aware_task(J_scaled, M, y_scaled, damping, lo, hi):
    n = J_scaled.shape[1]
    task_dim = J_scaled.shape[0]
    free = np.ones(n, dtype=bool)
    qdd_sat = np.zeros(n)
    best_s = 0.0
    best_free = free.copy()
    best_sat = qdd_sat.copy()
    qdd = np.zeros(n)
    for _ in range(n + 1):
        Jinv = _damped_inverse_on_columns(J_scaled, M, free, damping)
        residual = y_scaled - J_scaled @ qdd_sat
        qdd = qdd_sat.copy()
        qdd[free] = qdd_sat[free] + Jinv @ residual

        a = np.zeros(n)
        a[free] = Jinv @ y_scaled
        b = qdd - a
        s = _task_scaling_factor(a, b, lo, hi)
        if s > best_s:
            best_s, best_free, best_sat = s, free.copy(), qdd_sat.copy()
        violation = np.maximum(qdd - hi, lo - qdd)
        violation[~free] = -np.inf
        if np.max(violation) <= 1e-9:
            return qdd, 1.0, int(n - free.sum())
        j = int(np.argmax(violation))
        free[j] = False
        qdd_sat[j] = float(np.clip(qdd[j], lo[j], hi[j]))
        if int(free.sum()) < task_dim:
            free, qdd_sat = best_free, best_sat
            Jinv = _damped_inverse_on_columns(J_scaled, M, free, damping)
            qdd = qdd_sat.copy()
            qdd[free] = qdd_sat[free] + Jinv @ (
                best_s * y_scaled - J_scaled @ qdd_sat)
            return np.clip(qdd, lo, hi), best_s, int(n - free.sum())
    return np.clip(qdd, lo, hi), best_s, int(n - free.sum())


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
    limit_avoidance: 'LimitAvoidance | None' = None,
    feasibility: 'FeasibilityLimits | None' = None,
) -> RedundancyResult:


    J = np.asarray(J_arm, dtype=float)
    if J.ndim != 2 or J.shape[0] not in (5, 6):
        raise ValueError(f"J_arm must be (5|6, n_arm), got {J.shape}")
    task_dim = J.shape[0]
    n = J.shape[1]
    M = np.asarray(M_aa, dtype=float).reshape(n, n)
    if joint_acceleration_weight is not None:
        M = M + _diag(joint_acceleration_weight)
    y = np.asarray(task_rhs, dtype=float).reshape(task_dim)
    n_rot = task_dim - 3
    scale = np.diag([1.0, 1.0, 1.0] + [characteristic_length] * n_rot)
    J_scaled = scale @ J
    y_scaled = scale @ y
    singular_values = np.linalg.svd(J_scaled, compute_uv=False)
    sigma_min = float(np.min(singular_values)) if singular_values.size else 0.0
    sigma_max = float(np.max(singular_values)) if singular_values.size else 0.0
    condition_number = float("inf") if sigma_min <= 0.0 else sigma_max / sigma_min
    damping = adaptive_damping(singular_values, sigma_warn, sigma_critical, damping_min, damping_max)
    Jbar, _, _ = dynamic_consistent_inverse(J, M, characteristic_length, damping=damping)


    q = np.asarray(q_arm, dtype=float).ravel()
    n = q.size
    qd = np.asarray(qd_arm, dtype=float).reshape(n)
    q_nom = np.asarray(q_nominal, dtype=float).reshape(n)
    qdd_null = -_diag(kp_null) @ (q - q_nom) - _diag(kd_null) @ qd


    if limit_avoidance is not None and limit_avoidance.gain > 0.0:
        qdd_null = qdd_null + limit_avoidance.gain * _limit_gradient(
            q, limit_avoidance.lower, limit_avoidance.upper,
            limit_avoidance.activate_rad)


    Jbar0 = _undamped_inverse(J, M, characteristic_length)
    N = np.eye(n) - Jbar0 @ J_scaled
    qdd_task = Jbar @ y_scaled
    qdd_null_projected = N @ qdd_null


    alpha = 1.0
    gamma = 1.0
    if feasibility is not None:
        lo, hi = _feasible_acceleration_box(q, qd, M, feasibility)
        qdd_task, alpha, _ = _saturation_aware_task(
            J_scaled, M, y_scaled, damping, lo, hi)
        gamma = _uniform_scale(qdd_null_projected, lo - qdd_task, hi - qdd_task)
        qdd_null_projected = gamma * qdd_null_projected
    qdd_cmd = qdd_task + qdd_null_projected
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
        feasibility_scale=alpha,
        nullspace_scale=gamma,
    )
