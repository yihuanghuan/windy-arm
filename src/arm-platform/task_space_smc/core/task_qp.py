"""位置严格、姿态可让的保持任务，整合成一个带约束 QP 求解。

RNEA 对关节加速度是仿射的，两个分量每拍本来就要算：

    tau(qdd) = c + M qdd
      c = tau_bias + tau_friction_feedforward   （qdd = 0 的 RNEA，加上与 qdd
                                                 无关的静摩擦前馈项）
      M = 质量阵的手臂分块

所以力矩上限是 qdd 的线性约束，属于求解器内部，不是解完之后再截断。M 可逆，
关系双向成立，故直接以力矩为决策变量求解：

    决策变量 tau (n)，解完再取 qdd = M^-1 (tau - c)

优先级（高到低）：
    1. 力矩 + 力矩变化率     决策变量上的【硬】箱
    2. 末端位置             w_position
    3. 末端姿态             w_orientation      （不可行时先让这一项）
    4. qdd 箱               【软】箱，w_qdd_box_slack
                            （速度包线、限位制动、加速度上限）
    5. 回中 / 限位规避 / 平滑

硬箱选力矩而不是加速度：额定力矩是物理事实，速度包线与制动距离是策略。反过来
把加速度箱设成硬的，会在贴近限位且带速度时要求额定力矩产生不了的减速度，解出
超出额定的力矩，最终仍被适配器后端截断。软箱权重取得足够高，两者不冲突时包线
照样被严格遵守。

可行集非空是构造保证的：力矩箱按 build_torque_box 必为非空区间，且任一 tau 都
经 M^-1 映射到某个 qdd，故求解器不会硬失败。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from task_space_smc.core.dense_qp import OPTIMAL, solve_box_penalty_qp
from task_space_smc.core.redundancy_resolution import (
    FeasibilityLimits, LimitAvoidance, _feasible_acceleration_box,
    _limit_gradient, adaptive_damping)


@dataclass
class HoldQPWeights:
    # 代价 min w_p‖A_p q̈−y_p‖² + w_R‖A_R q̈−y_R‖² + w_n‖q̈−q̈_null‖² + w_d‖q̈−q̈_prev‖² + w_reg‖q̈‖²
    position: float = 1.0e4        # w_p，位置残差权重。大→位置更严格、其余项让位；小→位置精度让出去
    orientation: float = 1.0       # w_R，姿态残差权重（有效权重是 w_R·L²）。大→姿态跟得紧但与位置争自由度；小→姿态更快放弃
    joint_centering: float = 1.0e-2  # w_n，向零空间目标位形靠拢。大→位形稳但任务精度受损；小→位形自由漂移
    smoothness: float = 1.0e-2     # w_d，对上一拍 q̈ 的二次罚。大→指令平滑、响应变钝；小→跟得快但相邻拍跳变大
    regularization: float = 1.0e-6  # w_reg，Tikhonov 项保 H 正定。大→解更保守、有常态偏差；小→H 条件数变差
    qdd_box_slack: float = 1.0e8   # 加速度箱越界的罚权重（该箱软、力矩箱硬）。大→速度包线近乎严格；小→包线形同虚设
    characteristic_length: float = 0.2  # L (m)，把姿态折成长度量纲。大→姿态更被优先；小→位置更被优先


@dataclass
class HoldQPLimits:
    torque: np.ndarray
    torque_rate: np.ndarray
    qdd_max: np.ndarray
    dt: float
    q_lower: np.ndarray
    q_upper: np.ndarray
    velocity: np.ndarray | None = None
    brake_safety: float = 0.0


@dataclass
class HoldQPResult:
    qdd: np.ndarray
    tau: np.ndarray
    qdd_slack: np.ndarray          # 关节加速度箱的越界量（力矩箱是硬的，永不越界）
    position_residual: np.ndarray
    orientation_residual: np.ndarray
    qdd_box: tuple
    tau_box: tuple
    status: str
    iterations: int = 0
    active_count: int = 0
    solve_seconds: float = 0.0
    task_accel_scale: tuple = (1.0, 1.0)
    sigma_min: float = float("nan")
    damping: float = 0.0

    @property
    def ok(self) -> bool:
        return self.status == OPTIMAL


def build_qdd_box(q, qd, M, limits: HoldQPLimits):
    """关节加速度箱，合并速度包线、限位制动与加速度上限。

        lo = clip(-qdd_max, lo_v, hi_v),   hi = clip(+qdd_max, lo_v, hi_v)

    加速度上限是【裁进】速度/限位箱，而不是与之取交集：取交集在 qd 已经越出包线
    时会禁掉幅值超过 qdd_max 的恢复加速度，等于不许刹车；裁入则无条件保证
    lo <= hi，QP 永远拿不到空箱。
    """
    qdd_max = np.abs(np.asarray(limits.qdd_max, dtype=float).ravel())
    if limits.velocity is None:
        return -qdd_max, qdd_max
    feasibility = FeasibilityLimits(
        velocity=np.asarray(limits.velocity, dtype=float),
        lower=np.asarray(limits.q_lower, dtype=float),
        upper=np.asarray(limits.q_upper, dtype=float),
        dt=float(limits.dt),
        torque=(np.asarray(limits.torque, dtype=float)
                if limits.brake_safety > 0.0 else None),
        brake_safety=float(limits.brake_safety),
    )
    lo_v, hi_v = _feasible_acceleration_box(q, qd, M, feasibility)
    lo = np.clip(-qdd_max, lo_v, hi_v)
    hi = np.clip(qdd_max, lo_v, hi_v)
    # 关节越出包线时箱退化成一个点，上下界两行变成线性相关的等式对，正是主动集法
    # 最难处理的输入形状。展宽 1e-6 rad/s² 消掉退化，远低于本机可分辨的量级。
    return lo, np.maximum(hi, lo + 1e-6)


def build_torque_box(tau_prev, limits: HoldQPLimits):
    """额定力矩箱与从 tau_prev 出发一拍可达的变化率窗口取交：

        lo = max(-tau_max, tau_prev - rate*dt),  hi = min(+tau_max, tau_prev + rate*dt)

    tau_prev 先被裁进额定箱，否则上一拍若在额定箱外，整个变化率窗口可能落在箱外
    而得到空区间。tau_prev 必须是【适配器实际下发】的那个值，调用方还要复现适配器
    在指令超时后清零的规则；两边对变化率基准的理解不一致时，适配器会截断 QP 认为
    合法的指令。
    """
    tau_max = np.abs(np.asarray(limits.torque, dtype=float).ravel())
    rate = np.abs(np.asarray(limits.torque_rate, dtype=float).ravel())
    prev = np.clip(np.asarray(tau_prev, dtype=float).ravel(), -tau_max, tau_max)
    span = rate * max(float(limits.dt), 0.0)
    return np.maximum(-tau_max, prev - span), np.minimum(tau_max, prev + span)


def saturate_task_acceleration(nu, max_linear, max_angular):
    """限制任务空间指令加速度的幅值，方向不变：

        线速度  s_lin = min(1, max_linear / ||nu[:3]||)
        角速度  s_ang = min(1, max_angular / ||nu[3:]||)

    整体缩放而非逐轴截断，末端沿指令方向滞后，不会偏离指令直线。
    """
    nu = np.asarray(nu, dtype=float).ravel()
    linear, angular = nu[:3].copy(), nu[3:].copy()
    s_lin = s_ang = 1.0
    n_lin = float(np.linalg.norm(linear))
    if max_linear > 0.0 and n_lin > max_linear:
        s_lin = max_linear / n_lin
        linear = linear * s_lin
    n_ang = float(np.linalg.norm(angular))
    if max_angular > 0.0 and n_ang > max_angular:
        s_ang = max_angular / n_ang
        angular = angular * s_ang
    return np.concatenate([linear, angular]), (s_lin, s_ang)


def solve_hold_qp(
    J_task,
    M,
    c,
    y_task,
    q,
    qd,
    qdd_prev,
    tau_prev,
    q_nominal,
    kp_null,
    kd_null,
    weights: HoldQPWeights,
    limits: HoldQPLimits,
    limit_avoidance: LimitAvoidance | None = None,
    warm_set=None,
    max_iterations: int = 60,      # QP 迭代上限。大→更可能收敛但单拍耗时长；小→更常返回次优解
    sigma_warn: float = 0.03,      # 开始加阻尼的 σ_min 阈值。大→更早保守；小→近奇异处力矩尖峰
    sigma_critical: float = 0.012,  # 阻尼打满的 σ_min 阈值。大→提前进入最大阻尼、末端变钝；小→留给尖峰的余地更大
    damping_min: float = 0.00005,  # 远离奇异时的 λ。大→常态就有跟踪偏差；小→数值条件变差
    damping_max: float = 0.03,     # 奇异处的 λ。大→安全但任务误差更大；小→压不住 1/σ 放大
) -> HoldQPResult:
    """保持任务的一个控制拍。

    J_task / y_task 是 6 行的控制点雅可比与「期望任务加速度减去漂移项」，都在
    LOCAL_WORLD_ALIGNED 轴系下；c 与 M 是模块头部那个仿射力矩模型的两个分量。

    代价函数（先在加速度空间写出，再整体换元到力矩空间）：

        min  w_p||A_p qdd - y_p||^2 + w_R||A_R qdd - y_R||^2
           + w_n||qdd - qdd_null||^2 + w_d||qdd - qdd_prev||^2 + w_reg||qdd||^2

    姿态行统一乘特征长度 L 折成长度量纲，使两个残差可比。
    """
    J = np.asarray(J_task, dtype=float)
    M = np.asarray(M, dtype=float)
    c = np.asarray(c, dtype=float).ravel()
    y = np.asarray(y_task, dtype=float).ravel()
    q = np.asarray(q, dtype=float).ravel()
    qd = np.asarray(qd, dtype=float).ravel()
    qdd_prev = np.asarray(qdd_prev, dtype=float).ravel()
    n = M.shape[0]

    L = float(weights.characteristic_length)
    A_p, y_p = J[:3, :], y[:3]
    A_R, y_R = L * J[3:, :], L * y[3:]

    # 奇异处理与它替代的伪逆通路口径一致：阻尼最小二乘在 w||J qdd - y||² 之外
    # 加了 λ²||qdd||²，故这里等价的 Tikhonov 权重是 w_position · damping²。
    scale = np.concatenate([np.ones(3), np.full(3, L)])
    singular_values = np.linalg.svd(scale[:, None] * J, compute_uv=False)
    sigma_min = float(singular_values[-1]) if singular_values.size else 0.0
    damping = adaptive_damping(singular_values, sigma_warn, sigma_critical,
                               damping_min, damping_max)
    w_reg = float(weights.regularization) + float(weights.position) * damping ** 2

    qdd_null = (-np.asarray(kp_null, dtype=float).ravel() * (q - np.asarray(q_nominal, dtype=float).ravel())
                - np.asarray(kd_null, dtype=float).ravel() * qd)
    if limit_avoidance is not None and limit_avoidance.gain > 0.0:
        qdd_null = qdd_null + limit_avoidance.gain * _limit_gradient(
            q, limit_avoidance.lower, limit_avoidance.upper,
            limit_avoidance.activate_rad)

    w_p, w_R = float(weights.position), float(weights.orientation)
    w_n, w_d = float(weights.joint_centering), float(weights.smoothness)
    w_xi = float(weights.qdd_box_slack)

    H_qdd = (w_p * (A_p.T @ A_p) + w_R * (A_R.T @ A_R)
             + (w_n + w_d + w_reg) * np.eye(n))
    H_qdd = 0.5 * (H_qdd + H_qdd.T)
    g_qdd = -(w_p * (A_p.T @ y_p) + w_R * (A_R.T @ y_R)
              + w_n * qdd_null + w_d * qdd_prev)

    qdd_lo, qdd_hi = build_qdd_box(q, qd, M, limits)
    tau_lo, tau_hi = build_torque_box(tau_prev, limits)

    # 换元到力矩空间求解：代入 qdd = M^-1(tau - c)，二次型与线性项变为
    #   H_tau = M^-T H_qdd M^-1,   g_tau = M^-T g_qdd - H_tau c
    Minv = np.linalg.solve(M, np.eye(n))
    H_tau = Minv.T @ H_qdd @ Minv
    H_tau = 0.5 * (H_tau + H_tau.T)
    g_tau = Minv.T @ g_qdd - H_tau @ c

    solution = solve_box_penalty_qp(
        H_tau, g_tau, tau_lo, tau_hi,
        A=Minv, a0=-(Minv @ c), rlo=qdd_lo, rhi=qdd_hi, weight=w_xi,
        x0=np.clip(c + M @ qdd_prev, tau_lo, tau_hi),
        max_iterations=max_iterations,
    )

    tau = np.clip(solution.x, tau_lo, tau_hi)   # 投影已保证，这里只做数值收口
    qdd = Minv @ (tau - c)
    return HoldQPResult(
        qdd=qdd,
        tau=tau,
        qdd_slack=np.abs(solution.violation),
        position_residual=A_p @ qdd - y_p,
        orientation_residual=(A_R @ qdd - y_R) / L if L > 0 else A_R @ qdd - y_R,
        qdd_box=(qdd_lo, qdd_hi),
        tau_box=(tau_lo, tau_hi),
        status=solution.status,
        iterations=solution.iterations,
        active_count=int(np.count_nonzero(solution.violation)),
        sigma_min=sigma_min,
        damping=damping,
    )
