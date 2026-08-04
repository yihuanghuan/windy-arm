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
    # 可行性统一缩放系数（1.0 = 未受限）。见 FeasibilityLimits 的说明。
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
    # 关节数由 J 的列数推导
    # 任务维度由 J 的行数推导：6 = 位置+完整姿态，5 = 位置+笔轴方向
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
    """把 6D 位姿任务降为 5D：位置(3) + 笔轴方向(2)。

    ⚠️ 【笔轴是末端坐标系的 x 轴】（RViz 里的红轴）。

    这个约定由【物理笔架的安装方向】决定：笔沿 link7 的 x 轴延伸，所以
    要让笔垂直于纸面，就得让 x 轴垂直于纸面。

    取错轴的后果不是精度下降，而是【约束方向反了】：放开该锁死的笔尖指向、
    锁死无所谓的绕笔自转。而且 pen_axis_error() 用的是同一个轴，
    【自己验自己不会报警】—— 曾经 130 个测试全过而笔轴是错的。
    独立判据见 test/test_pen_axis_convention.py（用笔画点云的 SVD 法向做基准）。

    绕笔轴的自转对写字/画图任务无意义 —— 笔转多少度不影响笔迹。剔除它
    把零空间秩从 0 提到 1，那个自由度正好用来做限位规避。

    实测（whole 轨迹，120 个采样点）::

        6D 位置+完整姿态   位置可达  35%   姿态可达  38%   sigma最差 0.01497
        5D 位置+笔轴方向   位置可达  98%   笔轴可达 100%   sigma最差 0.03802

    做法：a = 末端在世界系下的笔轴方向；P = I − a·aᵀ 把绕 a 的分量投影掉
    （P 秩为 2）。J₅ = [J_pos ; P·J_rot]，rhs₅ = [a_cmd ; P·alpha_cmd]。
    """
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
    # P 秩为 2，所以 [J_pos; P J_rot] 有一行是线性相关的。取 P 的两个非零
    # 奇异方向作为基，得到真正的 5 行 —— 直接堆 6 行会让 J₅ 奇异。
    U, S, _ = np.linalg.svd(P)
    B = U[:, :2].T                      # 2x3，张成垂直于 a 的平面
    J5 = np.vstack([J[:3, :], B @ J[3:, :]])
    y5 = np.concatenate([y[:3], B @ y[3:]])
    return J5, y5, a



def pen_axis_error(R_actual, R_desired, axis_local=(1.0, 0.0, 0.0)):
    """笔轴对齐误差（rad）。

    pen_axis 模式下【完整 3D 姿态误差是误导性指标】—— 它包含我们故意不控制
    的绕笔轴自转，那一项会自由漂移。实测离线仿真里完整姿态误差报 0.56 rad，
    而笔轴本身对得很准。要看的是这个量。
    """
    a = np.asarray(R_actual, dtype=float).reshape(3, 3) @ np.asarray(
        axis_local, dtype=float).reshape(3)
    d = np.asarray(R_desired, dtype=float).reshape(3, 3) @ np.asarray(
        axis_local, dtype=float).reshape(3)
    a = a / max(np.linalg.norm(a), 1e-12)
    d = d / max(np.linalg.norm(d), 1e-12)
    return float(np.arccos(np.clip(a @ d, -1.0, 1.0)))


def _undamped_inverse(J_arm, M_aa, characteristic_length, rcond=1e-10):
    """M 加权广义逆的【截断伪逆】版本，专供零空间投影使用。

    与 dynamic_consistent_inverse 的差别只有一处：用 SVD 截断代替阻尼求逆。
    阻尼会让 I − J̄J 不再是投影算子；截断则保持 A⁺AA⁺ = A⁺，于是
    (J̄₀J)² = J̄₀J 严格成立，N 是真投影，J·N 回到机器精度量级。

    截断掉的方向正是近奇异、任务本来就使不上力的方向 —— 把它留给零空间
    在物理上也是对的。
    """
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
    """零空间限位规避的参数。"""
    lower: np.ndarray
    upper: np.ndarray
    gain: float = 0.10
    activate_rad: float = 0.5


def _limit_gradient(q, lower, upper, activate):
    """距限位 < activate 时线性增长的排斥梯度；中段恒为 0。"""
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
    """被控对象【真实存在】而控制器原先完全不知道的两个运动学硬夹。

    src/robotics/arm/torque_sim_arm.cc:306 每个积分子步做
        qd = clamp(qd, ±velocity_limit)          velocity_limit=[3,3,3,5,5,5] rad/s
    :309-314 再做位置硬夹并把撞限方向的 qd 置 0。控制器（本模块 /
    task_space_smc.py / actuator_adapter.py）里没有任何一处建模它们。

    【为什么必须让控制器知道】——真实 ROS 实测的因果链：
      1. 参考末端线速度峰值 0.387 m/s，配上 σ_min 0.01~0.06 的构型，
         所需关节速度就越过电机上限（纯运动学核算：1.2~1.24% 的采样需要
         >100% 速度包线，p99 ≈ 1.02~1.09 倍）。
      2. plant 于是【逐轴】夹速度。逐轴夹会把末端加速度的【方向】转掉——
         这正是 40mm 级离轨的形态，不是"沿路径滞后"。
      3. 误差一起来，SMC(λ=72,k=102) 立刻要 102·72·0.03 ≈ 220 m/s²，
         指令力矩最深处 −727.50 Nm（40.4 倍限幅），六轴力矩饱和。
    真实 ROS 逐点判夹：速度夹样本占 1.52%，却贡献 96.85% 的平方误差
    （剔除后 RMS 2.018 -> 0.361 mm）；速度夹∪位置夹占 2.41%、贡献 98.56%
    （剩余 0.245 mm）。P(err>1mm|速度夹)=65.3% vs P(err>1mm|无夹)=1.01%。

    【为什么不能靠放宽硬夹解决】——它是物理量。实测放宽只会搬家不会消除：
      max_velocity ×1.5 RMS 2.451~2.994（无改善）、×3 峰值 41→87.8mm 且
      安静时段误差地板从 0.0196mm 塌到 0.188mm、×30 直接发散到 RMS 50mm。
      力矩上限同理：tau_scale=2.0 时 RMS 2.018 -> 4.949、速度夹反而翻倍。
    所以正确做法是让控制器【服从】这两个夹子，而不是去动它们。
    """

    velocity: np.ndarray            # rad/s，与 config/motor_config.yaml 的 max_velocity 对齐
    lower: np.ndarray               # rad，关节位置下限（取自 URDF）
    upper: np.ndarray               # rad，关节位置上限
    dt: float                       # 控制周期 s（200 Hz -> 0.005）
    torque: np.ndarray | None = None      # Nm，用于估位置限的制动能力
    brake_safety: float = 0.5             # 制动能力打折系数（保守）


def _feasible_acceleration_box(q, qd, M, limits: FeasibilityLimits):
    """把 plant 的速度夹 + 位置夹翻译成【本控制周期内】的关节加速度盒。

    速度夹：下一步 qd+qdd·dt 必须留在 ±v 内 -> qdd ∈ [(-v-qd)/dt, (v-qd)/dt]。

    位置夹：只做一步前瞻是没用的（发现越界时已经刹不住），用制动距离约束——
    朝限位方向的速度不得超过 sqrt(2·a_brake·剩余余量)，于是靠近限位时允许
    速度连续地收敛到 0，控制器会自己提前减速而不是撞墙。a_brake 用解耦近似
    tau_limit/M_ii 再打 brake_safety 折。实测这条支路只在距限位 0.16~0.25 rad
    以内起作用（joint5: 3.2Nm/M55 -> 余量 0.156 rad 处才压到 5 rad/s）,
    正好覆盖那 3 个位置限事件（joint3 撞 ±1.57、joint5 撞 ±2.0，qd 被夹成
    精确的 0.000，误差 4.84mm，而 σ 健康 0.067~0.097、力矩甚至零饱和）。

    【不要把 0 强行塞进盒子】。当前速度若已在包线外（qd=-4.9 而上限 3.0），
    回到界内需要 qdd=+380 rad/s²，而强行包含 0 会把下界压成 0，等于允许
    "不修正"，下一步仍在界外 —— 那正是这一整套要消除的行为。
    这么做也不必要：v_lo <= 0 <= v_hi 恒成立（速度上限非负，制动项只把界
    往 0 收），所以 lo <= hi 天然成立，盒子永不为空，缩放问题恒有解。
    """
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
    """求最大的 a ∈ [0,1] 使 a·u 逐轴落在 [lo, hi] 内（统一标量，保方向）。

    只用于【零空间项】。任务项不能这么做 —— 见 _saturation_aware_task 的说明，
    真实 ROS 实测那样会锁死（RMS 2.47 -> 457 mm）。
    """
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
    """只用 `columns` 指定的那几个关节做 M 加权动力学一致（阻尼）逆。

    columns 全 True 且 damping 相同时，结果与 dynamic_consistent_inverse 完全
    一致 —— 这保证"无饱和"时新代码是严格的恒等变换。
    """
    Jf = J_scaled[:, columns]
    Mf = M[np.ix_(columns, columns)]
    X = np.linalg.solve(Mf, Jf.T)
    A = Jf @ X
    if damping > 0.0:
        A = A + (damping * damping) * np.eye(A.shape[0])
    return X @ np.linalg.solve(A, np.eye(A.shape[0]))


def _task_scaling_factor(a, b, lo, hi, tol=1e-9):
    """求最大 s ∈ [0,1] 使 s·a + b 逐轴落在 [lo, hi] 内（SNS 的同名子程序）。

    s 缩放的是【任务空间】的加速度需求，所以末端只是沿指令方向按比例变慢
    （"沿路径滞后"），方向不变。
    """
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
    """饱和感知的任务解算（Flacco & De Luca 的 SNS，搬到加速度层）。

    【为什么不能用"整条 qdd_task 乘一个 α"】——已在真实 ROS 上证伪。
    α = min_i(盒边界/需求_i)，只要有【一个】关节正贴着速度上限（hi_i = 0），
    α 就恒为 0，整条手臂的任务加速度被清零，于是以 3 rad/s 匀速滑走再也不
    减速。实测：域 90 RMS 457.19 mm / 峰值 695.6 mm，域 91（关掉位置制动项）
    RMS 192.03 mm，而同一棵树的基线是 2.466 mm。日志里 α<0.01 占 77.2% 的
    采样，joint2 精确停在 −3.000 rad/s 不动。
    而关节贴着速度上限【本身是良性的】：基线里 t=2.99 joint2 已经 −2.943
    rad/s，位置误差仍只有 0.025 mm。所以正确的处理不是"全体降速"，而是
    "把顶到限的那一轴按边界值固定住，剩下的关节把任务补回来"。

    算法：
      1. 用全部自由关节解 qdd = qdd_sat + J_free^# (y − J·qdd_sat)。
      2. 若逐轴都在盒内 -> 直接返回（s=1，与改动前完全一致）。
      3. 否则把越界最多的那一轴钉在它的盒边界上（这正是 plant 会做的事），
         从自由集里移除，回到 1 —— 关键差别在于【其余关节会重新解算并补偿】，
         末端方向因此保住；plant 单独夹的时候没人补偿，末端就被转向甩出去。
      4. 自由关节数不足以张成任务空间时，退回记录过的最优 (s*, W*)，
         用 s*·y 重解 —— 此时任务本身运动学不可行，末端沿指令方向按 s* 滞后。
    返回 (qdd_task, s, n_saturated)。
    """
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
        # qdd(s) = a·s + b：a 是与任务需求成正比的部分，b 是饱和轴钉住的部分
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
    # 关节数由 J 的列数推导
    # 任务维度由 J 的行数推导。6 = 位置+完整姿态；5 = 位置+笔轴方向
    # （绕笔轴的自转对写字任务无意义，剔除它把零空间秩从 0 提到 1）。
    # 实测：6D 任务下轨迹只有 35% 可达、sigma 最差 0.015；
    #       5D 任务下 98%/100% 可达、sigma 最差 0.038。
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
    # 关节数按输入推导。注意 6 关节做 6D 任务时零空间秩为 0，本模块的
    # 投影恒为零 —— 配置里已 nullspace.enabled=false 停用，这里只保证
    # 万一被调用时不会因写死 7 而抛异常。
    q = np.asarray(q_arm, dtype=float).ravel()
    n = q.size
    qd = np.asarray(qd_arm, dtype=float).reshape(n)
    q_nom = np.asarray(q_nominal, dtype=float).reshape(n)
    qdd_null = -_diag(kp_null) @ (q - q_nom) - _diag(kd_null) @ qd
    # 限位规避：只在距限位 < activate 时激活的排斥梯度，中段恒为 0。
    #
    # 【全局拉向中位是错的】—— 那样零空间项会全程与任务项抢自由度，
    # 实测把位置可达率从 100% 拉到 81%。只在临近限位时激活则相反：
    # 触限次数 19 -> 3，笔轴可达率 88% -> 100%，主任务几乎不受影响。
    if limit_avoidance is not None and limit_avoidance.gain > 0.0:
        qdd_null = qdd_null + limit_avoidance.gain * _limit_gradient(
            q, limit_avoidance.lower, limit_avoidance.upper,
            limit_avoidance.activate_rad)
    # 【零空间投影必须用无阻尼的广义逆构造】。
    #
    # 阻尼逆不是伪逆，所以 I − J̄_damped·J 不是投影算子（N² ≠ N），零空间项
    # 会直接【漏进任务空间】。实测这条正反馈环：
    #     sigma 下滑 -> 阻尼升到 0.0275 -> 泄漏 3.5e-09 涨到 2.4e-03（6 个
    #     数量级）-> 泄漏被当作任务扰动 -> 六轴力矩饱和 -> 位置误差 41 mm
    # sigma 一回升、阻尼退出，误差 200 ms 内自己回到 0.03 mm ——
    # 表现为偶发尖峰（占全程 1.82% 时间却贡献绝大部分 RMS），
    # 参数一激进就全程发作（曾实测泄漏 2.76e-02、RMS 116 mm）。
    #
    # 做法：任务项仍用阻尼逆 J̄（奇异保护不变），投影用截断伪逆 J̄₀。
    # 截断保证 J̄₀ 在近奇异时不爆，同时 (J̄₀J)² = J̄₀J 严格成立 ——
    # 因为 A⁺AA⁺ = A⁺，所以 N 是真投影，泄漏回到机器精度量级。
    Jbar0 = _undamped_inverse(J, M, characteristic_length)
    N = np.eye(n) - Jbar0 @ J_scaled
    qdd_task = Jbar @ y_scaled
    qdd_null_projected = N @ qdd_null
    # 【饱和感知解算】：把指令加速度压回 plant 硬夹允许的盒子里。
    #
    # 不加这一步时，控制器会命令一个必然被 plant 逐轴夹掉的加速度，plant 只夹
    # 越界那一轴、其余关节毫不知情 -> 末端加速度方向被转掉 -> 误差起飞 ->
    # 反馈 windup（最深 −727.50 Nm = 40 倍限幅）-> 力矩饱和 -> 末端被主动
    # 甩出 41.64 mm。加了之后顶到限的轴被钉在边界上，【其余关节重新解算把
    # 任务补回来】，末端方向得以保住。
    #
    # 零空间项在任务项之后、用剩余预算按 γ 统一缩放（剩余盒必含 0，因为任务
    # 解本身在盒内，所以 γ 恒有解）。零空间项本来就小（真实 ROS 实测
    # |tau_null| 中位仅 0.06 Nm），让它退让不影响限位规避的有效性。
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
