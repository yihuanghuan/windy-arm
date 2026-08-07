"""稠密严格凸 QP，Goldfarb-Idnani 对偶主动集法。

    min  1/2 z'Hz + g'z    s.t.  Cz >= d

H 必须对称正定。全部用稠密 numpy 实现，不引入 scipy 或外部求解器：本机没有装
quadprog / osqp / qpsolvers / cvxpy / proxsuite，200 Hz 实物回路也不适合临时引入。

选这个算法的三条性质：

* 从无约束极小点出发逐条加约束，不需要 Phase 1，也不需要可行性恢复；
* 每次迭代对偶目标严格上升，配合最小下标破平局不会循环；活跃约束至多 n 条，
  迭代次数因而有界；
* 无随机数、无自适应容差、不依赖计时，同样输入逐位给出同样输出。

严格凸是硬要求：H 正定使极小点唯一，而 QP 的唯一极小点是 g 与 d 的分片仿射、
局部 Lipschitz 函数 —— 状态的小变化只引起指令的小变化，跨活跃集切换时也成立，
这就是抗抖振的来源。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

OPTIMAL = "optimal"
MAX_ITER = "max_iter"
INFEASIBLE = "infeasible"
NUMERICAL = "numerical"


@dataclass
class QPProblem:
    """``min 1/2 z'Hz + g'z``  s.t. ``Cz >= d``."""

    H: np.ndarray
    g: np.ndarray
    C: np.ndarray
    d: np.ndarray

    def validate(self) -> None:
        n = self.H.shape[0]
        if self.H.shape != (n, n):
            raise ValueError("H must be square")
        if self.g.shape != (n,):
            raise ValueError("g must have length n")
        if self.C.ndim != 2 or self.C.shape[1] != n:
            raise ValueError("C must be m x n")
        if self.d.shape != (self.C.shape[0],):
            raise ValueError("d must have length m")


@dataclass
class QPSolution:
    z: np.ndarray
    active_set: list = field(default_factory=list)
    multipliers: np.ndarray = field(default_factory=lambda: np.zeros(0))
    iterations: int = 0
    status: str = OPTIMAL
    objective: float = 0.0

    @property
    def ok(self) -> bool:
        return self.status == OPTIMAL


def _spd_solve(S, rhs):
    """解 ``S x = rhs``，S 是应当正定的 Schur 补，返回 ``(x, 是否良态)``。

    Cholesky 兼作条件数判据：S 是正定阵的 Schur 补，恰好在活跃行线性相关时分解
    失败。用它而不是 np.linalg.cond，可以把一次 SVD 挡在每轮迭代之外。
    """
    try:
        np.linalg.cholesky(S)
    except np.linalg.LinAlgError:
        return np.linalg.lstsq(S, rhs, rcond=None)[0], False
    return np.linalg.solve(S, rhs), True


class _Inverse:
    """经 Cholesky 因子施加 ``H^-1``，不显式求逆。

    保持任务的 QP 是【故意】病态的：位置与姿态 1e4:1 的权重差就是任务优先级本身，
    松弛权重再压几个数量级，cond(H) 约 5e7。在这个条件数下显式求逆丢约八位有效
    数字，误差还会沿主动集步累积到「等式」行被明显违反。三角回代把误差压在分解
    本身的量级。
    """

    def __init__(self, H):
        self.factor = np.linalg.cholesky(H)     # raises if H is not PD

    def __call__(self, X):
        y = np.linalg.solve(self.factor, X)
        return np.linalg.solve(self.factor.T, y)


def _kkt_point(applyHinv, g, C, d, W, p=-1, u_p=0.0):
    """由工作集直接重算 ``(z, u)``，而不是累加步长走过去；活跃行数值相关时返回 None。

    对偶法的每个迭代点都满足

        H z + g = C_W' u + C_p u_p        且       C_W z = d_W

    故 (z, u) 是 (W, p, u_p) 的函数，可以整体重建。每轮重建而不是累加 z += t*dz，
    是为了不让上面那个病态把活跃行从它们的等式上漂走。
    """
    Hg = applyHinv(g)
    if not W:
        if p < 0:
            return -Hg, np.zeros(0)
        return applyHinv(C[p]) * u_p - Hg, np.zeros(0)

    N = C[W].T                        # n x |W|
    HN = applyHinv(N)
    S = N.T @ HN
    rhs = np.asarray(d)[W] + N.T @ Hg
    if p >= 0 and u_p != 0.0:
        rhs = rhs - N.T @ applyHinv(C[p]) * u_p
    try:
        u, well = _spd_solve(S, rhs)
    except np.linalg.LinAlgError:
        return None
    if not well:
        return None
    z = HN @ u - Hg
    if p >= 0 and u_p != 0.0:
        z = z + applyHinv(C[p]) * u_p
    return z, u


def solve_qp(
    problem: QPProblem,
    warm_set=None,
    max_iterations: int = 60,
    tol: float = 1e-9,
) -> QPSolution:
    """求解 ``problem``，方法见模块头部。

    warm_set 只是提示：仅当它能重建出乘子非负的一致 KKT 点时才被采纳，此后主循环
    照常运行，故热启动只影响迭代次数，不影响结果。
    """
    problem.validate()
    H = np.asarray(problem.H, dtype=float)
    g = np.asarray(problem.g, dtype=float)
    C = np.asarray(problem.C, dtype=float)
    d = np.asarray(problem.d, dtype=float)
    n = H.shape[0]
    m = C.shape[0]

    try:
        applyHinv = _Inverse(H)
    except np.linalg.LinAlgError:
        return QPSolution(np.zeros(n), [], np.zeros(0), 0, NUMERICAL, float("inf"))

    W: list = []
    u = np.zeros(0)
    z = -applyHinv(g)
    if not np.all(np.isfinite(z)):
        return QPSolution(np.zeros(n), [], np.zeros(0), 0, NUMERICAL, float("inf"))

    if warm_set:
        candidate = [int(i) for i in warm_set if 0 <= int(i) < m]
        # 去重并排序：active set 的语义与顺序无关，固定顺序才能保证确定性。
        candidate = sorted(set(candidate))
        if candidate:
            seeded = _kkt_point(applyHinv, g, C, d, candidate)
            if seeded is not None and np.all(seeded[1] >= -tol):
                z, u = seeded[0], seeded[1]
                W = candidate

    iterations = 0
    u_p = 0.0
    p = -1

    while iterations < max_iterations:
        iterations += 1

        if p < 0:
            residual = C @ z - d
            if W:
                # 已在工作集里的行按构造满足等式，浮点噪声不该把它们选成"最违反"。
                residual = residual.copy()
                residual[W] = np.inf
            p_candidate = int(np.argmin(residual))
            if residual[p_candidate] >= -tol:
                obj = float(0.5 * z @ H @ z + g @ z)
                return QPSolution(z, list(W), u.copy(), iterations - 1, OPTIMAL, obj)
            p = p_candidate
            u_p = 0.0

        Hcp = applyHinv(C[p])
        if W:
            N = C[W].T
            HN = applyHinv(N)
            S = N.T @ HN
            try:
                r, _ = _spd_solve(S, HN.T @ C[p])
            except np.linalg.LinAlgError:
                return QPSolution(z, list(W), u.copy(), iterations, NUMERICAL,
                                  float("inf"))
            z_step = Hcp - HN @ r
        else:
            r = np.zeros(0)
            z_step = Hcp

        denom = float(C[p] @ z_step)
        v_p = float(C[p] @ z - d[p])

        # denom == 0 表示 C[p] 落在活跃行张成的空间里，原始方向无法推进，
        # 只能走对偶步（丢一行）。
        t2 = np.inf if denom <= tol else -v_p / denom

        t1 = np.inf
        blocking = -1
        for k in range(len(W)):
            if r[k] > tol:
                ratio = u[k] / r[k]
                if ratio < t1 - 1e-15:
                    t1, blocking = ratio, k

        t = min(t1, t2)
        if not np.isfinite(t):
            # 既满足不了 p，也没有行可丢：问题不可行。调用方的弹性形式下不会走到
            # 这里，但静默返回垃圾解比明说更糟。
            return QPSolution(z, list(W), u.copy(), iterations, INFEASIBLE,
                              float("inf"))

        if t > 0.0:
            if len(W):
                u = u - t * r
            u_p += t

        if t2 <= t1:
            W.append(p)
            u = np.append(u, u_p)
            order = np.argsort(np.asarray(W, dtype=int), kind="stable")
            W = [W[i] for i in order]
            u = u[order]
            p, u_p = -1, 0.0
        else:
            W.pop(blocking)
            u = np.delete(u, blocking)

        # 由工作集重建迭代点，不累加步长：cond(H) ~ 5e7 时累加形式会漂到把
        # 已违反的约束报成 optimal（见 _kkt_point）。
        rebuilt = _kkt_point(applyHinv, g, C, d, W, p, u_p)
        if rebuilt is None:
            if not W:
                return QPSolution(z, [], np.zeros(0), iterations, NUMERICAL,
                                  float("inf"))
            # 工作集变得线性相关：丢掉最新加入的那一行继续。
            W.pop()
            u = np.delete(u, len(u) - 1)
            rebuilt = _kkt_point(applyHinv, g, C, d, W, p, u_p)
            if rebuilt is None:
                return QPSolution(z, list(W), u.copy(), iterations, NUMERICAL,
                                  float("inf"))
        z, u = rebuilt

    obj = float(0.5 * z @ H @ z + g @ z)
    return QPSolution(z, list(W), u.copy(), iterations, MAX_ITER, obj)


# --------------------------------------------------------------------------
# 「变量带箱约束 + 线性行软约束」的专用求解器
# --------------------------------------------------------------------------
#
# 保持任务走的是这条通路，不是上面的 solve_qp。原因是结构性的：把一个变量上下界
# 写成 +e_i·x >= lo 与 -e_i·x >= -hi 两行，主动集法就拿到一对【严格线性相关】的
# 法向量。关节已越出速度包线时加速度箱塌成一个点，两行同时贴在边界上，对偶主动集
# 法每轮都要处理这个相关性，在真实问题形状上会大面积误报 infeasible —— 而弹性
# 形式下这些算例按构造全都是可行的。
#
# 把上下界当【箱】用投影处理，这个失效模式就整个消失。再把松弛变量解析消去，剩下
# 一个凸的、C1 的、带箱约束的分片二次函数：
#
#     F(x) = 1/2 x'Hx + g'x + 1/2 Σ_i w_i·v_i(x)² ,   x ∈ [lo, hi]
#     v_i(x) = (A x + a0) 第 i 行超出 [rlo_i, rhi_i] 的量
#
# v 在拐点处为零，故 F 连续可微，投影（半光滑）牛顿迭代全局收敛。松弛只取 L2：
# L1 是经典的精确罚，但会在 v = 0 处留一个梯度跳变，正好毁掉这里赖以稳健的光滑性。
# 取 w = 1e6 时残余违反量约为 约束力/w ~ 1e-5 N·m，对 9 N·m 的限幅可视为精确。


@dataclass
class BoxPenaltySolution:
    x: np.ndarray
    violation: np.ndarray
    iterations: int
    status: str
    objective: float

    @property
    def ok(self) -> bool:
        return self.status == OPTIMAL


# 行越界量 v(r)：超上界取 r − rhi，低于下界取 r − rlo，箱内为 0
def _row_violation(r, rlo, rhi):
    return np.where(r > rhi, r - rhi, 0.0) + np.where(r < rlo, r - rlo, 0.0)


def _exact_line_step(x, step, H, g, A, a0, rlo, rhi, w, lo, hi):
    """在可行的 alpha 区间上精确求 ``F(x + alpha*step)`` 的极小点。

    沿一条直线，目标是一维凸分片二次函数，其导数分片线性且单调不减，故极小点有
    闭式解：把各行进出自己箱子的 alpha 全部收集为断点，一次性算出所有断点处的导数，
    在导数变号的那一段内线性插值。

        phi'(alpha) = step'H x + g'step + (step'H step)·alpha + v(alpha)'(w ⊙ A step)

    用精确步而不是 Armijo 回溯：只要求「有下降」会让迭代之字形往返，外层迭代次数
    与单次求解耗时都压不进 200 Hz 的预算。
    """
    # x + alpha*step 仍留在变量箱内的最大 alpha
    with np.errstate(divide="ignore", invalid="ignore"):
        to_hi = np.where(step > 0, (hi - x) / step, np.inf)
        to_lo = np.where(step < 0, (lo - x) / step, np.inf)
    alpha_box = float(np.min(np.minimum(to_hi, to_lo)))
    if not np.isfinite(alpha_box):
        alpha_box = 1.0
    alpha_box = max(0.0, min(alpha_box, 1.0e6))
    if alpha_box <= 0.0:
        return 0.0

    r0 = A @ x + a0
    s = A @ step
    d0 = float(step @ (H @ x) + g @ step)      # phi'(0) 中不含罚项的部分
    d1 = float(step @ (H @ step))

    breaks = []
    nonzero = np.abs(s) > 1e-300
    for bound in (rhi, rlo):
        cand = (bound[nonzero] - r0[nonzero]) / s[nonzero]
        breaks.append(cand[(cand > 0.0) & (cand < alpha_box)])
    alphas = np.unique(np.concatenate(breaks + [np.array([alpha_box])]))

    grid = np.concatenate([[0.0], alphas])
    R = r0[None, :] + grid[:, None] * s[None, :]
    V = _row_violation(R, rlo[None, :], rhi[None, :])
    deriv = d0 + d1 * grid + V @ (w * s)

    if deriv[0] >= 0.0:
        return 0.0
    for k in range(1, grid.size):
        if deriv[k] >= 0.0:
            span = deriv[k] - deriv[k - 1]
            if span <= 0.0:
                return float(grid[k])
            return float(grid[k - 1] + (grid[k] - grid[k - 1]) * (-deriv[k - 1]) / span)
    return float(alpha_box)


def solve_box_penalty_qp(
    H,
    g,
    lo,
    hi,
    A,
    a0,
    rlo,
    rhi,
    weight,
    x0=None,
    max_iterations: int = 40,
    tol: float = 1e-9,
) -> BoxPenaltySolution:
    """在 ``x ∈ [lo, hi]`` 上极小化 ``1/2 x'Hx + g'x + 1/2 w||v(Ax + a0)||²``。

    投影牛顿迭代：自由变量上解牛顿方程 (H + A'diag(w·活跃行)A)·Δ = −∇F，
    再沿该方向做精确线搜索并投影回箱内。

    H 必须对称正定，箱必须满足 lo <= hi。两条都由调用方保证，在
    :mod:`task_space_smc.core.task_qp` 中按构造成立。
    """
    H = np.asarray(H, dtype=float)
    g = np.asarray(g, dtype=float).ravel()
    lo = np.asarray(lo, dtype=float).ravel()
    hi = np.asarray(hi, dtype=float).ravel()
    A = np.asarray(A, dtype=float)
    a0 = np.asarray(a0, dtype=float).ravel()
    rlo = np.asarray(rlo, dtype=float).ravel()
    rhi = np.asarray(rhi, dtype=float).ravel()
    w = np.asarray(weight, dtype=float).ravel()
    if w.size == 1:
        w = np.full(rlo.size, float(w[0]))
    n = g.size

    x = np.clip(np.zeros(n) if x0 is None else np.asarray(x0, dtype=float).ravel(),
                lo, hi)

    def objective(xx):
        v = _row_violation(A @ xx + a0, rlo, rhi)
        return float(0.5 * xx @ H @ xx + g @ xx + 0.5 * np.sum(w * v * v)), v

    value, v = objective(x)
    status = MAX_ITER
    iterations = 0

    for iterations in range(1, max_iterations + 1):
        grad = H @ x + g + A.T @ (w * v)

        # 把梯度投影到箱上：压在边界上的分量既不是可走的方向，也不该计入收敛误差。
        pressed_lo = (x <= lo + 1e-15) & (grad > 0.0)
        pressed_hi = (x >= hi - 1e-15) & (grad < 0.0)
        free = ~(pressed_lo | pressed_hi)
        projected = np.where(free, grad, 0.0)
        # 收敛判据按梯度量级取相对值：任务权重 1e4 乘 O(10) 的需求，梯度天然在
        # 1e5~1e6，绝对阈值等于要求磨平最后几位十进制，会把已收敛的解报成 max_iter。
        gscale = max(1.0, float(np.max(np.abs(grad))))
        if float(np.max(np.abs(projected))) <= tol * gscale or not np.any(free):
            status = OPTIMAL
            break

        active_rows = (A @ x + a0 > rhi) | (A @ x + a0 < rlo)
        hess = H + A.T @ (A * (w * active_rows)[:, None])
        idx = np.flatnonzero(free)
        try:
            step_free = np.linalg.solve(hess[np.ix_(idx, idx)], -grad[idx])
        except np.linalg.LinAlgError:
            step_free = -grad[idx] / max(1.0, float(np.max(np.abs(hess))))
        step = np.zeros(n)
        step[idx] = step_free

        alpha = _exact_line_step(x, step, H, g, A, a0, rlo, rhi, w, lo, hi)
        if alpha <= 0.0:
            status = OPTIMAL          # 沿牛顿方向已无下降空间
            break
        trial = np.clip(x + alpha * step, lo, hi)
        trial_value, trial_v = objective(trial)
        if trial_value >= value:
            status = OPTIMAL          # 精确线搜索都换不来下降，就是最优点
            break
        moved = float(np.max(np.abs(trial - x)))
        x, value, v = trial, trial_value, trial_v
        if moved <= 1e-12 * max(1.0, float(np.max(np.abs(x)))):
            status = OPTIMAL          # 步长已到浮点分辨率，再迭代没有意义
            break

    if not np.all(np.isfinite(x)):
        return BoxPenaltySolution(np.clip(np.zeros(n), lo, hi),
                                  np.zeros(rlo.size), iterations, NUMERICAL,
                                  float("inf"))
    return BoxPenaltySolution(x, v, iterations, status, value)
