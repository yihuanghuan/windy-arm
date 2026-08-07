"""稠密 active-set QP 求解器本身的正确性、终止性与连续性。

【这个测试的存在理由】
QP 是新的力矩约束链路的地基。它一旦在某类输入上悄悄给错解，上层的
"力矩不越界""姿态让步"全部是假的 —— 而错解在 200 Hz 下表现为力矩抖振，
在日志里和调参失误长得一模一样，极难归因。

所以这里不测"跑通了"，只测四件事：
  1. 解与穷举 active set 的参考解逐位一致（不是"接近可行"）；
  2. 退化/冗余约束下有界终止（active-set 法唯一的失效模式是循环，
     200 Hz 下循环就是错过截止时间，不是给错答案）；
  3. 热启动只改迭代次数、不改答案；
  4. 解对线性项连续 —— 这是抗抖振论证的可测化身。
"""

import itertools
import os
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from task_space_smc.core.dense_qp import (  # noqa: E402
    INFEASIBLE, MAX_ITER, OPTIMAL, QPProblem, solve_qp)

RNG = np.random.default_rng(20260806)


def _spd(n, floor=0.5):
    A = RNG.normal(size=(n, n))
    return A @ A.T + floor * np.eye(n)


def _brute_force(H, g, C, d):
    """穷举所有 active set 组合取最优可行解 —— 参考实现，慢但无疑义。"""
    n, m = len(g), len(d)
    best, best_value = None, np.inf
    for k in range(0, min(n, m) + 1):
        for A in itertools.combinations(range(m), k):
            if A:
                N = C[list(A)].T
                kkt = np.block([[H, -N], [N.T, np.zeros((k, k))]])
                try:
                    sol = np.linalg.solve(kkt, np.concatenate([-g, d[list(A)]]))
                except np.linalg.LinAlgError:
                    continue
                z, u = sol[:n], sol[n:]
                if np.any(u < -1e-8):
                    continue
            else:
                z = np.linalg.solve(H, -g)
            if np.all(C @ z - d >= -1e-8):
                value = 0.5 * z @ H @ z + g @ z
                if value < best_value:
                    best_value, best = value, z
    return best


def test_matches_brute_force_active_set_enumeration():
    """必须与穷举参考解一致，不是"可行且目标值差不多"。

    单约束算例发现不了 active-set 的真 bug：错误只在【两条约束同时激活】
    且需要 drop 步时才显形。这里让随机算例覆盖到那些组合。
    """
    checked = 0
    for _ in range(300):
        n = int(RNG.integers(2, 5))
        m = int(RNG.integers(1, 6))
        H, g = _spd(n), RNG.normal(size=n)
        C, d = RNG.normal(size=(m, n)), RNG.normal(size=m) * 0.8
        reference = _brute_force(H, g, C, d)
        if reference is None:          # 该随机算例本身不可行，跳过
            continue
        checked += 1
        solution = solve_qp(QPProblem(H, g, C, d))
        assert solution.status == OPTIMAL
        assert np.allclose(solution.z, reference, atol=1e-6)
    assert checked > 150, "随机算例几乎全不可行，说明取样有问题"


def test_is_the_identity_when_no_constraint_binds():
    """没有约束起作用时必须原样返回无约束最优解。"""
    n = 5
    H, g = _spd(n), RNG.normal(size=n)
    unconstrained = np.linalg.solve(H, -g)
    C = np.vstack([np.eye(n), -np.eye(n)])
    d = np.concatenate([unconstrained - 10.0, -unconstrained - 10.0])
    solution = solve_qp(QPProblem(H, g, C, d))
    assert solution.status == OPTIMAL
    assert solution.active_set == []
    assert np.allclose(solution.z, unconstrained, atol=1e-10)


def test_terminates_within_the_cap_on_degenerate_and_redundant_constraints():
    """重复行、平行行、秩亏 active set 都必须有界终止。

    active-set 法唯一的失效模式是循环。在 200 Hz 上循环不是"答案不好"，
    是这一拍没算完 —— 而驱动的指令超时是 30 ms，连着丢几拍就进保位了。
    """
    n = 3
    H, g = _spd(n), RNG.normal(size=n)
    C = np.array([
        [1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],          # 与上一行完全重复
        [2.0, 0.0, 0.0],          # 与上一行平行
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],         # 与上一行反向，两条一起就是等式
        [1.0, 1.0, 0.0],
    ])
    d = np.array([0.5, 0.5, 1.0, 0.2, -0.2, 0.7])
    solution = solve_qp(QPProblem(H, g, C, d), max_iterations=60)
    assert solution.status == OPTIMAL
    assert solution.iterations < 60
    assert np.all(C @ solution.z - d >= -1e-7)


def test_warm_start_reaches_the_same_solution_as_cold_start():
    """热启动只准是优化，不准改变行为。"""
    n, m = 6, 14
    for _ in range(200):
        H, g = _spd(n, floor=1e-2), RNG.normal(size=n) * 5.0
        C = np.vstack([np.eye(n), -np.eye(n), RNG.normal(size=(m - 2 * n, n))])
        d = np.concatenate([-np.ones(n), -np.ones(n), RNG.normal(size=m - 2 * n)])
        cold = solve_qp(QPProblem(H, g, C, d))
        if cold.status != OPTIMAL:
            continue
        warm = solve_qp(QPProblem(H, g, C, d), warm_set=cold.active_set)
        assert warm.status == OPTIMAL
        assert np.allclose(warm.z, cold.z, atol=1e-9)


def test_a_nonsense_warm_set_is_discarded_not_trusted():
    """外部传进来的 active set 是提示，不是命令；错的必须被丢掉。"""
    n, m = 4, 8
    H, g = _spd(n), RNG.normal(size=n)
    C = np.vstack([np.eye(n), -np.eye(n)])
    d = np.concatenate([-np.ones(n) * 0.3, -np.ones(n) * 0.3])
    cold = solve_qp(QPProblem(H, g, C, d))
    poisoned = solve_qp(QPProblem(H, g, C, d), warm_set=[0, 4, 1, 5, 999, -3])
    assert poisoned.status == OPTIMAL
    assert np.allclose(poisoned.z, cold.z, atol=1e-9)


def test_solution_is_continuous_in_the_linear_term():
    """沿一条穿过多次 active set 切换的直线扫 g，解不许跳。

    这条就是【抗抖振】本身：严格凸 QP 的最优解是 g 的分段仿射、局部
    Lipschitz 函数，所以机器人状态的小变化只能引起指令的小变化 —— 包括
    在约束集切换的那一刻。把它测出来，而不是只在文档里声称。
    """
    n = 8
    H = _spd(n, floor=1e-2)
    C = np.vstack([np.eye(n), -np.eye(n)])
    d = np.concatenate([-np.ones(n), -np.ones(n)])
    g0, g1 = RNG.normal(size=n) * 30.0, RNG.normal(size=n) * 30.0

    zs, sets = [], []
    for a in np.linspace(0.0, 1.0, 2001):
        s = solve_qp(QPProblem(H, (1 - a) * g0 + a * g1, C, d))
        assert s.status == OPTIMAL
        zs.append(s.z)
        sets.append(tuple(s.active_set))

    steps = np.linalg.norm(np.diff(np.array(zs), axis=0), axis=1)
    changes = sum(1 for i in range(len(sets) - 1) if sets[i] != sets[i + 1])
    assert changes >= 3, "这条扫描没穿过 active set 切换，测试就没测到东西"
    # 分段仿射 => 相邻步长同量级；真跳变会是几十上百倍。
    assert steps.max() < 10.0 * np.median(steps)


def test_solve_cost_stays_inside_the_control_budget():
    """实际规模下求解代价必须远小于 200 Hz 的 5 ms 预算。

    超一拍不是"慢一点"：驱动 30 ms 收不到新指令就按超时保位，
    连续几次就等于 demo 中断。

    【为什么不断言墙钟 p99】
    这条原来断言 400 次求解的墙钟 p99 < 2 ms，结果是间歇性失败：单独跑
    6/6 全过，跑全套时偶尔红。原因不在求解器 —— 是这个断言在【测机器负载】
    而不是测代码。QA_record §5.1 记过同一件事：在这台机器上跑整套 pytest
    期间，Vicon 桥的延迟从十几毫秒跳到 mean 572 ms、max 2208 ms。测试套件
    自己就是那个负载源，于是一条墙钟断言在套件里必然随机红。
    一条会随机红的测试比没有测试更糟：它训练所有人重跑一遍就当没事。

    所以改成断言两件不受负载影响的事：
      * 迭代次数有界 —— 这才是算法真正的代价，完全确定性；
      * 墙钟【中位数】—— 中位数对偶发抢占不敏感，而 p99 专门抓抢占。
    两者都留着，因为迭代次数管"算法没退化"，中位墙钟管"常数因子没爆炸"。
    """
    n, m = 12, 30
    times, iterations, warm = [], [], None
    for _ in range(400):
        H, g = _spd(n, floor=1e-2), RNG.normal(size=n) * 10.0
        C = np.vstack([np.eye(n), -np.eye(n), RNG.normal(size=(m - 2 * n, n))])
        d = np.concatenate([-np.ones(n) * 5, -np.ones(n) * 5,
                            RNG.normal(size=m - 2 * n)])
        start = time.perf_counter()
        s = solve_qp(QPProblem(H, g, C, d), warm_set=warm)
        times.append(time.perf_counter() - start)
        iterations.append(s.iterations)
        warm = s.active_set if s.status == OPTIMAL else None

    # 确定性判据：n=12 时最多 12 个约束能同时激活，加上 drop 步也不该逼近
    # max_iterations=60 的哨兵。这条与机器快慢无关。
    assert max(iterations) < 40, "迭代次数 %d 逼近上限，算法退化了" % max(iterations)
    assert float(np.mean(iterations)) < 15.0

    # 常数因子判据：中位数，不是 p99。阈值给得宽，它要抓的是"慢了一个
    # 数量级"，不是"这一跑机器忙"。
    median_ms = float(np.median(times)) * 1e3
    assert median_ms < 2.0, "中位求解 %.2f ms，常数因子爆炸了" % median_ms


def test_iteration_cap_is_reported_rather_than_silently_returning_garbage():
    """迭代上限用尽必须如实报 max_iter，让上层走降级而不是当成好解。"""
    n, m = 6, 12
    H, g = _spd(n, floor=1e-3), RNG.normal(size=n) * 50.0
    C = np.vstack([np.eye(n), -np.eye(n)])
    d = np.concatenate([-np.ones(n) * 0.01, -np.ones(n) * 0.01])
    solution = solve_qp(QPProblem(H, g, C, d), max_iterations=1)
    assert solution.status in (MAX_ITER, OPTIMAL)
    if solution.status == MAX_ITER:
        assert np.all(np.isfinite(solution.z))


def test_a_genuinely_infeasible_problem_is_reported_not_papered_over():
    """互相矛盾的约束必须报 infeasible。

    上层的弹性构造保证这在实机上不会发生；但如果求解器在这里返回一个
    "看起来可行"的解，那条保证就没法被证伪了。
    """
    n = 2
    H, g = np.eye(n), np.zeros(n)
    C = np.array([[1.0, 0.0], [-1.0, 0.0]])
    d = np.array([1.0, 1.0])          # x >= 1 且 -x >= 1
    solution = solve_qp(QPProblem(H, g, C, d))
    assert solution.status == INFEASIBLE


def test_rejects_malformed_shapes():
    with pytest.raises(ValueError):
        solve_qp(QPProblem(np.eye(3), np.zeros(2), np.eye(3), np.zeros(3)))
    with pytest.raises(ValueError):
        solve_qp(QPProblem(np.eye(3), np.zeros(3), np.eye(2), np.zeros(2)))
