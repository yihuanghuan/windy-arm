"""零空间投影必须是【真投影】，即使在重阻尼下。

【这个测试的存在理由】
`N = I − J̄·J` 只有当 J̄ 是伪逆时才是投影算子。若用【阻尼】逆构造，
N² ≠ N，零空间运动会漏进任务空间。

实测这条正反馈环（6 关节 + 笔架，水平书写面，ROS 实时仿真）：
    sigma 下滑 -> 自适应阻尼升到 0.0275 -> 泄漏范数 3.5e-09 涨到 2.4e-03
    -> 泄漏被当作任务扰动 -> 六轴力矩全饱和 -> 位置误差 41 mm
sigma 回升后误差 200 ms 内自己回到 0.03 mm，所以表现为【偶发尖峰】：
误差 >1mm 的时间只占全程 1.82%，却贡献了绝大部分 RMS（2.4mm vs 其余时段
0.02mm）。参数一激进就全程发作 —— 曾实测泄漏 2.76e-02、RMS 116 mm。

当时 132 个测试全绿，因为没有任何一个检查 N 的投影性质。
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from task_space_smc.redundancy_resolution import resolve_acceleration  # noqa: E402

RNG = np.random.default_rng(20260802)
Z6 = np.zeros(6)


def _random_case(task_dim, n_arm, conditioning=1.0):
    """随机任务雅可比与正定质量阵；conditioning<1 时人为压低最小奇异值。"""
    J = RNG.normal(size=(task_dim, n_arm))
    u, s, vt = np.linalg.svd(J, full_matrices=False)
    s = np.linspace(1.0, conditioning, s.size)
    J = (u * s) @ vt
    L = RNG.normal(size=(n_arm, n_arm))
    M = L @ L.T + n_arm * np.eye(n_arm)
    return J, M


def _resolve(J, M, damping_max, qdd_null_seed=1.0):
    """用一个非零的零空间目标去解，这样泄漏才会显形。"""
    n = J.shape[1]
    return resolve_acceleration(
        J, M, RNG.normal(size=J.shape[0]),
        q_arm=Z6[:n], qd_arm=Z6[:n],
        q_nominal=np.full(n, -qdd_null_seed),      # 制造一个大的零空间需求
        kp_null=np.full(n, 50.0), kd_null=np.zeros(n),
        characteristic_length=0.2,
        sigma_warn=10.0, sigma_critical=0.0,       # 强制阻尼取到 damping_max
        damping_min=damping_max, damping_max=damping_max,
    )


@pytest.mark.parametrize("damping", [0.0, 0.003, 0.03, 0.1, 0.4])
def test_projector_is_idempotent_at_every_damping(damping):
    """N² = N。阻尼逆构造的 N 在 damping=0.03 时就会明显违反。"""
    J, M = _random_case(5, 6)
    res = _resolve(J, M, damping)
    N = res.N
    err = np.abs(N @ N - N).max()
    assert err < 1e-9, (
        f"damping={damping}: N 不是投影算子，|N²−N|_max = {err:.3e}。"
        f"零空间项会按这个量级漏进任务空间。")


@pytest.mark.parametrize("damping", [0.0, 0.003, 0.03, 0.1, 0.4])
def test_leakage_stays_at_machine_precision(damping):
    """J·N ≈ 0：零空间运动不得在任务空间产生加速度。"""
    J, M = _random_case(5, 6)
    res = _resolve(J, M, damping)
    assert res.nullspace_leakage_norm < 1e-9, (
        f"damping={damping}: 泄漏范数 {res.nullspace_leakage_norm:.3e}，"
        f"应为机器精度量级")


def test_leakage_does_not_grow_when_conditioning_degrades():
    """近奇异【正是】自适应阻尼最大的时候，也正是原来泄漏爆发的时候。"""
    worst = 0.0
    for cond in (1.0, 0.1, 0.01, 1e-3, 1e-4):
        J, M = _random_case(5, 6, conditioning=cond)
        res = _resolve(J, M, 0.03)
        worst = max(worst, res.nullspace_leakage_norm)
    assert worst < 1e-8, f"条件数退化时泄漏升到 {worst:.3e}"


def test_task_term_still_uses_damping():
    """修复不能把奇异保护一起去掉 —— 任务项仍须随阻尼变化。"""
    J, M = _random_case(5, 6, conditioning=1e-4)
    lo = _resolve(J, M, 0.0)
    hi = _resolve(J, M, 0.4)
    assert np.linalg.norm(hi.qdd_cmd) < np.linalg.norm(lo.qdd_cmd), (
        "重阻尼没有抑制近奇异处的加速度，说明任务项没有用到阻尼")


def test_six_dof_task_has_no_nullspace_and_no_leakage():
    """6 关节做 6D 任务时零空间秩为 0，N 应为零矩阵。"""
    J, M = _random_case(6, 6)
    res = _resolve(J, M, 0.03)
    assert np.abs(res.N).max() < 1e-9
    assert res.nullspace_leakage_norm < 1e-9
