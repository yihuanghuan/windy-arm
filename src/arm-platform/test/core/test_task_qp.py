"""保持任务 QP：约束不可越、姿态先让步、不可行时不振荡。

【这个测试的存在理由】
改造前的链路是"无约束求 qdd -> RNEA -> 后端 clip 力矩"。后端 clip 是抖振的
制造机：控制器算出一个它不被允许发的力矩，adapter 削掉，而控制器【永远不会
知道】自己被削了，下一拍照样再要一次。

把力矩箱搬进求解器之后，"不越界"就不再是一句承诺，而是可行集的构造性质。
这里逐条把它测成事实：
  * 力矩与力矩变化率【逐轴】不越界（不是"大致不越界"）；
  * 关节速度包线与限位制动在与力矩不冲突时被严格遵守；
  * 位置与姿态真冲突时，让步的必须是姿态；
  * 位置本身不可行时输出连续、不振荡；
  * 任意对抗输入下都必有解 —— 200 Hz 上求解失败等于软臂，
    而实测 joint2 失力后下沉 14.6°。
"""

import os
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from task_space_smc.core.dense_qp import OPTIMAL  # noqa: E402
from task_space_smc.core.task_qp import (  # noqa: E402
    HoldQPLimits, HoldQPWeights, build_qdd_box, build_torque_box,
    saturate_task_acceleration, solve_hold_qp)

RNG = np.random.default_rng(20260806)
N = 6

# 实机额定值。测试里写死，就是为了让"调高力矩上限"这件事必须先改测试。
TAU_MAX = np.array([9.0, 9.0, 9.0, 1.6, 1.6, 1.6])
TAU_RATE = np.array([1500.0, 1500.0, 1500.0, 500.0, 500.0, 350.0])
VLIM = np.array([3.0, 3.0, 3.0, 5.0, 5.0, 5.0])
QDD_MAX = np.array([40.0, 40.0, 40.0, 80.0, 80.0, 80.0])
DT = 0.005                      # 200 Hz


def _limits(dt=DT, velocity=VLIM, brake=0.5):
    return HoldQPLimits(
        torque=TAU_MAX, torque_rate=TAU_RATE, qdd_max=QDD_MAX, dt=dt,
        q_lower=np.full(N, -3.0), q_upper=np.full(N, 3.0),
        velocity=velocity, brake_safety=brake)


def _mass(scale=0.5):
    A = RNG.normal(size=(N, N))
    return A @ A.T + scale * np.eye(N)


def _solve(J, M, c, y, q, qd, qdd_prev, tau_prev, weights=None, limits=None):
    return solve_hold_qp(
        J, M, c, y, q, qd, qdd_prev, tau_prev,
        q_nominal=np.zeros(N), kp_null=np.full(N, 2.0), kd_null=np.full(N, 3.0),
        weights=weights or HoldQPWeights(), limits=limits or _limits())


# ---------------- 力矩与变化率

def test_torque_never_exceeds_the_rated_box():
    """2000 组随机输入下，逐轴力矩恒不越额定。

    这是力矩箱作为【硬】约束的直接断言。它硬不硬不是权重调出来的：
    tau 就是决策变量，箱子就是它的取值域。
    """
    for _ in range(2000):
        M = _mass()
        y = RNG.normal(size=6)
        y = y / np.linalg.norm(y) * RNG.uniform(0.0, 8.5)
        r = _solve(RNG.normal(size=(6, N)), M, RNG.normal(size=N) * 3.0, y,
                   RNG.uniform(-2, 2, N), RNG.normal(size=N) * 0.5,
                   np.zeros(N), RNG.normal(size=N) * 2.0)
        assert r.status == OPTIMAL
        assert np.all(np.abs(r.tau) <= TAU_MAX + 1e-9), np.round(r.tau, 4)


def test_torque_rate_never_exceeds_the_configured_slew():
    """3000 拍连续滚动，逐轴 |tau_k - tau_{k-1}| 不超 rate*dt。

    改造前这条限制【只】活在 actuator_adapter 的事后 clip 里，控制器完全不
    知情 —— 那是教科书级的极限环发生器。放进可行集之后它由构造成立。
    """
    M0 = _mass()
    J0 = RNG.normal(size=(6, N))
    q, qd = np.zeros(N), np.zeros(N)
    qdd_prev, tau_prev = np.zeros(N), np.zeros(N)
    worst = np.zeros(N)
    for k in range(3000):
        t = k * DT
        # 需求故意给得比实机激进，逼着解去贴边界。
        y = np.concatenate([3.0 * np.sin(5 * t) * np.ones(3),
                            8.0 * np.cos(7 * t) * np.ones(3)])
        r = _solve(J0 * (1 + 0.4 * np.sin(0.7 * t)), M0,
                   6.0 * np.sin(np.arange(N) + t), y, q, qd, qdd_prev, tau_prev)
        assert r.status == OPTIMAL
        worst = np.maximum(worst, np.abs(r.tau - tau_prev))
        qdd_prev, tau_prev = r.qdd, r.tau
        qd = np.clip(qd + r.qdd * DT, -VLIM, VLIM)
        q = np.clip(q + qd * DT, -3.0, 3.0)
    assert np.all(worst <= TAU_RATE * DT + 1e-9), np.round(worst, 4)


def test_rated_torque_is_not_quietly_raised():
    """力矩上限必须严格等于 [9,9,9,1.6,1.6,1.6]。

    调参会话里最容易发生的事就是"先把上限抬一点看看"。测试是唯一能活过
    那种会话的东西。
    """
    assert np.array_equal(TAU_MAX, np.array([9.0, 9.0, 9.0, 1.6, 1.6, 1.6]))
    lo, hi = build_torque_box(np.zeros(N), _limits(dt=10.0))   # dt 大到 rate 不起作用
    assert np.allclose(hi, TAU_MAX)
    assert np.allclose(lo, -TAU_MAX)


# ---------------- 速度包线与限位

def test_next_step_velocity_stays_inside_the_plant_envelope():
    """力矩够用时，下一拍关节速度不许冲出包线。"""
    M = np.eye(N) * 0.6
    for _ in range(400):
        qd = RNG.uniform(-1.0, 1.0, N) * VLIM * 0.9
        r = _solve(RNG.normal(size=(6, N)), M, np.zeros(N),
                   RNG.normal(size=6) * 0.5, np.zeros(N), qd,
                   np.zeros(N), np.zeros(N))
        assert r.status == OPTIMAL
        if np.max(r.qdd_slack) > 1e-6:
            continue                      # 与力矩箱真冲突，按设计让步
        assert np.all(np.abs(qd + r.qdd * DT) <= VLIM + 1e-6)


def test_position_wall_is_braked_not_hit():
    """贴着关节限位时，加速度箱必须要求减速而不是继续冲。"""
    q = np.full(N, 2.95)                  # 限位 3.0，只剩 50 mm
    qd = np.full(N, 1.0)                  # 还在往限位冲
    lo, hi = build_qdd_box(q, qd, np.eye(N) * 0.6, _limits())
    assert np.all(hi < 0.0), "还在朝限位跑，加速度上界必须是负的（即必须减速）"


def test_qdd_box_is_never_empty():
    """速度已经冲出包线时，箱子仍必须非空 —— 否则求解器无解可给。"""
    for _ in range(2000):
        q = RNG.uniform(-4, 4, N)             # 已经越过限位
        qd = RNG.normal(size=N) * 20.0        # 远超包线
        lo, hi = build_qdd_box(q, qd, _mass(1e-3), _limits())
        assert np.all(hi >= lo)


# ---------------- 分层

def test_orientation_yields_while_position_is_held_when_they_conflict():
    """位置与姿态真冲突时，让步的必须是姿态。这是分层需求本身。

    构造一个秩亏雅可比：位置三维可达，姿态只有一维可达。六维需求整体不可行，
    解必须把可用自由度先喂给位置。
    """
    J = np.vstack([np.eye(3, N), np.zeros((3, N))])
    J[3, 3] = 1.0
    r = _solve(J, np.eye(N) * 0.4, np.zeros(N),
               np.array([1.0, 1.0, 1.0, 5.0, 5.0, 5.0]),
               np.zeros(N), np.zeros(N), np.zeros(N), np.zeros(N))
    assert r.status == OPTIMAL
    pos = float(np.linalg.norm(r.position_residual))
    ori = float(np.linalg.norm(r.orientation_residual))
    assert ori > 1.0, "这个算例没有制造出真冲突，测试就没测到东西"
    assert pos < 1e-3 * ori, "位置残差 %.3e 相对姿态残差 %.3e 不够小" % (pos, ori)


def test_position_accuracy_does_not_degrade_as_orientation_demand_grows():
    """姿态需求扫三个数量级，位置残差不许跟着涨。

    这才是"优先级反转"的准确定义：姿态要得越凶，位置被偷走得越多。用比值做
    判据是错的 —— 姿态残差本来就随需求线性增长，比值会自动变好看，反而掩盖
    真正的反转。这里直接钉住位置残差本身。
    """
    J = np.vstack([np.eye(3, N), np.zeros((3, N))])
    J[3, 3] = 1.0
    residuals = []
    for demand in (0.5, 5.0, 50.0, 500.0):
        r = _solve(J, np.eye(N) * 0.4, np.zeros(N),
                   np.array([1.0, 1.0, 1.0, demand, demand, demand]),
                   np.zeros(N), np.zeros(N), np.zeros(N), np.zeros(N))
        assert r.status == OPTIMAL
        assert np.linalg.norm(r.orientation_residual) > 1.0 * (demand / 5.0) * 0.1
        residuals.append(float(np.linalg.norm(r.position_residual)))
    # 姿态需求放大 1000 倍，位置残差的变化必须小于自身的 1%。
    assert max(residuals) - min(residuals) < 0.01 * max(residuals), residuals
    # 而且位置残差本身要小：它只应来自正则化/居中项，不应来自姿态。
    assert max(residuals) < 1e-2, residuals


# ---------------- 连续性 / 不振荡

def test_output_is_continuous_when_the_position_task_becomes_infeasible():
    """把力矩上限从够用连续压到不够用，输出不许跳。"""
    J = RNG.normal(size=(6, N))
    M = _mass()
    steps = []
    previous = None
    for scale in np.linspace(1.0, 0.02, 600):
        limits = HoldQPLimits(
            torque=TAU_MAX * scale, torque_rate=TAU_RATE, qdd_max=QDD_MAX,
            dt=DT, q_lower=np.full(N, -3.0), q_upper=np.full(N, 3.0),
            velocity=VLIM, brake_safety=0.5)
        r = _solve(J, M, np.zeros(N), np.array([2.0, 2.0, 2.0, 1.0, 1.0, 1.0]),
                   np.zeros(N), np.zeros(N), np.zeros(N), np.zeros(N),
                   limits=limits)
        assert r.status == OPTIMAL
        if previous is not None:
            steps.append(float(np.linalg.norm(r.tau - previous)))
        previous = r.tau
    steps = np.array(steps)
    assert steps.max() < 10.0 * np.median(steps[steps > 0])


def test_infeasible_position_does_not_oscillate():
    """目标够不着时，力矩不许来回换向。

    这就是后端 clip 抖振的复现场景：需求恒定且远超能力。解应该稳定贴在
    边界上，而不是每拍在两侧之间弹。
    """
    J = np.vstack([np.eye(3, N), np.zeros((3, N))])
    J[3, 3] = 1.0
    M = np.eye(N) * 0.4
    # 关节限位放宽，让【速度包线】成为唯一起作用的约束：否则臂会撞上限位停住，
    # 力矩收敛到零，测出来的"不振荡"是假的（零附近的符号翻转全是浮点噪声）。
    limits = HoldQPLimits(
        torque=TAU_MAX, torque_rate=TAU_RATE, qdd_max=QDD_MAX, dt=DT,
        q_lower=np.full(N, -1e6), q_upper=np.full(N, 1e6),
        velocity=VLIM, brake_safety=0.5)
    q, qd = np.zeros(N), np.zeros(N)
    qdd_prev, tau_prev = np.zeros(N), np.zeros(N)
    history = []
    for _ in range(2000):
        # 恒定且完全够不着的需求（相当于目标在 20 cm 之外）
        r = _solve(J, M, np.zeros(N), np.array([50.0, 50.0, 50.0, 0.0, 0.0, 0.0]),
                   q, qd, qdd_prev, tau_prev, limits=limits)
        assert r.status == OPTIMAL
        history.append(r.tau.copy())
        qdd_prev, tau_prev = r.qdd, r.tau
        qd = np.clip(qd + r.qdd * DT, -VLIM, VLIM)
        q = q + qd * DT
    tail = np.array(history[500:])          # 掐掉起始爬升段
    # 恒定输入必须给出恒定输出：这正是"严格凸 => 解唯一 => 对输入连续"的
    # 直接后果，也正是后端 clip 做不到的那件事。
    spread = tail.max(axis=0) - tail.min(axis=0)
    assert np.all(spread < 1e-6), "力矩在稳态仍有 %s N*m 的摆动" % np.round(spread, 9)
    assert np.all(np.abs(tail) <= TAU_MAX + 1e-9)


def test_a_solution_always_exists_even_on_adversarial_input():
    """5000 组对抗输入必须条条有解。

    200 Hz 上一次硬失败就是一拍软臂，而软臂在这台机器上是实测会下沉的。
    """
    for k in range(5000):
        M = _mass(1e-3)
        J = RNG.normal(size=(6, N))
        if k % 3 == 0:                       # 近奇异
            u, s, vt = np.linalg.svd(J)
            s[-1] = 1e-9
            J = (u * s) @ vt
        r = _solve(J, M, RNG.normal(size=N) * 30.0, RNG.normal(size=6) * 100.0,
                   RNG.uniform(-4, 4, N),          # 已越限位
                   RNG.normal(size=N) * 20.0,      # 已超包线
                   RNG.normal(size=N) * 50.0,
                   RNG.choice([-1.0, 1.0], N) * TAU_MAX)   # 上一拍贴轨
        assert r.status == OPTIMAL
        assert np.all(np.isfinite(r.tau))
        assert np.all(np.abs(r.tau) <= TAU_MAX + 1e-9)


# ---------------- 任务加速度饱和

def test_task_acceleration_saturation_preserves_direction():
    """范数饱和必须保方向：末端沿指令方向滞后，不能拐弯。"""
    nu = np.array([30.0, 0.0, 40.0, 0.0, 20.0, 0.0])
    out, (s_lin, s_ang) = saturate_task_acceleration(nu, 3.0, 8.0)
    assert np.isclose(np.linalg.norm(out[:3]), 3.0)
    assert np.isclose(np.linalg.norm(out[3:]), 8.0)
    assert np.allclose(out[:3] / np.linalg.norm(out[:3]),
                       nu[:3] / np.linalg.norm(nu[:3]))
    assert s_lin < 1.0 and s_ang < 1.0


def test_task_acceleration_below_the_cap_is_untouched():
    nu = np.array([0.4, -0.2, 0.1, 0.3, 0.0, -0.5])
    out, scales = saturate_task_acceleration(nu, 3.0, 8.0)
    assert np.allclose(out, nu)
    assert scales == (1.0, 1.0)


# ---------------- 配置一致性

def _hold_config():
    path = os.path.join(os.path.dirname(__file__), "..", "..",
                        "task_space_smc", "config", "smc_params_hold.yaml")
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def test_hold_profile_declares_the_velocity_envelope():
    """hold 配置必须给出 joint_velocity / joint_brake_safety。

    这两个键此前【缺失】，于是 runner 解析出 feasibility_velocity=None，
    _feasibility_limits() 直接返回 None，速度约束与位置制动约束【全部消失】
    —— 而且是静默消失，日志里什么都看不到。
    """
    limits = _hold_config()["limits"]
    assert "joint_velocity" in limits, "缺 joint_velocity，约束会静默失效"
    assert "joint_brake_safety" in limits
    assert np.allclose(limits["joint_velocity"], VLIM)
    assert limits["joint_brake_safety"] > 0.0


def test_hold_profile_torque_limits_are_the_rated_values():
    limits = _hold_config()["limits"]
    assert np.allclose(limits["torque"], TAU_MAX)
    assert np.allclose(limits["current"], TAU_MAX)
    assert np.allclose(limits["torque_rate"], TAU_RATE)
