"""就位段任务空间参考轨迹的回归测试。

这里守住的核心事实：
  * 两端严格落在起点/终点，且速度、加速度都为零（否则交接处有阶跃）
  * 解析给出的速度/加速度确实是位置/姿态的导数（数值微分对照）
  * 角速度在世界系里是定轴的 —— 整个方案依赖这条，写错了姿态会走歪
  * 走的是【短程】测地线，不会绕远路
  * T<=0、θ→π 这两个边界不炸
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

from task_space_smc.core.approach_reference import (  # noqa: E402
    TaskSpaceApproach, quintic, rotation_exp, rotation_log)
from task_space_smc.core.quaternion_math import quat_to_rotation  # noqa: E402


def _rot(axis, deg):
    a = np.asarray(axis, dtype=float)
    return rotation_exp(a / np.linalg.norm(a) * np.radians(deg))


# ---------------------------------------------------------------- 时间标度

def test_quintic_endpoints_are_flat():
    """两端一阶二阶导都为零 —— 起步和到位没有加速度阶跃。"""
    for u in (0.0, 1.0):
        s, ds, dds = quintic(u)
        assert s == pytest.approx(u)
        assert ds == pytest.approx(0.0, abs=1e-12)
        assert dds == pytest.approx(0.0, abs=1e-12)


def test_quintic_clamps_outside_unit_interval():
    assert quintic(-5.0) == pytest.approx((0.0, 0.0, 0.0))
    assert quintic(+5.0) == pytest.approx((1.0, 0.0, 0.0))


def test_quintic_peak_values_match_the_constants_used_for_planning():
    """plan() 报的峰值速度/加速度用的是这两个常数，必须和实际极值一致。"""
    u = np.linspace(0.0, 1.0, 200001)
    v = np.array([quintic(x) for x in u])
    assert v[:, 1].max() == pytest.approx(TaskSpaceApproach.PEAK_DS, rel=1e-6)
    assert np.abs(v[:, 2]).max() == pytest.approx(
        TaskSpaceApproach.PEAK_DDS, rel=1e-4)


# ---------------------------------------------------------------- SO(3)

@pytest.mark.parametrize("deg", [0.0, 1e-5, 0.7, 30.0, 90.0, 179.0, 179.9999])
def test_rotation_log_exp_round_trip(deg):
    R = _rot([0.3, -0.7, 0.65], deg)
    assert np.allclose(rotation_exp(rotation_log(R)), R, atol=1e-9)


def test_rotation_log_takes_the_short_way():
    """350° 的旋转必须读成 −10°，不是 +350°。走远路会扫过没检查过的空间。"""
    R = _rot([0, 0, 1], 350.0)
    phi = rotation_log(R)
    assert np.degrees(np.linalg.norm(phi)) == pytest.approx(10.0, abs=1e-6)
    assert phi[2] < 0.0


def test_rotation_log_handles_exactly_180_degrees():
    R = _rot([0, 1, 0], 180.0)
    phi = rotation_log(R)
    assert np.degrees(np.linalg.norm(phi)) == pytest.approx(180.0, abs=1e-4)
    assert np.allclose(rotation_exp(phi), R, atol=1e-6)


# ---------------------------------------------------------------- 端点

def _make(T=6.0):
    return TaskSpaceApproach(
        start_position=[0.2468, 0.0325, 0.1954],
        start_rotation=_rot([0.2, 0.9, -0.3], 24.0),
        goal_position=[0.300, 0.040, 0.260],
        goal_rotation=_rot([0.2, 0.9, -0.3], 24.0) @ _rot([-0.5, 0.4, 0.77], 31.0),
        duration_sec=T,
    )


def test_endpoints_are_exact():
    a = _make()
    s0, s1 = a.sample(0.0), a.sample(a.duration_sec)
    assert np.allclose(s0.position, a.p0)
    assert np.allclose(quat_to_rotation(s0.quaternion_wxyz), a.R0, atol=1e-9)
    assert np.allclose(s1.position, a.p1)
    assert np.allclose(quat_to_rotation(s1.quaternion_wxyz), a.R1, atol=1e-9)


def test_endpoints_have_zero_velocity_and_acceleration():
    a = _make()
    for t in (0.0, a.duration_sec):
        s = a.sample(t)
        assert np.allclose(s.linear_velocity, 0.0, atol=1e-12)
        assert np.allclose(s.angular_velocity, 0.0, atol=1e-12)
        assert np.allclose(s.linear_acceleration, 0.0, atol=1e-12)
        assert np.allclose(s.angular_acceleration, 0.0, atol=1e-12)


def test_sampling_past_the_end_holds_the_goal():
    """到位后继续采样必须停在目标上，不能外推冲过去。"""
    a = _make()
    s = a.sample(a.duration_sec + 5.0)
    assert np.allclose(s.position, a.p1)
    assert np.allclose(s.linear_velocity, 0.0, atol=1e-12)
    assert a.finished(a.duration_sec + 5.0)
    assert not a.finished(0.5 * a.duration_sec)


def test_zero_duration_returns_the_goal_without_dividing_by_zero():
    a = TaskSpaceApproach([0, 0, 0], np.eye(3), [1, 2, 3], _rot([1, 0, 0], 20),
                          duration_sec=0.0)
    s = a.sample(0.0)
    assert np.allclose(s.position, [1, 2, 3])
    assert np.allclose(s.linear_velocity, 0.0)


# ---------------------------------------------------------------- 导数一致性

def test_linear_velocity_and_acceleration_are_the_true_derivatives():
    a = _make()
    h = 1e-6
    for t in np.linspace(0.3, 5.7, 12):
        v_num = (a.sample(t + h).position - a.sample(t - h).position) / (2 * h)
        acc_num = (a.sample(t + h).linear_velocity
                   - a.sample(t - h).linear_velocity) / (2 * h)
        assert np.allclose(a.sample(t).linear_velocity, v_num, atol=1e-6)
        assert np.allclose(a.sample(t).linear_acceleration, acc_num, atol=1e-5)


def test_angular_velocity_is_the_world_frame_derivative_of_the_rotation():
    """ω_w 必须满足 Ṙ = ω_w^ · R。写成体系或者写反了都会被这条抓住。"""
    a = _make()
    h = 1e-6
    for t in np.linspace(0.3, 5.7, 12):
        R = quat_to_rotation(a.sample(t).quaternion_wxyz)
        Rd = (quat_to_rotation(a.sample(t + h).quaternion_wxyz)
              - quat_to_rotation(a.sample(t - h).quaternion_wxyz)) / (2 * h)
        w = a.sample(t).angular_velocity
        W = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
        assert np.allclose(Rd, W @ R, atol=1e-6)


def test_angular_velocity_keeps_a_fixed_axis_in_world_frame():
    """整个方案建立在"世界系转轴恒定"这条推导上，直接钉死它。"""
    a = _make()
    dirs = []
    for t in np.linspace(0.5, 5.5, 20):
        w = a.sample(t).angular_velocity
        dirs.append(w / np.linalg.norm(w))
    dirs = np.array(dirs)
    assert np.allclose(dirs, dirs[0], atol=1e-9)
    assert np.allclose(dirs[0], a.axis_w / np.linalg.norm(a.axis_w), atol=1e-9)


# ---------------------------------------------------------------- 规划量

def test_plan_reports_the_quantities_used_for_the_safety_gate():
    a = _make(T=6.0)
    p = a.plan()
    assert p.distance_m == pytest.approx(
        float(np.linalg.norm(a.p1 - a.p0)))
    assert p.rotation_deg == pytest.approx(31.0, abs=1e-6)
    assert p.peak_linear_speed == pytest.approx(1.875 * p.distance_m / 6.0)
    # 与实际采样的极值对上
    v = max(np.linalg.norm(a.sample(t).linear_velocity)
            for t in np.linspace(0, 6, 20001))
    assert v == pytest.approx(p.peak_linear_speed, rel=1e-5)


def test_sampled_path_is_a_straight_line_between_the_endpoints():
    """末端位置走直线 —— 碰撞检查是按直线做的，弯了就检查不到。"""
    a = _make()
    pts = a.sample_positions(41)
    assert pts.shape == (41, 3)
    d = a.p1 - a.p0
    d = d / np.linalg.norm(d)
    for p in pts:
        r = p - a.p0
        assert np.allclose(r - np.dot(r, d) * d, 0.0, atol=1e-12)
    assert np.allclose(pts[0], a.p0)
    assert np.allclose(pts[-1], a.p1)
