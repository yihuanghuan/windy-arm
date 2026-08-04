"""交接偏置衰减的回归测试。

守住的核心事实：
  * τ=0 时参考量【就是】实际位姿 —— 交接误差恒为零，这是整个机制的全部意义
  * τ=T 之后偏置完全消失，不残留轨迹畸变
  * 速度/加速度前馈与位姿解析一致（数值微分校验），否则会自造误差
  * 角速度里 ω_traj 必须先被 R_off 旋转，不是简单相加
  * duration=0 时是逐位恒等变换，已有仿真基线不受影响
"""

import os
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from task_space_smc.handover_blend import (  # noqa: E402
    HandoverBlend, smoothstep_decay)
from task_space_smc.handover_blend import (  # noqa: E402
    _exp_so3, _log_so3)


def _rot(axis, angle):
    return _exp_so3(np.asarray(axis, dtype=float)
                    / np.linalg.norm(axis) * angle)


def _traj(t):
    """一条解析参考轨迹，位姿/速度/加速度严格自洽。"""
    w = 1.7
    p = np.array([0.30 + 0.04 * np.cos(w * t), 0.04 * np.sin(w * t), 0.26])
    v = np.array([-0.04 * w * np.sin(w * t), 0.04 * w * np.cos(w * t), 0.0])
    a = np.array([-0.04 * w**2 * np.cos(w * t),
                  -0.04 * w**2 * np.sin(w * t), 0.0])
    axis = np.array([0.0, 0.0, 1.0])
    R = _exp_so3(axis * (0.3 * np.sin(0.9 * t)))
    omega = axis * (0.3 * 0.9 * np.cos(0.9 * t))
    alpha = axis * (-0.3 * 0.81 * np.sin(0.9 * t))
    return p, R, v, omega, a, alpha


def _blend_at(blend, t):
    return blend.apply(t, *_traj(t))


# --------------------------------------------------------------------------
# smoothstep 本身
# --------------------------------------------------------------------------

def test_smoothstep_endpoints_are_c2():
    T = 3.0
    s0, sd0, sdd0 = smoothstep_decay(0.0, T)
    sT, sdT, sddT = smoothstep_decay(T, T)
    assert (s0, sd0, sdd0) == pytest.approx((1.0, 0.0, 0.0))
    assert (sT, sdT, sddT) == pytest.approx((0.0, 0.0, 0.0))


def test_smoothstep_is_monotone_decreasing():
    T = 3.0
    s = [smoothstep_decay(t, T)[0] for t in np.linspace(0, T, 200)]
    assert np.all(np.diff(s) <= 1e-12)


def test_smoothstep_derivatives_match_numerical():
    T = 2.5
    h = 1e-6
    for tau in (0.4, 1.0, 1.7, 2.2):
        s, sd, sdd = smoothstep_decay(tau, T)
        sp = smoothstep_decay(tau + h, T)[0]
        sm = smoothstep_decay(tau - h, T)[0]
        assert sd == pytest.approx((sp - sm) / (2 * h), abs=1e-5)
        assert sdd == pytest.approx((sp - 2 * s + sm) / h**2, abs=1e-2)


def test_peak_second_derivative_matches_the_documented_constant():
    """docstring 用 5.7735/T² 播报额外加速度，这里核对该常数。"""
    T = 3.0
    sdd = np.array([abs(smoothstep_decay(t, T)[2])
                    for t in np.linspace(0, T, 20001)])
    assert sdd.max() * T**2 == pytest.approx(10 * np.sqrt(3) / 3, rel=1e-3)


def test_zero_duration_is_inert():
    assert smoothstep_decay(0.0, 0.0) == (0.0, 0.0, 0.0)
    assert smoothstep_decay(5.0, -1.0) == (0.0, 0.0, 0.0)


# --------------------------------------------------------------------------
# 交接误差恒为零 —— 整个机制的目的
# --------------------------------------------------------------------------

def test_reference_equals_actual_pose_at_handover():
    """τ=0 时混合后的参考位姿必须逐位等于实际位姿。"""
    blend = HandoverBlend(3.0)
    t0 = 1.234
    p_traj, R_traj = _traj(t0)[0], _traj(t0)[1]
    p_act = p_traj + np.array([0.021, -0.008, 0.011])   # 22 mm 量级残差
    R_act = _rot([0.3, -0.7, 0.5], np.radians(20.7)) @ R_traj

    blend.capture(t0, p_act, R_act, p_traj, R_traj)
    p, R, v, w, a, wd = _blend_at(blend, t0)
    np.testing.assert_allclose(p, p_act, atol=1e-12)
    np.testing.assert_allclose(R, R_act, atol=1e-12)


def test_handover_position_error_is_exactly_zero_for_any_offset():
    rng = np.random.default_rng(7)
    for _ in range(30):
        blend = HandoverBlend(3.0)
        t0 = float(rng.uniform(0, 5))
        p_traj, R_traj = _traj(t0)[0], _traj(t0)[1]
        p_act = p_traj + rng.normal(0, 0.05, 3)
        R_act = _exp_so3(rng.normal(0, 0.5, 3)) @ R_traj
        blend.capture(t0, p_act, R_act, p_traj, R_traj)
        p, R = _blend_at(blend, t0)[:2]
        assert np.linalg.norm(p - p_act) < 1e-12
        assert np.linalg.norm(_log_so3(R @ R_act.T)) < 1e-9


def test_offset_is_gone_after_the_blend_window():
    blend = HandoverBlend(3.0)
    t0 = 0.0
    p_traj, R_traj = _traj(t0)[0], _traj(t0)[1]
    blend.capture(t0, p_traj + np.array([0.02, 0.02, 0.02]),
                  _rot([1, 1, 0], 0.3) @ R_traj, p_traj, R_traj)
    for t in (3.0, 3.5, 10.0):
        got = _blend_at(blend, t)
        want = _traj(t)
        for g, w in zip(got, want):
            np.testing.assert_allclose(g, w, atol=1e-12)


def test_blend_scale_decays_monotonically_to_zero():
    blend = HandoverBlend(3.0)
    p_traj, R_traj = _traj(0.0)[0], _traj(0.0)[1]
    blend.capture(0.0, p_traj + 0.02, R_traj, p_traj, R_traj)
    s = [blend.scale(t) for t in np.linspace(0, 3, 100)]
    assert s[0] == pytest.approx(1.0)
    assert s[-1] == pytest.approx(0.0)
    assert np.all(np.diff(s) <= 1e-12)


# --------------------------------------------------------------------------
# 前馈自洽性 —— 数值微分校验
# --------------------------------------------------------------------------

def _make_blend():
    blend = HandoverBlend(3.0)
    p_traj, R_traj = _traj(0.5)[0], _traj(0.5)[1]
    blend.capture(0.5, p_traj + np.array([0.021, -0.008, 0.011]),
                  _rot([0.3, -0.7, 0.5], np.radians(20.7)) @ R_traj,
                  p_traj, R_traj)
    return blend


@pytest.mark.parametrize("t", [0.5, 1.0, 1.7, 2.6, 3.4])
def test_linear_velocity_and_acceleration_are_consistent(t):
    blend = _make_blend()
    h = 1e-5
    p = _blend_at(blend, t)[0]
    pp = _blend_at(blend, t + h)[0]
    pm = _blend_at(blend, t - h)[0]
    v = _blend_at(blend, t)[2]
    a = _blend_at(blend, t)[4]
    np.testing.assert_allclose(v, (pp - pm) / (2 * h), atol=2e-5)
    np.testing.assert_allclose(a, (pp - 2 * p + pm) / h**2, atol=5e-2)


@pytest.mark.parametrize("t", [0.5, 1.0, 1.7, 2.6, 3.4])
def test_angular_velocity_is_consistent_with_the_rotation(t):
    """世界系角速度：Ṙ = [ω]× R。数值微分 R 反推 ω，与解析式比较。"""
    blend = _make_blend()
    h = 1e-5
    R = _blend_at(blend, t)[1]
    Rp = _blend_at(blend, t + h)[1]
    Rm = _blend_at(blend, t - h)[1]
    Rd = (Rp - Rm) / (2 * h)
    W = Rd @ R.T
    W = 0.5 * (W - W.T)
    omega_num = np.array([W[2, 1], W[0, 2], W[1, 0]])
    np.testing.assert_allclose(_blend_at(blend, t)[3], omega_num, atol=2e-4)


@pytest.mark.parametrize("t", [0.6, 1.2, 2.0, 2.8])
def test_angular_acceleration_is_consistent(t):
    blend = _make_blend()
    h = 1e-4
    wp = _blend_at(blend, t + h)[3]
    wm = _blend_at(blend, t - h)[3]
    np.testing.assert_allclose(
        _blend_at(blend, t)[5], (wp - wm) / (2 * h), atol=2e-3)


def test_omega_traj_must_be_rotated_by_the_offset():
    """守住一个容易写错的地方：ω_ref = s'·ω̂ + R_off·ω_traj，不是直接相加。

    构造一个大姿态偏置，使 R_off·ω_traj 明显不等于 ω_traj。
    """
    blend = HandoverBlend(3.0)
    p_traj, R_traj = _traj(0.0)[0], _traj(0.0)[1]
    blend.capture(0.0, p_traj, _rot([1.0, 0.0, 0.0], 1.2) @ R_traj,
                  p_traj, R_traj)
    w_blend = _blend_at(blend, 0.9)[3]
    s, sd, _ = smoothstep_decay(0.9, 3.0)
    omega_traj = _traj(0.9)[3]
    naive = sd * blend.delta_rotvec + omega_traj          # 错误写法：直接相加
    correct = sd * blend.delta_rotvec + _exp_so3(
        s * blend.delta_rotvec) @ omega_traj
    np.testing.assert_allclose(w_blend, correct, atol=1e-12)
    assert np.linalg.norm(w_blend - naive) > 1e-3, (
        "这个用例没能区分两种写法，需要更大的姿态偏置")


# --------------------------------------------------------------------------
# 关闭时必须是恒等变换
# --------------------------------------------------------------------------

def test_disabled_blend_is_bitwise_identity():
    blend = HandoverBlend(0.0)
    assert not blend.enabled
    p_traj, R_traj = _traj(0.0)[0], _traj(0.0)[1]
    blend.capture(0.0, p_traj + 0.5, _rot([1, 0, 0], 1.0) @ R_traj,
                  p_traj, R_traj)
    for t in (0.0, 1.0, 5.0):
        for got, want in zip(_blend_at(blend, t), _traj(t)):
            np.testing.assert_array_equal(got, want)
    assert blend.scale(0.0) == 0.0
    assert blend.peak_extra_acceleration() == 0.0


def test_capture_is_idempotent():
    """偏置必须是常量。第二次 capture 不得改写它，否则参考量会跳。"""
    blend = HandoverBlend(3.0)
    p_traj, R_traj = _traj(0.0)[0], _traj(0.0)[1]
    blend.capture(0.0, p_traj + 0.02, R_traj, p_traj, R_traj)
    first = blend.delta_position.copy()
    blend.capture(1.0, p_traj + 0.99, R_traj, p_traj, R_traj)
    np.testing.assert_array_equal(blend.delta_position, first)
    assert blend.t0 == 0.0


def test_apply_before_capture_is_identity():
    blend = HandoverBlend(3.0)
    for got, want in zip(_blend_at(blend, 1.0), _traj(1.0)):
        np.testing.assert_array_equal(got, want)


def test_negative_duration_is_rejected():
    with pytest.raises(ValueError, match="duration_sec"):
        HandoverBlend(-1.0)


# --------------------------------------------------------------------------
# 额外加速度的量级 —— 这是"不会把机械臂甩出去"的保证
# --------------------------------------------------------------------------

def test_extra_acceleration_stays_far_below_the_actuator_ceiling():
    """2026-08-03 位形下，六轴额定力矩折到末端约 48.8 m/s²。

    偏置衰减自身贡献必须远低于它，否则这个机制本身就成了新的饱和源。
    """
    blend = HandoverBlend(3.0)
    p_traj, R_traj = _traj(0.0)[0], _traj(0.0)[1]
    blend.capture(0.0, p_traj + np.array([0.0221, 0.0, 0.0]),
                  R_traj, p_traj, R_traj)
    peak = blend.peak_extra_acceleration()
    assert peak == pytest.approx(0.0142, abs=0.001)
    assert peak < 0.01 * 48.8


def test_shorter_blend_raises_extra_acceleration_quadratically():
    p_traj, R_traj = _traj(0.0)[0], _traj(0.0)[1]
    peaks = []
    for T in (1.0, 2.0, 4.0):
        b = HandoverBlend(T)
        b.capture(0.0, p_traj + np.array([0.0221, 0.0, 0.0]),
                  R_traj, p_traj, R_traj)
        peaks.append(b.peak_extra_acceleration())
    np.testing.assert_allclose(peaks[0] / peaks[1], 4.0, rtol=1e-9)
    np.testing.assert_allclose(peaks[1] / peaks[2], 4.0, rtol=1e-9)


# --------------------------------------------------------------------------
# 配置一致性
# --------------------------------------------------------------------------

def _config(name):
    from pathlib import Path
    import task_space_smc
    path = (Path(task_space_smc.__file__).resolve().parent
            / "config" / f"smc_params_{name}.yaml")
    if not path.exists():
        return None
    with open(path) as fh:
        return yaml.safe_load(fh) or {}


def test_hwtest_enables_the_blend():
    cfg = _config("hwtest")
    assert cfg is not None
    blend_sec = cfg["handover"]["reference_blend_sec"]
    assert blend_sec > 0.0, "实机配置必须开启，否则交接必饱和"
    # 22 mm 残差下额外加速度要远低于 48.8 m/s² 的能力上限
    assert 10 * np.sqrt(3) / 3 * 0.0221 / blend_sec**2 < 0.5


def test_simulation_baselines_leave_the_blend_off():
    checked = 0
    for name in ("whole6", "whole"):
        cfg = _config(name)
        if cfg is None:
            continue
        checked += 1
        assert float((cfg.get("handover") or {}).get(
            "reference_blend_sec", 0.0)) == 0.0, (
            f"{name} 是仿真基线，开启交接混合会改变其已验证的误差曲线")
    assert checked > 0


# --------------------------------------------------------------------------
# 5D 笔轴口径 —— 绕笔轴的自转是自由量，不得当成偏置
# --------------------------------------------------------------------------

def test_pen_axis_capture_ignores_free_spin_about_the_pen():
    """守住一个实测踩到的坑。

    approach 已经把笔轴对齐、位置收敛到 0.04 mm 时，按 6D 口径取 R_act·R_refᵀ
    报出的姿态偏置是 120.8° —— 那全是绕笔轴的自由自转。按 5D 口径应该≈0。
    """
    pen = np.array([1.0, 0.0, 0.0])
    R_ref = _rot([0.2, 0.9, -0.3], 0.7)
    R_act = R_ref @ _exp_so3(pen * 2.109)      # 只绕笔轴自转 120.8°
    p = np.array([0.3, 0.04, 0.26])

    six = HandoverBlend(3.0)
    six.capture(0.0, p, R_act, p, R_ref)
    assert np.degrees(np.linalg.norm(six.delta_rotvec)) == pytest.approx(
        120.8, abs=0.5), "用例没构造出预期的自由自转"

    five = HandoverBlend(3.0)
    five.capture(0.0, p, R_act, p, R_ref, pen_axis_local=pen)
    assert np.linalg.norm(five.delta_rotvec) < 1e-9, (
        "5D 口径把绕笔轴的自由自转当成了偏置")


def test_pen_axis_capture_still_corrects_a_real_axis_misalignment():
    pen = np.array([1.0, 0.0, 0.0])
    R_ref = np.eye(3)
    R_act = _rot([0.0, 0.0, 1.0], np.radians(20.7)) @ R_ref
    p = np.zeros(3)
    b = HandoverBlend(3.0)
    b.capture(0.0, p, R_act, p, R_ref, pen_axis_local=pen)
    assert np.degrees(np.linalg.norm(b.delta_rotvec)) == pytest.approx(
        20.7, abs=1e-6)
    R_blend = b.apply(0.0, p, R_ref, np.zeros(3), np.zeros(3),
                      np.zeros(3), np.zeros(3))[1]
    np.testing.assert_allclose(R_blend @ pen, R_act @ pen, atol=1e-12)


def test_pen_axis_blend_never_injects_spin_about_the_pen():
    """偏置轴必须垂直于笔轴 —— 否则就是在参考量里造自转。"""
    rng = np.random.default_rng(3)
    pen = np.array([1.0, 0.0, 0.0])
    for _ in range(50):
        R_ref = _exp_so3(rng.normal(0, 1.0, 3))
        R_act = _exp_so3(rng.normal(0, 1.0, 3))
        b = HandoverBlend(3.0)
        b.capture(0.0, np.zeros(3), R_act, np.zeros(3), R_ref,
                  pen_axis_local=pen)
        assert abs(np.dot(b.delta_rotvec, R_ref @ pen)) < 1e-9, "偏置含绕笔轴分量"


def test_antiparallel_pen_axes_do_not_produce_nan():
    pen = np.array([1.0, 0.0, 0.0])
    R_ref = np.eye(3)
    R_act = _rot([0.0, 0.0, 1.0], np.pi)      # 笔轴完全反向
    b = HandoverBlend(3.0)
    b.capture(0.0, np.zeros(3), R_act, np.zeros(3), R_ref, pen_axis_local=pen)
    assert np.all(np.isfinite(b.delta_rotvec))
    assert np.degrees(np.linalg.norm(b.delta_rotvec)) == pytest.approx(180.0)
    R_blend = b.apply(0.0, np.zeros(3), R_ref, np.zeros(3), np.zeros(3),
                      np.zeros(3), np.zeros(3))[1]
    np.testing.assert_allclose(R_blend @ pen, R_act @ pen, atol=1e-9)


# --------------------------------------------------------------------------
# 速度匹配 —— 2026-08-03 上机证明只补位置不够
# --------------------------------------------------------------------------

from task_space_smc.handover_blend import velocity_decay  # noqa: E402


def test_velocity_basis_endpoint_conditions():
    T = 3.0
    assert velocity_decay(0.0, T) == pytest.approx((0.0, 1.0, 0.0))
    assert velocity_decay(T, T) == pytest.approx((0.0, 0.0, 0.0))


def test_velocity_basis_derivatives_match_numerical():
    T = 2.5
    h = 1e-6
    for tau in (0.3, 1.0, 1.8, 2.3):
        v, d, dd = velocity_decay(tau, T)
        vp = velocity_decay(tau + h, T)[0]
        vm = velocity_decay(tau - h, T)[0]
        assert d == pytest.approx((vp - vm) / (2 * h), abs=1e-5)
        assert dd == pytest.approx((vp - 2 * v + vm) / h**2, abs=1e-2)


def test_velocity_basis_peak_second_derivative_constant():
    """peak_extra_acceleration 用 3.9403/T，这里核对。"""
    T = 3.0
    dd = np.array([abs(velocity_decay(t, T)[2])
                   for t in np.linspace(0, T, 40001)])
    assert dd.max() * T == pytest.approx(3.9403, rel=1e-3)


def test_reference_matches_actual_velocity_at_handover():
    """τ=0 时参考速度必须【等于】实际速度，滑模变量两项同时为零。"""
    blend = HandoverBlend(3.0)
    t0 = 0.7
    p_traj, R_traj, v_traj = _traj(t0)[0], _traj(t0)[1], _traj(t0)[2]
    p_act = p_traj + np.array([0.021, -0.008, 0.011])
    v_act = np.array([0.158, 0.007, -0.087])          # 实测量级 0.180 m/s
    blend.capture(t0, p_act, R_traj, p_traj, R_traj,
                  actual_linear_velocity=v_act,
                  reference_linear_velocity=v_traj)
    p, _, v = _blend_at(blend, t0)[:3]
    np.testing.assert_allclose(p, p_act, atol=1e-12)
    np.testing.assert_allclose(v, v_act, atol=1e-12)


def test_position_only_blend_leaves_the_velocity_error_that_killed_run5():
    """不传速度时速度误差原样保留 —— 这正是 2026-08-03 交接后 70 ms 触发看门狗的量。"""
    t0 = 0.7
    p_traj, R_traj, v_traj = _traj(t0)[0], _traj(t0)[1], _traj(t0)[2]
    v_act = v_traj + np.array([0.158, 0.007, -0.087])
    b = HandoverBlend(3.0)
    b.capture(t0, p_traj, R_traj, p_traj, R_traj)      # 只补位置
    v = _blend_at(b, t0)[2]
    assert np.linalg.norm(v - v_act) == pytest.approx(0.1804, abs=1e-3)
    # 位置+速度都补时残差为零
    b2 = HandoverBlend(3.0)
    b2.capture(t0, p_traj, R_traj, p_traj, R_traj,
               actual_linear_velocity=v_act, reference_linear_velocity=v_traj)
    assert np.linalg.norm(_blend_at(b2, t0)[2] - v_act) < 1e-12


def test_velocity_offset_is_gone_after_the_window():
    blend = HandoverBlend(3.0)
    p_traj, R_traj, v_traj = _traj(0.0)[0], _traj(0.0)[1], _traj(0.0)[2]
    blend.capture(0.0, p_traj + 0.02, R_traj, p_traj, R_traj,
                  actual_linear_velocity=v_traj + 0.2,
                  reference_linear_velocity=v_traj)
    for t in (3.0, 4.0, 12.0):
        for got, want in zip(_blend_at(blend, t), _traj(t)):
            np.testing.assert_allclose(got, want, atol=1e-12)


@pytest.mark.parametrize("t", [0.4, 1.1, 2.2, 2.9])
def test_velocity_blend_stays_analytically_consistent(t):
    """加了速度项后，位置/速度/加速度三者仍须互为导数。

    只取区间内部：τ=0 处 B(τ) 对 τ<0 是钳位的（B=0 而 B'=1），中心差分会
    跨到钳位区，测出来是真实斜率的一半 —— 那是取点问题，不是解析式问题。
    端点条件另由 test_reference_matches_actual_velocity_at_handover 锁定。
    """
    blend = HandoverBlend(3.0)
    p_traj, R_traj, v_traj = _traj(0.0)[0], _traj(0.0)[1], _traj(0.0)[2]
    blend.capture(0.0, p_traj + np.array([0.028, 0.0, 0.0]), R_traj,
                  p_traj, R_traj,
                  actual_linear_velocity=v_traj + np.array([0.158, 0.007, -0.087]),
                  reference_linear_velocity=v_traj)
    h = 1e-5
    p = _blend_at(blend, t)[0]
    pp = _blend_at(blend, t + h)[0]
    pm = _blend_at(blend, t - h)[0]
    np.testing.assert_allclose(_blend_at(blend, t)[2], (pp - pm) / (2 * h), atol=5e-5)
    np.testing.assert_allclose(_blend_at(blend, t)[4], (pp - 2 * p + pm) / h**2, atol=5e-2)


def test_extra_acceleration_with_measured_run5_offsets():
    """实测 Δp=28 mm、Δv=0.180 m/s、T=3 s 下额外加速度必须远低于 48.8 m/s²。"""
    b = HandoverBlend(3.0)
    b.delta_position = np.array([0.028, 0.0, 0.0])
    b.delta_velocity = np.array([0.1804, 0.0, 0.0])
    peak = b.peak_extra_acceleration()
    assert peak == pytest.approx(0.255, abs=0.01)
    assert peak < 0.01 * 48.8


def test_watchdog_must_judge_against_the_blended_reference():
    """守住 2026-08-03 第 7 次上机踩到的坑。

    交接偏置把参考量平移到实际位姿上，所以混合窗内对【真实轨迹】的误差按
    设计就等于偏置量（实测 36.3 mm）。若看门狗拿它去撞 50 mm 包线，正常工作
    会被误判成故障 —— 那次就是在 t=0.35 s 被打断的。看门狗必须看的是对
    【混合后参考量】的误差，那一项在交接瞬间恒为零。
    """
    blend = HandoverBlend(3.0)
    t0 = 0.0
    p_traj, R_traj, v_traj = _traj(t0)[0], _traj(t0)[1], _traj(t0)[2]
    p_act = p_traj + np.array([0.0363, 0.0, 0.0])       # 实测 36.3 mm
    blend.capture(t0, p_act, R_traj, p_traj, R_traj,
                  actual_linear_velocity=v_traj, reference_linear_velocity=v_traj)

    p_blend = _blend_at(blend, t0)[0]
    raw_error = np.linalg.norm(p_traj - p_act)
    tracking_error = np.linalg.norm(p_blend - p_act)

    assert raw_error == pytest.approx(0.0363, abs=1e-6)
    assert raw_error > 0.05 * 0.7, "用例应处在会误触发的量级附近"
    assert tracking_error < 1e-12, (
        "对混合后参考量的误差在交接瞬间必须恒为零 —— 这才是看门狗该看的量")
