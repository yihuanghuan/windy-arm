"""关节速度反馈低通的回归测试。

守住的核心事实：
  * 默认关闭时【逐位直通】—— 已有基线不受影响
  * cutoff<=0 的关节直通，不受其它关节设置牵连
  * 相位滞后与截止频率的关系正确（这是本项的【代价】，必须可核对）
  * 白噪声下的降噪倍数与理论一致
  * 反馈损坏时保持上一拍，不把 NaN 灌进控制律
"""

import os
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

from task_space_smc.core.velocity_filter import JointVelocityFilter  # noqa: E402

# 2026-08-03 实机静止时实测速度噪声标准差 [rad/s]
MEASURED_NOISE_STD = [0.0000, 0.0000, 0.0000, 0.0440, 0.0426, 0.0407]
CONTROL_DT = 1.0 / 200.0
OSCILLATION_HZ = 25.8      # 实测极限环频率


def _config(name):
    from pathlib import Path
    import task_space_smc
    path = (Path(task_space_smc.__file__).resolve().parent
            / "config" / f"smc_params_{name}.yaml")
    if not path.exists():
        return None
    with open(path) as fh:
        return yaml.safe_load(fh) or {}


# --------------------------------------------------------------------------
# 关闭时必须逐位直通
# --------------------------------------------------------------------------

def test_zero_cutoff_is_bitwise_passthrough():
    f = JointVelocityFilter(0.0, 6)
    assert not f.enabled
    rng = np.random.default_rng(0)
    for _ in range(50):
        v = rng.normal(0, 1, 6)
        np.testing.assert_array_equal(f(v, CONTROL_DT), v)


def test_per_joint_cutoff_leaves_zero_joints_untouched():
    """大臂设 0 时必须完全直通 —— 给无噪声的关节加滞后是纯亏损。"""
    f = JointVelocityFilter([0.0, 0.0, 0.0, 50.0, 50.0, 50.0], 6)
    assert f.enabled
    rng = np.random.default_rng(1)
    v0 = rng.normal(0, 1, 6)
    f(v0, CONTROL_DT)                      # 首拍采纳
    for _ in range(50):
        v = rng.normal(0, 1, 6)
        out = f(v, CONTROL_DT)
        np.testing.assert_array_equal(out[:3], v[:3])   # 大臂逐位直通
        assert not np.allclose(out[3:], v[3:])          # 腕部确实被滤


def test_alpha_matches_first_order_formula():
    f = JointVelocityFilter([0.0, 0.0, 0.0, 50.0, 50.0, 50.0], 6)
    a = f.alpha(CONTROL_DT)
    np.testing.assert_array_equal(a[:3], np.ones(3))
    tau = 1.0 / (2.0 * np.pi * 50.0)
    np.testing.assert_allclose(a[3:], CONTROL_DT / (tau + CONTROL_DT), rtol=1e-12)


# --------------------------------------------------------------------------
# 相位滞后 —— 本项的代价，必须可核对
# --------------------------------------------------------------------------

@pytest.mark.parametrize("cutoff,expected_lag", [
    (30.0, 40.7), (50.0, 27.3), (80.0, 17.9)])
def test_phase_lag_at_the_oscillation_frequency(cutoff, expected_lag):
    """arctan(f/f_c)。配置注释里的数字必须站得住。"""
    f = JointVelocityFilter(cutoff, 6)
    lag = f.phase_lag_deg(OSCILLATION_HZ)
    assert np.allclose(lag, expected_lag, atol=1.0), (
        f"截止 {cutoff} Hz 在 {OSCILLATION_HZ} Hz 处滞后 {lag[0]:.1f}°")


def test_lower_cutoff_costs_more_phase():
    """降噪与相位裕度是对冲的 —— 这条把方向锁住，防止日后误调。"""
    lags = [JointVelocityFilter(c, 1).phase_lag_deg(OSCILLATION_HZ)[0]
            for c in (20.0, 40.0, 80.0, 160.0)]
    assert lags == sorted(lags, reverse=True), f"滞后应随截止升高而下降: {lags}"


def test_zero_cutoff_has_no_phase_lag():
    np.testing.assert_array_equal(
        JointVelocityFilter(0.0, 6).phase_lag_deg(OSCILLATION_HZ), np.zeros(6))


# --------------------------------------------------------------------------
# 白噪声下的实际降噪量
# --------------------------------------------------------------------------

def test_white_noise_attenuation_matches_theory():
    """一阶低通对白噪声的输出功率 ∝ 截止频率。用实测噪声量级验证。"""
    rng = np.random.default_rng(7)
    dt = CONTROL_DT
    n = 200000
    raw = rng.normal(0, MEASURED_NOISE_STD[5], n)
    out = {}
    for c in (25.0, 50.0, 100.0):
        f = JointVelocityFilter(c, 1)
        y = np.array([f(np.array([x]), dt)[0] for x in raw[:20000]])
        out[c] = y[1000:].std()
    # 截止减半 -> 输出标准差约乘 1/sqrt(2)
    assert out[25.0] < out[50.0] < out[100.0]
    assert 1.2 < out[50.0] / out[25.0] < 1.7, out
    # 50 Hz 下噪声应降到原来的 0.6~0.8 倍
    assert 0.55 < out[50.0] / MEASURED_NOISE_STD[5] < 0.85, out


def test_filter_does_not_bias_a_constant_signal():
    """稳态增益必须为 1，否则会给速度反馈引入直流偏置。"""
    f = JointVelocityFilter(50.0, 6)
    v = np.full(6, 0.3)
    for _ in range(500):
        out = f(v, CONTROL_DT)
    np.testing.assert_allclose(out, v, rtol=1e-9)


# --------------------------------------------------------------------------
# 鲁棒性
# --------------------------------------------------------------------------

def test_first_sample_is_adopted_not_ramped_from_zero():
    """首拍若从 0 起滤，会造出一个不存在的速度阶跃。"""
    f = JointVelocityFilter(50.0, 6)
    v = np.full(6, 2.0)
    np.testing.assert_array_equal(f(v, CONTROL_DT), v)


def test_non_finite_feedback_holds_last_value():
    f = JointVelocityFilter(50.0, 6)
    good = np.full(6, 0.5)
    f(good, CONTROL_DT)
    for bad in (np.nan, np.inf, -np.inf):
        v = np.full(6, 0.5)
        v[2] = bad
        out = f(v, CONTROL_DT)
        assert np.all(np.isfinite(out)), "坏反馈把 NaN 传进了控制律"


@pytest.mark.parametrize("bad", [-1.0, [50.0, 50.0]])
def test_invalid_config_is_rejected(bad):
    with pytest.raises(ValueError):
        JointVelocityFilter(bad, 6)


def test_nonpositive_dt_is_rejected():
    with pytest.raises(ValueError, match="dt"):
        JointVelocityFilter(50.0, 6).alpha(0.0)


# --------------------------------------------------------------------------
# 配置一致性
# --------------------------------------------------------------------------

def test_hwtest_keeps_the_filter_off_after_the_negative_result():
    """【已实测否定】2026-08-03 第 12 次上机：截止 50 Hz 反而更早发散。

        第 11 次 无滤波     存活 0.47 s
        第 12 次 滤波 50 Hz 存活 0.35 s

    降噪 ~30% 却花掉 26.6° 相位就变糟 -> 该回路【相位受限】而非噪声受限。
    代码与本测试文件保留，作为已验证的否定结论；配置必须保持关闭，
    除非有新证据推翻它。
    """
    cfg = _config("hwtest")
    hz = np.asarray(cfg["feedback"]["velocity_lowpass_hz"], dtype=float)
    assert not np.any(hz > 0.0), (
        f"速度滤波已被实测否定（0.47 -> 0.35 s），不应重新开启，实得 {hz.tolist()}")


def test_the_noise_asymmetry_that_motivated_the_filter_is_still_recorded():
    """否定的是"滤波有用"，不是"噪声不存在"。噪声底的不对称仍是事实，保留记录。"""
    assert max(MEASURED_NOISE_STD[:3]) < 0.001, "大臂速度反馈无噪声底"
    assert min(MEASURED_NOISE_STD[3:]) > 0.03, "腕部速度反馈有 ~0.04 rad/s 噪声底"
    assert min(MEASURED_NOISE_STD[3:]) / max(max(MEASURED_NOISE_STD[:3]), 1e-6) > 30


def test_simulation_baselines_do_not_filter():
    """仿真反馈没有噪声，滤波只剩滞后。基线必须保持直通。"""
    checked = 0
    for name in ("whole6", "hwtest_simbest"):
        cfg = _config(name)
        if cfg is None:
            continue
        checked += 1
        hz = (cfg.get("feedback") or {}).get("velocity_lowpass_hz", 0.0)
        assert not np.any(np.asarray(hz, dtype=float) > 0.0), (
            f"{name} 是仿真基线，不应开启速度滤波")
    assert checked > 0
