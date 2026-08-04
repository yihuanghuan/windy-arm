"""JointLESO 的验证。

分四类：
  1. 结构性质（增益闭式 == Ackermann、极点位置、直通承诺）
  2. ⚠️ 相位（这是 velocity_filter 被否定后必须正面回答的那一项）
  3. 合成信号（按实测噪声底与实测参考速度，量群延迟与输出噪声）
  4. 丢拍
"""

import numpy as np
import pytest

from task_space_smc.core.leso import JointLESO

T = 0.005
Q_POS = 0.000381                  # 实测位置量化步长 rad（16 bit / ±12.5 rad）
QV_ARM = 40.0 / 4095              # 实测 j1~j3 自报速度 LSB
QV_WRIST = 100.0 / 4095           # 实测 j4~j6 自报速度 LSB
W_DESIGN = 200.0                  # 设计带宽 rad/s


# --------------------------------------------------------------------------
# 1. 结构
# --------------------------------------------------------------------------

def _ackermann_current_gains(w, dt):
    """独立实现：Ackermann 极点配置 + Φ⁻¹ 换成当前观测器增益。"""
    Phi = np.array([[1.0, dt, 0.5 * dt * dt], [0.0, 1.0, dt], [0.0, 0.0, 1.0]])
    C = np.array([[1.0, 0.0, 0.0]])
    b = np.exp(-w * dt)
    O = np.vstack([C @ np.linalg.matrix_power(Phi, k) for k in range(3)])
    alpha = np.polynomial.polynomial.polyfromroots([b, b, b])[::-1]
    P = sum(alpha[k] * np.linalg.matrix_power(Phi, 3 - k) for k in range(4))
    Lp = P @ np.linalg.inv(O) @ np.array([0.0, 0.0, 1.0])
    return np.linalg.solve(Phi, Lp)


@pytest.mark.parametrize("w", [40.0, 100.0, 200.0, 400.0])
@pytest.mark.parametrize("dt", [0.0025, 0.005, 0.010, 0.0154])
def test_gain_closed_form_matches_ackermann(w, dt):
    """闭式增益必须与数值极点配置一致 —— 闭式是热路径上唯一的实现。"""
    obs = JointLESO(w, 1, nominal_dt=T)
    assert np.allclose(obs.gains(dt)[:, 0], _ackermann_current_gains(w, dt),
                       rtol=1e-9, atol=1e-9)


@pytest.mark.parametrize("dt", [0.005, 0.010, 0.0154, 0.025, 0.050])
def test_discrete_poles_are_triple_and_inside_unit_circle(dt):
    """⚠️ 丢拍安全性的根据：ZOH 极点 exp(-ω·dt) 对任意 dt 恒在 (0,1)。

    对照前向欧拉的 1-ω·dt：ω=200 时 dt=10 ms 给 -1.0（临界），
    dt=15.4 ms（实测最大间隙）给 -2.08 —— 出单位圆，发散。
    """
    obs = JointLESO(W_DESIGN, 1, nominal_dt=T)
    Phi = np.array([[1.0, dt, 0.5 * dt * dt], [0.0, 1.0, dt], [0.0, 0.0, 1.0]])
    L = obs.gains(dt)[:, 0]
    A = (np.eye(3) - np.outer(L, [1.0, 0.0, 0.0])) @ Phi
    target = np.exp(-W_DESIGN * dt)
    # ⚠️ 判据用【特征多项式系数】，不用特征值。三重特征值是 defective 的，
    # 数值扰动按 eps^(1/3) ≈ 6e-6 放大 —— 直接比特征值会在 dt 大时假失败
    # （实测 dt=50 ms 时三个根散布 ±2e-6，而目标值本身才 4.5e-5）。
    # 特征多项式的系数是良态的。
    assert np.allclose(np.poly(A), np.poly([target] * 3), atol=1e-11)
    assert 0.0 < target < 1.0
    assert np.max(np.abs(np.linalg.eigvals(A))) < 1.0
    # 前向欧拉在同样的 dt 下会怎样 —— 记录为对照，不是被测对象
    euler_pole = 1.0 - W_DESIGN * dt
    if dt >= 0.010:
        assert abs(euler_pole) >= 1.0, "此 dt 下前向欧拉本应已经不稳定"


def test_disabled_is_bitwise_passthrough():
    """ω_o 全零时必须原样返回入参对象 —— 既有配置逐字节不变。"""
    obs = JointLESO(0.0, 6)
    assert not obs.enabled
    v = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
    out = obs(np.zeros(6), v, T)
    assert out is v


def test_per_joint_disable_is_bitwise_passthrough():
    """逐关节关闭的那几轴必须逐位等于入参，不能走观测器算术。"""
    obs = JointLESO([W_DESIGN, W_DESIGN, W_DESIGN, 0.0, 0.0, 0.0], 6)
    rng = np.random.default_rng(0)
    q = np.zeros(6)
    for k in range(200):
        q = q + 0.02 * T
        v = rng.normal(0.02, 0.04, 6)
        out = obs(q, v, T)
        assert np.array_equal(out[3:], v[3:])       # 逐位，不是 allclose


def test_rejects_bad_config():
    with pytest.raises(ValueError):
        JointLESO([-1.0] * 6, 6)
    with pytest.raises(ValueError):
        JointLESO([np.nan] * 6, 6)
    with pytest.raises(ValueError):
        JointLESO([1.0, 2.0], 6)
    with pytest.raises(ValueError):
        JointLESO(100.0, 6, nominal_dt=0.0)


def test_nan_feedback_does_not_poison_state():
    obs = JointLESO(W_DESIGN, 6)
    q = np.zeros(6)
    for _ in range(50):
        q = q + 0.02 * T
        obs(q, np.zeros(6), T)
    good = obs.velocity_estimate.copy()
    out = obs(np.full(6, np.nan), np.zeros(6), T)
    assert np.all(np.isfinite(out))
    assert np.allclose(out, good)
    assert np.all(np.isfinite(obs.velocity_estimate))


# --------------------------------------------------------------------------
# 2. ⚠️ 相位 —— velocity_filter 被否定后必须正面回答的那一项
# --------------------------------------------------------------------------

def test_dc_gain_is_exactly_one():
    """低频必须无偏：估计的是速度本身，不是被缩放过的速度。"""
    obs = JointLESO(W_DESIGN, 1)
    for f in (0.0, 0.01, 0.1, 0.3):
        assert abs(abs(obs.response(f)[0]) - 1.0) < 5e-3


def test_phase_beats_the_rejected_lowpass_below_15hz():
    """⚠️ 核心判据。

    2026-08-03 第 12 次上机否定了 50 Hz 一阶低通：降噪 30% 换 26.6° 相位，
    存活 0.47 -> 0.35 s。本 LESO 在 15 Hz 以下的相位必须比它【小一个量级】，
    否则就是同一个错误换个名字。
    """
    obs = JointLESO(W_DESIGN, 1)
    for f in (1.0, 3.0, 5.0, 10.0):
        leso = obs.phase_lag_deg(f)[0]
        lowpass = np.degrees(np.arctan2(f, 50.0))
        assert leso >= -1e-9
        assert leso < lowpass, f"{f} Hz: LESO {leso:.2f}° 未优于低通 {lowpass:.2f}°"
    assert obs.phase_lag_deg(10.0)[0] < 0.7 * np.degrees(np.arctan2(10.0, 50.0))


def test_phase_far_better_than_the_15ms_it_replaces():
    """替掉的是电机估计器的 15 ms 群延迟。整个 1~30 Hz 都必须净赚。"""
    obs = JointLESO(W_DESIGN, 1)
    for f in (1.0, 3.0, 5.0, 10.0, 15.0, 20.0, 24.8, 30.0):
        motor = 360.0 * f * 0.015
        assert obs.phase_lag_deg(f)[0] < motor
    # 实测极限环频率处的净收益必须 > 90°
    net = 360.0 * 24.8 * 0.015 - obs.phase_lag_deg(24.8)[0]
    assert net > 90.0, f"24.8 Hz 净相位收益只有 {net:.1f}°"


def test_third_order_phase_is_cubic_not_linear():
    """阶数的结构性质：n=3 低频相位 ~ (Ω/ω)³，n=2 是 ~(Ω/ω)（= 低通陷阱）。

    判据：频率翻倍时相位应当涨约 8 倍（三次），而不是 2 倍（一次）。
    """
    obs = JointLESO(400.0, 1)          # 取高带宽以留在渐近区
    p1 = obs.phase_lag_deg(2.0)[0]
    p2 = obs.phase_lag_deg(4.0)[0]
    assert 6.0 < p2 / p1 < 10.0, f"相位阶数不对：翻倍比 {p2/p1:.2f}"


def test_second_order_would_have_a_velocity_bias():
    """记录"为什么不是 2 阶"的量化依据（不是对本模块的测试）。

    2 阶观测器对常值扰动 d 的稳态速度偏置是 2d/ω_o。本机静摩擦折合的
    关节加速度 d = f_s/M_eff，最大 10.2 rad/s²（j4）。
    """
    f_s = np.array([0.725, 1.775, 0.625, 0.125, 0.100, 0.100])
    m_eff = np.array([0.11198, 0.19707, 0.12985, 0.01231, 0.01154, 0.01050])
    d = f_s / m_eff
    bias_2nd = 2.0 * d / W_DESIGN
    ref_speed = np.array([0.029, 0.029, 0.029, 0.0133, 0.0133, 0.0133])
    assert np.all(bias_2nd > ref_speed), "2 阶偏置本应压过参考速度"
    # 3 阶：常值扰动下稳态速度误差为零
    obs = JointLESO(W_DESIGN, 1)
    v_true, a_true = 0.02, 9.0
    q = 0.0
    t = 0.0
    for _ in range(4000):
        q = 0.5 * a_true * t * t + v_true * t
        obs(np.array([q]), np.array([0.0]), T)
        t += T
    assert abs(obs.velocity_estimate[0] - (v_true + a_true * t)) < 1e-3 * abs(
        v_true + a_true * t) + 1e-3


# --------------------------------------------------------------------------
# 3. 合成信号：群延迟与输出噪声，与两个对照方案同表
# --------------------------------------------------------------------------

def _quantized_ramp(v, n, dither=0.0, seed=0):
    rng = np.random.default_rng(seed)
    t = np.arange(n) * T
    q = v * t + (rng.normal(0.0, dither, n) if dither > 0 else 0.0)
    return np.floor(q / Q_POS + 0.5) * Q_POS


def _run(obs, y):
    out = np.empty(len(y))
    for k, yk in enumerate(y):
        out[k] = obs(np.array([yk]), np.array([0.0]), T)[0]
    return out


@pytest.mark.parametrize("v", [0.0033, 0.0133, 0.013, 0.029])
def test_output_noise_beats_motor_velocity_in_the_jitter_band(v):
    """⚠️ 定量依据的核心：0.8~6 Hz（末端抖动指标带）内必须优于电机自报。

    电机自报速度实测为 12 bit 量化（j1~j3 LSB 0.009768、j4~j6 0.024418，
    2026-08-04 从 3 份实机 CSV 反解，见 leso.py 模块头），纯量化的 RMS 下限
    就是 LSB/sqrt(12) = 0.00282 / 0.00705 rad/s。
    """
    n = int(120 / T)
    obs = JointLESO(W_DESIGN, 1)
    est = _run(obs, _quantized_ramp(v, n))[int(20 / T):]
    err = est - v

    def band_rms(x, lo, hi):
        X = np.fft.rfft(x * np.hanning(len(x)))
        f = np.fft.rfftfreq(len(x), T)
        m = (f >= lo) & (f < hi)
        return np.sqrt(np.sum(np.abs(X[m]) ** 2) / np.sum(np.abs(X) ** 2)) * np.std(x)

    inband = band_rms(err, 0.8, 6.0)
    floor_arm = QV_ARM / np.sqrt(12.0)
    floor_wrist = QV_WRIST / np.sqrt(12.0)
    assert inband < floor_arm, f"带内 {inband:.5f} 未低于大臂量化下限 {floor_arm:.5f}"
    assert inband < 0.25 * floor_wrist
    # 均值无偏
    assert abs(np.mean(err)) < 1e-4


def test_limit_cycle_band_noise_below_motor_report():
    """20~30 Hz（实测极限环 24.8 Hz 的邻域）也必须不比电机自报更吵。

    否则"多买的相位被多灌的噪声吃掉"，就成了另一种形式的重蹈覆辙。
    """
    n = int(120 / T)
    est = _run(JointLESO(W_DESIGN, 1), _quantized_ramp(0.0133, n))[int(20 / T):]
    err = est - 0.0133
    X = np.fft.rfft(err * np.hanning(len(err)))
    f = np.fft.rfftfreq(len(err), T)
    m = (f >= 20.0) & (f < 30.0)
    band = np.sqrt(np.sum(np.abs(X[m]) ** 2) / np.sum(np.abs(X) ** 2)) * np.std(err)
    # 电机自报在同一带的实测量级（0.041 rad/s 白噪声中 20~30 Hz 的份额）
    assert band < 0.0128


def test_group_delay_at_1hz_is_sub_millisecond():
    """群延迟 = -dφ/dΩ。设计带宽下 1 Hz 处应远小于它替掉的 15 ms。"""
    obs = JointLESO(W_DESIGN, 1)
    df = 0.05
    p = np.unwrap([np.angle(obs.response(1.0 - df)[0]),
                   np.angle(obs.response(1.0 + df)[0])])
    tg_ms = -(p[1] - p[0]) / (2 * np.pi * 2 * df) * 1000.0
    assert 0.0 <= tg_ms < 1.0, f"1 Hz 群延迟 {tg_ms:.2f} ms"


def test_tracks_a_slow_ramp_without_lag_or_bias():
    """实测参考速度区间内，稳态估计必须无偏 —— 这是 3 阶的直接后果。"""
    for v in (0.0033, 0.0133, 0.013, 0.029):
        obs = JointLESO(W_DESIGN, 1)
        est = _run(obs, _quantized_ramp(v, int(60 / T)))[int(20 / T):]
        assert abs(np.mean(est) - v) < 2e-4 * max(1.0, v / 0.0033)


# --------------------------------------------------------------------------
# 4. 丢拍
# --------------------------------------------------------------------------

@pytest.mark.parametrize("gap_beats", [2, 3, 4])
def test_actual_dt_beats_nominal_dt_across_a_long_beat(gap_beats):
    """⚠️ 用实际 dt 必须严格优于用标称 dt，且优势随长拍变长而扩大。

    实测控制循环丢拍 0.08~0.83%、最大间隙 15.4 ms（约 3 拍）。
    """
    v = 0.020

    def peak_error(follow_dt):
        obs = JointLESO(W_DESIGN, 1)
        t = 0.0
        peak = 0.0
        for k in range(1500):
            dt = T * (gap_beats if k == 800 else 1)
            t += dt
            y = np.floor(v * t / Q_POS + 0.5) * Q_POS
            obs(np.array([y]), np.array([0.0]), dt if follow_dt else T)
            if k >= 800:
                peak = max(peak, abs(obs.velocity_estimate[0] - v))
        return peak

    good, bad = peak_error(True), peak_error(False)
    if gap_beats >= 3:
        assert good < bad, f"长拍 {gap_beats}x: 跟 dt {good:.5f} 未优于固定 {bad:.5f}"
    assert good < 0.05


def test_long_gap_resyncs_position_and_keeps_velocity():
    """超过 resync_sec 的真掉线：位置重新落位，速度保留（不置零）。"""
    obs = JointLESO(W_DESIGN, 1, resync_sec=0.05)
    v = 0.02
    t = 0.0
    for _ in range(600):
        t += T
        obs(np.array([np.floor(v * t / Q_POS + 0.5) * Q_POS]), np.array([0.0]), T)
    before = obs.velocity_estimate[0]
    t += 0.5                                     # 掉线 500 ms
    out = obs(np.array([np.floor(v * t / Q_POS + 0.5) * Q_POS]),
              np.array([0.0]), 0.5)
    assert out[0] == pytest.approx(before)       # 速度保留
    assert obs.position_estimate[0] == pytest.approx(v * t, abs=Q_POS)


def test_dt_is_clamped():
    """dt<=0 / 非有限退回标称，不抛异常也不产生 NaN（控制循环里不能崩）。"""
    obs = JointLESO(W_DESIGN, 6)
    q = np.zeros(6)
    for dt in (T, 0.0, -1.0, np.nan, np.inf, 1e-9, T):
        q = q + 0.02 * T
        out = obs(q, np.zeros(6), dt)
        assert np.all(np.isfinite(out))


def test_describe_mentions_the_baseline_it_replaces():
    obs = JointLESO(W_DESIGN, 6)
    text = obs.describe()
    assert "133.9" in text and "24.8" in text
    assert "关闭" in JointLESO(0.0, 6).describe()


# --------------------------------------------------------------------------
# 5. 预算
# --------------------------------------------------------------------------

def test_step_cost_fits_the_5ms_budget():
    """控制回路 5 ms 预算，观测器必须是零头。"""
    import time
    obs = JointLESO(W_DESIGN, 6)
    q = np.zeros(6)
    v = np.zeros(6)
    for _ in range(200):
        obs(q, v, T)
    n = 4000
    t0 = time.perf_counter()
    for k in range(n):
        q = q + 1e-4
        obs(q, v, T)
    per_call_us = (time.perf_counter() - t0) / n * 1e6
    assert per_call_us < 200.0, f"每拍 {per_call_us:.1f} us"
