"""Smith 预测（死区时间补偿）的行为契约。

这些测试钉住的是【安全性质与退化性质】，不是精度主张 —— 精度只能实测。
设计推导与全部数值结论见 task_space_smc/smith_predictor.py 模块 docstring。
"""

import numpy as np
import pytest

from task_space_smc.core.smith_predictor import (
    DEFAULT_MAX_POSITION_CORRECTION,
    DEFAULT_MAX_VELOCITY_CORRECTION,
    JointStatePredictor,
    SmithPredictorConfig,
)

N = 6
DT = 0.005


def _cfg(**kw):
    kw.setdefault("enabled", True)
    kw.setdefault("beta_velocity", 1.0)
    kw.setdefault("beta_position", 1.0)
    return SmithPredictorConfig(N, **kw)


def _feed(pred, accel, ticks, dt=DT):
    for _ in range(ticks):
        pred.push_command(np.full(N, accel), dt)


# ---------------------------------------------------------------------------
# 退化性质：不开 = 现状，逐位不变
# ---------------------------------------------------------------------------

def test_disabled_is_bitwise_passthrough_of_the_same_objects():
    """缺省关闭时必须【连对象都原样返回】。

    与 loop_delay / velocity_filter 同一约定：没写这段配置的基线逐字节不变，
    连一次 ndarray 拷贝的浮点噪声都不引入。
    """
    pred = JointStatePredictor(SmithPredictorConfig(N))
    q, qd = np.arange(N, dtype=float), np.full(N, 0.3)
    _feed(pred, 5.0, 10)
    q_hat, qd_hat = pred.predict(q, qd)
    assert q_hat is q and qd_hat is qd


def test_beta_zero_is_passthrough_even_when_enabled():
    """开关打开但 β 全零也必须直通 —— 分档上机的第 0 档就是这个状态。"""
    pred = JointStatePredictor(
        SmithPredictorConfig(N, enabled=True, beta_velocity=0.0,
                             beta_position=0.0))
    q, qd = np.zeros(N), np.zeros(N)
    _feed(pred, 9.0, 10)
    assert pred.predict(q, qd) == (q, qd)


def test_empty_buffer_leaves_only_the_measurement_based_dead_reckoning():
    """⚠️ 缓冲为空时【速度修正必须恰好为零】，位置只剩死推项 β_p·T_p·q̇_meas。

    死推那一项不依赖模型，只依赖测量速度，所以它在没有任何指令历史时仍然
    成立而且是【对的】—— 位置测量滞后 T_p，用测得的速度把它推回来是纯运动学。
    真正不能做的是"没有指令历史却给出速度修正"，那才是凭空造运动。

    代价要记在账上：静止时 q̇_meas 就是噪声底（腕部实测 0.041~0.044 rad/s），
    死推把它折进位置估计，幅度 T_p·0.044 = 4.4e-4 rad ≈ 1.2 个编码器计数。
    这是 beta_position 缺省取 0 的理由之一（另一个见模块 docstring 第 6 节）。
    """
    pred = JointStatePredictor(_cfg())
    q, qd = np.full(N, 0.1), np.full(N, 0.2)
    q_hat, qd_hat = pred.predict(q, qd)
    assert np.allclose(qd_hat, qd), "没有指令历史却动了速度 = 凭空造运动"
    assert np.allclose(q_hat, q + pred.config.position_horizon_sec * qd)


def test_empty_buffer_and_zero_velocity_is_exactly_passthrough():
    """静止起步：缓冲空 + 速度为零 -> 修正量逐位为零。"""
    pred = JointStatePredictor(_cfg())
    q, qd = np.full(N, 0.1), np.zeros(N)
    q_hat, qd_hat = pred.predict(q, qd)
    assert np.array_equal(q_hat, q) and np.array_equal(qd_hat, qd)


# ---------------------------------------------------------------------------
# 正确性：常加速度下预测量必须精确等于运动学解析解
# ---------------------------------------------------------------------------

def test_constant_acceleration_recovers_exact_kinematics():
    """恒定 a 作用满 horizon 时 Δq̇ = a·T_v、Δq = q̇̂·T_p + ½a·T_p²。"""
    a, T_v, T_p = 2.0, 0.025, 0.010
    pred = JointStatePredictor(
        _cfg(velocity_horizon_sec=T_v, position_horizon_sec=T_p,
             max_velocity_correction=10.0, max_position_correction=10.0))
    _feed(pred, a, 20)                       # 100 ms >> horizon
    q, qd = np.zeros(N), np.zeros(N)
    q_hat, qd_hat = pred.predict(q, qd)
    assert np.allclose(qd_hat, a * T_v)
    assert np.allclose(q_hat, (a * T_v) * T_p + 0.5 * a * T_p * T_p)


def test_horizon_window_is_measured_in_seconds_not_ticks():
    """⚠️ 丢拍必须按【秒】截断。

    控制回路实测丢拍 0.08~0.83%、最大间隙 15.4 ms。若按固定拍数回溯，
    丢拍时 horizon 会变成 25~75 ms 之间的随机值 —— 而延迟是物理量
    （传输 + 电机内部估计器），不随丢拍变化。
    """
    a, T_v = 2.0, 0.025
    ref = JointStatePredictor(
        _cfg(velocity_horizon_sec=T_v, position_horizon_sec=0.0,
             max_velocity_correction=10.0, max_position_correction=10.0))
    _feed(ref, a, 20, dt=DT)
    dropped = JointStatePredictor(
        _cfg(velocity_horizon_sec=T_v, position_horizon_sec=0.0,
             max_velocity_correction=10.0, max_position_correction=10.0))
    for _ in range(10):                      # 每拍 15.4 ms（实测最大间隙）
        dropped.push_command(np.full(N, a), 0.0154)
    z = np.zeros(N)
    assert np.allclose(ref.predict(z, z)[1], dropped.predict(z, z)[1]), (
        "丢拍时预测量变了 —— 说明回溯是按拍数而不是按秒截断的")


def test_partial_window_uses_only_what_is_available():
    """缓冲不足 horizon 时只积到已有的那一段，绝不外插补齐。"""
    a, T_v = 2.0, 0.025
    pred = JointStatePredictor(
        _cfg(velocity_horizon_sec=T_v, position_horizon_sec=0.0,
             max_velocity_correction=10.0, max_position_correction=10.0))
    _feed(pred, a, 2)                        # 只有 10 ms
    z = np.zeros(N)
    assert np.allclose(pred.predict(z, z)[1], a * 2 * DT)


def test_beta_scales_the_correction_linearly():
    """β 是【预测强度】，必须严格线性 —— 分档上机靠这条可预测性。"""
    a = 3.0
    out = {}
    for beta in (0.0, 0.3, 0.5, 1.0):
        p = JointStatePredictor(
            _cfg(beta_velocity=beta, beta_position=0.0,
                 max_velocity_correction=10.0))
        _feed(p, a, 20)
        out[beta] = p.predict(np.zeros(N), np.zeros(N))[1][0]
    full = out[1.0]
    for beta in (0.0, 0.3, 0.5):
        assert out[beta] == pytest.approx(beta * full, rel=1e-12)


def test_prediction_leads_a_lagged_measurement():
    """核心承诺：对一个滞后 T_v 的测量，预测量应当逼近真值。

    合成一段常加速运动，喂给预测器"5 拍之前"的速度，检查它推回当前值。
    """
    a, T_v = 1.5, 0.025
    pred = JointStatePredictor(
        _cfg(velocity_horizon_sec=T_v, position_horizon_sec=0.0,
             max_velocity_correction=10.0, max_position_correction=10.0))
    _feed(pred, a, 40)
    t_now = 40 * DT
    qd_true = a * t_now
    qd_lagged = a * (t_now - T_v)            # 电机自报速度：滞后 5 拍
    _, qd_hat = pred.predict(np.zeros(N), np.full(N, qd_lagged))
    assert qd_hat[0] == pytest.approx(qd_true, rel=1e-12)
    assert abs(qd_hat[0] - qd_true) < abs(qd_lagged - qd_true)


# ---------------------------------------------------------------------------
# 安全边界
# ---------------------------------------------------------------------------

def test_velocity_correction_is_clamped_per_joint():
    """限幅逐关节生效：大臂 0.20、腕部 0.50（J̄ 行范数差 2.8 倍）。"""
    pred = JointStatePredictor(
        _cfg(beta_position=0.0, fallback_after_clamped_ticks=0))
    _feed(pred, 100.0, 20)                   # 荒谬的大加速度
    _, qd_hat = pred.predict(np.zeros(N), np.zeros(N))
    assert np.allclose(qd_hat, DEFAULT_MAX_VELOCITY_CORRECTION)


def test_position_correction_is_clamped():
    pred = JointStatePredictor(_cfg(fallback_after_clamped_ticks=0))
    _feed(pred, 100.0, 20)
    q_hat, _ = pred.predict(np.zeros(N), np.zeros(N))
    assert np.allclose(np.abs(q_hat), DEFAULT_MAX_POSITION_CORRECTION)


def test_sustained_clamping_latches_the_fallback():
    """⚠️ 连续触顶 = 模型与实物系统性不一致 -> 锁存退回，且必须【锁存】。

    退回之后即使指令恢复正常也不能自己解除：单拍恢复很可能只是穿过零点，
    而此时已经证明模型不可信。恢复只能由显式 reset()（阶段切换）触发。
    """
    pred = JointStatePredictor(_cfg(fallback_after_clamped_ticks=10))
    z = np.zeros(N)
    for _ in range(9):
        _feed(pred, 100.0, 6)
        pred.predict(z, z)
        assert not pred.tripped
    _feed(pred, 100.0, 6)
    pred.predict(z, z)
    assert pred.tripped and not pred.active

    pred._buf.clear()
    _feed(pred, 0.01, 20)                    # 指令恢复正常
    q, qd = np.full(N, 0.4), np.full(N, 0.1)
    assert pred.predict(q, qd) == (q, qd), "退回必须锁存，不能自愈"
    pred.reset()
    assert pred.active and not pred.tripped


def test_isolated_clamp_ticks_do_not_trip():
    """单拍触顶（一次饱和、一次丢帧）不该退回，计数必须清零。"""
    pred = JointStatePredictor(_cfg(fallback_after_clamped_ticks=3))
    z = np.zeros(N)
    for _ in range(20):
        pred._buf.clear()
        _feed(pred, 100.0, 6)                # 触顶
        pred.predict(z, z)
        pred._buf.clear()
        _feed(pred, 0.001, 6)                # 正常
        pred.predict(z, z)
    assert not pred.tripped


def test_nonfinite_command_falls_back_to_measurement():
    pred = JointStatePredictor(_cfg())
    pred.push_command(np.full(N, np.nan), DT)
    q, qd = np.full(N, 0.2), np.full(N, 0.3)
    q_hat, qd_hat = pred.predict(q, qd)
    assert np.allclose(q_hat, q) and np.allclose(qd_hat, qd)


def test_nonfinite_feedback_is_passed_through_untouched():
    """反馈本身坏了不是预测器该修的 —— 必须原样透传给看门狗。"""
    pred = JointStatePredictor(_cfg())
    _feed(pred, 1.0, 20)
    q = np.full(N, np.nan)
    qd = np.zeros(N)
    assert pred.predict(q, qd) == (q, qd)


def test_nonfinite_dt_is_not_buffered():
    """时钟异常时宁可少补，也不能把坏 dt 乘进积分。"""
    pred = JointStatePredictor(
        _cfg(max_velocity_correction=10.0, max_position_correction=10.0))
    for bad in (np.nan, np.inf, 0.0, -DT):
        pred.push_command(np.full(N, 5.0), bad)
    z = np.zeros(N)
    assert np.allclose(pred.predict(z, z)[1], 0.0)


# ---------------------------------------------------------------------------
# 配置校验
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("kw", [
    dict(beta_velocity=1.2),                 # 过补偿：horizon 扫描两端都掉收益
    dict(beta_velocity=-0.1),
    dict(beta_position=np.nan),
    dict(velocity_horizon_sec=-0.001),
    dict(position_horizon_sec=np.inf),
    dict(max_velocity_correction=-1.0),
    dict(max_velocity_correction=[1.0, 2.0]),   # 长度不对
    dict(fallback_after_clamped_ticks=-1),
])
def test_config_rejects_bad_values(kw):
    with pytest.raises(ValueError):
        SmithPredictorConfig(N, enabled=True, **kw)


def test_config_active_requires_both_switch_and_beta():
    assert not SmithPredictorConfig(N).active
    assert not SmithPredictorConfig(N, enabled=True).active          # β 全零
    assert not SmithPredictorConfig(N, beta_velocity=1.0).active     # 开关未开
    assert SmithPredictorConfig(N, enabled=True, beta_velocity=0.3).active


def test_describe_mentions_the_operating_point():
    text = SmithPredictorConfig(N, enabled=True, beta_velocity=0.3).describe()
    assert "0.30" in text and "25.0 ms" in text
