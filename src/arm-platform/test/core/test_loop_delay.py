"""仿真回路延迟建模的回归测试。

守住的核心事实：
  * 全零时【逐位直通】—— 已有仿真基线（whole6 / hwtest_simbest）不受影响
  * 秒 -> 拍的换算正确（25 ms @ 200 Hz = 5 拍、10 ms = 2 拍，与实测一致）
  * 指令侧预热输出零、反馈侧预热保持最旧那份（发零 = 控制器以为在零位 = 奇异）
  * 位置路与速度路滞后【不同】—— 这正是分段模型存在的理由
  * 位置量化按实测步长生效，且裸差分噪声 = 实测的 0.0762 rad/s
  * 相位滞后与频率的关系正确（这是整个失稳解释的量化依据）
  * 配置只在非 hardware 模式生效 —— 实物自带延迟，叠加即双重计入
  * 新旧键同时存在时以分段模型为准并【打警告】，不相加
"""

import os
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

from task_space_smc.core.loop_delay import (  # noqa: E402
    CommandDelayBuffer, DelayPlan, FeedbackDelayLine, FeedbackDelayModel,
    LEGACY_KEY, SEGMENTED_KEYS, quantize, resolve_simulation_delay)

CONTROL_DT = 1.0 / 200.0
MEASURED_DELAY_SEC = 0.025      # 第 15 次上机直连实测（速度路总量）
MEASURED_DELAY_STEPS = 5
MEASURED_POSITION_DELAY_SEC = 0.010     # 力矩 -> 位置二阶导，r=0.434/0.450
MEASURED_POSITION_DELAY_STEPS = 2
MEASURED_VELOCITY_EXTRA_SEC = 0.015     # 速度比位置多滞后 3 拍
MEASURED_QUANTUM_RAD = 0.000381         # 16 bit / ±12.5 rad
MEASURED_DIFF_NOISE = 0.0762            # quantum / dt
OSCILLATION_HZ = 24.8                   # 实测极限环频率


def _config(name):
    from pathlib import Path
    import task_space_smc
    path = (Path(task_space_smc.__file__).resolve().parent
            / "config" / f"smc_params_{name}.yaml")
    if not path.exists():
        return None
    with open(path) as fh:
        return yaml.safe_load(fh) or {}


# ==========================================================================
# 旧的指令侧单块模型（保留做兼容）
# ==========================================================================

def test_zero_delay_is_bitwise_passthrough():
    b = CommandDelayBuffer(0)
    assert not b.enabled
    rng = np.random.default_rng(0)
    for _ in range(50):
        v = rng.normal(0, 1, 6)
        out = b(v)
        assert out is v, "零延迟必须原样返回同一个对象，连拷贝都不做"


def test_negative_delay_is_rejected():
    with pytest.raises(ValueError, match="delay_steps"):
        CommandDelayBuffer(-1)


def test_measured_delay_maps_to_five_steps():
    """25 ms @ 200 Hz = 5 拍，与 j4/j6 互相关实测的滞后一致。"""
    assert CommandDelayBuffer.steps_for(
        MEASURED_DELAY_SEC, CONTROL_DT) == MEASURED_DELAY_STEPS


@pytest.mark.parametrize("sec,dt,steps", [
    (0.0, 0.005, 0), (0.005, 0.005, 1), (0.0249, 0.005, 5),
    (0.025, 0.01, 3), (0.1, 0.005, 20)])
def test_steps_conversion_rounds(sec, dt, steps):
    assert CommandDelayBuffer.steps_for(sec, dt) == steps


@pytest.mark.parametrize("sec,dt", [(-0.001, 0.005), (0.025, 0.0), (0.025, -1.0)])
def test_invalid_conversion_is_rejected(sec, dt):
    with pytest.raises(ValueError):
        CommandDelayBuffer.steps_for(sec, dt)


def test_output_is_the_command_from_n_steps_ago():
    b = CommandDelayBuffer(5)
    seq = [np.full(6, float(i)) for i in range(20)]
    out = [b(x) for x in seq]
    # 前 5 拍缓冲未满 -> 零
    for i in range(5):
        np.testing.assert_array_equal(out[i], np.zeros(6))
    # 之后逐拍对应 5 拍之前
    for i in range(5, 20):
        np.testing.assert_array_equal(out[i], seq[i - 5])


def test_warmup_emits_zero_not_the_current_command():
    """缓冲未满时若返回当前指令，等于把延迟在起步阶段抹掉 —— 那正是要避免的。"""
    b = CommandDelayBuffer(3)
    first = b(np.full(6, 7.0))
    np.testing.assert_array_equal(first, np.zeros(6))
    assert not np.allclose(first, 7.0)


def test_buffer_does_not_alias_the_caller_array():
    """调用方常复用同一个数组，缓冲必须自己拷贝，否则整段延迟会被后续写入污染。"""
    b = CommandDelayBuffer(2)
    buf = np.zeros(6)
    for i in range(6):
        buf[:] = i                    # 原地改写，模拟调用方复用
        b(buf)
    out = b(buf)
    assert not np.allclose(out, buf), "缓冲里存的是引用，被调用方改写了"


def test_reset_clears_the_pipeline():
    b = CommandDelayBuffer(3)
    for i in range(10):
        b(np.full(6, float(i)))
    b.reset()
    np.testing.assert_array_equal(b(np.full(6, 99.0)), np.zeros(6))


# ==========================================================================
# 反馈侧延迟线 —— 与指令侧的区别只在预热，而那个区别是必须的
# ==========================================================================

def test_feedback_line_zero_delay_is_bitwise_passthrough():
    line = FeedbackDelayLine(0)
    assert not line.enabled
    v = np.arange(6, dtype=float)
    assert line(v) is v


def test_feedback_line_negative_delay_is_rejected():
    with pytest.raises(ValueError, match="delay_steps"):
        FeedbackDelayLine(-1)


def test_feedback_warmup_holds_the_oldest_sample_never_zero():
    """反馈侧预热【绝不能】发零。

    发零 = 控制器认为机械臂在零位，而零位雅可比 sigma_min=0.00001
    （cond 326263），当场解出奇异位形下的巨大加速度需求，起步即炸 ——
    那是建模事故，不是被建模的现象。
    """
    line = FeedbackDelayLine(3)
    q0 = np.full(6, 1.4677)                      # 起步位形，远离零位
    out = [line(q0 + 0.001 * i) for i in range(3)]
    for o in out:
        np.testing.assert_allclose(o, q0)
        assert not np.allclose(o, 0.0), "反馈预热发零会让控制器以为在零位"


def test_feedback_line_delivers_the_sample_from_n_steps_ago():
    line = FeedbackDelayLine(2)
    seq = [np.full(6, float(i)) for i in range(12)]
    out = [line(x) for x in seq]
    for i in range(2):
        np.testing.assert_array_equal(out[i], seq[0])   # 预热保持最旧
    for i in range(2, 12):
        np.testing.assert_array_equal(out[i], seq[i - 2])


def test_feedback_line_does_not_alias_the_caller_array():
    line = FeedbackDelayLine(2)
    buf = np.zeros(6)
    for i in range(6):
        buf[:] = i
        line(buf)
    out = line(buf)
    assert not np.allclose(out, buf), "延迟线里存的是引用，被调用方改写了"


def test_feedback_line_reset_restarts_warmup():
    line = FeedbackDelayLine(2)
    for i in range(8):
        line(np.full(6, float(i)))
    line.reset()
    np.testing.assert_array_equal(line(np.full(6, 99.0)), np.full(6, 99.0))


# ==========================================================================
# 位置量化
# ==========================================================================

def test_quantize_zero_quantum_is_bitwise_passthrough():
    v = np.arange(6, dtype=float)
    assert quantize(v, 0.0) is v
    assert quantize(v, -1.0) is v


def test_quantize_snaps_to_multiples_of_the_measured_quantum():
    q = np.array([0.0, 1.4677, -1.4086, 0.3306, 0.0009, 0.0021])
    out = quantize(q, MEASURED_QUANTUM_RAD)
    ratio = out / MEASURED_QUANTUM_RAD
    np.testing.assert_allclose(ratio, np.round(ratio), atol=1e-9)
    assert np.max(np.abs(out - q)) <= 0.5 * MEASURED_QUANTUM_RAD + 1e-15


def test_quantize_error_is_bounded_by_half_a_step():
    rng = np.random.default_rng(3)
    q = rng.uniform(-3.0, 3.0, 5000)
    err = np.abs(quantize(q, MEASURED_QUANTUM_RAD) - q)
    assert err.max() <= 0.5 * MEASURED_QUANTUM_RAD + 1e-12


def test_quantize_rounds_half_up_like_steps_for():
    """与 steps_for 用同一个舍入约定，免得一份代码里两种规则并存。"""
    np.testing.assert_allclose(quantize(np.array([0.5, 1.5, 2.5]), 1.0),
                               np.array([1.0, 2.0, 3.0]))


# ==========================================================================
# 分段模型 —— 位置路与速度路滞后不同，这是它存在的全部理由
# ==========================================================================

def test_all_zero_segmented_model_is_bitwise_passthrough():
    """没写这几个键的配置必须逐位不变 —— whole6 / hwtest_simbest 靠这条。"""
    m = FeedbackDelayModel(control_period=CONTROL_DT)
    assert not m.enabled
    q = np.arange(6, dtype=float)
    qd = np.ones(6)
    tau = np.full(6, 0.5)
    out_q, out_qd, out_tau = m(q, qd, tau)
    assert out_q is q and out_qd is qd and out_tau is tau


def test_quantization_alone_enables_the_model():
    m = FeedbackDelayModel(position_quantum_rad=MEASURED_QUANTUM_RAD,
                           control_period=CONTROL_DT)
    assert m.enabled
    assert m.position_delay_steps == 0 and m.velocity_delay_steps == 0


def test_measured_split_gives_two_and_five_steps():
    """实测 10 ms / 25 ms 在 200 Hz 下就是 2 拍与 5 拍。"""
    m = FeedbackDelayModel(
        transport_delay_sec=MEASURED_POSITION_DELAY_SEC,
        velocity_extra_delay_sec=MEASURED_VELOCITY_EXTRA_SEC,
        position_quantum_rad=MEASURED_QUANTUM_RAD,
        control_period=CONTROL_DT)
    assert m.position_delay_steps == MEASURED_POSITION_DELAY_STEPS
    assert m.velocity_delay_steps == MEASURED_DELAY_STEPS
    assert m.position_delay_sec == pytest.approx(MEASURED_POSITION_DELAY_SEC)
    assert m.velocity_delay_sec == pytest.approx(MEASURED_DELAY_SEC)


def test_velocity_path_lags_the_position_path_by_three_steps():
    """速度比位置多滞后 3 拍 = 15 ms —— 实测（同一份数据两种测法）。"""
    m = FeedbackDelayModel(
        transport_delay_sec=MEASURED_POSITION_DELAY_SEC,
        velocity_extra_delay_sec=MEASURED_VELOCITY_EXTRA_SEC,
        control_period=CONTROL_DT)
    assert m.velocity_delay_steps - m.position_delay_steps == 3


def test_the_two_paths_really_deliver_different_samples():
    """不是只有属性对得上 —— 走一遍数据，两路取到的样本必须真的不同。"""
    m = FeedbackDelayModel(
        transport_delay_sec=MEASURED_POSITION_DELAY_SEC,
        velocity_extra_delay_sec=MEASURED_VELOCITY_EXTRA_SEC,
        control_period=CONTROL_DT)
    seen_q, seen_qd = [], []
    for i in range(20):
        q, qd, _ = m(np.full(6, float(i)), np.full(6, 100.0 + i), None)
        seen_q.append(q[0])
        seen_qd.append(qd[0])
    # 稳态：位置滞后 2 拍、速度滞后 5 拍
    for i in range(6, 20):
        assert seen_q[i] == pytest.approx(i - 2)
        assert seen_qd[i] == pytest.approx(100.0 + i - 5)


def test_effort_shares_the_transport_delay_with_position():
    """位置与力矩在同一帧串口报文里回来，共享 transport 延迟。"""
    m = FeedbackDelayModel(
        transport_delay_sec=MEASURED_POSITION_DELAY_SEC,
        velocity_extra_delay_sec=MEASURED_VELOCITY_EXTRA_SEC,
        control_period=CONTROL_DT)
    for i in range(12):
        q, _, tau = m(np.full(6, float(i)), np.zeros(6), np.full(6, float(i)))
    np.testing.assert_allclose(q, tau)


def test_effort_none_passes_through():
    m = FeedbackDelayModel(transport_delay_sec=0.010,
                           control_period=CONTROL_DT)
    _, _, tau = m(np.zeros(6), np.zeros(6), None)
    assert tau is None


def test_position_is_quantized_before_the_controller_sees_it():
    m = FeedbackDelayModel(position_quantum_rad=MEASURED_QUANTUM_RAD,
                           control_period=CONTROL_DT)
    q_true = np.array([1.4677, -1.4086, 0.3306, 0.0009, 0.0021, 0.0379])
    q_seen, _, _ = m(q_true, np.zeros(6), None)
    assert not np.allclose(q_seen, q_true), "位置没被量化"
    ratio = q_seen / MEASURED_QUANTUM_RAD
    np.testing.assert_allclose(ratio, np.round(ratio), atol=1e-9)


def test_velocity_is_not_quantized():
    """量化的是位置编码器，速度是电机自报的，不共用这个步长。"""
    m = FeedbackDelayModel(position_quantum_rad=MEASURED_QUANTUM_RAD,
                           control_period=CONTROL_DT)
    qd_true = np.full(6, 0.1234567)
    _, qd_seen, _ = m(np.zeros(6), qd_true, None)
    np.testing.assert_allclose(qd_seen, qd_true)


def test_bare_differentiation_noise_matches_the_measurement():
    """裸差分噪声 0.0762 rad/s，比电机自报速度噪声底（0.041~0.044）还大 1.7 倍。

    这条是"必须建位置量化"的量化依据：不建的话仿真里裸差分完美无噪，会假赢。
    """
    m = FeedbackDelayModel(position_quantum_rad=MEASURED_QUANTUM_RAD,
                           control_period=CONTROL_DT)
    assert m.quantization_velocity_noise() == pytest.approx(
        MEASURED_DIFF_NOISE, abs=1e-4)
    assert m.quantization_velocity_noise() > 0.0440


def test_negative_quantum_is_rejected():
    with pytest.raises(ValueError, match="position_quantum_rad"):
        FeedbackDelayModel(position_quantum_rad=-1e-4,
                           control_period=CONTROL_DT)


def test_model_reset_restarts_all_three_lines():
    m = FeedbackDelayModel(transport_delay_sec=0.010,
                           velocity_extra_delay_sec=0.015,
                           control_period=CONTROL_DT)
    for i in range(20):
        m(np.full(6, float(i)), np.full(6, float(i)), None)
    m.reset()
    q, qd, _ = m(np.full(6, 99.0), np.full(6, 99.0), None)
    np.testing.assert_allclose(q, 99.0)
    np.testing.assert_allclose(qd, 99.0)


# --------------------------------------------------------------------------
# 相位滞后 —— 整个失稳解释的量化依据
# --------------------------------------------------------------------------

def test_phase_lag_reaches_180_deg_at_20hz():
    """T_d=25 ms 的相位反转频率是 1/(2·T_d) = 20 Hz。"""
    b = CommandDelayBuffer(MEASURED_DELAY_STEPS)
    assert b.phase_lag_deg(20.0, CONTROL_DT) == pytest.approx(180.0, abs=0.1)


def test_phase_lag_at_the_observed_limit_cycle():
    """实测极限环 24.8 Hz 处滞后 223° —— 早已越过 180°。"""
    b = CommandDelayBuffer(MEASURED_DELAY_STEPS)
    lag = b.phase_lag_deg(OSCILLATION_HZ, CONTROL_DT)
    assert lag == pytest.approx(223.2, abs=1.0)
    assert lag > 180.0, "极限环频率处必须已越过相位反转，否则解释不成立"


def test_phase_lag_is_linear_in_frequency():
    b = CommandDelayBuffer(MEASURED_DELAY_STEPS)
    assert b.phase_lag_deg(10.0, CONTROL_DT) == pytest.approx(
        0.5 * b.phase_lag_deg(20.0, CONTROL_DT))


def test_bandwidth_budget_implied_by_the_delay():
    """要留 ~60° 相位裕度，穿越频率必须压到 7 Hz 以下。

    这条把"降增益到什么程度"从拍脑袋变成有依据 —— 24.8 Hz 处仍不稳，
    说明当前回路在那里增益 >= 1，需要压低约 24.8/7 = 3.5 倍。
    """
    b = CommandDelayBuffer(MEASURED_DELAY_STEPS)
    assert b.phase_lag_deg(7.0, CONTROL_DT) == pytest.approx(63.0, abs=1.0)
    assert b.phase_lag_deg(7.0, CONTROL_DT) < 90.0


def test_segmented_model_reproduces_the_same_phase_budget():
    """分段模型的速度路必须给出与旧单块模型完全相同的 223°。"""
    m = FeedbackDelayModel(
        transport_delay_sec=MEASURED_POSITION_DELAY_SEC,
        velocity_extra_delay_sec=MEASURED_VELOCITY_EXTRA_SEC,
        control_period=CONTROL_DT)
    assert m.velocity_phase_lag_deg(OSCILLATION_HZ) == pytest.approx(223.2, abs=1.0)
    assert m.position_phase_lag_deg(OSCILLATION_HZ) == pytest.approx(89.3, abs=1.0)
    assert m.velocity_phase_lag_deg(20.0) == pytest.approx(180.0, abs=0.1)


@pytest.mark.parametrize("transport_sec,steps", [
    (0.008, 2), (0.010, 2), (0.015, 3)])
def test_uncertain_position_delay_sweep_stays_representable(transport_sec, steps):
    """位置那 10 ms 只有 r=0.43/0.45，没坐实 —— 8/10/15 ms 都要能扫。

    ⚠️ 8 ms 与 10 ms 在 200 Hz 下【同为 2 拍】，扫这两个点得不到不同的结果。
       真要分辨得提高控制频率或改用分数延迟。这条测试把这个限制钉住，
       免得有人扫完 8/10 得到"不敏感"的结论而其实根本没扫到东西。
    """
    m = FeedbackDelayModel(transport_delay_sec=transport_sec,
                           control_period=CONTROL_DT)
    assert m.position_delay_steps == steps


# ==========================================================================
# 配置解析与新旧键的优先级
# ==========================================================================

def test_empty_config_disables_everything():
    plan = resolve_simulation_delay(None, CONTROL_DT, "simulation")
    assert isinstance(plan, DelayPlan)
    assert not plan.enabled
    assert not plan.command_delay.enabled
    assert not plan.feedback.enabled
    assert plan.warnings == ()


def test_legacy_key_alone_still_uses_the_command_side_model():
    plan = resolve_simulation_delay(
        {LEGACY_KEY: MEASURED_DELAY_SEC}, CONTROL_DT, "simulation")
    assert plan.command_delay.delay_steps == MEASURED_DELAY_STEPS
    assert not plan.feedback.enabled
    assert any("指令侧" in w for w in plan.warnings)


def test_segmented_keys_use_the_feedback_model_and_disable_the_command_side():
    plan = resolve_simulation_delay(
        {"transport_delay_sec": 0.010,
         "velocity_extra_delay_sec": 0.015,
         "position_quantum_rad": MEASURED_QUANTUM_RAD},
        CONTROL_DT, "simulation")
    assert not plan.command_delay.enabled, "分段模型下指令侧必须关掉，否则重复计入"
    assert plan.feedback.velocity_delay_steps == MEASURED_DELAY_STEPS
    assert plan.feedback.position_delay_steps == MEASURED_POSITION_DELAY_STEPS
    assert plan.warnings == ()


def test_both_kinds_of_keys_prefer_segmented_and_warn():
    """同写两套几乎一定是改到一半忘了删；静默取其一会让人以为总滞后是两者之和。"""
    plan = resolve_simulation_delay(
        {LEGACY_KEY: 0.025,
         "transport_delay_sec": 0.010,
         "velocity_extra_delay_sec": 0.015},
        CONTROL_DT, "simulation")
    assert not plan.command_delay.enabled
    assert plan.feedback.velocity_delay_steps == MEASURED_DELAY_STEPS
    assert plan.warnings, "同时写两套键必须打警告"
    assert any(LEGACY_KEY in w for w in plan.warnings)


def test_zero_control_group_is_representable_and_disabled():
    """对照组：分段键显式写 0 -> 不失稳的那一次。"""
    plan = resolve_simulation_delay(
        {"transport_delay_sec": 0.0,
         "velocity_extra_delay_sec": 0.0,
         "position_quantum_rad": MEASURED_QUANTUM_RAD},
        CONTROL_DT, "simulation")
    assert plan.feedback.position_delay_steps == 0
    assert plan.feedback.velocity_delay_steps == 0
    assert plan.feedback.enabled, "量化仍开着 —— 对照组只去掉延迟，不去掉量化"


@pytest.mark.parametrize("cfg", [
    {LEGACY_KEY: 0.025},
    {"transport_delay_sec": 0.010, "velocity_extra_delay_sec": 0.015,
     "position_quantum_rad": 0.000381},
])
def test_hardware_mode_disables_every_form_of_injection(cfg):
    """实物自带这些延迟与量化，再叠一层就是双重计入。"""
    plan = resolve_simulation_delay(cfg, CONTROL_DT, "hardware")
    assert not plan.enabled
    assert not plan.command_delay.enabled
    assert not plan.feedback.enabled
    assert any("hardware" in m for m in plan.messages)


def test_internal_simulation_mode_is_injected_too():
    plan = resolve_simulation_delay(
        {"transport_delay_sec": 0.010}, CONTROL_DT, "internal_simulation")
    assert plan.feedback.enabled


# --------------------------------------------------------------------------
# 配置一致性
# --------------------------------------------------------------------------

def test_hwtest_models_the_measured_split():
    cfg = _config("hwtest")
    sim = cfg["simulation"]
    assert sim["transport_delay_sec"] == pytest.approx(
        MEASURED_POSITION_DELAY_SEC)
    assert sim["velocity_extra_delay_sec"] == pytest.approx(
        MEASURED_VELOCITY_EXTRA_SEC)
    assert sim["position_quantum_rad"] == pytest.approx(MEASURED_QUANTUM_RAD)
    plan = resolve_simulation_delay(sim, CONTROL_DT, "simulation")
    # 速度路总量必须落回实测的 25 ms / 5 拍
    assert plan.feedback.velocity_delay_sec == pytest.approx(MEASURED_DELAY_SEC)
    assert plan.feedback.velocity_delay_steps == MEASURED_DELAY_STEPS
    assert plan.feedback.position_delay_steps == MEASURED_POSITION_DELAY_STEPS


def test_hwtest_does_not_carry_both_generations_of_keys():
    """留着旧键会让每次启动都打警告 —— 也说明配置改到一半。"""
    sim = _config("hwtest")["simulation"]
    assert LEGACY_KEY not in sim


def test_verified_baselines_carry_no_delay_key_of_either_kind():
    """whole6 是已验证基线，加延迟或量化会改变它的既有结论。"""
    for name in ("whole6",):
        cfg = _config(name)
        if cfg is None:
            continue
        sim = cfg.get("simulation") or {}
        assert float(sim.get(LEGACY_KEY, 0.0)) == 0.0, f"{name} 不应带仿真延迟"
        for key in SEGMENTED_KEYS:
            assert key not in sim, f"{name} 不应带分段延迟键 {key}"
        plan = resolve_simulation_delay(sim, CONTROL_DT, "simulation")
        assert not plan.enabled, f"{name} 的延迟方案必须整体关闭"
