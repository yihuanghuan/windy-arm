"""JointStateEstimator 的验证。

分七类：
  1. ⚠️【最重要】缺省逐位不变 —— 用【对象恒等 is】而不是数值相等
  2. LESO 的相位/群延迟/噪声，把设计里算出来的数钉死
  3. 丢拍（dt 变化）
  4. Smith 预测的模型误差偏差上界与退回机制
  5. 五条路线 A~E 的配置解析
  6. 反馈损坏、限幅、计数等安全边界
  7. 已验证基线不含 estimator 键 + hwtest 增益冻结

⚠️ 本文件里所有"合成信号"的结论都只用于验证【实现是否符合设计】，
   不用于支持任何"这个参数在实物上更好"的主张 —— 那必须实测。
"""

import os

import numpy as np
import pytest
import yaml

from task_space_smc.core.leso import JointLESO
from task_space_smc.core.smith_predictor import SmithPredictorConfig, JointStatePredictor
from task_space_smc.core.state_estimator import (
    MODEL_B_MIN_BANDWIDTH_HZ,
    JointStateEstimator,
    resolve_estimator,
)

T = 0.005                       # 控制周期 200 Hz
N = 6
Q_POS = 0.000381                # 实测位置量化步长 rad（16 bit / ±12.5 rad）
MAX_GAP = 0.0154                # 实测控制循环最大间隙 15.4 ms
W_DESIGN = 200.0                # 设计带宽 rad/s = 31.83 Hz
HZ_DESIGN = W_DESIGN / (2 * np.pi)
F_LIMIT_CYCLE = 24.8            # 实物极限环频率 Hz（没有任何模型能解释它）

CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "task_space_smc", "config")


def _config(name):
    with open(os.path.join(CONFIG_DIR, f"smc_params_{name}.yaml"),
              "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _est(cfg, n=N, period=T):
    return resolve_estimator(cfg, n, period).estimator


# ==========================================================================
# 1. ⚠️ 缺省逐位不变
#
# 判据是【对象恒等】而不是数值相等。走一遍 `x + 0.0` 的算术在 −0.0 上就会留痕
# （−0.0 + 0.0 == +0.0），对"逐位不变"这个承诺不干净。与 FeedbackDelayModel /
# JointVelocityFilter / JointLESO 已经建立的约定完全一致。
# ==========================================================================

def test_absent_section_is_object_identity():
    """不写 estimator 段 -> __call__ 原样返回传进来的那两个对象。"""
    est = _est(None)
    assert not est.enabled
    rng = np.random.default_rng(0)
    q, qd = rng.normal(size=N), rng.normal(size=N)
    q2, qd2 = est(q, qd, T)
    assert q2 is q and qd2 is qd          # is，不是 ==


def test_explicit_motor_source_is_object_identity():
    """路线 A 写成显式配置，与"删掉这一段"【逐位相同】。

    这条很有价值：路线 A 不是"重测一遍基线"，它在代码层面【就是】基线。
    所以五条路线在实验记录里对称可比，而不会因为写出来就变成另一次测量。
    """
    est = _est({"source": "motor"})
    assert not est.enabled
    q, qd = np.arange(N, dtype=float), np.arange(N, dtype=float) * 0.1
    q2, qd2 = est(q, qd, T)
    assert q2 is q and qd2 is qd


def test_disabled_path_preserves_negative_zero():
    """−0.0 必须原样出来。这正是"数值相等"验不出来而"对象恒等"能验出来的那一位。"""
    est = _est({"source": "motor", "predict_velocity_sec": 0.0})
    q = np.array([-0.0, 0.0, -0.0, 1.0, -1.0, 0.0])
    qd = np.array([-0.0] * N)
    q2, qd2 = est(q, qd, T)
    assert q2 is q and qd2 is qd
    assert np.signbit(q2[0]) and np.signbit(qd2[0])


def test_disabled_estimator_never_advances_state():
    """禁用时连计数器都不动 —— 说明整条路在函数第一行就返回了。"""
    est = _est({})
    for _ in range(50):
        est(np.zeros(N), np.zeros(N), T)
    assert est.updates == 0 and est.hold_count == 0
    assert est.predict_clamp_hits == 0


def test_zero_bandwidth_leso_is_disabled():
    """source: leso 但 bandwidth_hz=0 -> 实际直通，且会告警（不是静默生效）。"""
    plan = resolve_estimator({"source": "leso"}, N, T)
    assert not plan.enabled
    assert any("bandwidth_hz 是 0" in w for w in plan.warnings)


def test_push_command_is_free_when_not_needed():
    """缺省配置下 wants_commanded_acceleration 为 False，runner 可整段跳过。"""
    est = _est({"source": "finite_difference"})
    assert not est.wants_commanded_acceleration
    est.push_command(np.ones(N), T)           # 不该有任何副作用
    assert len(est._cmd_buf) == 0


# ==========================================================================
# 2. LESO 的相位 / 群延迟 / 噪声
#
# ⚠️ 这一组是 velocity_filter 被实测否定（2026-08-03 第 12 次上机：截止 50 Hz，
#    降噪 ~30% 却花掉 26.6° 相位，存活 0.47 -> 0.35 s，更早发散）之后必须正面
#    回答的那一项：你的相位从哪来、跟谁比。
# ==========================================================================

MOTOR_EXTRA_LAG_SEC = 0.015     # 电机内部速度估计器的群延迟（25 ms − 10 ms）


def _lag_deg(delay_sec, f):
    return 360.0 * f * delay_sec


def test_phase_budget_at_limit_cycle_frequency():
    """24.8 Hz 处三条路的相位账，钉住设计里的数。

    电机自报那多出来的 15 ms   133.9°   <- 要被换掉的
    位置裸后向差分（半拍）      22.3°   = 180·f·T
    LESO ω_o=200 rad/s          36.5°
    ——— 被否定的 50 Hz 低通     26.6°   但它是【叠加】上去的 => 净 −26.4°

    关键是【叠加 vs 替代】：低通把 26.6° 加在 133.9° 上，纯亏；
    裸差分与 LESO 把那 133.9° 整个换掉，净赚 111.6° / 97.4°。
    """
    motor = _lag_deg(MOTOR_EXTRA_LAG_SEC, F_LIMIT_CYCLE)
    assert motor == pytest.approx(133.92, abs=0.05)

    fd = _lag_deg(0.5 * T, F_LIMIT_CYCLE)      # 后向差分 = 半拍群延迟
    assert fd == pytest.approx(22.32, abs=0.05)

    leso = JointLESO(W_DESIGN, 1, nominal_dt=T).phase_lag_deg(F_LIMIT_CYCLE)[0]
    assert leso == pytest.approx(36.45, abs=0.10)

    # 净收益（替代，不是叠加）
    assert motor - fd == pytest.approx(111.60, abs=0.15)
    assert motor - leso == pytest.approx(97.47, abs=0.15)
    # ⚠️ 诚实地记下来：15 Hz 以上 LESO 的相位比那个被否定的低通【还大】
    # （36.45° vs 26.38°）。孤立比这两个数没有意义 —— 它们在回路里位置不同。
    assert leso > 26.38


def test_leso_group_delay_is_negligible_in_jitter_band():
    """末端抖动指标带 0.8~6 Hz 里，LESO 的群延迟应当可忽略。

    这是三阶（而不是二阶）带来的结构性质：低频相位是【三次】的
    −8·(Ω/ω_o)³，前两阶被分子零点 3ω_o 抵消掉。二阶的 H=ω²/(s+ω)² 低频相位是
    【一次】的 −2(Ω/ω) —— 那就是 velocity_filter 那个低通换个名字。
    """
    leso = JointLESO(W_DESIGN, 1, nominal_dt=T)
    for f, bound_ms in ((0.8, 0.02), (1.0, 0.05), (6.0, 1.0)):
        gd_ms = 1000.0 * leso.phase_lag_deg(f)[0] / (360.0 * f)
        assert 0.0 <= gd_ms < bound_ms, f"{f} Hz 群延迟 {gd_ms:.4f} ms"


def test_leso_low_frequency_phase_is_cubic_not_linear():
    """频率翻倍相位涨 ~8 倍（三次），不是 ~2 倍（一次 = 低通）。"""
    leso = JointLESO(W_DESIGN, 1, nominal_dt=T)
    for f in (0.5, 1.0, 2.0):
        ratio = leso.phase_lag_deg(2 * f)[0] / leso.phase_lag_deg(f)[0]
        assert ratio == pytest.approx(8.0, rel=0.10), f"{f} Hz -> {ratio}"


def _quantized_ramp_noise(omega_o, v_ref, n=30000, seed=None, warmup=3000):
    """在【量化位置的匀速斜坡】上量速度估计的输出噪声。

    合成条件取自实测：位置量化 0.000381 rad、200 Hz、参考速度取实测的腕部
    0.0133 rad/s（该关节参考速度低于自身电机噪声底 3~10 倍，是本方案最想改善
    的那一路）。
    """
    t = np.arange(n) * T
    q_true = v_ref * t
    q = np.floor(q_true / Q_POS + 0.5) * Q_POS
    leso = JointLESO(omega_o, 1, nominal_dt=T)
    out = np.empty(n)
    for k in range(n):
        out[k] = leso(q[k:k + 1], np.zeros(1), T)[0]
    fd = np.diff(q, prepend=q[0]) / T
    return out[warmup:], fd[warmup:]


@pytest.mark.parametrize("hz, expected_std", [
    (12.0, 0.0066),
    (15.0, 0.0094),
    (HZ_DESIGN, 0.0239),        # ω_o = 200 rad/s
    (40.0, 0.0290),             # 到这里与裸差分持平
])
def test_leso_noise_versus_bandwidth(hz, expected_std):
    """噪声随带宽的实测表。⚠️ 这张表是整条路线 C 的成本侧。

    读法：LESO 要在噪声上赢裸差分（0.0289）需要 ω_o ≲ 40 Hz；而带宽压低到
    赢很多（12 Hz 处 0.0066，4 倍）时，它在 24.8 Hz 要付的相位从 36.5° 涨到
    近百度，而裸差分只付 22.3°。**无模型输入时这两头没法同时要** —— 这正是
    路线 C 与路线 B 之间那个未解决的分歧，必须由实物裁决。
    """
    out, fd = _quantized_ramp_noise(2 * np.pi * hz, 0.0133)
    assert np.std(fd) == pytest.approx(0.0289, abs=0.0015)
    assert np.std(out) == pytest.approx(expected_std, abs=0.0015)


def test_leso_has_no_steady_state_velocity_bias_on_ramp():
    """三阶对【常值】扰动的稳态速度误差恒为 0 —— 这是不能用二阶的理由。

    二阶的稳态速度偏置是 2d/ω_o。本机静摩擦折合关节加速度 d = f_s/M_eff =
    4.8~10.2 rad/s²，ω_o=200 时偏置 0.048~0.10 rad/s —— 是参考速度
    （大臂 0.029、腕部 0.0133）的 3~30 倍，信号会被直接压垮。
    """
    out, _ = _quantized_ramp_noise(W_DESIGN, 0.0133)
    assert np.mean(out) == pytest.approx(0.0133, abs=2e-4)


def test_leso_with_model_input_is_near_zero_phase_when_model_is_true():
    """b0·u 通路：plant 恰为 q̈ = b0·u 时，速度估计对真速度全频段近似零相位。

    这是路线 C 全部价值的来源（观测器增益只作用在【模型残差】上）。
    ⚠️ 同时也说明它的脆弱之处：模型输入不准时，残差通路的相位就是全部相位，
    LESO 退化成纯微分器 —— 而第 16 次实测显示本机六轴里四轴的指令力矩低于
    静摩擦（真实 q̈≈0 而 b0·u≈1.87 rad/s²），模型输入很可能就是不准的。
    """
    n = 4000
    f = 2.0
    w = 2 * np.pi * f
    t = np.arange(n) * T
    amp = 0.02
    q = amp * np.sin(w * t)
    v = amp * w * np.cos(w * t)
    a = -amp * w * w * np.sin(w * t)

    with_u = JointLESO(W_DESIGN, 1, nominal_dt=T, b0=1.0)
    without_u = JointLESO(W_DESIGN, 1, nominal_dt=T)
    e_with = np.empty(n)
    e_without = np.empty(n)
    for k in range(n):
        e_with[k] = with_u(q[k:k + 1], np.zeros(1), T, u=a[k:k + 1])[0]
        e_without[k] = without_u(q[k:k + 1], np.zeros(1), T)[0]
    tail = slice(2000, None)
    err_with = np.max(np.abs(e_with[tail] - v[tail]))
    err_without = np.max(np.abs(e_without[tail] - v[tail]))
    # 实测约 22 倍（9.8e-5 对 2.2e-3）。阈值取 10 倍，留出实现细节的余量。
    assert err_with < 0.10 * err_without, (err_with, err_without)


def test_model_input_path_is_bit_identical_when_b0_is_zero():
    """b0=0 或不传 u -> 与加这条通路之前逐位相同。"""
    rng = np.random.default_rng(3)
    q = np.cumsum(rng.normal(scale=1e-3, size=(500, 2)), axis=0)
    a = rng.normal(size=(500, 2))
    ref = JointLESO(W_DESIGN, 2, nominal_dt=T)
    b0_zero = JointLESO(W_DESIGN, 2, nominal_dt=T, b0=0.0)
    no_u = JointLESO(W_DESIGN, 2, nominal_dt=T, b0=1.0)
    for k in range(500):
        r = ref(q[k], np.zeros(2), T)
        z = b0_zero(q[k], np.zeros(2), T, u=a[k])       # b0=0 -> u 无效
        n_ = no_u(q[k], np.zeros(2), T)                 # b0=1 但不传 u
        assert np.array_equal(r, z)
        assert np.array_equal(r, n_)


# ==========================================================================
# 3. 丢拍（dt 变化）
#
# 实测控制循环丢拍 0.08~0.83%、p99 周期 5.2 ms、最大间隙 15.4 ms。
# ==========================================================================

def test_zoh_pole_survives_measured_max_gap_but_forward_euler_does_not():
    """离散化选型的判据。这是"会当场炸设备"那一类。

        ω_o=200          dt=5ms   dt=10ms   dt=15.4ms(实测最大间隙)
        前向欧拉 1−ω_o·dt  0.000   −1.000    −2.080   <- 出单位圆，一次丢拍打飞
        ZOH exp(−ω_o·dt)   0.368    0.135     0.046   <- 恒在 (0,1)
    """
    leso = JointLESO(W_DESIGN, 1, nominal_dt=T)
    for dt, zoh in ((T, 0.36788), (0.010, 0.13534), (MAX_GAP, 0.04601)):
        assert leso.discrete_pole(dt)[0] == pytest.approx(zoh, abs=1e-4)
        assert abs(leso.discrete_pole(dt)[0]) < 1.0
    euler = 1.0 - W_DESIGN * MAX_GAP
    assert euler == pytest.approx(-2.08, abs=0.01)
    assert abs(euler) > 1.0     # 正是不用前向欧拉（也就不需要子步）的理由


def test_actual_dt_beats_nominal_dt_on_a_long_tick():
    """跟 dt 变 vs 固定标称 dt。固定 dt 的误差随长拍长度线性增长。

    对照设计里的合成实测（真速度 0.020 rad/s）：
        长拍     跟 dt 变   固定标称 dt
        2×5ms    0.0193     0.0192
        3×5ms    0.0192     0.0289
        4×5ms    0.0192     0.0313
    """
    v_ref = 0.020
    for mult in (3, 4):
        errs = {}
        for follow in (True, False):
            leso = JointLESO(W_DESIGN, 1, nominal_dt=T)
            t = 0.0
            peak = 0.0
            for k in range(400):
                dt = mult * T if k == 300 else T
                t += dt
                q = np.array([v_ref * t])
                out = leso(q, np.zeros(1), dt if follow else T)[0]
                if k >= 300:
                    peak = max(peak, abs(out - v_ref))
            errs[follow] = peak
        assert errs[True] < 0.35 * errs[False], (mult, errs)


def test_finite_difference_uses_actual_dt():
    """裸差分必须用实际 dt。丢拍时用标称周期会把速度算大 3 倍。"""
    est = _est({"source": "finite_difference"})
    q0 = np.zeros(N)
    est(q0, np.zeros(N), T)                       # 首拍：建立 prev_q
    q1 = np.full(N, 3e-4)
    _, qd = est(q1, np.zeros(N), 3 * T)           # 一次 3 倍长拍
    assert np.allclose(qd, 3e-4 / (3 * T))


def test_finite_difference_first_tick_falls_back_to_motor():
    """首帧没有上一份位置 -> 保留电机自报值，不是 0。

    从 0 起步会在第一拍造成一次假的减速冲击，与 JointVelocityFilter 的
    _primed 是同一个坑。
    """
    est = _est({"source": "finite_difference"})
    qd_motor = np.full(N, 0.017)
    _, qd = est(np.zeros(N), qd_motor, T)
    assert np.allclose(qd, qd_motor)


def test_estimator_survives_a_stream_of_dropped_ticks():
    """随机丢拍（含 15.4 ms 最大间隙）下输出必须保持有限且有界。"""
    rng = np.random.default_rng(11)
    est = _est({
        "source": "leso",
        "predict_velocity_sec": 0.025,
        "predict_source": "acceleration_estimate",
        "leso": {"bandwidth_hz": HZ_DESIGN},
    })
    q = np.zeros(N)
    v_ref = 0.02
    for k in range(3000):
        dt = MAX_GAP if rng.random() < 0.01 else T
        q = q + v_ref * dt
        qm = np.floor(q / Q_POS + 0.5) * Q_POS
        out_q, out_qd = est(qm, np.zeros(N), dt)
        assert np.all(np.isfinite(out_q)) and np.all(np.isfinite(out_qd))
        if k > 500:
            assert np.max(np.abs(out_qd)) < 1.0    # 看门狗包线的零头


def test_long_gap_beyond_resync_keeps_velocity_state():
    """超过 resync_sec 视为真掉线：z1 重新落位到测量位置，但【保留】z2/z3。

    本机运动很慢（0.003~0.03 rad/s），掉线前的速度仍是比 0 好的猜测；
    置 0 会造成一次假的减速冲击。
    """
    leso = JointLESO(W_DESIGN, 1, nominal_dt=T, resync_sec=0.05)
    v_ref = 0.02
    t = 0.0
    for _ in range(600):
        t += T
        leso(np.array([v_ref * t]), np.zeros(1), T)
    before = leso.velocity_estimate[0]
    t += 1.0
    after = leso(np.array([v_ref * t]), np.zeros(1), 1.0)[0]
    assert after == pytest.approx(before)


# ==========================================================================
# 4. Smith 预测：模型误差下的偏差上界与退回机制
# ==========================================================================

def _predictor(**kw):
    cfg = dict(source="motor", predict_velocity_sec=0.025,
               predict_position_sec=0.010)
    cfg.update(kw)
    return _est(cfg)


def test_prediction_bias_is_bounded_by_half_a_err_t_squared():
    """本形式（测量锚定的有限时域外推）的偏差上界是 ½·a_err·T²，且每拍清零。

    ⚠️ 这正是不能用【经典 Smith】（控制器外挂无延迟并联模型）的理由：计算力矩
    抵消之后被控对象是纯双积分器，经典 Smith 在积分型对象上并联模型开环无阻尼，
    直流建模偏差会让它无界漂移。同样条件下经典 Smith 1 秒漂 ½·0.411·1² = 206 mm，
    本形式在 25 ms 上界只有 0.128 mm。
    """
    a_err = 0.411 / 0.2         # 实测等效任务扰动折合到关节的量级 rad/s²
    horizon = 0.025
    # 只留位置路（beta_velocity=0），把速度测量给对 —— 这样位置预测里唯一的
    # 误差来源就是那条假加速度，正好隔离出 ½·a_err·T² 这一项。
    est = _est({"source": "motor",
                "predict_position_sec": horizon,
                "predict_velocity_sec": 0.0,
                "predict_clamp_rad": 1.0})
    v_true = 0.02
    t = 0.0
    bias = None
    for k in range(2000):
        t += T
        q = np.full(N, v_true * t)
        out_q, _ = est(q, np.full(N, v_true), T)
        est.push_command(np.full(N, a_err), T)     # 真实加速度是 0 -> 全是误差
        if k < 10:
            continue                               # 等 horizon 缓冲填满
        # 真实的 T_p 之后位置是 q + v·T_p（真实加速度为 0）
        bias = np.max(np.abs(out_q - (q + v_true * horizon)))
        # 上界 ½·a_err·T²，且【不随 k 增长】—— 每拍从最新测量重新起算。
        assert bias <= 0.5 * a_err * horizon ** 2 * 1.02 + 1e-12, (k, bias)
    assert bias == pytest.approx(0.5 * a_err * horizon ** 2, rel=0.02)
    # 折成末端（特征长度 0.2 m）就是设计里那个 0.128 mm；对照经典 Smith 在
    # 同样条件下 1 秒漂 ½·0.411·1² = 206 mm。
    assert 0.5 * 0.411 * horizon ** 2 * 1000.0 == pytest.approx(0.128, abs=0.001)


def test_prediction_bias_does_not_accumulate_over_time():
    """每拍从最新测量重新起算 -> 偏差不随时间累积（经典 Smith 会）。"""
    est = _predictor(predict_clamp_rad=1.0, predict_clamp_rad_s=10.0)
    q = np.zeros(N)
    early = late = None
    for k in range(6000):
        out_q, _ = est(q, np.zeros(N), T)
        est.push_command(np.full(N, 2.0), T)
        if k == 1000:
            early = np.max(np.abs(out_q - q))
        if k == 5999:
            late = np.max(np.abs(out_q - q))
    assert late == pytest.approx(early, rel=1e-9)


def test_zero_horizon_is_object_identity():
    """β/horizon 为 0 时【逐位退回当前控制器】—— 经典 Smith 没有这个性质，
    做不出安全分档。"""
    est = _est({"source": "motor", "predict_velocity_sec": 0.0,
                "predict_position_sec": 0.0})
    q, qd = np.full(N, 0.3), np.full(N, 0.02)
    q2, qd2 = est(q, qd, T)
    assert q2 is q and qd2 is qd


def test_beta_scales_the_correction_linearly():
    """β 是【同一 horizon 上的预测强度】，horizon 是实测的物理量。

    两个旋钮分工明确：horizon 写实测的 T_v/T_p 不动，分档上机只动 β
    （0.3 -> 0.5 -> 0.7）。horizon 当调参旋钮会让两者互相掩盖。
    """
    corr = {}
    for beta in (0.3, 0.5, 1.0):
        est = _predictor(predict_position_sec=0.0, beta_velocity=beta,
                         predict_clamp_rad_s=10.0)
        for _ in range(30):
            _, out_qd = est(np.zeros(N), np.zeros(N), T)
            est.push_command(np.full(N, 1.0), T)
        corr[beta] = out_qd[0]
    assert corr[0.5] / corr[1.0] == pytest.approx(0.5, rel=1e-9)
    assert corr[0.3] / corr[1.0] == pytest.approx(0.3, rel=1e-9)


def test_horizon_is_measured_in_seconds_not_ticks():
    """⚠️ 按【秒】回溯而不是按拍数。

    实测丢拍 0.08~0.83%、最大间隙 15.4 ms；按拍数会把 25 ms 的 horizon 变成
    25~75 ms 之间的随机值，而延迟是物理量（传输 + 电机内部估计器），单位是秒。
    """
    est = _predictor(predict_position_sec=0.0, predict_clamp_rad_s=10.0)
    # 全部用 3 倍长拍推进；25 ms 的 horizon 只能覆盖 1.67 个这样的拍
    for _ in range(30):
        _, out_qd = est(np.zeros(N), np.zeros(N), 3 * T)
        est.push_command(np.full(N, 1.0), 3 * T)
    assert out_qd[0] == pytest.approx(0.025, rel=1e-9)   # = a·T_v，与拍长无关


def test_clamp_then_latched_fallback_and_manual_reset():
    """连续触顶 -> 锁存退回（逐位直通），且【不自愈】，需显式 reset()。

    单拍触顶可能只是一次饱和或一次穿零；连续 50 ms 触顶意味着模型与实物系统性
    不一致，此时预测量已不可信。
    """
    est = _predictor(predict_position_sec=0.0,
                     predict_clamp_rad_s=0.01, predict_fallback_ticks=10)
    q, qd = np.full(N, 0.1), np.full(N, 0.0)
    tripped_at = None
    for k in range(40):
        out_q, out_qd = est(q, qd, T)
        est.push_command(np.full(N, 50.0), T)      # 巨大的假指令 -> 必触顶
        if est.predictor.tripped and tripped_at is None:
            tripped_at = k
    assert tripped_at is not None and tripped_at <= 12
    # 触发那一拍预测器报的是 tripped 而不是 clamped，所以计数是 ticks−1。
    assert est.predict_clamp_hits >= 9
    # 退回之后逐位直通
    out_q, out_qd = est(q, qd, T)
    assert out_q is q and out_qd is qd
    est.reset()
    assert not est.predictor.tripped


def test_predict_source_acceleration_estimate_needs_no_push_command():
    """路线 D：影子积分器由 LESO 的 z3 驱动，不依赖 push_command。

    b0=0 时 z3 估的是【总】加速度（含指令的效果），所以它【替代】而不是叠加
    commanded_acceleration，不会双重计入。
    """
    est = _est({
        "source": "leso",
        "predict_velocity_sec": 0.010,
        "predict_source": "acceleration_estimate",
        "predict_clamp_rad_s": 10.0,
        "leso": {"bandwidth_hz": HZ_DESIGN},
    })
    # 匀加速真实运动：z3 应当收敛到该加速度，速度预测量 ≈ a·T
    a_true = 0.5
    t = 0.0
    for _ in range(1500):
        t += T
        q = np.full(N, 0.5 * a_true * t * t)
        out_q, out_qd = est(q, np.zeros(N), T)
    assert est.acceleration_estimate[0] == pytest.approx(a_true, rel=0.05)
    assert out_qd[0] - a_true * t == pytest.approx(a_true * 0.010, abs=0.01)


def test_non_leso_joints_get_no_acceleration_estimate():
    """没有 LESO 的关节写 NaN，不是 0 —— 写 0 会被读成"扰动为零"。"""
    est = _est({
        "source": ["leso", "leso", "leso", "motor", "motor", "motor"],
        "leso": {"bandwidth_hz": HZ_DESIGN},
    })
    est(np.zeros(N), np.zeros(N), T)
    acc = est.acceleration_estimate
    assert np.all(np.isfinite(acc[:3]))
    assert np.all(np.isnan(acc[3:]))


# ==========================================================================
# 5. 五条路线 A~E 的配置解析
# ==========================================================================

ROUTES = {
    # 路线: (配置, 期望 source, 期望 (predict_pos, predict_vel), 期望 ω_o)
    "A": ({"source": "motor"},
          ("motor",) * 6, (0.0, 0.0), 0.0),
    "B": ({"source": "finite_difference"},
          ("finite_difference",) * 6, (0.0, 0.0), 0.0),
    "B'": ({"source": ["motor", "motor", "motor",
                       "finite_difference", "finite_difference",
                       "finite_difference"]},
           ("motor", "motor", "motor", "finite_difference",
            "finite_difference", "finite_difference"), (0.0, 0.0), 0.0),
    "C": ({"source": "leso", "leso": {"bandwidth_hz": HZ_DESIGN,
                                      "input": "none"}},
          ("leso",) * 6, (0.0, 0.0), W_DESIGN),
    "D": ({"source": "leso",
           "predict_position_sec": 0.010, "predict_velocity_sec": 0.010,
           "predict_source": "acceleration_estimate",
           "leso": {"bandwidth_hz": HZ_DESIGN,
                    "input": "commanded_acceleration",
                    "b0": 1.0, "input_delay_sec": 0.010}},
          ("leso",) * 6, (0.010, 0.010), W_DESIGN),
    "E": ({"source": "motor",
           "predict_position_sec": 0.0, "predict_velocity_sec": 0.025,
           "predict_source": "commanded_acceleration"},
          ("motor",) * 6, (0.0, 0.025), 0.0),
}


@pytest.mark.parametrize("route", sorted(ROUTES))
def test_route_configs_parse_to_the_intended_estimator(route):
    cfg, want_source, (want_p, want_v), want_omega = ROUTES[route]
    plan = resolve_estimator(cfg, N, T)
    est = plan.estimator
    assert est.velocity_source == want_source
    assert est.predictor.config.position_horizon_sec == pytest.approx(want_p)
    assert est.predictor.config.velocity_horizon_sec == pytest.approx(want_v)
    assert np.allclose(est.leso.omega_o, want_omega)
    # 只有 A 是禁用（= 基线），其余四条都必须真的改变行为
    assert est.enabled == (route != "A")


def test_route_a_is_bit_identical_to_no_section():
    """路线 A 与"不写 estimator 段"是【同一件事】。"""
    a = _est(ROUTES["A"][0])
    none = _est(None)
    q, qd = np.full(N, 0.7), np.full(N, -0.03)
    assert a(q, qd, T)[0] is none(q, qd, T)[0] is q


def test_only_model_input_and_commanded_prediction_need_push_command():
    """哪些路线要求 runner 每拍算一次有效指令加速度（约 5.5 µs）。"""
    wants = {r: _est(c).wants_commanded_acceleration
             for r, (c, *_rest) in ROUTES.items()}
    assert wants == {"A": False, "B": False, "B'": False,
                     "C": False, "D": True, "E": True}


def test_hwtest_config_estimator_section_resolves_to_baseline():
    """就地改 smc_params_hwtest.yaml 的 estimator 段，当前应当是路线 A。"""
    cfg = _config("hwtest")
    assert "estimator" in cfg, "hwtest 应当显式写出路线 A，便于实验记录对称可比"
    est = _est(cfg["estimator"])
    assert not est.enabled
    q, qd = np.full(N, 0.1), np.full(N, 0.2)
    assert est(q, qd, T) == (q, qd)


# ==========================================================================
# 6. 安全边界：反馈损坏、非法配置、开销
# ==========================================================================

def test_nonfinite_feedback_holds_last_output_and_counts():
    """反馈损坏时保持上一拍并计数，绝不把 NaN 灌进控制律或观测器状态。"""
    est = _est({"source": "leso", "leso": {"bandwidth_hz": HZ_DESIGN}})
    good_q = np.full(N, 0.1)
    for _ in range(10):
        last_q, last_qd = est(good_q, np.zeros(N), T)
    bad = good_q.copy()
    bad[2] = np.nan
    q, qd = est(bad, np.zeros(N), T)
    assert np.array_equal(q, last_q) and np.array_equal(qd, last_qd)
    assert est.hold_count == 1
    # 观测器状态没被污染：下一拍正常反馈回来，输出仍然有限
    q, qd = est(good_q, np.zeros(N), T)
    assert np.all(np.isfinite(q)) and np.all(np.isfinite(qd))


def test_nonfinite_command_never_enters_the_buffer():
    est = _predictor(predict_clamp_rad_s=10.0)
    est.push_command(np.full(N, np.nan), T)
    est.push_command(np.full(N, 1.0), -1.0)         # 非法 dt
    assert len(est.predictor._buf) == 0


@pytest.mark.parametrize("cfg", [
    {"source": "motor_reported"},                     # 拼错
    {"source": ["motor"] * 5},                        # 长度不对
    {"position_source": "z1"},
    {"predict_source": "torque"},
    {"leso": {"input": "measured_effort"}},           # 未实现
    {"leso": {"bandwidth_hz": -1.0}},
])
def test_invalid_config_raises_instead_of_silently_falling_back(cfg):
    """非法配置必须当场抛。静默回落会让整次上机的记录无法解释，而实物时段
    是串行的稀缺资源。"""
    with pytest.raises(ValueError):
        resolve_estimator(cfg, N, T)


def test_unknown_key_warns_but_does_not_crash():
    plan = resolve_estimator(
        {"source": "finite_difference", "leso_bandwidth_hz": 15.0}, N, T)
    assert any("无法识别的键" in w for w in plan.warnings)


def test_low_bandwidth_warns_about_model_b_divergence():
    """⚠️ 方向是反的：直觉上"低带宽更平滑更安全"，在这里不成立。

    模型 B(T_v=T_p=10.5 ms, g=2.2) 下 ω_o=100 rad/s 给出 σ=+1.56 —— 发散。
    """
    assert MODEL_B_MIN_BANDWIDTH_HZ == pytest.approx(19.10, abs=0.02)
    plan = resolve_estimator(
        {"source": "leso", "leso": {"bandwidth_hz": 15.0}}, N, T)
    assert any("低于" in w and "发散" in w for w in plan.warnings)
    plan_ok = resolve_estimator(
        {"source": "leso", "leso": {"bandwidth_hz": HZ_DESIGN}}, N, T)
    assert not any("发散" in w for w in plan_ok.warnings)


def test_smith_prediction_warns_about_the_frozen_gain_trap():
    """⚠️ c1 不变就开预测会让准静态误差从 2.10 mm 变成 3.45 mm（差 64%）。

    这条必须在启动日志里喊出来，否则"只开预测的对照组"会被读成"Smith 没用"。
    """
    plan = resolve_estimator({"source": "motor",
                              "predict_velocity_sec": 0.025}, N, T)
    joined = "".join(plan.warnings)
    assert "3.453" in joined and "对照组" in joined


def test_leso_input_none_warns_that_route_c_may_be_dominated_by_route_b():
    plan = resolve_estimator(
        {"source": "leso", "leso": {"bandwidth_hz": HZ_DESIGN}}, N, T)
    assert any("裸差分" in w for w in plan.warnings)


def test_per_tick_cost_stays_well_under_the_5ms_budget():
    """开销实测。控制回路是 Python @200 Hz、5 ms 预算。

    ⚠️ 最贵的路线 D 实测 80~130 µs/拍，比设计阶段估的 6 µs 贵一个数量级 ——
    不是算法贵，是 numpy 在 6 元素小数组上的 Python 调用开销（profile 显示没有
    单一热点）。它占预算 1.6~3.4%，与 runner 里既有的 compute_kinematics
    （8.44 µs × 2）、lstsq 6×6（8.41 µs × 2）是同一量级里偏大的一项，
    **不是可忽略的**，首次上机要连丢拍率一起看。

    这条测试的用途是防"有人把 pinocchio 调用塞进估计器"（那会是毫秒量级），
    不是精确计时 —— 本机整夜在跑实验，测量噪声很大，所以取 min-of-3
    并把阈值放到 0.5 ms（= 预算的 10%，实测的 4~6 倍）。
    """
    import time
    est = _est({
        "source": "leso",
        "predict_velocity_sec": 0.025, "predict_position_sec": 0.010,
        "predict_source": "acceleration_estimate",
        "leso": {"bandwidth_hz": HZ_DESIGN},
    })
    q = np.zeros(N)
    zero = np.zeros(N)
    for _ in range(500):                      # 预热
        est(q, zero, T)

    def one_pass(n=4000):
        t0 = time.perf_counter()
        for _ in range(n):
            est(q, zero, T)
        return 1000.0 * (time.perf_counter() - t0) / n

    per_tick_ms = min(one_pass() for _ in range(3))
    assert per_tick_ms < 0.5, f"{per_tick_ms:.4f} ms/拍"


def test_buffers_are_bounded_so_cost_does_not_grow():
    """长跑时缓冲必须有界 —— 无界增长会让开销随时间上涨，而实物跑的是分钟级。"""
    est = _est({"source": "leso", "predict_velocity_sec": 0.025,
                "leso": {"bandwidth_hz": HZ_DESIGN,
                         "input": "commanded_acceleration",
                         "input_delay_sec": 0.010}})
    q = np.zeros(N)
    for k in range(20000):
        q = q + 1e-5
        est(q, np.zeros(N), T)
        est.push_command(np.full(N, 0.5), T)
    assert len(est.predictor._buf) <= 16
    assert len(est._cmd_buf) <= 16


# ==========================================================================
# 7. 已验证基线不能被动到
# ==========================================================================

@pytest.mark.parametrize("name", ["whole6", "hwtest_simbest"])
def test_verified_baselines_declare_no_estimator(name):
    """⚠️ 这条才是"已有基线一个字都不能改"的真正保证：把"基线文件里没有这一段"
    变成 CI 断言。

    ⚠️ 只查【顶层】estimator 键 —— whole6 里另有一个 simulation_injection
    下的 base_estimator，与本模块无关，不能误伤。
    """
    cfg = _config(name)
    assert "estimator" not in cfg
    est = _est(cfg.get("estimator"))
    assert not est.enabled
    q, qd = np.full(6, 0.4), np.full(6, 0.05)
    assert est(q, qd, T) == (q, qd)


def test_hwtest_gains_are_frozen():
    """把 2026-08-04 那 15 组实测出来的最优点钉死在 CI 里。

    交接后位置 RMS 2.95~3.02 mm、末端 0.8~6 Hz 抖动 0.856~0.908 mm。
    ⚠️ 这是【实测的硬件天花板附近】：Λ=21(c1=36) 与 Λ=25(c1=35.5) 都当场触发
    看门狗，而解析判据对这两组给出 σ=−11.8"很安全"。动它就丢掉基线。
    """
    p = _config("hwtest")["position"]
    assert p["lambda"] == [18.0] * 3
    assert p["k"] == [8.0] * 3
    assert p["eta"] == [0.08] * 3
    assert p["phi"] == [0.02] * 3
    o = _config("hwtest")["orientation"]
    assert o["lambda"] == [3.6] * 3
    assert o["k"] == [6.0] * 3
    assert o["eta"] == [0.018] * 3
    assert o["phi"] == [0.02] * 3


def _load_runner_module():
    import importlib.util
    import os
    if importlib.util.find_spec("rclpy") is None:
        pytest.skip("没有 ROS 环境")
    root = os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))
    spec = importlib.util.spec_from_file_location(
        "_runner_alias_check",
        os.path.join(root, "task_space_smc", "hw", "whole_task_smc_ros_runner.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_runner_keeps_q_ctrl_aliased_to_q_when_everything_is_off():
    """⚠️ 接线层的逐位不变证据。

    改动前 ``_refresh_controller_feedback`` 的承诺是"模型关闭时
    ``q_ctrl is self.q``"，控制律、看门狗、CSV 全都建立在这条别名关系上
    （runner 里多处注释明确写了"两者是同一个数组，逐位等价于改动前"）。
    估计器接进去之后这条关系必须**原样保留** —— 单测估计器本身是不够的，
    真正会出问题的是接线那一层。这里直接调 runner 的未绑定方法，喂一个
    最小假 self，不需要起 ROS。
    """
    import types
    from task_space_smc.core.loop_delay import FeedbackDelayModel

    module = _load_runner_module()
    q = np.full(N, 0.3)
    qd = np.full(N, 0.02)
    eff = np.full(N, 0.1)
    fake = types.SimpleNamespace(
        q=q, qd=qd, feedback_effort=eff,
        feedback_delay=FeedbackDelayModel(control_period=T),   # 全零 = 直通
        estimator=_est(None),                                  # 缺省 = 直通
        simulated_base=True, control_period=T,
        _estimator_last_time=None)
    module.WholeTaskSmcRosRunner._refresh_controller_feedback(fake, now=1.0)
    assert fake.q_ctrl is q
    assert fake.qd_ctrl is qd
    assert fake.effort_ctrl is eff
    # CSV 的入口列也必须指向同一份，否则"入口 == 出口"这条自检就不成立
    assert fake._q_meas is q and fake._qd_meas is qd


def test_runner_estimator_sits_downstream_of_the_delay_model():
    """⚠️ 顺序必须是 feedback_delay -> estimator，不能反过来。

    反过来的话仿真里估计器会看到无延迟、无量化的 plant 真值，与实物不同构
    —— 等于让仿真替方案作弊。这与当初加位置量化要防的是同一个错误。
    判据：开着量化的延迟模型时，估计器【入口】的位置必须落在量化网格上。
    """
    import types
    from task_space_smc.core.loop_delay import FeedbackDelayModel

    module = _load_runner_module()
    fake = types.SimpleNamespace(
        q=np.full(N, 0.30001234), qd=np.full(N, 0.02),
        feedback_effort=np.zeros(N),
        feedback_delay=FeedbackDelayModel(
            transport_delay_sec=0.010, velocity_extra_delay_sec=0.015,
            position_quantum_rad=Q_POS, control_period=T),
        estimator=_est({"source": "finite_difference"}),
        simulated_base=True, control_period=T,
        _estimator_last_time=None)
    fake._estimator_dt = types.MethodType(
        module.WholeTaskSmcRosRunner._estimator_dt, fake)
    module.WholeTaskSmcRosRunner._refresh_controller_feedback(fake, now=1.0)
    on_grid = np.abs(fake._q_meas / Q_POS - np.round(fake._q_meas / Q_POS))
    assert np.all(on_grid < 1e-9), fake._q_meas
    assert np.all(np.isfinite(fake.q_ctrl)) and np.all(np.isfinite(fake.qd_ctrl))


def test_fill_estimator_row_covers_exactly_the_declared_columns():
    """CSV 新增列必须与 _fill_estimator_row 写的键【完全一致】。

    守的是一个具体的失败模式：DictWriter 对多写的键会抛，但对【漏写】的键
    只是留空 —— 而 plot_whole_task_smc._load_csv 对每一列都做 float(row[key])，
    空字符串会在事后分析时才炸，那时实物时段已经用掉了。
    """
    import importlib.util
    import os
    import types

    if importlib.util.find_spec("rclpy") is None:
        pytest.skip("没有 ROS 环境")
    root = os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))
    spec = importlib.util.spec_from_file_location(
        "_runner_row_check",
        os.path.join(root, "task_space_smc", "hw", "whole_task_smc_ros_runner.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    fake = types.SimpleNamespace(
        n_arm=N, estimator=_est(None),
        _q_meas=np.zeros(N), _qd_meas=np.zeros(N))
    row = {}
    module.WholeTaskSmcRosRunner._fill_estimator_row(fake, row)
    expected = {"estimator_predict_clamp_hits", "estimator_hold_count"}
    for i in range(1, N + 1):
        expected |= {f"joint_{i}_q_meas_rad", f"joint_{i}_qd_meas_rad_s",
                     f"joint_{i}_est_acc_rad_s2"}
    assert set(row) == expected
    assert len(row) == 20            # 18 逐关节 + 2 标量
    # 每一列都必须是 float 可读的（NaN 允许，空字符串不允许）
    for value in row.values():
        assert isinstance(value, float)


def test_velocity_lowpass_stays_all_zero():
    """⚠️【已实测否定的结论】反馈路低通：2026-08-03 第 12 次上机，截止 50 Hz，
    存活 0.47 -> 0.35 s，更早发散。降噪 ~30% 却花掉 26.6° 相位就变糟 ——
    证明该回路【相位受限】而非噪声受限。配置必须保持全零。
    """
    for name in ("hwtest", "hwtest_simbest", "whole6"):
        cfg = _config(name)
        lp = (cfg.get("feedback") or {}).get("velocity_lowpass_hz", 0.0)
        assert np.allclose(np.asarray(lp, dtype=float), 0.0), name


# ==========================================================================
# 看门狗：兜底必须看【测量值】，不能看估计器输出
# ==========================================================================
# 2026-08-04 三个独立评审同时指出的同一条缺陷。本机没有软件急停通道（协议无
# 使能/失能帧，急停只能物理断电），_watchdog_trip 是唯一的自动安全兜底。估计器
# 接进来之后 qd_ctrl 变成 FD/LESO/Smith 的输出，看门狗就成了"被测对象自己判
# 自己"。下面三条把三种已复现的失效路径钉死。

def _watchdog_fake(module, qd_meas, qd_ctrl, q_meas=None):
    """喂给 _watchdog_trip 的最小假 self。"""
    import types
    n = len(qd_meas)
    return types.SimpleNamespace(
        _watchdog_tripped=False,
        config={"limits": {"joint_velocity": [3.0, 3.0, 3.0, 5.0, 5.0, 5.0]}},
        n_arm=n,
        _qd_meas=np.asarray(qd_meas, dtype=float),
        qd_ctrl=np.asarray(qd_ctrl, dtype=float),
        _q_meas=np.asarray(q_meas if q_meas is not None else np.zeros(n)),
        last_position_error=0.0,
        get_logger=lambda: types.SimpleNamespace(
            error=lambda *a, **k: None, warn=lambda *a, **k: None),
    )


def test_watchdog_reads_measured_velocity_not_estimator_output():
    """估计器把超速压回包线内时，看门狗仍然必须触发。

    复现的是 Smith 限幅那条路：修正量上限 ±[0.2,0.2,0.2,0.5,0.5,0.5] rad/s
    可以往【不安全方向】走 —— j4 真实 5.90 rad/s（阈值 1.2×5=6.0）被压成
    5.40，阈值被偏置 8.3%，方向由指令符号决定。
    """
    module = _load_runner_module()
    qd_meas = np.array([0.0, 0.0, 0.0, 6.4, 0.0, 0.0])     # 真实超 6.0
    qd_ctrl = np.array([0.0, 0.0, 0.0, 5.9, 0.0, 0.0])     # 估计器压到阈值下
    fake = _watchdog_fake(module, qd_meas, qd_ctrl)
    assert module.WholeTaskSmcRosRunner._watchdog_trip(fake, now=1.0) is True
    assert fake._watchdog_tripped is True


def test_watchdog_still_trips_while_estimator_holds_a_frozen_state():
    """hold_on_invalid 期间估计器输出被冻结，看门狗不能跟着一起瞎。

    实测场景：j3 反馈混进 NaN，估计器（缺省 hold_on_invalid=True）从此返回
    上一拍的拷贝且不自愈；同时 j4 真实速度冲到 12.0 rad/s。冻结的那份是
    健康值，只有测量值知道出事了。
    """
    module = _load_runner_module()
    qd_meas = np.array([0.0, 0.0, 0.0, 12.0, 0.0, 0.0])
    qd_ctrl = np.zeros(6)                                   # 冻结在"健康"状态
    fake = _watchdog_fake(module, qd_meas, qd_ctrl)
    assert module.WholeTaskSmcRosRunner._watchdog_trip(fake, now=1.0) is True


def test_watchdog_does_not_trip_on_estimator_noise_alone():
    """反过来也要成立：测量值健康时，估计器自己的噪声不能造成误触发。

    裸差分在量化网格上会吐出 0.076 rad/s 量级的尖峰，若看门狗读的是估计值，
    腕部包线 5.0 虽然不至于被击穿，但这条断言把方向钉住 —— 判据的输入只有
    一个来源。
    """
    module = _load_runner_module()
    qd_meas = np.array([0.1, 0.1, 0.1, 0.2, 0.2, 0.2])
    qd_ctrl = np.array([0.1, 0.1, 0.1, 0.2, 0.2, 99.0])     # 估计器发疯
    fake = _watchdog_fake(module, qd_meas, qd_ctrl)
    assert module.WholeTaskSmcRosRunner._watchdog_trip(fake, now=1.0) is False


def test_smith_shadow_integrator_adds_b0_times_u_when_model_input_is_on():
    """b0≠0 时影子积分器必须拿到 z3 + b0·u，不能只拿 z3。

    z3 在有模型输入时是【残差】：模型越准 z3 越接近 0，只喂 z3 会让预测量随
    模型变准而塌向 0，路线 D 得出假的"Smith 没用"。
    """
    est_on = _est({
        "source": "leso",
        "predict_velocity_sec": 0.010,
        "predict_source": "acceleration_estimate",
        "beta_velocity": 1.0,
        "leso": {"bandwidth_hz": 31.83, "input": "commanded_acceleration",
                 "b0": 1.0},
    })
    est_off = _est({
        "source": "leso",
        "predict_velocity_sec": 0.010,
        "predict_source": "acceleration_estimate",
        "beta_velocity": 1.0,
        "leso": {"bandwidth_hz": 31.83, "input": "none"},
    })
    a = 1.87                                    # j2 上 b0·u 的实测量级 rad/s²
    q, v = np.zeros(N), np.zeros(N)
    for est in (est_on, est_off):
        for k in range(400):                    # 走到稳态
            est.push_command(np.full(N, a), T)
            q = q + v * T + 0.5 * a * T * T
            v = v + a * T
            est(q, v, T)
        q, v = np.zeros(N), np.zeros(N)
    on = est_on(np.zeros(N), np.zeros(N), T)[1]
    off = est_off(np.zeros(N), np.zeros(N), T)[1]
    # 两条路的预测修正量必须同量级；漏加 b0·u 时 on 只有 off 的约 1/5。
    assert abs(on[0]) > 0.5 * abs(off[0]), (on[0], off[0])
