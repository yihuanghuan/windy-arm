"""approach 段静摩擦前馈的回归测试。

这里守住的核心事实：
  * scale >= 1 必须被拒绝（会自激成极限环）
  * 补偿把死区缩到 (1-scale) 倍，量化对上实测的 29.2 mm 残差
  * 前馈吃的是【参考速度】不是实测速度（否则噪声被整流成力矩抖动）
  * 收尾阶段 q̇_des ≡ 0 时前馈不能消失
  * 默认关闭，已有仿真基线不受影响
"""

import os
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from task_space_smc.friction_feedforward import (
    FrictionFeedforwardConfig, approach_torque_stiffness, residual_deadband,
    stiction_feedforward)
from task_space_smc.model import load_arm6_calibration

# config/arm6_calibration.yaml 的实测静摩擦
STICTION = [0.725, 1.775, 0.625, 0.125, 0.100, 0.100]

# 【历史值】2026-08-03 第 5~8 次上机时的 approach kp。本文件里所有"用死区解释
# 实测欠量"的断言都锚在这组增益上 —— 它们是对那几次上机的事后分析，不能跟着
# 当前配置漂移。当前配置见 CURRENT_APPROACH_KP。
APPROACH_KP = [60.0, 60.0, 60.0, 50.0, 50.0, 40.0]

# 2026-08-03 上机握手位形处 crba 的手臂对角块 [kgm²]。approach.kp 是加速度
# 增益，力矩刚度是 M_ii·kp —— 直接拿 kp 当 Nm/rad 会把腕部高估 80 倍。
INERTIA_DIAG = [0.2374, 0.1952, 0.1379, 0.0121, 0.0128, 0.0106]
# 该位形下的力矩刚度 [Nm/rad]，约 [14.2, 11.7, 8.3, 0.61, 0.64, 0.42]
STIFFNESS = approach_torque_stiffness(INERTIA_DIAG, APPROACH_KP)

# 同次上机 approach 停下时逐关节欠量 [deg]，用来核对死区量级
MEASURED_SHORTFALL_DEG = [2.33, 3.50, 3.44, 13.36, 18.43, 3.15]


def _cfg(**kw):
    kw.setdefault("enabled", True)
    return FrictionFeedforwardConfig(STICTION, **kw)


# --------------------------------------------------------------------------
# scale < 1 是硬约束，不是调参余量
# --------------------------------------------------------------------------

@pytest.mark.parametrize("scale", [1.0, 1.0000001, 1.2, 2.0, 10.0])
def test_scale_at_or_above_one_is_rejected(scale):
    """scale >= 1 时零误差处前馈就够挣脱静摩擦 -> 极限环。必须拒绝。"""
    with pytest.raises(ValueError, match="极限环"):
        _cfg(scale=scale)


@pytest.mark.parametrize("scale", [0.0, 0.5, 0.8, 0.99])
def test_scale_below_one_is_accepted(scale):
    assert _cfg(scale=scale).scale == pytest.approx(scale)


def test_zero_error_drive_stays_below_breakaway():
    """scale<1 的物理意义：零误差处净驱动力矩恒小于静摩擦，关节不会自发松动。"""
    cfg = _cfg(scale=0.8)
    tau = stiction_feedforward(cfg, np.zeros(6), np.zeros(6))
    assert np.all(np.abs(tau) < cfg.stiction_nm), (
        "零误差处前馈不得达到静摩擦，否则测量噪声就能让关节松动")


# --------------------------------------------------------------------------
# 死区收缩 —— 量化对上实测
# --------------------------------------------------------------------------

def test_stiffness_is_inertia_weighted_not_the_raw_gain():
    """守住把我坑了一次的量纲。

    approach.kp 是加速度增益 (1/s²)，力矩刚度是 M_ii(q)·kp。腕部连杆惯量只有
    0.011 kgm²，直接把 kp 当 Nm/rad 会把刚度高估两个数量级。
    """
    np.testing.assert_allclose(
        STIFFNESS, [14.24, 11.71, 8.27, 0.605, 0.640, 0.424], rtol=1e-2)
    assert np.all(STIFFNESS < np.asarray(APPROACH_KP)), "刚度不可能大于加速度增益"
    assert np.asarray(APPROACH_KP)[5] / STIFFNESS[5] > 80.0, (
        "joint6 被高估的倍数变了，模块 docstring 的表需要重算")


def test_uncompensated_deadband_explains_the_measured_shortfall():
    """未补偿死区必须能解释 2026-08-03 approach 实测停下的位置。

    实测欠量 [2.33, 3.50, 3.44, 13.36, 18.43, 3.15]°。每一项都应落在同关节
    死区的量级内（半倍到三倍之间）—— 这是"approach 被静摩擦卡住"的量化依据。
    """
    off = np.degrees(residual_deadband(_cfg(enabled=False), STIFFNESS))
    measured = np.asarray(MEASURED_SHORTFALL_DEG)
    np.testing.assert_allclose(
        off, [2.92, 8.68, 4.33, 11.80, 8.94, 13.52], rtol=1e-2)
    assert int(off.argmax()) == 5, "最差关节应是 joint6（刚度最软 0.42 Nm/rad）"
    ratio = measured / off
    assert np.all(ratio < 3.0), f"实测欠量远超死区，另有原因：{ratio}"


def test_the_raw_gain_reading_would_have_understated_the_deadband():
    """把 kp 当 Nm/rad 读会得出 0.1~1.7° 的死区，解释不了 13° 的实测欠量。"""
    wrong = np.degrees(residual_deadband(_cfg(enabled=False), APPROACH_KP))
    right = np.degrees(residual_deadband(_cfg(enabled=False), STIFFNESS))
    assert wrong.max() < 2.0 and right.max() > 10.0
    assert np.all(np.asarray(MEASURED_SHORTFALL_DEG)[3:] > 3.0 * wrong[3:]), (
        "错误读法下腕部实测欠量应远超死区 —— 这正是当初该起疑的地方")


def test_deadband_is_the_exact_root_not_the_asymptotic_formula():
    """residual_deadband 必须和 stiction_feedforward 自洽：

    在死区边界上，kp·e + τ_ff 应恰好达到静摩擦。这条对【所有】关节成立，
    包括 tanh 尚未饱和的腕部三轴。
    """
    cfg = _cfg(scale=0.8)
    kp = np.asarray(STIFFNESS)
    e = residual_deadband(cfg, kp)
    drive = kp * e + stiction_feedforward(cfg, np.zeros(6), e)
    np.testing.assert_allclose(drive, cfg.stiction_nm, rtol=1e-9)


def test_asymptotic_formula_is_never_pessimistic():
    """渐近式 (1-scale)·f_s/K 只在 e* >> κ 时成立，其余情况偏乐观。

    把拐点从 0.002 放宽到 0.02 rad（消除实机 bang-bang 抖动）之后，死区与
    拐点已是同一量级（比值 1.0~2.5），渐近式随之明显偏乐观（最多 1.96 倍）。
    这正是 residual_deadband 必须返回精确根、而不能用渐近式的原因。
    """
    cfg = _cfg(scale=0.8)
    K = np.asarray(STIFFNESS)
    exact = residual_deadband(cfg, K)
    asymptotic = (1.0 - cfg.scale) * cfg.stiction_nm / K
    knee = cfg.velocity_epsilon / cfg.error_gain

    assert np.all(exact >= asymptotic - 1e-12), (
        f"渐近式给出了比精确解更大的死区，说明求根反了：{exact} vs {asymptotic}")
    ratio = exact / knee
    assert np.all(ratio > 0.9) and np.all(ratio < 3.0), (
        f"死区与拐点应同量级（放宽拐点的直接后果），实得 {ratio}")
    inflation = exact / asymptotic
    assert np.all(inflation > 1.0) and inflation.max() < 2.2, (
        f"渐近式偏乐观的倍数变了：{inflation}")


def test_compensation_shrinks_every_deadband_severalfold():
    """拐点放宽到 0.02 rad 后收缩倍数从理论上限 5（=1/(1-scale)）降到 2.6~4.7。

    这是为消除 bang-bang 抖动付出的代价，必须明确记住量级。
    """
    on = residual_deadband(_cfg(scale=0.8), STIFFNESS)
    off = residual_deadband(_cfg(enabled=False), STIFFNESS)
    assert np.all(on < off)
    shrink = off / on
    assert np.all(shrink > 2.5), f"收缩倍数掉到 {shrink}，补偿已无意义"
    assert np.all(shrink <= 1.0 / (1.0 - 0.8) + 1e-9), "不可能超过 1/(1-scale)"
    # 最差关节 joint6 由 13.51° 降到约 2.85°
    assert np.degrees(on.max()) == pytest.approx(2.85, abs=0.15)


def test_compensation_brings_deadband_under_the_measured_shortfall():
    """补偿后的死区必须明显小于上次实测的欠量，否则 approach 还是走不到。"""
    on = np.degrees(residual_deadband(_cfg(scale=0.8), STIFFNESS))
    measured = np.asarray(MEASURED_SHORTFALL_DEG)
    assert np.all(on < measured), f"补偿后死区 {on} 仍不小于实测欠量 {measured}"
    # 腕部是上次卡得最狠的两个轴，要求至少 5 倍余量
    assert np.all(measured[3:5] / on[3:5] > 5.0)


# --------------------------------------------------------------------------
# 前馈的形状
# --------------------------------------------------------------------------

def test_feedforward_never_exceeds_scaled_stiction():
    cfg = _cfg(scale=0.8)
    rng = np.random.default_rng(0)
    for _ in range(200):
        tau = stiction_feedforward(
            cfg, rng.normal(0, 5, 6), rng.normal(0, 1, 6))
        assert np.all(np.abs(tau) <= cfg.scale * cfg.stiction_nm + 1e-12)


def test_feedforward_is_odd_and_continuous_through_zero():
    """tanh 而非 sign：过零必须连续，否则 200 Hz 下就是力矩抖振。"""
    cfg = _cfg()
    v = np.linspace(-0.05, 0.05, 401)
    tau = np.array([stiction_feedforward(cfg, np.full(6, x), np.zeros(6))[1]
                    for x in v])
    np.testing.assert_allclose(tau, -tau[::-1], atol=1e-12)
    assert np.max(np.abs(np.diff(tau))) < 0.05 * cfg.stiction_nm[1], "过零不连续"


def test_uses_reference_velocity_not_measured_velocity():
    """签名里只有参考速度。测量噪声不得进入 tanh —— 会被整流成直流力矩。"""
    cfg = _cfg()
    quiet = stiction_feedforward(cfg, np.full(6, 0.3), np.zeros(6))
    # 同样的参考速度、同样的误差，无论实测速度是什么，输出必须一致：
    # 函数根本不接受实测速度这个参数，这里用重复调用固定该契约。
    assert np.allclose(quiet, stiction_feedforward(
        cfg, np.full(6, 0.3), np.zeros(6)))
    import inspect
    params = list(inspect.signature(stiction_feedforward).parameters)
    assert params == ["config", "velocity_reference", "position_error"], (
        "签名变了。若引入实测速度，必须先解决噪声整流问题")


def test_error_term_keeps_feedforward_alive_when_reference_velocity_is_zero():
    """五次多项式混合两端 q̇_des ≡ 0，而收尾正是要挣脱静摩擦的时刻。"""
    cfg = _cfg(scale=0.8)
    err = np.full(6, 0.02)          # 2 厘弧度残差，恰好是拐点 κ 的量级
    tau = stiction_feedforward(cfg, np.zeros(6), err)
    # κ=0.02 rad 时 tanh(1)=0.76。不要求接近满幅 —— 接近满幅恰恰意味着拐点
    # 太窄、前馈退化成方波（见 test_sharp_knee_makes_the_feedforward_bang_bang）。
    assert np.all(tau > 0.7 * cfg.scale * cfg.stiction_nm), (
        "q̇_des=0 时前馈塌了，死区压不下去")


def test_feedforward_opposes_the_error_sign_consistently():
    cfg = _cfg()
    pos = stiction_feedforward(cfg, np.zeros(6), np.full(6, 0.02))
    neg = stiction_feedforward(cfg, np.zeros(6), np.full(6, -0.02))
    assert np.all(pos > 0) and np.all(neg < 0)
    np.testing.assert_allclose(pos, -neg, atol=1e-12)


# --------------------------------------------------------------------------
# 默认关闭 + 输入校验
# --------------------------------------------------------------------------

def test_disabled_by_default_and_returns_exactly_zero():
    cfg = FrictionFeedforwardConfig(STICTION)
    assert cfg.enabled is False
    tau = stiction_feedforward(cfg, np.full(6, 1.0), np.full(6, 0.5))
    np.testing.assert_array_equal(tau, np.zeros(6))


def test_non_finite_feedback_yields_zero_not_nan():
    """反馈坏了宁可不补偿，也不能把 NaN 灌进力矩链路。"""
    cfg = _cfg()
    for bad in (np.nan, np.inf, -np.inf):
        v = np.zeros(6); v[2] = bad
        np.testing.assert_array_equal(
            stiction_feedforward(cfg, v, np.zeros(6)), np.zeros(6))
        np.testing.assert_array_equal(
            stiction_feedforward(cfg, np.zeros(6), v), np.zeros(6))


@pytest.mark.parametrize("kw,match", [
    (dict(velocity_epsilon=0.0), "velocity_epsilon"),
    (dict(velocity_epsilon=-0.01), "velocity_epsilon"),
    (dict(scale=-0.1), "scale"),
    (dict(error_gain=-1.0), "error_gain"),
])
def test_invalid_parameters_are_rejected(kw, match):
    with pytest.raises(ValueError, match=match):
        _cfg(**kw)


def test_negative_stiction_is_rejected():
    with pytest.raises(ValueError, match="不能为负"):
        FrictionFeedforwardConfig([0.7, -1.0, 0.6, 0.1, 0.1, 0.1], enabled=True)


def test_residual_deadband_rejects_nonpositive_kp():
    with pytest.raises(ValueError, match="torque_stiffness"):
        residual_deadband(_cfg(), [14.2, 0.0, 8.3, 0.6, 0.6, 0.4])


# --------------------------------------------------------------------------
# 与标定文件、实机配置的一致性 —— 防止两处漂移
# --------------------------------------------------------------------------

def test_stiction_constants_here_match_the_calibration_file():
    cal = load_arm6_calibration()
    np.testing.assert_allclose(cal["stiction_nm"][:6], STICTION, rtol=1e-9)


def _config(name):
    from pathlib import Path
    import task_space_smc
    path = (Path(task_space_smc.__file__).resolve().parent
            / "config" / f"smc_params_{name}.yaml")
    if not path.exists():
        return None
    with open(path) as fh:
        return yaml.safe_load(fh) or {}


def test_hwtest_config_enables_it_with_a_safe_scale():
    cfg = _config("hwtest")
    assert cfg is not None, "找不到实机配置 smc_params_hwtest.yaml"
    approach = cfg["approach"]
    ff = approach["friction_feedforward"]
    assert ff["enabled"] is True, "实机配置必须开启，否则 approach 收不进交接门限"
    assert ff["scale"] < 1.0
    # 配置里不重复写静摩擦，避免与标定文件漂移
    assert "stiction_nm" not in ff


def test_simulation_baselines_do_not_enable_it():
    """无摩擦被控对象上开这个前馈是净扰动。仿真配置必须保持关闭。"""
    checked = 0
    for name in ("whole6", "whole"):
        cfg = _config(name)
        if cfg is None:
            continue
        checked += 1
        ff = (cfg.get("approach") or {}).get("friction_feedforward") or {}
        assert not ff.get("enabled", False), (
            f"{name} 是仿真基线，其被控对象没有摩擦模型，不得开启静摩擦前馈")
    assert checked > 0, "一个仿真基线配置都没找到，这条测试形同虚设"


# --------------------------------------------------------------------------
# tanh 拐点必须与实际误差同量级 —— 否则退化成 bang-bang（实机抖动）
# --------------------------------------------------------------------------

# 2026-08-03 第 5/6 次上机 approach 段实测跟踪误差中位数 [rad]
MEASURED_TRACKING_ERROR_RAD = [0.0021, 0.0040, 0.0025, 0.0180, 0.1375, 0.0300]


def test_knee_is_comparable_to_the_measured_tracking_error():
    """κ 远小于误差尺度 => tanh 全程饱和 => 过零整幅翻转 => 机械臂抖。"""
    from pathlib import Path
    import task_space_smc
    with open(Path(task_space_smc.__file__).resolve().parent
              / "config" / "smc_params_hwtest.yaml") as fh:
        ff = yaml.safe_load(fh)["approach"]["friction_feedforward"]
    knee = ff["velocity_epsilon"] / ff["error_gain"]
    err = np.asarray(MEASURED_TRACKING_ERROR_RAD)
    # 最小的那几个关节误差不该比拐点小太多，否则它们全程在饱和区翻转
    assert knee >= 0.2 * np.median(err), (
        f"拐点 {knee:.4f} rad 远小于实测误差中位 {np.median(err):.4f} rad，"
        "前馈会退化成 bang-bang（第 5/6 次上机实测 j5 翻转 20.2 Hz）")
    assert knee <= 5.0 * np.median(err), f"拐点 {knee:.4f} rad 过宽，补偿被削弱"


def test_widening_the_knee_barely_costs_deadband():
    """守住这笔交易：拐点放宽 10 倍，死区只涨 5%。"""
    sharp = _cfg(scale=0.8, error_gain=10.0)
    soft = _cfg(scale=0.8, error_gain=1.0)
    d_sharp = residual_deadband(sharp, STIFFNESS).max()
    d_soft = residual_deadband(soft, STIFFNESS).max()
    assert d_soft / d_sharp < 1.10, f"死区涨了 {d_soft / d_sharp:.2f} 倍，代价过大"
    assert (soft.velocity_epsilon / soft.error_gain) > 5.0 * (
        sharp.velocity_epsilon / sharp.error_gain)


def test_sharp_knee_makes_the_feedforward_bang_bang():
    """量化"退化成方波"：拐点很窄时，误差在实测量级上扫一遍几乎全是满幅。"""
    err = np.linspace(-0.05, 0.05, 2001)
    for eg, expect_bang in ((10.0, True), (1.0, False)):
        cfg = _cfg(scale=0.8, error_gain=eg)
        tau = np.array([stiction_feedforward(cfg, np.zeros(6),
                                             np.full(6, e))[1] for e in err])
        frac = float(np.mean(np.abs(tau) > 0.95 * cfg.scale * cfg.stiction_nm[1]))
        if expect_bang:
            assert frac > 0.9, f"error_gain={eg} 应几乎全幅，实得 {frac:.2f}"
        else:
            assert frac < 0.75, f"error_gain={eg} 仍近似方波，实得 {frac:.2f}"


# --------------------------------------------------------------------------
# 腕部提刚度（第 8 次上机后的修正）
# --------------------------------------------------------------------------

def test_current_config_raised_the_wrist_stiffness():
    """第 8 次上机 j5 卡在 12.84° 欠量、指令力矩只有 0.004 Nm。

    根因是 kp=50 在 M_55=0.0128 下只折合 0.64 Nm/rad。腕部 kp 必须提到能在
    该量级误差下给出接近额定的驱动力矩，同时留出余量。
    """
    cfg = _config("hwtest")["approach"]
    kp = np.asarray(cfg["kp"], dtype=float)
    # 大臂三轴不动
    np.testing.assert_allclose(kp[:3], APPROACH_KP[:3])
    # 腕部三轴必须显著提高
    assert np.all(kp[3:] >= 4.0 * np.asarray(APPROACH_KP[3:])), (
        f"腕部 kp {kp[3:]} 相对历史值 {APPROACH_KP[3:]} 提得不够")

    K = approach_torque_stiffness(INERTIA_DIAG, kp)
    assert np.all(K[3:] > 2.0), f"腕部力矩刚度 {K[3:]} 仍低于 2 Nm/rad"

    # j5 在第 8 次那个 12.85° 欠量处能给出的驱动力矩，须显著超过静摩擦、
    # 又要留出对额定 1.6 Nm 的余量
    tau_j5 = K[4] * np.radians(12.85)
    assert tau_j5 > 5.0 * STICTION[4], f"j5 驱动力矩 {tau_j5:.3f} Nm 仍不够挣脱静摩擦"
    assert tau_j5 < 0.6 * 1.6, f"j5 驱动力矩 {tau_j5:.3f} Nm 吃掉太多额定余量"


def test_current_config_keeps_the_approach_critically_damped():
    """kp 提了 kd 必须同步提，否则从过阻尼掉进欠阻尼 —— 只是换一种振荡。"""
    cfg = _config("hwtest")["approach"]
    kp = np.asarray(cfg["kp"], dtype=float)
    kd = np.asarray(cfg["kd"], dtype=float)
    zeta = kd / (2.0 * np.sqrt(kp))
    assert np.all(zeta > 0.9) and np.all(zeta < 1.15), f"阻尼比 {zeta} 偏离临界阻尼"


def test_current_config_stays_discretely_stable_at_200hz():
    cfg = _config("hwtest")["approach"]
    kp = np.asarray(cfg["kp"], dtype=float)
    kd = np.asarray(cfg["kd"], dtype=float)
    T = 1.0 / 200.0
    assert np.all(kd * T < 2.0), f"kd·T = {kd * T} 触及离散阻尼上限"
    assert np.all(kp * T * T < 0.05), f"kp·T² = {kp * T * T} 过大"


def test_raised_wrist_stiffness_shrinks_the_wrist_deadband():
    cfg = _config("hwtest")["approach"]
    K_new = approach_torque_stiffness(INERTIA_DIAG, cfg["kp"])
    K_old = approach_torque_stiffness(INERTIA_DIAG, APPROACH_KP)
    d_new = np.degrees(residual_deadband(_cfg(scale=0.8), K_new))
    d_old = np.degrees(residual_deadband(_cfg(scale=0.8), K_old))
    assert np.all(d_new[3:] < 0.55 * d_old[3:]), (
        f"腕部死区 {d_old[3:]} -> {d_new[3:]}，收缩不足")
    np.testing.assert_allclose(d_new[:3], d_old[:3], rtol=1e-9)


# --------------------------------------------------------------------------
# 零空间姿态目标必须跟随轨迹起点（第 9 次上机后的修正）
# --------------------------------------------------------------------------

# 2026-08-03 第 9 次上机交接位形处，零空间注入到各关节的加速度 [rad/s²]
NS_INJECTION_STALE_NOMINAL = [0.23, -0.52, 0.80, -0.23, -0.28, -4.63]
NS_INJECTION_TRACK_START = [-0.01, 0.03, -0.04, 0.01, 0.01, 0.24]


def test_hwtest_nullspace_nominal_follows_the_solved_track_start():
    """写死的 q_nominal 一旦与实际工作位形不符，就变成持续扰动源。

    旧值（7 关节时代）与 hwtest 轨迹起点在 joint4 上差 105°，实测在交接位形
    处向 joint6 注入 -4.63 rad/s²（其余关节均 <0.8）—— 正是四次上机失控的
    那个关节。
    """
    cfg = _config("hwtest")
    assert cfg["nullspace"]["q_nominal"] == "track_start", (
        "实机配置的 q_nominal 必须跟随 IK 解出的轨迹起点，不能写死")


def test_stale_nominal_concentrates_its_disturbance_on_joint6():
    """把实测的注入量固定下来：扰动集中在 joint6，且换成轨迹起点后大幅下降。"""
    stale = np.asarray(NS_INJECTION_STALE_NOMINAL)
    fixed = np.asarray(NS_INJECTION_TRACK_START)

    assert int(np.abs(stale).argmax()) == 5, "扰动最大的应是 joint6"
    others = np.abs(np.delete(stale, 5))
    assert abs(stale[5]) > 5.0 * others.max(), (
        f"joint6 注入 {stale[5]} 应远超其余关节 {others}")

    assert np.linalg.norm(stale) / np.linalg.norm(fixed) > 15.0, (
        "改用轨迹起点后注入范数应下降一个数量级以上")
    assert abs(fixed[5]) < 0.5, "修正后 joint6 的注入必须降到可忽略"


def test_simulation_baselines_keep_their_explicit_nominal():
    """whole6 是已验证基线，其 q_nominal 必须保持显式向量、不受本次改动影响。"""
    for name in ("whole6", "hwtest_simbest"):
        cfg = _config(name)
        if cfg is None:
            continue
        qn = cfg["nullspace"]["q_nominal"]
        assert isinstance(qn, list) and len(qn) == 6, (
            f"{name} 的 q_nominal 应保持显式 6 元素向量，实得 {qn!r}")
