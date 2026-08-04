"""延迟稳定裕度判据。

边界层内 tanh(s/φ) -> s/φ，把 s = ė + Λe 代回控制律

    ν = ẍ_r + Λ·ė + K·s + η·tanh(s/φ)
      = ẍ_r + [Λ + K + η/φ]·ė + (K + η/φ)·Λ·e

所以【ė 的系数】c₁ = Λ + K + η/φ 才是决定延迟稳定性的量。
（2026-08-04 之前配置注释里的判据写成 K + η/φ，漏了 Λ，据此得出的
"低于临界"是错的 —— 实际超临界 9%，15 次上机全部在腕部发散。）

闭环特征方程（模型抵消精确时）：

    s² + c₁·s·e^(−s·T_v) + c₀·e^(−s·T_p) = 0,   c₀ = (K + η/φ)·Λ

纯速率反馈的临界值 c₁ = π/(2·T_v)。

⚠️ 这条判据【只覆盖仿真里存在的那个模态】。实物 24.8 Hz 极限环在这个
模型里产生不出来（纯延迟上限 10.4 Hz，速度路换一阶低通上限 13.2 Hz），
说明实物还有别的动力学。所以这里要求的裕度比"刚好稳定"厚。
"""

from pathlib import Path

import numpy as np
import pytest
import yaml

import task_space_smc

CONFIG_DIR = Path(task_space_smc.__file__).resolve().parent / "config"

# 仿真实测锚点（wf_156de504-e84，ROS 仿真，duration 60~75 s，hwtest 轨迹）
#   α=1.00 + 延迟   腕部 |v|max 5.000 rad/s（顶到包线）  交接后 RMS 6.699 mm
#   α=0.50 + 延迟   腕部 |v|max 0.441 rad/s              交接后 RMS 0.276 mm
#   α=0.35 + 延迟   腕部 |v|max 0.454 rad/s              交接后 RMS 0.303 mm
#   α=1.00 零延迟   腕部 |v|max 0.421 rad/s              交接后 RMS 0.010 mm
MEASURED_GROWTH_RATE_PER_S = 10.65   # α=1 时 |v4| 逐周期峰值对数拟合
MEASURED_LIMIT_CYCLE_HZ = 9.62       # α=1 未饱和段 (t∈[0.07,0.39])


def _config(name):
    with open(CONFIG_DIR / f"smc_params_{name}.yaml") as fh:
        return yaml.safe_load(fh) or {}


def _rate_gain(block):
    """c₁ = Λ + K + η/φ，取各轴最大（最严的那一轴决定稳定性）。"""
    lam = np.max(block["lambda"])
    k = np.max(block["k"])
    eta = np.max(block["eta"])
    phi = np.min(block["phi"])          # φ 越小切换项斜率越大
    return float(lam + k + eta / phi)


def _position_gain(block):
    """c₀ = (K + η/φ)·Λ。"""
    return float((np.max(block["k"]) + np.max(block["eta"]) / np.min(block["phi"]))
                 * np.max(block["lambda"]))


def _velocity_path_delay(cfg):
    sim = cfg.get("simulation", {}) or {}
    if "loop_delay_sec" in sim and "transport_delay_sec" not in sim:
        return float(sim["loop_delay_sec"])          # 旧的指令侧单块模型
    return float(sim.get("transport_delay_sec", 0.0)
                 + sim.get("velocity_extra_delay_sec", 0.0))


def _rightmost_root(c1, c0, t_v, t_p):
    """特征方程最右侧的振荡根。返回 (σ, f_Hz)，无振荡根时返回 None。"""
    def char(s):
        return s * s + c1 * s * np.exp(-s * t_v) + c0 * np.exp(-s * t_p)

    best = None
    for f0 in np.arange(1.0, 60.0, 0.5):
        for sig0 in (-40.0, -5.0, 0.0, 10.0):
            s = complex(sig0, 2 * np.pi * f0)
            for _ in range(200):
                h = 1e-7 * max(1.0, abs(s))
                d = (char(s + h) - char(s - h)) / (2 * h)
                if d == 0 or not np.isfinite(d):
                    break
                nxt = s - char(s) / d
                if not np.isfinite(nxt):
                    break
                if abs(nxt - s) < 1e-12:
                    s = nxt
                    break
                s = nxt
            if np.isfinite(s) and abs(char(s)) < 1e-6 and s.imag > 1e-3:
                if best is None or s.real > best.real:
                    best = s
    if best is None:
        return None
    return float(best.real), float(best.imag / (2 * np.pi))


def test_hwtest_rate_gain_is_below_the_pure_delay_critical_value():
    cfg = _config("hwtest")
    c1 = _rate_gain(cfg["position"])
    t_v = _velocity_path_delay(cfg)
    assert t_v > 0, "hwtest 必须建速度路延迟，否则这条判据无意义"
    critical = np.pi / (2 * t_v)
    # 留 40% 裕度：实物存在这个模型覆盖不到的 24.8 Hz 模态
    assert c1 < 0.6 * critical, (
        f"c₁ = Λ+K+η/φ = {c1:.1f} /s 相对临界 π/(2·T_v) = {critical:.1f} /s "
        f"裕度不足（比值 {c1 / critical:.2f}，要求 < 0.60）。"
        "2026-08-03 那组 c₁=68.4 是临界的 1.09 倍，15 次上机全部在腕部发散。")


def test_hwtest_closed_loop_has_no_growing_mode():
    """直接解特征方程，比上面那条 π/(2T) 近似严格。"""
    cfg = _config("hwtest")
    sim = cfg.get("simulation", {}) or {}
    t_v = _velocity_path_delay(cfg)
    t_p = float(sim.get("transport_delay_sec", t_v))
    root = _rightmost_root(_rate_gain(cfg["position"]),
                           _position_gain(cfg["position"]), t_v, t_p)
    if root is None:
        return                                   # 全实根，无振荡模态
    sigma, freq = root
    assert sigma < -5.0, (
        f"最右振荡根 σ = {sigma:+.2f} /s @ {freq:.2f} Hz —— "
        "要求 σ < −5 /s（e 折 < 200 ms），否则起振衰减太慢，"
        "实物上会被未建模动力学推过零。")


def test_orientation_block_is_not_the_binding_constraint():
    """姿态那组远在临界之下，不该跟着位置一起降 —— 降它只损失姿态精度。"""
    cfg = _config("hwtest")
    c1_pos = _rate_gain(cfg["position"])
    c1_ori = _rate_gain(cfg["orientation"])
    assert c1_ori < c1_pos, (
        f"姿态 c₁ = {c1_ori:.1f} 已超过位置 c₁ = {c1_pos:.1f}，"
        "此时降位置增益换不到裕度，判据要重算")


def test_model_reproduces_the_measured_simulation_limit_cycle():
    """用 2026-08-03 那组增益回代，特征方程应当复现仿真实测的 σ 与 f。

    这是这套解析判据唯一的实证依据 —— 它一旦对不上，上面两条就都不算数。
    """
    cfg = _config("hwtest")
    t_v = _velocity_path_delay(cfg)
    t_p = float((cfg.get("simulation", {}) or {}).get("transport_delay_sec", t_v))
    lam, k, eta, phi = 21.6, 30.6, 0.324, 0.02          # 降增益之前那一组
    root = _rightmost_root(lam + k + eta / phi, (k + eta / phi) * lam, t_v, t_p)
    assert root is not None, "旧增益下应当存在振荡根"
    sigma, freq = root
    assert sigma > 0, f"旧增益必须是发散的（实测 15 次上机全炸），得到 σ={sigma:+.2f}"
    assert sigma == pytest.approx(MEASURED_GROWTH_RATE_PER_S, rel=0.25), (
        f"预测增长率 {sigma:.2f} /s 与仿真实测 {MEASURED_GROWTH_RATE_PER_S} /s 不符")
    assert freq == pytest.approx(MEASURED_LIMIT_CYCLE_HZ, rel=0.25), (
        f"预测频率 {freq:.2f} Hz 与仿真实测 {MEASURED_LIMIT_CYCLE_HZ} Hz 不符")


def test_hardware_limit_cycle_is_outside_this_model():
    """实物 24.8 Hz 这个模型产生不出来 —— 把这个已知缺口钉住。

    纯延迟结构下，把增益一路抬上去频率也只到十几赫兹。所以不要用这套
    判据去解释实物频率，它只覆盖仿真里那个模态。
    """
    cfg = _config("hwtest")
    t_v = _velocity_path_delay(cfg)
    t_p = float((cfg.get("simulation", {}) or {}).get("transport_delay_sec", t_v))
    lam, k, eta, phi = 21.6, 30.6, 0.324, 0.02
    best_freq = 0.0
    for gain in (1.0, 1.5, 2.0, 3.0):
        root = _rightmost_root((lam + k + eta / phi) * gain,
                               (k + eta / phi) * lam * gain, t_v, t_p)
        if root is not None:
            best_freq = max(best_freq, root[1])
    assert best_freq < 20.0, (
        f"模型现在能产生 {best_freq:.1f} Hz —— 已接近实物实测的 24.8 Hz。"
        "若确属同一模态，应重新评估裕度要求并更新本测试。")
