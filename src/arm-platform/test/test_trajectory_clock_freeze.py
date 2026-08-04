"""拉入期间冻结轨迹时钟的回归测试。

背景
----
偏置衰减在 [0, blend_sec] 内把参考量从"当前实际位姿"平滑并到轨迹上，但轨迹
时钟原本照常走。hwtest 圆周期 25.13 s（launch 设 MB_HWTEST_SPEED=0.01），
blend_sec = 45 s —— 拉入期间参考点自己绕了 45/25.13 = 1.79 圈：

    本测试实测（launch 环境 MB_HWTEST_SPEED=0.01，|Δp| = 156.5 mm）：
        时钟照走：参考路径长 447.6 mm，包络 240.4 mm
        冻结时钟：参考路径长 156.5 mm，包络 156.5 mm（= |Δp| 的一条直线）

机械臂忠实跟随，于是 RViz 里是一团乱麻，而那 45 s 的绕圈没有任何任务意义。

守住的核心事实：
  * 开关【关闭】时逐位等价于改动前（reference_sample(t)）——
    smc_params_hwtest_simbest.yaml / smc_params_whole6.yaml 这些已验证基线
    不写这个键，代码缺省必须是 false，否则它们会在不改一个字的情况下变行为
  * 冻结期间参考轨迹恒等于起点 reference_sample(0.0)
  * t = blend_sec 处轨迹时钟从 0 开始，且 clock(t) 连续（参考位姿无跳变）
  * 交接处参考速度阶跃远低于圆周切向速度 0.010 m/s
  * capture 必须对【冻结后】的参考量取偏置，否则交接误差不再恒为零
"""

import os
import re
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from task_space_smc.handover_blend import (  # noqa: E402
    HandoverBlend, trajectory_clock)
from task_space_smc.trajectory_reference import (  # noqa: E402
    reference_sample, traj)

SHAPE = "hwtest"
DERIV_DT = 0.005
BLEND_SEC = 45.0
CONTROL_DT = 1.0 / 200.0

# 复位位姿 -> 轨迹起点的实测位置偏置量（HANDOFF 记录 156.5 mm）。方向随便取一个
# 单位向量即可 —— 本文件测的是【时钟】，不是 FK；用固定值避免把 URDF/标定的
# 变动牵扯进来。
DELTA_P = np.array([-0.1385, -0.0384, -0.0641])
DELTA_P = DELTA_P / np.linalg.norm(DELTA_P) * 0.1565

RUNNER_SRC = open(
    os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                 "task_space_smc", "whole_task_smc_ros_runner.py"),
    encoding="utf-8").read()


def _ref(t):
    return reference_sample(float(t), SHAPE, DERIV_DT)


def _blend_at_start(freeze):
    """按控制循环的口径 capture：参考量取【该帧实际喂进去的那一份】。"""
    r0 = _ref(trajectory_clock(0.0, BLEND_SEC, freeze))
    b = HandoverBlend(BLEND_SEC)
    b.capture(0.0, r0.position_w + DELTA_P, r0.rotation_w,
              r0.position_w, r0.rotation_w,
              actual_linear_velocity=np.zeros(3),
              reference_linear_velocity=r0.linear_velocity_w)
    return b


def _blended_pull_in_path(freeze, step=CONTROL_DT):
    """[0, blend_sec] 内混合后参考位置的采样点。"""
    b = _blend_at_start(freeze)
    pts = []
    for k in range(int(round(BLEND_SEC / step)) + 1):
        t = k * step
        r = _ref(trajectory_clock(t, BLEND_SEC, freeze))
        pts.append(b.apply(t, r.position_w, r.rotation_w,
                           r.linear_velocity_w, r.angular_velocity_w,
                           r.linear_acceleration_w, r.angular_acceleration_w)[0])
    return np.asarray(pts)


def _path_length(pts):
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


# --------------------------------------------------------------------------
# 开关关闭 = 逐位等价
# --------------------------------------------------------------------------

@pytest.mark.parametrize(
    "t", [0.0, 1e-9, 0.005, 1.0, 12.5, 44.999, 45.0, 45.001, 70.0, 123.456])
def test_clock_is_identity_when_switch_is_off(t):
    """关闭时轨迹时钟就是墙钟，逐位相同（不是 approx，是 ==）。"""
    assert trajectory_clock(t, BLEND_SEC, False) == float(t)


@pytest.mark.parametrize("t", [0.0, 0.005, 3.0, 30.0, 45.0, 60.0])
def test_reference_is_bit_identical_when_switch_is_off(t):
    """关闭时整条参考量（位姿/速度/加速度/笔状态）与改动前逐位相同。"""
    got = _ref(trajectory_clock(t, BLEND_SEC, False))
    want = _ref(t)
    for field in ("position_w", "quaternion_wxyz", "rotation_w",
                  "linear_velocity_w", "angular_velocity_w",
                  "linear_acceleration_w", "angular_acceleration_w"):
        np.testing.assert_array_equal(
            getattr(got, field), getattr(want, field),
            err_msg=f"{field} 在开关关闭时必须逐位相同")
    assert got.pen_down == want.pen_down
    assert got.stroke_id == want.stroke_id


def test_zero_blend_sec_leaves_the_clock_alone():
    """blend_sec<=0 时偏置衰减本身就是关的，冻结必须退回恒等映射。

    守的是 smc_params_whole6.yaml 这类【没有 handover 段】的配置：
    即使有人给它加上 freeze 开关，只要 blend_sec=0 行为就一个字不变。
    """
    for T in (0.0, -1.0, float("nan")):
        for t in (0.0, 1.0, 45.0, 100.0):
            assert trajectory_clock(t, T, True) == float(t)


# --------------------------------------------------------------------------
# 冻结期间：参考轨迹恒为起点
# --------------------------------------------------------------------------

@pytest.mark.parametrize(
    "t", [0.0, 0.005, 1.0, 12.566, 25.13, 44.0, 44.995])
def test_reference_is_pinned_to_the_start_while_frozen(t):
    """t < blend_sec 期间参考轨迹逐位等于 reference_sample(0.0)。"""
    assert trajectory_clock(t, BLEND_SEC, True) == 0.0
    got = _ref(trajectory_clock(t, BLEND_SEC, True))
    want = _ref(0.0)
    np.testing.assert_array_equal(got.position_w, want.position_w)
    np.testing.assert_array_equal(got.rotation_w, want.rotation_w)
    np.testing.assert_array_equal(got.linear_velocity_w, want.linear_velocity_w)


def test_pen_state_follows_the_frozen_clock():
    """笔状态与 desired_* 必须出自【同一时刻】的同一个 sample。

    冻结期间轨迹停在 t=0，笔状态就是起笔状态；要把拉入段摘出来看
    trajectory_clock==0（或 handover_blend>0）那一列，信息没有丢。
    """
    start = _ref(0.0)
    for t in (0.0, 10.0, 44.9):
        r = _ref(trajectory_clock(t, BLEND_SEC, True))
        assert r.pen_down == start.pen_down
        assert r.stroke_id == start.stroke_id


def test_frozen_reference_velocity_matches_a_standing_still_reference():
    """冻结期间参考【位置】不动，所以参考速度也必须≈0，否则前馈自相矛盾。

    hwtest 起点有 20% 周期的 smoothstep 起步整形，v(0) 实测 2.5e-08 m/s
    （MB_HWTEST_SPEED=0.01 时）—— 与"位置恒定"一致到 7 位有效数字以上。
    """
    v0 = np.linalg.norm(_ref(0.0).linear_velocity_w)
    assert v0 < 1e-5, f"轨迹起点速度 {v0} 太大，冻结会造出矛盾的速度前馈"


# --------------------------------------------------------------------------
# 交接点：时钟从 0 开始、位姿连续、速度阶跃可忽略
# --------------------------------------------------------------------------

def test_clock_restarts_from_zero_at_blend_end():
    assert trajectory_clock(BLEND_SEC, BLEND_SEC, True) == 0.0
    for x in (0.0, 0.005, 1.0, 25.13, 60.0):
        assert trajectory_clock(BLEND_SEC + x, BLEND_SEC, True) == pytest.approx(
            x, abs=1e-12)


def test_clock_is_continuous_so_the_reference_pose_never_jumps():
    """clock(t) 在 t=blend_sec 处左右极限都是 0 —— 参考位姿没有台阶。"""
    eps = 1e-9
    before = trajectory_clock(BLEND_SEC - eps, BLEND_SEC, True)
    after = trajectory_clock(BLEND_SEC + eps, BLEND_SEC, True)
    assert before == 0.0
    assert after == pytest.approx(0.0, abs=2e-9)
    p_before = _ref(before).position_w
    p_after = _ref(after).position_w
    np.testing.assert_allclose(p_before, p_after, atol=1e-9)


def test_reference_velocity_step_at_handover_stays_below_tangential_speed():
    """交接处参考速度阶跃必须落在圆周切向速度（0.010 m/s）的量级以下。

    (K+Λ)·Δv = 52.2 × 0.010 = 0.52 m/s²，相对该位形 48.8 m/s² 的能力上限是 1%。

    实测远小于这个上界：clock(t) 连续 + 轨迹自身的起步整形使 v(clock) 也连续，
    相邻控制帧之间实测 1.7e-07 m/s（MB_HWTEST_SPEED=0.01）/ 1.4e-06 m/s（0.02）。
    """
    v_peak = max(
        float(np.linalg.norm(_ref(t).linear_velocity_w))
        for t in np.arange(0.0, 2.0 * traj.ee_trajectory_period(SHAPE), 0.05))
    v_before = _ref(trajectory_clock(
        BLEND_SEC - CONTROL_DT, BLEND_SEC, True)).linear_velocity_w
    v_after = _ref(trajectory_clock(
        BLEND_SEC + CONTROL_DT, BLEND_SEC, True)).linear_velocity_w
    step = float(np.linalg.norm(v_after - v_before))
    assert step <= v_peak, "阶跃不可能超过轨迹自身的峰值切向速度"
    assert step < 0.010, "阶跃必须在圆周切向速度 0.010 m/s 的量级以下"
    assert step < 1e-4, f"实测应远小于上界，得到 {step}"


def test_demanded_acceleration_from_the_handover_step_is_negligible():
    """把上界 0.010 m/s 代进 hwtest 增益，需求加速度必须远低于能力上限。"""
    cfg = _config("hwtest")
    lam = float(np.max(cfg["position"]["lambda"]))
    k = float(np.max(cfg["position"]["k"]))
    demand = (lam + k) * 0.010
    # 不锁死具体数值 —— 增益本身由延迟稳定裕度决定（见
    # test_delay_stability_margin.py），2026-08-04 从 λ+K=52.2 降到 18.3。
    # 这里只管"交接阶跃的需求相对能力上限可忽略"这一条。
    assert demand < 0.02 * 48.8, (
        f"需求 {demand:.3f} m/s² 相对 48.8 m/s² 的能力上限应在 2% 以内")


# --------------------------------------------------------------------------
# 拉入段参考路径：从螺旋变成直线
# --------------------------------------------------------------------------

def test_frozen_pull_in_reference_is_a_straight_converging_line():
    """冻结后拉入段参考路径就是长度 |Δp| 的一条直线，单调收敛、不绕圈。"""
    pts = _blended_pull_in_path(freeze=True)
    length = _path_length(pts)
    envelope = float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)))
    np.testing.assert_allclose(length, np.linalg.norm(DELTA_P), rtol=1e-6)
    np.testing.assert_allclose(envelope, np.linalg.norm(DELTA_P), rtol=1e-6)
    # 终点必须落在轨迹起点上（偏置已完全衰减）
    np.testing.assert_allclose(pts[-1], _ref(0.0).position_w, atol=1e-12)
    # 直线：所有点与首末连线的横向偏差可忽略。
    # 不是【严格】为零：Δv = 0 − v_traj(0) 不恰好是零向量（起步整形残留
    # 2.5e-08 m/s），B(τ) 峰值 ≈0.35·T 把它放大成 ~1e-06 m 的横向鼓包。
    # 实测 0.73 µm（MB_HWTEST_SPEED=0.02）/ 0.09 µm（0.01），比 |Δp| 小 5 个量级。
    d = pts[-1] - pts[0]
    d = d / np.linalg.norm(d)
    lateral = np.linalg.norm(
        (pts - pts[0]) - np.outer((pts - pts[0]) @ d, d), axis=1)
    assert float(lateral.max()) < 1e-5


def test_running_clock_makes_the_pull_in_reference_spiral():
    """守住这次要修的缺陷本身：时钟照走时参考路径显著变长（绕圈）。

    实测长度比（路径长 / |Δp|）：
        MB_HWTEST_SPEED=0.01（launch 用的值，周期 25.13 s）  447.6/156.5 = 2.86
        MB_HWTEST_SPEED=0.02（模块缺省值，周期 12.57 s）      886.6/156.5 = 5.66
    断言取 >2 以对两种环境都成立。
    """
    frozen = _path_length(_blended_pull_in_path(freeze=True))
    running = _path_length(_blended_pull_in_path(freeze=False))
    assert running > 2.0 * frozen, (
        f"时钟照走时参考路径应显著更长：{running:.3f} vs {frozen:.3f}")
    turns = BLEND_SEC / traj.ee_trajectory_period(SHAPE)
    assert turns > 1.0, "blend_sec 超过一个轨迹周期才会绕圈，这是缺陷的成因"


# --------------------------------------------------------------------------
# capture 的取舍：偏置必须对【冻结后】的参考量取
# --------------------------------------------------------------------------

def test_capture_must_use_the_clocked_reference():
    """Δp 的定义是"实际量 − 参考量"，参考量是哪一份，偏置就得对哪一份取。

    否则 τ=0 处 e 不再恒等于零，整个偏置衰减机制（交接误差恒为零）失效。
    """
    p_act = _ref(0.0).position_w + DELTA_P
    b = _blend_at_start(freeze=True)
    r = _ref(trajectory_clock(0.0, BLEND_SEC, True))
    p_blend = b.apply(0.0, r.position_w, r.rotation_w, r.linear_velocity_w,
                      r.angular_velocity_w, r.linear_acceleration_w,
                      r.angular_acceleration_w)[0]
    assert float(np.linalg.norm(p_blend - p_act)) < 1e-12


def test_metric_against_an_unclocked_reference_would_be_meaningless():
    """解释为什么 pos_error / desired_* 必须也用冻结后的 ref。

    冻结开启后 t>blend_sec 的参考量是 reference_sample(t-blend_sec)。若指标仍拿
    reference_sample(t) 去比，比的是圆上差了 blend_sec 相位的【另一个点】，
    误差量级直逼圆直径 —— 整段跟踪的 RMS/peak 全部作废，不只是拉入段。
    """
    worst = max(
        float(np.linalg.norm(_ref(t).position_w
                             - _ref(t - BLEND_SEC).position_w))
        for t in np.arange(BLEND_SEC, BLEND_SEC + 30.0, 0.1))
    assert worst > 0.02, (
        f"相位差造成的伪误差 {worst * 1000:.1f} mm，与 1 mm 的精度规格差两个量级")


# --------------------------------------------------------------------------
# 配置与实现的一致性
# --------------------------------------------------------------------------

def _config(name):
    from pathlib import Path
    import task_space_smc
    path = (Path(task_space_smc.__file__).resolve().parent
            / "config" / f"smc_params_{name}.yaml")
    if not path.exists():
        return None
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def test_hwtest_enables_the_freeze():
    cfg = _config("hwtest")
    assert cfg is not None
    handover = cfg["handover"]
    assert handover["freeze_trajectory_during_blend"] is True
    assert handover["reference_blend_sec"] > 0.0


def test_verified_baselines_do_not_carry_the_key():
    """已验证基线不写这个键，配合代码缺省 false 保证它们逐位不变。"""
    for name in ("hwtest_simbest", "whole6", "whole"):
        cfg = _config(name)
        if cfg is None:
            continue
        handover = cfg.get("handover") or {}
        assert "freeze_trajectory_during_blend" not in handover, (
            f"{name} 是已验证基线，不应引入冻结开关")


def test_runner_defaults_the_switch_to_off():
    """代码缺省必须是 False —— simbest 的 blend_sec=3.0 且没写这个键。"""
    assert re.search(
        r'freeze_trajectory_during_blend\s*=\s*bool\(\s*\n?\s*handover_cfg\.get\('
        r'"freeze_trajectory_during_blend",\s*False\)\)',
        RUNNER_SRC), "缺省值必须显式写成 False"


def test_runner_feeds_the_clock_into_reference_sample():
    """控制循环必须喂 traj_t，且不得再有 reference_sample(t, ...) 的调用。"""
    assert "traj_t = self._trajectory_clock(t)" in RUNNER_SRC
    assert "ref = reference_sample(traj_t, self.shape, self.derivative_dt)" in RUNNER_SRC
    assert not re.search(r"reference_sample\(\s*t\s*,", RUNNER_SRC), \
        "控制循环里不能再直接用墙钟 t 采轨迹"


def test_runner_records_the_trajectory_clock_column():
    """CSV 必须有 trajectory_clock 列，跟踪段与 approach 段都要写。"""
    assert '"trajectory_clock",' in RUNNER_SRC
    assert '"trajectory_clock": float(traj_t),' in RUNNER_SRC
    assert '"trajectory_clock": 0.0,' in RUNNER_SRC
