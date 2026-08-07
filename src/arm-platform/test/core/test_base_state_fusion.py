"""基座 twist/accel：杠杆臂、跨通道对齐、缺数据不喂零。

【这个测试的存在理由】
桥此前只订阅 pose、自己差分出 twist/accel，而 Vicon 原始 twist/accel 一直在
发。改成用原始值之后有三个坑：

1. **杠杆臂有两个，长得都像"lever"。** 标定输出里模型 D 的 `lever_mm` 是
   17.84 mm，它被抄进了 `arm_ee_from_link6` —— 那是【末端标记】的杠杆臂。
   基座 twist/accel 要用的是 `arm_base_from_base_link.translation_m`，
   33.07 mm。用错差 15.2 mm，直接进 compute_task_bias_acceleration 变力矩。
2. **向心项 w x (w x r) 在等角速度下依然存在**，漏掉它静态测不出来。
3. **缺加速度绝不能喂零。** 0 是一个下游任何守卫都识别不出来的"合法值"。
"""

import os
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from task_space_smc.core.base_state_fusion import (  # noqa: E402
    ChannelSample, DerivativeConsistencyMonitor, FrameAssembler,
    SourceClockAligner, pair_by_source_time, shift_twist_accel, skew)

# 实测值，来自 config/vicon_extrinsics.yaml
R_BASE = np.array([0.019621, -0.026358, -0.003764])      # |r| = 33.07 mm
T_EE = np.array([0.0119, 0.0047, 0.0125])                # |t| = 17.89 mm


# ---------------- 杠杆臂

def _expm_skew(v):
    """精确的 Rodrigues 旋转矩阵。

    差分测试【必须】用精确旋转：一阶近似 ``I + skew(w t)`` 恰好把
    w x (w x r) 这一项丢掉，于是测试和被测代码会一起犯同一个错。
    """
    theta = float(np.linalg.norm(v))
    if theta < 1e-15:
        return np.eye(3)
    k = skew(np.asarray(v, dtype=float) / theta)
    return np.eye(3) + np.sin(theta) * k + (1.0 - np.cos(theta)) * (k @ k)


def test_lever_arm_matches_a_rigid_body_finite_difference_at_constant_rate():
    """常角速度：v_B 与 a_B 必须与精确刚体运动的差分一致。

    这一支专门验证向心项 —— 角加速度为零，所以 a_B 里剩下的【只有】
    w x (w x r)。漏掉它的实现在这里必红。
    """
    rng = np.random.default_rng(11)
    omega = rng.normal(size=3)
    v_R, a_R = rng.normal(size=3), rng.normal(size=3)
    h = 1e-5

    def point(t):
        return (v_R * t + 0.5 * a_R * t * t) + _expm_skew(omega * t) @ R_BASE

    v_B, a_B = shift_twist_accel(v_R, omega, a_R, np.zeros(3), R_BASE)
    v_fd = (point(h) - point(-h)) / (2 * h)
    a_fd = (point(h) - 2 * point(0.0) + point(-h)) / (h * h)
    assert np.allclose(v_B, v_fd, atol=1e-7)
    assert np.allclose(a_B, a_fd, atol=1e-4)


def test_lever_arm_matches_a_rigid_body_finite_difference_with_angular_accel():
    """角加速度沿同一轴时旋转仍有闭式解，用它验证 alpha x r 这一项。"""
    axis = np.array([0.0, 0.0, 1.0])
    w0, alpha_scalar = 1.7, -2.3
    omega, alpha = w0 * axis, alpha_scalar * axis
    v_R, a_R = np.array([0.3, -0.2, 0.1]), np.array([-0.4, 0.6, 0.2])
    h = 1e-5

    def point(t):
        angle = w0 * t + 0.5 * alpha_scalar * t * t
        return (v_R * t + 0.5 * a_R * t * t) + _expm_skew(angle * axis) @ R_BASE

    v_B, a_B = shift_twist_accel(v_R, omega, a_R, alpha, R_BASE)
    v_fd = (point(h) - point(-h)) / (2 * h)
    a_fd = (point(h) - 2 * point(0.0) + point(-h)) / (h * h)
    assert np.allclose(v_B, v_fd, atol=1e-7)
    assert np.allclose(a_B, a_fd, atol=1e-4)
    # 两项都必须真的在起作用，否则这条测试是空的
    assert np.linalg.norm(np.cross(alpha, R_BASE)) > 1e-3
    assert np.linalg.norm(np.cross(omega, np.cross(omega, R_BASE))) > 1e-3


def test_centripetal_term_survives_at_constant_angular_rate():
    """等角速度、零角加速度时，加速度必须仍有 w x (w x r)。

    漏掉这一项的代码在任何静态或匀速直线测试里都是对的，只有转起来才错。
    """
    omega = np.array([0.0, 0.0, 3.0])
    v_B, a_B = shift_twist_accel(np.zeros(3), omega, np.zeros(3), np.zeros(3), R_BASE)
    expected = np.cross(omega, np.cross(omega, R_BASE))
    assert np.allclose(a_B, expected)
    assert np.linalg.norm(expected) > 1e-4, "这个算例的向心项太小，测不出东西"


def test_lever_arm_uses_the_base_extrinsic_not_the_end_marker_lever():
    """必须用 33.07 mm 的基座外参，不是 17.89 mm 的末端标记杠杆臂。

    标定输出把后者也叫 "lever"，而且它就是 arm_ee_from_link6 —— 两个键
    在文件里只隔十几行。用错的后果是每个角加速度分量都带 15.2 mm 的错力臂。
    """
    assert not np.allclose(np.linalg.norm(R_BASE), np.linalg.norm(T_EE))
    assert abs(np.linalg.norm(R_BASE) - 0.03307) < 1e-4
    assert abs(np.linalg.norm(T_EE) - 0.01789) < 1e-4

    omega = np.array([0.0, 0.0, 2.0])
    v_right, _ = shift_twist_accel(np.zeros(3), omega, np.zeros(3), np.zeros(3), R_BASE)
    v_wrong, _ = shift_twist_accel(np.zeros(3), omega, np.zeros(3), np.zeros(3), T_EE)
    assert np.linalg.norm(v_right - v_wrong) > 1e-3, "两个杠杆臂给出的速度必须可区分"


def test_extrinsics_yaml_still_carries_both_vectors_unswapped():
    """配置文件里的两个向量不许被互换或对齐成同一个值。"""
    path = os.path.join(os.path.dirname(__file__), "..", "..",
                        "task_space_smc", "config", "vicon_extrinsics.yaml")
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    base = np.asarray(data["arm_base_from_base_link"]["translation_m"], dtype=float)
    ee = np.asarray(data["arm_ee_from_link6"]["translation_m"], dtype=float)
    assert np.allclose(base, R_BASE, atol=1e-6)
    assert np.allclose(ee, T_EE, atol=1e-6)
    assert np.linalg.norm(base) > np.linalg.norm(ee)


# ---------------- 源时钟对齐

def test_clock_offset_uses_the_minimum_not_the_mean():
    """偏置取窗口最小值：最小延迟的那一帧才代表时钟差，均值代表的是延迟。"""
    aligner = SourceClockAligner(window_sec=100.0)
    aligner.offset(1000.000, 500.000)      # 延迟 0
    aligner.offset(1000.100, 500.000)      # 延迟 100 ms
    assert aligner.offset(1000.200, 500.000) == pytest.approx(500.0)


def test_offset_window_lets_the_estimate_follow_clock_drift():
    """双窗口交替，估计要能跟着时钟漂移走，而不是锁死在最早那一帧。"""
    aligner = SourceClockAligner(window_sec=1.0)
    aligner.offset(1000.0, 500.0)          # 偏置 500.0
    offset = None
    for k in range(1, 12):
        offset = aligner.offset(1000.0 + k, 499.0 + k)   # 真偏置漂到 501.0
    assert offset == pytest.approx(501.0)


def test_a_zero_stamp_is_reported_as_absent_not_as_time_zero():
    """stamp=0 必须当"没人填"，不能当 1970 年。

    这正是 vrpn_listener 的 twist/accel 现状：实测 5156/5156 条 stamp=0、
    frame_id=''。当成真时间戳会算出 56 年的数据年龄。
    """
    aligner = SourceClockAligner()
    assert aligner.source_time(0, 0) is None
    assert aligner.source_time(-1, 0) is None
    assert aligner.source_time(500, 250_000_000) == pytest.approx(500.25)


def test_alignment_falls_back_to_arrival_when_the_source_stamp_is_missing():
    aligner = SourceClockAligner()
    aligned = aligner.align(1234.5, 0, 0)
    assert not aligned.used_source
    assert aligned.sample_time == pytest.approx(1234.5)
    assert aligned.stamp_local == pytest.approx(1234.5)
    assert aligned.latency == 0.0


def test_alignment_reports_the_capture_instant_not_the_arrival_instant():
    """发布的时间戳是【拍摄时刻】，所以陈旧性判据才有意义。

    按到达时刻打戳会把一个在网上飘了 190 ms 的样本报成 0 ms 新鲜。
    """
    aligner = SourceClockAligner(window_sec=100.0)
    # 源时钟与本机时钟同速前进，只差一个常量偏置 500 s（实机上是 11.992 h）。
    for k in range(20):                                  # 低延迟样本，建立偏置
        arrival = 1000.0 + k * 0.008
        aligner.align(arrival, 500 + int(k * 0.008), int((k * 0.008 % 1) * 1e9))
    source_sec, source_nsec = 500, int(0.160 * 1e9)      # 源时刻 500.160
    aligned = aligner.align(1000.310, source_sec, source_nsec)   # 到货晚了 150 ms
    assert aligned.used_source
    assert aligned.latency == pytest.approx(0.150, abs=2e-3)


def test_source_timestamps_can_be_switched_off_for_comparison():
    aligner = SourceClockAligner(enabled=False)
    assert aligner.source_time(500, 0) is None


# ---------------- 跨通道配对

def test_channels_with_skewed_source_times_are_rejected_rather_than_paired():
    """错位超限就丢弃，绝不外推。

    配错帧的辅助通道是"看起来没问题的错数据"，正是整套时间戳工作要防的。
    """
    buffer = [ChannelSample(100.000, np.zeros(3), 100.0),
              ChannelSample(100.008, np.ones(3), 100.008)]
    assert pair_by_source_time(100.009, buffer, 0.004) is not None
    assert pair_by_source_time(100.050, buffer, 0.004) is None
    assert pair_by_source_time(100.050, buffer, 1.0) is not None


def test_pairing_picks_the_nearest_sample():
    buffer = [ChannelSample(100.000, np.array([0.0, 0, 0]), 100.0),
              ChannelSample(100.008, np.array([1.0, 0, 0]), 100.008),
              ChannelSample(100.016, np.array([2.0, 0, 0]), 100.016)]
    picked = pair_by_source_time(100.007, buffer, 0.004)
    assert picked is not None and picked.value[0] == 1.0


def test_pairing_an_empty_buffer_is_a_miss_not_a_crash():
    assert pair_by_source_time(100.0, [], 0.004) is None


# ---------------- 一致性监视

def test_consistency_disagreement_is_flagged_only_after_it_persists():
    """单帧抖动不许触发；持续不一致才判失败。

    本地差分带着桥的 40 ms 低通滞后，运动中任何紧的瞬时判据都是错的。
    这条检查要抓的是粗错（单位错、坐标系错、刚体错），不是精度。
    """
    monitor = DerivativeConsistencyMonitor(linear_tol=0.05, angular_tol=0.5,
                                           window_sec=0.5, fail_sec=2.0)
    good = np.zeros(3)
    for k in range(200):
        monitor.update(k * 0.005, good, good, good, good)
    assert not monitor.failed

    # 单帧大偏差：中位数不动，不该触发
    monitor.update(1.0, np.array([5.0, 0, 0]), good, good, good)
    assert not monitor.failed

    # 持续 2 s 的粗错：必须触发
    for k in range(1000):
        now = 2.0 + k * 0.005
        monitor.update(now, np.array([5.0, 0, 0]), good, np.array([9.0, 0, 0]), good)
    assert monitor.failed


def test_consistency_monitor_reports_the_medians_it_judges_on():
    monitor = DerivativeConsistencyMonitor()
    for k in range(50):
        d_lin, d_ang = monitor.update(k * 0.005, np.array([0.02, 0, 0]),
                                      np.zeros(3), np.array([0.1, 0, 0]),
                                      np.zeros(3))
    assert d_lin == pytest.approx(0.02, rel=1e-6)
    assert d_ang == pytest.approx(0.1, rel=1e-6)


# ---------------- 整帧装配

def _assembler(**kw):
    kw.setdefault("timeout_sec", 0.003)
    return FrameAssembler("pose", ("pose", "twist", "accel"), **kw)


def test_a_frame_is_released_when_the_last_channel_arrives():
    """收齐即放行，触发点是"齐了"而不是"pose 到了"。

    实测每帧【最后】到达的是 accel（2342/2342 恒定），但这里刻意不依赖那个
    顺序 —— 硬编码到达顺序会在 Vicon 侧改动后悄悄坏掉，而"齐没齐"不会。
    """
    fa = _assembler()
    # 先让三条通道都被认识（装配器只等【来过】的通道）
    for c in ("pose", "twist", "accel"):
        fa.submit(c, (5, 0), c, 1000.0)
    key = (5, 8021000)
    assert fa.submit("pose", key, "p", 1000.000) == []
    assert fa.submit("accel", key, "a", 1000.0001) == []
    out = fa.submit("twist", key, "t", 1000.0002)      # 顺序换了也照样齐
    assert len(out) == 1 and out[0].complete
    assert set(out[0].samples) == {"pose", "twist", "accel"}
    assert out[0].wait_sec == pytest.approx(0.0002, abs=1e-9)


def test_the_wait_is_only_the_arrival_spread_not_a_frame():
    """额外延迟必须是"同帧到达跨度"，不是一个帧周期。

    实测跨度中位 0.158 ms、p95 0.424 ms、max 1.232 ms；"整体延迟一帧"要付
    8.021 ms，在 200 Hz 控制环上是 1.6 个控制拍。差 50 倍，这就是选装配而
    不是延迟一帧的全部理由。
    """
    fa = _assembler()
    for c in ("pose", "twist", "accel"):
        fa.submit(c, (5, 0), c, 1000.0)
    key = (5, 8021000)
    fa.submit("pose", key, "p", 1000.0)
    fa.submit("twist", key, "t", 1000.000158)
    out = fa.submit("accel", key, "a", 1000.000158)
    assert out[0].wait_sec < 0.001
    assert out[0].wait_sec * 1e3 == pytest.approx(0.158, abs=0.01)


def test_a_channel_that_never_arrives_never_blocks():
    """从没来过的通道不计入齐全条件 —— 否则 stamp=0 会拖死每一帧。

    ``stamp=0`` 的通道被上游整条丢弃、永远不 submit。如果装配器仍然等它，
    每帧都要白等一个完成超时。
    """
    fa = _assembler()
    out = fa.submit("pose", (5, 0), "p", 1000.0)
    assert len(out) == 1 and out[0].wait_sec == 0.0
    assert fa.required() == {"pose"}


def test_a_missing_channel_is_released_on_timeout():
    """来过又断了的通道靠超时兜底，绝不卡住不发。"""
    fa = _assembler(timeout_sec=0.003)
    for c in ("pose", "twist"):
        fa.submit(c, (5, 0), c, 1000.0)
    key = (5, 8021000)
    assert fa.submit("pose", key, "p", 1000.0) == []      # 还在等 twist
    out = fa.flush_expired(1000.0031)
    assert len(out) == 1
    assert not out[0].complete and out[0].missing == ("twist",)
    assert fa.timeouts == 1


def test_a_long_dead_channel_leaves_the_required_set():
    """断很久就踢出必需集合，稳态回到零等待；恢复后自动重新计入。"""
    fa = _assembler(absent_frames_to_drop=3)
    for c in ("pose", "twist"):
        fa.submit(c, (5, 0), c, 1000.0)
    for k in range(1, 6):
        fa.submit("pose", (5, k), "p", 1000.0 + k)
        fa.flush_expired(1000.0 + k + 0.01)
    assert fa.required() == {"pose"}, "断了很久的通道还在必需集合里"
    fa.submit("twist", (5, 99), "t", 1100.0)
    assert "twist" in fa.required(), "通道恢复后必须重新计入"


def test_frames_are_released_in_source_time_order():
    """输出必须按源时间递增。乱序的状态流会让下游的差分变成负 dt。"""
    fa = _assembler(timeout_sec=0.003)
    for c in ("pose", "twist"):
        fa.submit(c, (5, 0), c, 1000.0)
    fa.submit("pose", (5, 1), "p1", 1000.0)          # 老帧还缺 twist
    out = fa.submit("pose", (5, 2), "p2", 1000.001)  # 新帧先到齐也不许插队
    assert out == []
    fa.submit("twist", (5, 1), "t1", 1000.002)
    out = fa.submit("twist", (5, 2), "t2", 1000.002)
    keys = [f.key for f in out]
    assert keys == sorted(keys), "帧的放行顺序不是按源时间递增"


def test_a_late_sample_does_not_resurrect_a_released_frame():
    """迟到样本不能给已放行的帧重新开一个 pending。

    否则会攒出一堆只有辅助通道、没有位姿的僵尸帧，最后全部超时。
    """
    fa = _assembler()
    fa.submit("pose", (5, 1), "p", 1000.0)           # 只有 pose，立即放行
    out = fa.submit("twist", (5, 1), "t", 1000.001)  # 迟到
    assert out == []
    assert len(fa._pending) == 0
    assert fa.dropped_anchorless == 0


def test_a_frame_without_the_anchor_is_dropped_not_published():
    """没有位姿的帧无法发布，超时后丢弃并计数，不能当成正常帧放出去。"""
    fa = _assembler(timeout_sec=0.003)
    fa.submit("twist", (5, 7), "t", 1000.0)
    out = fa.flush_expired(1000.01)
    assert out == []
    assert fa.dropped_anchorless == 1


# ---------------- 桥必须复用共享实现

def test_the_bridge_imports_the_shared_aligner_instead_of_redefining_it():
    """把"复用既有时间方案"变成结构性约束，而不是口头约定。

    twist/accel/arm_ee 三条新通道各自持有一个 SourceClockAligner。如果桥
    里再写一份滑窗最小值，四条流就会有四套相似但不同的时间语义 —— 那正是
    QA_record 那次事故的形状。
    """
    path = os.path.join(os.path.dirname(__file__), "..", "..",
                        "task_space_smc", "hw", "vicon_floating_base_bridge.py")
    with open(path, "r", encoding="utf-8") as handle:
        source = handle.read()
    assert "from task_space_smc.core.base_state_fusion import" in source
    assert "SourceClockAligner" in source
    assert "self._offset_current = math.inf" not in source, "桥里还留着自己的滑窗实现"
