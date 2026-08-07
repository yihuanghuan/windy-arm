"""末端反馈：位置以 Vicon 为准，姿态永远不用 Vicon，掉线不许跳。

【这个测试的存在理由】
改造前控制回路里的"实测末端"是算出来的：arm_base 动捕 + 编码器 FK，
`/vrpn/arm_ee/*` 根本没被订阅。改成用 arm_ee 之后有两个坑必须钉死：

1. **arm_ee 的姿态通道不可用。** 它相对 link6 差 114.39°（第一次标定）/
   113.54°（第二次）。常量本身重复到 0.85°，但扣掉常量后的残差一次 9.97°、
   一次 43.55° —— 单个固定旋转解释不了它。根因是 joint6 零位与标记簇滚转
   数学上不可分离。所以【任何模式下】姿态都必须来自 FK。

2. **Vicon 位置是迟到的。** 实测 mean 171 ms / p95 432 ms / max 964 ms，而
   控制环 200 Hz、等效位置增益 k·λ=144 每米。原始灌进去等于对五分之一秒前
   的末端位置做反应。所以默认走偏置修正：FK 走高频，Vicon 只修慢变偏置，
   稳态仍精确收敛到 Vicon。
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from task_space_smc.core.ee_estimator import (  # noqa: E402
    FK, FK_FALLBACK, FK_ONLY, FK_STALE, VICON, VICON_BIAS_CORRECTED,
    VICON_DIRECT, EeExtrinsic, EeFeedbackConfig, EeFeedbackEstimator,
    control_point_bias, control_point_jacobian, skew)

DT = 0.005
T_EE = np.array([0.0119, 0.0047, 0.0125])       # vicon_extrinsics.yaml 实测值, 17.9 mm


def _rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _estimator(**kwargs):
    cfg = EeFeedbackConfig(**kwargs)
    return EeFeedbackEstimator(cfg, EeExtrinsic(T_EE))


def _drive(est, ticks, vicon_offset, start=0.0, fk_position=None, stale_after=None):
    """跑 ``ticks`` 拍；Vicon 位置 = FK 控制点 + ``vicon_offset``。"""
    fk_position = np.zeros(3) if fk_position is None else fk_position
    R = np.eye(3)
    out = None
    for k in range(ticks):
        now = start + k * DT
        fresh = stale_after is None or k < stale_after
        p_ctrl = fk_position + R @ T_EE
        out = est.update(
            now, fk_position, R, np.zeros(3), np.zeros(3), DT,
            vicon_position=(p_ctrl + vicon_offset) if fresh else None,
            vicon_stamp=now if fresh else None)
    return out


# ---------------- 偏置修正的三条极限行为

def test_bias_corrected_converges_onto_vicon_in_steady_state():
    """稳态必须精确落在 Vicon 上 —— 这才叫"末端位置以 arm_ee 为准"。"""
    est = _estimator(mode=VICON_BIAS_CORRECTED, bias_bandwidth_hz=0.2,
                     bias_init="zero", max_bias_rate_m_s=1.0)
    offset = np.array([0.004, -0.002, 0.003])
    result = _drive(est, 4000, offset)              # 20 s = 25 个时间常数
    assert np.allclose(result.bias, offset, atol=1e-4)
    assert np.allclose(result.position, result.fk_position + offset, atol=1e-4)


def test_infinite_bandwidth_degenerates_to_vicon_direct():
    """带宽拉到极大就退化成直接用 Vicon。"""
    est = _estimator(mode=VICON_BIAS_CORRECTED, bias_bandwidth_hz=1.0e6,
                     bias_init="zero", max_bias_rate_m_s=1.0e6)
    offset = np.array([0.004, -0.002, 0.003])
    result = _drive(est, 5, offset)
    assert np.allclose(result.position, result.vicon_position, atol=1e-9)


def test_zero_bandwidth_degenerates_to_fk_only():
    """带宽为零则偏置永远不动，等价于纯 FK。"""
    est = _estimator(mode=VICON_BIAS_CORRECTED, bias_bandwidth_hz=0.0,
                     bias_init="zero")
    result = _drive(est, 500, np.array([0.004, -0.002, 0.003]))
    assert np.allclose(result.bias, np.zeros(3), atol=1e-12)
    assert np.allclose(result.position, result.fk_position, atol=1e-12)


# ---------------- 掉线与恢复

def test_bias_freezes_across_a_dropout_and_the_estimate_never_steps():
    """掉线 1 秒：偏置冻结、输出连续、degraded 置位。

    位置增益 k·λ=144，5 mm 的阶跃就是 0.72 m/s² 的加速度需求 —— 掉线不该
    变成一次猛冲。
    """
    est = _estimator(mode=VICON_BIAS_CORRECTED, bias_init="first_sample")
    offset = np.array([0.005, 0.0, 0.0])
    _drive(est, 200, offset)
    frozen = est.bias.copy()

    positions = []
    for k in range(200):                            # 1 s 无数据
        now = 1.0 + k * DT
        r = est.update(now, np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3), DT,
                       vicon_position=None, vicon_stamp=None)
        positions.append(r.position.copy())
    assert np.allclose(est.bias, frozen, atol=1e-15), "掉线期间偏置必须冻结"
    assert r.degraded, "掉线必须置 degraded，不能静默继续"
    assert "年龄" in r.degrade_reason, r.degrade_reason
    steps = np.linalg.norm(np.diff(np.array(positions), axis=0), axis=1)
    assert steps.max() < 1e-12, "掉线期间输出出现了阶跃"


def test_recovery_slews_at_the_configured_rate_instead_of_jumping():
    """恢复时按限速爬回去，绝不允许一步跳到位。"""
    est = _estimator(mode=VICON_BIAS_CORRECTED, bias_init="zero",
                     bias_bandwidth_hz=50.0, max_bias_rate_m_s=0.01,
                     innovation_reject_m=1.0)
    offset = np.array([0.02, 0.0, 0.0])              # 20 mm 的新偏置
    previous = est.bias.copy()
    for k in range(400):
        now = k * DT
        r = est.update(now, np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3), DT,
                       vicon_position=np.eye(3) @ T_EE + offset, vicon_stamp=now)
        moved = float(np.linalg.norm(est.bias - previous))
        assert moved <= 0.01 * DT + 1e-12, "偏置单拍移动 %.6f m，超过限速" % moved
        previous = est.bias.copy()
    assert np.allclose(est.bias, offset, atol=1e-4), "限速爬升最终仍要收敛"


def test_a_marker_swap_sized_innovation_is_rejected_not_absorbed():
    """300 mm 级的突跳是刚体错配，不是噪声：必须拒绝并要求保位。"""
    est = _estimator(mode=VICON_BIAS_CORRECTED, bias_init="first_sample",
                     innovation_reject_m=0.05, innovation_reject_hold_sec=0.1)
    _drive(est, 200, np.array([0.002, 0.0, 0.0]))
    sane = est.bias.copy()
    for k in range(100):
        now = 1.0 + k * DT
        r = est.update(now, np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3), DT,
                       vicon_position=np.eye(3) @ T_EE + np.array([0.3, 0.0, 0.0]),
                       vicon_stamp=now)
    assert np.allclose(est.bias, sane, atol=1e-12), "错配数据被吸收进了偏置"
    assert r.degraded and r.hold_required


# ---------------- 姿态：永不来自 Vicon

@pytest.mark.parametrize("mode", [FK_ONLY, VICON_DIRECT, VICON_BIAS_CORRECTED])
def test_orientation_never_comes_from_vicon_in_any_mode(mode):
    """三种模式下姿态都必须逐位等于 FK，【包括】配了旋转外参的时候。

    这是一条"不许被误开"的测试：arm_ee 姿态差 114°，且扣掉常量后的残差在
    两次标定间差 4 倍，任何把它接进控制的改动都必须先让这条红。
    """
    extrinsic = EeExtrinsic(T_EE, rotation=_rot_z(1.99))     # 故意给一个旋转外参
    est = EeFeedbackEstimator(EeFeedbackConfig(mode=mode), extrinsic)
    R_fk = _rot_z(0.37)
    r = est.update(0.0, np.zeros(3), R_fk, np.zeros(3), np.zeros(3), DT,
                   vicon_position=R_fk @ T_EE, vicon_stamp=0.0,
                   vicon_rotation=_rot_z(2.4))
    assert np.array_equal(r.rotation, R_fk)


def test_orientation_residual_is_logged_but_not_used():
    """姿态残差要被记录下来（验收要看），但只是记录。"""
    est = _estimator(mode=VICON_BIAS_CORRECTED)
    R_fk = np.eye(3)
    r = est.update(0.0, np.zeros(3), R_fk, np.zeros(3), np.zeros(3), DT,
                   vicon_position=T_EE, vicon_stamp=0.0,
                   vicon_rotation=_rot_z(np.radians(114.39)))
    assert np.isclose(np.degrees(r.orientation_residual_rad), 114.39, atol=1e-6)
    assert np.array_equal(r.rotation, R_fk)


# ---------------- 速度选源：vicon 这条路必须真的用上 Vicon 的值

def _velocity_estimator(**kwargs):
    """默认参数下的估计器；innovation 门放开，专注看速度那一路。"""
    kwargs.setdefault("mode", VICON_BIAS_CORRECTED)
    kwargs.setdefault("bias_init", "first_sample")
    return _estimator(**kwargs)


def _tick(est, now, v_vicon=None, w_vicon=None, stamp=None, v_fk=None,
          w_fk=None):
    """跑一拍：FK 控制点与 Vicon 位置重合，只让速度成为变量。"""
    v_fk = np.zeros(3) if v_fk is None else np.asarray(v_fk, dtype=float)
    w_fk = np.zeros(3) if w_fk is None else np.asarray(w_fk, dtype=float)
    R = np.eye(3)
    # FK 控制点速度 = v_fk + w_fk x (R t)，把它当作 Vicon "应该"报出来的值
    return est.update(
        now, np.zeros(3), R, v_fk, w_fk, DT,
        vicon_position=R @ T_EE,
        vicon_stamp=now if stamp is None else stamp,
        vicon_linear_velocity=v_vicon,
        vicon_angular_velocity=w_vicon)


def test_velocity_source_vicon_actually_uses_the_vicon_value():
    """``velocity_source: vicon`` 必须【真的换掉那个数】。

    这条路以前是个空壳：只往 source 字符串上拼了 "+vicon_velocity"，
    linear 仍然是 v_fk。配置看起来生效了、数据里也有痕迹，但控制回路吃的
    还是 FK —— 这是最难发现的一类故障，所以这条测试盯的就是【值本身】。
    """
    est = _velocity_estimator(velocity_source=VICON,
                              velocity_cross_check_tol=1.0e6)
    v_vicon = np.array([0.31, -0.12, 0.07])
    r = _tick(est, 0.0, v_vicon=v_vicon)
    assert np.allclose(r.linear_velocity, v_vicon, atol=1e-15)
    assert r.linear_velocity_source == VICON
    # 对照组：默认配置下同样的输入必须【原样不用】
    fk_est = _velocity_estimator()
    r_fk = _tick(fk_est, 0.0, v_vicon=v_vicon)
    assert np.allclose(r_fk.linear_velocity, np.zeros(3), atol=1e-15)
    assert r_fk.linear_velocity_source == FK


def test_angular_velocity_source_is_a_separate_switch():
    """角速度是【独立开关】，不跟着线速度走。

    它多一层风险：实测 ||vel_quat|| = 3.850（不是单位四元数）、vel_quat[0]
    每帧精确为 0，暗示 listener 里那套 ω = axis·angle/vel_quat_dt 的换算可能
    整个是错的。一个开关同时翻两条通道，等于用一条未验证的通道去污染另一条。
    """
    est = _velocity_estimator(velocity_source=VICON,
                              angular_velocity_source=FK,
                              velocity_cross_check_tol=1.0e6,
                              angular_cross_check_tol=1.0e6)
    r = _tick(est, 0.0, v_vicon=np.array([0.3, 0.0, 0.0]),
              w_vicon=np.array([0.0, 0.0, 1.7]))
    assert r.linear_velocity_source == VICON
    assert r.angular_velocity_source == FK
    assert np.allclose(r.angular_velocity, np.zeros(3), atol=1e-15)


def test_defaults_stay_on_fk_for_both_channels():
    """默认必须是 fk —— Vicon 速度的尺度和坐标系还没有真实运动的数据可验。

    这条测试的作用是让"顺手把默认翻过去"这个改动先变红。
    """
    cfg = EeFeedbackConfig()
    assert cfg.velocity_source == FK
    assert cfg.angular_velocity_source == FK


def test_a_stale_vicon_sample_falls_back_to_fk_and_counts():
    """超过 max_age_sec 就回退 FK。

    Vicon 位置实测 mean 171 ms / p95 432 ms / max 964 ms 延迟。用一个 200 ms
    前的速度去做 200 Hz 的阻尼项，比直接用 FK 差 —— 所以宁可回退。
    """
    est = _velocity_estimator(velocity_source=VICON, max_age_sec=0.15,
                              velocity_cross_check_tol=1.0e6)
    _tick(est, 0.0, v_vicon=np.array([0.3, 0.0, 0.0]))       # 先建立 bias
    fresh = _tick(est, 1.0, v_vicon=np.array([0.3, 0.0, 0.0]), stamp=1.0)
    assert fresh.linear_velocity_source == VICON

    stale = _tick(est, 2.0, v_vicon=np.array([0.3, 0.0, 0.0]), stamp=1.7)
    assert stale.vicon_age == pytest.approx(0.3)
    assert stale.linear_velocity_source == FK_STALE
    assert np.allclose(stale.linear_velocity, np.zeros(3), atol=1e-15)
    assert est.linear_velocity_stale >= 1, "回退必须被计数"


def test_no_vicon_velocity_at_all_falls_back_to_fk():
    """桥没发速度（比如 twist 通道 stamp=0 被整条丢弃）也要回退，不是崩。"""
    est = _velocity_estimator(velocity_source=VICON,
                              angular_velocity_source=VICON)
    r = _tick(est, 0.0, v_vicon=None, w_vicon=None)
    assert r.linear_velocity_source == FK_STALE
    assert r.angular_velocity_source == FK_STALE


def test_the_cross_check_rejects_an_inconsistent_angular_rate_and_counts():
    """与 FK 粗量级不符的角速度必须被拒，并计数。

    这道闸的意义在坑 #3：vel_quat 的语义还没被验证过，换算可能整个是错的。
    拒绝 + 计数是唯一能同时做到"能用"和"错了会被发现"的做法 —— 一刀切禁用
    就永远验证不了它，盲信则会把一个错的量直接乘进 SMC 的阻尼项。
    """
    est = _velocity_estimator(angular_velocity_source=VICON,
                              angular_cross_check_tol=0.5,
                              angular_cross_check_floor_rad_s=0.2)
    # FK 说 0.3 rad/s，Vicon 报 3.85 rad/s（≈ ||vel_quat|| 那个量级）
    w_fk = np.array([0.0, 0.0, 0.3])
    r = _tick(est, 0.0, w_vicon=np.array([0.0, 0.0, 3.85]), w_fk=w_fk)
    assert r.angular_velocity_source == FK_FALLBACK
    assert np.allclose(r.angular_velocity, w_fk, atol=1e-15)
    assert est.angular_velocity_rejects == 1


def test_the_cross_check_passes_a_consistent_value():
    """量级相符就放行 —— 这道闸只抓粗错，不是精度判据。

    容差刻意宽：FK 与 Vicon 之间本来就有传输延迟造成的相位差，任何按精度
    定的阈值在运动时都会把好数据也拒掉。
    """
    est = _velocity_estimator(angular_velocity_source=VICON,
                              angular_cross_check_tol=0.5,
                              angular_cross_check_floor_rad_s=0.2)
    w_fk = np.array([0.0, 0.0, 1.0])
    w_vicon = np.array([0.0, 0.0, 1.4])              # 差 0.4 <= 0.5 * 1.0
    r = _tick(est, 0.0, w_vicon=w_vicon, w_fk=w_fk)
    assert r.angular_velocity_source == VICON
    assert np.allclose(r.angular_velocity, w_vicon, atol=1e-15)
    assert est.angular_velocity_rejects == 0


def test_the_floor_keeps_the_gate_meaningful_at_a_standstill():
    """静止时靠 floor 兜底，否则 tol * 0 = 0 会把任何值都判成"不符"。

    保持任务里 FK 速度本来就接近零，没有 floor 这道闸在最常见的工况下
    永远返回"拒绝"，等于把 vicon 这条路悄悄禁用掉。
    """
    est = _velocity_estimator(velocity_source=VICON,
                              velocity_cross_check_tol=0.5,
                              velocity_cross_check_floor_m_s=0.05)
    small = np.array([0.02, 0.0, 0.0])               # 20 mm/s < 0.5 * 50 mm/s
    r = _tick(est, 0.0, v_vicon=small, v_fk=np.zeros(3))
    assert r.linear_velocity_source == VICON
    # 但 0.3 m/s 就远超 floor 允许的范围，仍然要被拒
    r2 = _tick(est, DT, v_vicon=np.array([0.3, 0.0, 0.0]), v_fk=np.zeros(3))
    assert r2.linear_velocity_source == FK_FALLBACK


def test_a_rejected_position_innovation_also_rejects_the_velocity():
    """位置新息被拒（刚体错配）时，速度也必须一起拒。

    50 mm 级的残差说明这【根本不是同一个刚体】——位置拒了、速度还照用是
    自相矛盾：错刚体报的速度一样是错的。
    """
    est = _velocity_estimator(velocity_source=VICON, innovation_reject_m=0.05,
                              velocity_cross_check_tol=1.0e6)
    R = np.eye(3)
    _tick(est, 0.0, v_vicon=np.array([0.1, 0.0, 0.0]))       # 建立 bias
    r = est.update(1.0, np.zeros(3), R, np.zeros(3), np.zeros(3), DT,
                   vicon_position=R @ T_EE + np.array([0.3, 0.0, 0.0]),
                   vicon_stamp=1.0,
                   vicon_linear_velocity=np.array([0.1, 0.0, 0.0]))
    assert r.linear_velocity_source == FK_STALE
    assert np.allclose(r.linear_velocity, np.zeros(3), atol=1e-15)


def test_both_velocity_channels_are_logged_side_by_side():
    """FK 与 Vicon 两路都要逐拍留下来。

    Vicon 速度的尺度和坐标系【在有真实运动的数据出现之前无法验证】——
    只有把两列摆在一起看过的人，才有资格决定要不要翻默认值。
    """
    est = _velocity_estimator(velocity_source=VICON,
                              velocity_cross_check_tol=1.0e6)
    v_vicon = np.array([0.31, -0.12, 0.07])
    v_fk = np.array([0.30, -0.11, 0.06])
    r = _tick(est, 0.0, v_vicon=v_vicon, v_fk=v_fk)
    assert np.allclose(r.vicon_linear_velocity, v_vicon)
    assert np.allclose(r.fk_linear_velocity, v_fk)
    assert r.fk_angular_velocity is not None


def test_an_unknown_velocity_source_is_a_hard_error():
    """打错的来源名必须当场报错，不许静默走 FK 分支。

    静默回退是"配了却没生效"这类只有等数据回来才发现的故障。
    """
    with pytest.raises(ValueError):
        EeFeedbackEstimator(
            EeFeedbackConfig(velocity_source="vikon"), EeExtrinsic(T_EE))
    with pytest.raises(ValueError):
        EeFeedbackEstimator(
            EeFeedbackConfig(angular_velocity_source="vikon"),
            EeExtrinsic(T_EE))


@pytest.mark.parametrize("mode", [FK_ONLY, VICON_DIRECT, VICON_BIAS_CORRECTED])
def test_orientation_is_still_fk_when_vicon_velocity_is_in_use(mode):
    """接了速度之后，姿态【仍然】三种模式下都逐位等于 FK。

    这是把速度接进来时最容易顺手破掉的那条红线：arm_ee 姿态差 114°，扣掉
    常量后的残差在两次标定间差 4 倍（9.97° vs 43.55°），根因是 joint6 零位
    与标记簇滚转数学上不可分离 —— 它永远不能进控制。
    """
    extrinsic = EeExtrinsic(T_EE, rotation=_rot_z(1.99))
    est = EeFeedbackEstimator(
        EeFeedbackConfig(mode=mode, velocity_source=VICON,
                         angular_velocity_source=VICON,
                         velocity_cross_check_tol=1.0e6,
                         angular_cross_check_tol=1.0e6),
        extrinsic)
    R_fk = _rot_z(0.37)
    r = est.update(0.0, np.zeros(3), R_fk, np.zeros(3), np.zeros(3), DT,
                   vicon_position=R_fk @ T_EE, vicon_stamp=0.0,
                   vicon_rotation=_rot_z(2.4),
                   vicon_linear_velocity=np.array([0.2, 0.1, -0.3]),
                   vicon_angular_velocity=np.array([0.5, -0.4, 0.9]))
    assert np.array_equal(r.rotation, R_fk)
    # 且速度确实换成了 Vicon 的 —— 否则这条测试会因为"什么都没接上"而空过
    assert r.angular_velocity_source == VICON
    assert np.allclose(r.angular_velocity, [0.5, -0.4, 0.9])


# ---------------- 控制点几何

def test_control_point_jacobian_matches_a_finite_difference():
    """J_p' = J_p - skew(r)J_w 必须与有限差分一致。

    符号或坐标系搞错时它会悄悄退化成 J_p，而 17.9 mm 在静态检查里看不出来，
    一旦腕部转起来就看得出来了。
    """
    rng = np.random.default_rng(7)
    for _ in range(200):
        J = rng.normal(size=(6, 6))
        angle = rng.uniform(-np.pi, np.pi)
        R = _rot_z(angle)
        J_ctrl = control_point_jacobian(J, R, T_EE)
        # 控制点速度 = 末端速度 + w x (R t)
        qd = rng.normal(size=6)
        twist = J @ qd
        expected = twist[:3] + np.cross(twist[3:], R @ T_EE)
        assert np.allclose(J_ctrl[:3] @ qd, expected, atol=1e-12)
        assert np.allclose(J_ctrl[3:], J[3:])


def test_control_point_bias_keeps_the_centripetal_term():
    """b_p' 必须含 w x (w x r)：它在【等角速度】下依然存在，漏掉了静态测不出来。"""
    omega = np.array([0.0, 0.0, 2.0])
    R = np.eye(3)
    out = control_point_bias(np.zeros(6), omega, R, T_EE)
    expected = np.cross(omega, np.cross(omega, T_EE))
    assert np.allclose(out[:3], expected)
    assert np.linalg.norm(expected) > 0.0
    # 角加速度项
    out2 = control_point_bias(np.array([0, 0, 0, 0, 0, 3.0]), np.zeros(3), R, T_EE)
    assert np.allclose(out2[:3], np.cross(np.array([0, 0, 3.0]), T_EE))


def test_skew_is_the_cross_product_operator():
    rng = np.random.default_rng(3)
    a, b = rng.normal(size=3), rng.normal(size=3)
    assert np.allclose(skew(a) @ b, np.cross(a, b))


# ---------------- 构造期与捕获期的守卫

def test_a_vicon_mode_without_an_extrinsic_is_a_hard_error():
    """外参缺失必须当场报错，不许静默用单位阵。

    与 vicon_floating_base.launch.py 的 load_base_extrinsic 同一条原则：
    一个"还能跑"的错世界系，正是只会在机器人身上暴露的那类故障。
    """
    with pytest.raises(ValueError):
        EeFeedbackEstimator(EeFeedbackConfig(mode=VICON_BIAS_CORRECTED), None)
    # fk_only 不需要外参
    EeFeedbackEstimator(EeFeedbackConfig(mode=FK_ONLY), None)


def test_an_unknown_mode_is_rejected():
    with pytest.raises(ValueError):
        EeFeedbackEstimator(EeFeedbackConfig(mode="vicon_maybe"), EeExtrinsic(T_EE))


def test_capture_is_refused_until_a_vicon_sample_has_arrived():
    """没收到过 arm_ee 就捕获保持目标 = 静默退化成 fk_only 并把目标钉错地方。"""
    est = _estimator(mode=VICON_BIAS_CORRECTED)
    assert not est.ready_for_capture()
    _drive(est, 1, np.array([0.001, 0.0, 0.0]))
    assert est.ready_for_capture()


def test_first_sample_init_makes_the_bias_converged_before_capture():
    """bias_init=first_sample 时，第一拍之后偏置就已就位。

    否则捕获落在 FK 点上，随后偏置在几秒内长到真值，控制器会悄悄把臂推走
    —— 而 test_vicon_hold_demo 已经写明 1 cm 初始残差就是 1.5~1.9 m/s²。
    """
    est = _estimator(mode=VICON_BIAS_CORRECTED, bias_init="first_sample")
    offset = np.array([0.0031, -0.0012, 0.0007])
    r = _drive(est, 1, offset)
    assert np.allclose(est.bias, offset, atol=1e-15)
    assert np.allclose(r.position, r.vicon_position, atol=1e-15)
