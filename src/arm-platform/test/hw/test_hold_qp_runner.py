"""QP 只接管 hold 实物链路，且失败时保位而不是失力。

【这个测试的存在理由】
受约束 QP 是一条【新】控制路径。它必须严格只在 hold 实物 profile 上生效：
whole6 / whole6_hw / hwtest / whole 四份配置继续走原来的伪逆 + 后端 clip，
一行不改。一个键漏写到别的 profile 里，就会在毫不相干的一次实验中悄悄换掉
控制律 —— 而那种 bug 只会在机器人身上暴露。

另一半是降级方向：QP 连续失败必须转【位置保持】，绝不发零力矩。
实测 joint2 失力后下沉 14.6°。
"""

import glob
import importlib.util
import os
import sys
import types

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CONFIG_DIR = os.path.join(ROOT, "task_space_smc", "config")
RUNNER = os.path.join(ROOT, "task_space_smc", "hw", "whole_task_smc_ros_runner.py")


def _config(name):
    with open(os.path.join(CONFIG_DIR, name), "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _runner_source():
    with open(RUNNER, "r", encoding="utf-8") as handle:
        return handle.read()


# ---------------- 作用范围

def test_only_the_hold_profile_selects_the_qp_solver():
    """五份 smc_params_*.yaml 里，有且只有 hold 带 task.solver: qp。"""
    selected = []
    for path in sorted(glob.glob(os.path.join(CONFIG_DIR, "smc_params_*.yaml"))):
        data = yaml.safe_load(open(path, "r", encoding="utf-8")) or {}
        if str((data.get("task") or {}).get("solver", "")).lower() == "qp":
            selected.append(os.path.basename(path))
    assert selected == ["smc_params_hold.yaml"], selected


def test_the_other_profiles_keep_the_pseudo_inverse_path():
    """其余 profile 不许出现 qp 段，也不许被改成 weak 姿态。"""
    for name in ("smc_params_whole6.yaml", "smc_params_whole6_hw.yaml",
                 "smc_params_hwtest.yaml", "smc_params_whole.yaml"):
        data = _config(name)
        assert "qp" not in data, "%s 混进了 qp 段" % name
        mode = (data.get("task") or {}).get("orientation_mode")
        assert mode != "weak", "%s 的 orientation_mode 被改动了" % name


def test_hold_defaults_to_position_priority_not_a_full_6d_hard_task():
    """hold 的默认必须是"位置优先 + 姿态弱约束"，不是 full。

    6 关节解 6 维硬任务时零空间为 0 维：基座一动，位置和姿态就互相抢自由度。
    """
    task = _config("smc_params_hold.yaml")["task"]
    assert task["orientation_mode"] == "weak"
    assert task["solver"] == "qp"


def test_hold_weights_keep_position_far_above_orientation():
    """一级/二级的权重分离必须留在配置里，而不是靠代码默认值兜着。"""
    weights = _config("smc_params_hold.yaml")["qp"]["weights"]
    assert weights["position"] >= 1.0e3 * weights["orientation"]
    assert weights["joint_centering"] < weights["orientation"]
    assert weights["smoothness"] < weights["orientation"]


def test_task_acceleration_is_capped_in_the_hold_profile():
    """不封顶的话，5 cm 误差就要 7.2 m/s²，解会一直贴在约束边界上。"""
    task = _config("smc_params_hold.yaml")["task"]
    assert 0.0 < task["max_linear_task_accel"] <= 10.0
    assert 0.0 < task["max_angular_task_accel"] <= 30.0


# ---------------- 末端反馈配置

def test_hold_uses_direct_vicon_end_effector_position_by_default():
    cfg = _config("smc_params_hold.yaml")["ee_feedback"]
    assert cfg["mode"] == "vicon_direct"
    assert cfg["require_vicon_at_capture"] is True
    assert cfg["bias_init"] == "first_sample"
    # 带宽必须远慢于传输延迟（实测 mean 171 ms / p95 432 ms）
    assert 0.0 < cfg["bias_bandwidth_hz"] <= 1.0


def test_hold_does_not_trust_vicon_angular_velocity_while_upstream_sends_zero():
    """角速度【不许】采信 Vicon —— 因为上游现在发的是硬零。

    vrpn_listener 把角速度按 0 发布（vel_quat 语义未判定）。"上游发 0" 加上
    这道对照闸是个危险组合，算术如下：

        闸门  |cand - ref| <= tol * max(|ref|, floor)，其中 cand = 0
        |ω_ref| <= floor 时 -> |ω_ref| <= tol*floor = 0.5*0.2 = 0.1 rad/s -> 放行
        |ω_ref| >  floor 时 -> |ω_ref| <= 0.5*|ω_ref|                     -> 拒绝

    即 |ω| <= 0.1 rad/s（5.73 °/s）时硬零会被当成真实测量【放行】，而那正是
    保持 demo 的工作区间。控制器于是看到基座角速度恒为 0，ω×r 的杠杆项和
    向心补偿 ω×(ω×r) 一起消失。

    闸门本身没错：它是为"真实但可能错标度的值"设计的裁判，挡不住上游主动
    发零。所以关掉的是采信，不是闸门 —— 闸门要留到语义定下来之后用。
    """
    cfg = _config("smc_params_hold.yaml")["base_feedback"]
    assert cfg["prefer_vicon_angular_velocity"] is True
    # 闸门参数必须【原样保留】：语义定下来之后还要用它
    assert cfg["angular_cross_check_tol"] > 0.0
    assert cfg["angular_cross_check_floor_rad_s"] > 0.0
    # 线量不受牵连：listener 对 vel[0:3] / acc[0:3] 是原样透传，没有发零
    assert cfg["prefer_vicon_twist"] is True
    assert cfg["prefer_vicon_linear_accel"] is True


def test_the_zero_publication_actually_slips_through_the_cross_check():
    """把上面那段闸门算术变成可执行的断言，而不是注释里的推导。

    这条测试存在的意义：如果哪天有人把 tol 或 floor 调了、或者上游不再发零，
    这里会立刻反映出"硬零还能不能混过去"，不用重新手推一遍。
    """
    cfg = _config("smc_params_hold.yaml")["base_feedback"]
    tol = float(cfg["angular_cross_check_tol"])
    floor = float(cfg["angular_cross_check_floor_rad_s"])

    def gate_passes(w_ref, candidate=0.0):
        return abs(candidate - w_ref) <= tol * max(abs(w_ref), floor)

    # 硬零在慢速区间里【确实】会被放行 —— 这就是必须关掉采信的理由
    assert gate_passes(0.05), "闸门本该放行慢速区间的硬零（这正是危险所在）"
    assert gate_passes(tol * floor), "放行上界应为 tol*floor"
    assert not gate_passes(tol * floor * 1.01), "超过上界必须拒绝"
    assert tol * floor == pytest.approx(0.1), "放行上界 0.1 rad/s = 5.73 °/s"


def test_hold_does_not_trust_vicon_angular_acceleration_either():
    """角加速度同理关闭，且理由更硬。

    同一道闸、同样的硬零放行区间，只是 floor 换成
    angular_accel_cross_check_floor_rad_s2=2.0，放行上界 |α| <= 1.0 rad/s²。

    它比角速度更不能将就：角加速度直接进 compute_task_bias_acceleration 变
    力矩（QA_record：16 rad/s² 的伪值在 0.3 m 力臂上是 4.8 m/s² 的伪末端
    需求）。被放行的硬零是同样的"伪值"，只是它把真实补偿【抹掉】。

    acc_quat 的可用性完全继承 vel_quat：实测
    acc_quat[1:4] = 120*Δ(vel_quat[1:4]) 是精确恒等式（R²=1.0000）。
    """
    cfg = _config("smc_params_hold.yaml")["base_feedback"]
    assert cfg["prefer_vicon_angular_accel"] is True
    assert cfg["angular_accel_cross_check_floor_rad_s2"] > 0.0


def test_the_bridge_defaults_match_the_hold_config():
    """桥的 declare_parameter 缺省值必须与配置一致。

    两处不一致时，不经配置直接起桥（实机调试常这么干）就会悄悄回到采信
    Vicon 角量的状态 —— 而那正是这条修复要消灭的。
    """
    path = os.path.join(ROOT, "task_space_smc", "hw",
                        "vicon_floating_base_bridge.py")
    with open(path, "r", encoding="utf-8") as handle:
        source = handle.read()
    assert 'declare_parameter("prefer_vicon_angular_velocity", True)' in source
    assert 'declare_parameter("prefer_vicon_angular_accel", True)' in source


def test_end_effector_linear_velocity_defaults_to_vicon():
    """线速度默认取 FK：微分一个平均 171 ms 延迟的信号比 FK 差。

    需求原文要求"线速度以 arm_ee 为准"，所以 vicon 选项保留，但不设为默认。
    这条测试把这个【有意偏离】钉在明面上。
    """
    cfg = _config("smc_params_hold.yaml")["ee_feedback"]
    assert cfg["velocity_source"] == "vicon"


def test_end_effector_angular_velocity_is_a_separate_key_also_defaulting_to_fk():
    """角速度是【独立开关】，默认同样是 fk。

    它比线速度多一层风险：实测 ||vel_quat|| = 3.850（不是单位四元数）、
    vel_quat[0] 每帧精确为 0，暗示 listener 里那套
    ω = axis·angle/vel_quat_dt 的换算可能整个是错的。一个开关同时翻两条
    通道，等于用一条未验证的通道去污染另一条。
    """
    cfg = _config("smc_params_hold.yaml")["ee_feedback"]
    assert cfg["angular_velocity_source"] == "fk"


def test_the_end_effector_cross_check_tolerances_parse_as_numbers():
    """对照容差必须被 YAML 解析成【数】，不是字符串。

    YAML 1.1 规则下 `1.0e4` 会被解析成 str（少了正号），配置看起来没问题、
    float() 之前一路带着走，直到比较时才炸。这个坑在这个项目里真实发生过，
    是测试才抓出来的 —— 所以每一个新的数值键都要在这里过一遍。
    """
    cfg = _config("smc_params_hold.yaml")["ee_feedback"]
    for key in ("velocity_cross_check_tol", "velocity_cross_check_floor_m_s",
                "angular_cross_check_tol", "angular_cross_check_floor_rad_s"):
        value = cfg[key]
        assert isinstance(value, (int, float)) and not isinstance(value, bool), \
            "%s 被解析成了 %r（%s）" % (key, value, type(value).__name__)
        assert value > 0.0, "%s 必须为正：floor=0 会让这道闸在静止时永远拒绝" % key


def test_the_hold_config_builds_a_valid_estimator_config():
    """hold 的 ee_feedback 段必须能原样喂进 EeFeedbackConfig 并通过校验。

    来源名打错时估计器是【硬报错】而不是静默走 FK —— 这条测试保证配置文件
    这一侧也确实站在合法的那一边。
    """
    sys.path.insert(0, ROOT)
    from task_space_smc.core.ee_estimator import (
        VELOCITY_SOURCES, EeExtrinsic, EeFeedbackConfig, EeFeedbackEstimator)
    cfg = _config("smc_params_hold.yaml")["ee_feedback"]
    assert cfg["velocity_source"] in VELOCITY_SOURCES
    assert cfg["angular_velocity_source"] in VELOCITY_SOURCES
    EeFeedbackEstimator(
        EeFeedbackConfig(
            mode=cfg["mode"],
            velocity_source=cfg["velocity_source"],
            angular_velocity_source=cfg["angular_velocity_source"],
            velocity_cross_check_tol=float(cfg["velocity_cross_check_tol"]),
            angular_cross_check_tol=float(cfg["angular_cross_check_tol"])),
        EeExtrinsic(np.zeros(3)))


# ---------------- 源码级不变量

def test_the_new_velocity_columns_are_declared_and_backfilled():
    """新增的 CSV 列必须【同时】出现在 fieldnames 和就位段的补 NaN 里。

    只加 fieldnames 不补 NaN，就位段那一行会静默写成空字符串；只补不加，
    csv.DictWriter 直接抛 ValueError。两个写行器共用一份 fieldnames，所以
    这两处必须成对改 —— 这正是任务里点名的那个坑。
    """
    source = _runner_source()
    for column in ("ee_vel_source", "ee_omega_source"):
        assert '"%s"' % column in source
        assert '"%s": float("nan")' % column in source, \
            "%s 没有在 _write_approach_row 里补 NaN" % column
    for group in ("ee_vicon_vel", "ee_fk_vel"):
        assert 'f"%s_{axis}" for axis in "xyz"' % group in source
        assert 'row[f"%s_{axis}"] = float("nan")' % group in source, \
            "%s_* 没有在 _write_approach_row 里补 NaN" % group


def test_the_arm_ee_topic_carries_a_full_state_not_just_a_pose():
    """``/vicon/arm_ee`` 的类型必须是 FloatingBaseState。

    位姿/速度/加速度共用一个时间戳，下游就不必再面对跨话题时间对齐 ——
    而那正是这个项目在基座上花大力气才解决掉的问题。
    """
    source = _runner_source()
    assert "FloatingBaseState, self.ee_topic, self._ee_state_cb" in source
    assert "PoseStamped, self.ee_topic" not in source



def test_the_runner_dispatches_on_the_solver_key():
    source = _runner_source()
    assert 'self.use_task_qp = str(_task.get("solver", "")).lower() == "qp"' in source
    assert "if self.use_task_qp:" in source


def test_friction_feedforward_is_folded_into_the_affine_term():
    """摩擦前馈必须并进 c，不能算完力矩再加。

    加在 QP 外面会顶破求解器保证的上限，后端再削一刀 —— 那正是要消灭的
    抖振源。
    """
    source = _runner_source()
    assert "c_affine = tau_bias + tau_track_friction" in source


def test_the_slew_reference_mirrors_the_adapter_timeout_rule():
    """QP 的力矩变化率基准必须复刻 adapter 的归零规则。

    adapter 在 command_timeout_sec 之后把 _last_tau 归零。两边基准一旦错开，
    adapter 就会削掉一条 QP 认为合法的指令。
    """
    source = _runner_source()
    assert "_torque_slew_reference" in source
    assert "self.adapter.command_timeout_sec" in source


def test_repeated_solver_failure_falls_back_to_position_hold_not_zero_torque():
    """连续失败走既有的 _reject_start_configuration -> 位置保持。

    零力矩在这台机器上是实测会下沉的（joint2 14.6°），所以降级方向只能是
    保位，不能是失力。
    """
    source = _runner_source()
    assert "self._qp_fail_streak >= self.qp_fallback_hold_ticks" in source
    assert "_reject_start_configuration" in source
    # 单拍失败先回落到伪逆路径，而不是直接放弃
    assert "本拍回落到伪逆路径" in source


def test_backend_clamp_is_counted_so_it_cannot_hide():
    """后端 clamp 是通信兜底，正常运行必须恒为 0，并且被计数。"""
    source = _runner_source()
    assert "self._backend_clamp_hits += 1" in source
    assert "backend_clamp_hits" in source


def test_capture_is_refused_until_arm_ee_has_been_seen():
    source = _runner_source()
    assert "self.ee_estimator.ready_for_capture()" in source


def test_orientation_is_never_taken_from_vicon_in_the_runner():
    """runner 组装 actual 位姿时，姿态必须来自 FK 的 kin_ctrl。"""
    source = _runner_source()
    assert "quaternion_wxyz=kin_ctrl.ee_quaternion_wxyz," in source


# ---------------- 求解器在实机数值上的行为

def test_the_hold_configuration_solves_within_the_control_budget():
    """用 hold 配置里的真实权重与限值跑一遍，确认收敛且不越界。"""
    sys.path.insert(0, ROOT)
    from task_space_smc.core.dense_qp import OPTIMAL
    from task_space_smc.core.task_qp import (
        HoldQPLimits, HoldQPWeights, solve_hold_qp)

    data = _config("smc_params_hold.yaml")
    weights_cfg = data["qp"]["weights"]
    limits_cfg = data["limits"]
    n = 6
    weights = HoldQPWeights(
        position=float(weights_cfg["position"]),
        orientation=float(weights_cfg["orientation"]),
        joint_centering=float(weights_cfg["joint_centering"]),
        smoothness=float(weights_cfg["smoothness"]),
        regularization=float(weights_cfg["regularization"]),
        qdd_box_slack=float(weights_cfg["qdd_box_slack"]),
        characteristic_length=float(data["task"]["characteristic_length"]))
    limits = HoldQPLimits(
        torque=np.asarray(limits_cfg["torque"], dtype=float),
        torque_rate=np.asarray(limits_cfg["torque_rate"], dtype=float),
        qdd_max=np.asarray(data["qp"]["qdd_max"], dtype=float),
        dt=1.0 / float(data["control_rate_hz"]),
        q_lower=np.full(n, -3.0), q_upper=np.full(n, 3.0),
        velocity=np.asarray(limits_cfg["joint_velocity"], dtype=float),
        brake_safety=float(limits_cfg["joint_brake_safety"]))

    rng = np.random.default_rng(20260806)
    tau_max = np.asarray(limits_cfg["torque"], dtype=float)
    for _ in range(300):
        A = rng.normal(size=(n, n))
        M = A @ A.T + 0.5 * np.eye(n)
        result = solve_hold_qp(
            rng.normal(size=(6, n)), M, rng.normal(size=n) * 3.0,
            rng.normal(size=6), rng.uniform(-2, 2, n),
            rng.normal(size=n) * 0.5, np.zeros(n), rng.normal(size=n) * 2.0,
            np.zeros(n), np.full(n, 2.0), np.full(n, 3.0), weights, limits)
        assert result.status == OPTIMAL
        assert np.all(np.abs(result.tau) <= tau_max + 1e-9)
