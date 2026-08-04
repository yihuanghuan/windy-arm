"""阶段 1 FK 校验工具的离线验证。

核心断言：正弦拟合能把"质量误差"和"关节零位误差"分开。
两者在单关节重力曲线上表现不同——质量误差改变幅值 A，零位误差改变
相位 phi，而正弦的幅值与相位彼此独立可辨识。若这个分离不成立，阶段 1
就无法判断该不该进入阶段 2 的质量标定。
"""

import os
import re
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from task_space_smc.fk_gravity_verify import (  # noqa: E402
    JOINT_LOWER,
    JOINT_UPPER,
    DESK_CLEARANCE,
    DESK_EDGE_X,
    DESK_PLANE_Z,
    LIMIT_MARGIN,
    N_ARM,
    RATED_TORQUE,
    SWEEP_PLAN,
    GravityModel,
    check_setpoint,
    fit_sine,
    interpolate,
    MAX_STEP,
    model_reference,
    model_reference_measured,
    offset_threshold_deg,
    plan_next_step,
    zero_offset,
)

# 实测力矩噪声 std（/dev/ttyUSB0 静止保持抓包），用于生成带噪合成数据
TAU_NOISE = np.array([0.0251, 0.0645, 0.0627, 0.0071, 0.0060, 0.0060])
SAMPLE_RATE_HZ = 200.6      # 实测板子上报速率


@pytest.fixture(scope="module")
def model():
    return GravityModel()


# ---------------------------------------------------------------- 正弦拟合

def test_fit_recovers_amplitude_and_phase_exactly():
    q = np.linspace(-1.0, 1.0, 21)
    tau = 2.5 * np.sin(q - 0.3) + 0.07
    fit = fit_sine(q, tau)
    assert fit["ok"]
    assert fit["amplitude"] == pytest.approx(2.5, abs=1e-9)
    assert fit["phi"] == pytest.approx(0.3, abs=1e-9)
    assert fit["offset"] == pytest.approx(0.07, abs=1e-9)
    assert fit["rms_residual"] < 1e-12


def test_mass_error_moves_amplitude_but_not_phase():
    q = np.linspace(-1.0, 1.0, 21)
    truth = fit_sine(q, 1.0 * np.sin(q - 0.2))
    heavy = fit_sine(q, 1.4 * np.sin(q - 0.2))     # +40% 质量，零位不变
    assert heavy["amplitude"] / truth["amplitude"] == pytest.approx(1.4, abs=1e-9)
    assert heavy["phi"] == pytest.approx(truth["phi"], abs=1e-9)


def test_zero_offset_moves_phase_but_not_amplitude():
    q = np.linspace(-1.0, 1.0, 21)
    truth = fit_sine(q, 1.0 * np.sin(q - 0.2))
    shifted = fit_sine(q, 1.0 * np.sin(q - 0.2 - 0.05))   # 0.05 rad 零位偏差
    assert shifted["amplitude"] == pytest.approx(truth["amplitude"], abs=1e-9)
    assert shifted["phi"] - truth["phi"] == pytest.approx(0.05, abs=1e-9)


def test_the_two_faults_are_separable_when_both_present():
    """同时存在 +40% 质量误差和 0.05 rad 零位偏差时仍能各自还原。"""
    q = np.linspace(-1.0, 1.0, 21)
    truth = fit_sine(q, 1.0 * np.sin(q - 0.2))
    both = fit_sine(q, 1.4 * np.sin(q - 0.25))
    assert both["amplitude"] / truth["amplitude"] == pytest.approx(1.4, abs=1e-9)
    assert both["phi"] - truth["phi"] == pytest.approx(0.05, abs=1e-9)


def _sweep_estimates(joint, injected_rad, trials, seed):
    """在该关节标定过的采样配置下，用蒙特卡洛跑出零位估计的分布。"""
    plan = SWEEP_PLAN[joint]
    idx = joint - 1
    m = GravityModel()
    qs = np.linspace(plan["lo"], plan["hi"], plan["points"])
    posture = list(plan["posture"])

    taus = []
    for v in qs:
        q = np.array(posture, dtype=float)
        q[idx] = v + injected_rad          # 实际角 = 命令角 + 偏差
        taus.append(m.gravity(q)[idx])
    taus = np.array(taus)
    ref = model_reference(m, joint, posture, qs)

    sigma_tau = TAU_NOISE[idx] / np.sqrt(plan["dwell"] * SAMPLE_RATE_HZ)
    rng = np.random.default_rng(seed)
    return np.array([
        np.degrees(zero_offset(fit_sine(qs, taus + rng.normal(0.0, sigma_tau, taus.shape))["phi"],
                               ref["phi"]))
        for _ in range(trials)
    ])


@pytest.mark.parametrize("joint", sorted(SWEEP_PLAN))
def test_zero_offset_estimator_is_unbiased(joint):
    """注入 0.5 度，估计的均值必须回到 0.5 度。

    容差取均值的标准误 3 倍，而不是固定值：joint4 为避开桌面换了信号更弱的
    位姿，sigma 从 0.10 度退化到 0.37 度，固定 0.02 度的判据比标准误还小，
    必然误判。
    """
    trials = 200
    est = _sweep_estimates(joint, np.radians(0.5), trials=trials, seed=1000 + joint)
    tol = max(0.02, 3.0 * SWEEP_PLAN[joint]["sigma"] / np.sqrt(trials))
    assert est.mean() == pytest.approx(0.5, abs=tol), (
        f"joint{joint}: biased estimator, mean {est.mean():.4f} deg "
        f"(tolerance {tol:.4f})")


@pytest.mark.parametrize("joint", sorted(SWEEP_PLAN))
def test_measured_sigma_matches_the_recorded_plan(joint):
    """SWEEP_PLAN 里记录的 sigma 必须和该配置实际跑出来的一致。

    这个值决定判据阈值，写错了会直接导致漏判或误判。
    """
    est = _sweep_estimates(joint, np.radians(0.5), trials=200, seed=2000 + joint)
    recorded = SWEEP_PLAN[joint]["sigma"]
    assert est.std() == pytest.approx(recorded, rel=0.35), (
        f"joint{joint}: recorded sigma {recorded:.4f} deg but measured "
        f"{est.std():.4f} deg — update SWEEP_PLAN")


@pytest.mark.parametrize("joint", sorted(SWEEP_PLAN))
def test_the_decision_threshold_is_at_least_three_sigma(joint):
    """阈值必须 >= 3 sigma，否则噪声本身就会频繁触发'几何有问题'的误报。"""
    plan = SWEEP_PLAN[joint]
    assert offset_threshold_deg(joint) >= 3.0 * plan["sigma"] - 1e-9


@pytest.mark.parametrize("joint", sorted(SWEEP_PLAN))
def test_a_real_fault_is_detected_and_a_healthy_joint_is_not(joint):
    """阈值处的判别力：无故障时误报率低，注入 2x 阈值时检出率高。"""
    thresh = offset_threshold_deg(joint)

    healthy = _sweep_estimates(joint, 0.0, trials=200, seed=3000 + joint)
    false_alarm = np.mean(np.abs(healthy) > thresh)
    assert false_alarm <= 0.02, (
        f"joint{joint}: {false_alarm*100:.1f}% false-alarm rate on a healthy joint")

    faulty = _sweep_estimates(joint, np.radians(2 * thresh), trials=200,
                              seed=4000 + joint)
    detection = np.mean(np.abs(faulty) > thresh)
    assert detection >= 0.98, (
        f"joint{joint}: only {detection*100:.1f}% detection at 2x threshold")


@pytest.mark.parametrize("joint", sorted(SWEEP_PLAN))
def test_clean_model_data_shows_no_false_offset(joint):
    """无故障时零位偏差必须判为 0，否则会误报几何问题。"""
    plan = SWEEP_PLAN[joint]
    m = GravityModel()
    qs = np.linspace(plan["lo"], plan["hi"], 13)
    meas = model_reference(m, joint, list(plan["posture"]), qs)
    ref = model_reference(m, joint, list(plan["posture"]), qs)
    assert abs(np.degrees(zero_offset(meas["phi"], ref["phi"]))) < 1e-6


@pytest.mark.parametrize("joint", sorted(SWEEP_PLAN))
def test_planned_amplitude_matches_the_model(joint):
    plan = SWEEP_PLAN[joint]
    m = GravityModel()
    qs = np.linspace(plan["lo"], plan["hi"], 25)
    ref = model_reference(m, joint, list(plan["posture"]), qs)
    assert ref["amplitude"] == pytest.approx(plan["amp"], rel=0.02)
    swing = 0.5 * np.ptp([_tau_at(m, joint, plan, v) for v in qs])
    assert swing == pytest.approx(plan["swing"], rel=0.02)


def _tau_at(m, joint, plan, v):
    q = np.array(plan["posture"], dtype=float)
    q[joint - 1] = v
    return m.gravity(q)[joint - 1]


# ---------------------------------------------------------------- 安全检查

def test_every_planned_setpoint_passes_safety(model):
    for joint, plan in SWEEP_PLAN.items():
        for v in np.linspace(plan["lo"], plan["hi"], plan["points"]):
            q = np.array(plan["posture"], dtype=float)
            q[joint - 1] = v
            rep = check_setpoint(q, model)
            assert rep.ok, f"joint{joint} at {v:.4f}: {rep.reasons}"


def test_setpoint_near_a_limit_is_refused(model):
    q = np.zeros(N_ARM)
    q[2] = JOINT_LOWER[2] + LIMIT_MARGIN * 0.5     # joint3 贴近下限
    rep = check_setpoint(q, model)
    assert not rep.ok
    assert any("limit" in r for r in rep.reasons)


def test_setpoint_over_torque_budget_is_refused(model, monkeypatch):
    import task_space_smc.fk_gravity_verify as mod
    monkeypatch.setattr(mod, "RATED_TORQUE", np.full(N_ARM, 0.05))
    q = np.zeros(N_ARM)
    rep = mod.check_setpoint(q, model)
    assert not rep.ok
    assert any("rated" in r for r in rep.reasons)


def test_nan_setpoint_is_refused(model):
    q = np.zeros(N_ARM)
    q[0] = np.nan
    assert not check_setpoint(q, model).ok


def test_wrong_dimension_is_refused(model):
    assert not check_setpoint(np.zeros(7), model).ok


def test_large_move_is_split_into_bounded_steps():
    q_from = np.zeros(N_ARM)
    q_to = np.full(N_ARM, 1.5)
    steps = interpolate(q_from, q_to)
    assert np.allclose(steps[-1], q_to)
    prev = q_from
    for s in steps:
        assert np.max(np.abs(s - prev)) <= 0.35 + 1e-9
        prev = s


def test_joint6_is_excluded_because_it_has_no_gravity_signal(model):
    """link6 质心落在 joint6 轴上，转它不产生重力力矩。"""
    assert 6 not in SWEEP_PLAN
    q = np.zeros(N_ARM)
    taus = []
    for v in np.linspace(-1.0, 1.0, 21):
        q[5] = v
        taus.append(model.gravity(q)[5])
    assert np.ptp(taus) < 1e-3, "joint6 unexpectedly has a gravity signal"


def test_joint6_does_not_move_the_end_effector(model):
    """既然 link6 绕自身轴对称，joint6 只改变末端姿态，不改变末端位置。"""
    q = np.zeros(N_ARM)
    p0 = model.ee_position(q)
    q[5] = 1.0
    p1 = model.ee_position(q)
    assert np.linalg.norm(p1 - p0) < 1e-9


# ------------------------------------------------- 带跟踪滞后的分步移动

def test_plan_next_step_never_exceeds_max_step_even_with_lag():
    """核心回归：位置环有稳态滞后时，每一步仍不得超过 MAX_STEP。

    原实现预先把整条路径切好再逐个下发，机械臂滞后后，预算好的下一个点
    与实际位置的距离会超过 MAX_STEP，安全检查把正常移动判为违规并中止
    扫描（实测报 "single step of 0.353 rad exceeds MAX_STEP=0.35"）。
    """
    rng = np.random.default_rng(0)
    q_target = np.array([1.0, -0.9, 0.8, 1.1, -0.7, 0.6])
    q = np.zeros(N_ARM)
    # 实测最大跟踪滞后 0.1859 rad，这里取 0.20 更严
    lag = 0.20
    for _ in range(200):
        if np.max(np.abs(q_target - q)) <= MAX_STEP:
            break
        step = plan_next_step(q, q_target)
        assert np.max(np.abs(step - q)) <= MAX_STEP + 1e-9, (
            f"step of {np.max(np.abs(step - q)):.4f} rad exceeds MAX_STEP")
        # 机械臂停在 step 附近但带一个方向随机的滞后
        q = step + rng.uniform(-lag, lag, N_ARM)
    assert np.max(np.abs(q_target - q)) <= MAX_STEP + lag


def test_plan_next_step_converges_despite_lag():
    """滞后不得让分步移动卡死：每步净推进必须为正。"""
    q_target = np.full(N_ARM, 2.0)
    q = np.zeros(N_ARM)
    prev = np.max(np.abs(q_target - q))
    for _ in range(200):
        if prev <= MAX_STEP:
            break
        q = plan_next_step(q, q_target) - 0.19      # 每次都朝远离目标滞后
        cur = np.max(np.abs(q_target - q))
        assert cur < prev, "no net progress; move_through would spin forever"
        prev = cur
    assert prev <= MAX_STEP


def test_plan_next_step_returns_target_when_close():
    q = np.zeros(N_ARM)
    t = np.full(N_ARM, MAX_STEP * 0.5)
    assert np.allclose(plan_next_step(q, t), t)


# ------------------------------------------------- 桌面碰撞约束

def test_every_planned_setpoint_clears_the_desk(model):
    """扫描计划不得让任何采样点撞到桌面。

    这条最初不存在，代价是实机把整条臂压在了桌面上：joint4 原来的位姿
    需要 joint1 = -1.5 rad，把机械臂绕基座轴甩下 86 度。当时的安全检查
    只有关节限位和力矩预算，几何上毫无防线。
    """
    for joint, plan in SWEEP_PLAN.items():
        for v in np.linspace(plan["lo"], plan["hi"], plan["points"]):
            q = np.array(plan["posture"], dtype=float)
            q[joint - 1] = v
            bad = model.desk_violations(q)
            assert not bad, (
                f"joint{joint} at {v:.4f}: "
                + "; ".join(f"{n} at z={p[2]*1000:.0f}mm x={p[0]*1000:.0f}mm"
                            for n, p in bad))


def test_the_posture_that_hit_the_desk_is_now_refused(model):
    """回归：真实撞桌的那个位姿必须被拒绝。"""
    q = np.array([-1.5627, 0.8505, -0.7368, -0.1183, 1.5020, 1.5001])
    rep = check_setpoint(q, model)
    assert not rep.ok
    assert any("desk" in r for r in rep.reasons)


def test_desk_constraint_only_applies_behind_the_edge(model):
    """桌面只在 x < DESK_EDGE_X 一侧；桌沿之外低于桌面高度是允许的。"""
    pts = np.array([
        [DESK_EDGE_X - 0.10, 0.0, DESK_PLANE_Z - 0.05],   # 桌上方，太低 -> 违规
        [DESK_EDGE_X + 0.10, 0.0, DESK_PLANE_Z - 0.05],   # 桌沿之外 -> 允许
    ])
    over_desk = pts[:, 0] < DESK_EDGE_X
    too_low = pts[:, 2] < DESK_PLANE_Z + DESK_CLEARANCE
    assert (over_desk & too_low).tolist() == [True, False]


def test_joint4_posture_keeps_joint1_near_zero():
    """joint4 的位姿不得再把 joint1 甩到大角度 —— 那正是撞桌的直接原因。"""
    assert abs(SWEEP_PLAN[4]["posture"][0]) < 0.5, (
        "joint4 posture drives joint1 far from zero, which swings the whole "
        "arm down onto the desk")


# ------------------------------------------------- 参考相位必须用实测位姿

def test_nominal_posture_reference_is_biased_by_sag(model):
    """回归：用标称位姿算参考相位会引入几度的偏差。

    位置环有 kp 限制的稳态下垂，机械臂并不停在标称位姿。参考相位对其余
    关节的位置极其敏感 —— joint3 的扫描位姿里 joint2 下垂 0.073 rad，
    就足以让表观零位偏差漂 4 度，是力矩噪声给出的 1-sigma 的 37 倍。

    这个错误是被一次重复测量抓到的：joint3 两次独立扫描的零位偏差差了
    2.243 度，而 A / ratio / 残差 / 静摩擦四项都重复到 0.6% 以内。
    """
    joint = 3
    plan = SWEEP_PLAN[joint]
    nominal = np.array(plan["posture"], dtype=float)
    qs = np.linspace(plan["lo"], plan["hi"], plan["points"])

    sag = np.zeros(N_ARM)
    sag[1] = -0.073                      # joint2 的实测下垂量级
    measured = np.tile(nominal + sag, (len(qs), 1))
    measured[:, joint - 1] = qs

    ref_nom = model_reference(model, joint, list(nominal), qs)
    ref_meas = model_reference_measured(model, joint, measured)
    bias = abs(np.degrees(zero_offset(ref_nom["phi"], ref_meas["phi"])))

    assert bias > 3.0, (
        f"expected a multi-degree bias from posture sag, got {bias:.3f} deg")
    assert bias > 20 * plan["sigma"], (
        "posture bias must dominate the torque-noise sigma, otherwise this "
        "regression does not reproduce the real failure")


def test_measured_reference_recovers_the_true_offset_despite_sag(model):
    """用实测位姿算参考时，注入的零位偏差必须被正确还原，不受下垂影响。"""
    joint = 3
    plan = SWEEP_PLAN[joint]
    nominal = np.array(plan["posture"], dtype=float)
    qs = np.linspace(plan["lo"], plan["hi"], plan["points"])
    injected = np.radians(2.0)

    sag = np.zeros(N_ARM)
    sag[1] = -0.073
    actual = np.tile(nominal + sag, (len(qs), 1))
    actual[:, joint - 1] = qs

    # 真值：实际角 = 实测角 + injected
    truth = actual.copy()
    truth[:, joint - 1] += injected
    taus = np.array([model.gravity(q)[joint - 1] for q in truth])

    meas = fit_sine(actual[:, joint - 1], taus)
    ref = model_reference_measured(model, joint, actual)
    recovered = np.degrees(zero_offset(meas["phi"], ref["phi"]))
    assert recovered == pytest.approx(2.0, abs=0.05), (
        f"measured-posture reference should recover the injected offset, "
        f"got {recovered:.4f} deg")


def test_measured_reference_rejects_wrong_shape(model):
    with pytest.raises(ValueError):
        model_reference_measured(model, 3, np.zeros((5, N_ARM + 1)))


# ------------------------------------------------- 热身扫描

def test_warmup_default_is_at_least_one():
    """默认必须带热身。到达新位姿后的首扫不可信 —— joint2 的四次首扫
    散布在 -7.9 ~ +5.6 度（跨度 13.5 度），而热身后的两遍吻合到 0.007 度
    （0.11 sigma）。少了这一步，每次测量都是一个随机数。
    """
    import subprocess
    import task_space_smc.fk_gravity_verify as mod
    src = open(mod.__file__).read()
    assert "'--warmup', type=int, default=1" in src, (
        "warm-up must default to at least one discarded sweep")


def test_warmup_sweeps_are_flagged():
    """热身结果必须带 warmup 标记，否则会被当成正式数据参与拟合。"""
    import task_space_smc.fk_gravity_verify as mod
    src = open(mod.__file__).read()
    assert "warmup=is_warmup" in src
    assert "is_warmup: bool = False" in src


def test_merge_excludes_warmup_sweeps():
    """合并工具必须跳过 warmup 标记的扫描。"""
    from pathlib import Path
    merge = Path(__file__).resolve().parent.parent / "task_space_smc" / "merge_fk_verify.py"
    src = merge.read_text()
    assert 's.get("warmup")' in src


def test_run_sweep_always_receives_the_csv_writer():
    """静态检查：run_sweep 的 writer 参数不能被循环变量遮蔽。

    实测踩过一次：热身循环写成 `for w in range(args.warmup)`，把外层的
    csv writer 变量 w 覆盖成整数，正式扫描那一遍就传了个 int 进去，
    在硬件上跑了 3 分钟才崩。这条路径需要 ROS 才能触发，单元测试覆盖
    不到，所以用静态检查兜底。
    """
    import re
    import task_space_smc.fk_gravity_verify as mod
    src = open(mod.__file__).read()
    # 只看调用点，排除 `def run_sweep(...)` 这个定义
    calls = [c for c in re.findall(r"(?<!def )run_sweep\(node[^)]*\)", src, re.S)]
    assert calls, "no run_sweep call sites found"
    for c in calls:
        assert "w_csv" in c, f"run_sweep called without the csv writer: {c!r}"
    # 循环变量不得叫 w
    assert "for w in range(" not in src, (
        "a loop variable named `w` would shadow the csv writer")


# ------------------------------------------------- 自动收敛

def test_convergence_is_on_by_default():
    """默认必须是"扫到收敛"，不是固定热身遍数。

    实测该量在到达位姿后【单调漂移】而非随机抖动：两轮分别从 5.023 / 4.494
    度爬向 9.75 / 9.41 度，b 从 0.30 爬向 0.66；一轮 2 遍就收敛，另一轮 3 遍
    仍在爬。同期 joint2 温度升了 4 度而其余关节不变。固定遍数必然有时够、
    有时不够，而"不够"的那次看不出任何异常。
    """
    import task_space_smc.fk_gravity_verify as mod
    src = open(mod.__file__).read()
    assert "'--converge', action='store_true', default=True" in src


def test_unconverged_sweeps_are_marked_warmup():
    """未收敛的扫描必须标成 warmup，绝不能进标定。"""
    import task_space_smc.fk_gravity_verify as mod
    src = open(mod.__file__).read()
    assert "r['warmup'] = True      # 未收敛的都算热身" in src
    assert "did not converge" in src


def test_convergence_tolerance_is_two_sigma():
    """收敛判据用该关节自己的 sigma，不是统一常数 ——
    joint1 的 sigma 是 0.0294 度，joint4 是 0.3677 度，差 12 倍。"""
    import task_space_smc.fk_gravity_verify as mod
    src = open(mod.__file__).read()
    assert "tol = 2.0 * SWEEP_PLAN[j]['sigma']" in src

def test_tool_refuses_to_run_when_the_position_path_is_bypassed():
    """节点开着 enable_direct_effort_command 时，ControlLoop 走力矩分支并
    return，/student/joint_command 被完全旁路 —— 本工具发的每条指令都被无声
    忽略，扫描照常跑完却产出全是噪声的数据。

    实机上这个错误消耗了一次 25 分钟的扫描才被发现，而且它的表象（关节不动）
    与静摩擦、与 RS05 通道失效都很像，极易误诊。必须在入口拦下。
    """
    import task_space_smc.fk_gravity_verify as mod
    src = open(mod.__file__).read()
    assert 'def check_position_path_active' in src
    assert re.search(r'why = check_position_path_active\(node\)', src), \
        "main must call the guard before doing anything else"
    assert re.search(r'enable_direct_effort_command', src)

