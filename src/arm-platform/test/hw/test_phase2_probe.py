"""阶段 2 力矩探针的回归测试。

覆盖本轮实机调试中暴露的每一个 bug —— 它们都不是假想的失效模式，
而是真实发生过、且有几个是在机械臂压在桌子上的时候才发现的。
"""

import os
import re
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

from task_space_smc.hw.phase2_torque_probe import (  # noqa: E402
    ladder_levels, desk_margin, HOLD_P, HOLD_D, LADDER_DM, LADDER_RS,
    RAMP_STEP, RAMP_KD, ESCAPE_STEPS, ESCAPE_KD, BREAK_MAX, BREAK_INCR,
    DISPLACEMENT_ABORT, PROBE_POSTURE, PUBLISH_RATE,
)
from task_space_smc.hw.fk_gravity_verify import (  # noqa: E402
    GravityModel, N_ARM, JOINT_LOWER, JOINT_UPPER,
)

SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                   'task_space_smc', 'hw', 'phase2_torque_probe.py')


@pytest.fixture(scope='module')
def model():
    return GravityModel()


@pytest.fixture(scope='module')
def source():
    with open(SRC) as fh:
        return fh.read()


# --------------------------------------------------------------------------
# 阶梯本身
# --------------------------------------------------------------------------

def test_ladder_is_symmetric_and_returns_to_zero():
    """对称阶梯让每级都被双向覆盖，摩擦滞回在拟合中平均掉。"""
    for j in range(1, N_ARM + 1):
        lv = ladder_levels(j)
        assert lv[0] == 0.0 and lv[-1] == 0.0, f"joint{j} must start/end at zero"
        assert abs(sum(lv)) < 1e-12, f"joint{j} ladder is not balanced"
        assert set(-v for v in lv) == set(lv), f"joint{j} is not sign-symmetric"


def test_ladder_visits_each_level_from_both_directions():
    for j in (1, 4):
        lv = ladder_levels(j)
        for target in (lv[1], lv[2]):
            assert lv.count(target) == 2, (
                f"joint{j} level {target} must be visited twice (up and down) "
                f"so hysteresis cancels")


def test_ladder_displacement_stays_below_the_abort_guard():
    """自限位移 t_ff/kp 必须留出足够余量，否则正常测量会被误判为失控。

    2 倍是下限：RS 关节 kp=5，0.3 Nm 给 0.06 rad，守卫 0.15 = 2.5 倍。
    再紧的余量会让"关节比预期多动一点"直接中止实验；再松则失去异常检测能力。
    """
    for j in range(1, N_ARM + 1):
        peak = max(abs(v) for v in ladder_levels(j))
        ratio = DISPLACEMENT_ABORT / (peak / HOLD_P[j - 1])
        assert ratio > 2.0, (
            f"joint{j}: abort guard is only {ratio:.1f}x the expected "
            f"displacement {peak / HOLD_P[j - 1]:.4f} rad")


def test_rs_joints_get_smaller_levels_than_dm_joints():
    assert max(LADDER_RS) < max(LADDER_DM)
    assert max(LADDER_RS) / 6.0 < 0.25, "RS05 rated 6 Nm — keep under 25%"
    assert max(LADDER_DM) / 28.0 < 0.25, "DM4340 rated 28 Nm — keep under 25%"


# --------------------------------------------------------------------------
# 兜底增益：p=d=0 在 MIT 模式下等于关节失力
# --------------------------------------------------------------------------

def test_hold_gains_are_all_strictly_positive():
    assert len(HOLD_P) == N_ARM and len(HOLD_D) == N_ARM
    assert all(v > 0 for v in HOLD_P), "p=0 makes the joint go limp"
    assert all(v > 0 for v in HOLD_D)


def test_probe_never_commands_zero_gains(source):
    """探针的任何路径都不得下发 p=0 —— 那会让关节在重力下垮落。"""
    assert not re.search(r'msg\.p\s*=\s*\[0', source)
    assert not re.search(r'p\s*=\s*\[0\.0\]\s*\*\s*N_ARM', source)


# --------------------------------------------------------------------------
# 力矩必须足够克服重力：3.6 Nm 曾经卡在桌上抬不起来
# --------------------------------------------------------------------------

def test_ramp_torque_exceeds_worst_case_gravity(model):
    """joint2 在下垂位姿光重力就要约 3.1 Nm，步长给的力矩必须显著超过它。

    实机上 RAMP_STEP=0.12 (3.6 Nm) 只剩 0.5 Nm 净提升力，被接触摩擦吃掉，
    机械臂卡在桌面上抬不起来。
    """
    q = np.array([-0.0517, 1.8172, -1.0267, 0.0033, -0.0003, 0.0024])
    # 【不要再乘经验比值】。config/arm6.urdf 的连杆惯性参数已按实机标定
    # 修正过（first_moment_ratio 已烘进 URDF），model.gravity() 返回的
    # 就是实机重力。原来这里写 `* 1.445` 是因为当时 URDF 是 CAD 名义值、
    # 偏轻约 30%；现在再乘就是【重复修正一次】，会把 2.83 Nm 夸大成 4.09 Nm。
    worst = abs(model.gravity(q)[1])
    assert RAMP_STEP * HOLD_P[1] > worst * 1.5, (
        f"ramp gives {RAMP_STEP * HOLD_P[1]:.1f} Nm but joint2 gravity alone is "
        f"{worst:.1f} Nm — not enough margin to lift")


def test_escape_ladder_is_monotonic_and_starts_above_gravity():
    assert ESCAPE_STEPS == sorted(ESCAPE_STEPS)
    assert ESCAPE_STEPS[0] * HOLD_P[1] > 5.0, "first escape level must beat gravity"
    assert ESCAPE_STEPS[-1] * HOLD_P[1] < 0.5 * 28.0, "cap escape below 50% rated"


def test_escape_damping_keeps_breakfree_speed_bounded():
    """脱离瞬间的稳态速度 = 净力矩/kd，必须保持在低速。"""
    gravity = 3.1
    for st in ESCAPE_STEPS:
        v = (st * HOLD_P[1] - gravity) / ESCAPE_KD
        assert v < 1.5, f"escape step {st} would run at {v:.2f} rad/s"


def test_ramp_damping_is_high_enough_for_its_step():
    v = (RAMP_STEP * HOLD_P[1] - 3.1) / RAMP_KD
    assert 0.0 < v < 1.0, f"ramp terminal speed {v:.2f} rad/s out of range"


# --------------------------------------------------------------------------
# 桌面余量与脱离
# --------------------------------------------------------------------------

def test_desk_margin_is_negative_when_the_arm_rests_on_the_desk(model):
    """实机上真实出现过的姿态：joint3 壳体压在桌面上。"""
    resting = np.array([-0.0509, 1.8221, -1.4414, 0.0032, 0.0013, 0.0024])
    assert desk_margin(model, resting) < 0.0


def test_desk_margin_is_positive_at_the_probe_posture(model):
    assert desk_margin(model, np.array(PROBE_POSTURE)) > 0.0


def test_lifting_joint2_monotonically_improves_the_margin(model):
    """脱离方向必须是单调改善的，否则"严格改善"判据会把机械臂锁死。"""
    resting = np.array([-0.0509, 1.8221, -1.4414, 0.0032, 0.0013, 0.0024])
    target = np.array([-0.05, 1.00, -1.0, 0.0, 0.0, 0.0])
    margins = [desk_margin(model, resting + (target - resting) * t)
               for t in np.linspace(0, 1, 21)]
    assert all(b >= a - 1e-9 for a, b in zip(margins, margins[1:])), \
        "margin must never get worse along the escape path"
    assert margins[-1] > 0.0


def test_escape_requires_sustained_clearance_not_one_sample(source):
    """单帧噪声曾把 margin 报成 +0.0054（实际 -0.0551），导致过早交接。"""
    assert 'clear_count' in source, "escape must confirm clearance over many frames"
    assert re.search(r'clear_count\s*>=\s*(\d+)', source)
    n = int(re.search(r'clear_count\s*>=\s*(\d+)', source).group(1))
    assert n >= 10, f"only {n} frames of confirmation is too few"


def test_escape_does_not_hand_back_to_a_weaker_torque(source):
    """脱开后降回温和模式会立刻再次卡住 —— 必须维持力矩直到真正脱离。"""
    assert '_drive_until_clear' in source
    assert re.search(r'if moved > ESCAPE_MOVED:.*?_drive_until_clear',
                     source, re.S), \
        "breaking free must lead into drive-until-clear, not an early return"


# --------------------------------------------------------------------------
# 标度测量：必须用力矩反馈，不能用位移
# --------------------------------------------------------------------------

def test_both_alphas_are_reported(source):
    """两个 alpha 含义不同，必须都留下：

    alpha_torque = 前馈未被关节运动吸收的比例（不动=1，自由=0），
    alpha_displacement = kp × d(位移)/d(命令)，只在【启动力矩以上】才等于标度。
    把任一个单独当作 "0x12 标度" 都是错的。
    """
    assert re.search(r'taus\s*=\s*np\.array', source)
    assert re.search(r'alpha\s*=\s*float\(coef_t\[0\]\)', source)
    assert 'alpha_displacement' in source


def test_alpha_torque_is_not_labelled_as_the_scale(source):
    """曾把 alpha_torque=0.983 当成 "0x12 = Nm 验证通过" —— 它只说明关节没动。"""
    assert '0x12 = Nm' not in source, \
        "alpha_torque must not be presented as proof of the 0x12 scale"


def test_zero_displacement_is_not_reported_as_a_failure(source):
    """静摩擦可能大于整个阶梯 —— 那时位移全零但标度依然可测。"""
    assert 'np.ptp(taus_chk)' in source, \
        "the plumbing check must look at torque feedback, not displacement"


def test_plumbing_check_verifies_the_effort_path_is_enabled(source):
    """launch 里 enable_direct_effort_command 曾是硬编码 False，命令行传值被
    静默忽略，整轮阶梯跑完却什么都没发出去。"""
    assert 'check_effort_enabled' in source
    assert 'enable_direct_effort_command' in source


# --------------------------------------------------------------------------
# 启动力矩搜索
# --------------------------------------------------------------------------

def test_breakaway_displacement_stays_well_inside_the_joint_limits(model):
    """位移自限于 t_ff/kp。真正的安全属性不是某个固定弧度值，而是【相对该
    关节在测试位姿的限位余量】—— RS 关节 kp 只有 5，同样的力矩位移是 DM 的
    6 倍，用统一常数会要么卡死 RS 关节要么放过 DM 关节。
    """
    q = np.array(PROBE_POSTURE)
    for j in range(1, N_ARM + 1):
        disp = BREAK_MAX[j] / HOLD_P[j - 1]
        room = min(q[j - 1] - JOINT_LOWER[j - 1], JOINT_UPPER[j - 1] - q[j - 1])
        assert room / disp > 3.0, (
            f"joint{j}: displacement {disp:.3f} rad vs only {room:.3f} rad "
            f"of limit room ({room / disp:.1f}x)")


def test_breakaway_of_wrist_joints_cannot_reach_the_desk(model):
    """joint4/5/6 的上限比 DM 关节宽松，前提是它们转动不会让机械臂接近桌面。"""
    q = np.array(PROBE_POSTURE)
    base = desk_margin(model, q)
    for j in (4, 5, 6):
        disp = BREAK_MAX[j] / HOLD_P[j - 1]
        for sgn in (-1, 1):
            qq = q.copy()
            qq[j - 1] += sgn * disp
            assert desk_margin(model, qq) > min(base * 0.5, base - 0.01), (
                f"joint{j} moving {sgn * disp:+.3f} rad drops the desk margin "
                f"from {base:+.4f} to {desk_margin(model, qq):+.4f} m")


def test_breakaway_resolution_is_fine_enough_to_be_useful():
    for j in range(1, N_ARM + 1):
        steps = BREAK_MAX[j] / BREAK_INCR[j]
        assert steps >= 20, f"joint{j}: only {steps:.0f} levels is too coarse"


def test_breakaway_stays_within_a_quarter_of_rated_torque():
    """25% 额定是上限：足以越过实测静摩擦（DM 最大 1.75 Nm、RS ≥0.6 Nm），
    又远低于会造成机械损伤的水平。"""
    rated = [28.0, 28.0, 28.0, 6.0, 6.0, 6.0]
    for j in range(1, N_ARM + 1):
        assert BREAK_MAX[j] / rated[j - 1] <= 0.25, (
            f"joint{j} caps at {BREAK_MAX[j] / rated[j - 1] * 100:.0f}% of rated")


def test_breakaway_cap_exceeds_the_measured_stiction():
    """上限必须高于实测静摩擦，否则测不到起步点（首轮 joint4/5 就是这样）。"""
    measured = {1: 0.70, 2: 1.75, 3: 0.65}
    for j, f in measured.items():
        assert BREAK_MAX[j] > f * 1.5, (
            f"joint{j}: cap {BREAK_MAX[j]} Nm is too close to stiction {f} Nm")


# --------------------------------------------------------------------------
# 发布频率必须远高于节点的力矩超时
# --------------------------------------------------------------------------

def test_publish_rate_beats_the_effort_timeout():
    """节点的 effort_command_timeout_sec 默认 0.05s；发布周期必须远小于它，
    否则每个采样点都会掉进兜底。"""
    assert 1.0 / PUBLISH_RATE < 0.05 / 3.0

# --------------------------------------------------------------------------
# 通道失效必须与"静摩擦极大"区分开
# --------------------------------------------------------------------------

def test_breakaway_would_not_report_a_dead_channel_as_stiction(source):
    """RS05 三个通道曾在上电后不建立通信，表现与"静摩擦 >=1.5 Nm"完全一致：
    位置和力矩指令都无响应。恢复后实测静摩擦只有 0.12 Nm —— 差 12 倍。

    唯一的判别特征是【力矩反馈是否跟随命令】：
      真·静摩擦 -> 关节不动但 tau 精确跟随 (dTau/dc ~ 1)
      通道失效   -> 关节不动且 tau 恒定不变
    breakaway 的 trace 必须记录 tau，否则事后无法区分。
    """
    assert re.search(r"trace\.append\(dict\(.*tau=", source, re.S), \
        "the breakaway trace must record tau so a dead channel is detectable"


def test_joint_is_returned_to_the_posture_after_each_measurement(source):
    """joint4 测完停在离位姿 0.161 rad 处，触发了 joint5/6 的位移守卫，
    导致后两个关节整个测不成。测完必须【验证】回位，不能只发一次保持指令。"""
    assert 'def return_joint' in source
    assert re.search(r'why = return_joint\(node, joint\)', source), \
        "run_breakaway must return the joint before moving on"

def test_effort_hold_target_is_latched_not_recaptured_every_cycle():
    """ControlLoop 的 !got_effort_command_ 分支曾每周期调用
    MakeHoldPositionCommand()，而后者以【当前】位置为目标。于是本周期
    「保持在现在的位置」-> 重力拉低一点 -> 下周期以更低位置为基准，形成棘轮。
    实机上因此三次垂到桌面，最后一次桌面余量 -0.075 m、joint3 距限位仅
    0.026 rad。保持目标必须只锁存一次。
    """
    cc = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                      'src', 'ros_nodes', 'student_arm_node.cc')
    src = open(cc).read()
    assert 'effort_hold_latched_' in src, "the latch flag must exist"
    m = re.search(r'if \(!got_effort_command_\) \{(.*?)\n    \}', src, re.S)
    assert m, "could not locate the pre-first-command hold branch"
    body = m.group(1)
    assert 'effort_hold_latched_' in body, (
        "the hold branch must be guarded by the latch, otherwise it recaptures "
        "the (sagging) current position every cycle")
    assert re.search(r'effort_hold_latched_\s*=\s*false', src), (
        "the latch must be released when a real command arrives, so a later "
        "dropout re-latches at that moment's position")

def test_phase3_disables_the_drift_guard_during_transit():
    """漂移守卫用于发现「本该静止的关节动了」。大范围转移时所有关节都在按
    设计移动，套用它会把正常转移误判为漂移 —— 实机上阶段 3 第一次运行就在
    起步瞬间中止（joint3 距扫描位姿 2.496 rad 是转移距离，不是漂移）。
    """
    p3 = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                      'task_space_smc', 'hw', 'phase3_gravity_calibration.py')
    src = open(p3).read()
    assert 'check_drift' in src
    assert re.search(r'check_drift=False', src), \
        "the transit to the sweep posture must disable the drift guard"
    assert re.search(r'def guard\(self, posture, joint_idx: int,\s*\n?\s*check_drift',
                     src), "guard must accept the flag"

@pytest.mark.parametrize("joint", [1, 2, 3, 4, 5])
def test_phase3_analysis_recovers_truth_from_noiseless_data(joint, model):
    """用【模型自己生成的无噪声数据】跑一遍阶段 3 的分析链路，
    正确结果必须是 ratio=1、零位偏差=0。

    这能锁死 fit_sine / model_reference_measured 的键名与签名 —— 实机上
    因为把 fit['amplitude'] 写成 fit['A']、把 model_reference_measured 的
    签名记错（多传一个参数），扫描跑完 13 个点才在分析阶段崩掉。
    """
    from task_space_smc.hw.fk_gravity_verify import (
        fit_sine, model_reference_measured, zero_offset, SWEEP_PLAN)
    p = SWEEP_PLAN[joint]
    post = np.array(p['posture'], float)
    poses, taus = [], []
    for v in np.linspace(p['lo'], p['hi'], p['points']):
        q = post.copy()
        q[joint - 1] = v
        poses.append(q)
        taus.append(model.gravity(q)[joint - 1])
    poses = np.array(poses)
    fit = fit_sine(poses[:, joint - 1], np.array(taus))
    ref = model_reference_measured(model, joint, poses)
    assert fit['amplitude'] / ref['amplitude'] == pytest.approx(1.0, abs=1e-6)
    assert np.degrees(zero_offset(fit['phi'], ref['phi'])) == pytest.approx(0.0, abs=1e-6)

def test_fence_is_modelled_and_catches_the_real_incident(model):
    """桌面围栏此前完全不在碰撞模型里，joint6 因此撞上它。

    围栏几何（用户实测）：沿 -x 延伸的竖直挡板，+y 侧约 420~440mm（模型取
    保守值 400mm），高度约 331mm（基座到 joint3 的距离）。

    检验：原 joint1 扫描起点 q1=-1.024 必须被判为违规 —— 那正是实机上
    joint6 伸到 y=468.7mm 撞围栏的位形；而新的扫描区间必须全程安全。
    """
    from task_space_smc.hw.fk_gravity_verify import (
        SWEEP_PLAN, check_setpoint, FENCE_Y, FENCE_HEIGHT, FENCE_CLEARANCE)
    assert FENCE_Y > 0 and FENCE_HEIGHT > 0 and FENCE_CLEARANCE > 0
    post = np.array(SWEEP_PLAN[1]['posture'], float)

    old_start = post.copy()
    old_start[0] = -1.024
    assert not check_setpoint(old_start, model).ok, (
        "the old joint1 sweep start must be rejected — that is where joint6 "
        "actually hit the fence")

    p = SWEEP_PLAN[1]
    for v in np.linspace(p['lo'], p['hi'], p['points']):
        q = post.copy()
        q[0] = v
        assert check_setpoint(q, model).ok, f"joint1 sweep point {v:+.3f} unsafe"

def test_phase5_counts_unique_telemetry_not_loop_iterations():
    """脉冲循环以约 800 Hz 运行而遥测只有 200 Hz，若每次循环都记一笔会得到
    约 4 倍重复值。拟合不受影响（重复值等权），但报出的采样数虚高 4 倍 ——
    实机上因此先把 joint4/5 误判为「采样点太少、不可行」，实际完全可行。
    """
    p5 = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                      'task_space_smc', 'hw', 'phase5_inertia_id.py')
    src = open(p5).read()
    assert re.search(r'if not samples or samp\[1:\] != samples\[-1\]\[1:\]', src), \
        "pulse() must de-duplicate repeated telemetry before counting samples"


def test_phase5_pulse_has_a_hard_time_bound():
    """kp=0 下主动加速：脉冲若未按时终止，joint4 在 200 rad/s² 下 0.5 秒
    可冲出 6 rad 撞限位。时长上限必须硬编码在循环条件里。"""
    from task_space_smc.hw.phase5_inertia_id import (
        PULSE_MAX_SEC, DISP_ABORT, SPEED_ABORT)
    assert 0 < PULSE_MAX_SEC <= 0.3, "脉冲硬上限应在 0.3s 以内"
    assert 0 < DISP_ABORT <= 0.15
    assert 0 < SPEED_ABORT <= 3.0
    p5 = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                      'task_space_smc', 'hw', 'phase5_inertia_id.py')
    src = open(p5).read()
    assert re.search(r'while time\.time\(\) - t0 < PULSE_MAX_SEC', src), \
        "时长上限必须在 while 条件里，不能只靠循环内的 break"

def test_runner_uses_model_joint_names_not_a_hardcoded_seven():
    """/joint_states 回调若按写死的 7 关节名遍历，6 关节的消息里找不到 joint7
    就会直接 return，feedback_generation 永不递增 —— runner 认为「没有新反馈」
    而不发指令，plant 又要收到指令才步进，两边死锁。

    实机表现极具迷惑性：RViz 里模型散开、报 "Frame [world] does not exist"，
    看起来像 TF 或 URDF 的问题，实际是控制循环从未执行（TF 只在循环里发布）。
    """
    r = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                     'task_space_smc', 'hw', 'whole_task_smc_ros_runner.py')
    src = open(r).read()
    assert re.search(r'self\.arm_joint_names = list\(self\.model\.arm_joint_names\)', src), \
        "runner must take joint names from the model"
    assert not re.search(r'for j, name in enumerate\(DEFAULT_ARM_JOINT_NAMES\)', src), \
        "the feedback callback must not iterate a hardcoded 7-joint list"
    assert not re.search(r'msg\.name = list\(DEFAULT_ARM_JOINT_NAMES\)', src)


def test_torque_sim_arm_derives_joint_count_from_config():
    """TorqueSimArm 曾在 BuildModel() 之前用默认值 7 校验配置，把 6 关节配置
    判为 "requires a seven-joint arm configuration" —— 报错信息看起来像有意的
    约束，实际是校验时机不对。"""
    cc = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                      'src', 'robotics', 'arm', 'torque_sim_arm.cc')
    src = open(cc).read()
    assert 'seven-joint arm configuration' not in src
    assert re.search(r'arm_dof_ = static_cast<int>\(arm_config\.joints\.size\(\)\)', src), \
        "InitFromConfig must take the joint count from the config"
    assert 'const int urdf_dof = model_.nv - 6' in src, \
        "BuildModel must cross-check against the URDF"

def test_build_model_must_not_clobber_config_limits():
    """BuildModel 在 InitFromConfig 之后执行。无条件重置 torque_limit_/
    lower_limit_ 会把配置里的 9/1.6 Nm 冲成 1.0、限位冲成 ±π ——
    plant 连自重都撑不住。

    实机表现极具迷惑性：RViz 里模型乱飞，看起来像控制器发散或 z 轴翻转，
    实际是所有关节卡在 ±π（TorqueSimArm 的默认限位），末端误差 371 mm。
    """
    cc = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                      'src', 'robotics', 'arm', 'torque_sim_arm.cc')
    src = open(cc).read()
    m = re.search(r'void TorqueSimArm::BuildModel\(\).*?\n\}', src, re.S)
    assert m, "BuildModel not found"
    body = m.group(0)
    assert re.search(r'if \(torque_limit_\.size\(\) != arm_dof_\)', body), \
        "BuildModel must only reset limits when the size actually changed"
    assert 'ApplyArmature()' in body, \
        "the plant must apply the same armature as the controller"


def test_plant_and_controller_share_one_armature_source():
    """控制器 (model.py) 与 plant (torque_sim_arm.cc) 必须读同一个标定文件。
    不一致时控制器按 M=0.11~0.20 算力矩、plant 按 M=0.0006~0.079 积分，
    实际加速度比预期大 2.5~20 倍，仿真必然发散。

    离线仿真【不会】暴露这个问题 —— 它用同一个模型既做控制又做积分，
    模型误差自动抵消。
    """
    cc = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                      'src', 'robotics', 'arm', 'torque_sim_arm.cc')
    py = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                      'task_space_smc', 'core', 'model.py')
    assert 'arm6_calibration.yaml' in open(cc).read()
    assert 'armature_kgm2' in open(py).read()

