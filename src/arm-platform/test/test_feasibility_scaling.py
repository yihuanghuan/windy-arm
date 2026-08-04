"""控制器必须【知道并服从】被控对象的运动学硬夹（速度夹 + 位置夹）。

【这些测试的存在理由】
src/robotics/arm/torque_sim_arm.cc:306 每个积分子步都做
    qd = clamp(qd, ±velocity_limit)          [3,3,3,5,5,5] rad/s
:309-314 再做位置硬夹并把撞限方向的 qd 置 0。而控制器（redundancy_resolution /
task_space_smc / actuator_adapter）里【没有任何一处建模它们】——grep 全链路为零。

后果不是"末端沿路径滞后"，而是 plant 只夹越界那一轴、其余关节毫不知情：
    需求超包线 -> plant 逐轴夹 -> 末端加速度方向被转掉 -> 误差起飞
    -> SMC(λ=72,k=102) 要 102·72·0.03 ≈ 220 m/s² -> 指令 −727.50 Nm
       （40.4 倍限幅）-> 六轴力矩饱和 -> 末端被主动甩出 41.64 mm
真实 ROS 逐点判夹（14608 样本）：速度夹只占 1.52% 的采样却贡献 96.85% 的
平方误差（剔除后 RMS 2.018 -> 0.361 mm）；速度夹∪位置夹占 2.41%、贡献 98.56%
（剩余 0.245 mm）。P(err>1mm | 速度夹)=65.3%，P(err>1mm | 无夹)=1.01%。
"被动滞后"解释不了这个形态：err>10mm 时末端实际速度中位 0.44~0.46 m/s，
而 |v_ref|+|v_base@ee| 的滞后上界中位只有 0.18 m/s。

【为什么不能放宽硬夹】它是物理量，且实测放宽只搬家不消除：
max_velocity ×1.5 RMS 2.451~2.994（无改善）、×3 峰值 41 -> 87.8mm 且安静
时段误差地板从 0.0196 塌到 0.188mm、×30 发散到 RMS 50mm；tau_scale=2.0
RMS 2.018 -> 4.949。所以唯一正确的方向是让控制器服从它们。

【为什么不能"整条 qdd 乘一个 α"】这是本文件最重要的一组测试。
把 α = min_i(盒边界/需求_i) 直接乘到 qdd_task 上，只要有【一个】关节正贴着
速度上限（hi_i = 0），α 就恒为 0，整条手臂的任务加速度被清零、以 3 rad/s
匀速滑走再也不减速。真实 ROS 实测（域 90/91，同一棵树基线 2.466 mm）：
    统一 α 缩放 + 位置制动   RMS 457.19 mm  峰值 695.6 mm  α<0.01 占 77.2%
    统一 α 缩放（仅速度盒）  RMS 192.03 mm  峰值 598.4 mm  α<0.01 占 60.3%
而"关节贴着速度上限"本身是良性的 —— 基线里 t=2.99 时 joint2 已经 −2.943
rad/s，位置误差仍只有 0.025 mm。所以正确处理是把顶到限的轴钉在边界上、
【让其余关节重新解算把任务补回来】（SNS），而不是全体降速。

改动前这些测试全部失败（`resolve_acceleration()` 根本没有 feasibility 形参，
smc_params_whole6.yaml 也没有 limits.joint_velocity 键）。
"""

import os
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from task_space_smc.redundancy_resolution import (  # noqa: E402
    FeasibilityLimits, _feasible_acceleration_box, resolve_acceleration)

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DT = 0.005                                   # 200 Hz，与 control_rate_hz 一致
VLIM = np.array([3.0, 3.0, 3.0, 5.0, 5.0, 5.0])
TAU = np.array([18.0, 18.0, 18.0, 3.2, 3.2, 3.2])
QLO = np.array([-4.0, -3.14, -1.57, -4.0, -2.0, -4.0])
QHI = np.array([4.0, 1.57, 1.57, 4.0, 2.0, 4.0])


def _case(seed=7):
    """一个条件良好的 5D 任务雅可比 + 正定质量阵。"""
    rng = np.random.default_rng(seed)
    J = rng.normal(size=(5, 6))
    L = rng.normal(size=(6, 6))
    M = 0.05 * (L @ L.T) + 0.15 * np.eye(6)
    return J, M


def _limits(brake_safety=0.5, dt=DT):
    return FeasibilityLimits(
        velocity=VLIM, lower=QLO, upper=QHI, dt=dt,
        torque=TAU if brake_safety > 0.0 else None,
        brake_safety=brake_safety)


def _solve(J, M, rhs, q, qd, feasibility=None, kp_null=2.0, kd_null=3.0,
           q_nominal=None):
    return resolve_acceleration(
        J, M, rhs, q, qd,
        np.zeros(6) if q_nominal is None else q_nominal,
        np.full(6, kp_null), np.full(6, kd_null),
        characteristic_length=0.2,
        sigma_warn=0.04, sigma_critical=0.012,
        damping_min=5e-5, damping_max=0.03,
        feasibility=feasibility)


# --------------------------------------------------------------------------
# 1. 速度夹：指令加速度不得把关节速度推出电机包线
# --------------------------------------------------------------------------

def test_command_never_pushes_joint_velocity_past_plant_clamp():
    """下一步的 qd 必须留在 ±velocity_limit 内 —— 否则 plant 会逐轴夹掉。"""
    J, M = _case()
    # joint1 已经跑在 3.0 rad/s 的 99.7%，同时给一个很强的任务需求。
    qd = np.array([2.99, -4.9, 0.5, 0.0, 0.0, 0.0])
    q = np.zeros(6)
    rhs = np.array([40.0, -25.0, 15.0, 3.0, -2.0])

    free = _solve(J, M, rhs, q, qd)                       # 改动前的行为
    fixed = _solve(J, M, rhs, q, qd, feasibility=_limits())

    qd_next_free = qd + free.qdd_cmd * DT
    qd_next_fixed = qd + fixed.qdd_cmd * DT
    # 先确认这个场景确实会触发 plant 的硬夹（否则测试没有区分力）
    assert np.any(np.abs(qd_next_free) > VLIM + 1e-9), (
        "构造的场景没有触发速度夹，测试失去意义")
    assert np.all(np.abs(qd_next_fixed) <= VLIM + 1e-9), (
        f"指令仍会越过电机速度包线: {qd_next_fixed} vs {VLIM}")


def test_saturated_joint_is_pinned_and_the_others_compensate():
    """【本次改动的核心】一轴顶到速度上限时，任务加速度必须由其余关节补回来。

    plant 的做法是只夹那一轴、其余关节保持原指令 -> 末端加速度方向被转掉。
    这条测试就是量这个转角：SNS 解的末端加速度应与原始需求完全一致（还有
    5 个自由关节张成 5D 任务空间），而"逐轴裁剪"的对照组会明显偏离。
    """
    J, M = _case(21)
    qd = np.array([-0.5, -2.998, 0.6, 0.0, 0.5, 0.05])   # joint2 贴着 −3
    q = np.zeros(6)
    # 【需求方向必须指向限位】否则盒子根本不夹。原来的 rhs 让 joint2 往
    # 正方向走（qdd=+5.7，上界 1199.6），完全不触发裁剪，测试没有区分力。
    # 取负号让它继续往 -3 方向压，此时下界只有 -0.4。
    rhs = -np.array([6.0, -4.0, 3.0, 0.8, -0.5])

    free = _solve(J, M, rhs, q, qd)
    fixed = _solve(J, M, rhs, q, qd, feasibility=_limits())

    lo = (-VLIM - qd) / DT
    hi = (VLIM - qd) / DT
    clipped = np.clip(free.qdd_cmd, lo, hi)              # plant 的逐轴裁剪
    assert not np.allclose(clipped, free.qdd_cmd), "场景没有触发裁剪，无区分力"

    want = J @ free.qdd_cmd
    got_sns = J @ fixed.qdd_cmd
    got_clip = J @ clipped

    def cos(a, b):
        return float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)))

    assert np.allclose(got_sns, want, rtol=1e-6, atol=1e-6), (
        f"SNS 没有把任务补回来: {got_sns} vs {want}")
    assert cos(got_clip, want) < 1.0 - 1e-4, (
        "对照组构造失败：逐轴裁剪在此场景下没有转向末端加速度")
    assert fixed.feasibility_scale == 1.0


def test_no_lockup_when_a_joint_rides_the_velocity_ceiling():
    """【回归 457mm 事故】一轴精确停在速度上限时不得把整条指令清零。

    "整条 qdd 乘统一 α" 在这里给 α = 0（hi_2 = 0 而需求 qdd_2 < 0），
    真实 ROS 上因此 RMS 从 2.466 mm 崩到 457.19 mm、峰值 695.6 mm，
    日志里 77.2% 的采样 α < 0.01、joint2 精确卡在 −3.000 rad/s。
    """
    J, M = _case(33)
    qd = np.array([-0.486, -3.0, 0.593, -0.004, 0.483, 0.055])  # 事故现场实测值
    q = np.zeros(6)
    # 需求方向必须让 joint2 继续往 -3 压（qdd[1] < 0），否则 lo[1]=0 这个
    # 卡死点根本不起作用。原 rhs 给出 qdd[1]=+0.988（远离限位），复现不出事故。
    rhs = -np.array([3.0, -2.0, 1.5, 0.4, -0.3])

    free = _solve(J, M, rhs, q, qd)
    fixed = _solve(J, M, rhs, q, qd, feasibility=_limits())

    # 该场景对"统一 α 缩放"确实是致命的：α 会是 0
    lo = (-VLIM - qd) / DT
    hi = (VLIM - qd) / DT
    # joint2 顶在【负】限位 -3.0，被卡死的是下界：lo[1] == 0。
    # （原来这里查 hi[1]，符号写反了 —— hi[1] = (3-(-3))/dt = 1200，永远不为 0。）
    assert lo[1] == 0.0, f"没有复现出卡死构型: lo[1]={lo[1]}"
    assert free.qdd_cmd[1] < 0.0, (
        f"任务需求没有指向限位方向（qdd[1]={free.qdd_cmd[1]:.3f}），"
        f"α 不会被压到 0，场景无区分力")

    assert np.linalg.norm(fixed.qdd_cmd) > 0.5 * np.linalg.norm(free.qdd_cmd), (
        f"指令被清零了：|qdd| {np.linalg.norm(fixed.qdd_cmd):.4f} vs "
        f"{np.linalg.norm(free.qdd_cmd):.4f}")
    assert fixed.feasibility_scale > 0.99, (
        f"任务被无谓地降速到 s={fixed.feasibility_scale}")
    assert np.all(np.abs(qd + fixed.qdd_cmd * DT) <= VLIM + 1e-9)


def test_task_is_uniformly_scaled_only_when_truly_infeasible():
    """自由关节不足以张成任务空间时才降速，且必须是【任务空间统一缩放】——
    末端沿指令方向按比例滞后，方向不变。逐轴裁剪会把方向转掉。"""
    J, M = _case(41)
    # 【真不可行】= 自由关节数 < 任务维度(5)。六轴里让【四轴】同时贴限且
    # 需求全部指向限位方向，剩 2 个自由关节张不成 5D 任务空间。
    #
    # 原场景只贴了三轴、且需求方向未全部指向限位，SNS 用剩余关节解出来了
    # （实测 qdd=[-115.6,182.1,-182.0,154.8,-104.2,-97.7]，下一步全在包线内），
    # 那是【可行】的情况，本来就不该降速。
    qd = np.array([3.0, 3.0, 3.0, 5.0, 0.0, 0.0])
    q = np.zeros(6)
    rhs = np.array([600.0, 600.0, 600.0, 600.0, 600.0])
    fixed = _solve(J, M, rhs, q, qd, feasibility=_limits())
    free = _solve(J, M, rhs, q, qd)
    assert fixed.feasibility_scale < 1.0, "该场景应判为不可行并降速"
    assert np.all(np.abs(qd + fixed.qdd_cmd * DT) <= VLIM + 1e-9)
    # 降速后仍要保末端方向（允许饱和轴钉住带来的少量偏差）
    want = J @ free.qdd_cmd
    got = J @ fixed.qdd_cmd
    assert float(got @ want / (np.linalg.norm(got) * np.linalg.norm(want))) > 0.9


# --------------------------------------------------------------------------
# 2. 位置夹：靠近限位时必须提前减速，而不是撞墙后被 plant 把 qd 置 0
# --------------------------------------------------------------------------

def test_brake_constraint_stops_before_the_position_wall():
    """闭环推进：joint1 被一个恒定的、朝上限的强需求推着走。

    改动前 plant 只能在 q 越界后把 qd 硬置 0（真实 ROS 里 joint3 撞 ±1.57、
    joint5 撞 ±2.0，qd 变成精确的 0.000，误差 4.84mm，而此时 σ 健康
    0.067~0.097、阻尼在下限 5e-5、力矩甚至零饱和 —— 与奇异和泄漏都无关）。
    加了制动约束后关节应【自己】把速度收敛到 0，撞墙速度接近 0。
    """
    _, M = _case(3)
    lim = _limits()

    def roll(with_brake):
        q = np.array([3.0, 0.0, 0.0, 0.0, 0.0, 0.0])   # joint1 距上限 1.0 rad
        qd = np.zeros(6)
        for _ in range(4000):
            qdd = np.zeros(6)
            qdd[0] = 80.0                              # 一直要求朝上限加速
            if with_brake:
                _, hi = _feasible_acceleration_box(q, qd, M, lim)
                qdd = np.minimum(qdd, hi)
            qd = np.clip(qd + qdd * DT, -VLIM, VLIM)   # plant 的速度夹
            q_next = q + qd * DT
            if q_next[0] >= QHI[0]:                    # plant 的位置夹
                return float(qd[0])
            q = q_next
        return 0.0

    speed_without = roll(False)
    speed_with = roll(True)
    assert speed_without > 2.0, (
        f"对照组没有以高速撞限位（{speed_without:.3f} rad/s），场景无效")
    assert speed_with < 0.35, (
        f"仍以 {speed_with:.3f} rad/s 撞上位置限，制动约束没有起作用")


def test_brake_constraint_only_bites_near_the_limit():
    """制动约束不能全程干扰主任务 —— 远离限位时必须完全不生效。

    实测有效余量只有 0.07~0.10 rad（a_brake = 0.5·τ/M_ii = 47~151 rad/s²，
    v²/2a）。这条测试防止有人把 brake_safety 调小到把整条轨迹拖慢。
    """
    _, M = _case(5)
    lo, hi = _feasible_acceleration_box(np.zeros(6), np.zeros(6), M, _limits())
    # 制动项生效的条件是 sqrt(2·a_brake·余量) < v_max，即余量 < v²/(2·a_brake)。
    # 【不能一律断言零位处不生效】：joint5 的 a_brake 只有 2.5 rad/s²，
    # 需要 5 rad/s 就要 5 m 余量，而它的行程只有 ±2 rad —— 它【全程】受制动
    # 约束，这是关节行程决定的，不是参数调错。实测 sqrt(2·a·QHI)=3.19 < 5。
    md = np.diag(np.asarray(M, dtype=float))
    a_brake = 0.5 * np.asarray(TAU) / md
    brake_v = np.sqrt(2.0 * a_brake * np.asarray(QHI))
    binds = brake_v < np.asarray(VLIM)
    assert np.allclose(hi[~binds], (VLIM / DT)[~binds]), (
        "余量足够的关节，上界应就是纯速度夹")
    assert np.allclose(lo[~binds], (-VLIM / DT)[~binds])
    # 受制动约束的关节，上界必须严格小于纯速度夹，且等于制动速度换算值
    assert np.all(hi[binds] < (VLIM / DT)[binds])
    assert np.allclose(hi[binds], (brake_v / DT)[binds])
    # 且这不能是全部关节 —— 否则说明 brake_safety 被调得过小，全程拖慢轨迹
    assert binds.sum() < len(VLIM), (
        f"所有关节都受制动约束（{binds}），brake_safety 过小会拖慢整条轨迹")


# --------------------------------------------------------------------------
# 3. 优先级与不变量
# --------------------------------------------------------------------------

def test_nullspace_yields_its_budget_to_the_task():
    """任务项优先吃盒子，零空间项只在剩余预算里缩放，合成结果仍在盒内。"""
    J, M = _case(17)
    qd = np.array([2.995, 0.0, 0.0, 4.99, 0.0, 0.0])
    q = np.zeros(6)
    rhs = np.array([50.0, 10.0, -10.0, 2.0, 2.0])
    res = _solve(J, M, rhs, q, qd, feasibility=_limits(),
                 kp_null=400.0, kd_null=0.0, q_nominal=np.full(6, -3.0))
    qd_next = qd + res.qdd_cmd * DT
    assert np.all(np.abs(qd_next) <= VLIM + 1e-9), (
        f"零空间项把指令推出了包线: {qd_next}")
    assert res.nullspace_scale <= 1.0 + 1e-12


@pytest.mark.parametrize("seed", [0, 1, 2, 3, 4])
def test_command_unchanged_when_everything_is_feasible(seed):
    """一切可行时新代码必须是【严格的恒等变换】—— 不能顺手改变基线行为。"""
    J, M = _case(seed)
    q = np.zeros(6)
    qd = np.zeros(6)
    rhs = np.array([0.4, -0.3, 0.2, 0.1, -0.1])
    free = _solve(J, M, rhs, q, qd)
    fixed = _solve(J, M, rhs, q, qd, feasibility=_limits())
    assert fixed.feasibility_scale == 1.0
    assert fixed.nullspace_scale == 1.0
    assert np.allclose(fixed.qdd_cmd, free.qdd_cmd, atol=1e-12, rtol=1e-12)


def test_box_is_never_empty():
    """盒子必须恒有解，否则控制器在最需要它的时刻退化。

    这【不等于】"盒子必须含 0"：v_lo <= 0 <= v_hi 恒成立（速度上限非负，
    位置制动项只把界往 0 收），所以 lo <= hi 天然成立，不需要额外塞 0 进去。
    """
    _, M = _case()
    for qd in (np.zeros(6), np.array([9.0, -9.0, 0.0, 0.0, 0.0, 0.0]),
               np.array([-2.9, 2.9, 0.0, 4.9, -4.9, 0.0])):
        lo, hi = _feasible_acceleration_box(np.zeros(6), qd, M, _limits())
        assert np.all(lo <= hi + 1e-12), f"盒子为空: qd={qd} lo={lo} hi={hi}"


def test_box_forces_return_when_velocity_is_already_out_of_range():
    """qd 已在包线外时，盒子必须【强制】把它拉回来，不能允许"零加速度"。

    这是 test_command_never_pushes_joint_velocity_past_plant_clamp 的前置条件。
    早期实现用 max(...,0)/min(...,0) 把 0 塞进盒子以"保证有解"，结果是
    qd=-4.9（上限 3.0）时下界被压成 0 —— 允许不修正，下一步仍在界外，
    正是这一整套要消除的行为。而"保证有解"另有出路，见 test_box_is_never_empty。
    """
    _, M = _case()
    qd = np.array([9.0, -9.0, 0.0, 0.0, 0.0, 0.0])
    lo, hi = _feasible_acceleration_box(np.zeros(6), qd, M, _limits())
    # joint1 正跑在 +9（上限 3）-> 必须【负】加速；joint2 反之
    assert hi[0] < 0.0, f"joint1 已超速 +9，上界应为负值，实际 {hi[0]}"
    assert lo[1] > 0.0, f"joint2 已超速 -9，下界应为正值，实际 {lo[1]}"
    # 且一步之内必须回到包线内
    dt = DT
    assert abs(qd[0] + hi[0] * dt) <= VLIM[0] + 1e-9
    assert abs(qd[1] + lo[1] * dt) <= VLIM[1] + 1e-9


# --------------------------------------------------------------------------
# 4. 配置一致性：控制器认为的硬夹必须等于 plant 真正执行的硬夹
# --------------------------------------------------------------------------

def _plant_velocity_limits():
    """复刻 torque_sim_arm.cc:InitFromConfig 的取值路径。"""
    motor = yaml.safe_load(
        open(os.path.join(REPO, "config", "motor_config.yaml"),
             encoding="utf-8"))["motor_models"]
    arm = yaml.safe_load(
        open(os.path.join(REPO, "config", "arm_config.yaml"),
             encoding="utf-8"))["arms"]["a_l1_gamma6"]["joints"]
    return [float(motor[j["model"]]["max_velocity"]) for j in arm]


def test_config_joint_velocity_matches_the_plant_clamp():
    """smc_params_whole6.yaml 的 limits.joint_velocity 必须逐轴等于
    plant 实际用的 motor_config.yaml max_velocity。

    这两处一旦漂移，控制器就会按一个【不存在】的包线解算：给大了等于没修，
    给小了等于凭空拖慢轨迹。arm_version 见 launch/task_space_smc_whole6.launch.py
    的 'gamma6' -> arm_config.yaml 的 a_l1_gamma6。
    """
    cfg = yaml.safe_load(
        open(os.path.join(REPO, "task_space_smc", "config",
                          "smc_params_whole6.yaml"), encoding="utf-8"))
    declared = cfg["limits"]["joint_velocity"]
    plant = _plant_velocity_limits()
    assert len(declared) == len(plant) == 6
    assert [float(v) for v in declared] == plant, (
        f"控制器 {declared} != plant {plant}")


def test_urdf_position_limits_are_not_looser_than_the_plant():
    """制动约束用的是 URDF 限位（模型自带）。它必须【不宽于】plant 用的
    arm_config 限位，否则控制器会朝一个 plant 更早夹住的位置刹车 —— 刹不住。

    实测两者确有差异：joint2 URDF 上限 1.57、arm_config 2.0（URDF 更紧，
    保守，安全）。这条测试盯住这个方向不被反转。
    """
    import pinocchio as pin
    model = pin.buildModelFromUrdf(
        os.path.join(REPO, "config", "arm6_tool.urdf"),
        pin.JointModelFreeFlyer())
    urdf_lo = np.asarray(model.lowerPositionLimit[7:], dtype=float)
    urdf_hi = np.asarray(model.upperPositionLimit[7:], dtype=float)
    arm = yaml.safe_load(
        open(os.path.join(REPO, "config", "arm_config.yaml"),
             encoding="utf-8"))["arms"]["a_l1_gamma6"]["joints"]
    plant_lo = np.array([float(j["lower_limit"]) for j in arm])
    plant_hi = np.array([float(j["upper_limit"]) for j in arm])
    assert np.all(urdf_lo >= plant_lo - 1e-9), f"{urdf_lo} vs {plant_lo}"
    assert np.all(urdf_hi <= plant_hi + 1e-9), f"{urdf_hi} vs {plant_hi}"
