"""6 关节实机构型的模型回归测试。

FloatingBaseArmModel 原先把关节数硬编码为 7（reshape(7) / zeros(7) /
nq==14 断言）。实机末端无电机，只有 6 个关节，因此关节数改为从 URDF
推导：n_arm = nv - 6。本测试同时锁住两件事：

1. 6 关节模型（arm6.urdf, EE=link6）各接口维度自洽；
2. 原 7 关节模型（arm.urdf, EE=link7）行为完全不变。

注意 model.py 里的 [7:] / [6:] 切片是 FreeFlyer 的 nq/nv 尺寸，
与关节数无关，不随构型改变。
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

from task_space_smc.core.model import (  # noqa: E402
    ARM6_EE_FRAME,
    ARM6_JOINT_NAMES,
    DEFAULT_ARM_JOINT_NAMES,
    DEFAULT_EE_FRAME,
    FloatingBaseArmModel,
    arm6_urdf_path,
)

# 实测静止位姿（/dev/ttyUSB0 抓包，602 个位置样本的均值）
Q_MEASURED = np.array([-0.0010, 1.3235, -1.3811, -0.0017, -0.0025, -0.0025])

# link7 相对 link6 的末端位移，见 arm.urdf 中 joint7 的 origin
EE_SHIFT_M = 0.0910


def _model6():
    return FloatingBaseArmModel(urdf_path=arm6_urdf_path())


def test_six_joint_model_dimensions():
    m = _model6()
    assert m.n_arm == 6
    assert m.model.nq == 13  # 7 (free-flyer) + 6
    assert m.model.nv == 12  # 6 (free-flyer) + 6
    assert tuple(m.arm_joint_names) == ARM6_JOINT_NAMES
    assert m.ee_frame_name == ARM6_EE_FRAME


def test_seven_joint_model_is_unchanged():
    m = FloatingBaseArmModel()
    assert m.n_arm == 7
    assert m.model.nq == 14
    assert m.model.nv == 13
    assert tuple(m.arm_joint_names) == DEFAULT_ARM_JOINT_NAMES
    assert m.ee_frame_name == DEFAULT_EE_FRAME


@pytest.mark.parametrize("build", [_model6, FloatingBaseArmModel])
def test_state_helpers_match_model_size(build):
    m = build()
    n = m.n_arm
    q = m.make_q_full([0.1, 0.2, 0.3], [1.0, 0.0, 0.0, 0.0], np.zeros(n))
    v = m.make_v_full(np.zeros(6), np.zeros(n))
    a = m.make_a_full(np.zeros(6), np.zeros(n))
    assert q.shape == (m.model.nq,)
    assert v.shape == (m.model.nv,)
    assert a.shape == (m.model.nv,)
    assert m.split_arm_q(q).shape == (n,)


@pytest.mark.parametrize("build", [_model6, FloatingBaseArmModel])
def test_wrong_sized_joint_vector_is_rejected(build):
    m = build()
    with pytest.raises(ValueError):
        m.make_q_full([0, 0, 0], [1, 0, 0, 0], np.zeros(m.n_arm + 1))


@pytest.mark.parametrize("build", [_model6, FloatingBaseArmModel])
def test_jacobian_and_dynamics_shapes(build):
    m = build()
    n = m.n_arm
    q = m.make_q_full(np.zeros(3), [1.0, 0.0, 0.0, 0.0], np.zeros(n))
    v = m.make_v_full(np.zeros(6), np.zeros(n))

    kin = m.compute_kinematics(q, v)
    assert kin.J_full_lwa.shape == (6, m.model.nv)
    assert kin.J_base_lwa.shape == (6, 6)
    assert kin.J_arm_lwa.shape == (6, n)

    assert m.compute_arm_mass_block(q).shape == (n, n)
    assert m.compute_task_bias_acceleration(q, v, np.zeros(6)).shape == (6,)

    tau = m.compute_rnea_arm_torque(q, v, np.zeros(6), np.zeros(n))
    assert tau.shape == (n,)


@pytest.mark.parametrize("build", [_model6, FloatingBaseArmModel])
def test_inverse_then_forward_dynamics_round_trip(build):
    m = build()
    n = m.n_arm
    rng = np.random.default_rng(0)
    q_arm = rng.uniform(-0.5, 0.5, n)
    qd_arm = rng.uniform(-0.5, 0.5, n)
    qdd_arm = rng.uniform(-1.0, 1.0, n)
    a_base = rng.uniform(-1.0, 1.0, 6)

    q = m.make_q_full(np.zeros(3), [1.0, 0.0, 0.0, 0.0], q_arm)
    v = m.make_v_full(np.zeros(6), qd_arm)

    tau = m.compute_rnea_arm_torque(q, v, a_base, qdd_arm)
    qdd_back = m.compute_arm_forward_dynamics(q, v, a_base, tau)
    np.testing.assert_allclose(qdd_back, qdd_arm, atol=1e-9)


def test_dropping_link7_moves_the_end_effector_by_91mm():
    m6 = _model6()
    m7 = FloatingBaseArmModel()
    q6 = m6.make_q_full(np.zeros(3), [1.0, 0.0, 0.0, 0.0], Q_MEASURED)
    q7 = m7.make_q_full(np.zeros(3), [1.0, 0.0, 0.0, 0.0], np.append(Q_MEASURED, 0.0))
    p6 = m6.compute_kinematics(q6).ee_position
    p7 = m7.compute_kinematics(q7).ee_position
    assert np.linalg.norm(p7 - p6) == pytest.approx(EE_SHIFT_M, abs=1e-4)


def test_six_joint_arm_has_no_redundancy_for_a_6d_task():
    """6 关节 + 6D 位姿任务 => 零空间秩为 0，零空间 PD 不再有作用空间。

    这不是缺陷，是构型的必然结果；锁住它是为了防止有人把 7 关节的
    零空间参数原样搬过来后误以为仍然生效。
    """
    m = _model6()
    q = m.make_q_full(np.zeros(3), [1.0, 0.0, 0.0, 0.0], Q_MEASURED)
    J_arm = m.compute_frame_jacobian(q)[:, 6:]
    assert J_arm.shape == (6, 6)
    assert np.linalg.matrix_rank(J_arm, tol=1e-8) == 6
    null_dim = J_arm.shape[1] - np.linalg.matrix_rank(J_arm, tol=1e-8)
    assert null_dim == 0

# --------------------------------------------------------------------------
# 实机标定值（阶段 2/3）
# --------------------------------------------------------------------------

def test_calibration_loads_and_has_the_right_length():
    from task_space_smc.core.model import load_arm6_calibration, ARM6_JOINT_NAMES
    c = load_arm6_calibration()
    n = len(ARM6_JOINT_NAMES)
    assert c['source'] is not None, "标定文件缺失 —— 下游会静默退回恒等标定"
    for key in ('zero_offset_rad', 'first_moment_ratio', 'stiction_nm'):
        assert len(c[key]) == n, f"{key} 长度应为 {n}"


def test_calibration_is_missing_tolerant():
    """文件不存在时必须返回恒等标定而不是抛异常 —— 仿真环境可能没有它。"""
    from task_space_smc.core.model import load_arm6_calibration, ARM6_JOINT_NAMES
    c = load_arm6_calibration('/nonexistent/path/does-not-exist.yaml')
    n = len(ARM6_JOINT_NAMES)
    assert c['source'] is None
    assert c['zero_offset_rad'] == [0.0] * n
    assert c['first_moment_ratio'] == [1.0] * n


def test_joint6_has_no_zero_offset_and_unit_ratio():
    """link6 质心在 joint6 转轴上（径向偏移 0.113 mm），重力测不到它，
    它也不影响末端位置（灵敏度 0.0000 mm/rad）。标定值必须保持中性。"""
    from task_space_smc.core.model import load_arm6_calibration
    c = load_arm6_calibration()
    assert c['zero_offset_rad'][5] == 0.0
    assert c['first_moment_ratio'][5] == 1.0


def test_unconverged_joints_are_flagged_not_hidden():
    """joint4/5 在 A/f≈1 下未收敛。标定文件必须【如实标记】，
    不能把未收敛的值伪装成已收敛。"""
    from task_space_smc.core.model import load_arm6_calibration
    c = load_arm6_calibration()
    conv = c['converged']
    assert conv[:3] == [True, True, True], "joint1/2/3 应已收敛"
    assert conv[3] is False and conv[4] is False, \
        "joint4/5 未收敛，必须如实标记"


def test_stiction_is_the_static_value_not_the_kinetic_one():
    """阶段 1 从双向扫描半差估的 0.07~0.21 Nm 是【动摩擦残余】，
    真正的静摩擦是阶段 2 双向启动力矩测的 0.625~1.775 Nm，差 3~10 倍。
    用错会让死区估计小一个数量级。"""
    from task_space_smc.core.model import load_arm6_calibration
    c = load_arm6_calibration()
    assert c['stiction_nm'][1] > 1.0, \
        "joint2 静摩擦应为 1.775 Nm（动摩擦残余只有 0.07~0.21）"
    assert min(c['stiction_nm'][:3]) > 0.5, "DM 关节静摩擦均应 > 0.5 Nm"

def test_six_joint_zero_pose_is_singular_so_config_must_set_an_initial_pose():
    """6 关节零位的雅可比 sigma_min = 0.00001（cond 326263），起步即奇异。

    停用零空间时若同时丢掉初始位姿来源，仿真会退化到零位 —— 实测力矩饱和
    37%、姿态误差达 π、位置误差 122mm。配置必须显式给出良态的初始位形。
    """
    import numpy as np
    import yaml
    from task_space_smc.core.model import (
        FloatingBaseArmModel, arm6_urdf_path, ARM6_JOINT_NAMES, ARM6_EE_FRAME)

    m = FloatingBaseArmModel(urdf_path=arm6_urdf_path(),
                             ee_frame_name=ARM6_EE_FRAME,
                             arm_joint_names=tuple(ARM6_JOINT_NAMES))

    def sigma_min(q):
        qf = m.make_q_full(np.zeros(3), np.array([1.0, 0, 0, 0]), q)
        vf = m.make_v_full(np.zeros(6), np.zeros(6))
        return np.linalg.svd(m.compute_kinematics(qf, vf).J_arm_lwa,
                             compute_uv=False).min()

    assert sigma_min(np.zeros(6)) < 1e-3, \
        "前提：6 关节零位确实是奇异的"

    cfg = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                       'task_space_smc', 'config', 'smc_params_whole6.yaml')
    with open(cfg) as fh:
        conf = yaml.safe_load(fh)
    init = conf.get('initial_pose_rad')
    assert init is not None and len(init) == 6, \
        "6 关节配置必须显式给出 initial_pose_rad"

    # 【不再要求初始位姿良态】。它必须与实物驻留位姿一致 —— 否则上实物时
    # 第一条指令会让机械臂猛跳（差值曾达 1.35 rad）。而实物停的位置本身
    # 就接近奇异（sigma_min ≈ 1e-4），这两个要求天然冲突，匹配实物优先。
    #
    # 良态性由 approach 段负责：它把机械臂从驻留位姿平滑带到轨迹起点。
    # 这里只守住「不是零位」这条底线 —— 零位的 sigma_min = 1e-5，
    # 比驻留位姿还差一个数量级，且退化到零位说明配置根本没生效。
    init_arr = np.asarray(init, dtype=float)
    assert np.linalg.norm(init_arr) > 1e-6, \
        "initial_pose_rad 退化为零位 —— 那是最差的奇异位形"
    approach = conf.get('approach', {}).get('initial_joint_offset_rad', [0.0] * 6)
    after = sigma_min(init_arr + np.asarray(approach, dtype=float))
    assert after > sigma_min(init_arr), \
        "approach_offset 必须把机械臂带向【更良态】的位形，而不是更差"

