"""任务空间就位（approach.mode = task_space）的回归测试。

这里守住的核心事实：
  * 缺省仍是 joint_space —— 已验证的仿真基线不因为新增这条路而改变
  * 实物配置（smc_params_hwtest.yaml）确实切到了 task_space
  * 工作台护栏能拦下穿桌面、撞围栏、走太快三类规划
  * 被护栏拒绝时【一条力矩都不发】（靠驱动的指令超时保位兜底），
    而不是发一条"安全一点"的力矩
  * 就位段有看门狗，且它的位置判据吃的是任务空间误差
"""

import importlib.util
import os
import sys
import types

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

from task_space_smc.core.approach_reference import (  # noqa: E402
    TaskSpaceApproach, rotation_exp)

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
CONFIG_DIR = os.path.join(ROOT, "task_space_smc", "config")


def _load_runner_module():
    if importlib.util.find_spec("rclpy") is None:
        pytest.skip("没有 ROS 环境")
    spec = importlib.util.spec_from_file_location(
        "_runner_task_approach",
        os.path.join(ROOT, "task_space_smc", "hw",
                     "whole_task_smc_ros_runner.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _Log:
    def __init__(self):
        self.info_msgs, self.warn_msgs, self.error_msgs = [], [], []

    def info(self, m):
        self.info_msgs.append(str(m))

    def warning(self, m):
        self.warn_msgs.append(str(m))

    warn = warning

    def error(self, m):
        self.error_msgs.append(str(m))


DEFAULT_GUARD = dict(
    enabled=True, desk_plane_z=0.0, desk_edge_x=0.02, desk_clearance=0.08,
    fence_y=0.40, fence_height=0.331, fence_clearance=0.03,
    max_linear_speed=0.15, max_linear_accel=0.60)


def _fake(module, guard=None, duration=8.0):
    log = _Log()
    fake = types.SimpleNamespace(
        shape="hwtest", derivative_dt=1e-4, start_delay_sec=duration,
        _reference_sample=lambda t: module.reference_sample(t, "hwtest", 1e-4),
        approach_guard=dict(guard or DEFAULT_GUARD),
        task_approach=None, task_approach_rejected=False,
        orientation_mode="full", pen_axis_local=np.array([1.0, 0.0, 0.0]),
        n_arm=6, q_ctrl=np.zeros(6),
        config={"singularity": {"sigma_warn": 0.03, "sigma_critical": 0.012}},
        get_logger=lambda: log)
    # 奇异门是共用的：就位段和 hold 保持段（没有就位段）都走这一个实现。
    # 这里绑真方法而不是打桩，免得测试通过而被测的那条路已经换掉。
    fake._start_singularity_reason = (
        lambda kin: module.WholeTaskSmcRosRunner._start_singularity_reason(
            fake, kin))
    return fake, log


def _kin(position, rotation=None, sigma_min=0.0425):
    """缺省带一个【健康】的雅可比（sigma_min = j5=40° 的实测值），
    这样护栏类测试只考察工作台判据，不被奇异门抢答。"""
    return types.SimpleNamespace(
        ee_position=np.asarray(position, dtype=float),
        ee_rotation=np.asarray(
            rotation if rotation is not None else np.eye(3), dtype=float),
        J_arm_lwa=np.diag([1.0, 0.9, 0.8, 0.7, 0.6, float(sigma_min)]))


# --------------------------------------------------------------- 配置

def test_default_is_still_joint_space():
    """不写 approach.mode 时必须仍是关节空间 —— 已验证基线不受影响。"""
    module = _load_runner_module()
    fake = types.SimpleNamespace(config={"approach": {}})
    approach = fake.config.get("approach", {})
    assert str(approach.get("mode", "joint_space")) == "joint_space"
    # 6DOF 仿真配置不应带实物的 task-space approach 模式。
    for name in ("smc_params_whole6.yaml",):
        cfg = yaml.safe_load(open(os.path.join(CONFIG_DIR, name)))
        assert (cfg.get("approach") or {}).get("mode") is None, name


def test_hardware_config_uses_task_space_approach():
    cfg = yaml.safe_load(
        open(os.path.join(CONFIG_DIR, "smc_params_hwtest.yaml")))
    assert cfg["approach"]["mode"] == "task_space"
    g = cfg["approach"]["workspace_guard"]
    assert g["enabled"] is True
    # 护栏数值必须与 fk_gravity_verify 里那套实测常数一致，否则两处会漂
    assert g["desk_plane_z"] == 0.0
    assert g["desk_edge_x"] == pytest.approx(0.02)
    assert g["fence_height"] == pytest.approx(0.331)


def test_unknown_mode_raises_instead_of_silently_falling_back():
    """拼错的 mode 必须当场抛错。

    实物时段是串行的稀缺资源：静默回落到 joint_space 会让整次上机的记录
    无法解释 —— 日志上写着 task_space，实际跑的是另一条路。
    """
    src = open(os.path.join(ROOT, "task_space_smc", "hw",
                            "whole_task_smc_ros_runner.py"),
               encoding="utf-8").read()
    i = src.index("self.approach_mode")
    window = src[i:i + 600]
    assert "raise ValueError" in window, "approach.mode 没有做取值校验"
    assert "joint_space" in window and "task_space" in window


# --------------------------------------------------------------- 护栏

def test_plan_accepts_a_clean_path():
    module = _load_runner_module()
    fake, log = _fake(module)
    ok = module.WholeTaskSmcRosRunner._plan_task_space_approach(
        fake, _kin([0.2468, 0.0325, 0.1954]))
    assert ok is True
    assert fake.task_approach_rejected is False
    assert any("就位（任务空间 SMC）" in m for m in log.info_msgs)
    assert not log.error_msgs


def test_plan_rejects_a_path_that_grazes_the_desk():
    """起点压在 −x 侧、离台面只有 1 cm —— 直线一定贴着桌面走。"""
    module = _load_runner_module()
    fake, log = _fake(module)
    ok = module.WholeTaskSmcRosRunner._plan_task_space_approach(
        fake, _kin([-0.20, 0.03, 0.010]))
    assert ok is False
    assert fake.task_approach_rejected is True
    assert any("桌面" in m for m in log.error_msgs), log.error_msgs


def test_plan_rejects_a_path_that_hits_the_fence():
    module = _load_runner_module()
    fake, log = _fake(module)
    ok = module.WholeTaskSmcRosRunner._plan_task_space_approach(
        fake, _kin([-0.25, 0.55, 0.20]))
    assert ok is False
    assert any("围栏" in m for m in log.error_msgs), log.error_msgs


def test_plan_rejects_a_move_that_would_be_too_fast():
    """同样的距离，start_delay 太短就必须拒绝并提示加大它。"""
    module = _load_runner_module()
    fake, log = _fake(module, duration=0.4)
    ok = module.WholeTaskSmcRosRunner._plan_task_space_approach(
        fake, _kin([0.05, 0.30, 0.45]))
    assert ok is False
    assert any("start_delay_sec" in m for m in log.error_msgs), log.error_msgs


def test_guard_can_be_switched_off_but_says_so_loudly():
    module = _load_runner_module()
    g = dict(DEFAULT_GUARD, enabled=False)
    fake, log = _fake(module, guard=g)
    ok = module.WholeTaskSmcRosRunner._plan_task_space_approach(
        fake, _kin([-0.20, 0.03, 0.010]))
    assert ok is True
    assert any("护栏已关闭" in m for m in log.warn_msgs)


# --------------------------------------------------------------- 拒绝后的行为

def test_rejected_plan_publishes_no_torque_at_all():
    """被拒绝后必须【什么都不发】。

    不发力矩 -> student_arm_node 的 effort 指令超时（0.05 s）接管，用默认增益
    位置保持 —— 那是协议里唯一设计过的安全兜底。发一条"小一点"的力矩反而会
    把机械臂留在无人负责的状态。
    """
    module = _load_runner_module()
    fake, _ = _fake(module)
    fake.task_approach_rejected = True
    published = []
    fake._publish_motor_command = lambda m: published.append(m)
    fake._publish_base_tf = lambda bs: published.append("tf")
    out = module.WholeTaskSmcRosRunner._run_task_space_approach(
        fake, 0.1, 1.0)
    assert out is None
    assert published == [], "拒绝之后不应有任何下发或发布"


# --------------------------------------------------------------- 参考量

def test_approach_reference_starts_at_the_current_pose_so_error_starts_at_zero():
    """就位参考量从【当前位姿】起步，SMC 看到的初始误差因此是 0。

    这是任务空间就位相对"直接把参考量设成轨迹起点"的关键区别：后者第一拍
    就有 245 mm 误差乘 c0=216 -> 53 m/s² 的需求，实机上就是一次猛冲。
    """
    p0 = np.array([0.2468, 0.0325, 0.1954])
    a = TaskSpaceApproach(p0, np.eye(3), [0.30, 0.04, 0.26],
                          rotation_exp([0.1, -0.2, 0.05]), 8.0)
    s = a.sample(0.0)
    assert np.allclose(s.position, p0)
    assert np.allclose(s.linear_velocity, 0.0)
    # 末端停在轨迹起点上，速度归零 —— 交接时 handover_blend 面对的残差极小
    e = a.sample(8.0)
    assert np.allclose(e.position, [0.30, 0.04, 0.26])
    assert np.allclose(e.linear_velocity, 0.0)


# --------------------------------------------------------------- 起点奇异

def _kin_with_jacobian(position, J, rotation=None):
    return types.SimpleNamespace(
        ee_position=np.asarray(position, dtype=float),
        ee_rotation=np.asarray(
            rotation if rotation is not None else np.eye(3), dtype=float),
        J_arm_lwa=np.asarray(J, dtype=float))


def _sing_fake(module, sigma_min):
    """造一个 sigma_min 可控的 6×6 雅可比。"""
    J = np.diag([1.0, 0.9, 0.8, 0.7, 0.6, float(sigma_min)])
    fake, log = _fake(module)
    fake.orientation_mode = "full"
    fake.pen_axis_local = np.array([1.0, 0.0, 0.0])
    fake.n_arm = 6
    fake.q_ctrl = np.zeros(6)
    fake.config = {"singularity": {"sigma_warn": 0.03, "sigma_critical": 0.012}}
    return fake, log, J


def test_singular_start_pose_is_rejected_before_any_torque():
    """j5≈0 的腕部奇异起点必须拒绝就位。

    实测 j5=0.17° 时 sigma_min = 0.00019，比 sigma_critical 0.012 低 63 倍。
    在这种位形上伪逆会给出一个数，但那个数与期望的末端运动无关 ——
    机械臂会往任意方向窜。而这一步【不能】由任务空间 SMC 自己解决：
    它正是被奇异卡住的那一方。
    """
    module = _load_runner_module()
    fake, log, J = _sing_fake(module, 0.00019)
    ok = module.WholeTaskSmcRosRunner._plan_task_space_approach(
        fake, _kin_with_jacobian([0.108, 0.011, 0.170], J))
    assert ok is False
    assert fake.task_approach_rejected is True
    msg = "\n".join(log.error_msgs)
    assert "奇异" in msg
    assert "wrist_prepose" in msg, "拒绝时必须告诉操作者下一步怎么做"


def test_near_singular_start_warns_but_proceeds():
    module = _load_runner_module()
    fake, log, J = _sing_fake(module, 0.02)      # 介于 critical 与 warn 之间
    ok = module.WholeTaskSmcRosRunner._plan_task_space_approach(
        fake, _kin_with_jacobian([0.2468, 0.0325, 0.1954], J))
    assert ok is True
    assert any("接近奇异" in m for m in log.warn_msgs)
    assert not log.error_msgs


def test_healthy_start_pose_neither_warns_nor_errors():
    module = _load_runner_module()
    fake, log, J = _sing_fake(module, 0.0425)    # j5=40° 实测值
    ok = module.WholeTaskSmcRosRunner._plan_task_space_approach(
        fake, _kin_with_jacobian([0.2468, 0.0325, 0.1954], J))
    assert ok is True
    assert not log.warn_msgs and not log.error_msgs
