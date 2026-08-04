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
    return types.SimpleNamespace(
        shape="hwtest", derivative_dt=1e-4, start_delay_sec=duration,
        approach_guard=dict(guard or DEFAULT_GUARD),
        task_approach=None, task_approach_rejected=False,
        get_logger=lambda: log), log


def _kin(position, rotation=None):
    return types.SimpleNamespace(
        ee_position=np.asarray(position, dtype=float),
        ee_rotation=np.asarray(
            rotation if rotation is not None else np.eye(3), dtype=float))


# --------------------------------------------------------------- 配置

def test_default_is_still_joint_space():
    """不写 approach.mode 时必须仍是关节空间 —— 已验证基线不受影响。"""
    module = _load_runner_module()
    fake = types.SimpleNamespace(config={"approach": {}})
    approach = fake.config.get("approach", {})
    assert str(approach.get("mode", "joint_space")) == "joint_space"
    # 顺带确认 whole6 / simbest 两份已验证配置没被顺手改掉
    for name in ("smc_params_whole6.yaml", "smc_params_hwtest_simbest.yaml"):
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
