"""实物 demo「基座浮动 + 末端位置保持」的回归测试。

这条 demo 的安全性建立在三件事上，每一件都在下面守住：
  * 保持目标是【接管那一拍实测的末端位姿】，所以第一拍的任务空间误差严格为
    零，机械臂不会先窜向一个谁也没要求的点；
  * 它没有就位段，因此起点奇异门必须在跟踪段的第一拍自己查，
    不能指望 `_plan_task_space_approach`（那条路根本不会走）；
  * 拒绝起跑时是【原地位置保持】而不是零力矩。协议没有失能帧，
    零力矩等于失力 —— 实测 joint2 失力后下沉 14.6°。
"""

import importlib.util
import os
import sys
import types

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

from task_space_smc.core.trajectory_reference import (  # noqa: E402
    hold_reference, reference_sample, set_hold_reference)

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
LAUNCH_DIR = os.path.join(ROOT, "launch")


def _load_runner_module():
    if importlib.util.find_spec("rclpy") is None:
        pytest.skip("没有 ROS 环境")
    spec = importlib.util.spec_from_file_location(
        "_runner_hold_demo",
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


@pytest.fixture(autouse=True)
def _restore_hold_pose():
    position, rotation = hold_reference()
    yield
    set_hold_reference(position, rotation)


def _rotation_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _kin(position, rotation=None, sigma_min=0.0425):
    return types.SimpleNamespace(
        ee_position=np.asarray(position, dtype=float),
        ee_rotation=np.asarray(
            rotation if rotation is not None else np.eye(3), dtype=float),
        J_arm_lwa=np.diag([1.0, 0.9, 0.8, 0.7, 0.6, float(sigma_min)]))


def _fake(module, offset=(0.0, 0.0, 0.0), q_ctrl=None):
    log = _Log()
    fake = types.SimpleNamespace(
        shape="hold", is_hold_shape=True, hold_capture_measured=True,
        _hold_captured=False, derivative_dt=1e-4, n_arm=6,
        reference_offset_local=np.asarray(offset, dtype=float),
        q_ctrl=np.asarray(
            q_ctrl if q_ctrl is not None else np.arange(6) * 0.1, dtype=float),
        _q_meas=np.asarray(
            q_ctrl if q_ctrl is not None else np.arange(6) * 0.1, dtype=float),
        track_start_q=np.zeros(6),
        orientation_mode="full", pen_axis_local=np.array([1.0, 0.0, 0.0]),
        _watchdog_tripped=False, _watchdog_trip_time=None,
        watchdog_reason="", _watchdog_hold_q=None, watchdog_hold_sec=1.0,
        config={"singularity": {"sigma_warn": 0.03, "sigma_critical": 0.012}},
        _resolve_track_start_nullspace_nominal=lambda: None,
        get_logger=lambda: log)
    fake._reference_sample = (
        lambda t: module.WholeTaskSmcRosRunner._reference_sample(fake, t))
    fake._start_singularity_reason = (
        lambda kin: module.WholeTaskSmcRosRunner._start_singularity_reason(
            fake, kin))
    return fake, log


# ------------------------------------------------------- 捕获实测位姿

def test_capture_puts_the_reference_exactly_on_the_measured_pose():
    """第一拍误差必须是【严格】零。

    差一点点都不行：SMC 的位置增益在边界层外有效值 151~187，
    1 cm 的初始残差就是 1.5~1.9 m/s² 的加速度需求 —— 接管瞬间一次猛冲。
    """
    module = _load_runner_module()
    fake, log = _fake(module)
    measured = _kin([0.2731, 0.0184, 0.2455], _rotation_z(0.42))
    module.WholeTaskSmcRosRunner._capture_hold_reference(fake, measured)

    ref = fake._reference_sample(0.0)
    assert np.allclose(ref.position_w, measured.ee_position, atol=1e-12)
    assert np.allclose(ref.rotation_w, measured.ee_rotation, atol=1e-12)
    assert fake._hold_captured is True
    assert any("保持目标（实测捕获）" in m for m in log.info_msgs)


def test_capture_compensates_the_configured_tool_offset():
    """配置带 reference_offset_local_m 时，_reference_sample 会把偏移加回去。
    捕获必须先把它减掉，否则"保持当前位姿"会保持到偏移之外的另一个点。"""
    module = _load_runner_module()
    offset = np.array([-0.084, 0.0, 0.035])
    fake, _ = _fake(module, offset=offset)
    measured = _kin([0.2731, 0.0184, 0.2455], _rotation_z(0.42))
    module.WholeTaskSmcRosRunner._capture_hold_reference(fake, measured)

    ref = fake._reference_sample(0.0)
    assert np.allclose(ref.position_w, measured.ee_position, atol=1e-12)
    # 存进轨迹模块的是【减掉偏移之后】的点，不是末端点本身。
    stored, _ = hold_reference()
    assert np.allclose(stored,
                       measured.ee_position - measured.ee_rotation @ offset)


def test_capture_takes_the_current_configuration_as_the_track_start():
    """保持目标就是当前位姿，那么跟踪段起点位形就是当前位形。
    再去解一遍 IK 只会挑到另一个"限位余量更大"的解，凭空造出一段位移。"""
    module = _load_runner_module()
    q = np.array([-0.06, 1.55, -1.40, 0.0, 0.69, 0.003])
    fake, _ = _fake(module, q_ctrl=q)
    module.WholeTaskSmcRosRunner._capture_hold_reference(
        fake, _kin([0.27, 0.02, 0.24]))
    assert np.allclose(fake.track_start_q, q)


def test_reference_stays_put_forever_after_capture():
    module = _load_runner_module()
    fake, _ = _fake(module)
    module.WholeTaskSmcRosRunner._capture_hold_reference(
        fake, _kin([0.27, 0.02, 0.24], _rotation_z(0.2)))
    for t in (0.0, 7.3, 600.0):
        sample = reference_sample(t, "hold")
        assert np.allclose(sample.position_w, [0.27, 0.02, 0.24])
        assert np.allclose(sample.linear_velocity_w, 0.0, atol=1e-12)


# ------------------------------------------------------- 起点奇异门

def test_singular_start_is_caught_even_without_an_approach_segment():
    """hold 没有就位段，奇异门只剩跟踪段第一拍这一次机会。

    实测 j5 = 0.17°（上电休息位形）时 sigma_min = 0.00019，
    比 sigma_critical 0.012 低 63 倍。保持段的起点位形是操作员摆出来的，
    不是配置里挑好的，更不能不查。
    """
    module = _load_runner_module()
    fake, log = _fake(module)
    reason = fake._start_singularity_reason(_kin([0.1, 0.0, 0.17],
                                                 sigma_min=0.00019))
    assert reason, "奇异起点必须给出拒绝理由"
    assert "wrist_prepose" in reason, "拒绝时必须告诉操作者下一步怎么做"


def test_healthy_start_is_not_rejected():
    module = _load_runner_module()
    fake, log = _fake(module)
    assert fake._start_singularity_reason(
        _kin([0.27, 0.02, 0.24], sigma_min=0.0425)) == ""
    assert not log.warn_msgs and not log.error_msgs


def test_rejection_holds_position_instead_of_going_limp():
    """拒绝起跑时发的是位置保持指令，不是零力矩。

    协议里没有使能 / 失能帧，零力矩就是失力；实测 joint2 失力后下沉 14.6°。
    """
    module = _load_runner_module()
    fake, log = _fake(module)
    q_meas = np.array([-0.06, 1.55, -1.40, 0.0, 0.69, 0.003])
    fake._q_meas = q_meas

    module.WholeTaskSmcRosRunner._reject_start_configuration(
        fake, 12.5, "起点位形奇异：sigma_min 太小")
    assert fake._watchdog_tripped is True
    assert fake._watchdog_trip_time == 12.5
    assert np.allclose(fake._watchdog_hold_q, q_meas)
    assert any("原地保持位置" in m for m in log.error_msgs)

    published = []
    fake.command_pub = types.SimpleNamespace(publish=published.append)
    fake.internal_plant = False
    fake.feedback_generation = 7
    fake.last_command_feedback_generation = -1
    from builtin_interfaces.msg import Time
    fake.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_msg=Time))
    module.WholeTaskSmcRosRunner._publish_motor_command(fake, None)

    assert len(published) == 1
    msg = published[0]
    assert msg.enable == 1, "位置保持必须是使能的，否则等于失力"
    assert np.allclose(msg.position, q_meas)
    assert np.allclose(msg.current, 0.0)
    assert all(p > 0.0 for p in msg.p), "保位增益不能是零"


# ------------------------------------------------------- launch 接线

def _launch_source(name):
    with open(os.path.join(LAUNCH_DIR, name), encoding="utf-8") as f:
        return f.read()


def test_demo_launch_pins_the_hardware_vicon_hold_combination():
    """这三个参数是这条 demo 的定义，不该是可以在命令行改错的自由项。"""
    src = _launch_source("vicon_hold_demo.launch.py")
    assert "'mode': 'hardware'" in src
    assert "'trajectory_shape': 'hold'" in src
    assert "'base_state_mode': 'vicon'" in src
    assert "'start_delay_sec': 0.0" in src


def test_demo_launch_defaults_to_not_moving_the_arm():
    """hardware_enabled 缺省 false：不显式打开，适配器输出恒为零。
    与 task_space_smc_whole6.launch.py 同一套约定。"""
    src = _launch_source("vicon_hold_demo.launch.py")
    i = src.index("DeclareLaunchArgument(\n            'hardware_enabled'")
    assert "default_value='false'" in src[i:i + 200]


def test_demo_launch_does_not_inject_an_analytic_base_on_top_of_vicon():
    """基座由动捕给。解析摆动半径必须是 0，否则两个基座会叠在一起。"""
    src = _launch_source("vicon_hold_demo.launch.py")
    assert "os.environ.setdefault('MB_BASE_RADIUS', '0.0')" in src
    assert "os.environ.setdefault('MB_BASE_Z_AMP', '0.0')" in src


def test_demo_launch_forwards_the_vrpn_endpoint():
    """一键启动 Vicon 时必须把服务器 IP 和端口都传给 VRPN launch。"""
    src = _launch_source("vicon_hold_demo.launch.py")
    assert "'port': LaunchConfiguration('vicon_port')" in src
    assert "DeclareLaunchArgument('vicon_server', default_value='192.168.200.60')" in src
    assert "DeclareLaunchArgument('vicon_port', default_value='3883')" in src


def test_demo_launch_locks_the_dedicated_hardware_hold_config():
    """hold 实物参数不能被遗留的仿真环境变量覆盖。"""
    src = _launch_source("vicon_hold_demo.launch.py")
    assert "smc_params_hold.yaml" in src
    assert "os.environ.get('WINDYLAB_SMC_CONFIG')" not in src


def test_vicon_base_mode_is_hardware_only():
    """analytic 基座下用 vicon 反馈是配置错误，必须当场抛错而不是静默回落。"""
    src = open(os.path.join(ROOT, "task_space_smc", "hw",
                            "whole_task_smc_ros_runner.py"),
               encoding="utf-8").read()
    i = src.index('self.base_state_mode = ')
    window = src[i:i + 600]
    assert "raise ValueError" in window
    assert "mode=hardware" in window or 'self.mode != "hardware"' in window
