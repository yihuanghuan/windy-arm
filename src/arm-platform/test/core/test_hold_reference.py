"""``hold`` 形状（末端世界系定点保持）的回归测试。

守住的核心事实：
  * 参考量与时间【严格】无关 —— 位置、姿态是常量，一阶二阶导恒为零。
    这是整个 demo 的立论基础：末端偏差全部来自基座运动没补干净，
    没有轨迹跟踪误差混在里面。
  * `set_hold_reference` 能把目标改成运行期实测的位姿，且拒绝非法姿态。
  * 一个不动的点不产生静态预览折线。
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

from task_space_smc.core.trajectory_reference import (  # noqa: E402
    drawing_segments, hold_reference, reference_sample, set_hold_reference,
    traj)


@pytest.fixture(autouse=True)
def _restore_hold_pose():
    """hold 目标是模块级状态。任何一条测试改了它都要还回去，
    否则测试之间会互相污染 —— 而且是"单独跑过、全套跑挂"那种。"""
    position, rotation = hold_reference()
    yield
    set_hold_reference(position, rotation)


def _rotation_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def test_reference_is_constant_and_all_derivatives_are_zero():
    set_hold_reference([0.31, 0.02, 0.25], _rotation_z(0.3))
    first = reference_sample(0.0, "hold")
    for t in (0.0, 0.37, 12.5, 1e4):
        sample = reference_sample(t, "hold")
        assert np.allclose(sample.position_w, [0.31, 0.02, 0.25])
        assert np.allclose(sample.rotation_w, first.rotation_w)
        assert np.allclose(sample.linear_velocity_w, 0.0, atol=1e-12)
        assert np.allclose(sample.angular_velocity_w, 0.0, atol=1e-12)
        assert np.allclose(sample.linear_acceleration_w, 0.0, atol=1e-9)
        assert np.allclose(sample.angular_acceleration_w, 0.0, atol=1e-9)


def test_set_hold_reference_moves_the_target():
    set_hold_reference([0.28, -0.01, 0.30])
    assert np.allclose(reference_sample(5.0, "hold").position_w,
                       [0.28, -0.01, 0.30])
    position, _ = hold_reference()
    assert np.allclose(position, [0.28, -0.01, 0.30])


def test_rotation_only_target_keeps_the_previous_position():
    set_hold_reference([0.30, 0.0, 0.26], np.eye(3))
    set_hold_reference([0.30, 0.0, 0.26], _rotation_z(0.5))
    assert np.allclose(reference_sample(0.0, "hold").rotation_w,
                       _rotation_z(0.5))


@pytest.mark.parametrize("bad_rotation", [
    np.diag([1.0, 1.0, -1.0]),                       # 镜像：det = -1
    np.array([[1.0, 0.5, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),   # 非正交
])
def test_illegal_rotation_is_rejected(bad_rotation):
    """镜像阵能通过正交性检查却让姿态左右翻转 —— 交给控制律就是一个
    永远追不到的目标，而且很难从日志看出来。必须当场抛错。"""
    with pytest.raises(ValueError):
        set_hold_reference([0.3, 0.0, 0.26], bad_rotation)


def test_non_finite_target_is_rejected():
    with pytest.raises(ValueError):
        set_hold_reference([0.3, float("nan"), 0.26])


def test_hold_has_no_static_preview_path():
    """一个不动的点没有路径。返回空列表而不是长度为零的折线：后者在 RViz
    里是几千个重合点的退化 Marker，只会盖住真正要看的实际轨迹。"""
    assert drawing_segments("hold") == []


def test_hold_period_is_only_a_default_runtime():
    """保持没有周期。这个值只是 duration_sec:=0 时的缺省运行时长。"""
    assert traj.ee_trajectory_period("hold") == pytest.approx(
        traj.hold_period())
    assert traj.hold_period() > 0.0


def test_hold_is_pen_down_so_the_actual_path_gets_drawn():
    """实际轨迹（红线）靠 pen_down 才会被记录。保持得好不好，
    看的就是那团红点散成多大。"""
    sample = reference_sample(3.0, "hold")
    assert sample.pen_down is True


def test_adding_hold_did_not_disturb_the_existing_shapes():
    """纯增量：hwtest 的圆心、半径、周期一个都不能变。"""
    assert traj.ee_trajectory_period("hwtest") == pytest.approx(
        2.0 * np.pi * 0.04 / 0.02, rel=1e-6)
    start = traj.ee_trajectory_sample(0.0, "hwtest")
    assert np.allclose(start.position, [0.30, 0.04, 0.26])
