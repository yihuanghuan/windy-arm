"""可视化标记的发布成本必须【封顶】，不能随运行时长增长。

背景：原实现每 visualization_stride 拍把整条已走轨迹重建成 LINE_STRIP 发一次，
点数每次 +1 且无上限。第 19 次上机实测（hw04，60 s，rviz:=false）：

    控制周期 中位 5.002 ms，p99 10.014 ms（整拍丢失），最大 23.9 ms
    丢拍 124 次 / 60 s（1.07%），且随时间单调增长：
        0-10 s 零次 -> 最后 5 s 有 52 次
    【52.4% 的丢拍落在 idx % 7 == 0 上，随机只应 14.3%】
    （stride 取 6/8 无富集：21.0% vs 16.7%、10.5% vs 12.5%）

7 就是 visualization_stride（30 Hz 可视化 / 200 Hz 控制）。
操作者的观察是"一顿一顿的移动，像发信号发一下停一下"。

这个文件钉住两条：抽稀函数本身正确，以及点数上限确实能把成本压在预算内。
"""

import numpy as np
import pytest

from task_space_smc.whole_task_smc_ros_runner import (
    MAX_PUBLISHED_PATH_POINTS, decimate_path)

# 实测：构造一个 geometry_msgs/Point 约 0.75 µs（含字段赋值）。
# 用它换算发布成本，避免测试依赖 ROS 运行时。
SECONDS_PER_POINT = 0.75e-6
CONTROL_PERIOD_SEC = 1.0 / 200.0


def test_short_path_is_returned_unchanged():
    pts = [np.array([float(i), 0.0, 0.0]) for i in range(10)]
    out = decimate_path(pts, MAX_PUBLISHED_PATH_POINTS)
    assert out is pts, "未超上限时不该复制，那是每 7 拍都要付的开销"


def test_long_path_is_capped():
    pts = [np.array([float(i), 0.0, 0.0]) for i in range(50000)]
    out = decimate_path(pts, MAX_PUBLISHED_PATH_POINTS)
    assert len(out) <= MAX_PUBLISHED_PATH_POINTS


def test_endpoints_are_preserved():
    """抽稀不能丢首尾 —— 丢了尾点就看不到机械臂当前在哪。"""
    for n in (601, 1000, 11763, 50000):
        pts = [np.array([float(i), 0.0, 0.0]) for i in range(n)]
        out = decimate_path(pts, MAX_PUBLISHED_PATH_POINTS)
        assert np.allclose(out[0], pts[0]), f"n={n} 丢了首点"
        assert np.allclose(out[-1], pts[-1]), f"n={n} 丢了尾点"


def test_decimation_covers_the_whole_path_not_just_the_tail():
    """必须是均匀抽稀，不是截断 —— 截断会丢掉早期轨迹。"""
    n = 20000
    pts = [np.array([float(i), 0.0, 0.0]) for i in range(n)]
    out = decimate_path(pts, MAX_PUBLISHED_PATH_POINTS)
    xs = np.array([p[0] for p in out])
    # 中位数应当落在整条路径的中部，截断的话会挤在尾部
    assert 0.3 * n < np.median(xs) < 0.7 * n, (
        f"抽稀后中位 {np.median(xs):.0f} 不在路径中部，看起来是截断而非抽稀")
    assert np.all(np.diff(xs) > 0), "抽稀必须保序"


def test_publish_cost_stays_within_the_control_period():
    """封顶后的重建成本必须远小于一个控制周期，且不随路径增长。"""
    capped = MAX_PUBLISHED_PATH_POINTS * SECONDS_PER_POINT
    assert capped < 0.25 * CONTROL_PERIOD_SEC, (
        f"上限 {MAX_PUBLISHED_PATH_POINTS} 点 -> 约 {capped * 1000:.2f} ms，"
        f"超过控制周期 {CONTROL_PERIOD_SEC * 1000:.1f} ms 的 25%。"
        "第 19 次上机就是被这一项挤到 1.07% 的整拍丢失。")


@pytest.mark.parametrize("n", [1660, 11763, 100000])
def test_cost_is_flat_in_path_length(n):
    """无论跑多久，发布点数都不变 —— 这是这次修复的全部意义。"""
    pts = [np.array([float(i), 0.0, 0.0]) for i in range(n)]
    assert len(decimate_path(pts, MAX_PUBLISHED_PATH_POINTS)) \
        <= MAX_PUBLISHED_PATH_POINTS


def test_degenerate_limits_still_give_a_drawable_line():
    pts = [np.array([float(i), 0.0, 0.0]) for i in range(100)]
    for limit in (0, 1, 2):
        out = decimate_path(pts, limit)
        assert len(out) >= 2, f"limit={limit} 返回了画不出折线的点数"
