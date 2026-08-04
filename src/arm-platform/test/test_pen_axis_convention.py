"""笔轴约定必须与轨迹几何一致。

【这个测试的存在理由】
pen_axis_local 曾被设成 [0,0,1]（机器人学的通常约定），但本机的轨迹生成器
用的是末端 x 轴（RViz 红轴），由物理笔架的安装方向决定。
    竖直板布局(vertical)沿用 arm.urdf 的历史约定，与此不同，故本测试只覆盖
    当前生效的布局。

取错轴的后果不是精度下降，而是【约束反了】：5D 任务会放开该锁死的笔尖指向，
锁死无所谓的绕笔自转。当时 130 个测试全过、离线仿真报"笔轴误差 0.0159 rad
几乎不变" —— 因为 pen_axis_error() 用的是同一个错轴，自己验自己。

所以这里【不用控制器的任何量自验】，只拿轨迹本身的几何做基准：落笔时
期望姿态的笔轴必须垂直于该笔画所在的平面。
"""

import os
import sys

import numpy as np
import pytest
import yaml

# 【必须在 import 轨迹模块之前设】—— 该模块在【模块级】读这个环境变量。
# 6 关节实机构型用 horizontal；默认 vertical 是 7 关节的历史布局。
os.environ.setdefault("MB_STATION_LAYOUT", "horizontal")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from task_space_smc import trajectory_reference as TR  # noqa: E402


CONFIGS = ("smc_params_whole6.yaml", "smc_params_hwtest.yaml")


def _config_pen_axis(name="smc_params_whole6.yaml"):
    path = os.path.join(
        os.path.dirname(__file__), "..", "task_space_smc", "config", name)
    with open(path, "r", encoding="utf-8") as handle:
        cfg = yaml.safe_load(handle)
    return np.asarray(cfg["task"]["pen_axis_local"], dtype=float)


def test_all_six_joint_configs_agree_on_the_pen_axis():
    """两份 6 关节配置的笔轴约定必须一致，否则换配置就换了物理含义。"""
    axes = {n: _config_pen_axis(n) for n in CONFIGS}
    ref = axes[CONFIGS[0]]
    for name, axis in axes.items():
        assert np.allclose(axis, ref), (
            f"{name} 的 pen_axis_local={axis.tolist()} 与 "
            f"{CONFIGS[0]} 的 {ref.tolist()} 不一致")


def test_code_defaults_match_the_config():
    """代码默认值必须与配置一致 —— 配置漏写这一项时不能静默用回旧约定。"""
    from task_space_smc.redundancy_resolution import pen_axis_task, pen_axis_error
    import inspect
    want = _config_pen_axis()
    for fn in (pen_axis_task, pen_axis_error):
        default = np.asarray(
            inspect.signature(fn).parameters["axis_local"].default, dtype=float)
        assert np.allclose(default, want), (
            f"{fn.__name__} 的默认笔轴 {default.tolist()} "
            f"与配置的 {want.tolist()} 不一致")


def _stroke_samples():
    """按 stroke_id 分组的落笔采样：{stroke: [(position, rotation), ...]}."""
    groups: dict[int, list] = {}
    for t in np.linspace(0.0, 73.0, 900):
        sample = TR.reference_sample(float(t), "whole")
        stroke = int(sample.stroke_id)
        if stroke < 0:
            continue
        groups.setdefault(stroke, []).append((
            np.asarray(sample.position_w, dtype=float).reshape(3),
            np.asarray(sample.rotation_w, dtype=float).reshape(3, 3),
        ))
    return {k: v for k, v in groups.items() if len(v) >= 12}


def test_pen_axis_is_normal_to_each_drawing_plane():
    """每个笔画都落在一个平面上，笔轴必须是该平面的法向。

    平面法向由笔画点云的最小奇异方向给出 —— 与控制器无关的独立基准。
    """
    axis_local = _config_pen_axis()
    checked = 0
    for stroke, samples in _stroke_samples().items():
        points = np.array([p for p, _ in samples])
        centered = points - points.mean(axis=0)
        _, sv, vt = np.linalg.svd(centered)
        # 【必须要求笔画真正张成 2 维】。直线笔画（如 windylab 的 'l'，
        # stroke 207，sv[1]=0.0000）落在无穷多个平面上，SVD 给出的"法向"
        # 是垂直于该直线的平面内的任意方向 —— 拿它当基准会误判。
        if sv[1] < 1e-2 or sv[2] > 1e-6 * max(sv[0], 1e-12):
            continue
        normal = vt[2]
        for _, rotation in samples:
            pen = rotation @ axis_local
            pen = pen / np.linalg.norm(pen)
            # |cos| 而非 cos：法向的正负号由 SVD 任意给出。
            alignment = abs(float(pen @ normal))
            assert alignment > 0.999, (
                f"stroke {stroke}: 笔轴与书写平面法向夹角 "
                f"{np.degrees(np.arccos(min(alignment, 1.0))):.1f}°，"
                f"应为 0°。pen_axis_local={axis_local.tolist()} 取错了轴 —— "
                f"笔轴由物理笔架的安装方向决定，当前应为末端 x 轴")
        checked += 1
    assert checked >= 3, f"只验到 {checked} 个共面笔画，样本不足以说明问题"


def test_the_conventional_z_axis_would_fail_this_check():
    """反向验证：证明这个测试确实能抓住当初那个错误。

    没有这一条，上面的断言可能因为过松而永远通过。
    """
    normals = []
    pens_z = []
    for stroke, samples in _stroke_samples().items():
        points = np.array([p for p, _ in samples])
        _, sv, vt = np.linalg.svd(points - points.mean(axis=0))
        if sv[1] < 1e-2 or sv[2] > 1e-6 * max(sv[0], 1e-12):
            continue
        normals.append(vt[2])
        rotation = samples[len(samples) // 2][1]
        pens_z.append(rotation @ np.array([0.0, 0.0, 1.0]))
    assert normals, "没有找到共面笔画"
    worst = max(abs(float(p @ n)) for p, n in zip(pens_z, normals))
    assert worst < 0.5, (
        "末端 z 轴竟然也垂直于书写平面，说明这个检查区分不了两种约定")
