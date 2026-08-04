"""控制节点本身必须可导入。

守住一个真实发生过的空白：2026-08-03 一次字符串替换把 import 行改坏
（`from ...handover_blend import HandoverBlend` 是
 `... import HandoverBlend, trajectory_clock` 的前缀，替换后 `, trajectory_clock`
被接到了新加的 loop_delay import 行上），**332 项测试全绿**，直到 ROS 仿真
起不来才发现：

    ImportError: cannot import name 'trajectory_clock' from 'task_space_smc.core.loop_delay'

原因是所有既有测试都只 import 子模块或读源码文本，没有一条真正加载控制节点。
"""

import importlib.util
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

RUNNER = os.path.join(ROOT, "task_space_smc", "hw", "whole_task_smc_ros_runner.py")


@pytest.mark.skipif(importlib.util.find_spec("rclpy") is None,
                    reason="没有 ROS 环境")
def test_runner_module_imports_cleanly():
    """真正加载模块 —— 语法检查和读源码都发现不了 import 名字写错。"""
    spec = importlib.util.spec_from_file_location("_runner_import_check", RUNNER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)          # 出错就直接抛，不吞异常


@pytest.mark.skipif(importlib.util.find_spec("rclpy") is None,
                    reason="没有 ROS 环境")
def test_runner_exposes_the_pieces_it_wires_together():
    """节点用到的几个模块必须都在它的命名空间里，防止 import 行被改半截。"""
    spec = importlib.util.spec_from_file_location("_runner_symbol_check", RUNNER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for name in ("HandoverBlend", "trajectory_clock", "CommandDelayBuffer",
                 "FeedbackDelayModel", "resolve_simulation_delay",
                 "resolve_estimator",
                 "JointVelocityFilter", "FrictionFeedforwardConfig",
                 "stiction_feedforward", "residual_deadband",
                 "approach_torque_stiffness"):
        assert hasattr(module, name), f"控制节点里缺少 {name}"


def test_every_task_space_smc_module_imports():
    """逐个按【包路径】导入 task_space_smc 下的模块。

    必须用 importlib.import_module 而不是按文件路径加载 —— 后者会破坏包内的
    相对导入，报出一堆与真实 import 无关的假失败（我第一版就是这么写错的）。
    """
    import glob
    import importlib

    failures = []
    # 2026-08-04 分层后模块在 task_space_smc/{core,sim,hw}/ 下，模块名要带桶名。
    # 用目录名而不是写死一张表 —— 新增桶或搬动模块时这条扫描自动跟上。
    pattern = os.path.join(ROOT, "task_space_smc", "*", "*.py")
    for path in sorted(glob.glob(pattern)):
        name = os.path.basename(path)[:-3]
        bucket = os.path.basename(os.path.dirname(path))
        if name.startswith("_") or name == "whole_task_smc_ros_runner":
            continue
        if bucket in ("config", "data", "docs", "__pycache__"):
            continue
        try:
            importlib.import_module(f"task_space_smc.{bucket}.{name}")
        except ImportError as exc:            # 缺 ROS/可选依赖不算失败
            if any(k in str(exc) for k in ("rclpy", "dummy_interface", "serial")):
                continue
            failures.append(f"{name}: {exc}")
    assert not failures, "以下模块导入失败：\n  " + "\n  ".join(failures)
