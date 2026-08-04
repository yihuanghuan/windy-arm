"""控制器配置的加载与装配。

放在 core 而不是 sim 的理由（2026-08-04 目录分层时抽出来的）：
``controller_from_config`` / ``default_config_path`` / ``load_config`` 原先长在
``sim/offline_simulation.py`` 里，而 ``hw/whole_task_smc_ros_runner.py`` 与
``hw/torque_controller_node.py`` 都要用它们造控制器。分层之后那就成了
**实物层依赖仿真层**的倒挂 —— 实物运行器为了读一份 yaml 去 import 一整个离线
仿真 harness，既不合逻辑，也把 sim 里那些只有离线才需要的依赖拖进了实物路径。

这三个函数本身跟仿真没有任何关系，它们做的是"yaml -> SMCGains -> TaskSpaceSMC"，
是控制器装配，属于 core。``sim/offline_simulation.py`` 保留同名再导出，既有
调用方（含外部脚本与文档里的命令行）一个字不用改。

⚠️ ``default_config_path()`` 指向 ``task_space_smc/config/smc_params.yaml``。
   config 目录留在包根、没有跟着分层走，所以这里用 ``parents[1]`` 上溯出 core/。
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from .task_space_smc import SMCGains, TaskSpaceSMC


def _arr(config: dict, section: str, key: str, n: int) -> np.ndarray:
    return np.asarray(config[section][key], dtype=float).reshape(n)


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def default_config_path() -> str:
    # parents[0] = core/，parents[1] = task_space_smc/。config 在包根。
    return str(Path(__file__).resolve().parents[1] / "config" / "smc_params.yaml")


def controller_from_config(config: dict) -> TaskSpaceSMC:
    gains = SMCGains(
        lambda_p=_arr(config, "position", "lambda", 3),
        k_p=_arr(config, "position", "k", 3),
        eta_p=_arr(config, "position", "eta", 3),
        phi_p=_arr(config, "position", "phi", 3),
        lambda_R=_arr(config, "orientation", "lambda", 3),
        k_R=_arr(config, "orientation", "k", 3),
        eta_R=_arr(config, "orientation", "eta", 3),
        phi_R=_arr(config, "orientation", "phi", 3),
    )
    return TaskSpaceSMC(gains)
