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
