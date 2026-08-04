#!/usr/bin/env python3
"""Ablation study for whole-task SMC simulation injection terms."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from copy import deepcopy
from datetime import datetime
from pathlib import Path

import yaml

if __package__ in (None, ""):
    # 直接当脚本跑时把【包的父目录】放进 sys.path。2026-08-04 目录分层后
    # 模块下沉到 core/ sim/ hw/，原来写死的 package_dir.parent 会指到
    # task_space_smc/ 本身（少上溯一级），import task_space_smc.* 全部失败。
    # 改成逐级上溯直到走出带 __init__.py 的目录，对任意层深都成立。
    _here = Path(__file__).resolve().parent
    _root = _here
    while (_root / "__init__.py").exists():
        _root = _root.parent
    sys.path = [p for p in sys.path if Path(p or ".").resolve() != _here]
    sys.path.insert(0, str(_root))

from task_space_smc.sim.tune_whole_task_smc import evaluate


TOP_LEVEL_BASE_KEYS = (
    "translation_amplitude_m",
    "translation_frequency_hz",
    "translation_phase_rad",
    "rpy_amplitude_deg",
    "rpy_frequency_hz",
    "rpy_phase_rad",
)


ZERO_SECTIONS = {
    "base_estimator": {
        "delay_sec": 0.0,
        "position_noise_std_m": 0.0,
        "attitude_noise_std_deg": 0.0,
        "linear_velocity_noise_std_m_s": 0.0,
        "angular_velocity_noise_std_rad_s": 0.0,
        "linear_acceleration_noise_std_m_s2": 0.0,
        "angular_acceleration_noise_std_rad_s2": 0.0,
    },
    "payload": {
        "mass_error_kg": 0.0,
        "gravity_w_m_s2": [0.0, 0.0, -9.81],
    },
    "friction": {
        "viscous_nm_per_rad_s": [0.0] * 7,
        "coulomb_nm": [0.0] * 7,
        "stribeck_velocity_rad_s": 0.03,
    },
    "backlash": {
        "torque_deadband_nm": [0.0] * 7,
    },
    "motor": {
        "drive_delay_sec": 0.0,
        "torque_gain": [1.0] * 7,
        "torque_bias_nm": [0.0] * 7,
        "torque_noise_std_nm": [0.0] * 7,
        "torque_ripple_amplitude_nm": [0.0] * 7,
        "torque_ripple_frequency_hz": [0.0] * 7,
        "torque_ripple_phase_rad": [0.0] * 7,
    },
    "gust": {
        "profile": "gust",
        "seed": 17,
        "period_sec": 4.0,
        "width_sec": 0.8,
        "base_amplitude_m": 0.0,
        "joint_torque_amplitude_nm": [0.0] * 7,
    },
}


def _zero_base_motion(injection: dict) -> None:
    injection["translation_amplitude_m"] = [0.0, 0.0, 0.0]
    injection["translation_frequency_hz"] = [0.0, 0.0, 0.0]
    injection["translation_phase_rad"] = [0.0, 0.0, 0.0]
    injection["rpy_amplitude_deg"] = [0.0, 0.0, 0.0]
    injection["rpy_frequency_hz"] = [0.0, 0.0, 0.0]
    injection["rpy_phase_rad"] = [0.0, 0.0, 0.0]
    injection["base_motion"] = {
        "translation_amplitude_m": [0.0, 0.0, 0.0],
        "translation_frequency_hz": [0.0, 0.0, 0.0],
        "translation_phase_rad": [0.0, 0.0, 0.0],
        "rpy_amplitude_deg": [0.0, 0.0, 0.0],
        "rpy_frequency_hz": [0.0, 0.0, 0.0],
        "rpy_phase_rad": [0.0, 0.0, 0.0],
    }


def _restore_base_motion(base_injection: dict, target: dict) -> None:
    for key in TOP_LEVEL_BASE_KEYS:
        if key in base_injection:
            target[key] = deepcopy(base_injection[key])
    target.pop("base_motion", None)
    if "base_motion" in base_injection:
        target["base_motion"] = deepcopy(base_injection["base_motion"])


def zero_injection_config(base_config: dict) -> dict:
    config = deepcopy(base_config)
    injection = config.setdefault("simulation_injection", {})
    seed = injection.get("seed", 20260730)
    injection.clear()
    injection["enabled"] = True
    injection["seed"] = seed
    _zero_base_motion(injection)
    for section, values in ZERO_SECTIONS.items():
        injection[section] = deepcopy(values)
    return config


def enable_term(base_config: dict, term: str) -> dict:
    config = zero_injection_config(base_config)
    base_injection = base_config.get("simulation_injection", {})
    injection = config["simulation_injection"]
    if term == "base_motion":
        _restore_base_motion(base_injection, injection)
    elif term in ZERO_SECTIONS:
        injection[term] = deepcopy(base_injection.get(term, ZERO_SECTIONS[term]))
    else:
        raise ValueError(f"unknown ablation term: {term}")
    return config


def disable_term_from_all(base_config: dict, term: str) -> dict:
    config = deepcopy(base_config)
    injection = config.setdefault("simulation_injection", {})
    injection["enabled"] = True
    if term == "base_motion":
        _zero_base_motion(injection)
    elif term in ZERO_SECTIONS:
        injection[term] = deepcopy(ZERO_SECTIONS[term])
    else:
        raise ValueError(f"unknown ablation term: {term}")
    return config


def injection_disabled_config(base_config: dict) -> dict:
    config = deepcopy(base_config)
    config.setdefault("simulation_injection", {})["enabled"] = False
    return config


def _case_configs(base_config: dict, include_leave_one_out: bool) -> list[tuple[str, str, dict]]:
    terms = ["base_motion", "base_estimator", "payload", "friction", "backlash", "motor", "gust"]
    cases: list[tuple[str, str, dict]] = [
        ("injection_disabled", "enabled_false_full_config", injection_disabled_config(base_config)),
        ("zero_injection", "enabled_true_all_terms_zero", zero_injection_config(base_config)),
    ]
    for term in terms:
        cases.append((f"only_{term}", f"enabled_true_only_{term}", enable_term(base_config, term)))
    all_enabled = deepcopy(base_config)
    all_enabled.setdefault("simulation_injection", {})["enabled"] = True
    cases.append(("all_enabled", "enabled_true_all_terms", all_enabled))
    if include_leave_one_out:
        for term in terms:
            cases.append((f"all_except_{term}", f"enabled_true_without_{term}", disable_term_from_all(base_config, term)))
    return cases


def main() -> int:
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=here / "config" / "smc_params_whole.yaml")
    parser.add_argument("--output-dir", type=Path, default=here / "data" / "ablation")
    parser.add_argument("--duration-sec", type=float, default=3.0)
    parser.add_argument("--dt", type=float, default=0.004)
    parser.add_argument("--include-leave-one-out", action="store_true")
    args = parser.parse_args()

    with args.config.open("r", encoding="utf-8") as stream:
        base_config = yaml.safe_load(stream)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_dir.expanduser().resolve() / stamp
    configs_dir = output_dir / "configs"
    configs_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for case, description, config in _case_configs(base_config, args.include_leave_one_out):
        config_path = configs_dir / f"{case}.yaml"
        with config_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(config, stream, sort_keys=False)
        metrics = evaluate(config, args.duration_sec, args.dt)
        row = {
            "case": case,
            "description": description,
            "config": str(config_path),
            **metrics,
        }
        rows.append(row)
        print(json.dumps(row), flush=True)

    results_path = output_dir / "ablation_results.csv"
    with results_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    summary = {
        "duration_sec": args.duration_sec,
        "dt": args.dt,
        "case_count": len(rows),
        "results_csv": str(results_path),
        "configs_dir": str(configs_dir),
        "best_by_position_rms": min(rows, key=lambda row: row["position_rms_m"]),
        "worst_by_position_rms": max(rows, key=lambda row: row["position_rms_m"]),
        "cases": rows,
    }
    summary_path = output_dir / "ablation_summary.json"
    with summary_path.open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
