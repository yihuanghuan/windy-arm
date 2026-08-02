#!/usr/bin/env python3
"""Offline floating-base task-space SMC validation harness."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import yaml

if __package__ in (None, ""):
    package_dir = Path(__file__).resolve().parent
    if (package_dir / "__init__.py").exists():
        sys.path = [p for p in sys.path if Path(p or ".").resolve() != package_dir]
        sys.path.insert(0, str(package_dir.parent))
    else:
        sys.path.insert(0, str(package_dir))

from task_space_smc.base_state import SimulationInjectedBase, SinusoidSpec
from task_space_smc.model import DEFAULT_ARM_JOINT_NAMES, FloatingBaseArmModel
from task_space_smc.pose_error import PoseState
from task_space_smc.quaternion_math import quat_normalize, quaternion_log_error
from task_space_smc.redundancy_resolution import resolve_acceleration
from task_space_smc.task_space_smc import SMCGains, TaskSpaceSMC


def _arr(config: dict, section: str, key: str, n: int) -> np.ndarray:
    return np.asarray(config[section][key], dtype=float).reshape(n)


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


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


def make_base(profile: str) -> SimulationInjectedBase:
    if profile == "none":
        trans_amp = [0.0, 0.0, 0.0]
        rpy_amp = [0.0, 0.0, 0.0]
    elif profile == "translation":
        trans_amp = [0.015, 0.01, 0.008]
        rpy_amp = [0.0, 0.0, 0.0]
    elif profile == "rotation":
        trans_amp = [0.0, 0.0, 0.0]
        rpy_amp = np.deg2rad([4.0, 3.0, 2.0])
    else:
        trans_amp = [0.012, 0.009, 0.006]
        rpy_amp = np.deg2rad([4.0, 3.0, 2.0])
    translation = SinusoidSpec.from_lists(trans_amp, [0.55, 0.75, 0.95], [0.0, 0.7, 1.1])
    rpy = SinusoidSpec.from_lists(rpy_amp, [0.45, 0.65, 0.85], [0.2, 1.0, 1.7])
    return SimulationInjectedBase(translation, rpy)


def desired_pose(t: float, initial: PoseState, mode: str) -> PoseState:
    if mode == "circle":
        radius = 0.015
        w = 2.0 * math.pi * 0.2
        offset = np.array([radius * math.sin(w * t), radius * (1.0 - math.cos(w * t)), 0.0])
        vel = np.array([radius * w * math.cos(w * t), radius * w * math.sin(w * t), 0.0])
        acc = np.array([-radius * w * w * math.sin(w * t), radius * w * w * math.cos(w * t), 0.0])
    else:
        offset = np.zeros(3)
        vel = np.zeros(3)
        acc = np.zeros(3)
    return PoseState(
        position=initial.position + offset,
        quaternion_wxyz=initial.quaternion_wxyz,
        linear_velocity=vel,
        angular_velocity=np.zeros(3),
        linear_acceleration=acc,
        angular_acceleration=np.zeros(3),
    )


def run_offline(
    config_path: str,
    output_dir: str,
    profile: str = "sixdof",
    reference_mode: str = "hold",
    duration: float = 1.0,
    dt: float | None = None,
    seed: int = 1,
) -> dict:
    config = load_config(config_path)
    if dt is None:
        dt = 1.0 / float(config["control_rate_hz"])
    model = FloatingBaseArmModel()
    controller = controller_from_config(config)
    base = make_base(profile)
    rng = np.random.default_rng(seed)

    q_arm = np.asarray(config["nullspace"]["q_nominal"], dtype=float).reshape(7) + rng.normal(0.0, 0.015, 7)
    qd_arm = np.zeros(7)
    base0 = base.sample(0.0)
    q_full = model.make_q_full(base0.position_w, base0.quaternion_wxyz, q_arm)
    v_full = model.make_v_full(base0.velocity_lwa, qd_arm)
    kin0 = model.compute_kinematics(q_full, v_full)
    initial_desired = PoseState(
        position=kin0.ee_position.copy(),
        quaternion_wxyz=kin0.ee_quaternion_wxyz.copy(),
        linear_velocity=np.zeros(3),
        angular_velocity=np.zeros(3),
        linear_acceleration=np.zeros(3),
        angular_acceleration=np.zeros(3),
    )

    rows: list[dict] = []
    max_tau = np.zeros(7)
    saturation_count = 0
    steps = max(1, int(round(duration / dt)))
    torque_limit = np.asarray(config["limits"]["torque"], dtype=float).reshape(7)

    for k in range(steps):
        t = k * dt
        bs = base.sample(t)
        q_full = model.make_q_full(bs.position_w, bs.quaternion_wxyz, q_arm)
        v_full = model.make_v_full(bs.velocity_lwa, qd_arm)
        kin = model.compute_kinematics(q_full, v_full)
        actual = PoseState(
            position=kin.ee_position,
            quaternion_wxyz=kin.ee_quaternion_wxyz,
            linear_velocity=kin.ee_velocity_lwa[:3],
            angular_velocity=kin.ee_velocity_lwa[3:],
        )
        desired = desired_pose(t, initial_desired, reference_mode)
        smc = controller.compute(desired, actual)
        b6 = model.compute_task_bias_acceleration(q_full, v_full, bs.acceleration_lwa)
        M_aa = model.compute_arm_mass_block(q_full)
        rr = resolve_acceleration(
            kin.J_arm_lwa,
            M_aa,
            smc.nu - b6,
            q_arm,
            qd_arm,
            config["nullspace"]["q_nominal"],
            config["nullspace"]["kp"],
            config["nullspace"]["kd"],
            float(config["task"]["characteristic_length"]),
            float(config["singularity"]["sigma_warn"]),
            float(config["singularity"]["sigma_critical"]),
            float(config["singularity"]["damping_min"]),
            float(config["singularity"]["damping_max"]),
        )
        tau = model.compute_rnea_arm_torque(q_full, v_full, bs.acceleration_lwa, rr.qdd_cmd)
        tau_limited = np.clip(tau, -torque_limit, torque_limit)
        saturation_count += int(np.any(np.abs(tau) > torque_limit))
        max_tau = np.maximum(max_tau, np.abs(tau_limited))
        qdd = model.compute_arm_forward_dynamics(q_full, v_full, bs.acceleration_lwa, tau_limited)
        qd_arm = qd_arm + qdd * dt
        q_arm = q_arm + qd_arm * dt
        e_R = quaternion_log_error(desired.quaternion_wxyz, actual.quaternion_wxyz)
        rows.append(
            {
                "t": t,
                "pos_error_norm": float(np.linalg.norm(smc.position_error)),
                "rot_error_rad": float(np.linalg.norm(e_R)),
                "s_pos_norm": float(np.linalg.norm(smc.position_sliding)),
                "s_rot_norm": float(np.linalg.norm(smc.orientation_sliding)),
                "sigma_min": rr.sigma_min,
                "damping": rr.damping,
                "nullspace_leakage": rr.nullspace_leakage_norm,
                "tau_max_abs": float(np.max(np.abs(tau_limited))),
            }
        )

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    csv_path = out / f"offline_{profile}_{reference_mode}.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    metrics = {
        "profile": profile,
        "reference_mode": reference_mode,
        "steps": steps,
        "dt": dt,
        "final_position_error_m": rows[-1]["pos_error_norm"],
        "max_position_error_m": max(r["pos_error_norm"] for r in rows),
        "final_orientation_error_rad": rows[-1]["rot_error_rad"],
        "max_orientation_error_rad": max(r["rot_error_rad"] for r in rows),
        "min_sigma": min(r["sigma_min"] for r in rows),
        "max_damping": max(r["damping"] for r in rows),
        "max_nullspace_leakage": max(r["nullspace_leakage"] for r in rows),
        "max_tau_abs_by_joint": max_tau.tolist(),
        "torque_saturation_ratio": saturation_count / steps,
        "csv": str(csv_path),
    }
    metrics_path = out / f"metrics_{profile}_{reference_mode}.json"
    with open(metrics_path, "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2)
    metrics["metrics_path"] = str(metrics_path)
    return metrics


def default_config_path() -> str:
    return str(Path(__file__).resolve().parent / "config" / "smc_params.yaml")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=default_config_path())
    parser.add_argument("--output-dir", default="/tmp/task_space_smc_offline")
    parser.add_argument("--profile", default="sixdof", choices=["none", "translation", "rotation", "sixdof"])
    parser.add_argument("--reference-mode", default="hold", choices=["hold", "circle"])
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--dt", type=float, default=None)
    args = parser.parse_args()
    metrics = run_offline(args.config, args.output_dir, args.profile, args.reference_mode, args.duration, args.dt)
    print(json.dumps(metrics, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
