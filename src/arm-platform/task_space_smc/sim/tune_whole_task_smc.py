#!/usr/bin/env python3
"""Deterministic gain sweep for the floating-base whole-task SMC simulation."""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import sys
from collections import deque
from copy import deepcopy
from datetime import datetime
from pathlib import Path

import numpy as np

# 关节数：6 关节实物上 joint7 不存在
from task_space_smc.core.model import ARM6_JOINT_NAMES
N_ARM = len(ARM6_JOINT_NAMES)
import pinocchio as pin
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

from task_space_smc.core.model import FloatingBaseArmModel
from task_space_smc.sim.offline_simulation import controller_from_config
from task_space_smc.core.pose_error import PoseState
from task_space_smc.core.quaternion_math import quat_to_rotation, quaternion_log_error, rotation_to_quat
from task_space_smc.core.redundancy_resolution import resolve_acceleration
from task_space_smc.core.trajectory_reference import AnalyticBaseSample, base_sample, reference_sample, traj, whole_period


def _inj(config, *keys, default=None):
    node = config.get("simulation_injection", {})
    for key in keys:
        if not isinstance(node, dict) or key not in node:
            return default
        node = node[key]
    return node


def _inj_enabled(config):
    return bool(_inj(config, "enabled", default=True))


def _inj_vec(config, *keys, default, n=N_ARM):
    return np.asarray(_inj(config, *keys, default=default), dtype=float).reshape(n)


def _small_rotation(rotvec):
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-12:
        return np.eye(3)
    return pin.exp3(np.asarray(rotvec, dtype=float).reshape(3))


def _simulation_base_sample(config, t, shape, derivative_dt, rng=None):
    nominal = base_sample(t, shape, t, derivative_dt)
    if not _inj_enabled(config):
        return nominal, {"gust_active": 0, "gust_envelope": 0.0}

    amp = _inj_vec(
        config,
        "base_motion",
        "translation_amplitude_m",
        default=_inj(config, "translation_amplitude_m", default=[0.0, 0.0, 0.0]),
        n=3,
    )
    freq = _inj_vec(
        config,
        "base_motion",
        "translation_frequency_hz",
        default=_inj(config, "translation_frequency_hz", default=[0.0, 0.0, 0.0]),
        n=3,
    )
    phase = _inj_vec(
        config,
        "base_motion",
        "translation_phase_rad",
        default=_inj(config, "translation_phase_rad", default=[0.0, 0.0, 0.0]),
        n=3,
    )
    wt = 2.0 * math.pi * freq * float(t) + phase
    p_inj = amp * np.sin(wt)
    v_inj = amp * (2.0 * math.pi * freq) * np.cos(wt)
    a_inj = -amp * (2.0 * math.pi * freq) ** 2 * np.sin(wt)

    rpy_amp = np.deg2rad(_inj_vec(
        config,
        "base_motion",
        "rpy_amplitude_deg",
        default=_inj(config, "rpy_amplitude_deg", default=[0.0, 0.0, 0.0]),
        n=3,
    ))
    rpy_freq = _inj_vec(
        config,
        "base_motion",
        "rpy_frequency_hz",
        default=_inj(config, "rpy_frequency_hz", default=[0.0, 0.0, 0.0]),
        n=3,
    )
    rpy_phase = _inj_vec(
        config,
        "base_motion",
        "rpy_phase_rad",
        default=_inj(config, "rpy_phase_rad", default=[0.0, 0.0, 0.0]),
        n=3,
    )
    rwt = 2.0 * math.pi * rpy_freq * float(t) + rpy_phase
    rpy = rpy_amp * np.sin(rwt)
    rpy_d = rpy_amp * (2.0 * math.pi * rpy_freq) * np.cos(rwt)
    rpy_dd = -rpy_amp * (2.0 * math.pi * rpy_freq) ** 2 * np.sin(rwt)

    gust_cfg = _inj(config, "gust", default={}) or {}
    gust_amp = float(gust_cfg.get("base_amplitude_m", 0.0))
    gust_profile = str(gust_cfg.get("profile", "gust"))
    gust_seed = int(gust_cfg.get("seed", _inj(config, "seed", default=20260730)))
    gust_period = float(gust_cfg.get("period_sec", 4.0))
    gust_width = float(gust_cfg.get("width_sec", 0.8))
    g0 = traj.base_disturbance_sample(
        float(t),
        gust_amp,
        seed=gust_seed,
        profile=gust_profile,
        gust_period_sec=gust_period,
        gust_width_sec=gust_width,
    )
    h = max(float(derivative_dt), 1e-4)
    gm = traj.base_disturbance_sample(
        float(t) - h, gust_amp, seed=gust_seed, profile=gust_profile,
        gust_period_sec=gust_period, gust_width_sec=gust_width).total_offset
    gp = traj.base_disturbance_sample(
        float(t) + h, gust_amp, seed=gust_seed, profile=gust_profile,
        gust_period_sec=gust_period, gust_width_sec=gust_width).total_offset
    p_gust = np.asarray(g0.total_offset, dtype=float).reshape(3)
    v_gust = (gp - gm) / (2.0 * h)
    a_gust = (gp - 2.0 * p_gust + gm) / (h * h)

    R_true = nominal.rotation_wb @ _small_rotation(rpy)
    sample = AnalyticBaseSample(
        position_w=nominal.position_w + p_inj + p_gust,
        quaternion_wxyz=rotation_to_quat(R_true),
        rotation_wb=R_true,
        velocity_lwa=nominal.velocity_lwa + np.concatenate([v_inj + v_gust, rpy_d]),
        acceleration_lwa=nominal.acceleration_lwa + np.concatenate([a_inj + a_gust, rpy_dd]),
    )
    return sample, {
        "gust_active": int(bool(g0.event_active)),
        "gust_envelope": float(g0.event_strength),
    }


def _noisy_delayed_base_sample(config, t, true_bs, history, rng):
    if not _inj_enabled(config):
        return true_bs
    history.append((float(t), true_bs))
    delay = max(0.0, float(_inj(config, "base_estimator", "delay_sec", default=0.0)))
    while len(history) > 2 and history[1][0] < float(t) - delay - 1.0:
        history.pop(0)
    target = float(t) - delay
    delayed = history[0][1]
    for stamp, sample in history:
        if stamp <= target:
            delayed = sample
        else:
            break
    pos_std = float(_inj(config, "base_estimator", "position_noise_std_m", default=0.0))
    att_std = math.radians(float(_inj(config, "base_estimator", "attitude_noise_std_deg", default=0.0)))
    vel_std = float(_inj(config, "base_estimator", "linear_velocity_noise_std_m_s", default=0.0))
    gyro_std = float(_inj(config, "base_estimator", "angular_velocity_noise_std_rad_s", default=0.0))
    acc_std = float(_inj(config, "base_estimator", "linear_acceleration_noise_std_m_s2", default=0.0))
    alpha_std = float(_inj(config, "base_estimator", "angular_acceleration_noise_std_rad_s2", default=0.0))
    R = quat_to_rotation(delayed.quaternion_wxyz) @ _small_rotation(rng.normal(0.0, att_std, 3))
    return AnalyticBaseSample(
        position_w=delayed.position_w + rng.normal(0.0, pos_std, 3),
        quaternion_wxyz=rotation_to_quat(R),
        rotation_wb=R,
        velocity_lwa=delayed.velocity_lwa + np.concatenate([
            rng.normal(0.0, vel_std, 3),
            rng.normal(0.0, gyro_std, 3),
        ]),
        acceleration_lwa=delayed.acceleration_lwa + np.concatenate([
            rng.normal(0.0, acc_std, 3),
            rng.normal(0.0, alpha_std, 3),
        ]),
    )


def _payload_disturbance_torque(config, kin):
    mass = float(_inj(config, "payload", "mass_error_kg", default=0.0))
    if abs(mass) < 1e-12:
        return np.zeros(N_ARM)
    gravity = np.asarray(_inj(config, "payload", "gravity_w_m_s2", default=[0.0, 0.0, -9.81]), dtype=float).reshape(3)
    return kin.J_arm_lwa[:3, :].T @ (mass * gravity)


def _gust_joint_torque(config, t):
    amp = _inj_vec(config, "gust", "joint_torque_amplitude_nm", default=[0.0] * N_ARM, n=N_ARM)
    if not _inj_enabled(config) or not np.any(np.abs(amp) > 0.0):
        return np.zeros(N_ARM)
    gust_cfg = _inj(config, "gust", default={}) or {}
    period = float(gust_cfg.get("period_sec", 4.0))
    width = float(gust_cfg.get("width_sec", 0.8))
    if period <= 1e-9 or width <= 1e-9:
        return np.zeros(N_ARM)
    k = math.floor(float(t) / period)
    phase = float(t) - k * period
    if phase < 0.0 or phase >= width:
        return np.zeros(N_ARM)
    envelope = 0.5 * (1.0 - math.cos(2.0 * math.pi * phase / width))
    rng = np.random.default_rng([int(gust_cfg.get("seed", _inj(config, "seed", default=20260730))), int(k), 0x746175])
    direction = rng.uniform(-1.0, 1.0, N_ARM)
    direction /= np.maximum(np.linalg.norm(direction), 1e-12)
    return envelope * amp * direction


def _actual_plant_torque(config, tau_cmd, t, qd, kin, motor_history, rng):
    tau_cmd = np.asarray(tau_cmd, dtype=float).reshape(N_ARM)
    if not _inj_enabled(config):
        return tau_cmd
    delay = max(0.0, float(_inj(config, "motor", "drive_delay_sec", default=0.0)))
    motor_history.append((float(t), tau_cmd.copy()))
    while len(motor_history) > 2 and motor_history[1][0] < float(t) - delay - 1.0:
        motor_history.popleft()
    target = float(t) - delay
    delayed = motor_history[0][1]
    for stamp, cmd in motor_history:
        if stamp <= target:
            delayed = cmd
        else:
            break
    gain = _inj_vec(config, "motor", "torque_gain", default=[1.0] * N_ARM, n=N_ARM)
    bias = _inj_vec(config, "motor", "torque_bias_nm", default=[0.0] * N_ARM, n=N_ARM)
    noise_std = _inj_vec(config, "motor", "torque_noise_std_nm", default=[0.0] * N_ARM, n=N_ARM)
    ripple = _inj_vec(config, "motor", "torque_ripple_amplitude_nm", default=[0.0] * N_ARM, n=N_ARM)
    ripple_freq = _inj_vec(config, "motor", "torque_ripple_frequency_hz", default=[0.0] * N_ARM, n=N_ARM)
    ripple_phase = _inj_vec(config, "motor", "torque_ripple_phase_rad", default=[0.0] * N_ARM, n=N_ARM)
    tau_motor = gain * delayed + bias + ripple * np.sin(2.0 * math.pi * ripple_freq * float(t) + ripple_phase) + rng.normal(0.0, noise_std, 7)
    deadband = _inj_vec(config, "backlash", "torque_deadband_nm", default=[0.0] * N_ARM, n=N_ARM)
    tau_transmitted = np.sign(tau_motor) * np.maximum(np.abs(tau_motor) - deadband, 0.0)
    viscous = _inj_vec(config, "friction", "viscous_nm_per_rad_s", default=[0.0] * N_ARM, n=N_ARM)
    coulomb = _inj_vec(config, "friction", "coulomb_nm", default=[0.0] * N_ARM, n=N_ARM)
    stribeck = max(1e-6, float(_inj(config, "friction", "stribeck_velocity_rad_s", default=0.03)))
    tau_friction = viscous * qd + coulomb * np.tanh(qd / stribeck)
    return tau_transmitted - tau_friction + _payload_disturbance_torque(config, kin) + _gust_joint_torque(config, t)


def _solve_initial_pose(model, config, derivative_dt):
    bs, _ = _simulation_base_sample(config, 0.0, "whole", derivative_dt)
    ref = reference_sample(0.0, "whole", derivative_dt)
    q = np.asarray(config["nullspace"]["q_nominal"], dtype=float).copy()
    scale = np.diag([1.0, 1.0, 1.0, 0.12, 0.12, 0.12])
    for _ in range(250):
        q_full = model.make_q_full(bs.position_w, bs.quaternion_wxyz, q)
        kin = model.compute_kinematics(
            q_full, model.make_v_full(bs.velocity_lwa, np.zeros(N_ARM)))
        error = np.concatenate([
            ref.position_w - kin.ee_position,
            pin.log3(ref.rotation_w @ kin.ee_rotation.T),
        ])
        if np.linalg.norm(error[:3]) < 5e-5 and np.linalg.norm(error[3:]) < 2e-3:
            break
        jacobian = scale @ kin.J_arm_lwa
        delta = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + 1e-4 * np.eye(6), scale @ error)
        q = np.clip(
            q + 0.45 * delta,
            model.arm_lower_position_limit,
            model.arm_upper_position_limit,
        )
    return q


def evaluate(config, duration_sec, dt, derivative_dt=0.005):
    model = FloatingBaseArmModel()
    controller = controller_from_config(config)
    q = _solve_initial_pose(model, config, derivative_dt)
    qd = np.zeros(N_ARM)
    torque_limit = np.asarray(config["limits"]["torque"], dtype=float)
    torque_rate = np.asarray(config["limits"]["torque_rate"], dtype=float)
    last_torque = np.zeros(N_ARM)
    position_errors = []
    orientation_errors = []
    max_torque = np.zeros(N_ARM)
    saturated_steps = 0
    limit_hits = 0
    steps = int(math.ceil(duration_sec / dt))
    seed = int(_inj(config, "seed", default=20260730))
    rng = np.random.default_rng(seed)
    base_history = []
    motor_history = deque()

    for step in range(steps):
        t = step * dt
        bs_true, _ = _simulation_base_sample(config, t, "whole", derivative_dt)
        bs_est = _noisy_delayed_base_sample(config, t, bs_true, base_history, rng)
        q_full_true = model.make_q_full(bs_true.position_w, bs_true.quaternion_wxyz, q)
        v_full_true = model.make_v_full(bs_true.velocity_lwa, qd)
        kin_true = model.compute_kinematics(q_full_true, v_full_true)
        q_full_ctrl = model.make_q_full(bs_est.position_w, bs_est.quaternion_wxyz, q)
        v_full_ctrl = model.make_v_full(bs_est.velocity_lwa, qd)
        kin_ctrl = model.compute_kinematics(q_full_ctrl, v_full_ctrl)
        ref = reference_sample(t, "whole", derivative_dt)
        desired = PoseState(
            position=ref.position_w,
            quaternion_wxyz=ref.quaternion_wxyz,
            linear_velocity=ref.linear_velocity_w,
            angular_velocity=ref.angular_velocity_w,
            linear_acceleration=ref.linear_acceleration_w,
            angular_acceleration=ref.angular_acceleration_w,
        )
        actual = PoseState(
            position=kin_ctrl.ee_position,
            quaternion_wxyz=kin_ctrl.ee_quaternion_wxyz,
            linear_velocity=kin_ctrl.ee_velocity_lwa[:3],
            angular_velocity=kin_ctrl.ee_velocity_lwa[3:],
        )
        command = controller.compute(desired, actual)
        bias = model.compute_task_bias_acceleration(
            q_full_ctrl, v_full_ctrl, bs_est.acceleration_lwa)
        resolved = resolve_acceleration(
            kin_ctrl.J_arm_lwa,
            model.compute_arm_mass_block(q_full_ctrl),
            command.nu - bias,
            q,
            qd,
            config["nullspace"]["q_nominal"],
            config["nullspace"]["kp"],
            config["nullspace"]["kd"],
            float(config["task"]["characteristic_length"]),
            float(config["singularity"]["sigma_warn"]),
            float(config["singularity"]["sigma_critical"]),
            float(config["singularity"]["damping_min"]),
            float(config["singularity"]["damping_max"]),
            config["nullspace"].get("joint_acceleration_weight"),
        )
        raw_torque = model.compute_rnea_arm_torque(
            q_full_ctrl, v_full_ctrl, bs_est.acceleration_lwa, resolved.qdd_cmd)
        torque = np.clip(raw_torque, -torque_limit, torque_limit)
        torque = np.clip(
            torque,
            last_torque - torque_rate * dt,
            last_torque + torque_rate * dt,
        )
        saturated_steps += int(np.any(np.abs(raw_torque - torque) > 1e-9))
        last_torque = torque
        max_torque = np.maximum(max_torque, np.abs(torque))
        plant_torque = _actual_plant_torque(
            config, torque, t, qd, kin_true, motor_history, rng)
        qdd = model.compute_arm_forward_dynamics(
            q_full_true, v_full_true, bs_true.acceleration_lwa, plant_torque)
        qd += qdd * dt
        q_next = q + qd * dt
        hit = ((q_next < model.arm_lower_position_limit)
               | (q_next > model.arm_upper_position_limit))
        limit_hits += int(np.any(hit))
        q = np.clip(
            q_next, model.arm_lower_position_limit, model.arm_upper_position_limit)
        qd[hit] = 0.0
        position_errors.append(float(np.linalg.norm(ref.position_w - kin_true.ee_position)))
        orientation_errors.append(float(np.linalg.norm(
            quaternion_log_error(ref.quaternion_wxyz, kin_true.ee_quaternion_wxyz))))

    errors = np.asarray(position_errors)
    rotations = np.asarray(orientation_errors)
    rms = float(np.sqrt(np.mean(errors * errors)))
    peak = float(np.max(errors))
    saturation_ratio = saturated_steps / max(steps, 1)
    limit_hit_ratio = limit_hits / max(steps, 1)
    score = (
        1000.0 * rms
        + 250.0 * peak
        + 0.03 * float(np.max(max_torque))
        + 50.0 * saturation_ratio
        + 100.0 * limit_hit_ratio
    )
    if rms >= 0.001 or peak >= 0.002:
        score += 1000.0
    return {
        "score": score,
        "position_rms_m": rms,
        "position_peak_m": peak,
        "orientation_rms_rad": float(np.sqrt(np.mean(rotations * rotations))),
        "orientation_peak_rad": float(np.max(rotations)),
        "torque_saturation_ratio": saturation_ratio,
        "joint_limit_hit_ratio": limit_hit_ratio,
        "max_tau_abs_by_joint": max_torque.tolist(),
        "pass_1mm_rms_2mm_peak": rms < 0.001 and peak < 0.002,
    }


def _scaled_config(base, lambda_scale, switch_scale, orientation_scale):
    candidate = deepcopy(base)
    for key in ("lambda", "k"):
        candidate["position"][key] = [
            float(value) * lambda_scale for value in base["position"][key]]
    candidate["position"]["eta"] = [
        float(value) * switch_scale for value in base["position"]["eta"]]
    for key in ("lambda", "k", "eta"):
        candidate["orientation"][key] = [
            float(value) * orientation_scale for value in base["orientation"][key]]
    return candidate


def _gain_scaled_config(base, position_scale, switch_scale, orientation_scale, nullspace_scale):
    candidate = _scaled_config(base, position_scale, switch_scale, orientation_scale)
    for key in ("kp", "kd"):
        candidate["nullspace"][key] = [
            float(value) * nullspace_scale for value in base["nullspace"][key]]
    return candidate


def main() -> int:
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", type=Path, default=here / "config" / "smc_params_whole.yaml")
    parser.add_argument("--output-dir", type=Path, default=here / "data" / "autotune")
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--max-candidates", type=int, default=12)
    parser.add_argument(
        "--full-period", action="store_true",
        help="Evaluate every candidate over the complete whole-task period.")
    args = parser.parse_args()
    with args.config.open("r", encoding="utf-8") as stream:
        base = yaml.safe_load(stream)
    duration = whole_period() if args.full_period else args.duration_sec
    scales = list(itertools.product(
        (0.75, 1.0, 1.4, 1.8, 2.4),
        (0.75, 1.0, 1.4, 1.8),
        (0.75, 1.0, 1.3, 1.7),
        (0.6, 1.0, 1.6, 2.2),
    ))
    scales.sort(key=lambda item: sum(abs(value - 1.0) for value in item))
    scales = scales[:max(1, args.max_candidates)]
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_dir.expanduser().resolve() / stamp
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = []

    for index, (position_scale, switch_scale, orientation_scale, nullspace_scale) in enumerate(scales, 1):
        candidate = _gain_scaled_config(
            base, position_scale, switch_scale, orientation_scale, nullspace_scale)
        metrics = evaluate(candidate, duration, args.dt)
        row = {
            "candidate": index,
            "position_scale": position_scale,
            "switch_scale": switch_scale,
            "orientation_scale": orientation_scale,
            "nullspace_scale": nullspace_scale,
            **metrics,
        }
        rows.append(row)
        print(json.dumps(row), flush=True)

    rows.sort(key=lambda row: row["score"])
    best = rows[0]
    best_config = _scaled_config(
        base,
        best["position_scale"],
        best["switch_scale"],
        best["orientation_scale"],
    )
    best_config = _gain_scaled_config(
        base,
        best["position_scale"],
        best["switch_scale"],
        best["orientation_scale"],
        best["nullspace_scale"],
    )
    best_config_path = output_dir / "smc_params_whole_best.yaml"
    with best_config_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(best_config, stream, sort_keys=False)
    results_path = output_dir / "tuning_results.csv"
    with results_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        "duration_sec": duration,
        "dt": args.dt,
        "candidate_count": len(rows),
        "best": best,
        "best_config": str(best_config_path),
        "results_csv": str(results_path),
    }
    summary_path = output_dir / "tuning_summary.json"
    with summary_path.open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
