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

from task_space_smc.core.base_state import SimulationInjectedBase, SinusoidSpec
from task_space_smc.core.model import (
    DEFAULT_ARM_JOINT_NAMES, FloatingBaseArmModel, arm6_urdf_path,
    ARM6_JOINT_NAMES, ARM6_EE_FRAME, ARM6_URDF_NAME,
    ARM6_TOOL_EE_FRAME, ARM6_TOOL_URDF_NAME, arm6_tool_urdf_path,
)
from task_space_smc.core.pose_error import PoseState
from task_space_smc.core.quaternion_math import quat_normalize, quaternion_log_error
from task_space_smc.core.redundancy_resolution import (
    resolve_acceleration, pen_axis_task, pen_axis_error, LimitAvoidance)
from task_space_smc.core.task_space_smc import SMCGains, TaskSpaceSMC


# 配置加载/控制器装配已搬到 core/config_loader.py —— 实物运行器也要用它们，
# 留在 sim 里会形成"实物层 import 仿真层"的倒挂。这里再导出，既有调用方
# （task_space_smc.sim.offline_simulation.controller_from_config 等）不受影响。
from task_space_smc.core.config_loader import (  # noqa: E402,F401
    _arr, controller_from_config, default_config_path, load_config)


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
    # URDF 由配置指定 —— 否则 6 关节配置会配上 7 关节模型，
    # 直到 reshape 才报错（实测：cannot reshape array of size 6 into shape (7,)）。
    urdf = config.get("urdf")
    if urdf == "arm6_tool" or urdf == ARM6_TOOL_URDF_NAME:
        model = FloatingBaseArmModel(urdf_path=arm6_tool_urdf_path(),
                                     ee_frame_name=ARM6_TOOL_EE_FRAME,
                                     arm_joint_names=ARM6_JOINT_NAMES)
    elif urdf == "arm6" or urdf == ARM6_URDF_NAME:
        model = FloatingBaseArmModel(urdf_path=arm6_urdf_path(),
                                     ee_frame_name=ARM6_EE_FRAME,
                                     arm_joint_names=tuple(ARM6_JOINT_NAMES))
    elif urdf:
        model = FloatingBaseArmModel(urdf_path=urdf)
    else:
        model = FloatingBaseArmModel()
    controller = controller_from_config(config)
    base = make_base(profile)
    rng = np.random.default_rng(seed)

    # 关节数由模型推导。零空间在 6 关节 6D 任务下秩为 0、已停用，
    # 所以起始位姿不再从 nullspace.q_nominal 取。
    n_arm = model.n_arm
    # 起始位形优先取 initial_pose_rad。退化为零位是危险的 —— 6 关节零位的
    # 雅可比 sigma_min = 0.00001（cond 326263），起步即奇异。
    init = config.get("initial_pose_rad")
    q_nominal = config.get("nullspace", {}).get("q_nominal")
    if init is not None and len(init) == n_arm:
        q_arm = np.asarray(init, dtype=float).reshape(n_arm)
    elif q_nominal is not None and len(q_nominal) == n_arm:
        q_arm = np.asarray(q_nominal, dtype=float).reshape(n_arm)
    else:
        q_arm = np.zeros(n_arm)
    q_arm = q_arm + rng.normal(0.0, 0.015, n_arm)
    qd_arm = np.zeros(n_arm)
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
    max_tau = np.zeros(n_arm)
    _task = config.get("task", {})
    orientation_mode = str(_task.get("orientation_mode", "full"))
    pen_axis_local = np.asarray(_task.get("pen_axis_local", [1.0, 0.0, 0.0]), dtype=float)
    _la = _task.get("limit_avoidance") or {}
    limit_avoidance = None
    if float(_la.get("gain", 0.0)) > 0.0:
        limit_avoidance = LimitAvoidance(
            lower=model.model.lowerPositionLimit[7:].copy(),
            upper=model.model.upperPositionLimit[7:].copy(),
            gain=float(_la["gain"]),
            activate_rad=float(_la.get("activate_rad", 0.5)))
    ns = config.get("nullspace") or {}
    ns_enabled = bool(ns.get("enabled", True)) and "q_nominal" in ns
    ns_q_nominal = (np.asarray(ns["q_nominal"], dtype=float).reshape(n_arm)
                    if ns_enabled else np.zeros(n_arm))
    ns_kp = (np.asarray(ns["kp"], dtype=float).reshape(n_arm)
             if ns_enabled else np.zeros(n_arm))
    ns_kd = (np.asarray(ns["kd"], dtype=float).reshape(n_arm)
             if ns_enabled else np.zeros(n_arm))
    saturation_count = 0
    steps = max(1, int(round(duration / dt)))
    torque_limit = np.asarray(config["limits"]["torque"], dtype=float).reshape(n_arm)

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
        # 5D 任务（位置 + 笔轴方向），零空间秩 0 -> 1
        J_task, rhs_task = kin.J_arm_lwa, smc.nu - b6
        if orientation_mode == "pen_axis":
            J_task, rhs_task, _ = pen_axis_task(
                J_task, rhs_task, kin.ee_rotation, pen_axis_local)
        rr = resolve_acceleration(
            J_task,
            M_aa,
            rhs_task,
            q_arm,
            qd_arm,
            # 零空间在 6 关节做 6D 任务时秩为 0，投影恒为零。配置里
            # nullspace.enabled=false 时这三项不存在，传中性值即可 ——
            # resolve_acceleration 仍需形参，但其贡献必然是 0。
            ns_q_nominal,
            ns_kp,
            ns_kd,
            float(config["task"]["characteristic_length"]),
            float(config["singularity"]["sigma_warn"]),
            float(config["singularity"]["sigma_critical"]),
            float(config["singularity"]["damping_min"]),
            float(config["singularity"]["damping_max"]),
            limit_avoidance=limit_avoidance,
        )
        tau = model.compute_rnea_arm_torque(q_full, v_full, bs.acceleration_lwa, rr.qdd_cmd)
        tau_limited = np.clip(tau, -torque_limit, torque_limit)
        saturation_count += int(np.any(np.abs(tau) > torque_limit))
        max_tau = np.maximum(max_tau, np.abs(tau_limited))
        qdd = model.compute_arm_forward_dynamics(q_full, v_full, bs.acceleration_lwa, tau_limited)
        qd_arm = qd_arm + qdd * dt
        q_arm = q_arm + qd_arm * dt
        e_R = quaternion_log_error(desired.quaternion_wxyz, actual.quaternion_wxyz)
        import pinocchio as _pin
        _qd = np.asarray(desired.quaternion_wxyz, dtype=float)
        _R_des = _pin.Quaternion(_qd[0], _qd[1], _qd[2], _qd[3]).toRotationMatrix()
        e_axis = pen_axis_error(kin.ee_rotation, _R_des, pen_axis_local)
        rows.append(
            {
                "t": t,
                "pos_error_norm": float(np.linalg.norm(smc.position_error)),
                "rot_error_rad": float(np.linalg.norm(e_R)),
                "pen_axis_error": float(e_axis),
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
        "max_pen_axis_error_rad": max(r["pen_axis_error"] for r in rows),
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
