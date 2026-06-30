#!/usr/bin/env python3
"""LL100-LH420 避障 NMPC 的离线闭环调参器(独立于独立臂的 tune_mpc_params.py)。

不启动 ROS、不画图。复用 move_ll100_lh420_nmpc_demo 的同一套 MpcIkConfig + ll100_traj_pub
的路径模式,离线 roll 出**闭环**轨迹:被控对象复刻 LL100 底层 student_arm_node 的
SmoothPositionController(固定开启速度前馈),MPC 每拍从被控对象真实状态反馈重解。
避障 violation/margin 在被控对象实际经过的状态上量,用作"安全硬门",再按跟踪误差排序。

用法:
    python3 tune_ll100_nmpc.py --path-mode avoidance_sweep
    python3 tune_ll100_nmpc.py --path-mode avoidance_sweep --max-combinations 2 --periods 0.5
需先 source ROS + 工作空间 + ACADOS(导入 demo 模块和求解器)。
"""

import argparse
import contextlib
import csv
import itertools
import os
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Tuple

import numpy as np

from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK


PUBLISH_RATE_HZ = 50.0
CONTROL_DT = 1.0 / PUBLISH_RATE_HZ

EE_FRAME = "F_Link"
LINK_FRAMES = ("A_Link", "B_Link", "C_Link", "D_Link", "E_Link", "F_Link")
EVAL_PERIOD_SEC = 30.0
TERMINAL_TOLERANCE = 0.025
VIOLATION_TOL = 1.0e-4  # 缓冲侵入 <= 0.1mm 视为安全(数值/离散噪声)
DEFAULT_MAX_VELOCITY = 0.5  # 底层关节限速 rad/s(= ll100_lh420_control.launch.py 默认)

# SmoothPositionController velocity-feedforward 配置默认值(= VelocityFeedforwardConfig 默认)。
# LL100 底层固定开启速度前馈,无开关。
VFF_GAIN = 0.7        # velocity_feedforward_gain
VFF_TAU = 0.15        # position_feedback_tau
VFF_FB_VLIM = 0.4     # feedback_velocity_limit
VFF_MAX_ACCEL = 5.0   # max_acceleration

COARSE_CAPSULE_SAMPLES = [1, 2, 3]
# 含 > max_velocity*dt(=0.01)的步长,闭环下会暴露底层跟不上的滞后
COARSE_MAX_JOINT_STEP = [0.008, 0.01, 0.015, 0.02]
COARSE_PREVIEW_STRIDE = [4, 6]
COARSE_BODY_MARGIN = [0.015, 0.02]


def rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(values * values)))


def maybe_limit(candidates: List["ParamSet"], max_combinations) -> List["ParamSet"]:
    if max_combinations is None or max_combinations <= 0:
        return candidates
    return candidates[:max_combinations]


def vertical_down_rotation() -> np.ndarray:
    # 末端 x 轴对齐 base_link -Z,y 轴对齐 +Y,z 轴对齐 +X(= demo 的 vertical_down)。
    return np.array(
        [
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
        ],
        dtype=float,
    )


@contextlib.contextmanager
def suppress_native_output(enabled: bool):
    if not enabled:
        yield
        return
    stdout_fd = os.dup(1)
    stderr_fd = os.dup(2)
    try:
        with open(os.devnull, "w") as devnull:
            os.dup2(devnull.fileno(), 1)
            os.dup2(devnull.fileno(), 2)
            yield
    finally:
        os.dup2(stdout_fd, 1)
        os.dup2(stderr_fd, 2)
        os.close(stdout_fd)
        os.close(stderr_fd)


@dataclass(frozen=True, order=True)
class ParamSet:
    capsule_samples_per_segment: int
    max_joint_step: float
    path_preview_stride: int
    body_obstacle_margin: float


@dataclass
class TrialResult:
    params: ParamSet
    path_mode: str
    safe: bool
    score: float
    violation_max: float
    min_margin: float
    fallback_count: int
    solver_failure_count: int
    rms_position_error: float
    max_position_error: float
    mean_solve_time_ms: float
    samples: int
    recommended_command: str


def coarse_candidates() -> List[ParamSet]:
    return [
        ParamSet(cs, s, ps, bm)
        for cs, s, ps, bm in itertools.product(
            COARSE_CAPSULE_SAMPLES,
            COARSE_MAX_JOINT_STEP,
            COARSE_PREVIEW_STRIDE,
            COARSE_BODY_MARGIN,
        )
    ]


def build_config(p: ParamSet, path_mode: str, backend: str) -> Tuple[MpcIkConfig, str]:
    from mpc_pinocchio_ik import DEFAULT_LL100_URDF
    from move_ll100_lh420_nmpc_demo import DEFAULT_LL100_BODY_OBSTACLE_BOXES, LL100_NMPC_URDF

    build_dir = (
        f"/tmp/windylab_acados_tune_ll100_{path_mode}"
        f"_cs{p.capsule_samples_per_segment}_s{int(round(p.max_joint_step * 1000))}"
        f"_ps{p.path_preview_stride}_bm{int(round(p.body_obstacle_margin * 1000))}"
    )
    cfg = MpcIkConfig(
        horizon_steps=8,
        control_dt=CONTROL_DT,
        max_joint_step=p.max_joint_step,
        success_tolerance=TERMINAL_TOLERANCE,
        solver_backend=backend,
        acados_build_dir=build_dir,
        position_weight=120.0,
        terminal_position_weight=1200.0,
        orientation_weight=0.3,
        terminal_orientation_weight=3.0,
        obstacle_avoidance_enabled=True,
        obstacle_hard_constraint=True,
        obstacle_safety_margin=0.04,
        obstacle_disk_half_thickness=0.02,
        obstacle_ll100_urdf_path=DEFAULT_LL100_URDF,
        obstacle_coordinate_mode="ll100_base",
        obstacle_keypoint_frames=LINK_FRAMES,
        body_obstacle_enabled=True,
        body_obstacle_hard_constraint=True,
        body_obstacle_margin=p.body_obstacle_margin,
        body_obstacle_boxes=DEFAULT_LL100_BODY_OBSTACLE_BOXES,
        capsule_radius=0.018,
        capsule_samples_per_segment=p.capsule_samples_per_segment,
        capsule_frame_sequence=LINK_FRAMES,
        current_posture_weight=0.005,
        center_posture_weight=0.01,
    )
    return cfg, LL100_NMPC_URDF


def score_of(viol_max: float, fallback: int, solver_fail: int,
             rms_pos: float, max_pos: float, mean_ms: float, safe: bool) -> float:
    # 安全硬门:不安全的候选直接加一大常数,排在所有安全候选之后;
    # 安全候选之间再按 violation + 跟踪误差排序。
    return (
        (0.0 if safe else 1.0e5)
        + 1.0e6 * viol_max
        + 500.0 * fallback
        + 500.0 * solver_fail
        + 1000.0 * rms_pos
        + 1000.0 * max_pos
        + 0.01 * mean_ms
    )


def recommended_command(p: ParamSet, path_mode: str) -> str:
    return (
        "python3 move_ll100_lh420_nmpc_demo.py --ros-args "
        f"-p path_mode:={path_mode} "
        f"-p capsule_samples_per_segment:={p.capsule_samples_per_segment} "
        f"-p max_joint_step:={p.max_joint_step:.4f} "
        f"-p path_preview_stride:={p.path_preview_stride} "
        f"-p body_obstacle_margin:={p.body_obstacle_margin:.4f}"
    )


def evaluate(p: ParamSet, path_mode: str, backend: str,
             periods: float, quiet_build: bool, max_velocity: float) -> TrialResult:
    import pinocchio as pin
    from ll100_traj_pub import _build_path as build_ll100_path

    cfg, urdf = build_config(p, path_mode, backend)
    # 只在构造求解器时静音(ACADOS codegen/编译);step() 之后是安静的。
    with suppress_native_output(quiet_build):
        mpc = MpcPinocchioIK(urdf_path=urdf, ee_frame=EE_FRAME, config=cfg)

    count = max(20, int(round(EVAL_PERIOD_SEC / CONTROL_DT)))
    path = build_ll100_path(path_mode, count)
    locked_rotation = vertical_down_rotation()

    # 闭环:q_plant 是被控对象(student_arm_node 的 SmoothPositionController)的真实状态,
    # MPC 每拍从 q_plant 反馈重解。LL100 底层固定开启速度前馈(velocity feedforward),
    # 复刻该控制器:速度前馈 v_ff + 位置反馈 v_fb(τ=0.15、限速 0.4)+ 加速度限(5.0),再积分。
    q_plant = np.clip(pin.neutral(mpc.model), mpc.q_lower, mpc.q_upper)
    q_cmd = q_plant.copy()                 # 已发布的位置命令(命令不可用时保持上一条)
    last_v = np.zeros_like(q_plant)        # 上一拍速度命令(加速度限用)

    actual_pos, _ = mpc.forward(q_plant)
    nearest = int(np.argmin(np.linalg.norm(path - actual_pos.reshape(1, 3), axis=1)))

    horizon = cfg.horizon_steps
    offsets = p.path_preview_stride * np.arange(horizon)
    warm_start = None
    steps = max(1, int(round(periods * EVAL_PERIOD_SEC / CONTROL_DT)))
    warmup = min(50, steps // 4)

    fallback = 0
    solver_fail = 0
    pos_errs: List[float] = []
    solve_ms: List[float] = []
    realized: List[np.ndarray] = []
    for step in range(steps):
        base = (nearest + step) % len(path)
        targets = path[(base + offsets) % len(path)]
        start = time.perf_counter()
        q_next, ok, info = mpc.step(
            targets, q_plant, warm_start=warm_start, target_rotation=locked_rotation
        )
        solve_ms.append(1000.0 * float(info.get("solve_time", time.perf_counter() - start)))
        warm_start = info["trajectory"]
        if int(info.get("status", 0)) != 0 or not bool(info.get("success", True)):
            solver_fail += 1
        if bool(info.get("command_usable", ok)):
            q_cmd = q_next                 # 发布位置命令
            qdot_ref = (q_next - q_plant) / CONTROL_DT   # demo 发布的前馈速度 dq_cmd
        else:
            fallback += 1                  # 命令不可用:保持上一条位置命令,前馈速度置 0
            qdot_ref = np.zeros_like(q_plant)
        # 被控对象推进一步(复刻 SmoothPositionController 速度前馈),再作为下一拍反馈
        v_ff = np.clip(qdot_ref, -max_velocity, max_velocity)
        v_fb = np.clip((q_cmd - q_plant) / VFF_TAU, -VFF_FB_VLIM, VFF_FB_VLIM)
        v_raw = np.clip(VFF_GAIN * v_ff + v_fb, -max_velocity, max_velocity)
        accel = VFF_MAX_ACCEL * CONTROL_DT
        v_cmd = np.clip(v_raw, last_v - accel, last_v + accel)
        q_new = np.clip(q_plant + v_cmd * CONTROL_DT, mpc.q_lower, mpc.q_upper)
        last_v = (q_new - q_plant) / CONTROL_DT          # 夹关节限位后回算实际速度
        q_plant = q_new
        if step >= warmup:
            realized.append(q_plant.copy())
            actual_pos, _ = mpc.forward(q_plant)
            pos_errs.append(float(np.linalg.norm(actual_pos - path[base])))

    # 闭环真实安全:在被控对象实际经过的状态序列上量 violation/margin(不是 MPC 预测 plan)
    if realized:
        diag = mpc._obstacle_diagnostics(np.asarray(realized, dtype=float))
        viol_max = float(diag.get("obstacle_violation_max") or 0.0)
        mm = diag.get("min_obstacle_margin")
        min_margin = float(mm) if mm is not None else 0.0
    else:
        viol_max = 0.0
        min_margin = 0.0

    pos = np.asarray(pos_errs, dtype=float) if pos_errs else np.zeros(1)
    rms_pos = rms(pos)
    max_pos = float(np.max(pos))
    mean_ms = float(np.mean(solve_ms)) if solve_ms else 0.0
    safe = (viol_max <= VIOLATION_TOL) and (fallback == 0)
    score = score_of(viol_max, fallback, solver_fail, rms_pos, max_pos, mean_ms, safe)
    return TrialResult(
        params=p, path_mode=path_mode, safe=safe, score=score,
        violation_max=viol_max,
        min_margin=(0.0 if min_margin == float("inf") else min_margin),
        fallback_count=fallback, solver_failure_count=solver_fail,
        rms_position_error=rms_pos, max_position_error=max_pos,
        mean_solve_time_ms=mean_ms, samples=len(pos_errs),
        recommended_command=recommended_command(p, path_mode),
    )


def run(args: argparse.Namespace) -> int:
    candidates = maybe_limit(coarse_candidates(), args.max_combinations)
    print(f"[ll100 避障调参] path_mode={args.path_mode} 候选 {len(candidates)} 组", flush=True)
    results: List[TrialResult] = []
    for i, p in enumerate(candidates):
        print(f"  [{i + 1}/{len(candidates)}] {p}", flush=True)
        try:
            r = evaluate(p, args.path_mode, args.backend, args.periods, args.quiet_build,
                         args.max_velocity)
        except Exception as exc:  # noqa: BLE001 - 单组失败不应中断整轮搜索
            print(f"    跳过(异常): {exc}", flush=True)
            continue
        results.append(r)
        print(
            f"    safe={r.safe} score={r.score:.2f} viol_max={r.violation_max:.6f} "
            f"min_margin={r.min_margin:.4f} fallback={r.fallback_count} "
            f"rms_pos={r.rms_position_error:.4f} mean_ms={r.mean_solve_time_ms:.2f}",
            flush=True,
        )
    if not results:
        print("没有可用结果(检查 path_mode 是否为 ll100_traj_pub 支持的模式)")
        return 1

    results.sort(key=lambda r: r.score)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = args.output_dir / f"tune_ll100_{args.path_mode}_{stamp}.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "rank", "safe", "score", "path_mode", "capsule_samples_per_segment",
            "max_joint_step", "path_preview_stride", "body_obstacle_margin",
            "violation_max", "min_margin", "fallback_count", "solver_failure_count",
            "rms_position_error", "max_position_error", "mean_solve_time_ms", "samples",
            "recommended_command",
        ])
        for rank, r in enumerate(results, 1):
            writer.writerow([
                rank, r.safe, f"{r.score:.4f}", r.path_mode,
                r.params.capsule_samples_per_segment, f"{r.params.max_joint_step:.4f}",
                r.params.path_preview_stride, f"{r.params.body_obstacle_margin:.4f}",
                f"{r.violation_max:.8f}", f"{r.min_margin:.6f}", r.fallback_count,
                r.solver_failure_count, f"{r.rms_position_error:.6f}",
                f"{r.max_position_error:.6f}", f"{r.mean_solve_time_ms:.3f}", r.samples,
                r.recommended_command,
            ])
    print(f"\nCSV: {csv_path}", flush=True)
    print(f"\nTop {min(args.top_k, len(results))}(先看 safe=True,再看 rms_pos):")
    for rank, r in enumerate(results[: args.top_k], 1):
        print(
            f"  #{rank} safe={r.safe} viol_max={r.violation_max:.6f} "
            f"rms_pos={r.rms_position_error:.4f} fallback={r.fallback_count} | "
            f"cs={r.params.capsule_samples_per_segment} step={r.params.max_joint_step} "
            f"preview={r.params.path_preview_stride} body_margin={r.params.body_obstacle_margin}"
        )
    best = results[0]
    print(f"\n推荐(score 最低):\n{best.recommended_command}")
    if not best.safe:
        print("⚠️ 最优候选未通过安全门(violation 或 fallback);建议改路径或扩大候选范围。")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="LL100 避障 NMPC 离线闭环调参")
    parser.add_argument(
        "--path-mode",
        default="avoidance_sweep",
        help="路径模式(ll100_traj_pub 支持的模式,如 avoidance_sweep / lower_obstacle_circle / circle 等)",
    )
    parser.add_argument(
        "--max-velocity",
        type=float,
        default=DEFAULT_MAX_VELOCITY,
        help="闭环 plant 的底层关节限速 rad/s(应与 student_arm max_velocity 一致)",
    )
    parser.add_argument("--backend", choices=("acados", "casadi", "scipy"), default="acados")
    parser.add_argument("--periods", type=float, default=1.0, help="离线仿真的路径周期数")
    parser.add_argument("--top-k", type=int, default=10, help="终端打印前 K 名")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/home/windylab/code/windylab_ws/src/arm-platform/demo/data"),
    )
    parser.add_argument(
        "--quiet-build",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="是否静音 ACADOS 代码生成/编译输出,默认静音;排查构建问题用 --no-quiet-build",
    )
    parser.add_argument(
        "--max-combinations",
        type=int,
        default=None,
        help="限制最多评估多少组参数;适合快速 smoke test",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.periods <= 0.0:
        raise ValueError("--periods 必须大于 0")
    if args.top_k < 1:
        raise ValueError("--top-k 必须大于等于 1")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
