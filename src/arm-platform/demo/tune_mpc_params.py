#!/usr/bin/env python3
"""Offline coarse-to-fine tuning for the MPC IK horizontal circle demo.

The script does not start ROS and does not plot figures. It directly rolls the
MpcPinocchioIK solver over a horizontal circle with the initial end-effector
orientation locked, then ranks parameter sets by a balanced position/orientation
score. Use --max-combinations for quick smoke tests.
"""

import argparse
import contextlib
import csv
import itertools
import math
import os
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK
from pinocchio_ik import PinocchioIK


PUBLISH_RATE_HZ = 50.0
CONTROL_DT = 1.0 / PUBLISH_RATE_HZ
HORIZONTAL_CIRCLE_CENTER = np.array([0.35, 0.0, 0.15])
HORIZONTAL_CIRCLE_RADIUS = 0.08
HORIZONTAL_CIRCLE_PERIOD_SEC = 8.0
TERMINAL_TOLERANCE = 0.01

COARSE_HORIZON_STEPS = [8, 12, 16]
COARSE_MAX_JOINT_STEP = [0.08, 0.10]
COARSE_ORIENTATION_WEIGHT = [0.5, 1.0, 2.0]
COARSE_TERMINAL_ORIENTATION_WEIGHT = [5.0, 10.0, 20.0]

FINE_HORIZON_DELTA = [-2, 0, 2]
FINE_MAX_STEP_DELTA = [-0.01, 0.0, 0.01]
FINE_WEIGHT_SCALE = [0.75, 1.0, 1.25]


@dataclass(frozen=True, order=True)
class ParamSet:
    horizon_steps: int
    max_joint_step: float
    orientation_weight: float
    terminal_orientation_weight: float


@dataclass
class TrialResult:
    phase: str
    params: ParamSet
    orientation_target: str
    score: float
    rms_position_error: float
    max_position_error: float
    rms_orientation_error: float
    max_orientation_error: float
    rms_control_delta: float
    max_control_delta: float
    tracking_bad_count: int
    fallback_count: int
    solver_failure_count: int
    mean_solve_time_ms: float
    max_solve_time_ms: float
    samples: int
    recommended_command: str


def target_position(t: float) -> np.ndarray:
    w = 2.0 * math.pi / HORIZONTAL_CIRCLE_PERIOD_SEC
    return HORIZONTAL_CIRCLE_CENTER + HORIZONTAL_CIRCLE_RADIUS * np.array(
        [math.cos(w * t), math.sin(w * t), 0.0]
    )


def target_horizon(t: float, horizon_steps: int) -> np.ndarray:
    times = t + CONTROL_DT * np.arange(horizon_steps)
    return np.array([target_position(float(ti)) for ti in times])


def coarse_candidates() -> List[ParamSet]:
    return [
        ParamSet(
            horizon_steps=h,
            max_joint_step=s,
            orientation_weight=ow,
            terminal_orientation_weight=tow,
        )
        for h, s, ow, tow in itertools.product(
            COARSE_HORIZON_STEPS,
            COARSE_MAX_JOINT_STEP,
            COARSE_ORIENTATION_WEIGHT,
            COARSE_TERMINAL_ORIENTATION_WEIGHT,
        )
    ]


def fine_candidates(top_results: Sequence[TrialResult], already_seen: Iterable[ParamSet]) -> List[ParamSet]:
    seen = set(already_seen)
    candidates = set()
    for result in top_results:
        p = result.params
        horizons = sorted({max(1, p.horizon_steps + delta) for delta in FINE_HORIZON_DELTA})
        max_steps = sorted({round(p.max_joint_step + delta, 4) for delta in FINE_MAX_STEP_DELTA if p.max_joint_step + delta > 0.0})
        ori_weights = sorted({round(p.orientation_weight * scale, 4) for scale in FINE_WEIGHT_SCALE if p.orientation_weight * scale > 0.0})
        terminal_weights = sorted({
            round(p.terminal_orientation_weight * scale, 4)
            for scale in FINE_WEIGHT_SCALE
            if p.terminal_orientation_weight * scale > 0.0
        })
        for h, s, ow, tow in itertools.product(horizons, max_steps, ori_weights, terminal_weights):
            candidate = ParamSet(h, s, ow, tow)
            if candidate not in seen:
                candidates.add(candidate)
    return sorted(candidates)


def maybe_limit(candidates: List[ParamSet], max_combinations: Optional[int]) -> List[ParamSet]:
    if max_combinations is None or max_combinations <= 0:
        return candidates
    return candidates[:max_combinations]


def vertical_down_rotation() -> np.ndarray:
    # 末端 x 轴对齐世界 -Z,y 轴对齐世界 +Y,z 轴对齐世界 +X。
    return np.array(
        [
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
        ],
        dtype=float,
    )


def target_rotation_for_mode(mpc: MpcPinocchioIK, q_current: np.ndarray, orientation_target: str) -> np.ndarray:
    if orientation_target == "vertical_down":
        return vertical_down_rotation()
    _, locked_rotation = mpc.forward(q_current)
    return locked_rotation


def evaluate(
    params: ParamSet,
    backend: str,
    periods: float,
    phase: str,
    trial_index: int,
    orientation_target: str,
) -> TrialResult:
    config = MpcIkConfig(
        horizon_steps=params.horizon_steps,
        control_dt=CONTROL_DT,
        max_joint_step=params.max_joint_step,
        success_tolerance=TERMINAL_TOLERANCE,
        solver_backend=backend,
        orientation_weight=params.orientation_weight,
        terminal_orientation_weight=params.terminal_orientation_weight,
        acados_build_dir=str(acados_build_dir(params, phase, trial_index)),
    )
    mpc = MpcPinocchioIK(config=config)
    seed_ik = PinocchioIK()
    q_current, seed_ok = seed_ik.solve(target_position(0.0))
    if not seed_ok:
        raise RuntimeError("水平圆起始点 IK 不可达,请检查轨迹参数")

    locked_rotation = target_rotation_for_mode(mpc, q_current, orientation_target)
    warm_start = None
    if orientation_target != "initial":
        q_pose, pose_ok, pose_info = mpc.solve(
            target_position(0.0),
            q_init=q_current,
            target_rotation=locked_rotation,
        )
        if bool(pose_info.get("command_usable", pose_ok)):
            q_current = q_pose
            warm_start = pose_info["trajectory"]
    previous_q = q_current.copy()

    position_errors = []
    orientation_errors = []
    control_deltas = []
    solve_times_ms = []
    tracking_bad_count = 0
    fallback_count = 0
    solver_failure_count = 0

    steps = max(1, int(round(periods * HORIZONTAL_CIRCLE_PERIOD_SEC / CONTROL_DT)))
    for step in range(steps):
        t = step * CONTROL_DT
        targets = target_horizon(t, params.horizon_steps)

        start = time.perf_counter()
        q_next, ok, info = mpc.step(
            targets,
            q_current,
            warm_start=warm_start,
            target_rotation=locked_rotation,
        )
        wall_solve_time = time.perf_counter() - start
        solve_time_ms = 1000.0 * float(info.get("solve_time", wall_solve_time))
        solve_times_ms.append(solve_time_ms)
        warm_start = info["trajectory"]

        tracking_ok = bool(info.get("tracking_ok", ok))
        command_usable = bool(info.get("command_usable", ok))
        if not tracking_ok:
            tracking_bad_count += 1
        if int(info.get("status", 0)) != 0 or not bool(info.get("success", True)):
            solver_failure_count += 1

        if command_usable:
            q_current = q_next
        else:
            fallback_count += 1

        pos, rot = mpc.forward(q_current)
        pos_error = float(np.linalg.norm(pos - target_position(t)))
        ori_error = float(np.linalg.norm(mpc._orientation_error(rot, locked_rotation)))
        control_delta = float(np.linalg.norm(q_current - previous_q))

        position_errors.append(pos_error)
        orientation_errors.append(ori_error)
        control_deltas.append(control_delta)
        previous_q = q_current.copy()

    position_errors = np.asarray(position_errors, dtype=float)
    orientation_errors = np.asarray(orientation_errors, dtype=float)
    control_deltas = np.asarray(control_deltas, dtype=float)
    solve_times_ms = np.asarray(solve_times_ms, dtype=float)

    rms_position_error = rms(position_errors)
    max_position_error = float(np.max(position_errors))
    rms_orientation_error = rms(orientation_errors)
    max_orientation_error = float(np.max(orientation_errors))
    rms_control_delta = rms(control_deltas)
    max_control_delta = float(np.max(control_deltas))
    mean_solve_time_ms = float(np.mean(solve_times_ms))
    max_solve_time_ms = float(np.max(solve_times_ms))
    score = balanced_score(
        rms_position_error=rms_position_error,
        max_position_error=max_position_error,
        rms_orientation_error=rms_orientation_error,
        max_orientation_error=max_orientation_error,
        rms_control_delta=rms_control_delta,
        tracking_bad_count=tracking_bad_count,
        fallback_count=fallback_count,
        solver_failure_count=solver_failure_count,
        mean_solve_time_ms=mean_solve_time_ms,
    )

    return TrialResult(
        phase=phase,
        params=params,
        orientation_target=orientation_target,
        score=score,
        rms_position_error=rms_position_error,
        max_position_error=max_position_error,
        rms_orientation_error=rms_orientation_error,
        max_orientation_error=max_orientation_error,
        rms_control_delta=rms_control_delta,
        max_control_delta=max_control_delta,
        tracking_bad_count=tracking_bad_count,
        fallback_count=fallback_count,
        solver_failure_count=solver_failure_count,
        mean_solve_time_ms=mean_solve_time_ms,
        max_solve_time_ms=max_solve_time_ms,
        samples=steps,
        recommended_command=recommended_command(params, orientation_target),
    )


def balanced_score(
    rms_position_error: float,
    max_position_error: float,
    rms_orientation_error: float,
    max_orientation_error: float,
    rms_control_delta: float,
    tracking_bad_count: int,
    fallback_count: int,
    solver_failure_count: int,
    mean_solve_time_ms: float,
) -> float:
    return (
        1000.0 * rms_position_error
        + 1000.0 * max_position_error
        + 100.0 * rms_orientation_error
        + 100.0 * max_orientation_error
        + 50.0 * rms_control_delta
        + 0.2 * tracking_bad_count
        + 500.0 * fallback_count
        + 500.0 * solver_failure_count
        + 0.01 * mean_solve_time_ms
    )


def rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(values * values)))


def acados_build_dir(params: ParamSet, phase: str, trial_index: int) -> Path:
    name = (
        f"windylab_acados_tune_{phase}_{trial_index:04d}"
        f"_h{params.horizon_steps}"
        f"_s{int(round(params.max_joint_step * 1000.0))}"
        f"_ow{int(round(params.orientation_weight * 1000.0))}"
        f"_tow{int(round(params.terminal_orientation_weight * 1000.0))}"
    )
    return Path("/tmp") / name


def recommended_command(params: ParamSet, orientation_target: str = "initial") -> str:
    target_arg = ""
    if orientation_target != "initial":
        target_arg = f"-p orientation_target:={orientation_target} "
    return (
        "python3 move_arm_horizontal_circle_mpc_ik_demo.py --ros-args "
        f"-p horizon_steps:={params.horizon_steps} "
        f"-p max_joint_step:={params.max_joint_step:.4f} "
        "-p lock_orientation:=true "
        f"{target_arg}"
        f"-p orientation_weight:={params.orientation_weight:.4f} "
        f"-p terminal_orientation_weight:={params.terminal_orientation_weight:.4f}"
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


def load_top_results_from_csv(path: Path, top_count: int) -> List[TrialResult]:
    results: List[TrialResult] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = sorted(reader, key=lambda row: float(row["score"]))

    for row in rows[:top_count]:
        params = ParamSet(
            horizon_steps=int(row["horizon_steps"]),
            max_joint_step=float(row["max_joint_step"]),
            orientation_weight=float(row["orientation_weight"]),
            terminal_orientation_weight=float(row["terminal_orientation_weight"]),
        )
        results.append(
            TrialResult(
                phase=row.get("phase", "loaded"),
                params=params,
                orientation_target=row.get("orientation_target", "initial") or "initial",
                score=float(row["score"]),
                rms_position_error=float(row["rms_position_error"]),
                max_position_error=float(row["max_position_error"]),
                rms_orientation_error=float(row["rms_orientation_error"]),
                max_orientation_error=float(row["max_orientation_error"]),
                rms_control_delta=float(row["rms_control_delta"]),
                max_control_delta=float(row["max_control_delta"]),
                tracking_bad_count=int(row["tracking_bad_count"]),
                fallback_count=int(row["fallback_count"]),
                solver_failure_count=int(row["solver_failure_count"]),
                mean_solve_time_ms=float(row["mean_solve_time_ms"]),
                max_solve_time_ms=float(row["max_solve_time_ms"]),
                samples=int(row["samples"]),
                recommended_command=row.get("recommended_command") or recommended_command(
                    params,
                    row.get("orientation_target", "initial") or "initial",
                ),
            )
        )
    return results


def write_results(path: Path, results: Sequence[TrialResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "rank",
        "phase",
        "orientation_target",
        "score",
        "horizon_steps",
        "max_joint_step",
        "orientation_weight",
        "terminal_orientation_weight",
        "rms_position_error",
        "max_position_error",
        "rms_orientation_error",
        "max_orientation_error",
        "rms_control_delta",
        "max_control_delta",
        "tracking_bad_count",
        "fallback_count",
        "solver_failure_count",
        "mean_solve_time_ms",
        "max_solve_time_ms",
        "samples",
        "recommended_command",
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for rank, result in enumerate(sorted(results, key=lambda r: r.score), start=1):
            writer.writerow(result_to_row(rank, result))


def result_to_row(rank: int, result: TrialResult) -> Dict[str, object]:
    p = result.params
    return {
        "rank": rank,
        "phase": result.phase,
        "orientation_target": result.orientation_target,
        "score": f"{result.score:.6f}",
        "horizon_steps": p.horizon_steps,
        "max_joint_step": f"{p.max_joint_step:.6f}",
        "orientation_weight": f"{p.orientation_weight:.6f}",
        "terminal_orientation_weight": f"{p.terminal_orientation_weight:.6f}",
        "rms_position_error": f"{result.rms_position_error:.9f}",
        "max_position_error": f"{result.max_position_error:.9f}",
        "rms_orientation_error": f"{result.rms_orientation_error:.9f}",
        "max_orientation_error": f"{result.max_orientation_error:.9f}",
        "rms_control_delta": f"{result.rms_control_delta:.9f}",
        "max_control_delta": f"{result.max_control_delta:.9f}",
        "tracking_bad_count": result.tracking_bad_count,
        "fallback_count": result.fallback_count,
        "solver_failure_count": result.solver_failure_count,
        "mean_solve_time_ms": f"{result.mean_solve_time_ms:.6f}",
        "max_solve_time_ms": f"{result.max_solve_time_ms:.6f}",
        "samples": result.samples,
        "recommended_command": result.recommended_command,
    }


def print_top(results: Sequence[TrialResult], top_k: int, csv_path: Path) -> None:
    ranked = sorted(results, key=lambda r: r.score)
    print(f"\n调参结果已保存: {csv_path}")
    print(f"Top {min(top_k, len(ranked))} 参数组合:")
    header = (
        "rank score    h  step   ow    tow    "
        "rms_pos max_pos rms_ori max_ori bad fallback mean_ms max_ms"
    )
    print(header)
    for rank, result in enumerate(ranked[:top_k], start=1):
        p = result.params
        print(
            f"{rank:>4d} {result.score:>7.3f} "
            f"{p.horizon_steps:>3d} {p.max_joint_step:>6.3f} "
            f"{p.orientation_weight:>5.2f} {p.terminal_orientation_weight:>6.2f} "
            f"{result.rms_position_error:>7.4f} {result.max_position_error:>7.4f} "
            f"{result.rms_orientation_error:>7.4f} {result.max_orientation_error:>7.4f} "
            f"{result.tracking_bad_count:>3d} {result.fallback_count:>8d} "
            f"{result.mean_solve_time_ms:>7.2f} {result.max_solve_time_ms:>7.2f}"
        )

    if ranked:
        print("\n推荐运行命令:")
        print(ranked[0].recommended_command)


def run_phase(
    phase: str,
    candidates: List[ParamSet],
    backend: str,
    periods: float,
    max_combinations: Optional[int],
    start_index: int,
    quiet_build: bool,
    orientation_target: str,
) -> List[TrialResult]:
    limited_candidates = maybe_limit(candidates, max_combinations)
    if len(limited_candidates) < len(candidates):
        print(f"{phase} 搜索: 限制为 {len(limited_candidates)}/{len(candidates)} 组")
    else:
        print(f"{phase} 搜索: {len(limited_candidates)} 组")

    results = []
    total = len(limited_candidates)
    for offset, params in enumerate(limited_candidates, start=1):
        trial_index = start_index + offset - 1
        print(
            f"[{phase} {offset}/{total}] "
            f"h={params.horizon_steps}, step={params.max_joint_step:.3f}, "
            f"ow={params.orientation_weight:.3f}, tow={params.terminal_orientation_weight:.3f}",
            flush=True,
        )
        with suppress_native_output(quiet_build and backend == "acados"):
            result = evaluate(params, backend, periods, phase, trial_index, orientation_target)
        results.append(result)
        print(
            f"    score={result.score:.3f}, "
            f"rms_pos={result.rms_position_error:.4f}, max_pos={result.max_position_error:.4f}, "
            f"rms_ori={result.rms_orientation_error:.4f}, max_ori={result.max_orientation_error:.4f}, "
            f"bad={result.tracking_bad_count}, fallback={result.fallback_count}, "
            f"mean_ms={result.mean_solve_time_ms:.2f}",
            flush=True,
        )
    return results


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="粗到细搜索 MPC IK 参数")
    parser.add_argument("--backend", choices=("acados", "casadi", "scipy"), default="acados")
    parser.add_argument(
        "--orientation-target",
        choices=("initial", "vertical_down"),
        default="initial",
        help="锁姿态目标:initial 锁定起始 IK 姿态;vertical_down 锁定末端 x 轴竖直朝下",
    )
    parser.add_argument("--periods", type=float, default=1.0, help="仿真水平圆周期数")
    parser.add_argument("--top-k", type=int, default=10, help="终端打印前 K 名")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/home/windylab/code/windylab_ws/src/arm-platform/demo/data"),
    )
    parser.add_argument("--skip-fine", action="store_true", help="只运行粗搜索")
    parser.add_argument(
        "--fine-from-csv",
        type=Path,
        default=None,
        help="从已有粗搜索 CSV 的 Top 参数继续细搜索,不重复粗搜索",
    )
    parser.add_argument(
        "--fine-top",
        type=int,
        default=5,
        help="--fine-from-csv 或粗搜索后用于生成细搜索邻域的 Top 数量",
    )
    parser.add_argument(
        "--quiet-build",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="是否静音 ACADOS 代码生成和编译输出,默认静音;需要排查构建问题时可用 --no-quiet-build",
    )
    parser.add_argument(
        "--max-combinations",
        type=int,
        default=None,
        help="限制每个搜索阶段最多评估多少组参数;适合快速 smoke test",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.periods <= 0.0:
        raise ValueError("--periods 必须大于 0")
    if args.top_k < 1:
        raise ValueError("--top-k 必须大于等于 1")
    if args.fine_top < 1:
        raise ValueError("--fine-top 必须大于等于 1")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = args.output_dir / f"mpc_tuning_{stamp}.csv"
    all_results: List[TrialResult] = []

    if args.fine_from_csv is not None:
        loaded_results = load_top_results_from_csv(args.fine_from_csv, args.fine_top)
        if not loaded_results:
            raise RuntimeError(f"没有从 CSV 读到可用于细搜索的参数: {args.fine_from_csv}")
        print(f"从已有 CSV 读取 Top {len(loaded_results)} 生成细搜索邻域: {args.fine_from_csv}")
        fine = fine_candidates(loaded_results, already_seen=[r.params for r in loaded_results])
        fine_results = run_phase(
            "fine",
            fine,
            args.backend,
            args.periods,
            args.max_combinations,
            start_index=0,
            quiet_build=args.quiet_build,
            orientation_target=args.orientation_target,
        )
        if all(result.orientation_target == args.orientation_target for result in loaded_results):
            all_results.extend(loaded_results)
        else:
            print("已有 CSV 的姿态目标与本次不同,只使用其参数生成邻域,不混入旧评分。")
        all_results.extend(fine_results)
    else:
        coarse = coarse_candidates()
        coarse_results = run_phase(
            "coarse",
            coarse,
            args.backend,
            args.periods,
            args.max_combinations,
            start_index=0,
            quiet_build=args.quiet_build,
            orientation_target=args.orientation_target,
        )
        all_results.extend(coarse_results)

        if not args.skip_fine and coarse_results:
            top_for_fine = sorted(coarse_results, key=lambda r: r.score)[: args.fine_top]
            fine = fine_candidates(top_for_fine, already_seen=[r.params for r in all_results])
            fine_results = run_phase(
                "fine",
                fine,
                args.backend,
                args.periods,
                args.max_combinations,
                start_index=len(coarse_results),
                quiet_build=args.quiet_build,
                orientation_target=args.orientation_target,
            )
            all_results.extend(fine_results)

    write_results(csv_path, all_results)
    print_top(all_results, args.top_k, csv_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
