#!/usr/bin/env python3
"""Evaluate LL100-LH420 ROS closed-loop bench outputs offline."""

import argparse
import csv
import json
import math
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
CONTROL_DT = 0.02
ARM_JOINT_NAMES = ["A_Joint", "B_Joint", "C_Joint", "D_Joint", "E_Joint", "F_Joint"]
EE_FRAME = "F_Link"
LINK_FRAMES = ("A_Link", "B_Link", "C_Link", "D_Link", "E_Link", "F_Link")
CONFIG_ORDER = (
    "default_lower",
    "default_circle",
    "default_front",
    "tuned_lower",
    "tuned_circle",
    "tuned_front",
)

DEFAULT_LL100_BODY_OBSTACLE_BOXES = (
    (0.05, 0.0, -0.12, 0.30, 0.24, 0.09),
    (-0.05, 0.0, 0.00, 0.24, 0.18, 0.08),
    (0.31, 0.0, -0.13, 0.09, 0.12, 0.05),
)
DEFAULT_CAPSULE_JOINT_RADII = (0.052, 0.062, 0.058, 0.050, 0.049, 0.040)
DEFAULT_CAPSULE_LINK_RADII = (0.045, 0.035, 0.043, 0.016, 0.035)
CAPSULE_CENTERLINE_POINTS = (
    ("A_Link", +0.01171, -0.00023, +0.00756, 0.04),
    ("B_Link", +0.00044, -0.00245, -0.00858, 0.05),
    ("B_Link", -0.00111, +0.08214, +0.00228, 0.0405),
    ("B_Link", +0.00004, +0.18057, -0.00091, 0.06),
    ("C_Link", -0.00708, +0.04951, -0.01685, 0.07),
    ("D_Link", -0.00005, -0.00001, +0.06566, 0.0248),
    ("D_Link", +0.00000, +0.00068, +0.12460, 0.0389),
    ("E_Link", +0.01872, +0.00203, +0.00453, 0.0457),
    ("E_Link", +0.06485, +0.00128, -0.02248, 0.0516),
    ("F_Link", +0.00004, -0.00001, -0.00189, 0.0223),
)


@dataclass
class Summary:
    config: str
    path_mode: str
    samples: int
    plant_hz: float
    latency_samples: int
    compute_mean_ms: float
    compute_p95_ms: float
    end_to_end_mean_ms: float
    end_to_end_p95_ms: float
    effective_hz: float
    rms_error_m: float
    max_error_m: float
    hf_err_rms_m: float
    cmd_acc_rms_rad_s2: float
    violation_max_m: float
    min_margin_m: float
    min_front_margin_m: float
    min_body_margin_m: float
    path_dist_rms_m: float = 0.0   # 到路径折线最近距离(相位无关走廊偏差)
    path_dist_max_m: float = 0.0
    aligned_rms_m: float = 0.0     # 最优相位对齐后的跟踪 RMS(消常数相位差)
    notes: str = ""


def rms(values: np.ndarray) -> float:
    if values.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(values * values)))


def _steady_slice(n: int) -> slice:
    return slice(int(0.2 * n), n)


def _high_frequency_error_rms(pos_errs: np.ndarray) -> float:
    if pos_errs.size == 0:
        return 0.0
    win = int(0.5 / CONTROL_DT)
    if win <= 1:
        return rms(pos_errs)
    if pos_errs.size < win:
        return rms(pos_errs - float(np.mean(pos_errs)))
    kernel = np.ones(win, dtype=float) / float(win)
    trend = np.convolve(pos_errs, kernel, mode="same")
    detrended = pos_errs - trend
    if detrended.size > 2 * win:
        detrended = detrended[win:-win]
    return rms(detrended) if detrended.size else 0.0


def _command_acceleration_rms(q_cmds: np.ndarray) -> float:
    if q_cmds.shape[0] < 3:
        return 0.0
    acc = np.diff(q_cmds, n=2, axis=0) / (CONTROL_DT * CONTROL_DT)
    return rms(acc)


def _fallback_build_path(mode: str, count: int) -> np.ndarray:
    def lerp(a, b, alpha):
        return (1.0 - alpha) * a + alpha * b

    def resample_polyline(points, n):
        pts = np.asarray(points, dtype=float)
        lengths = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        total = float(np.sum(lengths))
        if total < 1e-12:
            return np.repeat(pts[:1], n, axis=0)
        distances = np.concatenate([[0.0], np.cumsum(lengths)])
        samples = np.linspace(0.0, total, n, endpoint=False)
        out = []
        seg = 0
        for distance in samples:
            while seg + 1 < len(lengths) and distance > distances[seg + 1]:
                seg += 1
            alpha = (distance - distances[seg]) / max(lengths[seg], 1e-12)
            out.append(lerp(pts[seg], pts[seg + 1], alpha))
        return np.asarray(out, dtype=float)

    if mode == "circle":
        center = np.array([0.70, 0.0, 0.10], dtype=float)
        radius = 0.08
        return np.asarray(
            [
                center
                + np.array([0.0, radius * math.cos(2.0 * math.pi * i / count), radius * math.sin(2.0 * math.pi * i / count)])
                for i in range(count)
            ],
            dtype=float,
        )
    if mode == "lower_obstacle_circle":
        center = np.array([0.70, 0.02, -0.21], dtype=float)
        radius = 0.08
        return np.asarray(
            [
                center
                + np.array([radius * math.cos(2.0 * math.pi * i / count), radius * math.sin(2.0 * math.pi * i / count), 0.0])
                for i in range(count)
            ],
            dtype=float,
        )
    if mode == "avoidance_sweep":
        return resample_polyline(
            [
                [0.50, -0.18, 0.15],
                [0.49, -0.07, 0.14],
                [0.49, 0.00, 0.14],
                [0.49, 0.07, 0.14],
                [0.50, 0.18, 0.15],
                [0.56, 0.12, 0.16],
                [0.58, 0.00, 0.17],
                [0.56, -0.12, 0.16],
                [0.50, -0.18, 0.15],
            ],
            count,
        )
    if mode == "front_wing_arc":
        left = np.array([0.62, -0.24, -0.22], dtype=float)
        right = np.array([0.54, -0.06, -0.22], dtype=float)
        rise = 0.08
        pts = []
        for i in range(count):
            phase = 2.0 * i / max(1, count)
            tau = phase if phase <= 1.0 else 2.0 - phase
            s = 0.5 * (1.0 - math.cos(math.pi * tau))
            p = (1.0 - s) * left + s * right
            p = p.copy()
            p[2] += rise * math.sin(math.pi * s)
            pts.append(p)
        return np.asarray(pts, dtype=float)
    raise ValueError(f"unsupported path_mode={mode!r}")


def _load_path_builder() -> Callable[[str, int], np.ndarray]:
    try:
        from ll100_traj_pub import _build_path as build_path

        return build_path
    except Exception:
        return _fallback_build_path


def _path_polyline_distance(path: np.ndarray, pts: np.ndarray, chunk: int = 512) -> np.ndarray:
    """每个实际点到(循环)路径折线的最近距离——与相位无关的走廊偏差口径。"""
    seg_a = path
    seg_b = np.roll(path, -1, axis=0)
    ab = seg_b - seg_a
    ab_len2 = np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-12)
    out = np.empty(len(pts))
    for lo in range(0, len(pts), chunk):
        p = pts[lo:lo + chunk]                                # (c,3)
        ap = p[:, None, :] - seg_a[None, :, :]                 # (c,M,3)
        s = np.clip(np.einsum("cmj,mj->cm", ap, ab) / ab_len2[None, :], 0.0, 1.0)
        closest = seg_a[None, :, :] + s[..., None] * ab[None, :, :]
        out[lo:lo + chunk] = np.linalg.norm(p[:, None, :] - closest, axis=2).min(axis=1)
    return out


def _best_phase_rms(path: np.ndarray, actual: np.ndarray, base_phase: np.ndarray) -> np.ndarray:
    """在整数相位偏移上搜最优对齐(消掉限速追赶造成的常数相位差),返回对齐后的逐点误差。

    只用于消除"参考起点假设"的系统偏差;沿路径的真实滞后/超前仍会体现在误差里。
    """
    m = len(path)
    stride = max(1, len(actual) // 1500)
    sub_actual = actual[::stride]
    sub_phase = base_phase[::stride]
    best_k, best_val = 0, float("inf")
    for k in range(0, m, max(1, m // 720)):
        ref = _sample_cyclic_path(path, sub_phase + k)
        val = float(np.sqrt(np.mean(np.sum((sub_actual - ref) ** 2, axis=1))))
        if val < best_val:
            best_val, best_k = val, k
    ref_full = _sample_cyclic_path(path, base_phase + best_k)
    return np.linalg.norm(actual - ref_full, axis=1)


def _sample_cyclic_path(path: np.ndarray, phase: np.ndarray) -> np.ndarray:
    n = len(path)
    wrapped = np.mod(phase, n)
    i0 = np.floor(wrapped).astype(int)
    alpha = (wrapped - i0).reshape(-1, 1)
    i1 = (i0 + 1) % n
    return (1.0 - alpha) * path[i0] + alpha * path[i1]


def _read_json(path: Path) -> Dict[str, object]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def _float_or_nan(value: object) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def _load_latency_csv(path: Optional[Path]) -> Dict[str, float]:
    if path is None or not path.exists():
        return {
            "latency_samples": 0,
            "compute_mean_ms": float("nan"),
            "compute_p95_ms": float("nan"),
            "end_to_end_mean_ms": float("nan"),
            "end_to_end_p95_ms": float("nan"),
            "effective_hz": float("nan"),
        }
    elapsed = []
    compute = []
    e2e = []
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            elapsed.append(_float_or_nan(row.get("elapsed_sec")))
            compute.append(_float_or_nan(row.get("compute_sec")))
            e2e.append(_float_or_nan(row.get("end_to_end_sec")))
    elapsed_arr = np.asarray(elapsed, dtype=float)
    compute_arr = np.asarray(compute, dtype=float)
    e2e_arr = np.asarray(e2e, dtype=float)
    good = np.isfinite(elapsed_arr) & np.isfinite(compute_arr) & np.isfinite(e2e_arr)
    elapsed_arr = elapsed_arr[good]
    compute_arr = compute_arr[good]
    e2e_arr = e2e_arr[good]
    if compute_arr.size == 0:
        return _load_latency_csv(None)
    if elapsed_arr.size > 1 and elapsed_arr[-1] > elapsed_arr[0]:
        effective_hz = float((elapsed_arr.size - 1) / (elapsed_arr[-1] - elapsed_arr[0]))
    else:
        effective_hz = float("nan")
    return {
        "latency_samples": int(compute_arr.size),
        "compute_mean_ms": float(np.mean(compute_arr) * 1000.0),
        "compute_p95_ms": float(np.percentile(compute_arr, 95) * 1000.0),
        "end_to_end_mean_ms": float(np.mean(e2e_arr) * 1000.0),
        "end_to_end_p95_ms": float(np.percentile(e2e_arr, 95) * 1000.0),
        "effective_hz": effective_hz,
    }


def _find_latency_csv(run_dir: Path) -> Optional[Path]:
    preferred = run_dir / "latency.csv"
    if preferred.exists():
        return preferred
    files = sorted(run_dir.glob("latency*.csv"), key=lambda p: p.stat().st_mtime_ns)
    return files[-1] if files else None


def _load_plant_csv(path: Path) -> Dict[str, np.ndarray]:
    times = []
    command_seq = []
    q_rows = []
    cmd_rows = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            q = [_float_or_nan(row.get(f"q_{name}")) for name in ARM_JOINT_NAMES]
            cmd = [_float_or_nan(row.get(f"cmd_q_{name}")) for name in ARM_JOINT_NAMES]
            if not all(np.isfinite(q)):
                continue
            times.append(_float_or_nan(row.get("t_sec")))
            seq_value = _float_or_nan(row.get("command_seq"))
            command_seq.append(int(seq_value) if np.isfinite(seq_value) else 0)
            q_rows.append(q)
            cmd_rows.append(cmd)
    t = np.asarray(times, dtype=float)
    q = np.asarray(q_rows, dtype=float).reshape(-1, len(ARM_JOINT_NAMES))
    cmd_q = np.asarray(cmd_rows, dtype=float).reshape(-1, len(ARM_JOINT_NAMES))
    seq = np.asarray(command_seq, dtype=int)
    if t.size == 0:
        return {"t": t, "q": q, "cmd_q": cmd_q, "command_seq": seq}
    good = np.isfinite(t) & np.all(np.isfinite(q), axis=1)
    return {"t": t[good], "q": q[good], "cmd_q": cmd_q[good], "command_seq": seq[good]}


def _make_geometry():
    from mpc_pinocchio_ik import DEFAULT_LL100_URDF, MpcIkConfig, MpcPinocchioIK, PACKAGE_ROOT

    urdf = str(PACKAGE_ROOT / "config" / "ll100_lh420_nmpc.urdf")
    cfg = MpcIkConfig(
        horizon_steps=1,
        control_dt=CONTROL_DT,
        max_joint_step=0.01,
        solver_backend="acados",
        obstacle_avoidance_enabled=True,
        obstacle_safety_margin=0.005,
        obstacle_disk_half_thickness=0.02,
        obstacle_ll100_urdf_path=DEFAULT_LL100_URDF,
        obstacle_coordinate_mode="ll100_base",
        obstacle_keypoint_frames=LINK_FRAMES,
        front_obstacle_capsule=True,
        body_obstacle_enabled=True,
        body_obstacle_margin=0.005,
        body_obstacle_boxes=DEFAULT_LL100_BODY_OBSTACLE_BOXES,
        capsule_radius=0.018,
        capsule_joint_radii=DEFAULT_CAPSULE_JOINT_RADII,
        capsule_link_radii=DEFAULT_CAPSULE_LINK_RADII,
        capsule_points=CAPSULE_CENTERLINE_POINTS,
        capsule_samples_per_segment=3,
        capsule_frame_sequence=LINK_FRAMES,
        obstacle_repulsion_margin=0.03,
        obstacle_repulsion_weight=1.0e4,
    )
    return MpcPinocchioIK(urdf_path=urdf, ee_frame=EE_FRAME, config=cfg)


def _disk_signed_distance(point: np.ndarray, disk: np.ndarray) -> float:
    rel = np.asarray(point, dtype=float) - disk[:3]
    normal = disk[3:6]
    d_axial = float(normal @ rel)
    lateral = rel - d_axial * normal
    excess = max(float(np.linalg.norm(lateral)) - float(disk[6]), 0.0)
    return float(math.sqrt(excess * excess + d_axial * d_axial))


def _box_signed_distance(point: np.ndarray, box: np.ndarray) -> float:
    q = np.abs(np.asarray(point, dtype=float) - box[:3]) - box[3:]
    outside = np.maximum(q, 0.0)
    outside_dist = float(np.linalg.norm(outside))
    inside_dist = float(min(max(q[0], q[1], q[2]), 0.0))
    return outside_dist + inside_dist


def _safety_metrics(mpc, q_rows: np.ndarray) -> Dict[str, float]:
    disks = mpc.obstacle_disks()
    boxes = mpc.body_obstacle_boxes()
    front_margin = float("inf")
    body_margin = float("inf")
    violation = 0.0
    for q in q_rows:
        samples = mpc.capsule_sample_positions(q)
        radii = mpc.capsule_sample_radii()
        if samples.size == 0:
            points = list(mpc.obstacle_keypoint_positions(q).values())
            samples = np.asarray(points, dtype=float)
            radii = np.zeros(len(points), dtype=float)
        for sample, radius in zip(samples, radii):
            for disk in disks:
                margin = _disk_signed_distance(sample, disk) - float(radius) - float(mpc.config.obstacle_safety_margin)
                front_margin = min(front_margin, margin)
                violation = max(violation, max(0.0, -margin))
            for box in boxes:
                margin = _box_signed_distance(sample, box) - float(radius) - float(mpc.config.body_obstacle_margin)
                body_margin = min(body_margin, margin)
                violation = max(violation, max(0.0, -margin))
    margins = [m for m in (front_margin, body_margin) if np.isfinite(m)]
    return {
        "violation_max_m": float(violation),
        "min_margin_m": float(min(margins)) if margins else float("nan"),
        "min_front_margin_m": float(front_margin) if np.isfinite(front_margin) else float("nan"),
        "min_body_margin_m": float(body_margin) if np.isfinite(body_margin) else float("nan"),
    }


def _infer_path_mode(run_dir: Path, meta: Dict[str, object]) -> str:
    if meta.get("path_mode"):
        return str(meta["path_mode"])
    name = run_dir.name
    if "circle" in name and "lower" not in name:
        return "circle"
    if "front" in name:
        return "front_wing_arc"
    return "lower_obstacle_circle"


def _period_sec(meta: Dict[str, object]) -> float:
    for key in ("demo_params", "config_params"):
        params = meta.get(key)
        if isinstance(params, dict) and "period_sec" in params:
            return float(params["period_sec"])
    return 20.0


def evaluate_run(run_dir: Path) -> Summary:
    meta = _read_json(run_dir / "run_meta.json")
    path_mode = _infer_path_mode(run_dir, meta)
    period_sec = _period_sec(meta)
    plant_path = run_dir / "plant.csv"
    if not plant_path.exists():
        raise FileNotFoundError(f"missing plant.csv in {run_dir}")
    plant = _load_plant_csv(plant_path)
    if plant["t"].size == 0:
        raise ValueError(f"empty plant CSV: {plant_path}")
    command_mask = plant["command_seq"] > 0
    if not np.any(command_mask):
        raise ValueError(f"plant CSV has no command rows: {plant_path}")
    first = int(np.argmax(command_mask))
    t = plant["t"][first:] - plant["t"][first]
    q_rows = plant["q"][first:]
    cmd_q = plant["cmd_q"][first:]
    finite_cmd = np.all(np.isfinite(cmd_q), axis=1)
    cmd_q = cmd_q[finite_cmd]

    if t.size > 1 and t[-1] > t[0]:
        plant_hz = float((t.size - 1) / (t[-1] - t[0]))
    else:
        plant_hz = float("nan")

    mpc = _make_geometry()
    actual_pos = np.asarray([mpc.forward(q)[0] for q in q_rows], dtype=float)
    build_path = _load_path_builder()
    path = build_path(path_mode, max(20, int(round(period_sec / CONTROL_DT))))
    nearest = int(np.argmin(np.linalg.norm(path - actual_pos[0].reshape(1, 3), axis=1)))
    phase = nearest + (t / period_sec) * len(path)
    reference = _sample_cyclic_path(path, phase)
    errors = np.linalg.norm(actual_pos - reference, axis=1)
    steady = _steady_slice(len(errors))
    steady_errors = errors[steady]
    steady_actual = actual_pos[steady]
    path_dists = _path_polyline_distance(path, steady_actual)
    aligned_errors = _best_phase_rms(path, steady_actual, phase[steady])
    steady_cmd_q = cmd_q[_steady_slice(len(cmd_q))] if cmd_q.size else np.zeros((0, len(ARM_JOINT_NAMES)))

    latency = _load_latency_csv(_find_latency_csv(run_dir))
    safety = _safety_metrics(mpc, plant["q"])

    return Summary(
        config=run_dir.name,
        path_mode=path_mode,
        samples=int(q_rows.shape[0]),
        plant_hz=plant_hz,
        latency_samples=int(latency["latency_samples"]),
        compute_mean_ms=float(latency["compute_mean_ms"]),
        compute_p95_ms=float(latency["compute_p95_ms"]),
        end_to_end_mean_ms=float(latency["end_to_end_mean_ms"]),
        end_to_end_p95_ms=float(latency["end_to_end_p95_ms"]),
        effective_hz=float(latency["effective_hz"]),
        rms_error_m=rms(steady_errors),
        max_error_m=float(np.max(steady_errors)) if steady_errors.size else 0.0,
        hf_err_rms_m=_high_frequency_error_rms(steady_errors),
        cmd_acc_rms_rad_s2=_command_acceleration_rms(steady_cmd_q),
        violation_max_m=float(safety["violation_max_m"]),
        min_margin_m=float(safety["min_margin_m"]),
        min_front_margin_m=float(safety["min_front_margin_m"]),
        min_body_margin_m=float(safety["min_body_margin_m"]),
        path_dist_rms_m=rms(path_dists),
        path_dist_max_m=float(np.max(path_dists)) if path_dists.size else 0.0,
        aligned_rms_m=rms(aligned_errors),
        notes=(
            "reference aligned to nearest path point at first command row; "
            "t is plant CSV time minus that row; safety evaluated on all plant CSV q rows"
        ),
    )


def _summary_dirs(root: Path) -> List[Path]:
    ordered = [root / name for name in CONFIG_ORDER if (root / name / "plant.csv").exists()]
    seen = {p.resolve() for p in ordered}
    extras = sorted(
        p for p in root.iterdir()
        if p.is_dir() and (p / "plant.csv").exists() and p.resolve() not in seen
    )
    return ordered + extras


def _fmt(value: float, digits: int = 6) -> str:
    if not np.isfinite(value):
        return ""
    return f"{value:.{digits}g}"


def write_outputs(summaries: List[Summary], output_dir: Path) -> Tuple[Path, Path]:
    csv_path = output_dir / "ros_closed_loop_summary.csv"
    md_path = output_dir / "ros_closed_loop_summary.md"
    fields = list(Summary.__dataclass_fields__.keys())
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for s in summaries:
            writer.writerow(s.__dict__)

    md_cols = [
        "config",
        "path",
        "N",
        "compute mean ms",
        "compute p95 ms",
        "e2e p95 ms",
        "hz",
        "rms mm",
        "max mm",
        "path rms mm",
        "path max mm",
        "aligned rms mm",
        "hf mm",
        "cmd acc",
        "viol mm",
        "min margin mm",
    ]
    lines = [
        "| " + " | ".join(md_cols) + " |",
        "| " + " | ".join(["---"] * len(md_cols)) + " |",
    ]
    for s in summaries:
        lines.append(
            "| "
            + " | ".join(
                [
                    s.config,
                    s.path_mode,
                    str(s.samples),
                    _fmt(s.compute_mean_ms, 4),
                    _fmt(s.compute_p95_ms, 4),
                    _fmt(s.end_to_end_p95_ms, 4),
                    _fmt(s.effective_hz, 4),
                    _fmt(1000.0 * s.rms_error_m, 4),
                    _fmt(1000.0 * s.max_error_m, 4),
                    _fmt(1000.0 * s.path_dist_rms_m, 4),
                    _fmt(1000.0 * s.path_dist_max_m, 4),
                    _fmt(1000.0 * s.aligned_rms_m, 4),
                    _fmt(1000.0 * s.hf_err_rms_m, 4),
                    _fmt(s.cmd_acc_rms_rad_s2, 4),
                    _fmt(1000.0 * s.violation_max_m, 4),
                    _fmt(1000.0 * s.min_margin_m, 4),
                ]
            )
            + " |"
        )
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return csv_path, md_path


def evaluate_root(root: Path) -> List[Summary]:
    summaries = []
    for run_dir in _summary_dirs(root):
        summary = evaluate_run(run_dir)
        summaries.append(summary)
        print_summary(summary)
    return summaries


def print_summary(summary: Summary) -> None:
    print(
        f"{summary.config}: compute_p95={summary.compute_p95_ms:.3f}ms "
        f"e2e_p95={summary.end_to_end_p95_ms:.3f}ms "
        f"rms={summary.rms_error_m * 1000.0:.2f}mm "
        f"hf={summary.hf_err_rms_m * 1000.0:.2f}mm "
        f"viol={summary.violation_max_m * 1000.0:.3f}mm",
        flush=True,
    )


def _write_synthetic_run(run_dir: Path, samples: int = 240) -> None:
    import pinocchio as pin

    from mpc_pinocchio_ik import PACKAGE_ROOT

    run_dir.mkdir(parents=True, exist_ok=True)
    meta = {
        "config_name": run_dir.name,
        "path_mode": "lower_obstacle_circle",
        "demo_params": {"path_mode": "lower_obstacle_circle", "period_sec": 20.0},
        "duration_sec": samples * CONTROL_DT,
    }
    (run_dir / "run_meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")

    model = pin.buildModelFromUrdf(str(PACKAGE_ROOT / "config" / "ll100_lh420_nmpc.urdf"))
    q0 = np.asarray(pin.neutral(model), dtype=float)
    with (run_dir / "plant.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "t_sec",
                "ros_time_sec",
                "command_seq",
                "command_age_sec",
                *[f"q_{name}" for name in ARM_JOINT_NAMES],
                *[f"v_{name}" for name in ARM_JOINT_NAMES],
                *[f"cmd_q_{name}" for name in ARM_JOINT_NAMES],
                *[f"cmd_v_{name}" for name in ARM_JOINT_NAMES],
            ]
        )
        for i in range(samples):
            t = i * CONTROL_DT
            wiggle = 0.02 * math.sin(2.0 * math.pi * t / 3.0)
            q = q0.copy()
            q[0] += wiggle
            q[1] += 0.5 * wiggle
            v = np.zeros_like(q)
            cmd = q + 0.002 * np.sin(2.0 * math.pi * t)
            cmd_v = np.zeros_like(q)
            writer.writerow([t, 1000.0 + t, i + 1, 0.001, *q.tolist(), *v.tolist(), *cmd.tolist(), *cmd_v.tolist()])

    with (run_dir / "latency.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["tick", "elapsed_sec", "feedback_age_sec", "compute_sec", "end_to_end_sec"])
        for i in range(80):
            elapsed = 5.0 + i * CONTROL_DT
            compute = 0.006 + 0.0005 * math.sin(i)
            feedback = 0.009 + 0.0002 * math.cos(i)
            writer.writerow([i + 1, elapsed, feedback, compute, feedback + compute])


def run_self_test() -> Path:
    root = Path(tempfile.mkdtemp(prefix="ll100_ros_closed_loop_eval_selftest_"))
    _write_synthetic_run(root / "default_lower")
    summaries = evaluate_root(root)
    write_outputs(summaries, root)
    print(f"self-test output: {root}", flush=True)
    return root


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, help="Single config directory")
    parser.add_argument("--root", type=Path, help="Root containing config directories")
    parser.add_argument("--self-test", action="store_true", help="Generate synthetic CSVs and dry-run evaluation")
    args = parser.parse_args()

    if args.self_test:
        run_self_test()
        return 0
    if bool(args.run_dir) == bool(args.root):
        parser.error("provide exactly one of --run-dir or --root, or use --self-test")

    if args.run_dir:
        run_dir = args.run_dir.expanduser().resolve()
        summaries = [evaluate_run(run_dir)]
        print_summary(summaries[0])
        out_dir = run_dir
    else:
        out_dir = args.root.expanduser().resolve()
        summaries = evaluate_root(out_dir)
    csv_path, md_path = write_outputs(summaries, out_dir)
    print(f"wrote {csv_path}", flush=True)
    print(f"wrote {md_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
