#!/usr/bin/env python3
"""Generate tracking, error, torque, and 3D reports from an SMC run."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import re
import numpy as np


def _load_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"No samples in {path}")
    return {
        key: np.asarray([float(row[key]) for row in rows], dtype=float)
        for key in rows[0]
    }


def _drawing_segments(points, pen_down, stroke_ids):
    segments = []
    current = []
    current_stroke = None
    for point, visible, stroke_id in zip(points, pen_down, stroke_ids):
        stroke_id = int(stroke_id)
        if not bool(visible):
            if len(current) >= 2:
                segments.append(np.asarray(current))
            current = []
            current_stroke = None
            continue
        if current_stroke is None or stroke_id != current_stroke:
            if len(current) >= 2:
                segments.append(np.asarray(current))
            current = [point]
            current_stroke = stroke_id
        else:
            current.append(point)
    if len(current) >= 2:
        segments.append(np.asarray(current))
    return segments


def generate_report(csv_path: str | Path, output_dir: str | Path) -> list[str]:
    csv_path = Path(csv_path).expanduser().resolve()
    output_dir = Path(output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    data = _load_csv(csv_path)
    t = data["t"]
    outputs: list[str] = []

    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for axis, name in zip(axes, "xyz"):
        axis.plot(t, data[f"desired_{name}_m"] * 1000.0, label="desired", linewidth=1.4)
        axis.plot(t, data[f"actual_{name}_m"] * 1000.0, label="actual", linewidth=1.0)
        axis.set_ylabel(f"{name} (mm)")
        axis.grid(True, alpha=0.3)
    axes[0].legend(loc="best")
    axes[-1].set_xlabel("time (s)")
    fig.suptitle("World-frame end-effector tracking")
    fig.tight_layout()
    path = output_dir / "01_xyz_tracking.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))

    fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    axes[0].plot(t, data["pos_error_m"] * 1000.0, color="#d62728")
    axes[0].axhline(1.0, color="#555555", linestyle="--", linewidth=1.0, label="1 mm RMS target")
    axes[0].axhline(2.0, color="#111111", linestyle=":", linewidth=1.0, label="2 mm peak target")
    axes[0].set_ylabel("position error (mm)")
    axes[0].legend(loc="best")
    axes[1].plot(t, np.rad2deg(data["rot_error_rad"]), color="#1f77b4")
    axes[1].set_ylabel("orientation error (deg)")
    axes[1].set_xlabel("time (s)")
    for axis in axes:
        axis.grid(True, alpha=0.3)
    fig.tight_layout()
    path = output_dir / "02_tracking_error.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))

    fig, axis = plt.subplots(figsize=(11, 6))
    # 关节数取自数据本身，不写死 —— 6 关节实物只有 6 列。
    # 数的必须是【真正要画的那些列】：CSV 里没有 q1/q2/… 这种列，
    # 原先按 q\d+ 数恒为 0，只是靠 `or 6` 兜底碰巧对上 6 关节，
    # 7 关节时会静默少画一条。（同一行还把 data 误写成了未定义的 df，
    # 导致 NameError，报告图【一张都没生成过】。）
    n_joints = sum(1 for c in data
                   if re.fullmatch(r'joint_\d+_torque_nm', str(c)))
    for i in range(n_joints):
        axis.plot(t, data[f"joint_{i + 1}_torque_nm"], label=f"joint{i + 1}", linewidth=0.9)
    axis.set_xlabel("time (s)")
    axis.set_ylabel("command torque (Nm)")
    axis.grid(True, alpha=0.3)
    axis.legend(ncol=4, loc="best")
    fig.tight_layout()
    path = output_dir / "03_joint_torque.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))

    fig = plt.figure(figsize=(9, 8))
    axis = fig.add_subplot(111, projection="3d")
    desired = np.column_stack([data[f"desired_{name}_m"] for name in "xyz"])
    actual = np.column_stack([data[f"actual_{name}_m"] for name in "xyz"])
    desired_segments = _drawing_segments(
        desired, data["pen_down"], data["stroke_id"])
    actual_segments = _drawing_segments(
        actual, data["pen_down"], data["stroke_id"])
    for index, segment in enumerate(desired_segments):
        stride = max(1, len(segment) // 500)
        axis.plot(
            *segment[::stride].T,
            color="#2ca02c",
            label="desired" if index == 0 else None,
            linewidth=2.2,
        )
    for index, segment in enumerate(actual_segments):
        stride = max(1, len(segment) // 500)
        axis.plot(
            *segment[::stride].T,
            color="#d62728",
            label="actual" if index == 0 else None,
            linewidth=2.8,
        )
    axis.set_xlabel("world x (m)")
    axis.set_ylabel("world y (m)")
    axis.set_zlabel("world z (m)")
    axis.legend(loc="best")
    fig.tight_layout()
    path = output_dir / "04_trajectory_3d.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))
    return outputs


def _newest_csv(data_dir: Path) -> Path:
    candidates = list(data_dir.glob("whole_task_smc_trajectory_*.csv"))
    if not candidates:
        raise FileNotFoundError(f"No whole-task SMC trajectory CSV in {data_dir}")
    return max(candidates, key=lambda item: item.stat().st_mtime)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=Path)
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "data",
    )
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    csv_path = args.csv or _newest_csv(args.data_dir)
    output_dir = args.output_dir or csv_path.with_name(
        csv_path.stem.replace("trajectory", "report"))
    print(json.dumps(generate_report(csv_path, output_dir), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
