#!/usr/bin/env python3
"""Summarize the second-order solver while first-order remains in control."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def _finite(values):
    values = np.asarray(values, dtype=float)
    return values[np.isfinite(values)]


def analyze(path: Path) -> dict:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"empty loop CSV: {path}")

    def col(name):
        return np.asarray([float(row[name]) for row in rows], dtype=float)

    available = col("shadow_available") > 0.5
    status = col("shadow_status")
    usable = col("shadow_command_usable") > 0.5
    solve_ms = _finite(col("shadow_solve_time_ms")[available])
    q_delta = _finite(col("shadow_q_delta_norm_rad")[available])
    fk_delta = _finite(col("shadow_fk_delta_m")[available])
    n = len(rows)
    result = {
        "loop_csv": str(path),
        "ticks": n,
        "availability_ratio": float(np.mean(available)),
        "command_usable_ratio": float(np.mean(usable)),
        "nonzero_status_count": int(np.sum(available & (status != 0))),
        "solve_time_ms": {
            "p50": float(np.percentile(solve_ms, 50)) if solve_ms.size else None,
            "p95": float(np.percentile(solve_ms, 95)) if solve_ms.size else None,
            "p99": float(np.percentile(solve_ms, 99)) if solve_ms.size else None,
            "max": float(np.max(solve_ms)) if solve_ms.size else None,
        },
        "first_vs_second": {
            "q_next_norm_rad_median": float(np.median(q_delta)) if q_delta.size else None,
            "q_next_norm_rad_p95": float(np.percentile(q_delta, 95)) if q_delta.size else None,
            "fk_delta_mm_median": float(np.median(fk_delta) * 1000.0) if fk_delta.size else None,
            "fk_delta_mm_p95": float(np.percentile(fk_delta, 95) * 1000.0) if fk_delta.size else None,
        },
    }
    p99 = result["solve_time_ms"]["p99"]
    failures = []
    if result["availability_ratio"] < 0.995:
        failures.append("shadow availability < 99.5%")
    if result["nonzero_status_count"]:
        failures.append("second-order solver returned nonzero status")
    if p99 is None or p99 >= 20.0:
        failures.append("second-order solve-time P99 >= 20 ms")
    result["ready_for_low_speed_ab"] = not failures
    result["failures"] = failures
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("loop_csv", type=Path)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    result = analyze(args.loop_csv)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    if args.out:
        args.out.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result["ready_for_low_speed_ab"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
