#!/usr/bin/env python3
"""Compact first-order vs second-order low-speed A/B report."""

import argparse
import json
from pathlib import Path


def get(obj, path):
    for key in path.split("."):
        obj = obj[key]
    return obj


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("first_order", type=Path)
    parser.add_argument("second_order", type=Path)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    first = json.loads(args.first_order.read_text())
    second = json.loads(args.second_order.read_text())
    paths = {
        "actual_mean_mm": "actual_error.steady.mean_mm",
        "actual_rms_mm": "actual_error.steady.rms_mm",
        "actual_p95_mm": "actual_error.steady.p95_mm",
        "command_p95_mm": "cmd_error_at_next_tick.steady.p95_mm",
        "exec_gap_rms_mm": "exec_gap_cartesian.steady.rms_mm",
        "solve_p99_ms": "solver.solve_time_ms.p99",
        "guard_blocks": "guard.block_count",
        "solver_nonzero": "solver.nonzero_status_count",
        "command_usable_ratio": "solver.command_usable_ratio",
    }
    report = {"first_order": {}, "second_order": {}, "delta_second_minus_first": {}}
    for name, path in paths.items():
        a, b = get(first, path), get(second, path)
        report["first_order"][name] = a
        report["second_order"][name] = b
        report["delta_second_minus_first"][name] = b - a
    report["experiment_valid"] = bool(
        first.get("experiment_valid") and second.get("experiment_valid")
        and report["first_order"]["solver_nonzero"] == 0
        and report["second_order"]["solver_nonzero"] == 0
        and report["first_order"]["solve_p99_ms"] < 20.0
        and report["second_order"]["solve_p99_ms"] < 20.0
    )
    first_p95 = report["first_order"]["actual_p95_mm"]
    second_p95 = report["second_order"]["actual_p95_mm"]
    report["promotion_pass"] = bool(
        report["experiment_valid"]
        and second_p95 <= first_p95 * 1.10 + 0.1
        and report["second_order"]["guard_blocks"]
        <= report["first_order"]["guard_blocks"]
    )
    report["recommendation"] = (
        "candidate may proceed to repeated A/B"
        if report["promotion_pass"]
        else "keep first_order production default")
    text = json.dumps(report, indent=2, ensure_ascii=False)
    print(text)
    if args.out:
        args.out.write_text(text)
    return 0 if report["experiment_valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
