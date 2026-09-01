#!/usr/bin/env python3
"""Run Phase4 zero/ratio ablations from a baseline arm6 calibration YAML.

The script creates temporary candidate YAMLs and runs
phase4_gravity_compensation.py for each case.  It never modifies the official
calibration YAML.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import subprocess
import sys
import time
from itertools import combinations
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
CONFIG_DIR = HERE.parents[1] / "config"
DEFAULT_OFFICIAL = CONFIG_DIR / "arm6_calibration.yaml"
DEFAULT_CANDIDATE = CONFIG_DIR / "arm6_calibration_candidate.yaml"
DEFAULT_OUT = HERE / "data" / "phase4_ablation"
PHASE4 = HERE / "phase4_gravity_compensation.py"
RESULT_RE = re.compile(r"results ->\s*(\S+)")


def load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text()) or {}


def dump_yaml(path: Path, data: dict) -> None:
    path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))


def phase4_uses_stiction() -> bool:
    """Return True only if stiction is used outside loading/logging."""
    text = PHASE4.read_text()
    active = []
    for line in text.splitlines():
        stripped = line.strip()
        if "stiction" not in stripped:
            continue
        if stripped.startswith("self.stiction ="):
            continue
        if "stiction_nm:" in stripped:
            continue
        active.append(stripped)
    return bool(active)


def make_cases() -> list[tuple[str, tuple[int, ...]]]:
    joints = (0, 1, 2)
    cases: list[tuple[str, tuple[int, ...]]] = [("baseline", ())]
    for n in (1, 2, 3):
        for combo in combinations(joints, n):
            name = "J" + "_J".join(str(i + 1) for i in combo)
            cases.append((name, combo))
    return cases


def build_yaml(base: dict, source: dict, combo: tuple[int, ...]) -> dict:
    data = json.loads(json.dumps(base))
    for key in ("zero_offset_rad", "zero_offset_deg", "first_moment_ratio"):
        vals = list(data[key])
        src = list(source[key])
        for idx in combo:
            vals[idx] = src[idx]
        data[key] = vals
    conv = list(data.get("zero_offset_converged", [False] * 6))
    src_conv = list(source.get("zero_offset_converged", conv))
    for idx in combo:
        conv[idx] = src_conv[idx]
    data["zero_offset_converged"] = conv
    data["ablation_note"] = {
        "baseline": str(DEFAULT_OFFICIAL),
        "phase3_source": str(DEFAULT_CANDIDATE),
        "replaced_zero_ratio_joints": [int(i + 1) for i in combo],
        "writes_official_yaml": False,
    }
    return data


def parse_phase4_report(path: Path | None) -> dict:
    if path is None or not path.exists():
        return {"report": str(path) if path else None, "levels": []}
    data = json.loads(path.read_text())
    levels = []
    for rec in data.get("levels", []):
        item = {
            "name": rec.get("name"),
            "ok": bool(rec.get("ok")),
            "reason": rec.get("reason"),
            "residual_max_rad": rec.get("residual_max"),
            "residual_max_deg": (
                float(rec["residual_max"]) * 180.0 / 3.141592653589793
                if rec.get("residual_max") is not None else None
            ),
            "drift_ptp_rad": rec.get("drift_ptp"),
        }
        levels.append(item)
    return {"report": str(path), "levels": levels}


def run_case(name: str, yaml_path: Path, out_dir: Path, args) -> dict:
    cmd = [
        sys.executable,
        str(PHASE4),
        "--calibration-yaml",
        str(yaml_path),
    ]
    if args.ignore_workspace_guard:
        cmd.append("--ignore-workspace-guard")
    if args.max_level is not None:
        cmd += ["--max-level", str(args.max_level)]

    print(f"\n===== PHASE4 ABLATION {name}: {yaml_path} =====", flush=True)
    proc = subprocess.run(cmd, text=True, capture_output=True)
    log = out_dir / f"{name}.log"
    log.write_text(proc.stdout + proc.stderr)
    print(proc.stdout, end="", flush=True)
    if proc.stderr:
        print(proc.stderr, end="", file=sys.stderr, flush=True)

    report = None
    match = RESULT_RE.search(proc.stdout + proc.stderr)
    if match:
        report = Path(match.group(1))
    parsed = parse_phase4_report(report)
    parsed.update({"case": name, "yaml": str(yaml_path), "returncode": proc.returncode, "log": str(log)})
    return parsed


def summarize_rows(results: list[dict]) -> list[dict]:
    rows = []
    for result in results:
        row = {"case": result["case"], "report": result.get("report"), "returncode": result.get("returncode")}
        failed = None
        for level in result.get("levels", []):
            lname = str(level.get("name", "")).split()[0]
            if level.get("ok"):
                row[f"{lname}_deg"] = level.get("residual_max_deg")
                row[f"{lname}_drift"] = level.get("drift_ptp_rad")
            elif failed is None:
                failed = f"{level.get('name')}: {level.get('reason')}"
        row["failure"] = failed
        rows.append(row)
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--official", type=Path, default=DEFAULT_OFFICIAL)
    ap.add_argument("--candidate", type=Path, default=DEFAULT_CANDIDATE)
    ap.add_argument("--out-dir", type=Path, default=None)
    ap.add_argument("--settle-sec", type=float, default=8.0)
    ap.add_argument("--max-level", type=int, default=None)
    ap.add_argument("--ignore-workspace-guard", action="store_true")
    args = ap.parse_args()

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = args.out_dir or (DEFAULT_OUT / stamp)
    yaml_dir = out_dir / "yamls"
    yaml_dir.mkdir(parents=True, exist_ok=True)

    official = load_yaml(args.official)
    candidate = load_yaml(args.candidate)
    stiction_active = phase4_uses_stiction()
    print(f"official baseline: {args.official}")
    print(f"phase3 candidate : {args.candidate}")
    print(f"phase4 uses stiction in control/protection: {stiction_active}")

    results = []
    cases = make_cases()
    for idx, (name, combo) in enumerate(cases):
        data = build_yaml(official, candidate, combo)
        yaml_path = yaml_dir / f"{idx:02d}_{name}.yaml"
        dump_yaml(yaml_path, data)
        if idx > 0 and args.settle_sec > 0:
            time.sleep(args.settle_sec)
        results.append(run_case(name, yaml_path, out_dir, args))

    summary = {
        "official": str(args.official),
        "candidate": str(args.candidate),
        "phase4_uses_stiction": stiction_active,
        "results": results,
    }
    summary_path = out_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False))

    rows = summarize_rows(results)
    csv_path = out_dir / "summary.csv"
    fieldnames = sorted({k for row in rows for k in row.keys()})
    with csv_path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print("\n===== SUMMARY =====")
    for row in rows:
        compact = {k: row.get(k) for k in ("case", "L0_deg", "L1_deg", "L2_deg", "L3_deg", "L4_deg", "failure")}
        print(json.dumps(compact, ensure_ascii=False))
    print(f"summary_json: {summary_path}")
    print(f"summary_csv : {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
