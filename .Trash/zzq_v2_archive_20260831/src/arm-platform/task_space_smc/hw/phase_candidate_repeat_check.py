#!/usr/bin/env python3
"""Repeat Phase4/Phase5 with a candidate calibration from a fixed posture."""

from __future__ import annotations

import argparse
import csv
import json
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
CONFIG_DIR = HERE.parents[1] / "config"
DEFAULT_CANDIDATE = CONFIG_DIR / "arm6_calibration_candidate.yaml"
DEFAULT_OUT = HERE / "data" / "candidate_repeat"
PHASE4 = HERE / "phase4_gravity_compensation.py"
PHASE5 = HERE / "phase5_inertia_id.py"
RESULT_RE = re.compile(r"results ->\s*(\S+)")


def parse_report_path(text: str) -> Path | None:
    match = RESULT_RE.search(text)
    return Path(match.group(1)) if match else None


def run(cmd: list[str], log_path: Path) -> tuple[int, Path | None]:
    proc = subprocess.run(cmd, text=True, capture_output=True)
    text = proc.stdout + proc.stderr
    log_path.write_text(text)
    print(proc.stdout, end="", flush=True)
    if proc.stderr:
        print(proc.stderr, end="", file=sys.stderr, flush=True)
    return proc.returncode, parse_report_path(text)


def phase4_row(report: Path | None, repeat: int) -> dict:
    row = {"stage": "phase4", "repeat": repeat, "report": str(report) if report else None}
    if report is None or not report.exists():
        row["failure"] = "missing report"
        return row
    data = json.loads(report.read_text())
    for rec in data.get("levels", []):
        lname = str(rec.get("name", "")).split()[0]
        if rec.get("ok"):
            row[f"{lname}_deg"] = float(rec["residual_max"]) * 180.0 / np.pi
            row[f"{lname}_drift"] = rec.get("drift_ptp")
        elif not row.get("failure"):
            row["failure"] = f"{rec.get('name')}: {rec.get('reason')}"
    return row


def phase5_rows(report: Path | None, repeat: int) -> list[dict]:
    rows = []
    if report is None or not report.exists():
        return [{"stage": "phase5", "repeat": repeat, "joint": None,
                 "report": str(report) if report else None, "failure": "missing report"}]
    data = json.loads(report.read_text())
    for rec in data.get("results", []):
        if rec.get("pos") and rec.get("neg"):
            align = rec.get("alignment") or {}
            row = {
                "stage": "phase5",
                "repeat": repeat,
                "joint": rec.get("joint"),
                "report": str(report),
                "ok": rec.get("ok"),
                "M_pos": rec["pos"].get("M"),
                "M_neg": rec["neg"].get("M"),
                "M_mean": rec.get("M_mean"),
                "M_split_abs": rec.get("M_split_abs"),
                "friction_pos": rec["pos"].get("friction_dynamic"),
                "friction_neg": rec["neg"].get("friction_dynamic"),
                "friction_measured": rec.get("friction_measured"),
                "alignment_lag_sec": align.get("lag_sec_mean"),
                "alignment_corr_abs": align.get("corr_abs_mean"),
                "failure": rec.get("reason"),
            }
            rows.append(row)
            continue
        row = {
            "stage": "phase5",
            "repeat": repeat,
            "joint": rec.get("joint"),
            "report": str(report),
            "ok": rec.get("ok"),
            "M": rec.get("M"),
            "M_stderr": rec.get("M_stderr"),
            "friction_implied": rec.get("friction_implied"),
            "friction_measured": rec.get("friction_measured"),
            "n_pulses": rec.get("n_pulses"),
            "failure": rec.get("reason"),
        }
        rows.append(row)
    return rows


def summarize_phase5(rows: list[dict]) -> list[dict]:
    out = []
    joints = sorted({r.get("joint") for r in rows if r.get("joint") is not None})
    for joint in joints:
        jr = [r for r in rows if r.get("joint") == joint and r.get("ok")]
        if jr and any("M_pos" in r for r in jr):
            item = {"joint": joint, "n_ok": len(jr),
                    "friction_measured": next((r.get("friction_measured") for r in jr), None)}
            for key in ("M_pos", "M_neg", "M_mean", "M_split_abs",
                        "friction_pos", "friction_neg",
                        "alignment_lag_sec", "alignment_corr_abs"):
                vals = np.array([r[key] for r in jr if r.get(key) is not None], dtype=float)
                if vals.size:
                    item[f"{key}_mean"] = float(vals.mean())
                    item[f"{key}_std"] = float(vals.std(ddof=1)) if vals.size > 1 else 0.0
                    item[f"{key}_values"] = vals.tolist()
            for key in ("friction_pos", "friction_neg"):
                vals = [r[key] for r in jr if r.get(key) is not None]
                item[f"{key}_signs"] = [int(np.sign(v)) for v in vals]
            out.append(item)
            continue
        vals = np.array([r["M"] for r in jr if r.get("M") is not None], dtype=float)
        fr = np.array([r["friction_implied"] for r in jr
                       if r.get("friction_implied") is not None], dtype=float)
        fm = next((r.get("friction_measured") for r in jr
                   if r.get("friction_measured") is not None), None)
        item = {"joint": joint, "n_ok": len(jr), "friction_measured": fm}
        if vals.size:
            item.update({
                "M_mean": float(vals.mean()),
                "M_std": float(vals.std(ddof=1)) if vals.size > 1 else 0.0,
                "M_cv": float(vals.std(ddof=1) / abs(vals.mean())) if vals.size > 1 and vals.mean() else 0.0,
                "M_values": vals.tolist(),
            })
        if fr.size:
            item.update({
                "friction_mean": float(fr.mean()),
                "friction_std": float(fr.std(ddof=1)) if fr.size > 1 else 0.0,
                "friction_signs": [int(np.sign(x)) for x in fr.tolist()],
                "friction_values": fr.tolist(),
            })
        out.append(item)
    return out


def write_csv(path: Path, rows: list[dict]) -> None:
    keys = sorted({k for row in rows for k in row})
    with path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--candidate", type=Path, default=DEFAULT_CANDIDATE)
    ap.add_argument("--posture", default="0.0387,0.8207,-0.6804,2.1650,1.3398,1.4196")
    ap.add_argument("--phase4-repeats", type=int, default=3)
    ap.add_argument("--phase5-repeats", type=int, default=3)
    ap.add_argument("--joints", default="1,2,3,4,5")
    ap.add_argument("--bidirectional", action="store_true")
    ap.add_argument("--settle-sec", type=float, default=8.0)
    ap.add_argument("--out-dir", type=Path, default=None)
    args = ap.parse_args()

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = args.out_dir or (DEFAULT_OUT / stamp)
    out_dir.mkdir(parents=True, exist_ok=True)
    phase4_rows = []
    phase5_all = []

    print(f"candidate: {args.candidate}")
    print(f"fixed posture: {args.posture}")

    for i in range(1, args.phase4_repeats + 1):
        if i > 1 and args.settle_sec > 0:
            time.sleep(args.settle_sec)
        print(f"\n===== PHASE4 repeat {i}/{args.phase4_repeats} =====", flush=True)
        cmd = [sys.executable, str(PHASE4), "--calibration-yaml", str(args.candidate),
               "--posture", args.posture]
        rc, report = run(cmd, out_dir / f"phase4_repeat_{i}.log")
        row = phase4_row(report, i)
        row["returncode"] = rc
        phase4_rows.append(row)

    for i in range(1, args.phase5_repeats + 1):
        if args.settle_sec > 0:
            time.sleep(args.settle_sec)
        print(f"\n===== PHASE5 repeat {i}/{args.phase5_repeats} =====", flush=True)
        cmd = [sys.executable, str(PHASE5), "--calibration-yaml", str(args.candidate),
               "--posture", args.posture, "--joints", args.joints]
        if args.bidirectional:
            cmd.append("--bidirectional")
        rc, report = run(cmd, out_dir / f"phase5_repeat_{i}.log")
        rows = phase5_rows(report, i)
        for row in rows:
            row["returncode"] = rc
        phase5_all.extend(rows)

    phase5_summary = summarize_phase5(phase5_all)
    summary = {
        "candidate": str(args.candidate),
        "posture": args.posture,
        "phase4": phase4_rows,
        "phase5": phase5_all,
        "phase5_summary": phase5_summary,
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    write_csv(out_dir / "phase4_summary.csv", phase4_rows)
    write_csv(out_dir / "phase5_runs.csv", phase5_all)
    write_csv(out_dir / "phase5_summary.csv", phase5_summary)

    print("\n===== PHASE4 SUMMARY =====")
    for row in phase4_rows:
        compact = {k: row.get(k) for k in ("repeat", "L0_deg", "L1_deg", "L2_deg", "L3_deg", "L4_deg", "failure")}
        print(json.dumps(compact, ensure_ascii=False))
    print("\n===== PHASE5 SUMMARY =====")
    for row in phase5_summary:
        print(json.dumps(row, ensure_ascii=False))
    print(f"summary: {out_dir / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
