#!/usr/bin/env python3
"""Run three independent J1/J2/J3/J5 calibration rounds from one baseline.

This is orchestration only: it calls the existing Phase 3 and Phase 5 tools,
keeps all logs, aggregates zero/ratio and F+/F-, writes a final candidate YAML,
and then runs Phase 4 plus a dynamic acceptance pass.  It never writes the
official arm6_calibration.yaml.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml


HERE = Path(__file__).resolve()
HW = HERE.parent
CONFIG = HERE.parents[2] / "config"
DEFAULT_BASELINE = CONFIG / "arm6_calibration_candidate.yaml"
OUT = HW / "data" / "phase_j1235_full_repeat"
J1235 = [1, 2, 3, 5]
PHASE4_POSTURE = "0.0387,0.8207,-0.6804,2.1650,1.3398,1.4196"
PHASE5_POSTURES = (
    "0.0387,0.8207,-0.6804,2.1650,1.3398,1.4196;"
    "0.0387,0.7307,-0.6104,2.0350,1.3398,1.4196;"
    "0.0387,0.9107,-0.7504,2.2950,1.3398,1.4196"
)
PHASE3_TRANSIT_TIMEOUT = "240.0"
PHASE5_MOVE_TIMEOUT = "120.0"


ZERO_SPAN_LIMIT_DEG = {1: 0.6, 2: 0.8, 3: 0.6, 5: 1.5}
RATIO_CV_LIMIT = 0.08
FRICTION_CV_LIMIT = 0.35


def stats(vals: list[float]) -> dict:
    arr = np.asarray([v for v in vals if v is not None and np.isfinite(v)],
                     dtype=float)
    if arr.size == 0:
        return dict(n=0, mean=None, span=None, cv=None, values=[])
    mean = float(np.mean(arr))
    span = float(np.max(arr) - np.min(arr))
    cv = float(np.std(arr, ddof=1) / abs(mean)) if arr.size > 1 and abs(mean) > 1e-9 else 0.0
    return dict(n=int(arr.size), mean=mean, span=span, cv=cv,
                values=arr.tolist())


def run_logged(cmd: list[str], log_path: Path, env: dict) -> None:
    print(f"[RUN] {' '.join(cmd)}", flush=True)
    print(f"[LOG] {log_path}", flush=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w") as log:
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, env=env, cwd=str(HERE.parents[4]))
        assert proc.stdout is not None
        for line in proc.stdout:
            log.write(line)
            log.flush()
            if ("results ->" in line or "VERDICT" in line
                    or "J dir ok" in line or "candidate" in line
                    or "REFUSED" in line or "失败" in line):
                print(line.rstrip(), flush=True)
        rc = proc.wait()
    if rc != 0:
        tail = "".join(log_path.read_text(errors="replace").splitlines(True)[-80:])
        raise RuntimeError(f"command failed rc={rc}: {' '.join(cmd)}\n{tail}")


def read_current_j6(env: dict) -> float | None:
    try:
        proc = subprocess.run(
            ["ros2", "topic", "echo", "/joint_states", "--once"],
            cwd=str(HERE.parents[4]), env=env, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5.0,
            check=False)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if proc.returncode != 0:
        return None
    text = proc.stdout.split("---", 1)[0]
    try:
        doc = yaml.safe_load(text)
        pos = list(doc["position"])
        if len(pos) >= 6:
            return float(pos[5])
    except (yaml.YAMLError, TypeError, KeyError, ValueError):
        pass
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if line.strip() != "position:":
            continue
        vals = []
        for item in lines[i + 1:]:
            stripped = item.strip()
            if not stripped.startswith("- "):
                break
            try:
                vals.append(float(stripped[2:].strip()))
            except ValueError:
                break
        if len(vals) >= 6:
            return vals[5]
    return None


def replace_j6(vec_text: str, j6: float) -> str:
    vals = [float(x.strip()) for x in vec_text.split(",") if x.strip()]
    if len(vals) != 6:
        raise ValueError(f"expected 6 posture values, got {len(vals)}")
    vals[5] = float(j6)
    return ",".join(f"{v:.6f}" for v in vals)


def replace_j6_in_postures(postures: str, j6: float) -> str:
    return ";".join(
        replace_j6(chunk, j6)
        for chunk in postures.split(";")
        if chunk.strip())


def newest_after(pattern: str, before: set[Path]) -> Path:
    paths = set(HW.glob(pattern))
    new = sorted(paths - before, key=lambda p: p.stat().st_mtime)
    if not new:
        raise FileNotFoundError(f"no new result for {pattern}")
    return new[-1]


def apply_phase3_to_candidate(baseline: Path, phase3_json: Path,
                              out_yaml: Path) -> dict:
    data = yaml.safe_load(baseline.read_text())
    result = json.loads(phase3_json.read_text())
    accepted, rejected = [], []
    for row in result.get("results", []):
        joint = int(row.get("joint"))
        idx = joint - 1
        if joint not in J1235:
            continue
        if row.get("accepted"):
            data["zero_offset_rad"][idx] = float(row["zero_offset_rad"])
            data["zero_offset_deg"][idx] = float(row["zero_offset_deg"])
            data["first_moment_ratio"][idx] = float(row["ratio"])
            data["zero_offset_converged"][idx] = bool(row.get("converged"))
            accepted.append(joint)
        else:
            rejected.append(dict(joint=joint, reason=row.get("reason")
                                 or "; ".join(row.get("quality", {}).get("reasons", []))))
    data.setdefault("metadata", {})["round_phase3"] = dict(
        source=str(phase3_json), accepted_joints=accepted, rejected=rejected)
    out_yaml.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
    return dict(accepted=accepted, rejected=rejected)


def phase3_hardware_failed(phase3_json: Path) -> str | None:
    result = json.loads(phase3_json.read_text())
    for row in result.get("results", []):
        if row.get("ok", False):
            continue
        reason = str(row.get("reason", "")).lower()
        if any(token in reason for token in (
                "transit failed", "goto timed out", "desk", "speed",
                "dwell got no samples")):
            return f"joint{row.get('joint')}: {row.get('reason')}"
    return None


def aggregate(rounds: list[dict], baseline: Path, out_yaml: Path,
              summary_path: Path) -> dict:
    base = yaml.safe_load(baseline.read_text())
    final = yaml.safe_load(baseline.read_text())
    if not isinstance(final.get("kinetic_friction_pos_nm"), list):
        final["kinetic_friction_pos_nm"] = list(final.get("kinetic_friction_nm", [None] * 6))
    if not isinstance(final.get("kinetic_friction_neg_nm"), list):
        final["kinetic_friction_neg_nm"] = list(final.get("kinetic_friction_nm", [None] * 6))

    summary = dict(baseline=str(baseline), rounds=rounds, joints={})
    for joint in J1235:
        idx = joint - 1
        z, ratio, fpos, fneg = [], [], [], []
        phase3_passes, phase5_passes = [], []
        for r in rounds:
            p3 = json.loads(Path(r["phase3_json"]).read_text())
            row = next((x for x in p3.get("results", [])
                        if int(x.get("joint")) == joint), None)
            if row and row.get("accepted"):
                z.append(float(row["zero_offset_deg"]))
                ratio.append(float(row["ratio"]))
                phase3_passes.append(r["round"])
            p5 = json.loads(Path(r["phase5_json"]).read_text())
            item = (p5.get("summary") or {}).get(str(joint))
            if item:
                dpos = item["directions"]["pos"]
                dneg = item["directions"]["neg"]
                if dpos.get("ok") and dneg.get("ok"):
                    fpos.append(float(dpos["friction_nm"]))
                    fneg.append(float(dneg["friction_nm"]))
                    phase5_passes.append(r["round"])
        zst = stats(z)
        rst = stats(ratio)
        fpst = stats(fpos)
        fnst = stats(fneg)
        zero_pass = (
            zst["n"] >= 2
            and zst["span"] is not None
            and zst["span"] <= ZERO_SPAN_LIMIT_DEG[joint]
            and rst["n"] >= 2
            and rst["cv"] is not None
            and rst["cv"] <= RATIO_CV_LIMIT
        )
        friction_pass = (
            fpst["n"] >= 2 and fnst["n"] >= 2
            and fpst["mean"] is not None and fnst["mean"] is not None
            and fpst["mean"] >= 0.0 and fnst["mean"] >= 0.0
            and fpst["cv"] <= FRICTION_CV_LIMIT
            and fnst["cv"] <= FRICTION_CV_LIMIT
        )
        joint_pass = bool(zero_pass and friction_pass)
        summary["joints"][str(joint)] = dict(
            joint=joint,
            zero_offset_deg=zst,
            ratio=rst,
            friction_pos_nm=fpst,
            friction_neg_nm=fnst,
            phase3_pass_rounds=phase3_passes,
            phase5_pass_rounds=phase5_passes,
            zero_ratio_pass=zero_pass,
            friction_pass=friction_pass,
            PASS=joint_pass,
            baseline=dict(
                zero_offset_deg=base["zero_offset_deg"][idx],
                ratio=base["first_moment_ratio"][idx],
                friction_pos=(base.get("kinetic_friction_pos_nm") or [None] * 6)[idx],
                friction_neg=(base.get("kinetic_friction_neg_nm") or [None] * 6)[idx],
            ),
        )
        if zero_pass:
            final["zero_offset_deg"][idx] = zst["mean"]
            final["zero_offset_rad"][idx] = float(np.radians(zst["mean"]))
            final["first_moment_ratio"][idx] = rst["mean"]
            final["zero_offset_converged"][idx] = True
        if friction_pass:
            final["kinetic_friction_pos_nm"][idx] = fpst["mean"]
            final["kinetic_friction_neg_nm"][idx] = fnst["mean"]
            final["kinetic_friction_nm"][idx] = float(0.5 * (fpst["mean"] + fnst["mean"]))

    meta = final.setdefault("metadata", {})
    meta["final_candidate_created_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    meta["final_candidate_source"] = str(baseline)
    meta["final_candidate_summary"] = str(summary_path)
    out_yaml.write_text(yaml.safe_dump(final, allow_unicode=True, sort_keys=False))
    summary["final_candidate"] = str(out_yaml)
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    return summary


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE)
    ap.add_argument("--rounds", type=int, default=3)
    ap.add_argument("--j6-hold-rad", type=float, default=None,
                    help="override Phase4/5 posture J6; use current J6 when omitted")
    ap.add_argument("--skip-validation", action="store_true")
    args = ap.parse_args()

    stamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = OUT / f"j1235_full_{stamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    baseline = run_dir / "baseline.yaml"
    shutil.copy2(args.baseline, baseline)
    env = os.environ.copy()
    env.setdefault("ROS_DOMAIN_ID", "77")
    env["PYTHONUNBUFFERED"] = "1"
    env["ARM6_CALIBRATION_YAML"] = str(baseline)
    j6_hold = args.j6_hold_rad if args.j6_hold_rad is not None else read_current_j6(env)
    phase4_posture = PHASE4_POSTURE
    phase5_postures = PHASE5_POSTURES
    if j6_hold is not None:
        phase4_posture = replace_j6(PHASE4_POSTURE, j6_hold)
        phase5_postures = replace_j6_in_postures(PHASE5_POSTURES, j6_hold)
        print(f"[RUN] holding current J6 in Phase4/5 postures: {j6_hold:.6f} rad",
              flush=True)
    else:
        print("[WARN] could not read current J6; using built-in Phase4/5 postures",
              flush=True)

    rounds = []
    for i in range(1, args.rounds + 1):
        print(f"\n===== FULL ROUND {i}/{args.rounds} =====", flush=True)
        before_p3 = set(HW.glob("data/phase3/phase3_calibration_*.json"))
        phase3_log = run_dir / f"round{i}_phase3.log"
        run_logged([
            sys.executable, str(HW / "phase3_gravity_calibration.py"),
            "--joints", "1,2,3,5",
            "--bidirectional-joints", "5",
            "--calibration-yaml", str(baseline),
            "--transit-timeout", PHASE3_TRANSIT_TIMEOUT,
        ], phase3_log, env)
        phase3_json = newest_after("data/phase3/phase3_calibration_*.json", before_p3)
        hw_failure = phase3_hardware_failed(phase3_json)
        if hw_failure:
            raise RuntimeError(
                f"Phase3 hardware/protocol failure; not continuing to Phase5: "
                f"{hw_failure}")
        round_yaml = run_dir / f"round{i}_candidate_after_phase3.yaml"
        merge_info = apply_phase3_to_candidate(baseline, phase3_json, round_yaml)

        before_p5 = set(HW.glob("data/phase5_directional/phase5_directional_friction_*.json"))
        phase5_log = run_dir / f"round{i}_phase5.log"
        run_logged([
            sys.executable, str(HW / "phase5_directional_friction_dynamic.py"),
            "--joints", "1,2,3,5",
            "--scales", "2.0,2.5,3.0",
            "--calibration-yaml", str(round_yaml),
            "--postures", phase5_postures,
            "--move-timeout", PHASE5_MOVE_TIMEOUT,
        ], phase5_log, env)
        phase5_json = newest_after(
            "data/phase5_directional/phase5_directional_friction_*.json", before_p5)
        rounds.append(dict(round=i, phase3_json=str(phase3_json),
                           phase5_json=str(phase5_json),
                           round_candidate=str(round_yaml),
                           phase3_merge=merge_info,
                           phase3_log=str(phase3_log),
                           phase5_log=str(phase5_log)))

    final_yaml = run_dir / "arm6_calibration_final_candidate.yaml"
    summary_path = run_dir / "summary.json"
    summary = aggregate(rounds, baseline, final_yaml, summary_path)
    print(f"\nsummary -> {summary_path}")
    print(f"final_candidate -> {final_yaml}")
    print("\nJ zero_mean zero_span ratio_mean ratio_cv F+_mean F+_cv F-_mean F-_cv PASS")
    for joint in J1235:
        s = summary["joints"][str(joint)]
        print(
            f"J{joint} "
            f"{s['zero_offset_deg']['mean']} {s['zero_offset_deg']['span']} "
            f"{s['ratio']['mean']} {s['ratio']['cv']} "
            f"{s['friction_pos_nm']['mean']} {s['friction_pos_nm']['cv']} "
            f"{s['friction_neg_nm']['mean']} {s['friction_neg_nm']['cv']} "
            f"{s['PASS']}")

    if args.skip_validation:
        return 0

    before_p4 = set(HW.glob("data/phase4/phase4_gravity_comp_*.json"))
    phase4_log = run_dir / "final_phase4.log"
    run_logged([
        sys.executable, str(HW / "phase4_gravity_compensation.py"),
        "--calibration-yaml", str(final_yaml),
        "--posture", phase4_posture,
    ], phase4_log, env)
    phase4_json = newest_after("data/phase4/phase4_gravity_comp_*.json", before_p4)

    before_p5 = set(HW.glob("data/phase5_directional/phase5_directional_friction_*.json"))
    phase5_log = run_dir / "final_dynamic_acceptance.log"
    run_logged([
        sys.executable, str(HW / "phase5_directional_friction_dynamic.py"),
        "--joints", "1,2,3,5",
        "--scales", "2.0,2.5,3.0",
        "--calibration-yaml", str(final_yaml),
        "--postures", phase5_postures,
        "--move-timeout", PHASE5_MOVE_TIMEOUT,
    ], phase5_log, env)
    phase5_json = newest_after(
        "data/phase5_directional/phase5_directional_friction_*.json", before_p5)
    summary["validation"] = dict(
        phase4_json=str(phase4_json),
        phase4_log=str(phase4_log),
        dynamic_json=str(phase5_json),
        dynamic_log=str(phase5_log),
    )
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"\nvalidation phase4 -> {phase4_json}")
    print(f"validation dynamic -> {phase5_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
