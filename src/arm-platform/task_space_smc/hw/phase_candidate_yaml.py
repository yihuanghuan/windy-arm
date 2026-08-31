"""Build/update arm6 calibration candidate YAML without touching the official file."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
import sys

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from task_space_smc.core.model import ARM6_JOINT_NAMES, arm6_calibration_path


HERE = Path(__file__).resolve()
DEFAULT_CANDIDATE = HERE.parents[2] / "config" / "arm6_calibration_candidate.yaml"
N = len(ARM6_JOINT_NAMES)


def latest(pattern: str) -> Path:
    paths = sorted(HERE.parent.glob(pattern))
    if not paths:
        raise FileNotFoundError(pattern)
    return paths[-1]


def load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text())


def dump_yaml(path: Path, data: dict) -> None:
    path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))


def ensure_vec(data: dict, key: str, default) -> list:
    vals = data.get(key)
    if not isinstance(vals, list) or len(vals) != N:
        vals = [default for _ in range(N)]
        data[key] = vals
    return vals


def ensure_candidate(path: Path, official: Path) -> dict:
    if path.exists():
        return load_yaml(path)
    data = load_yaml(official)
    meta = data.setdefault("metadata", {})
    meta["candidate_created_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    meta["candidate_source"] = str(official)
    dump_yaml(path, data)
    return data


def merge_phase2(data: dict, result_path: Path) -> dict:
    result = json.loads(result_path.read_text())
    stiction = ensure_vec(data, "stiction_nm", 0.0)
    deadband = ensure_vec(data, "deadband_rad", 0.0)
    accepted = []
    rejected = []
    hold_p = [30.0, 30.0, 30.0, 5.0, 5.0, 5.0]
    for row in result.get("results", []):
        j = int(row["joint"]) - 1
        if row.get("ok") and row.get("stiction_nm") is not None:
            stiction[j] = float(row["stiction_nm"])
            deadband[j] = float(row.get("deadband_rad", stiction[j] / hold_p[j]))
            accepted.append(j + 1)
        else:
            rejected.append(dict(joint=j + 1, reason=row.get("reason") or "not ok"))
    data.setdefault("metadata", {})["candidate_phase2"] = dict(
        source=str(result_path), accepted_joints=accepted, rejected=rejected)
    return data


def merge_phase3(data: dict, result_path: Path) -> dict:
    result = json.loads(result_path.read_text())
    zero = ensure_vec(data, "zero_offset_rad", 0.0)
    zero_deg = ensure_vec(data, "zero_offset_deg", 0.0)
    ratio = ensure_vec(data, "first_moment_ratio", 1.0)
    converged = ensure_vec(data, "zero_offset_converged", False)
    accepted = []
    rejected = []
    for row in result.get("results", []):
        j = int(row["joint"]) - 1
        if row.get("accepted"):
            zero[j] = float(row["zero_offset_rad"])
            zero_deg[j] = float(row["zero_offset_deg"])
            ratio[j] = float(row["ratio"])
            converged[j] = bool(row.get("converged"))
            accepted.append(j + 1)
        else:
            reasons = row.get("quality", {}).get("reasons")
            rejected.append(dict(
                joint=j + 1,
                reason=row.get("reason") or "; ".join(reasons or ["not accepted"])))
    data.setdefault("metadata", {})["candidate_phase3"] = dict(
        source=str(result_path), accepted_joints=accepted, rejected=rejected)
    return data


def merge_phase5(data: dict, result_path: Path) -> dict:
    result = json.loads(result_path.read_text())
    inertia = ensure_vec(data, "inertia_kgm2", None)
    stderr = ensure_vec(data, "inertia_stderr_kgm2", None)
    kinetic = ensure_vec(data, "kinetic_friction_nm", None)
    accepted = []
    rejected = []
    for row in result.get("results", []):
        j = int(row["joint"]) - 1
        ok = bool(row.get("ok"))
        f_imp = row.get("friction_implied")
        f_meas = row.get("friction_measured")
        credible = False
        if ok and f_imp is not None and f_meas is not None:
            tol = max(0.10, 0.5 * abs(float(f_meas)))
            credible = abs(float(f_imp) - float(f_meas)) <= tol
        if credible:
            inertia[j] = float(row["M"])
            stderr[j] = float(row["M_stderr"])
            kinetic[j] = float(f_imp)
            accepted.append(j + 1)
        else:
            rejected.append(dict(
                joint=j + 1,
                reason=f"friction implied {f_imp} not close to measured {f_meas}"))
    data.setdefault("metadata", {})["candidate_phase5"] = dict(
        source=str(result_path), accepted_joints=accepted, rejected=rejected)
    return data


def print_summary(data: dict, path: Path) -> None:
    print(f"candidate: {path}")
    print("zero_offset_deg:", [round(float(v), 3) for v in data["zero_offset_deg"]])
    print("first_moment_ratio:", [round(float(v), 4) for v in data["first_moment_ratio"]])
    print("stiction_nm:", [round(float(v), 4) for v in data["stiction_nm"]])
    print("inertia_kgm2:", data.get("inertia_kgm2"))
    meta = data.get("metadata", {})
    for key in ("candidate_phase2", "candidate_phase3", "candidate_phase5"):
        if key in meta:
            print(f"{key}: {meta[key]}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--candidate", type=Path, default=DEFAULT_CANDIDATE)
    ap.add_argument("--official", type=Path, default=Path(arm6_calibration_path()))
    ap.add_argument("--phase2", type=Path, default=None)
    ap.add_argument("--phase3", type=Path, default=None)
    ap.add_argument("--phase5", type=Path, default=None)
    ap.add_argument("--init", action="store_true")
    args = ap.parse_args()

    data = ensure_candidate(args.candidate, args.official)
    if args.phase2:
        data = merge_phase2(data, args.phase2)
    if args.phase3:
        data = merge_phase3(data, args.phase3)
    if args.phase5:
        data = merge_phase5(data, args.phase5)
    data.setdefault("metadata", {})["candidate_updated_at"] = time.strftime(
        "%Y-%m-%d %H:%M:%S")
    dump_yaml(args.candidate, data)
    print_summary(data, args.candidate)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
