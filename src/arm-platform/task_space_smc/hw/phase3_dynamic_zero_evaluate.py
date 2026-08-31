"""离线评估动态零位标定结果。

读取 phase3_dynamic_zero_*.json 和对应 processed CSV，比较不同 zero offset
候选在同一段动态数据上的 Pinocchio RNEA 残差 RMS。只读数据，只写评估 JSON。
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from task_space_smc.hw.fk_gravity_verify import GravityModel, N_ARM  # noqa: E402
from task_space_smc.hw.phase3_dynamic_zero_calibration import (  # noqa: E402
    ACTIVE, rnea_tau,
)


STATIC_RUN1 = np.array([
    0.03897077608327311,
    0.08921582544793427,
    0.0017947995480055174,
    0.05389154486890846,
    -0.06592826105455603,
    0.0,
])
STATIC_RUN2 = np.array([
    0.030832876069075605,
    0.0887499133939055,
    0.0231952129303072,
    0.10281641162071224,
    -0.06506409890097394,
    0.0,
])


def load_processed(path: Path) -> dict:
    rows = list(csv.DictReader(path.open()))
    q = np.array([[float(r[f"q_J{j}"]) for j in range(1, N_ARM + 1)]
                  for r in rows])
    qd = np.array([[float(r[f"qdot_J{j}"]) for j in range(1, N_ARM + 1)]
                   for r in rows])
    qdd = np.array([[float(r[f"qddot_J{j}"]) for j in range(1, N_ARM + 1)]
                    for r in rows])
    tau = np.array([[float(r[f"tau_meas_J{j}"]) for j in range(1, N_ARM + 1)]
                    for r in rows])
    segment = np.array([r["segment"] for r in rows])
    mask = np.ones(len(rows), dtype=bool)
    if len(rows) > 100:
        mask[:50] = False
        mask[-50:] = False
    return dict(q=q, qd=qd, qdd=qdd, tau=tau, segment=segment, mask=mask)


def rms_for(model: GravityModel, data: dict,
            q0: np.ndarray, segment: str | None) -> dict:
    mask = data["mask"].copy()
    if segment is not None:
        mask &= data["segment"] == segment
    raw = []
    for qi, vi, ai, yi in zip(
            data["q"][mask], data["qd"][mask],
            data["qdd"][mask], data["tau"][mask]):
        raw.append(rnea_tau(model, qi + q0, vi, ai)[ACTIVE] - yi[ACTIVE])
    raw = np.asarray(raw)
    return dict(
        n_samples=int(raw.shape[0]),
        residual_all_rms_nm=float(np.sqrt(np.mean(raw ** 2))),
        residual_rms_nm=np.sqrt(np.mean(raw ** 2, axis=0)).tolist(),
        residual_max_abs_nm=np.max(np.abs(raw), axis=0).tolist(),
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--json",
        default="src/arm-platform/task_space_smc/hw/data/phase3/"
        "phase3_dynamic_zero_20260812_163431.json")
    ap.add_argument(
        "--csv",
        default="src/arm-platform/task_space_smc/hw/data/phase3/"
        "phase3_dynamic_zero_processed_20260812_163431.csv")
    args = ap.parse_args()

    json_path = Path(args.json)
    csv_path = Path(args.csv)
    run = json.loads(json_path.read_text())
    data = load_processed(csv_path)
    model = GravityModel()
    candidates = {
        "old_yaml": np.asarray(run["existing_zero_offset_rad"], dtype=float),
        "static_run1": STATIC_RUN1,
        "static_run2": STATIC_RUN2,
    }
    for fit in run["fits"]:
        candidates[f"dynamic_{fit['segment']}"] = np.asarray(
            fit["zero_offset_rad"], dtype=float)

    out = dict(
        source_json=str(json_path),
        source_csv=str(csv_path),
        candidates={},
        writes_yaml=False,
    )
    for name, q0 in candidates.items():
        out["candidates"][name] = dict(
            zero_offset_rad=q0.tolist(),
            zero_offset_deg=np.degrees(q0).tolist(),
            all=rms_for(model, data, q0, None),
            forward=rms_for(model, data, q0, "forward"),
            reverse=rms_for(model, data, q0, "reverse"),
        )
    eval_path = json_path.with_name(json_path.stem + "_evaluation.json")
    eval_path.write_text(json.dumps(out, indent=2, ensure_ascii=False))
    print(json.dumps(dict(result=str(eval_path), candidates=out["candidates"]),
                     indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
