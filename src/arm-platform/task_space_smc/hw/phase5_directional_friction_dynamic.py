#!/usr/bin/env python3
"""Phase 5 directional dynamic friction test for J2/J4.

This keeps the candidate zero offset and gravity ratio frozen.  It runs
positive/negative torque pulses at several amplitudes and safe postures,
accepts only pulses whose measured motion really went in the commanded
direction, and estimates direction-dependent Coulomb friction with the original
Phase 5 acceleration-vs-pulse intercept method.  RNEA residuals are kept only
as a dynamic validation signal.  It never writes the official calibration YAML.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from dummy_interface.msg import MotorControl
except ImportError as exc:
    rclpy = None
    _IMPORT_ERROR = exc

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from task_space_smc.core.model import load_arm6_calibration
from task_space_smc.hw.fk_gravity_verify import (
    GravityModel, N_ARM, JOINT_LOWER, JOINT_UPPER, RATED_TORQUE, check_setpoint,
)
from task_space_smc.hw.phase4_gravity_compensation import (
    BASE_D, BASE_P, desk_margin,
)


HERE = Path(__file__).resolve()
DEFAULT_CANDIDATE = HERE.parents[2] / "config" / "arm6_calibration_candidate.yaml"
OUT = HERE.parent / "data" / "phase5_directional"

DEFAULT_POSTURES = (
    "0.0387,0.8207,-0.6804,2.1650,1.3398,1.4196;"
    "0.0387,0.7307,-0.6104,2.0350,1.3398,1.4196;"
    "0.0387,0.9107,-0.7504,2.2950,1.3398,1.4196"
)

PUBLISH_RATE = 100.0
PULSE_SEC = 0.24
SETTLE_SEC = 1.5
RECOVER_SEC = 2.7
DESK_MARGIN_MIN = 0.10
DISP_ABORT = 0.13
SPEED_ABORT = 2.0
MOVE_TIMEOUT = 45.0


def parse_vec(text: str) -> np.ndarray:
    vals = [float(x.strip()) for x in text.split(",") if x.strip()]
    if len(vals) != N_ARM:
        raise ValueError(f"expected {N_ARM} comma-separated values")
    return np.asarray(vals, dtype=float)


def parse_postures(text: str) -> list[np.ndarray]:
    return [parse_vec(chunk) for chunk in text.split(";") if chunk.strip()]


def parse_scales(text: str) -> list[float]:
    return [float(x.strip()) for x in text.split(",") if x.strip()]


def robust_stats(vals: list[float]) -> dict:
    arr = np.asarray([v for v in vals if np.isfinite(v)], dtype=float)
    if arr.size == 0:
        return dict(n=0, median=None, mean=None, std=None, min=None, max=None)
    q1, q3 = np.percentile(arr, [25.0, 75.0])
    iqr = max(float(q3 - q1), 1e-12)
    keep = arr[(arr >= q1 - 1.5 * iqr) & (arr <= q3 + 1.5 * iqr)]
    if keep.size == 0:
        keep = arr
    return dict(
        n=int(keep.size),
        median=float(np.median(keep)),
        mean=float(np.mean(keep)),
        std=float(np.std(keep, ddof=1)) if keep.size > 1 else 0.0,
        min=float(np.min(keep)),
        max=float(np.max(keep)),
    )


def dynamic_fit(records: list[dict], sign: int) -> dict:
    recs = [r for r in records if r.get("accepted") and int(r["direction"]) == sign]
    if len(recs) < 3:
        return dict(ok=False, reason=f"only {len(recs)} direction-correct pulses",
                    n=len(recs), records=len(recs))
    tau = np.asarray([r["tau_extra"] for r in recs], dtype=float)
    qdd = np.asarray([r["qdd_fit"] for r in recs], dtype=float)
    A = np.column_stack([tau, np.ones_like(tau)])
    coef, *_ = np.linalg.lstsq(A, qdd, rcond=None)
    slope, intercept = coef
    resid = qdd - A @ coef
    ok = bool(slope > 0.0)
    friction = float(-sign * intercept / slope) if ok else None
    return dict(
        ok=ok,
        n=len(recs),
        slope=float(slope),
        intercept=float(intercept),
        inertia_kgm2=float(1.0 / slope) if ok else None,
        friction_nm=friction,
        qdd_fit_rmse=float(np.sqrt(np.mean(resid ** 2))),
        tau_extra_nm=tau.tolist(),
        qdd_rad_s2=qdd.tolist(),
        postures=sorted({int(r["posture_index"]) for r in recs}),
    )


class DirectionalFrictionNode(Node):
    def __init__(self, calibration_yaml: str):
        super().__init__("phase5_directional_friction_dynamic")
        self.model = GravityModel()
        self.cal = load_arm6_calibration(calibration_yaml)
        self.offset = np.asarray(self.cal["zero_offset_rad"], dtype=float)
        self.ratio = np.asarray(self.cal["first_moment_ratio"], dtype=float)
        self.stiction = np.asarray(self.cal["stiction_nm"], dtype=float)
        armature = self.cal.get("armature_kgm2")
        if armature is not None:
            self.model.model.armature[:] = np.asarray(
                [0.0 if x is None else float(x) for x in armature], dtype=float)
            self.model.data = self.model.model.createData()
        self.q = None
        self.v = None
        self.tau = None
        self.pub = self.create_publisher(
            MotorControl, "/student/joint_effort_command", 10)
        self.create_subscription(JointState, "/joint_states", self._cb, 50)
        self.get_logger().info(f"calibration YAML: {self.cal['source']}")
        self.get_logger().info(
            f"zero_offset_deg: {np.round(np.degrees(self.offset), 3).tolist()}")
        self.get_logger().info(
            f"first_moment_ratio: {np.round(self.ratio, 4).tolist()}")
        self.get_logger().info(
            f"stiction_nm: {np.round(self.stiction, 4).tolist()}")
        self.get_logger().info(
            f"armature_kgm2: {self.model.model.armature.tolist()}")

    def _cb(self, msg):
        if len(msg.position) == N_ARM:
            self.q = np.asarray(msg.position, dtype=float)
        if len(msg.velocity) == N_ARM:
            self.v = np.asarray(msg.velocity, dtype=float)
        if len(msg.effort) == N_ARM:
            self.tau = np.asarray(msg.effort, dtype=float)

    def gravity_scaled(self, q_encoder: np.ndarray) -> np.ndarray:
        return self.model.gravity(q_encoder + self.offset) * self.ratio

    def rnea_scaled(self, q_encoder: np.ndarray, qd: np.ndarray, qdd: np.ndarray) -> np.ndarray:
        q_model = q_encoder + self.offset
        tau = pin.rnea(self.model.model, self.model.data, q_model, qd, qdd).copy()
        g = self.model.gravity(q_model)
        return (tau - g) + self.ratio * g

    def send(self, q_cmd, kp=None, kd=None, tau_ff=None):
        msg = MotorControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [float(x) for x in q_cmd]
        msg.velocity = [0.0] * N_ARM
        msg.current = [float(x) for x in (
            self.gravity_scaled(self.q if self.q is not None else q_cmd)
            if tau_ff is None else tau_ff)]
        msg.p = [float(x) for x in (BASE_P if kp is None else kp)]
        msg.d = [float(x) for x in (BASE_D if kd is None else kd)]
        msg.enable = 1
        self.pub.publish(msg)

    def wait_state(self, timeout=10.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout and (
                self.q is None or self.v is None or self.tau is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.q is not None and self.v is not None and self.tau is not None

    def check_effort_enabled(self, timeout=5.0) -> str | None:
        from rcl_interfaces.srv import GetParameters
        cli = self.create_client(GetParameters, "/student_arm_node/get_parameters")
        if not cli.wait_for_service(timeout_sec=timeout):
            return "student_arm_node parameter service unavailable"
        req = GetParameters.Request()
        req.names = ["enable_direct_effort_command"]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.result() is None or not fut.result().values:
            return "could not read enable_direct_effort_command"
        if not fut.result().values[0].bool_value:
            return "student_arm_node is not in direct effort mode"
        return None

    def safety_reason(self, q) -> str | None:
        q = np.asarray(q, dtype=float)
        rep = check_setpoint(q, self.model)
        if not rep.ok:
            return "; ".join(rep.reasons)
        mg = desk_margin(self.model, q)
        if mg < DESK_MARGIN_MIN:
            return f"desk/fence margin {mg:+.4f}"
        lim = np.minimum(q - JOINT_LOWER, JOINT_UPPER - q)
        if lim.min() < 0.08:
            return f"joint{int(np.argmin(lim)) + 1} within 0.08 rad of a limit"
        return None

    def hard_safety_reason(self, q) -> str | None:
        mg = desk_margin(self.model, q)
        if mg < 0.0:
            return f"desk/fence margin {mg:+.4f}"
        lim = np.minimum(q - JOINT_LOWER, JOINT_UPPER - q)
        if lim.min() < 0.05:
            return f"joint{int(np.argmin(lim)) + 1} within 0.05 rad of a limit"
        return None

    def move_to(self, target: np.ndarray, timeout=None) -> str | None:
        if timeout is None:
            timeout = MOVE_TIMEOUT
        why = self.safety_reason(target)
        if why:
            return f"target unsafe: {why}"
        if self.q is None:
            return "no current state"
        start = self.q.copy()
        dist = float(np.max(np.abs(target - start)))
        ramp_sec = max(4.0, dist / 0.16)
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < timeout:
            now = time.time()
            alpha = min(1.0, (now - t0) / ramp_sec)
            q_cmd = start + alpha * (target - start)
            if now >= nxt:
                self.send(q_cmd)
                nxt = now + 1.0 / PUBLISH_RATE
            rclpy.spin_once(self, timeout_sec=0.002)
            if self.q is None:
                continue
            why = self.hard_safety_reason(self.q)
            if why:
                return why
            if np.max(np.abs(self.q - target)) < 0.055:
                return None
        return f"move_to timed out {np.max(np.abs(target - self.q)):.3f} rad away"

    def hold(self, q_hold, seconds=1.0):
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < seconds:
            now = time.time()
            if now >= nxt:
                self.send(q_hold)
                nxt = now + 1.0 / PUBLISH_RATE
            rclpy.spin_once(self, timeout_sec=0.002)

    def pulse(self, q_hold: np.ndarray, joint: int, direction: int,
              tau_extra: float, posture_index: int, scale: float) -> dict:
        idx = joint - 1
        why = self.move_to(q_hold)
        if why:
            return dict(ok=False, accepted=False, joint=joint, direction=direction,
                        scale=scale, tau_extra=tau_extra,
                        posture_index=posture_index,
                        reason=f"move_to posture failed: {why}")
        self.hold(q_hold, SETTLE_SEC)
        q0 = self.q.copy()
        kp = list(BASE_P)
        kd = list(BASE_D)
        kp[idx] = 0.0
        kd[idx] *= 0.20
        rows = []
        t0 = time.time()
        nxt = t0
        abort = None
        while time.time() - t0 < PULSE_SEC:
            now = time.time()
            if now >= nxt:
                q_now = self.q if self.q is not None else q_hold
                tff = self.gravity_scaled(q_now)
                tff[idx] += tau_extra
                if np.any(np.abs(tff) > 0.92 * RATED_TORQUE):
                    abort = "command torque near rated limit"
                    break
                self.send(q_hold, kp=kp, kd=kd, tau_ff=tff)
                nxt = now + 1.0 / PUBLISH_RATE
            rclpy.spin_once(self, timeout_sec=0.001)
            if self.q is None or self.v is None or self.tau is None:
                continue
            why = self.hard_safety_reason(self.q)
            if why:
                abort = why
                break
            disp = float(self.q[idx] - q0[idx])
            speed = float(abs(self.v[idx]))
            if abs(disp) > DISP_ABORT:
                abort = f"displacement {disp:+.4f} rad"
                break
            if speed > SPEED_ABORT:
                abort = f"speed {speed:.2f} rad/s"
                break
            rows.append(dict(t=float(now - t0),
                             q=self.q.tolist(),
                             qd=self.v.tolist(),
                             tau=self.tau.tolist()))
        self.hold(q_hold, RECOVER_SEC)
        out = dict(ok=True, accepted=False, joint=joint, direction=direction,
                   scale=scale, tau_extra=tau_extra, posture_index=posture_index,
                   posture=q_hold.tolist(), rows=rows, aborted=abort)
        return analyze_pulse(self, out)


def analyze_pulse(node: DirectionalFrictionNode, run: dict) -> dict:
    rows = run.get("rows") or []
    joint = int(run["joint"])
    idx = joint - 1
    sign = int(run["direction"])
    if len(rows) < 8:
        run.update(reason=f"too few rows: {len(rows)}")
        return run
    t = np.asarray([r["t"] for r in rows], dtype=float)
    q = np.asarray([r["q"] for r in rows], dtype=float)
    qd = np.asarray([r["qd"] for r in rows], dtype=float)
    tau = np.asarray([r["tau"] for r in rows], dtype=float)
    if np.any(np.diff(t) <= 0.0):
        run.update(reason="nonmonotonic sample time")
        return run
    qdd = np.gradient(qd, t, axis=0, edge_order=1)
    A = np.column_stack([t, np.ones_like(t)])
    coef, *_ = np.linalg.lstsq(A, qd[:, idx], rcond=None)
    disp = float(q[-1, idx] - q[0, idx])
    v_peak_signed = float(qd[np.argmax(np.abs(qd[:, idx])), idx])
    qdd_fit = float(coef[0])
    v_mean = float(np.mean(qd[:, idx]))
    # Joint 2 must visibly clear backlash; J4 has a much smaller torque scale.
    disp_min = 0.012 if joint == 2 else 0.006
    v_min = 0.10 if joint == 2 else 0.06
    qdd_min = 0.8 if joint == 2 else 0.5
    direction_ok = (
        sign * disp > disp_min
        and sign * v_peak_signed > v_min
        and sign * qdd_fit > qdd_min
    )
    sample_mask = sign * qd[:, idx] > (0.04 if joint == 2 else 0.025)
    sample_mask &= sign * qdd[:, idx] > 0.0
    friction_samples = []
    residual_samples = []
    for qi, vi, ai, yi in zip(q[sample_mask], qd[sample_mask],
                              qdd[sample_mask], tau[sample_mask]):
        pred = node.rnea_scaled(qi, vi, ai)
        resid = yi - pred
        residual_samples.append(float(resid[idx]))
        friction_samples.append(float(sign * resid[idx]))
    fr = robust_stats(friction_samples)
    run.update(
        n_rows=len(rows),
        displacement=disp,
        v_peak_signed=v_peak_signed,
        v_mean=v_mean,
        qdd_fit=qdd_fit,
        n_direction_samples=int(np.sum(sample_mask)),
        residual_joint_nm=residual_samples,
        friction_samples_nm=friction_samples,
        friction_estimate_nm=fr,
    )
    if not direction_ok:
        run.update(reason=(
            f"direction gate failed: disp={disp:+.4f}, "
            f"v_peak={v_peak_signed:+.3f}, qdd={qdd_fit:+.3f}"))
        return run
    if fr["n"] < 5 or fr["median"] is None:
        run.update(reason=f"too few direction-correct residual samples: {fr['n']}")
        return run
    run["accepted"] = True
    return run


def summarize(records: list[dict]) -> dict:
    summary = {}
    for joint in sorted({int(r["joint"]) for r in records}):
        jr = [r for r in records if int(r["joint"]) == joint]
        item = {"joint": joint, "directions": {}, "ok": True, "reason": None}
        for sign, key in ((1, "pos"), (-1, "neg")):
            dr = [r for r in jr if int(r["direction"]) == sign and r.get("accepted")]
            fit = dynamic_fit(jr, sign)
            postures = sorted({int(r["posture_index"]) for r in dr})
            by_posture = {}
            for p in postures:
                pr = [r for r in jr if r.get("accepted")
                      and int(r["direction"]) == sign
                      and int(r["posture_index"]) == p]
                by_posture[str(p)] = dynamic_fit(pr, sign)
            posture_f = [
                v["friction_nm"] for v in by_posture.values()
                if v.get("ok") and v.get("friction_nm") is not None
            ]
            ok = (fit.get("ok") and fit.get("n", 0) >= 6
                  and len(postures) >= 2
                  and fit.get("friction_nm") is not None
                  and fit["friction_nm"] >= 0.0)
            if ok:
                if posture_f:
                    pf = np.asarray(posture_f, dtype=float)
                    spread = float(np.max(np.abs(pf - fit["friction_nm"])))
                    ok = (float(np.min(pf)) >= -0.08
                          and spread <= max(0.35, 2.0 * abs(fit["friction_nm"]) + 0.05))
            item["directions"][key] = dict(
                ok=bool(ok),
                n_accepted=len(dr),
                postures=postures,
                dynamic_fit=fit,
                friction_nm=fit.get("friction_nm"),
                by_posture=by_posture,
            )
            if not ok:
                item["ok"] = False
                item["reason"] = f"{key} direction failed quality gate"
        if item["ok"]:
            pos = item["directions"]["pos"]["friction_nm"]
            neg = item["directions"]["neg"]["friction_nm"]
            item["kinetic_friction_avg_nm"] = float(0.5 * (pos + neg))
        summary[str(joint)] = item
    return summary


def validation(records: list[dict], summary: dict) -> dict:
    out = {}
    for joint_s, item in summary.items():
        joint = int(joint_s)
        idx = joint - 1
        vals_before = []
        vals_after = []
        for r in records:
            if int(r["joint"]) != joint or not r.get("accepted"):
                continue
            sign = int(r["direction"])
            key = "pos" if sign > 0 else "neg"
            friction = item["directions"][key]["friction_nm"]
            if friction is None:
                continue
            for resid in r.get("residual_joint_nm") or []:
                vals_before.append(float(resid))
                vals_after.append(float(resid) - sign * float(friction))
        arr0 = np.asarray(vals_before, dtype=float)
        arr1 = np.asarray(vals_after, dtype=float)
        if arr0.size == 0:
            out[joint_s] = dict(ok=False, reason="no accepted residual samples")
            continue
        out[joint_s] = dict(
            ok=True,
            n=int(arr0.size),
            residual_rms_before=float(np.sqrt(np.mean(arr0 ** 2))),
            residual_rms_after=float(np.sqrt(np.mean(arr1 ** 2))),
            residual_max_before=float(np.max(np.abs(arr0))),
            residual_max_after=float(np.max(np.abs(arr1))),
            residual_mean_after=float(np.mean(arr1)),
            residual_std_after=float(np.std(arr1)),
        )
    return out


def update_candidate(path: Path, report: dict) -> dict:
    data = yaml.safe_load(path.read_text())
    n = N_ARM
    kinetic = data.get("kinetic_friction_nm")
    if not isinstance(kinetic, list) or len(kinetic) != n:
        kinetic = [None] * n
        data["kinetic_friction_nm"] = kinetic
    pos = data.get("kinetic_friction_pos_nm")
    if not isinstance(pos, list) or len(pos) != n:
        pos = list(kinetic)
        data["kinetic_friction_pos_nm"] = pos
    neg = data.get("kinetic_friction_neg_nm")
    if not isinstance(neg, list) or len(neg) != n:
        neg = list(kinetic)
        data["kinetic_friction_neg_nm"] = neg
    accepted = []
    rejected = []
    for joint_s, item in report["summary"].items():
        j = int(joint_s) - 1
        if item.get("ok"):
            p = item["directions"]["pos"]["friction_nm"]
            m = item["directions"]["neg"]["friction_nm"]
            pos[j] = float(p)
            neg[j] = float(m)
            kinetic[j] = float(0.5 * (p + m))
            accepted.append(j + 1)
        else:
            rejected.append(dict(joint=j + 1, reason=item.get("reason")))
    meta = data.setdefault("metadata", {})
    meta["candidate_updated_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    meta["candidate_phase5_directional_friction"] = dict(
        source=report["report_path"],
        accepted_joints=accepted,
        rejected=rejected,
        note="zero_offset and first_moment_ratio frozen; official YAML untouched",
    )
    path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
    return dict(candidate=str(path), accepted_joints=accepted, rejected=rejected,
                kinetic_friction_nm=kinetic,
                kinetic_friction_pos_nm=pos,
                kinetic_friction_neg_nm=neg)


def print_plan(args, postures: list[np.ndarray]) -> int:
    model = GravityModel()
    cal = load_arm6_calibration(str(args.calibration_yaml))
    stiction = np.asarray(cal["stiction_nm"], dtype=float)
    print("=" * 80)
    print("PLAN only: Phase5 directional friction dynamic test")
    print(f"calibration: {cal['source']}")
    print(f"joints: {args.joints}; scales: {args.scales}")
    print(f"move_timeout: {args.move_timeout:.1f}s")
    for pi, p in enumerate(postures):
        rep = check_setpoint(p, model)
        margin = desk_margin(model, p)
        print(f"posture{pi}: {np.round(p, 4).tolist()} margin={margin:+.4f} ok={rep.ok}")
        if not rep.ok or margin < DESK_MARGIN_MIN:
            print(f"REFUSED: posture{pi} unsafe: {rep.reasons}")
            return 3
        for joint in [int(x) for x in args.joints.split(",") if x.strip()]:
            for sign in (-1, 1):
                edge = p.copy()
                edge[joint - 1] += sign * DISP_ABORT
                rep2 = check_setpoint(edge, model)
                margin2 = desk_margin(model, edge)
                if not rep2.ok or margin2 < 0.0:
                    print(f"REFUSED: posture{pi} joint{joint} pulse edge unsafe: "
                          f"{np.round(edge, 4).tolist()} margin={margin2:+.4f} {rep2.reasons}")
                    return 4
    for j in [int(x) for x in args.joints.split(",") if x.strip()]:
        torques = [stiction[j - 1] * s for s in parse_scales(args.scales)]
        print(f"joint{j}: stiction={stiction[j - 1]:.3f} Nm, pulses={np.round(torques, 3).tolist()} Nm each direction")
    print("official YAML will not be written")
    print("=" * 80)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--calibration-yaml", type=Path, default=DEFAULT_CANDIDATE)
    ap.add_argument("--joints", default="2,4")
    ap.add_argument("--postures", default=DEFAULT_POSTURES)
    ap.add_argument("--scales", default="1.5,2.0,2.5,3.0")
    ap.add_argument("--move-timeout", type=float, default=45.0,
                    help="seconds allowed for each move_to posture transition")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--update-candidate", action="store_true")
    ap.add_argument("--analyze-report", type=Path, default=None,
                    help="reanalyze an existing JSON without commanding hardware")
    args = ap.parse_args()

    postures = parse_postures(args.postures)
    global MOVE_TIMEOUT
    MOVE_TIMEOUT = float(args.move_timeout)
    if args.dry_run:
        return print_plan(args, postures)
    if args.analyze_report:
        report = json.loads(args.analyze_report.read_text())
        records = report.get("records", [])
        # Old reports may have rejected direction-correct pulses because their
        # instantaneous RNEA residual was negative.  Reapply only the measured
        # motion gate for the dynamic intercept method.
        for rec in records:
            joint = int(rec["joint"])
            sign = int(rec["direction"])
            disp = float(rec.get("displacement") or 0.0)
            vp = float(rec.get("v_peak_signed") or 0.0)
            qdd = float(rec.get("qdd_fit") or 0.0)
            disp_min = 0.012 if joint == 2 else 0.006
            v_min = 0.10 if joint == 2 else 0.06
            qdd_min = 0.8 if joint == 2 else 0.5
            rec["accepted"] = bool(sign * disp > disp_min
                                   and sign * vp > v_min
                                   and sign * qdd > qdd_min)
        report["summary"] = summarize(records)
        report["validation"] = validation(records, report["summary"])
        report["analysis_method"] = "direction-gated Phase5 qdd-vs-pulse intercept"
        if args.update_candidate:
            report["candidate_update"] = update_candidate(args.calibration_yaml, report)
        args.analyze_report.write_text(json.dumps(
            report, indent=2, ensure_ascii=False, default=float))
        print(f"reanalyzed -> {args.analyze_report}")
        for joint_s, item in report["summary"].items():
            v = report["validation"].get(joint_s, {})
            for key in ("pos", "neg"):
                d = item["directions"][key]
                fit = d["dynamic_fit"]
                print(f"J{joint_s} {key} ok={d['ok']} n={d['n_accepted']} "
                      f"M={fit.get('inertia_kgm2')} F={fit.get('friction_nm')} "
                      f"qdd_rmse={fit.get('qdd_fit_rmse')} "
                      f"rms_before={v.get('residual_rms_before')} "
                      f"rms_after={v.get('residual_rms_after')}")
        print("candidate_update:", report.get("candidate_update"))
        return 0
    if rclpy is None:
        print(f"rclpy unavailable: {_IMPORT_ERROR}", file=sys.stderr)
        return 2
    joints = [int(x) for x in args.joints.split(",") if x.strip()]
    scales = parse_scales(args.scales)

    OUT.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    rclpy.init()
    node = DirectionalFrictionNode(str(args.calibration_yaml))
    records = []
    try:
        if not node.wait_state():
            print("no /joint_states", file=sys.stderr)
            return 5
        why = node.check_effort_enabled()
        if why:
            print(f"REFUSED: {why}", file=sys.stderr)
            return 6
        for pi, posture in enumerate(postures):
            why = node.safety_reason(posture)
            if why:
                print(f"REFUSED: posture{pi} unsafe: {why}", file=sys.stderr)
                return 7
            for joint in joints:
                f = float(node.stiction[joint - 1])
                for scale in scales:
                    for sign in (1, -1):
                        tau_extra = sign * f * scale
                        node.get_logger().info(
                            f"posture{pi} joint{joint} dir {sign:+d} "
                            f"pulse {tau_extra:+.3f} Nm ({scale:.1f}x)")
                        rec = node.pulse(posture, joint, sign, tau_extra, pi, scale)
                        records.append(rec)
                        status = "ACCEPT" if rec.get("accepted") else "reject"
                        fr = rec.get("friction_estimate_nm", {}).get("median")
                        node.get_logger().info(
                            f"  {status}: disp={rec.get('displacement')} "
                            f"vpk={rec.get('v_peak_signed')} qdd={rec.get('qdd_fit')} "
                            f"F={fr} reason={rec.get('reason')}")
        node.hold(postures[0], 1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    summ = summarize(records)
    val = validation(records, summ)
    report_path = OUT / f"phase5_directional_friction_{stamp}.json"
    csv_path = OUT / f"phase5_directional_friction_{stamp}_summary.csv"
    report = dict(
        report_path=str(report_path),
        calibration_yaml=str(args.calibration_yaml),
        zero_offset_rad=node.offset.tolist(),
        first_moment_ratio=node.ratio.tolist(),
        joints=joints,
        scales=scales,
        postures=[p.tolist() for p in postures],
        records=records,
        summary=summ,
        validation=val,
        candidate_update=None,
    )
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False, default=float))
    if args.update_candidate:
        report["candidate_update"] = update_candidate(args.calibration_yaml, report)
        report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False, default=float))

    with csv_path.open("w", newline="") as fh:
        fields = [
            "joint", "direction", "ok", "n_accepted", "inertia_kgm2",
            "friction_nm", "qdd_fit_rmse", "postures",
            "rms_before", "rms_after", "max_before", "max_after",
        ]
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for joint_s, item in summ.items():
            v = val.get(joint_s, {})
            for key in ("pos", "neg"):
                d = item["directions"][key]
                fit = d["dynamic_fit"]
                writer.writerow(dict(
                    joint=joint_s,
                    direction=key,
                    ok=d["ok"],
                    n_accepted=d["n_accepted"],
                    inertia_kgm2=fit.get("inertia_kgm2"),
                    friction_nm=fit.get("friction_nm"),
                    qdd_fit_rmse=fit.get("qdd_fit_rmse"),
                    postures=d["postures"],
                    rms_before=v.get("residual_rms_before"),
                    rms_after=v.get("residual_rms_after"),
                    max_before=v.get("residual_max_before"),
                    max_after=v.get("residual_max_after"),
                ))

    print(f"\nresults -> {report_path}")
    print(f"summary_csv -> {csv_path}")
    print("\nJ dir ok n M F qdd_rmse RMS_before RMS_after")
    for joint_s, item in summ.items():
        v = val.get(joint_s, {})
        for key in ("pos", "neg"):
            d = item["directions"][key]
            fit = d["dynamic_fit"]
            print(f"J{joint_s} {key:>3} {d['ok']} {d['n_accepted']:>2} "
                  f"{fit.get('inertia_kgm2')} {fit.get('friction_nm')} "
                  f"{fit.get('qdd_fit_rmse')} "
                  f"{v.get('residual_rms_before')} {v.get('residual_rms_after')}")
    if report.get("candidate_update"):
        print(f"\ncandidate updated -> {args.calibration_yaml}")
        print(report["candidate_update"])
    else:
        print("\ncandidate not updated (pass --update-candidate to merge accepted joints)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
