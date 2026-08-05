from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from dummy_interface.msg import MotorState
except ImportError as exc:
    rclpy = None
    _IMPORT_ERROR = exc

import pinocchio as pin


N_ARM = 6
JOINT_NAMES = [f"joint{i}" for i in range(1, N_ARM + 1)]

JOINT_LOWER = np.array([-4.0, -3.1415, -1.5708, -4.0, -2.0, -4.0])
JOINT_UPPER = np.array([4.0, 2.0, 1.5708, 4.0, 2.0, 4.0])
RATED_TORQUE = np.array([9.0, 9.0, 9.0, 1.6, 1.6, 1.6])


TAU_NOISE = np.array([0.0251, 0.0645, 0.0627, 0.0071, 0.0060, 0.0060])

LIMIT_MARGIN = 0.15


TORQUE_BUDGET = 0.65


DESK_PLANE_Z = 0.0
DESK_EDGE_X = 0.02
DESK_CLEARANCE = 0.08


FENCE_Y = 0.40
FENCE_HEIGHT = 0.331
FENCE_CLEARANCE = 0.05


MAX_STEP = 0.35
MAX_MOVE_STEPS = 200


SAFE_WAYPOINT = np.array([-0.2968, 1.0676, 0.7130, -0.5496, -0.3997, 0.7471])


SWEEP_PLAN = {


    1: dict(posture=[-0.0239, 0.0482, 1.4088, 1.5000, 0.8844, -0.8976],
            lo=-0.8800, hi=1.4000, amp=5.4649, swing=4.8322,
            points=13, dwell=2.0, sigma=0.0233),
    2: dict(posture=[-0.0384, -0.0359, 1.1034, -1.5000, -1.5000, -1.5000],
            lo=-1.0359, hi=0.9641, amp=4.3304, swing=3.6436,
            points=13, dwell=5.0, sigma=0.0620),


    3: dict(posture=[0.0998, 1.2224, -0.3036, 1.0787, 0.2736, 0.2837],
            lo=-1.4200, hi=1.0000, amp=1.1306, swing=1.0556,
            points=21, dwell=5.0, sigma=0.1427),


    4: dict(posture=[0.0161, 0.7043, -0.7045, 1.4000, 1.4000, 1.4000],
            lo=-3.5000, hi=3.5000, amp=0.1716, swing=0.1706,
            points=13, dwell=7.5, sigma=0.0348),


    5: dict(posture=[-0.0069, 0.3348, 1.2292, -1.3767, 0.0000, -0.3033],
            lo=-1.7000, hi=1.7000, amp=0.1723, swing=0.1723,
            points=21, dwell=5.0, sigma=0.0493),


}


ZERO_OFFSET_ENGINEERING_THRESHOLD_DEG = 0.5


SHAPE_RESIDUAL_LIMIT = 0.05


POSITION_SETTLE_SPAN = 1.5e-3


TRACKING_ERROR_WARN = 0.01


APPROACH_OVERSHOOT = 0.15


DIRECTION_SCALE_MIN = 0.7
DIRECTION_SCALE_MAX = 1.3


def offset_threshold_deg(joint: int) -> float:
    sigma = SWEEP_PLAN[joint]["sigma"]
    return max(ZERO_OFFSET_ENGINEERING_THRESHOLD_DEG, 3.0 * sigma)


def urdf_path() -> str:
    here = Path(__file__).resolve()
    local = here.parents[2] / "config" / "arm6.urdf"
    if local.exists():
        return str(local)
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("manipulator"), "arm6.urdf")


class GravityModel:

    def __init__(self, path: str | None = None):
        self.path = path or urdf_path()
        self.model = pin.buildModelFromUrdf(self.path)
        if self.model.nq != N_ARM:
            raise ValueError(
                f"{self.path} has nq={self.model.nq}, expected {N_ARM}. "
                "This tool is for the 6-joint hardware configuration."
            )
        self.data = self.model.createData()
        self.ee_frame_id = self.model.getFrameId("link6")

    def gravity(self, q: np.ndarray) -> np.ndarray:
        return pin.computeGeneralizedGravity(
            self.model, self.data, np.asarray(q, dtype=float)).copy()

    def ee_position(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.ee_frame_id].translation.copy()

    def link_points(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pts = [self.data.oMi[self.model.getJointId(f"joint{j}")].translation.copy()
               for j in range(1, N_ARM + 1)]
        pts.append(self.data.oMf[self.ee_frame_id].translation.copy())
        return np.array(pts)

    def desk_violations(self, q: np.ndarray) -> list[tuple[str, np.ndarray]]:
        names = [f"joint{j}" for j in range(1, N_ARM + 1)] + ["ee"]
        bad = []
        for name, p in zip(names, self.link_points(q)):
            over_desk = p[0] < DESK_EDGE_X
            if over_desk and p[2] < DESK_PLANE_Z + DESK_CLEARANCE:
                bad.append((name, p))
            elif (over_desk and p[1] > FENCE_Y - FENCE_CLEARANCE
                    and p[2] < DESK_PLANE_Z + FENCE_HEIGHT):
                bad.append((name, p))
        return bad


@dataclass
class SafetyReport:
    ok: bool
    reasons: list[str] = field(default_factory=list)


def check_setpoint(q: np.ndarray, model: GravityModel,
                   q_from: np.ndarray | None = None) -> SafetyReport:
    rep = SafetyReport(ok=True)
    q = np.asarray(q, dtype=float)

    if q.shape != (N_ARM,):
        rep.ok = False
        rep.reasons.append(f"expected {N_ARM} joints, got {q.shape}")
        return rep
    if not np.all(np.isfinite(q)):
        rep.ok = False
        rep.reasons.append("contains NaN/Inf")
        return rep

    lo_slack = q - (JOINT_LOWER + LIMIT_MARGIN)
    hi_slack = (JOINT_UPPER - LIMIT_MARGIN) - q
    for j in range(N_ARM):
        if lo_slack[j] < 0 or hi_slack[j] < 0:
            rep.ok = False
            rep.reasons.append(
                f"joint{j+1}={q[j]:.4f} is within {LIMIT_MARGIN} rad of its limit "
                f"[{JOINT_LOWER[j]}, {JOINT_UPPER[j]}]")

    tau = model.gravity(q)
    frac = np.abs(tau) / RATED_TORQUE
    if np.max(frac) > TORQUE_BUDGET:
        j = int(np.argmax(frac))
        rep.ok = False
        rep.reasons.append(
            f"predicted gravity torque on joint{j+1} is {tau[j]:.3f} Nm = "
            f"{frac[j]*100:.0f}% of rated, over the {TORQUE_BUDGET*100:.0f}% budget")

    for name, p in model.desk_violations(q):
        rep.ok = False
        rep.reasons.append(
            f"{name} would be at x={p[0]*1000:.0f} y={p[1]*1000:.0f} "
            f"z={p[2]*1000:.0f} mm, i.e. over the desk (x<{DESK_EDGE_X*1000:.0f}mm) "
            f"and below the {DESK_CLEARANCE*1000:.0f}mm clearance")

    if q_from is not None:
        step = np.max(np.abs(q - np.asarray(q_from, dtype=float)))
        if step > MAX_STEP:
            rep.ok = False
            rep.reasons.append(
                f"single step of {step:.3f} rad exceeds MAX_STEP={MAX_STEP}")
    return rep


class FlushingWriter:

    def __init__(self, handle):
        self._handle = handle
        self._writer = csv.writer(handle)

    def writerow(self, row):
        self._writer.writerow(row)
        self._handle.flush()
        os.fsync(self._handle.fileno())


def plan_next_step(q_now: np.ndarray, q_target: np.ndarray) -> np.ndarray:
    q_now = np.asarray(q_now, dtype=float)
    q_target = np.asarray(q_target, dtype=float)
    span = float(np.max(np.abs(q_target - q_now)))
    if span <= MAX_STEP:
        return q_target.copy()
    return q_now + (q_target - q_now) * (MAX_STEP / span)


def interpolate(q_from: np.ndarray, q_to: np.ndarray) -> list[np.ndarray]:
    q_from = np.asarray(q_from, dtype=float)
    q_to = np.asarray(q_to, dtype=float)
    span = np.max(np.abs(q_to - q_from))
    n = max(1, int(np.ceil(span / MAX_STEP)))
    return [q_from + (q_to - q_from) * (k + 1) / n for k in range(n)]


def fit_sine(q: np.ndarray, tau: np.ndarray) -> dict:
    q = np.asarray(q, dtype=float)
    tau = np.asarray(tau, dtype=float)
    if q.size < 3:
        return dict(ok=False, reason="need at least 3 points")

    M = np.column_stack([np.sin(q), np.cos(q), np.ones_like(q)])
    coef, *_ = np.linalg.lstsq(M, tau, rcond=None)
    a1, a2, b = coef
    amplitude = float(np.hypot(a1, a2))
    phi = float(np.arctan2(-a2, a1))
    resid = tau - M @ coef
    return dict(
        ok=True,
        amplitude=amplitude,
        phi=phi,
        offset=float(b),
        rms_residual=float(np.sqrt(np.mean(resid ** 2))),
        max_residual=float(np.max(np.abs(resid))),
        n_points=int(q.size),
    )


def model_reference(model: GravityModel, joint: int,
                    posture: list[float], qs: np.ndarray) -> dict:
    j = joint - 1
    taus = []
    for v in qs:
        q = np.array(posture, dtype=float)
        q[j] = v
        taus.append(model.gravity(q)[j])
    return fit_sine(qs, np.array(taus))


def model_reference_measured(model: GravityModel, joint: int,
                             q_full: np.ndarray) -> dict:
    q_full = np.asarray(q_full, dtype=float)
    if q_full.ndim != 2 or q_full.shape[1] != N_ARM:
        raise ValueError(f"expected an (n, {N_ARM}) array of measured poses")
    j = joint - 1
    taus = [model.gravity(q)[j] for q in q_full]
    return fit_sine(q_full[:, j], np.array(taus))


class FkGravityVerifier(Node):
    def __init__(self, args):
        super().__init__('fk_gravity_verify')
        self.args = args
        self.model = GravityModel()
        self.q = None
        self.qd = None
        self.tau = None
        self.joint_order = None


        self.safety_abort = False


        self.unidirectional_violations = 0

        self.pub = self.create_publisher(JointState, '/student/joint_command', 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self.create_subscription(MotorState, '/student/joint_feedback',
                                 self._on_feedback, 10)

    def _on_joint_state(self, msg: JointState):
        if len(msg.position) != N_ARM:
            self.get_logger().error(
                f"/joint_states has {len(msg.position)} joints, expected {N_ARM}. "
                "Are you running student_arm6.launch.py?", throttle_duration_sec=5.0)
            return
        self.joint_order = list(msg.name)
        self.q = np.array(msg.position, dtype=float)
        self.qd = np.array(msg.velocity, dtype=float) if len(msg.velocity) == N_ARM \
            else np.zeros(N_ARM)
        if len(msg.effort) == N_ARM:
            self.tau = np.array(msg.effort, dtype=float)

    def _on_feedback(self, msg: MotorState):
        if len(msg.current) == N_ARM:
            self.tau = np.array(msg.current, dtype=float)


    def wait_for_state(self, timeout=10.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.q is not None and self.tau is not None:
                return True
        return False

    def publish(self, q_target: np.ndarray):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(v) for v in q_target]
        msg.velocity = [0.0] * N_ARM
        self.pub.publish(msg)

    def goto(self, q_target: np.ndarray, timeout=15.0) -> bool:
        q_target = np.asarray(q_target, dtype=float)
        t0 = time.time()
        window = []
        while time.time() - t0 < timeout:
            self.publish(q_target)
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.q is None:
                continue
            window.append((time.time(), self.q.copy()))
            window = [(t, q) for t, q in window
                      if time.time() - t <= self.args.settle_sec]
            if len(window) >= 20 and time.time() - t0 > self.args.settle_sec:
                span = np.max(np.ptp(np.array([q for _, q in window]), axis=0))
                if span <= POSITION_SETTLE_SPAN:
                    err = float(np.max(np.abs(self.q - q_target)))
                    if err > TRACKING_ERROR_WARN:
                        self.get_logger().warn(
                            f"settled with tracking error {err:.4f} rad "
                            f"(kp-limited sag, expected on gravity-loaded joints)")
                    return True
        self.get_logger().warn(
            f"goto: motion did not settle within {timeout:g}s, "
            f"residual {np.max(np.abs(self.q - q_target)):.4f} rad")
        return False

    def approach(self, q_target: np.ndarray, joint_idx: int) -> bool:
        q_target = np.asarray(q_target, dtype=float)
        for overshoot in (APPROACH_OVERSHOOT, APPROACH_OVERSHOOT * 0.5,
                          APPROACH_OVERSHOOT * 0.25):
            q_over = q_target.copy()
            q_over[joint_idx] += overshoot
            if check_setpoint(q_over, self.model).ok:
                if not self.goto(q_over):
                    return False
                return self.goto(q_target)


        self.get_logger().warn(
            f"no safe overshoot for joint{joint_idx + 1} at "
            f"{q_target[joint_idx]:+.4f}; approaching directly")
        self.unidirectional_violations += 1
        return self.goto(q_target)

    def dwell_record(self, q_target: np.ndarray, seconds: float) -> dict | None:
        qs, taus = [], []
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.publish(q_target)
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.q is not None and self.tau is not None:
                qs.append(self.q.copy())
                taus.append(self.tau.copy())
        if len(qs) < 10:
            return None
        qs = np.array(qs)
        taus = np.array(taus)
        return dict(
            q_mean=qs.mean(axis=0), q_std=qs.std(axis=0),
            tau_mean=taus.mean(axis=0), tau_std=taus.std(axis=0),
            n=len(qs),
        )

    def move_through(self, q_target: np.ndarray,
                     allow_waypoint: bool = True) -> bool:
        q_target = np.asarray(q_target, dtype=float)
        rep = check_setpoint(q_target, self.model)
        if not rep.ok:
            self.get_logger().error("REFUSED target: " + "; ".join(rep.reasons))
            self.safety_abort = True
            return False

        if self._direct_move(q_target):
            return True
        if not allow_waypoint or self.safety_abort:
            return False


        self.get_logger().warn(
            "direct transit refused; retrying via the safe waypoint")
        self.safety_abort = False


        self.unidirectional_violations = 0
        if not self._direct_move(SAFE_WAYPOINT):
            self.get_logger().error("could not reach the safe waypoint either")
            self.safety_abort = True
            return False
        if not self._direct_move(q_target):
            self.get_logger().error("still refused after the waypoint")
            self.safety_abort = True
            return False
        return True

    def _direct_move(self, q_target: np.ndarray) -> bool:
        q_target = np.asarray(q_target, dtype=float)
        for _ in range(MAX_MOVE_STEPS):
            if float(np.max(np.abs(q_target - self.q))) <= MAX_STEP:
                return self.goto(q_target)
            step = plan_next_step(self.q, q_target)
            rep = check_setpoint(step, self.model)
            if not rep.ok:
                self.get_logger().warn("refused step: " + "; ".join(rep.reasons))
                return False
            if not self.goto(step):
                return False
        self.get_logger().error(
            f"move_through made no progress in {MAX_MOVE_STEPS} steps; "
            f"still {np.max(np.abs(q_target - self.q)):.4f} rad away")
        return False


def run_direction_check(node: FkGravityVerifier, joints: list[int],
                        delta: float, q_home: np.ndarray) -> list[dict]:
    results = []
    q_home = np.asarray(q_home, dtype=float).copy()
    for j in joints:
        idx = j - 1
        node.get_logger().info(f"--- direction check joint{j} (delta = +{delta} rad) ---")
        q_plus = q_home.copy()
        q_plus[idx] += delta
        rep = check_setpoint(q_plus, node.model, q_from=q_home)
        if not rep.ok:
            q_plus[idx] = q_home[idx] - delta
            rep = check_setpoint(q_plus, node.model, q_from=q_home)
            if not rep.ok:
                node.get_logger().error(f"joint{j} SKIPPED: " + "; ".join(rep.reasons))
                results.append(dict(joint=j, ok=False, reason="; ".join(rep.reasons)))
                continue
        commanded = q_plus[idx] - q_home[idx]


        if not node.move_through(q_home):
            results.append(dict(joint=j, ok=False, reason="could not reach baseline"))
            continue
        base = node.dwell_record(q_home, 1.0)
        if base is None:
            results.append(dict(joint=j, ok=False, reason="no baseline samples"))
            continue
        q_a = float(base["q_mean"][idx])

        if not node.move_through(q_plus):
            results.append(dict(joint=j, ok=False, reason="move failed"))
            node.move_through(q_home)
            continue
        rec = node.dwell_record(q_plus, 1.0)
        measured = float(rec["q_mean"][idx] - q_a) if rec else float("nan")
        node.move_through(q_home)


        scale_err = abs(measured - commanded)
        ratio = float(measured / commanded) if commanded else float("nan")
        sign_ok = bool(np.isfinite(measured)
                       and np.sign(measured) == np.sign(commanded))
        scale_ok = bool(np.isfinite(ratio)
                        and DIRECTION_SCALE_MIN <= abs(ratio) <= DIRECTION_SCALE_MAX)
        ok = sign_ok and scale_ok
        results.append(dict(joint=j, ok=ok, sign_ok=sign_ok, scale_ok=scale_ok,
                            commanded=commanded, measured=measured,
                            scale_error=scale_err, scale_ratio=ratio))
        node.get_logger().info(
            f"    commanded {commanded:+.4f} -> measured {measured:+.4f} rad  "
            f"ratio {ratio:+.3f}  sign {'OK' if sign_ok else 'INVERTED'}  "
            f"scale {'OK' if scale_ok else 'OFF'}")
    return results


def run_sweep(node: FkGravityVerifier, joint: int, n_points: int | None,
              dwell: float | None, csv_writer, is_warmup: bool = False) -> dict:
    plan = SWEEP_PLAN[joint]
    idx = joint - 1
    posture = list(plan["posture"])
    n_points = n_points or plan["points"]
    dwell = dwell if dwell is not None else plan["dwell"]
    qs = np.linspace(plan["lo"], plan["hi"], n_points)

    node.get_logger().info(
        ("=== WARM-UP " if is_warmup else "=== ") +
        f"joint{joint} sweep: {n_points} points x {dwell:g}s over "
        f"[{plan['lo']:.4f}, {plan['hi']:.4f}], model amplitude {plan['amp']:.4f} Nm, "
        f"expected 1-sigma {plan['sigma']:.4f} deg ===")

    q_start = np.array(posture, dtype=float)
    q_start[idx] = qs[0]
    if not node.move_through(q_start):
        return dict(joint=joint, ok=False, reason="could not reach start posture")

    samples = {"fwd": [], "rev": []}
    for direction, order in (("fwd", qs), ("rev", qs[::-1])):
        node.get_logger().info(f"  sweeping {direction}")
        for k, v in enumerate(order):
            q_t = np.array(posture, dtype=float)
            q_t[idx] = v


            ok = node.move_through(q_t) if k == 0 else node.approach(q_t, idx)
            if not ok:
                return dict(joint=joint, ok=False, reason=f"move failed at {v:.4f}")
            rec = node.dwell_record(q_t, dwell)
            if rec is None:
                return dict(joint=joint, ok=False, reason=f"no samples at {v:.4f}")


            samples[direction].append((rec["q_mean"].copy(), rec["tau_mean"][idx],
                                       rec["tau_std"][idx], rec["n"]))
            csv_writer.writerow([joint, f"{direction}{'_warmup' if is_warmup else ''}",
                                 f"{rec['q_mean'][idx]:.6f}",
                                 f"{rec['tau_mean'][idx]:.6f}",
                                 f"{rec['tau_std'][idx]:.6f}", rec["n"]]
                                + [f"{v:.6f}" for v in rec["q_mean"]])
            node.get_logger().info(
                f"    q={rec['q_mean'][idx]:+.4f}  tau={rec['tau_mean'][idx]:+.4f}"
                f" +/- {rec['tau_std'][idx]:.4f} Nm  (n={rec['n']})")

    fwd_q = np.array([s[0] for s in samples["fwd"]])
    rev_q = np.array([s[0] for s in samples["rev"]])[::-1]
    fwd_tau = np.array([s[1] for s in samples["fwd"]])
    rev_tau = np.array([s[1] for s in samples["rev"]])[::-1]

    q_full_avg = 0.5 * (fwd_q + rev_q)
    q_avg = q_full_avg[:, idx]
    tau_avg = 0.5 * (fwd_tau + rev_tau)
    stiction = 0.5 * (fwd_tau - rev_tau)

    meas = fit_sine(q_avg, tau_avg)
    ref = model_reference_measured(node.model, joint, q_full_avg)
    ref_nominal = model_reference(node.model, joint, posture, q_avg)
    posture_bias_deg = float(np.degrees(zero_offset(ref_nominal["phi"], ref["phi"])))

    out = dict(joint=joint, ok=True, warmup=is_warmup, posture=posture,
               posture_measured_mean=q_full_avg.mean(axis=0).tolist(),
               posture_bias_deg=posture_bias_deg,
               unidirectional_violations=node.unidirectional_violations,
               points=n_points, dwell=dwell,
               expected_sigma_deg=plan["sigma"],
               threshold_deg=offset_threshold_deg(joint),
               measured=meas, model=ref,
               stiction_mean=float(np.mean(np.abs(stiction))),
               stiction_max=float(np.max(np.abs(stiction))))
    if meas["ok"] and ref["ok"]:
        out["amplitude_ratio"] = meas["amplitude"] / ref["amplitude"] \
            if ref["amplitude"] > 1e-9 else float("nan")
        out["zero_offset_rad"] = zero_offset(meas["phi"], ref["phi"])
        out["zero_offset_deg"] = float(np.degrees(out["zero_offset_rad"]))
    return out


def zero_offset(phi_measured: float, phi_model: float) -> float:
    wrapped = np.arctan2(np.sin(phi_measured - phi_model),
                         np.cos(phi_measured - phi_model))
    return float(-wrapped)


def print_plan(model: GravityModel, joints: list[int],
               n_points: int | None, dwell: float | None):
    print("=" * 78)
    print("PLAN (dry run — nothing is commanded)")
    print("=" * 78)
    total = 0.0
    for j in joints:
        if j not in SWEEP_PLAN:
            print(f"\n  joint{j}: NO GRAVITY SIGNAL — link6 CoM sits on the joint6 axis "
                  f"(y/z offset 0.05/0.10 mm), so rotating it neither moves the centre "
                  f"of mass nor the end effector. Its zero offset is unobservable this "
                  f"way; it only gets the direction/scale check.")
            continue
        plan = SWEEP_PLAN[j]
        pts = n_points or plan["points"]
        dw = dwell if dwell is not None else plan["dwell"]
        qs = np.linspace(plan["lo"], plan["hi"], pts)
        worst = 0.0
        bad = []
        for v in qs:
            q = np.array(plan["posture"], dtype=float)
            q[j - 1] = v
            rep = check_setpoint(q, model)
            if not rep.ok:
                bad.append((v, rep.reasons))
            worst = max(worst, float(np.max(np.abs(model.gravity(q)) / RATED_TORQUE)))
        secs = pts * 2 * (dw + 2.0)
        total += secs

        tuned = (pts == plan["points"] and abs(dw - plan["dwell"]) < 1e-9)
        sigma_txt = (f"{plan['sigma']:.4f} deg (Monte-Carlo)" if tuned
                     else "unknown (config differs from the calibrated one)")
        print(f"\n  joint{j}: {pts} pts x 2 directions x {dw:g}s over "
              f"[{plan['lo']:.4f}, {plan['hi']:.4f}]")
        print(f"     other joints held at   : {plan['posture']}")
        print(f"     model amplitude        : {plan['amp']:.4f} Nm "
              f"(observed swing {plan['swing']:.4f})")
        print(f"     peak torque            : {worst*100:.1f}% of rated")
        print(f"     zero-offset 1-sigma    : {sigma_txt}")
        print(f"     decision threshold     : {offset_threshold_deg(j):.3f} deg")
        print(f"     estimated time         : {secs/60:.1f} min")
        if bad:
            print(f"     *** {len(bad)} SETPOINTS REFUSED ***")
            for v, reasons in bad[:3]:
                print(f"         q={v:.4f}: {'; '.join(reasons)}")
    print(f"\n  TOTAL ESTIMATED TIME: {total/60:.1f} min")
    print("=" * 78)


def check_position_path_active(node, timeout: float = 5.0) -> str | None:
    from rcl_interfaces.srv import GetParameters
    cli = node.create_client(GetParameters, '/student_arm_node/get_parameters')
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    req = GetParameters.Request()
    req.names = ['enable_direct_effort_command']
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if fut.result() is None or not fut.result().values:
        return None
    if fut.result().values[0].bool_value:
        return ("student_arm_node is in DIRECT EFFORT mode — it bypasses the "
                "position path entirely, so every command from this tool would "
                "be silently ignored. Relaunch with "
                "enable_direct_effort_command:=false")
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--joints', default='1,2,3,4,5',
                    help='逗号分隔的关节号，默认 1,2,3,4,5（joint6 无重力信号）')
    ap.add_argument('--points', type=int, default=None,
                    help='覆盖扫描点数；默认用每个关节各自标定过的配置')
    ap.add_argument('--dwell', type=float, default=None,
                    help='覆盖每点采样秒数；默认用每个关节各自标定过的配置')
    ap.add_argument('--settle-sec', type=float, default=0.6,
                    help='判稳窗口长度：位置在该窗口内不再变化即认为停稳')
    ap.add_argument('--direction-delta', type=float, default=0.10,
                    help='方向校验的幅度 (rad)')
    ap.add_argument('--direction-only', action='store_true', help='只做方向校验')
    ap.add_argument('--converge', action='store_true', default=True,
                    help='反复扫描直到连续两遍的零位偏差吻合到 2-sigma 内，'
                         '而不是固定热身遍数。实测该量在到达位姿后单调漂移，'
                         '两轮分别从 5.023/4.494 度爬向 9.75/9.41 度，收敛所需'
                         '遍数不同（一轮 2 遍、一轮 3 遍仍未到），固定遍数不可靠。'
                         '同期 joint2 温度升了 4 度，其余关节基本不变。')
    ap.add_argument('--no-converge', dest='converge', action='store_false',
                    help='关闭自动收敛，改用固定 --warmup 遍数')
    ap.add_argument('--max-sweeps', type=int, default=6,
                    help='收敛模式下单个关节最多扫多少遍')
    ap.add_argument('--warmup', type=int, default=1,
                    help='到达测试位姿后先跑几遍【丢弃】的扫描。实测：到达新'
                         '位姿后的首扫会被污染 —— joint2 四次首扫散布在 '
                         '-7.9~+5.6 度，而热身后的两遍吻合到 0.007 度。')
    ap.add_argument('--repeat', type=int, default=1,
                    help='把每个关节重复扫描 N 次并报告离散度。'
                         '单次拟合得再好也不能证明结果正确 —— 本轮的教训是'
                         'joint2/joint3 各自单次残差都很小，重复测量却差 9 度。')
    ap.add_argument('--goto', default=None,
                    help='只驱动到指定位姿后退出，如 "0,1.32,-1.37,0,0,0"')
    ap.add_argument('--dry-run', action='store_true', help='只打印计划，不动机械臂')
    ap.add_argument('--output-dir',
                    default=str(Path(__file__).resolve().parent / 'data' / 'fk_verify'))
    args = ap.parse_args()

    joints = [int(x) for x in args.joints.split(',') if x.strip()]
    model = GravityModel()

    if args.dry_run:
        print_plan(model, joints, args.points, args.dwell)
        q0 = np.zeros(N_ARM)
        print(f"\n  EE(link6) at q=0 : {np.round(model.ee_position(q0), 5)} m")
        return 0

    if rclpy is None:
        print(f"rclpy unavailable: {_IMPORT_ERROR}", file=sys.stderr)
        return 2

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')

    rclpy.init()
    node = FkGravityVerifier(args)
    try:
        if not node.wait_for_state():
            node.get_logger().error(
                "no /joint_states or torque feedback. Start student_arm6.launch.py "
                "with publish_joint_feedback:=true first.")
            return 3

        why = check_position_path_active(node)
        if why:
            node.get_logger().error(why)
            return 6

        node.get_logger().info(f"joint order from /joint_states: {node.joint_order}")
        if node.joint_order != JOINT_NAMES:
            node.get_logger().error(
                f"unexpected joint order {node.joint_order}, expected {JOINT_NAMES}")
            return 4

        q_home = node.q.copy()
        node.get_logger().info(f"start pose: {np.round(q_home, 4).tolist()}")


        rep_home = check_setpoint(q_home, model)
        if not rep_home.ok:
            node.get_logger().warn(
                "start pose is unsafe (" + "; ".join(rep_home.reasons) +
                "); using the safe waypoint as home instead")
            if not node.move_through(SAFE_WAYPOINT):
                node.get_logger().error("could not reach the safe waypoint")
                return 6
            q_home = node.q.copy()
            node.safety_abort = False
            node.get_logger().info(
                f"home reset to {np.round(q_home, 4).tolist()}")
        node.get_logger().info(
            f"predicted EE(link6): {np.round(model.ee_position(q_home), 4).tolist()} m "
            f"| distance from base_link: {np.linalg.norm(model.ee_position(q_home)):.4f} m")

        report = dict(timestamp=stamp, start_pose=q_home.tolist(),
                      urdf=model.path, joints=joints)

        if args.goto:
            q_goal = np.array([float(x) for x in args.goto.split(',')], dtype=float)
            rep = check_setpoint(q_goal, model)
            if not rep.ok:
                node.get_logger().error("REFUSED: " + "; ".join(rep.reasons))
                return 5
            node.get_logger().info(f"driving to {np.round(q_goal, 4).tolist()}")
            node.move_through(q_goal)
            node.dwell_record(q_goal, 2.0)
            node.get_logger().info(
                f"arrived at {np.round(node.q, 4).tolist()}, "
                f"tracking error {np.round(node.q - q_goal, 4).tolist()}")
            return 0

        report['direction_check'] = run_direction_check(
            node, joints, args.direction_delta, q_home)
        if args.direction_only:
            node.move_through(q_home)
        else:
            csv_path = out_dir / f'fk_verify_{stamp}.csv'
            with open(csv_path, 'w', newline='') as fh:
                w_csv = FlushingWriter(fh)
                w_csv.writerow(['joint', 'direction', 'q', 'tau_mean', 'tau_std', 'n']
                           + [f'q{k}_meas' for k in range(1, N_ARM + 1)])
                sweeps = []
                for j in joints:
                    if j not in SWEEP_PLAN:
                        node.get_logger().warn(
                            f"joint{j} has no gravity signal, skipping sweep")
                        sweeps.append(dict(joint=j, ok=False,
                                           reason='no gravity signal'))
                        continue
                    if args.converge:
                        prev = None
                        accepted = False
                        for k in range(args.max_sweeps):
                            r = run_sweep(node, j, args.points, args.dwell,
                                          w_csv, is_warmup=(prev is None))
                            if node.safety_abort or not r.get('ok'):
                                sweeps.append(r)
                                break
                            off = r.get('zero_offset_deg', float('nan'))
                            tol = 2.0 * SWEEP_PLAN[j]['sigma']
                            if prev is not None and abs(off - prev) <= tol:
                                node.get_logger().info(
                                    f"joint{j} converged: {prev:+.3f} -> {off:+.3f} deg "
                                    f"(within {tol:.3f} deg)")
                                r['warmup'] = False
                                r['converged'] = True
                                sweeps.append(r)
                                accepted = True
                                break
                            if prev is not None:
                                node.get_logger().info(
                                    f"joint{j} sweep {k + 1}: {off:+.3f} deg "
                                    f"(prev {prev:+.3f}, drift {off - prev:+.3f}) "
                                    f"— not converged, continuing")
                            r['warmup'] = True
                            sweeps.append(r)
                            prev = off
                        if not accepted and not node.safety_abort:
                            node.get_logger().error(
                                f"joint{j} did not converge in {args.max_sweeps} "
                                f"sweeps; all of them are marked as warm-up so "
                                f"nothing unconverged reaches the calibration")
                        continue
                    for wi in range(args.warmup):
                        node.get_logger().info(
                            f"--- joint{j} WARM-UP {wi + 1}/{args.warmup} "
                            f"(data recorded but excluded from the fit) ---")
                        r = run_sweep(node, j, args.points, args.dwell, w_csv,
                                      is_warmup=True)
                        sweeps.append(r)
                        if node.safety_abort or not r.get('ok'):
                            break
                    for rep in range(args.repeat):
                        if args.repeat > 1:
                            node.get_logger().info(
                                f"--- joint{j} repeat {rep + 1}/{args.repeat} ---")
                        r = run_sweep(node, j, args.points, args.dwell, w_csv)
                        r['repeat_index'] = rep
                        sweeps.append(r)
                        if node.safety_abort or not r.get('ok'):
                            break
                    if node.safety_abort:
                        node.get_logger().error(
                            f"safety check refused during joint{j}; aborting the "
                            f"whole run instead of continuing to the next joint")
                        report['aborted_at_joint'] = j
                        break
            report['sweeps'] = sweeps
            report['csv'] = str(csv_path)
            node.get_logger().info("returning to start pose")
            node.move_through(q_home)

        json_path = out_dir / f'fk_verify_{stamp}.json'
        with open(json_path, 'w') as fh:
            json.dump(report, fh, indent=2, default=float)
        node.get_logger().info(f"report written to {json_path}")
        print_verdict(report)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def print_verdict(report: dict):
    print("\n" + "=" * 78)
    print("VERDICT")
    print("=" * 78)
    for r in report.get('direction_check', []):
        if r.get('ok'):
            print(f"  joint{r['joint']} direction/scale : OK "
                  f"(cmd {r['commanded']:+.4f} -> meas {r['measured']:+.4f})")
        else:
            print(f"  joint{r['joint']} direction/scale : FAIL — "
                  f"{r.get('reason', 'sign or scale mismatch')}")

    if 'sweeps' not in report:
        return
    print()
    print(f"  {'joint':<8} {'A meas':>9} {'A model':>9} {'ratio':>7} "
          f"{'zero offset':>12} {'+/-1sig':>8} {'resid/A':>8} "
          f"{'stiction':>9}  verdict")
    offsets = {}
    for s in report['sweeps']:
        if not s.get('ok'):
            print(f"  joint{s['joint']:<3} -- {s.get('reason')}")
            continue
        m, ref = s['measured'], s['model']
        phi_deg = s.get('zero_offset_deg', float('nan'))
        sigma = s.get('expected_sigma_deg', float('nan'))


        rel_resid = (m['rms_residual'] / m['amplitude']
                     if m['amplitude'] > 1e-9 else float('inf'))
        if rel_resid > SHAPE_RESIDUAL_LIMIT:
            verdict = '*** GEOMETRY FAULT (shape) ***'
        elif abs(phi_deg) < 3.0 * sigma:
            verdict = 'zero offset not significant'
        else:
            verdict = 'zero offset -> apply as calibration'
            offsets[s['joint']] = phi_deg
        print(f"  joint{s['joint']:<3} {m['amplitude']:>9.4f} {ref['amplitude']:>9.4f} "
              f"{s.get('amplitude_ratio', float('nan')):>7.3f} "
              f"{phi_deg:>9.3f}deg {sigma:>8.4f} {rel_resid:>8.4f} "
              f"{s['stiction_mean']:>9.4f}  {verdict}")

    print("\n  怎么读这张表：")
    print("    resid/A 大   -> 正弦形状被破坏 = 真正的几何错（连杆长度/轴向），必须停下来查")
    print("    resid/A 小   -> 形状正确，模型结构没问题")
    print("    zero offset  -> 关节零位与 URDF 零位之差，是标定值不是故障，")
    print("                    URDF 零位与实际关节零位本就不同，这里把它量出来")
    print("    ratio != 1   -> 质量误差，阶段 2 标定，此前实测约 1.32~1.40")
    if offsets:
        print("\n  测得的零位偏差（实际角 = 命令角 + offset，单位 rad）：")
        vec = [round(np.radians(offsets.get(j, 0.0)), 6) for j in range(1, N_ARM + 1)]
        print(f"    {vec}")
        print("    joint6 无重力信号，其偏差保持 0")
    print("=" * 78)


if __name__ == '__main__':
    sys.exit(main())
