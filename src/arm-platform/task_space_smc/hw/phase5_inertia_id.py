"""关节等效惯量辨识（实物）。

在保持位姿上给单个关节叠加一个超过静摩擦的力矩脉冲，量测由此产生的角加速度：

    q̈ = (τ_extra − f) / M

对若干个脉冲幅值做一元线性回归 q̈ = a·τ_extra + b，得到

    M = 1/a,        隐含摩擦 f = −b/a

隐含摩擦与实测静摩擦一致，即说明上面这个模型成立、M 可信。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from dummy_interface.msg import MotorControl
except ImportError as exc:
    rclpy = None
    _IMPORT_ERROR = exc

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from task_space_smc.hw.fk_gravity_verify import (
    GravityModel, check_setpoint, N_ARM, JOINT_LOWER, JOINT_UPPER,
)
from task_space_smc.hw.phase4_gravity_compensation import (
    desk_margin, BASE_P, BASE_D,
)
from task_space_smc.core.model import load_arm6_calibration

OUT = Path(__file__).resolve().parent / "data" / "phase5"

STICTION = [0.725, 1.775, 0.625, 0.125, 0.100, 0.100]


PULSE_SCALES = [1.3, 1.6, 1.9, 2.2, 2.5]

PUBLISH_RATE = 100.0
PULSE_MAX_SEC = 0.20
DISP_ABORT = 0.10
SPEED_ABORT = 2.0
SETTLE_SEC = 1.5
RECOVER_SEC = 3.0
DESK_MARGIN_MIN = 0.10


class InertiaId(Node):
    def __init__(self, calibration_yaml: str | None = None):
        super().__init__("phase5_inertia_id")
        self.model = GravityModel()
        cal = load_arm6_calibration(calibration_yaml)
        self.offset = np.array(cal["zero_offset_rad"], dtype=float)
        self.ratio = np.array(cal["first_moment_ratio"], dtype=float)
        self.stiction = np.array(cal["stiction_nm"], dtype=float)
        self.cal_source = cal["source"]
        self.q = None
        self.v = None
        self.tau = None
        self.stamp = None
        self.pub = self.create_publisher(
            MotorControl, "/student/joint_effort_command", 10)
        self.create_subscription(JointState, "/joint_states", self._cb, 50)
        self.get_logger().info(f"calibration YAML: {self.cal_source}")
        self.get_logger().info(
            f"zero_offset_deg: {np.round(np.degrees(self.offset), 3).tolist()}")
        self.get_logger().info(
            f"first_moment_ratio: {np.round(self.ratio, 4).tolist()}")
        self.get_logger().info(
            f"stiction_nm: {np.round(self.stiction, 4).tolist()}")

    def _cb(self, m):
        if len(m.position) == N_ARM:
            self.q = np.array(m.position)
        if len(m.velocity) == N_ARM:
            self.v = np.array(m.velocity)
        if len(m.effort) == N_ARM:
            self.tau = np.array(m.effort)
        self.stamp = time.time()

    def feedforward(self, q):
        return self.model.gravity(np.asarray(q, float) + self.offset) * self.ratio

    def send(self, posture, kp, kd, tff):
        msg = MotorControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [float(v) for v in posture]
        msg.velocity = [0.0] * N_ARM
        msg.current = [float(v) for v in tff]
        msg.p = [float(v) for v in kp]
        msg.d = [float(v) for v in kd]
        msg.enable = 1
        self.pub.publish(msg)

    def wait_state(self, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout and (self.q is None or self.v is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.q is not None and self.v is not None

    def check_effort_enabled(self, timeout=5.0):
        from rcl_interfaces.srv import GetParameters
        cli = self.create_client(GetParameters,
                                 '/student_arm_node/get_parameters')
        if not cli.wait_for_service(timeout_sec=timeout):
            return "parameter service unavailable"
        req = GetParameters.Request()
        req.names = ['enable_direct_effort_command']
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.result() is None or not fut.result().values:
            return "could not read enable_direct_effort_command"
        if not fut.result().values[0].bool_value:
            return "node is in POSITION mode — relaunch with effort enabled"
        return None

    def hold(self, q_hold, seconds, kp_frac=1.0, kd_frac=1.0):
        kp = [BASE_P[i] * kp_frac for i in range(N_ARM)]
        kd = [BASE_D[i] * kd_frac for i in range(N_ARM)]
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < seconds:
            now = time.time()
            if now >= nxt:
                self.send(q_hold, kp, kd,
                          self.feedforward(self.q if self.q is not None else q_hold))
                nxt = now + 1.0 / PUBLISH_RATE
            rclpy.spin_once(self, timeout_sec=0.002)

    def move_to(self, q_target, *, timeout=35.0, settle=2.0):
        q_target = np.asarray(q_target, dtype=float)
        if self.q is None:
            return "no current joint state"
        rep = check_setpoint(q_target, self.model)
        mg = desk_margin(self.model, q_target)
        if not rep.ok or mg < DESK_MARGIN_MIN:
            return f"target posture unsafe (margin {mg:+.4f}, reasons={rep.reasons})"
        q_start = self.q.copy()
        dist = float(np.max(np.abs(q_target - q_start)))
        ramp_sec = max(4.0, dist / 0.18)
        t0 = time.time()
        stable_since = None
        nxt = t0
        while time.time() - t0 < timeout:
            now = time.time()
            alpha = min(1.0, (now - t0) / ramp_sec)
            cmd = q_start + alpha * (q_target - q_start)
            if now >= nxt:
                self.send(cmd, BASE_P, BASE_D,
                          self.feedforward(self.q if self.q is not None else cmd))
                nxt = now + 1.0 / PUBLISH_RATE
            rclpy.spin_once(self, timeout_sec=0.002)
            if self.q is None:
                continue
            if self.v is not None and np.max(np.abs(self.v)) > 1.0:
                return f"speed {np.max(np.abs(self.v)):.2f} rad/s exceeded"
            if desk_margin(self.model, self.q) < 0.0:
                return f"desk/fence margin {desk_margin(self.model, self.q):+.4f}"
            lim = np.minimum(self.q - JOINT_LOWER, JOINT_UPPER - self.q)
            if lim.min() < 0.05:
                return f"joint{int(np.argmin(lim)) + 1} within 0.05 rad of a limit"
            err = float(np.max(np.abs(self.q - q_target)))
            vmax = float(np.max(np.abs(self.v))) if self.v is not None else 0.0
            if err < 0.05:
                return None
            if err < 0.06 and vmax < 0.12:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= settle:
                    return None
            else:
                stable_since = None
        return f"move_to timed out {np.max(np.abs(q_target - self.q)):.3f} rad away"

    def pulse(self, q_hold, joint, tau_extra) -> tuple[dict | None, str | None]:
        idx = joint - 1
        kp = [0.0] * N_ARM
        kd = [BASE_D[i] * 0.2 for i in range(N_ARM)]
        q0 = self.q.copy()
        samples = []
        t0 = time.time()
        nxt = t0
        why = None
        while time.time() - t0 < PULSE_MAX_SEC:
            now = time.time()
            if now >= nxt:
                q_now = self.q if self.q is not None else q_hold
                tff = self.feedforward(q_now)
                tau_ff = float(tff[idx])
                tff[idx] += tau_extra
                self.send(q_hold, kp, kd, tff)
                nxt = now + 1.0 / PUBLISH_RATE
            rclpy.spin_once(self, timeout_sec=0.001)
            if self.q is None:
                continue
            d = abs(self.q[idx] - q0[idx])
            sp = abs(self.v[idx]) if self.v is not None else 0.0


            samp = dict(t=float(now - t0),
                        q=float(self.q[idx]),
                        v=float(self.v[idx]) if self.v is not None else 0.0,
                        effort=float(self.tau[idx]) if self.tau is not None else None,
                        tau_ff=float(tau_ff) if 'tau_ff' in locals() else None,
                        tau_extra_cmd=float(tau_extra))
            prev_key = None if not samples else (
                samples[-1]["q"], samples[-1]["v"], samples[-1]["effort"])
            key = (samp["q"], samp["v"], samp["effort"])
            if prev_key != key:
                samples.append(samp)
            if d > DISP_ABORT:
                why = f"displacement {d:.4f} rad"
                break
            if sp > SPEED_ABORT:
                why = f"speed {sp:.2f} rad/s"
                break
            mg = desk_margin(self.model, self.q)
            if mg < 0.0:
                why = f"desk/fence margin {mg:+.4f}"
                break
            lim = min(self.q[idx] - JOINT_LOWER[idx], JOINT_UPPER[idx] - self.q[idx])
            if lim < 0.10:
                why = f"within {lim:.3f} rad of a limit"
                break
        if len(samples) < 5:
            return None, (why or "too few samples")
        t = np.array([s["t"] for s in samples])
        qq = np.array([s["q"] for s in samples])
        vv = np.array([s["v"] for s in samples])

        # 脉冲段内按匀加速拟合速度 v(t) = q̈·t + v0，最小二乘取斜率作为 q̈
        A = np.column_stack([t, np.ones_like(t)])
        coef, *_ = np.linalg.lstsq(A, vv, rcond=None)
        resid = vv - A @ coef
        return dict(tau_extra=float(tau_extra), qdd=float(coef[0]),
                    v0=float(coef[1]), n=len(samples),
                    duration=float(t[-1]), displacement=float(qq[-1] - qq[0]),
                    v_max=float(np.max(np.abs(vv))),
                    fit_resid=float(np.sqrt(np.mean(resid ** 2))),
                    aborted=why,
                    effort_qdd_alignment=effort_qdd_alignment(samples),
                    samples=samples), None


def effort_qdd_alignment(samples: list[dict], max_lag: int = 6) -> dict:
    if len(samples) < 8:
        return dict(ok=False, reason="too few samples")
    t = np.array([s["t"] for s in samples], dtype=float)
    v = np.array([s["v"] for s in samples], dtype=float)
    effort = np.array([
        np.nan if s.get("effort") is None else float(s["effort"])
        for s in samples
    ], dtype=float)
    tau_ff = np.array([
        np.nan if s.get("tau_ff") is None else float(s["tau_ff"])
        for s in samples
    ], dtype=float)
    if np.any(~np.isfinite(effort)) or np.any(~np.isfinite(tau_ff)):
        return dict(ok=False, reason="missing effort")
    if np.any(np.diff(t) <= 0):
        return dict(ok=False, reason="nonmonotonic time")
    qdd = np.gradient(v, t)
    effort_resid = effort - tau_ff
    dt = float(np.median(np.diff(t)))
    best = None
    scans = []
    for lag in range(-max_lag, max_lag + 1):
        if lag < 0:
            x = effort_resid[-lag:]
            y = qdd[:lag]
        elif lag > 0:
            x = effort_resid[:-lag]
            y = qdd[lag:]
        else:
            x = effort_resid
            y = qdd
        if len(x) < 6 or np.std(x) < 1e-9 or np.std(y) < 1e-9:
            corr = float("nan")
        else:
            corr = float(np.corrcoef(x - x.mean(), y - y.mean())[0, 1])
        item = dict(lag_samples=lag, lag_sec=lag * dt, corr=corr)
        scans.append(item)
        if np.isfinite(corr) and (best is None or abs(corr) > abs(best["corr"])):
            best = item
    return dict(ok=best is not None,
                best=best,
                scan=scans,
                qdd_gradient_mean=float(np.mean(qdd)),
                qdd_gradient_std=float(np.std(qdd)),
                effort_resid_mean=float(np.mean(effort_resid)),
                effort_resid_std=float(np.std(effort_resid)))


def identify_joint(node: InertiaId, joint: int, q_hold) -> dict:
    idx = joint - 1
    f = float(node.stiction[idx])
    recs = []
    for sc in PULSE_SCALES:
        te = f * sc
        node.get_logger().info(
            f"  joint{joint} 脉冲 {te:.3f} Nm ({sc:.1f}x 静摩擦)")
        node.hold(q_hold, SETTLE_SEC)
        rec, why = node.pulse(q_hold, joint, te)
        node.hold(q_hold, RECOVER_SEC)
        if rec is None:
            node.get_logger().warn(f"    跳过: {why}")
            continue
        recs.append(rec)
        node.get_logger().info(
            f"    qdd={rec['qdd']:+7.2f} rad/s²  位移 {rec['displacement']:+.4f} rad  "
            f"v_max {rec['v_max']:.2f}  n={rec['n']}"
            + (f"  [提前终止: {rec['aborted']}]" if rec['aborted'] else ""))
    if len(recs) < 3:
        return dict(joint=joint, ok=False, reason=f"only {len(recs)} usable pulses",
                    records=recs)
    # 回归 q̈ = a·τ_extra + b，取 M = 1/a、隐含摩擦 = −b/a；
    # se 是斜率的标准误，经 |se/a²| 传播成 M 的标准误。
    te = np.array([r['tau_extra'] for r in recs])
    qdd = np.array([r['qdd'] for r in recs])
    A = np.column_stack([te, np.ones_like(te)])
    coef, *_ = np.linalg.lstsq(A, qdd, rcond=None)
    slope, intercept = coef
    resid = qdd - A @ coef
    n = len(te)
    se = float(np.sqrt(np.sum(resid ** 2) / max(n - 2, 1) /
                       max(np.sum((te - te.mean()) ** 2), 1e-12)))
    M = 1.0 / slope if slope > 0 else float('nan')
    return dict(joint=joint, ok=slope > 0, M=float(M),
                M_stderr=float(abs(se / slope ** 2)) if slope > 0 else None,
                slope=float(slope), intercept=float(intercept),
                friction_implied=float(-intercept / slope) if slope > 0 else None,
                friction_measured=f, n_pulses=n, records=recs)


def fit_direction(records: list[dict], sign: int) -> dict:
    recs = [r for r in records if int(np.sign(r["tau_extra"])) == sign]
    if len(recs) < 3:
        return dict(ok=False, reason=f"only {len(recs)} usable pulses")
    te = np.array([r["tau_extra"] for r in recs], dtype=float)
    qdd = np.array([r["qdd"] for r in recs], dtype=float)
    A = np.column_stack([te, np.ones_like(te)])
    coef, *_ = np.linalg.lstsq(A, qdd, rcond=None)
    slope, intercept = coef
    resid = qdd - A @ coef
    n = len(te)
    se = float(np.sqrt(np.sum(resid ** 2) / max(n - 2, 1) /
                       max(np.sum((te - te.mean()) ** 2), 1e-12)))
    ok = bool(slope > 0)
    return dict(ok=ok,
                direction="pos" if sign > 0 else "neg",
                M=float(1.0 / slope) if ok else None,
                M_stderr=float(abs(se / slope ** 2)) if ok else None,
                slope=float(slope),
                intercept=float(intercept),
                friction_dynamic=float(-sign * intercept / slope) if ok else None,
                n_pulses=n,
                qdd_values=qdd.tolist(),
                tau_values=te.tolist(),
                fit_rmse=float(np.sqrt(np.mean(resid ** 2))))


def summarize_alignment(records: list[dict]) -> dict:
    best = []
    for rec in records:
        align = rec.get("effort_qdd_alignment") or {}
        if align.get("ok") and align.get("best"):
            b = dict(align["best"])
            b["tau_extra"] = rec.get("tau_extra")
            b["direction"] = "pos" if rec.get("tau_extra", 0.0) > 0 else "neg"
            best.append(b)
    if not best:
        return dict(ok=False, reason="no valid alignment scans")
    lags = np.array([b["lag_sec"] for b in best], dtype=float)
    corrs = np.array([b["corr"] for b in best], dtype=float)
    return dict(ok=True,
                lag_sec_mean=float(np.mean(lags)),
                lag_sec_std=float(np.std(lags, ddof=1)) if len(lags) > 1 else 0.0,
                corr_abs_mean=float(np.mean(np.abs(corrs))),
                best_by_pulse=best)


def identify_joint_bidirectional(node: InertiaId, joint: int, q_hold) -> dict:
    idx = joint - 1
    f = float(node.stiction[idx])
    recs = []
    for sc in PULSE_SCALES:
        for sign, label in ((1, "+"), (-1, "-")):
            te = sign * f * sc
            node.get_logger().info(
                f"  joint{joint} {label}脉冲 {te:+.3f} Nm ({sc:.1f}x 静摩擦)")
            node.hold(q_hold, SETTLE_SEC)
            rec, why = node.pulse(q_hold, joint, te)
            node.hold(q_hold, RECOVER_SEC)
            if rec is None:
                node.get_logger().warn(f"    跳过: {why}")
                continue
            rec["direction"] = "pos" if sign > 0 else "neg"
            rec["scale"] = sc
            recs.append(rec)
            best = (rec.get("effort_qdd_alignment") or {}).get("best") or {}
            lag = best.get("lag_sec")
            corr = best.get("corr")
            node.get_logger().info(
                f"    qdd={rec['qdd']:+7.2f} rad/s²  位移 {rec['displacement']:+.4f} rad  "
                f"v_max {rec['v_max']:.2f}  n={rec['n']}"
                + (f"  lag={lag:+.4f}s corr={corr:+.2f}" if lag is not None and corr is not None else "")
                + (f"  [提前终止: {rec['aborted']}]" if rec['aborted'] else ""))
    pos = fit_direction(recs, 1)
    neg = fit_direction(recs, -1)
    ok = bool(pos.get("ok") and neg.get("ok"))
    m_vals = [x["M"] for x in (pos, neg) if x.get("ok") and x.get("M") is not None]
    return dict(joint=joint, ok=ok,
                M_mean=float(np.mean(m_vals)) if m_vals else None,
                M_split_abs=float(abs(pos["M"] - neg["M"])) if pos.get("ok") and neg.get("ok") else None,
                pos=pos, neg=neg,
                friction_measured=f,
                alignment=summarize_alignment(recs),
                records=recs)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--joints', default='2')
    ap.add_argument('--calibration-yaml', default=None,
                    help='candidate arm6 calibration YAML used for feedforward/stiction')
    ap.add_argument('--posture', default=None, help='逗号分隔的 6 个安全初始角度')
    ap.add_argument('--bidirectional', action='store_true',
                    help='同姿态正负双向、多幅值脉冲，分别拟合方向相关动摩擦')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()
    joints = [int(x) for x in args.joints.split(',')]

    if args.dry_run:
        m = GravityModel()
        print("=" * 72)
        print("PLAN (dry run — nothing is commanded)")
        print("=" * 72)
        print(f"{'关节':<7}{'静摩擦':>9}{'脉冲力矩(Nm)':>28}{'硬上限':>9}")
        cal = load_arm6_calibration(args.calibration_yaml)
        stiction = cal["stiction_nm"]
        print(f"calibration YAML: {cal['source']}")
        print(f"stiction_nm: {[round(v, 4) for v in stiction]}")
        for j in joints:
            f = stiction[j - 1]
            lv = "  ".join(f"{f*s:.2f}" for s in PULSE_SCALES)
            print(f"joint{j:<2}{f:>9.3f}   {lv:>25}{PULSE_MAX_SEC:>8.2f}s")
        print(f"\n中止: 位移 > {DISP_ABORT} rad / 速度 > {SPEED_ABORT} rad/s / "
              f"距限位 < 0.10 rad / 桌面围栏越界")
        print(f"每次脉冲前静置 {SETTLE_SEC}s，脉冲后位置保持 {RECOVER_SEC}s 回位")
        print("=" * 72)
        return 0
    if rclpy is None:
        print(f"rclpy unavailable: {_IMPORT_ERROR}", file=sys.stderr)
        return 2

    OUT.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    rclpy.init()
    node = InertiaId(args.calibration_yaml)
    results = []
    try:
        if not node.wait_state():
            print("no /joint_states", file=sys.stderr)
            return 3
        why = node.check_effort_enabled()
        if why:
            print(f"REFUSED: {why}", file=sys.stderr)
            return 4
        if node.cal_source is None:
            print("REFUSED: 标定文件缺失", file=sys.stderr)
            return 5
        q_target = None
        if args.posture:
            vals = [float(x.strip()) for x in args.posture.split(',') if x.strip()]
            if len(vals) != N_ARM:
                print(f"REFUSED: --posture expects {N_ARM} values", file=sys.stderr)
                return 7
            q_target = np.array(vals, dtype=float)
            node.get_logger().info(
                f"moving to requested posture {np.round(q_target, 4).tolist()}")
            why = node.move_to(q_target)
            if why:
                print(f"REFUSED: could not reach requested posture: {why}",
                      file=sys.stderr)
                return 8
            node.hold(q_target, 2.0)
        q_hold = q_target.copy() if q_target is not None else node.q.copy()
        rep = check_setpoint(q_hold, node.model)
        mg = desk_margin(node.model, q_hold)
        node.get_logger().info(
            f"起始位姿 {np.round(q_hold, 4).tolist()}  余量 {mg:+.4f}  "
            f"{'安全' if rep.ok else '未通过'}")
        if not rep.ok or mg < 0.10:
            print(f"REFUSED: 位姿不适合 (margin {mg:+.4f})", file=sys.stderr)
            return 6
        for j in joints:
            node.get_logger().info(f"===== joint{j} 惯量辨识 =====")
            r = (identify_joint_bidirectional(node, j, q_hold)
                 if args.bidirectional else identify_joint(node, j, q_hold))
            results.append(r)
            if args.bidirectional and r.get('ok'):
                node.get_logger().info(
                    f"joint{j}: M+={r['pos']['M']:.5f}, f+={r['pos']['friction_dynamic']:.3f}; "
                    f"M-={r['neg']['M']:.5f}, f-={r['neg']['friction_dynamic']:.3f}")
            elif r.get('ok'):
                node.get_logger().info(
                    f"joint{j}: M = {r['M']:.5f} ± {r['M_stderr']:.5f} kg·m²  "
                    f"隐含摩擦 {r['friction_implied']:.3f} (实测 {r['friction_measured']:.3f})")
        node.hold(q_hold, 2.0)
    finally:
        path = OUT / f"phase5_{stamp}.json"
        path.write_text(json.dumps(dict(results=results), indent=2, default=float))
        print(f"\nresults -> {path}")
        print(f"\n{'关节':<7}{'M(kg·m²)':>12}{'±σ':>10}{'隐含摩擦':>10}"
              f"{'实测摩擦':>10}{'脉冲数':>7}")
        for r in results:
            if not r.get('ok'):
                print(f"joint{r['joint']:<2}  失败: {r.get('reason')}")
                continue
            if args.bidirectional:
                print(f"joint{r['joint']:<2}"
                      f" M+={r['pos']['M']:.5f} f+={r['pos']['friction_dynamic']:.3f}"
                      f" M-={r['neg']['M']:.5f} f-={r['neg']['friction_dynamic']:.3f}"
                      f" lag={r['alignment'].get('lag_sec_mean')}")
                continue
            print(f"joint{r['joint']:<2}{r['M']:>12.5f}{r['M_stderr']:>10.5f}"
                  f"{r['friction_implied']:>10.3f}{r['friction_measured']:>10.3f}"
                  f"{r['n_pulses']:>7}")
        print("\n判据: 隐含摩擦应接近实测静摩擦 —— 两者一致说明模型 "
              "qdd=(tau-f)/M 成立，M 可信。")
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
