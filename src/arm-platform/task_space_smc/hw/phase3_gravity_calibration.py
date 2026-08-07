"""重力模型标定（实物）：逐关节扫掠，解出零位偏置与一阶矩比例。

单关节扫掠、其余关节锁死，实测力矩按 fk_gravity_verify.fit_sine 拟合成

    tau(q) = A·sin(q − φ) + b

与模型侧同样拟合的 (A_model, φ_model) 对比，得到两个标定量：

    zero_offset  = −wrap(φ − φ_model)      编码器零位偏置
    ratio        = A / A_model             一阶矩（质量×质心距）的比例修正

零位偏置会反过来改变下一轮扫掠时模型算出的力矩，故迭代扫掠直到偏置变化小于
tol_deg 为止。RATIO / STICTION 是前一阶段已标出的比例与静摩擦，扫掠时作为前馈使用。
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
    GravityModel, check_setpoint, fit_sine, model_reference_measured,
    zero_offset, JOINT_LOWER, JOINT_UPPER, N_ARM, SWEEP_PLAN,
    APPROACH_OVERSHOOT, DESK_PLANE_Z, DESK_EDGE_X, DESK_CLEARANCE,
)

OUT = Path(__file__).resolve().parent / "data" / "phase3"


HOLD_P = [30.0, 30.0, 30.0, 5.0, 5.0, 5.0]
HOLD_D = [1.0, 1.0, 1.0, 0.1, 0.1, 0.1]


RATIO = {1: 1.223, 2: 1.445, 3: 1.220, 4: 1.479, 5: 0.920, 6: 1.000}


STICTION = {1: 0.725, 2: 1.775, 3: 0.625, 4: 0.125, 5: 0.100, 6: 0.100}


POINTS = {1: 13, 2: 13, 3: 21, 4: 13, 5: 21}


DWELL = {1: 2.0, 2: 2.5, 3: 5.0, 4: 7.5, 5: 5.0}

RAMP_STEP = 0.20
RAMP_KD = 10.0
SPEED_ABORT = 1.3
SETTLE_SPAN = 1.5e-3
SETTLE_WIN = 0.6
PUBLISH_RATE = 100.0
DISPLACEMENT_ABORT = 0.15


# 桌面余量：只看伸到桌面上方（x < DESK_EDGE_X）的连杆点，取它们离
# 「桌面 + 安全间隙」最近的垂直距离，负值即已越界。
def desk_margin(model: GravityModel, q: np.ndarray) -> float:
    m = np.inf
    for p in model.link_points(np.asarray(q, float)):
        if p[0] < DESK_EDGE_X:
            m = min(m, p[2] - (DESK_PLANE_Z + DESK_CLEARANCE))
    return float(m)


class Calibrator(Node):
    def __init__(self):
        super().__init__("phase3_gravity_calibration")
        self.model = GravityModel()
        self.q = None
        self.v = None
        self.tau = None
        self.joint_order = None
        self.unidirectional_violations = 0
        self.pub = self.create_publisher(
            MotorControl, "/student/joint_effort_command", 10)
        self.create_subscription(JointState, "/joint_states", self._cb, 50)

    def _cb(self, m):
        if self.joint_order is None and m.name:
            self.joint_order = list(m.name)
        if len(m.position) == N_ARM:
            self.q = np.array(m.position)
        if len(m.velocity) == N_ARM:
            self.v = np.array(m.velocity)
        if len(m.effort) == N_ARM:
            self.tau = np.array(m.effort)


    def check_effort_enabled(self, timeout: float = 5.0) -> str | None:
        from rcl_interfaces.srv import GetParameters
        cli = self.create_client(GetParameters,
                                 '/student_arm_node/get_parameters')
        if not cli.wait_for_service(timeout_sec=timeout):
            return "student_arm_node parameter service unavailable"
        req = GetParameters.Request()
        req.names = ['enable_direct_effort_command']
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.result() is None or not fut.result().values:
            return "could not read enable_direct_effort_command"
        if not fut.result().values[0].bool_value:
            return ("student_arm_node is in POSITION mode — the torque "
                    "feedforward this stage depends on would be silently "
                    "dropped. Relaunch with enable_direct_effort_command:=true")
        return None

    def wait_state(self, timeout: float = 10.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout and (self.q is None or self.tau is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.q is not None and self.tau is not None


    # 重力前馈 tau_ff = ratio ⊙ g(q)，ratio 是已标定的一阶矩比例修正
    def feedforward(self, q: np.ndarray) -> np.ndarray:
        g = self.model.gravity(np.asarray(q, float))
        return np.array([g[i] * RATIO[i + 1] for i in range(N_ARM)])

    def send(self, posture, tff=None, d_gains=None):
        msg = MotorControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [float(v) for v in posture]
        msg.velocity = [0.0] * N_ARM
        msg.current = [0.0] * N_ARM if tff is None else [float(v) for v in tff]
        msg.p = list(HOLD_P)
        msg.d = list(HOLD_D) if d_gains is None else list(d_gains)
        msg.enable = 1
        self.pub.publish(msg)

    def guard(self, posture, joint_idx: int,
              check_drift: bool = True) -> str | None:
        if self.q is None:
            return None
        if check_drift:
            dq = np.abs(self.q - np.asarray(posture, float))
            dq[joint_idx] = 0.0
            if np.max(dq) > DISPLACEMENT_ABORT:
                j = int(np.argmax(dq)) + 1
                return f"joint{j} drifted {dq.max():.3f} rad from the posture"
        if self.model.desk_violations(self.q):
            return "desk violation at the measured pose"
        if self.v is not None and np.max(np.abs(self.v)) > SPEED_ABORT:
            return f"speed {np.max(np.abs(self.v)):.2f} rad/s exceeded"
        return None

    def goto(self, q_target, joint_idx: int, timeout: float = 40.0,
             check_drift: bool = True) -> str | None:
        q_target = np.asarray(q_target, float)
        period = 1.0 / PUBLISH_RATE
        t0 = time.time()
        nxt = t0
        win = []
        while time.time() - t0 < timeout:
            now = time.time()
            if self.q is None:
                rclpy.spin_once(self, timeout_sec=0.05)
                continue
            why = self.guard(q_target, joint_idx, check_drift)
            if why:
                return why
            err = q_target - self.q


            if np.max(np.abs(err)) < RAMP_STEP:
                win.append(self.q.copy())
                if len(win) > 60:
                    win.pop(0)
                    if np.max(np.ptp(np.array(win), axis=0)) < SETTLE_SPAN:
                        return None
            else:
                win.clear()
            if now >= nxt:
                cmd = self.q + np.clip(err, -RAMP_STEP, RAMP_STEP)
                if desk_margin(self.model, cmd) < 0.0:
                    return "commanded pose would intrude on the desk"
                self.send(cmd, self.feedforward(cmd), [RAMP_KD] * N_ARM)
                nxt = now + period
            rclpy.spin_once(self, timeout_sec=0.002)
        return f"goto timed out {np.max(np.abs(q_target - self.q)):.3f} rad away"

    def approach(self, q_target, joint_idx: int) -> str | None:
        q_target = np.asarray(q_target, float)
        for overshoot in (APPROACH_OVERSHOOT, APPROACH_OVERSHOOT * 0.5,
                          APPROACH_OVERSHOOT * 0.25):
            q_over = q_target.copy()
            q_over[joint_idx] += overshoot
            if check_setpoint(q_over, self.model).ok and \
                    desk_margin(self.model, q_over) > 0.0:
                why = self.goto(q_over, joint_idx)
                if why:
                    return why
                return self.goto(q_target, joint_idx)

        self.get_logger().warn(
            f"no safe overshoot for joint{joint_idx + 1} at "
            f"{q_target[joint_idx]:+.4f}; approaching directly")
        self.unidirectional_violations += 1
        return self.goto(q_target, joint_idx)

    def dwell(self, q_target, joint_idx: int, seconds: float) -> dict | None:
        q_target = np.asarray(q_target, float)
        tff = self.feedforward(q_target)
        period = 1.0 / PUBLISH_RATE
        qs, taus = [], []
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < seconds:
            now = time.time()
            if now >= nxt:
                self.send(q_target, tff)
                nxt = now + period
            rclpy.spin_once(self, timeout_sec=0.002)
            if self.q is not None and self.tau is not None:
                qs.append(self.q.copy())
                taus.append(self.tau.copy())
        if len(qs) < 10:
            return None
        qs, taus = np.array(qs), np.array(taus)
        return dict(q_mean=qs.mean(axis=0), q_std=qs.std(axis=0),
                    tau_mean=taus.mean(axis=0), tau_std=taus.std(axis=0),
                    n=len(qs))


def run_sweep(node: Calibrator, joint: int, offsets: dict,
              n_points: int, dwell: float) -> dict:
    idx = joint - 1
    plan = SWEEP_PLAN[joint]
    posture = np.array(plan['posture'], float)
    qs_cmd = np.linspace(plan['lo'], plan['hi'], n_points)


    first = posture.copy()
    first[idx] = qs_cmd[0]
    if node.q is not None and np.max(np.abs(node.q - first)) > 0.05:
        node.get_logger().info(
            f"  transiting to the sweep posture "
            f"({np.max(np.abs(node.q - first)):.3f} rad away)")
        why = node.goto(first, idx, timeout=90.0, check_drift=False)
        if why:
            return dict(joint=joint, ok=False, reason=f"transit failed: {why}",
                        rows=[])

    rows = []
    for k, qv in enumerate(qs_cmd):
        target = posture.copy()
        target[idx] = qv
        rep = check_setpoint(target, node.model)
        if not rep.ok:
            node.get_logger().warn(
                f"  skip q={qv:+.4f}: {'; '.join(rep.reasons)}")
            continue
        why = node.approach(target, idx)
        if why:
            node.get_logger().error(f"  abort at q={qv:+.4f}: {why}")
            return dict(joint=joint, ok=False, reason=why, rows=rows)
        rec = node.dwell(target, idx, dwell)
        if rec is None:
            return dict(joint=joint, ok=False, reason="dwell got no samples",
                        rows=rows)
        rows.append(dict(q_cmd=float(qv),
                         q_meas=float(rec['q_mean'][idx]),
                         posture_meas=rec['q_mean'].tolist(),
                         tau=float(rec['tau_mean'][idx]),
                         tau_std=float(rec['tau_std'][idx]),
                         sag=float(qv - rec['q_mean'][idx]),
                         n=rec['n']))
        node.get_logger().info(
            f"  [{k + 1}/{n_points}] q={rec['q_mean'][idx]:+.4f} "
            f"tau={rec['tau_mean'][idx]:+.4f}+/-{rec['tau_std'][idx]:.4f} "
            f"sag={qv - rec['q_mean'][idx]:+.4f}")

    if len(rows) < 5:
        return dict(joint=joint, ok=False, reason=f"only {len(rows)} points",
                    rows=rows)

    # 拟合都用【实测】角度而不是指令角度：粘滑使实际停位与指令差一个 sag，
    # 用指令角度会把这段差异算进相位，直接污染零位偏置。
    q_meas = np.array([r['q_meas'] for r in rows])
    tau = np.array([r['tau'] for r in rows])
    fit = fit_sine(q_meas, tau)

    ref = model_reference_measured(
        node.model, joint,
        np.array([r['posture_meas'] for r in rows]))
    off = zero_offset(fit['phi'], ref['phi'])
    sags = np.abs([r['sag'] for r in rows])
    return dict(joint=joint, ok=True, rows=rows,
                A=fit['amplitude'], phi=fit['phi'], b=fit['offset'],
                resid=fit['rms_residual'], A_model=ref['amplitude'],
                ratio=(fit['amplitude'] / ref['amplitude']
                       if ref['amplitude'] else 0.0),
                zero_offset_rad=off, zero_offset_deg=float(np.degrees(off)),
                sag_max=float(sags.max()), sag_median=float(np.median(sags)),
                resid_over_A=(float(fit['rms_residual'] / fit['amplitude'])
                              if fit['amplitude'] else 0.0))


def calibrate_joint(node: Calibrator, joint: int, warmup: int,
                    max_sweeps: int, tol_deg: float) -> dict:
    idx = joint - 1
    n = POINTS[joint]
    dwell = DWELL[joint]
    offsets = {}
    sweeps = []
    node.get_logger().info(
        f"===== joint{joint}: {n} points x {dwell}s, "
        f"stiction {STICTION[joint]:.3f} Nm -> deadband "
        f"{np.degrees(STICTION[joint] / HOLD_P[idx]):.2f} deg =====")

    for s in range(warmup + max_sweeps):
        tag = "WARM-UP" if s < warmup else f"sweep {s - warmup + 1}"
        node.get_logger().info(f"--- joint{joint} {tag} ---")
        r = run_sweep(node, joint, offsets, n, dwell)
        r['warmup'] = s < warmup
        r['tag'] = tag
        sweeps.append(r)
        if not r.get('ok'):
            return dict(joint=joint, ok=False, reason=r.get('reason'),
                        sweeps=sweeps)
        node.get_logger().info(
            f"  {tag}: A={r['A']:.4f} ratio={r['ratio']:.3f} "
            f"offset={r['zero_offset_deg']:+.3f}deg "
            f"resid/A={r['resid_over_A']:.3f} sag_max={r['sag_max']:.4f}")
        if r['warmup']:
            continue
        real = [x for x in sweeps if not x['warmup'] and x.get('ok')]
        if len(real) >= 2:
            drift = real[-1]['zero_offset_deg'] - real[-2]['zero_offset_deg']
            if abs(drift) <= tol_deg:
                node.get_logger().info(
                    f"joint{joint} converged: {real[-2]['zero_offset_deg']:+.3f}"
                    f" -> {real[-1]['zero_offset_deg']:+.3f} deg "
                    f"(within {tol_deg:.3f})")
                return dict(joint=joint, ok=True, converged=True,
                            zero_offset_deg=real[-1]['zero_offset_deg'],
                            zero_offset_rad=real[-1]['zero_offset_rad'],
                            ratio=float(np.mean([x['ratio'] for x in real])),
                            sweeps=sweeps)
            node.get_logger().info(
                f"joint{joint} {tag}: {real[-1]['zero_offset_deg']:+.3f} deg "
                f"(prev {real[-2]['zero_offset_deg']:+.3f}, drift {drift:+.3f})"
                f" — not converged, continuing")
    real = [x for x in sweeps if not x['warmup'] and x.get('ok')]
    return dict(joint=joint, ok=bool(real), converged=False,
                zero_offset_deg=real[-1]['zero_offset_deg'] if real else None,
                zero_offset_rad=real[-1]['zero_offset_rad'] if real else None,
                ratio=float(np.mean([x['ratio'] for x in real])) if real else None,
                sweeps=sweeps)


def print_plan(joints: list[int]) -> None:
    model = GravityModel()
    print("=" * 76)
    print("PLAN (dry run — nothing is commanded)")
    print("=" * 76)
    print(f"{'关节':<8}{'点数':>6}{'dwell':>7}{'耗时(min)':>11}{'静摩擦':>9}"
          f"{'死区(°)':>10}{'预期σ(°)':>11}{'ratio':>8}")
    total = 0.0
    for j in joints:
        if j == 6:
            print(f"joint6   —— 无重力信号（link6 质心在转轴上），跳过")
            continue
        n, d = POINTS[j], DWELL[j]
        t = n * (d + 3) / 60
        total += t
        db = np.degrees(STICTION[j] / HOLD_P[j - 1])
        print(f"joint{j:<3}{n:>6}{d:>7.1f}{t:>11.1f}{STICTION[j]:>9.3f}"
              f"{db:>10.2f}{db / np.sqrt(n):>11.3f}{RATIO[j]:>8.3f}")
    print(f"\n单遍合计 {total:.1f} min，含 1 遍热身约 {total * 2:.1f} min")

    print(f"\n各关节位姿安全检查:")
    for j in joints:
        if j == 6:
            continue
        plan = SWEEP_PLAN[j]
        post = np.array(plan['posture'], float)
        bad = 0
        for qv in np.linspace(plan['lo'], plan['hi'], POINTS[j]):
            q = post.copy()
            q[j - 1] = qv
            if not check_setpoint(q, model).ok or desk_margin(model, q) < 0.0:
                bad += 1
        print(f"  joint{j}: {POINTS[j] - bad}/{POINTS[j]} 点安全"
              f"{'' if bad == 0 else '  <-- 有违规点，先修位姿'}")
    print("=" * 76)


def print_verdict(results: list[dict]) -> None:
    print("\n" + "=" * 76)
    print("VERDICT")
    print("=" * 76)
    print(f"{'关节':<8}{'零位偏差(°)':>13}{'ratio':>8}{'resid/A':>9}"
          f"{'sag_max':>9}{'收敛':>7}")
    offsets = [0.0] * N_ARM
    for r in results:
        if not r.get('ok'):
            print(f"joint{r['joint']:<3}  失败: {r.get('reason')}")
            continue
        real = [x for x in r['sweeps'] if not x['warmup'] and x.get('ok')]
        last = real[-1] if real else {}
        offsets[r['joint'] - 1] = r['zero_offset_rad']
        print(f"joint{r['joint']:<3}{r['zero_offset_deg']:>13.3f}"
              f"{r['ratio']:>8.3f}{last.get('resid_over_A', 0):>9.3f}"
              f"{last.get('sag_max', 0):>9.4f}"
              f"{'是' if r.get('converged') else '否':>7}")
    print(f"\n零位偏差向量（实际角 = 命令角 + offset，rad）:")
    print(f"  {[round(v, 6) for v in offsets]}")
    print("\n怎么读:")
    print("  resid/A 只在 A/f > 5 时才指示几何错误。joint4/5 的 A≈f，")
    print("    残差由摩擦主导，resid/A 大是正常的，应改看 ratio 的跨遍复现性。")
    print("  sag_max 应接近摩擦死区 f/kp；若远大于它说明前馈没生效。")
    print("=" * 76)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--joints', default='1,2,3,4,5')
    ap.add_argument('--warmup', type=int, default=1,
                    help='丢弃的热身扫描遍数（首扫必被污染）')
    ap.add_argument('--max-sweeps', type=int, default=3)
    ap.add_argument('--tol-deg', type=float, default=0.4,
                    help='收敛判据：连续两遍零位偏差之差')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    joints = [int(x) for x in args.joints.split(',')]
    if args.dry_run:
        print_plan(joints)
        return 0
    if rclpy is None:
        print(f"rclpy unavailable: {_IMPORT_ERROR}", file=sys.stderr)
        return 2

    OUT.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    rclpy.init()
    node = Calibrator()
    results = []
    try:
        if not node.wait_state():
            print("no /joint_states or torque feedback — start "
                  "student_arm6.launch.py with publish_joint_feedback:=true",
                  file=sys.stderr)
            return 3
        why = node.check_effort_enabled()
        if why:
            print(f"REFUSED: {why}", file=sys.stderr)
            return 4
        node.get_logger().info("effort path confirmed enabled")

        for j in joints:
            if j == 6:
                node.get_logger().info("joint6 has no gravity signal, skipping")
                continue
            r = calibrate_joint(node, j, args.warmup, args.max_sweeps,
                                args.tol_deg)
            results.append(r)
            if not r.get('ok'):
                node.get_logger().error(f"joint{j}: {r.get('reason')}")
                break
    finally:
        path = OUT / f"phase3_calibration_{stamp}.json"
        path.write_text(json.dumps(
            dict(points=POINTS, dwell=DWELL, ratio=RATIO, stiction=STICTION,
                 unidirectional_violations=node.unidirectional_violations,
                 results=results),
            indent=2, default=float))
        print(f"\nresults -> {path}")
        print_verdict(results)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
