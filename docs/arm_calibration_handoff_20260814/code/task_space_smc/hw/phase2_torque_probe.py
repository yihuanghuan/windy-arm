"""力矩通道标定（实物）：测各关节的静摩擦挣脱力矩与力矩指令标度。

两步都在位置保持（kp = HOLD_P）之上叠加单关节力矩前馈 tau_ff 来做：

1. 挣脱力矩（find_breakaway）
   把 tau_ff 按 BREAK_INCR 逐级抬高，直到该关节位移超过 BREAK_MOVED。
   此时的 tau_ff 即静摩擦力矩 f。正负两个方向各测一次。

2. 力矩标度（measure_scale）
   在挣脱值以上线性扫若干点。位置环稳态满足 kp·Δq = alpha·tau_ff，故对
   (tau_ff, q) 做一元线性回归，斜率乘 kp 就是标度：

       alpha = kp · dq/d(tau_ff)

   alpha 是「实际输出力矩 / 指令力矩」，理想为 1；偏离说明力矩常数、
   减速比或传动效率的取值不对。

安全侧：位移超过 2·tau_ff/kp + 0.05 rad 即判为跑飞（该式是位置环在此前馈下
应有的稳态位移的两倍加余量），越过桌面几何同样立即停。
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
    GravityModel, check_setpoint, JOINT_LOWER, JOINT_UPPER, N_ARM,
)

OUT = Path(__file__).resolve().parent / "data" / "phase2"


HOLD_P = [30.0, 30.0, 30.0, 5.0, 5.0, 5.0]
HOLD_D = [1.0, 1.0, 1.0, 0.1, 0.1, 0.1]


LADDER_DM = [0.3, 0.6, 0.9]
LADDER_RS = [0.1, 0.2, 0.3]


PROBE_POSTURE = [-0.046, 1.1538, -1.0466, 0.0033, 0.001, 0.0024]

DISPLACEMENT_ABORT = 0.15
SIGN_GUARD_FACTOR = 3.0
SETTLE_SPAN = 1.5e-3
SETTLE_WIN = 0.6
PUBLISH_RATE = 100.0


# 力矩阶梯：0 → +base 各级 → 回 0 → −base 各级 → 回 0，正反对称往返一圈，
# 用同一组幅值上下各走一遍才能看出迟滞。
def ladder_levels(joint: int) -> list[float]:
    base = LADDER_DM if joint <= 3 else LADDER_RS

    up = [0.0] + base + base[::-1][1:] + [0.0]
    down = [-v for v in base] + [-v for v in base[::-1][1:]] + [0.0]
    return up + down


class Probe(Node):
    def __init__(self, args):
        super().__init__("phase2_torque_probe")
        self.args = args
        self.model = GravityModel()
        self.q = None
        self.v = None
        self.tau = None
        self.pub = self.create_publisher(
            MotorControl, "/student/joint_effort_command", 10)
        self.create_subscription(JointState, "/joint_states", self._cb, 50)

    def _cb(self, m):
        if len(m.position) == N_ARM:
            self.q = np.array(m.position)
        if len(m.velocity) == N_ARM:
            self.v = np.array(m.velocity)
        if len(m.effort) == N_ARM:
            self.tau = np.array(m.effort)

    def check_effort_enabled(self, timeout=5.0) -> str | None:
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
            return ("enable_direct_effort_command is False on the node — "
                    "torque commands would be silently ignored. Relaunch with "
                    "enable_direct_effort_command:=true")
        return None

    def wait_state(self, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout and self.q is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.q is not None

    def send(self, posture, tff, d_gains=None):
        msg = MotorControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [float(v) for v in posture]
        msg.velocity = [0.0] * N_ARM
        msg.current = [float(v) for v in tff]
        msg.p = list(HOLD_P)
        msg.d = list(d_gains) if d_gains is not None else list(HOLD_D)
        msg.enable = 1
        self.pub.publish(msg)

    def guard(self, posture) -> str | None:
        if self.q is None:
            return None
        dq = np.abs(self.q - np.asarray(posture))
        if np.max(dq) > DISPLACEMENT_ABORT:
            j = int(np.argmax(dq)) + 1
            return f"joint{j} displaced {dq.max():.3f} rad > {DISPLACEMENT_ABORT}"
        if self.model.desk_violations(self.q):
            return "desk violation at measured q"
        return None

    def run_level(self, posture, tff, seconds) -> tuple[dict | None, str | None]:
        t0 = time.time()
        win, rows = [], []
        period = 1.0 / PUBLISH_RATE
        nxt = t0
        settled_at = None
        while True:
            now = time.time()
            if now >= nxt:
                self.send(posture, tff)
                nxt += period
            rclpy.spin_once(self, timeout_sec=0.002)
            why = self.guard(posture)
            if why:
                return None, why
            if self.q is None:
                continue
            win.append((now, self.q.copy()))
            win = [(t, v) for t, v in win if now - t <= SETTLE_WIN]
            if settled_at is None:
                if len(win) >= 20 and now - t0 > SETTLE_WIN and \
                        np.max(np.ptp(np.array([v for _, v in win]), axis=0)) <= SETTLE_SPAN:
                    settled_at = now
            else:
                rows.append((self.q.copy(), self.tau.copy()
                             if self.tau is not None else np.zeros(N_ARM)))
                if now - settled_at >= seconds:
                    break
            if now - t0 > 25.0:
                return None, "level did not settle in 25 s"
        Q = np.array([r[0] for r in rows])
        T = np.array([r[1] for r in rows])
        return dict(q_mean=Q.mean(axis=0).tolist(), q_std=Q.std(axis=0).tolist(),
                    tau_mean=T.mean(axis=0).tolist(), n=len(rows)), None


RAMP_STEP = 0.20
RAMP_KD = 10.0
RAMP_SPEED_ABORT = 1.3


GRAV_RATIO = {1: 1.223, 2: 1.445, 3: 1.220, 4: 1.479, 5: 0.920, 6: 1.000}


def gravity_ff(model: GravityModel, q: np.ndarray) -> list[float]:
    g = model.gravity(np.asarray(q, float))
    return [float(g[i] * GRAV_RATIO[i + 1]) for i in range(N_ARM)]

STICTION_NM = {1: 0.725, 2: 1.775, 3: 0.625, 4: 0.125, 5: 0.100, 6: 0.100}
ASSIST_STEP = np.array([max(0.02, 2.0 * STICTION_NM[j] / HOLD_P[j - 1])
                        for j in range(1, N_ARM + 1)])

def desk_margin(model: GravityModel, q: np.ndarray) -> float:
    from task_space_smc.hw.fk_gravity_verify import (
        DESK_PLANE_Z, DESK_EDGE_X, DESK_CLEARANCE)
    m = np.inf
    for p in model.link_points(q):
        if p[0] < DESK_EDGE_X:
            m = min(m, p[2] - (DESK_PLANE_Z + DESK_CLEARANCE))
    return float(m)


def ramp_to(node: Probe, target, max_step=RAMP_STEP, settle=2.0) -> str | None:
    target = np.asarray(target, float)
    period = 1.0 / PUBLISH_RATE
    t0 = time.time()
    nxt = t0
    win_settle = []
    while True:
        now = time.time()
        if node.q is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.time() - t0 > 10.0:
                return "no /joint_states"
            continue
        err = target - node.q


        if np.max(np.abs(err)) < max_step:
            win_settle.append(node.q.copy())
            if len(win_settle) > 60:
                win_settle.pop(0)
                if np.max(np.ptp(np.array(win_settle), axis=0)) < 1e-3:
                    break
        else:
            win_settle.clear()
        step = np.clip(err, -max_step, max_step)
        cmd = node.q + step


        m_now = desk_margin(node.model, node.q)
        m_next = desk_margin(node.model, cmd)
        if m_next < 0.0 and m_next <= m_now:
            return (f"desk margin would not improve "
                    f"({m_now:+.4f} -> {m_next:+.4f} m) on the way to "
                    f"{np.round(target, 3).tolist()}")
        if node.v is not None and np.max(np.abs(node.v)) > RAMP_SPEED_ABORT:
            return (f"ramp speed {np.max(np.abs(node.v)):.2f} rad/s exceeded "
                    f"{RAMP_SPEED_ABORT}")
        if now >= nxt:
            node.send(cmd, gravity_ff(node.model, cmd),
                      d_gains=[RAMP_KD] * N_ARM)
            nxt = now + period
        rclpy.spin_once(node, timeout_sec=0.002)
        if now - t0 > 90.0:
            return (f"ramp timed out, still {np.max(np.abs(err)):.3f} rad away "
                    f"(q={np.round(node.q, 3).tolist()})")

    t1 = time.time()
    while time.time() - t1 < settle:
        node.send(target, [0.0] * N_ARM)
        rclpy.spin_once(node, timeout_sec=0.005)
    return None


ESCAPE_STEPS = [0.20, 0.28, 0.36, 0.45]
ESCAPE_KD = 10.0
ESCAPE_MOVED = 0.02
ESCAPE_SPEED_ABORT = 1.3


def _drive_until_clear(node: Probe, target, idx: int, step: float,
                       timeout: float = 40.0) -> str | None:
    period = 1.0 / PUBLISH_RATE
    t0 = time.time()
    nxt = t0
    best = desk_margin(node.model, node.q)
    stall_since = t0
    clear_count = 0
    while time.time() - t0 < timeout:
        now = time.time()
        if now >= nxt:
            d = np.clip(np.asarray(target, float) - node.q,
                        -ASSIST_STEP, ASSIST_STEP)
            d[idx] = np.clip(target[idx] - node.q[idx], -step, step)
            cmd = node.q + d
            node.send(cmd, gravity_ff(node.model, cmd),
                      d_gains=[ESCAPE_KD] * N_ARM)
            nxt = now + period
        rclpy.spin_once(node, timeout_sec=0.002)
        if node.v is not None and np.max(np.abs(node.v)) > ESCAPE_SPEED_ABORT:
            return f"speed {np.max(np.abs(node.v)):.2f} rad/s exceeded"
        m = desk_margin(node.model, node.q)


        if m > 0.005:
            clear_count += 1
        else:
            clear_count = 0
        if clear_count >= 30:
            node.get_logger().info(
                f"  clear of the desk (margin {m:+.4f} m sustained), braking")
            t1 = time.time()
            while time.time() - t1 < 0.8:
                node.send(node.q, gravity_ff(node.model, node.q),
                          d_gains=[ESCAPE_KD] * N_ARM)
                rclpy.spin_once(node, timeout_sec=0.005)
            return None
        if m > best + 1e-4:
            best, stall_since = m, now
        elif now - stall_since > 6.0:
            return (f"stalled at margin {m:+.4f} m under {30 * step:.1f} Nm "
                    f"for 6 s")
    return f"drive-until-clear timed out at margin {best:+.4f} m"


def escape_desk(node: Probe, target, joint: int = 2) -> tuple[str | None, list]:
    idx = joint - 1
    period = 1.0 / PUBLISH_RATE
    log = []
    for step in ESCAPE_STEPS:
        q_start = node.q.copy()
        m_start = desk_margin(node.model, q_start)
        t0 = time.time()
        nxt = t0
        moved = 0.0
        while time.time() - t0 < 3.0:
            now = time.time()
            if now >= nxt:
                cmd = node.q.copy()

                d = np.clip(np.asarray(target, float) - node.q,
                            -ASSIST_STEP, ASSIST_STEP)
                d[idx] = np.clip(target[idx] - node.q[idx], -step, step)
                cmd = node.q + d
                node.send(cmd, gravity_ff(node.model, cmd),
                          d_gains=[ESCAPE_KD] * N_ARM)
                nxt = now + period
            rclpy.spin_once(node, timeout_sec=0.002)
            if node.v is not None and np.max(np.abs(node.v)) > ESCAPE_SPEED_ABORT:
                node.send(node.q, [0.0] * N_ARM, d_gains=[ESCAPE_KD] * N_ARM)
                return (f"speed {np.max(np.abs(node.v)):.2f} rad/s exceeded "
                        f"{ESCAPE_SPEED_ABORT} at step {step}"), log
            moved = abs(node.q[idx] - q_start[idx])
            if moved > ESCAPE_MOVED:
                break
        m_now = desk_margin(node.model, node.q)
        log.append(dict(step=float(step), torque_nm=float(30.0 * step),
                        moved_rad=float(moved), margin_before=m_start,
                        margin_after=m_now))
        node.get_logger().info(
            f"  escape step {step:.2f} ({30 * step:.1f} Nm): "
            f"joint{joint} moved {moved:+.4f} rad, "
            f"margin {m_start:+.4f} -> {m_now:+.4f} m")
        if m_now > 0.0:
            node.get_logger().info("  clear of the desk")
            return None, log
        if moved > ESCAPE_MOVED:
            node.get_logger().info(
                f"  moving at {30 * step:.1f} Nm — continuing until clear")


            why = _drive_until_clear(node, target, idx, step)
            log.append(dict(step=float(step), phase="drive_until_clear",
                            margin_after=desk_margin(node.model, node.q)))
            return why, log
    return (f"still stuck after {30 * ESCAPE_STEPS[-1]:.1f} Nm "
            f"({100 * 30 * ESCAPE_STEPS[-1] / 28:.0f}% of rated)"), log


BREAK_MAX = {1: 3.5, 2: 3.5, 3: 3.5, 4: 1.5, 5: 1.5, 6: 1.5}
BREAK_INCR = {1: 0.10, 2: 0.10, 3: 0.10, 4: 0.05, 5: 0.05, 6: 0.05}
BREAK_MOVED = 5.0e-3
BREAK_DWELL = 1.2


def find_breakaway(node: Probe, joint: int, sign: int) -> dict:
    idx = joint - 1
    kp = HOLD_P[idx]
    posture = list(PROBE_POSTURE)
    incr, tmax = BREAK_INCR[joint], BREAK_MAX[joint]
    period = 1.0 / PUBLISH_RATE

    base, why = node.run_level(posture, [0.0] * N_ARM, 1.0)
    if why or base is None:
        return dict(sign=sign, ok=False, reason=why or "no baseline")
    q0 = base["q_mean"][idx]

    trace = []
    c = 0.0
    while c < tmax - 1e-9:
        c = min(c + incr, tmax)
        tff = [0.0] * N_ARM
        tff[idx] = sign * c
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < BREAK_DWELL:
            now = time.time()
            if now >= nxt:
                node.send(posture, tff)
                nxt = now + period
            rclpy.spin_once(node, timeout_sec=0.002)
            if node.q is None:
                continue

            # 位置环在 tau_ff = c 下的稳态位移是 c/kp，取两倍再加 0.05 rad 余量
            # 作为自限位；超过它说明关节已经跑飞，不是正常的稳态偏移。
            allow = 2.0 * c / kp + 0.05
            if abs(node.q[idx] - q0) > allow:
                node.run_level(posture, [0.0] * N_ARM, 0.5)
                return dict(sign=sign, ok=False, trace=trace,
                            reason=f"joint{joint} ran past the self-limit "
                                   f"({abs(node.q[idx] - q0):.4f} > {allow:.4f} rad) "
                                   f"at {sign * c:+.2f} Nm")
            if node.model.desk_violations(node.q):
                node.run_level(posture, [0.0] * N_ARM, 0.5)
                return dict(sign=sign, ok=False, trace=trace,
                            reason="desk violation")
        dq = node.q[idx] - q0
        trace.append(dict(tff=float(sign * c), dq=float(dq),
                          tau=float(node.tau[idx]) if node.tau is not None else 0.0))
        if abs(dq) > BREAK_MOVED:
            node.get_logger().info(
                f"  joint{joint} {'+' if sign > 0 else '-'} broke away at "
                f"{c:.2f} Nm (dq={dq:+.4f} rad)")
            node.run_level(posture, [0.0] * N_ARM, 1.0)
            return dict(sign=sign, ok=True, breakaway_nm=float(c),
                        dq_at_break=float(dq), trace=trace)
    node.run_level(posture, [0.0] * N_ARM, 1.0)
    return dict(sign=sign, ok=False, trace=trace,
                reason=f"no motion up to {tmax:.1f} Nm — stiction exceeds it")


def measure_scale(node: Probe, joint: int, t_break: float, sign: int,
                  n_points: int = 6) -> dict:
    idx = joint - 1
    kp = HOLD_P[idx]
    posture = list(PROBE_POSTURE)
    tmax = BREAK_MAX[joint]
    lo = min(t_break * 1.15, tmax)
    if lo >= tmax - 1e-9:
        return dict(ok=False, reason=f"breakaway {t_break:.2f} Nm leaves no "
                                     f"headroom below the {tmax:.1f} Nm cap")
    levels = np.linspace(lo, tmax, n_points)
    pts = []
    for c in levels:
        tff = [0.0] * N_ARM
        tff[idx] = sign * float(c)
        rec, why = node.run_level(posture, tff, 1.5)
        if why or rec is None:
            break
        pts.append((sign * float(c), rec["q_mean"][idx]))
    node.run_level(posture, [0.0] * N_ARM, 1.0)
    if len(pts) < 3:
        return dict(ok=False, reason=f"only {len(pts)} usable points")
    # 回归 q = (alpha/kp)·tau_ff + b，斜率乘 kp 得标度 alpha；
    # se_slope 是斜率的标准误，同样乘 kp 传播成 alpha 的标准误。
    cs = np.array([p[0] for p in pts])
    qs = np.array([p[1] for p in pts])
    A = np.column_stack([cs, np.ones_like(cs)])
    coef, *_ = np.linalg.lstsq(A, qs, rcond=None)
    resid = qs - A @ coef
    n = len(cs)
    se_slope = float(np.sqrt(np.sum(resid ** 2) / max(n - 2, 1) /
                             max(np.sum((cs - cs.mean()) ** 2), 1e-12)))
    return dict(ok=True, alpha=float(coef[0] * kp),
                alpha_stderr=float(se_slope * kp),
                slope_rad_per_nm=float(coef[0]), kp_assumed=kp,
                rms_resid_rad=float(np.sqrt(np.mean(resid ** 2))),
                n_points=n, t_range=[float(cs.min()), float(cs.max())],
                points=[[float(a), float(b)] for a, b in pts])


def return_joint(node: Probe, joint: int, tol: float = 0.02,
                 timeout: float = 20.0) -> str | None:
    idx = joint - 1
    period = 1.0 / PUBLISH_RATE
    target = np.asarray(PROBE_POSTURE, float)
    t0 = time.time()
    nxt = t0
    while time.time() - t0 < timeout:
        now = time.time()
        if node.q is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            continue
        err = target[idx] - node.q[idx]
        if abs(err) < tol:
            t1 = time.time()
            while time.time() - t1 < 1.0:
                node.send(target, [0.0] * N_ARM)
                rclpy.spin_once(node, timeout_sec=0.005)
            if abs(target[idx] - node.q[idx]) < tol * 2:
                return None
            t0 = time.time()
            continue
        if now >= nxt:
            cmd = node.q.copy()
            cmd[idx] = node.q[idx] + np.clip(err, -RAMP_STEP, RAMP_STEP)
            node.send(cmd, [0.0] * N_ARM)
            nxt = now + period
        rclpy.spin_once(node, timeout_sec=0.002)
    return (f"joint{joint} did not return to the posture "
            f"({target[idx] - node.q[idx]:+.4f} rad off)")


def run_breakaway(node: Probe, joint: int) -> dict:
    out = {}
    for sign in (+1, -1):
        node.get_logger().info(
            f"===== breakaway joint{joint} {'+' if sign > 0 else '-'} =====")
        out['pos' if sign > 0 else 'neg'] = find_breakaway(node, joint, sign)
    p, n = out['pos'], out['neg']


    cands = [(v['breakaway_nm'], sg) for v, sg in ((p, +1), (n, -1))
             if v.get('ok')]
    if cands:
        tb, sg = min(cands)
        node.get_logger().info(
            f"===== scale joint{joint} ({'+' if sg > 0 else '-'}, "
            f"above {tb:.2f} Nm) =====")
        out['scale'] = measure_scale(node, joint, tb, sg)
        if out['scale'].get('ok'):
            node.get_logger().info(
                f"joint{joint}: alpha = {out['scale']['alpha']:.4f} "
                f"+/- {out['scale']['alpha_stderr']:.4f}")
        else:
            node.get_logger().warn(
                f"joint{joint} scale: {out['scale'].get('reason')}")


    why = return_joint(node, joint)
    if why:
        node.get_logger().warn(why)
        out['return_warning'] = why
    if p.get("ok") and n.get("ok"):


        out['stiction_nm'] = (p['breakaway_nm'] + n['breakaway_nm']) / 2.0
        out['residual_nm'] = (n['breakaway_nm'] - p['breakaway_nm']) / 2.0
        out['deadband_rad'] = out['stiction_nm'] / HOLD_P[joint - 1]
    return out


def run_ladder(node: Probe, joint: int, dwell: float) -> dict:
    idx = joint - 1
    posture = list(PROBE_POSTURE)
    levels = ladder_levels(joint)
    kp = HOLD_P[idx]
    records = []


    base, why = node.run_level(posture, [0.0] * N_ARM, dwell)
    if why or base is None:
        return dict(joint=joint, ok=False, reason=why or "no baseline")
    q0 = base["q_mean"][idx]
    node.get_logger().info(f"joint{joint} baseline q = {q0:+.4f}")

    first_nonzero_checked = False
    for c in levels:
        tff = [0.0] * N_ARM
        tff[idx] = c
        rec, why = node.run_level(posture, tff, dwell)
        if why or rec is None:

            node.run_level(posture, [0.0] * N_ARM, 0.5)
            return dict(joint=joint, ok=False, reason=why or "level failed",
                        records=records)
        dq = rec["q_mean"][idx] - q0
        records.append(dict(c=c, dq=dq, q=rec["q_mean"], tau=rec["tau_mean"],
                            n=rec["n"]))
        node.get_logger().info(
            f"  c={c:+.2f} Nm -> dq={dq:+.5f} rad "
            f"(expect ~{c / kp:+.5f} if alpha=1)")
        if c != 0.0 and not first_nonzero_checked:
            first_nonzero_checked = True
            expect = c / kp
            if dq * expect < 0 or abs(dq) > SIGN_GUARD_FACTOR * abs(expect):
                node.run_level(posture, [0.0] * N_ARM, 0.5)
                return dict(joint=joint, ok=False, records=records,
                            reason=f"sign/scale guard: dq={dq:+.5f} vs "
                                   f"expected {expect:+.5f}")

    cs = np.array([r["c"] for r in records])
    dqs = np.array([r["dq"] for r in records])


    taus_chk = np.array([r["tau"][idx] for r in records])
    if np.ptp(taus_chk) < 1e-6:
        return dict(joint=joint, ok=False, records=records,
                    reason="the torque feedback did not move at all across the "
                           "whole ladder — the command is not reaching the "
                           "motor, or this joint's feedback is frozen")

    A = np.column_stack([cs, np.ones_like(cs)])


    taus = np.array([r["tau"][idx] for r in records])
    coef_t, *_ = np.linalg.lstsq(A, taus, rcond=None)
    alpha = float(coef_t[0])
    resid_t = taus - A @ coef_t
    n = len(cs)
    se = float(np.sqrt(np.sum(resid_t ** 2) / max(n - 2, 1) /
                       max(np.sum((cs - cs.mean()) ** 2), 1e-12)))


    coef_d, *_ = np.linalg.lstsq(A, dqs, rcond=None)
    alpha_disp = float(coef_d[0] * kp)
    resid_d = dqs - A @ coef_d

    return dict(joint=joint, ok=True,
                alpha=alpha, alpha_stderr=se,
                alpha_ci95=[alpha - 1.96 * se, alpha + 1.96 * se],
                rms_resid_nm=float(np.sqrt(np.mean(resid_t ** 2))),
                alpha_displacement=alpha_disp,
                slope_rad_per_nm=float(coef_d[0]),
                rms_resid_rad=float(np.sqrt(np.mean(resid_d ** 2))),
                max_abs_dq=float(np.max(np.abs(dqs))),
                stiction_lower_bound_nm=(
                    float(np.max(np.abs(cs))) if np.max(np.abs(dqs)) < 3e-3
                    else None),
                kp_assumed=kp, records=records)


def print_plan(args):
    model = GravityModel()
    print("=" * 72)
    print("PLAN (dry run — nothing is commanded)")
    print("=" * 72)
    q = np.array(PROBE_POSTURE)
    rep = check_setpoint(q, model)
    print(f"测试位姿 {PROBE_POSTURE}")
    print(f"  安全检查: {'通过' if rep.ok else 'REFUSED: ' + '; '.join(rep.reasons)}")
    if not rep.ok:
        print("  -> 位姿被拒，不会执行任何命令")
    print(f"  限位余量: {np.round(np.minimum(q - JOINT_LOWER, JOINT_UPPER - q), 3).tolist()}")
    g = model.gravity(q)
    print(f"  该位姿重力力矩: {np.round(g, 3).tolist()} Nm")
    for j in [int(x) for x in args.joints.split(',')]:
        lv = ladder_levels(j)
        kp = HOLD_P[j - 1]
        mx = max(abs(v) for v in lv)
        print(f"\n  joint{j}: 阶梯 {lv}")
        print(f"    t_ff 上限 {mx} Nm，预期最大位移 {mx / kp:.4f} rad "
              f"(守卫 {DISPLACEMENT_ABORT})")
        print(f"    级数 {len(lv)}，每级稳定后采样 {args.dwell:g}s，"
              f"预计 {len(lv) * (args.dwell + 3) / 60:.1f} min")
    print("\n判据: alpha = kp × slope ∈ [0.85, 1.15] 且符号守卫不触发")
    print("=" * 72)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--joints', default='2', help='逗号分隔，如 2 或 1,2,3,4,5,6')
    ap.add_argument('--dwell', type=float, default=2.0, help='每级稳定后采样秒数')
    ap.add_argument('--posture', default=None,
                    help='逗号分隔的 6 个角度，覆盖默认测试位姿')
    ap.add_argument('--breakaway', action='store_true',
                    help='测启动力矩（静摩擦）而非标度阶梯')
    ap.add_argument('--goto-only', action='store_true',
                    help='只移动到测试位姿，不做任何力矩注入')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    global PROBE_POSTURE
    if args.posture:
        vals = [float(v) for v in args.posture.split(',')]
        if len(vals) != N_ARM:
            print(f"--posture needs {N_ARM} values, got {len(vals)}", file=sys.stderr)
            return 2
        PROBE_POSTURE = vals

    if args.dry_run:
        print_plan(args)
        return 0
    if rclpy is None:
        print(f"rclpy unavailable: {_IMPORT_ERROR}", file=sys.stderr)
        return 2

    OUT.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    rclpy.init()
    node = Probe(args)
    results = []
    try:
        if not node.wait_state():
            print("no /joint_states — start student_arm6.launch.py with "
                  "enable_direct_effort_command:=true first", file=sys.stderr)
            return 3

        rep = check_setpoint(np.array(PROBE_POSTURE), node.model)
        if not rep.ok:
            print(f"probe posture REFUSED: {'; '.join(rep.reasons)}", file=sys.stderr)
            return 4
        if desk_margin(node.model, node.q) < 0.0:
            node.get_logger().warn(
                f"arm is intruding on the desk "
                f"(margin {desk_margin(node.model, node.q):+.4f} m) — escaping")
            why, esc_log = escape_desk(node, PROBE_POSTURE)
            for r in esc_log:
                node.get_logger().info(f"    {r}")
            if why:
                print(f"could not escape the desk: {why}", file=sys.stderr)
                return 6
        dq0 = np.abs(node.q - np.array(PROBE_POSTURE))
        if np.max(dq0) > 5e-3:
            node.get_logger().info(
                f"moving to the probe posture ({np.max(dq0):.3f} rad away)")
            why = ramp_to(node, PROBE_POSTURE)
            if why:
                print(f"could not reach the probe posture: {why}", file=sys.stderr)
                return 4
            node.get_logger().info(
                f"at probe posture: {np.round(node.q, 4).tolist()}")
        why = node.check_effort_enabled()
        if why:
            print(f"REFUSED: {why}", file=sys.stderr)
            return 5
        node.get_logger().info("effort path confirmed enabled on the node")
        if args.goto_only:
            node.get_logger().info("goto-only: no torque will be injected")
            return 0
        for j in [int(x) for x in args.joints.split(',')]:
            if args.breakaway:
                r = run_breakaway(node, j)
                r['joint'] = j
                r['ok'] = 'stiction_nm' in r
                results.append(r)
                if r['ok']:
                    node.get_logger().info(
                        f"joint{j}: stiction {r['stiction_nm']:.2f} Nm, "
                        f"deadband ±{r['deadband_rad']:.4f} rad, "
                        f"residual {r['residual_nm']:+.2f} Nm")
                continue
            node.get_logger().info(f"===== ladder joint{j} =====")
            r = run_ladder(node, j, args.dwell)
            results.append(r)
            if r.get("ok"):
                node.get_logger().info(
                    f"joint{j}: alpha_torque = {r['alpha']:.4f}"
                    f" +/- {r['alpha_stderr']:.4f}, "
                    f"alpha_disp = {r['alpha_displacement']:.4f}, "
                    f"max|dq| = {r['max_abs_dq']:.5f} rad")
            else:
                node.get_logger().error(f"joint{j}: {r.get('reason')}")
                break

        node.run_level(PROBE_POSTURE, [0.0] * N_ARM, 1.0)
    finally:
        path = OUT / f"phase2_ladder_{stamp}.json"
        path.write_text(json.dumps(dict(posture=PROBE_POSTURE, results=results),
                                   indent=2, default=float))
        print(f"\nresults -> {path}")
        print(f"\n{'joint':<7} {'α_torque':>8} {'±1σ':>8}  {'max|dq|':>12}  说明")
        print("  (α_torque = 前馈未被运动吸收的比例，非 0x12 标度；"
              "标度见 --breakaway)")
        for r in results:
            if args.breakaway:
                if r.get('ok'):
                    print(f"joint{r['joint']:<3} 静摩擦 {r['stiction_nm']:>5.2f} Nm  "
                          f"死区 ±{r['deadband_rad']:.4f} rad "
                          f"(±{np.degrees(r['deadband_rad']):.2f}°)  "
                          f"残差 {r['residual_nm']:+.2f} Nm" +
                          (f"  α={r['scale']['alpha']:.4f}"
                           if r.get('scale', {}).get('ok') else ''))
                else:
                    for k in ('pos', 'neg'):
                        if k in r and not r[k].get('ok'):
                            print(f"joint{r['joint']:<3} {k}: {r[k].get('reason')}")
                continue
            if not r.get("ok"):
                print(f"joint{r['joint']:<3} {'—':>8} ✗ {r.get('reason')}")
                continue


            print(f"joint{r['joint']:<3} {r['alpha']:>8.4f} "
                  f"±{r['alpha_stderr']:.4f}  "
                  f"位移 {r['max_abs_dq']:.5f} rad  "
                  f"{'几乎不动(摩擦主导)' if r['max_abs_dq'] < 3e-3 else '有移动'}")
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
