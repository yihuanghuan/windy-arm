#!/usr/bin/env python3
"""阶段 2：0x12 力矩通路验证（首次向实机发送力矩前馈）。

两种测法，位置环全程以默认增益兜底：

ladder（默认，对全部 6 关节有效）
    在固定位姿向单个关节注入恒定 t_ff = c，静平衡移动 Δq = α·c/kp。
    kp 已被重力数据隐式验证（实测 τ_g/下垂 = 2.2/0.073 = 30.1 ≈ 命令值 30），
    因此斜率 Δq/c 直接给出 0x12 的标度 α —— 不依赖重力模型，
    对重力恒零的 joint6 同样有效。阶梯对称（0→+max→0→−max→0），
    每级从两个方向各测一次，摩擦滞回在平均中抵消。

ksweep（交叉验证，joint1~3）
    t_ff = k·ĝ_model(q)，扫 k 0→1.8→0。下垂 ∝ (ratio − α·k)，
    过零点 k* = ratio/α。ratio 已实测（joint1 1.223 / joint2 1.41~1.48 /
    joint3 1.22），k* 落在实测 ratio ±15% 即证明 α ≈ 1。

安全设计
    - 位置环全程保持默认 kp/kd，任何时刻撤掉 t_ff 都回到位置保持
    - t_ff 上限：DM 关节 0.9 Nm、RS05 关节 0.3 Nm（额定的 10%/19%）
    - 位移守卫：任何关节偏离位姿 >0.15 rad 立即归零中止
    - 符号守卫：首个非零级的位移若与预期反向或超预期 3 倍，立即归零中止
    - 桌面约束：实测 q 每周期检查
    - 命令流 ≥50 Hz 连续发布；节点侧兜底已改为位置保持（不再垮臂），
      心跳断链时节点自动拦下非零前馈
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
except ImportError as exc:  # 离线 dry-run 仍可用
    rclpy = None
    _IMPORT_ERROR = exc

# 按路径直接调用本脚本时，其所在目录会被前置到 sys.path，使得包名
# task_space_smc 解析到同目录下的 task_space_smc.py 而非包本身。
# 把包的父目录插到最前面，与 merge_fk_verify.py 的做法一致。
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from task_space_smc.fk_gravity_verify import (  # noqa: E402
    GravityModel, check_setpoint, JOINT_LOWER, JOINT_UPPER, N_ARM,
)

OUT = Path(__file__).resolve().parent / "data" / "phase2"

# 与 student_arm_node 的 p_gain/d_gain 默认一致（兜底也用它们）
HOLD_P = [30.0, 30.0, 30.0, 5.0, 5.0, 5.0]
HOLD_D = [1.0, 1.0, 1.0, 0.1, 0.1, 0.1]

# 阶梯的 t_ff 电平（Nm 命令值）。上限取额定的 10%（DM）/19%（RS05）——
# 位移预期 c/kp ≤ 0.03/0.06 rad，远小于 0.15 rad 守卫。
LADDER_DM = [0.3, 0.6, 0.9]
LADDER_RS = [0.1, 0.2, 0.3]

# 默认测试位姿。选它的依据（不是上电位姿 —— 那个 joint3 距下限只有 0.204 rad，
# 不足以容纳 3.5 Nm 上限对应的 0.117 rad 位移）：
#   - 各关节限位余量 / 最大位移 ≥ 4.5 倍
#   - 桌面余量 +0.086 m，且能吸收 0.35 rad 的下垂
#   - joint1~3 重力不为零，便于观察
# 阶梯法测的是斜率，与位姿基本无关，只要关节不在限位附近且位姿全程稳定。
PROBE_POSTURE = [-0.046, 1.1538, -1.0466, 0.0033, 0.001, 0.0024]

DISPLACEMENT_ABORT = 0.15     # rad，任何关节偏离位姿超过它立即中止
SIGN_GUARD_FACTOR = 3.0       # 首级位移超预期该倍数即中止
SETTLE_SPAN = 1.5e-3
SETTLE_WIN = 0.6
PUBLISH_RATE = 100.0          # Hz，须远高于 0.05s 超时


def ladder_levels(joint: int) -> list[float]:
    base = LADDER_DM if joint <= 3 else LADDER_RS
    # 0 -> +全部 -> 0 -> -全部 -> 0，每级双向覆盖
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
        """确认 student_arm_node 的力矩通路确实开着。

        launch 里这个参数一度是硬编码 False，命令行传 true 被静默忽略，
        导致整轮阶梯跑完、dq 全为 0、却被报告成 "alpha=0 标度异常"——
        看起来像测量结论，其实是命令从未送达。开测前必须查一次。
        """
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
        """保持该电平至稳定并采样。返回 (记录, 中止原因)。"""
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


# joint2 在下垂位姿光重力就要 3.08 Nm (模型 2.13 × 实测 ratio 1.445)，
# 所以步长必须给出【显著超过】重力的力矩，否则净提升力被接触摩擦吃掉。
# 0.20 -> 6.0 Nm，净 2.9 Nm；实测移动速度 0.1~0.3 rad/s，安全。
RAMP_STEP = 0.20
RAMP_KD = 10.0            # 高阻尼把速度压住：v = 净力矩/kd
RAMP_SPEED_ABORT = 1.3    # rad/s



# 非目标关节的步长必须让每个关节都【超过自己的静摩擦】才推得动。
# 统一用 0.02 rad 时 joint3 只得到 30*0.02=0.6 Nm，而它的静摩擦是 0.625 Nm
# —— 刚好推不动，脱离过程因此卡在余量 -0.012 m 处。取 2 倍静摩擦对应的步长。

# 阶段 2 实测的一阶质量矩比值，用于脱离时的重力前馈
GRAV_RATIO = {1: 1.223, 2: 1.445, 3: 1.220, 4: 1.479, 5: 0.920, 6: 1.000}


def gravity_ff(model: GravityModel, q: np.ndarray) -> list[float]:
    """脱离/转移时的重力前馈 = ratio x g_model。

    不加前馈时，位置误差必须同时克服【重力 + 静摩擦】：实机上 joint2 需要
    |4.776| + 1.775 = 6.55 Nm，而脱离阶梯只给 6.00 Nm —— 差一点点，于是卡在
    余量 -0.014 m 处推不动。加前馈后只需克服静摩擦，6 Nm 绰绰有余。
    0x12 标度已由阶段 2 验证（alpha = 1.033 +/- 0.024）。
    """
    g = model.gravity(np.asarray(q, float))
    return [float(g[i] * GRAV_RATIO[i + 1]) for i in range(N_ARM)]

STICTION_NM = {1: 0.725, 2: 1.775, 3: 0.625, 4: 0.125, 5: 0.100, 6: 0.100}
ASSIST_STEP = np.array([max(0.02, 2.0 * STICTION_NM[j] / HOLD_P[j - 1])
                        for j in range(1, N_ARM + 1)])

def desk_margin(model: GravityModel, q: np.ndarray) -> float:
    """最小桌面余量（m）。负值 = 已侵入安全余量。"""
    from task_space_smc.fk_gravity_verify import (
        DESK_PLANE_Z, DESK_EDGE_X, DESK_CLEARANCE)
    m = np.inf
    for p in model.link_points(q):
        if p[0] < DESK_EDGE_X:
            m = min(m, p[2] - (DESK_PLANE_Z + DESK_CLEARANCE))
    return float(m)


def ramp_to(node: Probe, target, max_step=RAMP_STEP, settle=2.0) -> str | None:
    """经力矩通道把机械臂移到 target（零前馈 + 默认增益 = 纯位置保持）。

    启用力矩通路后 ControlLoop 在 effort 分支直接 return，位置路径
    (/student/joint_command) 被旁路，所以移动也必须走这条通道。
    每步重新从【实测】位置规划，不用预先算好的路径——预算路径会与
    实际下垂脱节（这是位置模式扫描时踩过的坑）。
    """
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
        # 收敛判据不能是"实测 == 目标"：位置环的 kp 下垂使实测【永远】
        # 停在目标下方 gravity/kp 处（joint2 约 0.10 rad），那个判据一定超时。
        # 正确判据是"胡萝卜已经不再被限幅（即直接命令目标）且位置稳定"。
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
        # 脱离模式：起点可能已经侵入（机械臂垂到桌上）。此时绝对判据会
        # 拒绝每一步、把机械臂锁死在违规位置，所以改用【严格改善】判据：
        # 安全的步照常放行，仍侵入但余量增大的步也放行，只拒绝让情况变坏的。
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
    # 到位后按住 target 让它稳定（kp 下垂会使实测停在 target 附近而非其上）
    t1 = time.time()
    while time.time() - t1 < settle:
        node.send(target, [0.0] * N_ARM)
        rclpy.spin_once(node, timeout_sec=0.005)
    return None


# 脱离阶梯：机械臂垂到桌面被托住时，位置环的常规步长不足以抬起。
# 需要克服的是【重力】而非仅摩擦——joint2 在下垂位姿的重力约 2.6 Nm
# (模型 1.8 × 实测 ratio 1.445)，所以 3.6 Nm 的指令只剩 1.0 Nm 净提升力，
# 被接触摩擦吃掉。逐级加力，一旦动起来立即降回温和模式。
ESCAPE_STEPS = [0.20, 0.28, 0.36, 0.45]   # kp=30 -> 6.0 / 8.4 / 10.8 / 13.5 Nm
ESCAPE_KD = 10.0          # 脱离瞬间的阻尼：v = 净力矩/kd，压到 <1 rad/s
ESCAPE_MOVED = 0.02       # rad，判定"动了"
ESCAPE_SPEED_ABORT = 1.3  # rad/s（最高一级的稳态速度 1.04，留余量）


def _drive_until_clear(node: Probe, target, idx: int, step: float,
                       timeout: float = 40.0) -> str | None:
    """维持脱离力矩驱动，直到桌面余量转正（或到达目标）。"""
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
        # 连续确认：单帧噪声曾把 margin 报成 +0.0054，实际是 -0.0551，
        # 导致过早交接、随即再次卡住。要求连续 30 帧(约 0.3s)都为正。
        if m > 0.005:
            clear_count += 1
        else:
            clear_count = 0
        if clear_count >= 30:
            node.get_logger().info(
                f"  clear of the desk (margin {m:+.4f} m sustained), braking")
            t1 = time.time()
            while time.time() - t1 < 0.8:   # 高阻尼刹住再交接
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
    """逐级加大力矩把机械臂从桌面抬起。返回 (失败原因, 每级记录)。"""
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
                # 只在需要抬起的那个关节上加大步长，其余关节原地保持
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
            # 关键：必须保持这个力矩【直到真正脱离】。降回温和模式(3.6 Nm)
            # 会立刻再次卡住 —— joint2 光重力就要 3.08 Nm，温和模式只剩
            # 0.5 Nm 净提升力，被接触摩擦吃掉。
            why = _drive_until_clear(node, target, idx, step)
            log.append(dict(step=float(step), phase="drive_until_clear",
                            margin_after=desk_margin(node.model, node.q)))
            return why, log
    return (f"still stuck after {30 * ESCAPE_STEPS[-1]:.1f} Nm "
            f"({100 * 30 * ESCAPE_STEPS[-1] / 28:.0f}% of rated)"), log


# 启动力矩搜索的上限。位移自限于 t_ff/kp，所以这些值决定最大位移：
# DM 3.5/30 = 0.117 rad，RS 0.6/5 = 0.120 rad。
# RS05 关节 (4/5/6) 额定 6 Nm，kp=5。首轮扫到 0.6 Nm (10% 额定) 时 joint4/5
# 都没起步，说明静摩擦更高，故提到 1.5 Nm (25% 额定)，位移自限 0.30 rad ——
# 这三个关节的限位余量 3.997/1.999/3.998 都够，且它们对桌面最低点无影响。
BREAK_MAX = {1: 3.5, 2: 3.5, 3: 3.5, 4: 1.5, 5: 1.5, 6: 1.5}
BREAK_INCR = {1: 0.10, 2: 0.10, 3: 0.10, 4: 0.05, 5: 0.05, 6: 0.05}
BREAK_MOVED = 5.0e-3      # rad，约 13 个编码器 LSB，远高于噪声
BREAK_DWELL = 1.2         # s，每级保持


def find_breakaway(node: Probe, joint: int, sign: int) -> dict:
    """沿 sign 方向逐级加大 t_ff，直到关节起步，返回启动力矩。

    位置环全程在位：加了 t_ff 后新的静平衡在 dq = t_ff/kp，位移【自限】，
    不会失控，所以不必失力也能测。这比原方案的 kp=kd=0 死区法安全得多。
    """
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
            # 位移会自限在 t_ff/kp，所以守卫按它自适应，而不是固定阈值
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
    """在【启动力矩以上】扫描 t_ff，用位移斜率定 0x12 的标度 α。

    关节一旦在动，新的静平衡满足  kp·dq = α·t_ff − r − f_kinetic。
    r 和 f_kinetic 都是常数，所以【斜率】 d(dq)/d(t_ff) = α/kp 与它们无关 ——
    这正是这个测法优于任何单点测量的地方。

    kp 本身已被重力数据独立验证（实测 τ_g/下垂 = 30.1 ≈ 命令值 30）。
    """
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
    """把单个关节确实带回测试位姿。

    单纯发一次 run_level(posture, 0) 不够：它的稳定判据（0.6s 窗口内
    ptp < 1.5e-3）会把【缓慢回位】误判成已稳定，于是关节停在半路。
    joint4 就这样停在离位姿 0.161 rad 处，触发了下一个关节的位移守卫，
    导致 joint5/6 整个测不成。这里用胡萝卜逼近并【验证】到位。
    """
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
    """双向测启动力矩。中点=重力平衡点，半宽=静摩擦。"""
    out = {}
    for sign in (+1, -1):
        node.get_logger().info(
            f"===== breakaway joint{joint} {'+' if sign > 0 else '-'} =====")
        out['pos' if sign > 0 else 'neg'] = find_breakaway(node, joint, sign)
    p, n = out['pos'], out['neg']
    # 标度测量选启动力矩【较小】的方向 —— 它到上限的可扫区间更长，
    # 斜率拟合的杠杆更大。
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
    # 关键：测完必须把该关节【确实】带回位姿，否则它残留的位移会触发
    # 下一个关节测量的全局位移守卫。
    why = return_joint(node, joint)
    if why:
        node.get_logger().warn(why)
        out['return_warning'] = why
    if p.get("ok") and n.get("ok"):
        # f = 半和 = 静摩擦（可靠）
        # r = 半差 = 静止时的未平衡残差；它受【进入方向】影响（关节停在
        #     摩擦带内何处），混合了重力模型误差，不能单独当作重力读数。
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

    # 基线（c=0）
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
            # 立即回零并中止
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

    # 位移【精确】全零意味着连一个编码器 LSB (3.834e-4 rad) 都没动过。
    # 真实的"标度为 0"不会这么干净：位置反馈总有 LSB 级抖动。这是管路
    # 不通（命令未送达 / 反馈冻结），必须与真正的测量失败区分开。
    # 位移全零【不】代表失败：静摩擦可能大于整个阶梯的幅度，而力矩反馈法
    # 依然能定出标度。真正的管路故障要看【力矩反馈】是否也毫无响应。
    taus_chk = np.array([r["tau"][idx] for r in records])
    if np.ptp(taus_chk) < 1e-6:
        return dict(joint=joint, ok=False, records=records,
                    reason="the torque feedback did not move at all across the "
                           "whole ladder — the command is not reaching the "
                           "motor, or this joint's feedback is frozen")

    A = np.column_stack([cs, np.ones_like(cs)])

    # 【主判据】力矩反馈法：alpha = d(反馈力矩)/d(命令力矩)。
    # MIT 模式下 tau = kp(p_des-p) + kd(...) + t_ff，关节不动时前两项不变，
    # 所以反馈力矩的变化【就是】前馈的变化 —— 这是对 0x12 标度的直接测量，
    # 不经过"力矩->运动"，因此不受静摩擦和 kp 影响。
    taus = np.array([r["tau"][idx] for r in records])
    coef_t, *_ = np.linalg.lstsq(A, taus, rcond=None)
    alpha = float(coef_t[0])
    resid_t = taus - A @ coef_t
    n = len(cs)
    se = float(np.sqrt(np.sum(resid_t ** 2) / max(n - 2, 1) /
                       max(np.sum((cs - cs.mean()) ** 2), 1e-12)))

    # 【副判据】位移法：alpha_disp = kp · d(位移)/d(命令)。它经过"力矩->运动"，
    # 会被静摩擦压低 —— 两者的比值正是摩擦占前馈的比重，对阶段 3 有用。
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
        # 起点必须已在测试位姿附近：位移守卫直接生效，避免本工具承担大范围移动
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
        # 结束：回到纯位置保持
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
            # alpha_torque = 前馈【未】被关节运动吸收的比例：
            #   关节完全不动 -> 1（摩擦吃掉全部）
            #   关节自由移动 -> 0（位置环重新平衡，总力矩回到重力值）
            # 它【不是】0x12 的标度。标度只能由启动力矩【以上】的位移斜率
            # 给出（--breakaway），因为那时关节确实在动。
            print(f"joint{r['joint']:<3} {r['alpha']:>8.4f} "
                  f"±{r['alpha_stderr']:.4f}  "
                  f"位移 {r['max_abs_dq']:.5f} rad  "
                  f"{'几乎不动(摩擦主导)' if r['max_abs_dq'] < 3e-3 else '有移动'}")
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
