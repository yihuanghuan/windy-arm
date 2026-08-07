"""重力补偿验收（实物）：逐级抽掉位置反馈，看重力前馈能否独自撑住位姿。

每一级把位置环增益按比例缩小

    kp = kp_frac · BASE_P,   kd = kd_frac · BASE_D

前馈始终为 tau_ff = ratio ⊙ g(q)。kp_frac 从 1.00 一路降到 0（L4「悬浮」级，
位置比例项完全撤掉，只留阻尼），记录各级的稳态残差

    residual = mean(q_measured) − q_hold

前馈准确时残差不随 kp_frac 下降而放大；若在低增益级发散，说明前馈的一阶矩或
零位偏置还不对。
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
    DESK_PLANE_Z, DESK_EDGE_X, DESK_CLEARANCE, FENCE_Y, FENCE_HEIGHT,
    FENCE_CLEARANCE,
)
from task_space_smc.core.model import load_arm6_calibration

OUT = Path(__file__).resolve().parent / "data" / "phase4"

BASE_P = [30.0, 30.0, 30.0, 5.0, 5.0, 5.0]
BASE_D = [1.0, 1.0, 1.0, 0.1, 0.1, 0.1]


LEVELS = [
    dict(name="L0 基线", kp_frac=1.00, kd_frac=1.00),
    dict(name="L1", kp_frac=0.50, kd_frac=0.50),
    dict(name="L2", kp_frac=0.20, kd_frac=0.30),
    dict(name="L3", kp_frac=0.05, kd_frac=0.20),
    dict(name="L4 悬浮", kp_frac=0.00, kd_frac=0.20),
]

PUBLISH_RATE = 100.0
SETTLE_SEC = 3.0
HOLD_SEC = 4.0
DRIFT_ABORT = 0.20
SPEED_ABORT = 0.5
DESK_MARGIN_MIN = 0.10
FENCE_MARGIN_MIN = 0.05


# 工作空间余量：对伸到桌面上方（x < DESK_EDGE_X）的连杆点，同时取它离桌面的
# 垂直余量和离围栏的水平余量，返回其中最小的一个，负值即已越界。
def desk_margin(model: GravityModel, q: np.ndarray) -> float:
    m = np.inf
    for p in model.link_points(np.asarray(q, float)):
        if p[0] < DESK_EDGE_X:
            m = min(m, p[2] - DESK_PLANE_Z - DESK_CLEARANCE)
            if p[2] < DESK_PLANE_Z + FENCE_HEIGHT:
                m = min(m, FENCE_Y - FENCE_CLEARANCE - p[1])
    return float(m)


class Compensator(Node):
    def __init__(self, *, ignore_workspace_guard: bool = False):
        super().__init__("phase4_gravity_compensation")
        self.model = GravityModel()
        cal = load_arm6_calibration()
        self.offset = np.array(cal["zero_offset_rad"], dtype=float)
        self.ratio = np.array(cal["first_moment_ratio"], dtype=float)
        self.cal_source = cal["source"]
        # 仅供动捕标定场地（确认四周无桌面/围栏）使用。关节限位、速度和
        # 漂移保护绝不因此关闭。
        self.ignore_workspace_guard = bool(ignore_workspace_guard)
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

    def feedforward(self, q: np.ndarray) -> np.ndarray:
        qc = np.asarray(q, float) + self.offset
        return self.model.gravity(qc) * self.ratio

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
            return ("student_arm_node is in POSITION mode — this stage needs "
                    "the torque path. Relaunch with "
                    "enable_direct_effort_command:=true")
        return None

    def wait_state(self, timeout: float = 10.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout and (self.q is None or self.tau is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.q is not None and self.tau is not None

    def guard(self, q_start) -> str | None:
        if self.q is None:
            return None
        drift = np.max(np.abs(self.q - q_start))
        if drift > DRIFT_ABORT:
            j = int(np.argmax(np.abs(self.q - q_start))) + 1
            return f"joint{j} drifted {drift:.3f} rad (limit {DRIFT_ABORT})"
        if self.v is not None and np.max(np.abs(self.v)) > SPEED_ABORT:
            return f"speed {np.max(np.abs(self.v)):.2f} rad/s exceeded"
        if not self.ignore_workspace_guard:
            mg = desk_margin(self.model, self.q)
            if mg < 0.0:
                return f"desk/fence margin {mg:+.4f} m"
        lim = np.minimum(self.q - JOINT_LOWER, JOINT_UPPER - self.q)
        if lim.min() < 0.05:
            return f"joint{int(np.argmin(lim)) + 1} within 0.05 rad of a limit"
        return None

    def run_level(self, q_hold, level) -> tuple[dict | None, str | None]:
        kp = [BASE_P[i] * level["kp_frac"] for i in range(N_ARM)]
        kd = [BASE_D[i] * level["kd_frac"] for i in range(N_ARM)]
        period = 1.0 / PUBLISH_RATE
        t0 = time.time()
        nxt = t0
        samples = []
        while time.time() - t0 < SETTLE_SEC + HOLD_SEC:
            now = time.time()
            if now >= nxt:
                self.send(q_hold, kp, kd, self.feedforward(self.q if self.q is not None else q_hold))
                nxt = now + period
            rclpy.spin_once(self, timeout_sec=0.002)
            why = self.guard(q_hold)
            if why:
                return None, why
            if self.q is not None and now - t0 > SETTLE_SEC:
                samples.append((self.q.copy(),
                                self.tau.copy() if self.tau is not None
                                else np.zeros(N_ARM)))
        if len(samples) < 10:
            return None, "not enough samples"
        Q = np.array([s[0] for s in samples])
        T = np.array([s[1] for s in samples])
        err = Q.mean(axis=0) - np.asarray(q_hold, float)
        return dict(name=level["name"], kp_frac=level["kp_frac"],
                    kd_frac=level["kd_frac"],
                    residual_rad=err.tolist(),
                    residual_max=float(np.max(np.abs(err))),
                    drift_ptp=float(np.max(np.ptp(Q, axis=0))),
                    tau_mean=T.mean(axis=0).tolist(),
                    n=len(samples)), None


# 随机搜一个适合做本验收的起始位形：过安全闸、桌面与围栏余量够、离限位有 0.30 rad
# 空间，并且大臂三轴的最大重力力矩尽量接近 2.5 N·m —— 太小则前馈误差被淹没，
# 太大则低增益级本身就不安全。固定随机种子保证每次选到同一个位形。
def pick_start_posture(model: GravityModel) -> tuple[np.ndarray, dict]:
    rng = np.random.default_rng(11)
    lo = np.maximum(JOINT_LOWER, -1.4)
    hi = np.minimum(JOINT_UPPER, 1.4)
    best = None
    for _ in range(20000):
        q = rng.uniform(lo, hi)
        if not check_setpoint(q, model).ok:
            continue
        if desk_margin(model, q) < DESK_MARGIN_MIN:
            continue
        room = np.minimum(q - JOINT_LOWER, JOINT_UPPER - q).min()
        if room < 0.30:
            continue
        g = np.abs(model.gravity(q))

        score = -abs(g[:3].max() - 2.5)
        if best is None or score > best[0]:
            best = (score, q.copy(), desk_margin(model, q), room, g)
    _, q, mg, room, g = best
    return q, dict(desk_margin=mg, limit_room=room,
                   gravity=np.round(g, 3).tolist())


def print_plan() -> None:
    model = GravityModel()
    cal = load_arm6_calibration()
    q, info = pick_start_posture(model)
    print("=" * 74)
    print("PLAN (dry run — nothing is commanded)")
    print("=" * 74)
    print(f"标定来源: {cal['source']}")
    print(f"零位偏差(°): {[round(np.degrees(v), 2) for v in cal['zero_offset_rad']]}")
    print(f"ratio      : {cal['first_moment_ratio']}")
    print(f"\n起始位姿: {np.round(q, 4).tolist()}")
    print(f"  桌面/围栏余量 {info['desk_margin']:.4f} m  (要求 >= {DESK_MARGIN_MIN})")
    print(f"  限位最小余量  {info['limit_room']:.3f} rad  (要求 >= 0.30)")
    print(f"  重力力矩      {info['gravity']} Nm")
    print(f"\n分级方案（kd 始终保留，阻尼不产生位置牵引）:")
    print(f"{'级':<10}{'kp':>8}{'kd':>8}{'joint1~3 kp':>14}{'joint4~6 kp':>14}")
    for lv in LEVELS:
        print(f"{lv['name']:<10}{lv['kp_frac']*100:>7.0f}%{lv['kd_frac']*100:>7.0f}%"
              f"{BASE_P[0]*lv['kp_frac']:>14.2f}{BASE_P[3]*lv['kp_frac']:>14.2f}")
    print(f"\n每级 {SETTLE_SEC:.0f}s 稳定 + {HOLD_SEC:.0f}s 采样")
    print(f"中止条件: 漂移 > {DRIFT_ABORT} rad / 速度 > {SPEED_ABORT} rad/s / "
          f"桌面围栏越界 / 距限位 < 0.05 rad")
    print("=" * 74)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--max-level', type=int, default=len(LEVELS) - 1,
                    help='最高执行到第几级（0-4）。默认全程到 kp=0')
    ap.add_argument('--posture', default=None, help='逗号分隔的 6 个角度')
    ap.add_argument('--ignore-workspace-guard', action='store_true',
                    help='仅在确认四周无桌面/围栏的动捕标定场地使用；'
                         '不关闭关节限位、速度或漂移保护')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    if args.dry_run:
        print_plan()
        return 0
    if rclpy is None:
        print(f"rclpy unavailable: {_IMPORT_ERROR}", file=sys.stderr)
        return 2

    OUT.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    rclpy.init()
    node = Compensator(ignore_workspace_guard=args.ignore_workspace_guard)
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
            print("REFUSED: 标定文件缺失 —— 前馈会退回恒等标定，机械臂会掉",
                  file=sys.stderr)
            return 5

        q_hold = node.q.copy()
        rep = check_setpoint(q_hold, node.model)
        mg = desk_margin(node.model, q_hold)
        reasons = list(rep.reasons)
        if args.ignore_workspace_guard:
            # check_setpoint 的桌面/围栏项是场地相关的软约束；保留所有
            # 关节限位项。实际硬限位余量仍由 guard() 的 0.05 rad 强制执行。
            reasons = [r for r in reasons if 'desk' not in r and 'fence' not in r]
            reasons = [r for r in reasons if 'within 0.15 rad' not in r]
        posture_ok = not reasons
        node.get_logger().info(
            f"起始位姿 {np.round(q_hold, 4).tolist()}  "
            f"余量 {mg:+.4f} m  安全检查 {'通过' if posture_ok else '未通过'}")
        if not posture_ok or (not args.ignore_workspace_guard and mg < DESK_MARGIN_MIN):
            print(f"REFUSED: 当前位姿不适合悬浮测试 "
                  f"(margin {mg:+.4f}, 要求 >= {DESK_MARGIN_MIN})", file=sys.stderr)
            return 6

        g = node.feedforward(q_hold)
        node.get_logger().info(f"该位姿前馈力矩 {np.round(g, 3).tolist()} Nm")

        for i, lv in enumerate(LEVELS):
            if i > args.max_level:
                node.get_logger().info(f"停在 --max-level {args.max_level}")
                break
            node.get_logger().info(
                f"--- {lv['name']}  kp={lv['kp_frac']*100:.0f}% "
                f"kd={lv['kd_frac']*100:.0f}% ---")
            rec, why = node.run_level(q_hold, lv)
            if why:
                node.get_logger().error(f"{lv['name']} 中止: {why}")

                safe = LEVELS[max(i - 1, 0)]
                node.get_logger().warn(f"回退到 {safe['name']}")
                for _ in range(300):
                    node.send(node.q if node.q is not None else q_hold,
                              [BASE_P[k] * safe['kp_frac'] for k in range(N_ARM)],
                              [BASE_D[k] * safe['kd_frac'] for k in range(N_ARM)],
                              node.feedforward(node.q if node.q is not None else q_hold))
                    rclpy.spin_once(node, timeout_sec=0.01)
                results.append(dict(name=lv['name'], ok=False, reason=why))
                break
            results.append(dict(ok=True, **rec))
            node.get_logger().info(
                f"  残余误差 max {rec['residual_max']:.4f} rad "
                f"({np.degrees(rec['residual_max']):.2f}°)  "
                f"抖动 {rec['drift_ptp']:.5f} rad")


            base = next((r for r in results if r.get('ok')
                         and r['kp_frac'] == 1.0), None)
            if base and lv['kp_frac'] > 0:
                expect = base['residual_max'] / max(lv['kp_frac'], 1e-9)
                if rec['residual_max'] > 3.0 * expect:
                    node.get_logger().error(
                        f"  残余误差 {rec['residual_max']:.4f} 超预期 "
                        f"{expect:.4f} 的 3 倍 —— 前馈可能有误，停止下探")
                    break
    finally:
        path = OUT / f"phase4_{stamp}.json"
        path.write_text(json.dumps(dict(levels=results), indent=2, default=float))
        print(f"\nresults -> {path}")
        print(f"\n{'级':<10}{'kp':>7}{'残余误差(°)':>14}{'抖动(rad)':>12}  判定")
        for r in results:
            if not r.get('ok'):
                print(f"{r['name']:<10}{'':>7}{'':>14}{'':>12}  中止: {r.get('reason')}")
                continue
            print(f"{r['name']:<10}{r['kp_frac']*100:>6.0f}%"
                  f"{np.degrees(r['residual_max']):>14.2f}{r['drift_ptp']:>12.5f}")
        if results and results[-1].get('ok') and results[-1]['kp_frac'] == 0.0:
            print("\n>>> kp=0 下机械臂保持住了 —— 悬浮成立。")
            print("    现在可以用手推一下：应能推动，松手停在原地。")
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
