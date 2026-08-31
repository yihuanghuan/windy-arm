#!/usr/bin/env python3
"""用一段慢速三维弧线拟合 ``arm_base -> base_link`` 外参（仅实物）。

现场约定（给定，不参与拟合）：Vicon 刚体 arm_ee 与 FK 末端系 link6 重合，即
Re_T_E = I。在该约定下，整个数据集由一个固定变换解释：

    V_T_E_meas(k) = V_T_B · B_T_E(q_k)

未知量只有 V_T_B 一个。拟合后剩下的全部残差都归因于 arm_base 没有落在真正的
base_link 上，也就是本工具要产出的量：

    Rb_T_B = inverse(V_T_Rb) · V_T_B

Rb_T_B 直接填进 vicon_floating_base_bridge.py 的
rigid_from_base_translation_m / rigid_from_base_quaternion_xyzw。

不与末端外参联合拟合：两个未知量都放开就成了 AX = XB 的手眼标定形式，需要另一套
激励条件；这里按现场约定把 arm_ee 当作可信参考。

重力前馈发的是 g_urdf(q + zero_offset)，【不】再乘 first_moment_ratio：
config/arm6.urdf 的连杆惯量里已经烘进了辨识出的一阶矩（总质量 2.353 kg 对名义
1.982 kg，各关节重力比 1.19~1.36）。这与 student_arm_node 自己的 gravity_hold 口径
一致，故结束时把控制权交回驱动不会改变臂正在使用的力矩。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np

try:
    import pinocchio as pin
    import rclpy
    from dummy_interface.msg import MotorControl
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except ImportError as exc:  # pragma: no cover - hardware dependency
    rclpy = None
    _IMPORT_ERROR = exc

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from task_space_smc.core.model import load_arm6_calibration
from task_space_smc.hw.fk_gravity_verify import (
    GravityModel, JOINT_LOWER, JOINT_UPPER, N_ARM)


RATE_HZ = 100.0
# 刚度与驱动的保持增益一致，结束时交回控制权无缝。腕部阻尼在扫掠期间从驱动的
# 0.1 提到 0.3：阻尼比 ζ = kd / (2·sqrt(kp·J))，按辨识出的等效惯量
# 0.0123/0.0115/0.0105 kg·m²，kd=0.1 时 ζ 只有 0.20~0.22，每次静摩擦释放都过冲；
# joint6 最差（不承重力载荷、惯量纯为转子），释放速度 τ_静摩擦/kd = 0.1/0.1 = 1.0 rad/s，
# 与触发速度守卫的 0.501 rad/s 吻合。kd=0.3 时 ζ 升到 0.60~0.65，释放速度降到约 0.33 rad/s。
KP = np.array([30.0, 30.0, 30.0, 5.0, 5.0, 5.0])
KD = np.array([1.0, 1.0, 1.0, 0.3, 0.3, 0.3])
KD_HANDOVER = np.array([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])

# --- 运动安全阈值（运动阶段触发即停机并报告） -----------------------------
MIN_LIMIT_MARGIN = 0.05      # rad，距关节硬限位的余量
# 实测速度按 0.10 s 滑动窗判定，不看单样本。全关节粘滑：指令峰值 0.25 rad/s 时，
# 位置差分出的瞬时速度可达 1.24 rad/s —— 关节先卡在摩擦死区里不动，再整块挣脱
# （joint6 的轨迹是每 5 ms 走 0.003 rad，然后连续几个采样完全静止）。挣脱速度由静摩擦
# 释放决定而非指令：指令 0.05 / 0.164 / 0.25 rad/s 实测瞬时速度都 >= 0.5 rad/s。
# 同一段运动在 0.10 s 窗上平均后峰值只有 0.42 rad/s 并重新跟随指令，所以这个窗
# 才分得开「摩擦」与「失控」。0.5 rad/s 的失控在这个窗下仍能在 0.05 rad 行程内抓住。
MAX_MEASURED_SPEED = 0.50    # rad/s，SPEED_WINDOW_SEC 窗内均值
SPEED_WINDOW_SEC = 0.10
MAX_INSTANT_SPEED = 3.00     # rad/s，单样本兜底，只拦硬失控
# 跟踪误差按 0.05 rad 判，但关节自身的摩擦死区 = 静摩擦/kp 是位置精度的物理下限，
# 超过 0.05 时以死区为准：joint2 静摩擦 1.775 N·m 对 kp=30 即 0.059 rad，实测停在
# 0.0599 rad，那是死区不是跟踪故障。这不影响数据 —— 每个样本记的都是实测 q。
MAX_SETTLED_ERROR = 0.05     # rad，逐关节死区放宽之前的基准
SETTLED_ERROR_ALLOWANCE = 0.01
# 「数据流中断」按 Vicon 源时钟（服务器打进 VRPN 消息的帧时间）判定，不按本地到达。
# 本机 WiFi 链路上到达间隔可达 206 ms 而底层帧流完好：源时钟间隔实测 8.017 ms、
# σ = 0.474 ms，到达间隔却在 0.09~180 ms 之间散开（docs/QA_record.md）。
# 按到达判断会在投递抖动上误停，且抓不到任何真实丢帧。
MAX_VICON_GAP = 0.15         # s，静态窗过滤阈值；运动阶段仍是安全中止阈值
# 链路真死的兜底，按到达时间判。臂静止、源流完好（0.15 s 内零丢帧）时，
# 到达间隔仍能到 0.300 s，故阈值不能靠近这个量级。
MAX_VICON_SILENCE = 1.00     # s，完全没有数据到达 = 链路已死
MAX_JOINT_GAP = 0.10         # s，/joint_states 静默 = 串口心跳故障

# --- 采集窗 ----------------------------------------------------------------
MIN_CAPTURE_PAIRS = 200      # 每个姿态至少保留 200 对 source timestamp 配对帧
READY_FRAMES = 10             # 启动前只用于确认话题已经有数据
PAIR_MAX_DT = 0.010          # arm_base 与 arm_ee 的 source timestamp 最大差
SETTLE_SEC = 2.0             # 到位后 qdot 接近零并连续稳定等待 2 s
SETTLE_QDOT_MAX = 0.03       # rad/s，静态采样前的关节速度门槛
SETTLE_TIMEOUT_SEC = 10.0
CAPTURE_TIMEOUT_SEC = 10.0   # 10 s 内不足 MIN_CAPTURE_PAIRS 才判该点失败
LATENCY_SPIKE_SEC = 0.15     # 仅丢弃对应到达延迟异常帧，不终止整次标定

HERE = Path(__file__).resolve()
DEFAULT_CANDIDATE_YAML = HERE.parents[2] / "config" / "arm6_calibration_candidate.yaml"

# --- 轨迹 ------------------------------------------------------------------
# q_k = q0 + A·(sin(F·θ_k + Φ) − sin(Φ)),   θ_k = 2πk/N
# 减 sin(Φ) 这一项使首点恰好等于实测起始位姿，末段也回到该点。
# joint2 取二倍基频，末端路径才是真正的空间曲线而不是平面弧；joint3 只朝远离下限的
# 方向动（起点距下限 0.19 rad）。joint6 幅值故意取小：它的轴穿过 link6 原点，只改变
# 末端姿态，缩小它几乎不损失标定信息，却能减少这个最容易触发速度守卫的关节的静摩擦释放次数。
AMPLITUDE = np.array([0.45, 0.14, 0.15, 0.35, 0.40, 0.25])
PHASE = np.array([0.0, -math.pi / 2, -math.pi / 2, math.pi / 3,
                  math.pi / 2, 2 * math.pi / 3])
FREQUENCY = np.array([1.0, 2.0, 1.0, 1.0, 1.0, 1.0])
N_POINTS = 16
# 这是指令峰值的上限而非目标：实际由 MIN_SEGMENT_SEC 先行约束，剖面峰值约 0.16 rad/s，
# 对速度守卫留约 3 倍余量。指令放慢到 0.05 rad/s 反而更糟：各关节全部卡在自己的摩擦
# 死区（0.020~0.059 rad）里，再以约 0.49 rad/s 的瞬时速度整块挣脱。
# 取现场上限 0.50 rad/s 的一半，使【指令】峰值不会与速度守卫重合：取 0.50 时，
# 去起始位姿的长距离转移会把 joint6 指令到正好 0.50 rad/s，任何跟踪过冲都会踩到
# 上面第一个量化档（41 LSB = 0.5007 rad/s）而中止。
PEAK_JOINT_SPEED = 0.125     # rad/s，五次剖面的指令峰值
MIN_SEGMENT_SEC = 2.0        # s，单段时长下限，保证平滑

# 五次多项式 10u³−15u⁴+6u⁵ 的导数峰值为 1.875/T，故跨度为 d 的一段需要
# T >= 1.875·d / PEAK_JOINT_SPEED 才不超速。
QUINTIC_PEAK = 1.875


def quintic(u: float) -> float:
    u = float(np.clip(u, 0.0, 1.0))
    return 10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5


def quat_to_rotation(q_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = np.asarray(q_xyzw, dtype=float)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(n) or n < 1e-9:
        raise ValueError("invalid quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotation_to_quat(r: np.ndarray) -> np.ndarray:
    """旋转矩阵转 xyzw 四元数，按迹与最大对角元选分支，避开除零放大。"""
    r = np.asarray(r, dtype=float)
    t = float(np.trace(r))
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        q = np.array([(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
                      (r[1, 0] - r[0, 1]) / s, 0.25 * s])
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        q = np.array([0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s,
                      (r[2, 1] - r[1, 2]) / s])
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        q = np.array([(r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s,
                      (r[0, 2] - r[2, 0]) / s])
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        q = np.array([(r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s,
                      (r[1, 0] - r[0, 1]) / s])
    q = q / float(np.linalg.norm(q))
    return q if q[3] >= 0.0 else -q


def rotation_angle(r: np.ndarray) -> float:
    return math.acos(float(np.clip((np.trace(r) - 1.0) * 0.5, -1.0, 1.0)))


def project_rotation(m: np.ndarray) -> np.ndarray:
    """离 m 最近的旋转矩阵（Kabsch / 正交 Procrustes）：R = U·Vᵀ，
    det < 0 时翻转 U 的最后一列，保证结果在 SO(3) 而不是含反射。
    """
    u, _, vt = np.linalg.svd(np.asarray(m, dtype=float))
    r = u @ vt
    if np.linalg.det(r) < 0.0:
        u = u.copy()
        u[:, -1] *= -1.0
        r = u @ vt
    return r


def mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    if not rotations:
        raise ValueError("no rotations to average")
    return project_rotation(sum(rotations))


def plan_waypoints(q0: np.ndarray, n_points: int) -> np.ndarray:
    theta = np.linspace(0.0, 2.0 * math.pi, int(n_points), endpoint=False)
    return np.array([q0 + AMPLITUDE * (np.sin(FREQUENCY * t + PHASE)
                                       - np.sin(PHASE)) for t in theta])


def segment_time(q_from: np.ndarray, q_to: np.ndarray) -> float:
    span = float(np.max(np.abs(np.asarray(q_to) - np.asarray(q_from))))
    return max(MIN_SEGMENT_SEC, QUINTIC_PEAK * span / PEAK_JOINT_SPEED)


def limit_margin(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    return np.minimum(q - JOINT_LOWER, JOINT_UPPER - q)


class Abort(RuntimeError):
    pass


class CaptureFailure(RuntimeError):
    """一个姿态的有效配对帧不足；标定继续到下一个姿态。"""


def _stream_quality(rows):
    """过滤 source gap 和传输延迟异常帧，坏帧不升级为全局中止。"""
    if not rows:
        return {"rows": [], "dropped_gap": 0, "dropped_latency": 0,
                "max_gap_sec": 0.0, "max_latency_residual_sec": 0.0}
    ordered = sorted(rows, key=lambda row: row[1])
    offset = float(np.median([row[0] - row[1] for row in ordered]))
    clean = []
    dropped_gap = dropped_latency = 0
    max_gap = max_latency = 0.0
    previous_source = None
    for row in ordered:
        source = row[1]
        if previous_source is not None:
            gap = source - previous_source
            if gap > 0.0:
                max_gap = max(max_gap, float(gap))
            if gap > MAX_VICON_GAP:
                dropped_gap += 1
                previous_source = source
                continue
        latency_residual = abs((row[0] - source) - offset)
        max_latency = max(max_latency, float(latency_residual))
        previous_source = source
        if latency_residual > LATENCY_SPIKE_SEC:
            dropped_latency += 1
            continue
        clean.append(row)
    return {"rows": clean, "dropped_gap": dropped_gap,
            "dropped_latency": dropped_latency, "max_gap_sec": max_gap,
            "max_latency_residual_sec": max_latency}


def _pair_by_source_timestamp(base_rows, ee_rows):
    """按 source timestamp 最近邻一对一配对，只保留差值小于 10 ms 的样本。"""
    base = sorted(base_rows, key=lambda row: row[1])
    ee = sorted(ee_rows, key=lambda row: row[1])
    pairs = []
    j = 0
    for b in base:
        while j + 1 < len(ee) and ee[j + 1][1] <= b[1]:
            j += 1
        candidates = []
        if j < len(ee):
            candidates.append((abs(ee[j][1] - b[1]), j))
        if j > 0:
            candidates.append((abs(ee[j - 1][1] - b[1]), j - 1))
        if candidates:
            dt, idx = min(candidates)
            if dt < PAIR_MAX_DT:
                pairs.append((b, ee[idx]))
                if idx == j:
                    j += 1
    return pairs


class BaseExtrinsicCalibrator(Node):
    def __init__(self, base_topic: str, ee_topic: str, calibration_yaml: str):
        super().__init__("vicon_base_extrinsic_calibrate")
        self.model = GravityModel()
        self.calibration_yaml = str(Path(calibration_yaml).expanduser().resolve())
        cal = load_arm6_calibration(self.calibration_yaml)
        self.get_logger().info("FK calibration YAML: %s" % self.calibration_yaml)
        self.zero_offset = np.asarray(cal["zero_offset_rad"], dtype=float)
        deadband = np.asarray(cal["stiction_nm"], dtype=float) / KP
        self.settled_error_limit = np.maximum(
            MAX_SETTLED_ERROR, deadband + SETTLED_ERROR_ALLOWANCE)

        self.q: np.ndarray | None = None
        self.qd: np.ndarray | None = None
        self.q_stamp = -math.inf
        self.joint_log: deque[tuple[float, np.ndarray]] = deque(maxlen=4000)
        self.peak_speed = np.zeros(N_ARM)
        self.kd = KD.copy()

        # 每行为 (到达单调时间, Vicon 源时间戳, 位置, 旋转)
        self.base: deque[tuple[float, float, np.ndarray, np.ndarray]] = deque(maxlen=16000)
        self.ee: deque[tuple[float, float, np.ndarray, np.ndarray]] = deque(maxlen=16000)
        self.base_stamp = -math.inf
        self.ee_stamp = -math.inf
        self.last_gap = {"arm_base": 0.0, "arm_ee": 0.0}
        self.worst_gap = {"arm_base": 0.0, "arm_ee": 0.0}

        self.pub = self.create_publisher(MotorControl, "/student/joint_effort_command", 10)
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 50)
        self.create_subscription(PoseStamped, base_topic, self._base_cb, 200)
        self.create_subscription(PoseStamped, ee_topic, self._ee_cb, 200)
        self.base_topic, self.ee_topic = base_topic, ee_topic

    # ---------------- 回调 ----------------
    def _joint_cb(self, msg: JointState) -> None:
        if len(msg.position) != N_ARM:
            return
        now = time.monotonic()
        self.q = np.asarray(msg.position, dtype=float)
        self.qd = (np.asarray(msg.velocity, dtype=float)
                   if len(msg.velocity) == N_ARM else np.zeros(N_ARM))
        self.q_stamp = now
        self.joint_log.append((now, self.q.copy()))

    @staticmethod
    def _pose(msg: PoseStamped) -> tuple[float, np.ndarray, np.ndarray] | None:
        p = np.array([msg.pose.position.x, msg.pose.position.y,
                      msg.pose.position.z], dtype=float)
        if not np.all(np.isfinite(p)):
            return None
        try:
            r = quat_to_rotation(np.array([
                msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w]))
        except ValueError:
            return None
        source = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
        return source, p, r

    def _accept(self, name: str, buf: deque, msg: PoseStamped) -> float:
        """Append one pose and return its arrival time, tracking source gaps."""
        pose = self._pose(msg)
        if pose is None:
            return -math.inf
        arrival = time.monotonic()
        if buf:
            gap = pose[0] - buf[-1][1]
            # 乱序投递是传输产物，不是丢帧，不计入间隔统计
            if gap > 0.0:
                self.last_gap[name] = gap
                self.worst_gap[name] = max(self.worst_gap[name], gap)
        buf.append((arrival, *pose))
        return arrival

    def _base_cb(self, msg: PoseStamped) -> None:
        stamp = self._accept("arm_base", self.base, msg)
        if math.isfinite(stamp):
            self.base_stamp = stamp

    def _ee_cb(self, msg: PoseStamped) -> None:
        stamp = self._accept("arm_ee", self.ee, msg)
        if math.isfinite(stamp):
            self.ee_stamp = stamp

    # ---------------- 收发 ----------------
    def spin(self, seconds: float = 0.002) -> None:
        rclpy.spin_once(self, timeout_sec=seconds)

    def wait_ready(self, timeout: float = 15.0) -> None:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin(0.05)
            if (self.q is not None and len(self.base) >= READY_FRAMES
                    and len(self.ee) >= READY_FRAMES):
                return
        missing = []
        if self.q is None:
            missing.append("/joint_states (is student_arm6.launch.py running?)")
        if len(self.base) < READY_FRAMES:
            missing.append(self.base_topic)
        if len(self.ee) < READY_FRAMES:
            missing.append(self.ee_topic)
        raise Abort("no data on: " + ", ".join(missing))

    def gravity_feedforward(self) -> np.ndarray:
        if self.q is None:
            return np.zeros(N_ARM)
        return self.model.gravity(self.q + self.zero_offset)

    def command(self, q_ref: np.ndarray) -> None:
        msg = MotorControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [float(v) for v in q_ref]
        msg.velocity = [0.0] * N_ARM
        msg.current = [float(v) for v in self.gravity_feedforward()]
        msg.p = [float(v) for v in KP]
        msg.d = [float(v) for v in self.kd]
        msg.enable = 1
        self.pub.publish(msg)

    def windowed_speed(self) -> np.ndarray:
        """Mean |dq/dt| per joint over the trailing SPEED_WINDOW_SEC.

        Taken from measured positions rather than the reported velocity
        channel: the wrist velocity words carry spikes even at a standstill
        (0.159 rad/s with the arm provably not moving), while positions do not.
        """
        cutoff = time.monotonic() - SPEED_WINDOW_SEC
        rows = [(t, q) for t, q in self.joint_log if t >= cutoff]
        if len(rows) < 3:
            return np.zeros(N_ARM)
        dt = rows[-1][0] - rows[0][0]
        if dt < 1e-3:
            return np.zeros(N_ARM)
        return np.abs(rows[-1][1] - rows[0][1]) / dt

    # ---------------- 守卫 ----------------
    def guard(self, q_ref: np.ndarray, *, settled: bool,
              check_vicon_gap: bool = True) -> None:
        now = time.monotonic()
        if self.q is None or self.qd is None:
            raise Abort("no joint feedback yet")
        age = now - self.q_stamp
        if age > MAX_JOINT_GAP:
            raise Abort("/joint_states silent for %.3f s (> %.2f s): serial "
                        "heartbeat fault" % (age, MAX_JOINT_GAP))
        for name, stamp in (("arm_base", self.base_stamp), ("arm_ee", self.ee_stamp)):
            if check_vicon_gap and self.last_gap[name] > MAX_VICON_GAP:
                raise Abort("Vicon %s dropped frames for %.3f s on the source "
                            "clock (> %.2f s)"
                            % (name, self.last_gap[name], MAX_VICON_GAP))
            silence = now - stamp
            if silence > MAX_VICON_SILENCE:
                raise Abort("nothing arrived on Vicon %s for %.3f s (> %.2f s): "
                            "link down" % (name, silence, MAX_VICON_SILENCE))
        margin = limit_margin(self.q)
        if float(np.min(margin)) < MIN_LIMIT_MARGIN:
            j = int(np.argmin(margin))
            raise Abort("joint%d is %.4f rad from its hard limit (< %.2f)"
                        % (j + 1, margin[j], MIN_LIMIT_MARGIN))
        instant = np.abs(self.qd)
        if float(np.max(instant)) > MAX_INSTANT_SPEED:
            j = int(np.argmax(instant))
            raise Abort("joint%d hit %.3f rad/s in one sample (> %.2f): runaway"
                        % (j + 1, instant[j], MAX_INSTANT_SPEED))
        speed = self.windowed_speed()
        self.peak_speed = np.maximum(self.peak_speed, speed)
        if float(np.max(speed)) > MAX_MEASURED_SPEED:
            j = int(np.argmax(speed))
            raise Abort("joint%d averaged %.3f rad/s over the last %.2f s "
                        "(> %.2f); peak so far per joint %s"
                        % (j + 1, speed[j], SPEED_WINDOW_SEC,
                           MAX_MEASURED_SPEED,
                           np.round(self.peak_speed, 3).tolist()))
        if settled:
            err = np.abs(self.q - np.asarray(q_ref, dtype=float))
            over = err - self.settled_error_limit
            if float(np.max(over)) > 0.0:
                j = int(np.argmax(over))
                raise Abort("settled tracking error on joint%d is %.4f rad "
                            "(> %.4f)" % (j + 1, err[j],
                                          self.settled_error_limit[j]))

    # ---------------- 运动 ----------------
    def hold(self, q_ref: np.ndarray, seconds: float, *, settled: bool,
             check_vicon_gap: bool = True) -> None:
        end = time.monotonic() + seconds
        next_pub = time.monotonic()
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_pub:
                self.command(q_ref)
                next_pub = now + 1.0 / RATE_HZ
            self.spin()
            self.guard(q_ref, settled=settled, check_vicon_gap=check_vicon_gap)

    def wait_zero_velocity(self, q_ref: np.ndarray) -> None:
        """要求 qdot 连续低于门槛 2 s，再清理运动阶段缓存。"""
        deadline = time.monotonic() + SETTLE_TIMEOUT_SEC
        stable_since = None
        while time.monotonic() < deadline:
            now = time.monotonic()
            self.command(q_ref)
            self.spin()
            self.guard(q_ref, settled=True, check_vicon_gap=False)
            # JointState.velocity 在静止时有编码器量化跳变；用实测位置的
            # 0.1 s 窗口差分判断真正的 qdot，避免把静止误判成持续运动。
            qdot = float(np.max(self.windowed_speed()))
            if qdot <= SETTLE_QDOT_MAX:
                stable_since = stable_since if stable_since is not None else now
                if now - stable_since >= SETTLE_SEC:
                    return
            else:
                stable_since = None
        raise CaptureFailure("qdot did not stay below %.3f rad/s within %.1f s"
                             % (SETTLE_QDOT_MAX, SETTLE_TIMEOUT_SEC))

    def move(self, q_from: np.ndarray, q_to: np.ndarray, move_sec: float) -> None:
        t0 = time.monotonic()
        next_pub = t0
        while True:
            now = time.monotonic()
            u = (now - t0) / move_sec
            if u >= 1.0:
                break
            q_ref = q_from + quintic(u) * (q_to - q_from)
            if now >= next_pub:
                self.command(q_ref)
                next_pub = now + 1.0 / RATE_HZ
            self.spin()
            self.guard(q_ref, settled=False)

    def capture(self, q_ref: np.ndarray) -> dict:
        """静置后按 source timestamp 配对采样；坏帧只丢弃，不中止全局标定。"""
        self.wait_zero_velocity(q_ref)
        # 清除运动阶段和静置阶段积压的数据，采集窗从这里开始。
        self.base.clear()
        self.ee.clear()
        self.joint_log.clear()
        t_start = time.monotonic()
        deadline = t_start + CAPTURE_TIMEOUT_SEC

        while True:
            self.hold(q_ref, 0.05, settled=True, check_vicon_gap=False)
            base = [r for r in self.base if r[0] >= t_start]
            ee = [r for r in self.ee if r[0] >= t_start]
            joints = [(t, q) for t, q in self.joint_log if t >= t_start]
            base_quality = _stream_quality(base)
            ee_quality = _stream_quality(ee)
            pairs = _pair_by_source_timestamp(
                base_quality["rows"], ee_quality["rows"])
            if len(pairs) >= MIN_CAPTURE_PAIRS:
                break
            if time.monotonic() > deadline:
                raise CaptureFailure("10 s capture window has only %d valid pairs "
                                     "(need %d)" % (len(pairs), MIN_CAPTURE_PAIRS))

        base = [pair[0] for pair in pairs]
        ee = [pair[1] for pair in pairs]
        window_sec = max(pairs[-1][0][1] - pairs[0][0][1],
                         pairs[-1][1][1] - pairs[0][1][1])

        def reduce(rows):
            pts = np.asarray([r[2] for r in rows])
            p = np.median(pts, axis=0)
            r = mean_rotation([row[3] for row in rows])
            spread = float(np.max(np.linalg.norm(pts - p, axis=1)))
            orientation_spread = float(np.degrees(max(
                rotation_angle(r.T @ row[3]) for row in rows)))
            return p, r, spread, orientation_spread

        p_b, r_b, spread_b, ori_spread_b = reduce(base)
        p_e, r_e, spread_e, ori_spread_e = reduce(ee)
        q_meas = np.median(np.asarray([q for _, q in joints]), axis=0)
        return dict(
            q_encoder=q_meas, q_command=np.asarray(q_ref, dtype=float).copy(),
            base_p=p_b, base_r=r_b, ee_p=p_e, ee_r=r_e,
            base_frames=len(base), ee_frames=len(ee), valid_pairs=len(pairs),
            joint_frames=len(joints), base_spread_m=spread_b,
            ee_spread_m=spread_e,
            base_orientation_spread_deg=ori_spread_b,
            ee_orientation_spread_deg=ori_spread_e,
            base_gap_max_sec=base_quality["max_gap_sec"],
            ee_gap_max_sec=ee_quality["max_gap_sec"],
            base_dropped_gap=base_quality["dropped_gap"],
            ee_dropped_gap=ee_quality["dropped_gap"],
            base_dropped_latency=base_quality["dropped_latency"],
            ee_dropped_latency=ee_quality["dropped_latency"],
            pair_time_delta_max_sec=float(max(
                abs(b[1] - e[1]) for b, e in pairs)),
            window_sec=window_sec)

    # ---------------- 运动学 ----------------
    def fk(self, q_encoder: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        q = np.asarray(q_encoder, dtype=float) + self.zero_offset
        pin.forwardKinematics(self.model.model, self.model.data, q)
        pin.updateFramePlacements(self.model.model, self.model.data)
        pose = self.model.data.oMf[self.model.ee_frame_id]
        return pose.translation.copy(), pose.rotation.copy()


def fit_fixed_transform(samples: list[dict], fk) -> dict:
    """只用末端【位置】拟合 V_T_B（Kabsch / 正交 Procrustes）：

        R_vb = project_rotation( (p_e − p̄_e)ᵀ · (p_be − p̄_be) )
        p_vb = p̄_e − R_vb · p̄_be

    不能直接对 4×4 齐次矩阵取 || V_T_E_meas(k) − T·B_T_E(q_k) ||_F：该范数把
    O(1) 量级的旋转块与以米计（O(0.2)）的平移块相加，姿态项的权重约是位置项的
    100 倍 —— 而本机坏掉的恰恰是姿态通道。16 个位姿实测：两团位置点云的两两距离
    只差 8.3 mm RMS，而 arm_ee 的姿态与 link6 差 114°，扣掉这个常量后仍有约 10° RMS。
    按该范数拟合会让坏通道拖垮好通道，残差 137 mm，而只用位置拟合能解释到 9 mm。

    旋转因此由位置点云给出。点云在这里是真三维的（主轴延展 74 / 46 / 14 mm），
    足以唯一确定旋转。不联合拟合末端外参：现场约定 arm_ee 的【原点】与 link6 原点
    重合，正是这条约定让本拟合成立，也是数据支持的那部分约定。
    """
    fk_poses = [fk(s["q_encoder"]) for s in samples]
    p_be = np.asarray([p for p, _ in fk_poses])
    p_e = np.asarray([s["ee_p"] for s in samples])
    p_be_bar, p_e_bar = p_be.mean(axis=0), p_e.mean(axis=0)

    r_vb = project_rotation((p_e - p_e_bar).T @ (p_be - p_be_bar))
    p_vb = p_e_bar - r_vb @ p_be_bar

    # 仅诊断用：能把两边姿态对齐的那个常量偏转 c，以及扣掉它之后还剩多少。
    # c 的角度大说明 Re_R_E = I 不成立；剩余残差大说明 arm_ee 的姿态通道
    # 根本不能用于标定。
    c = project_rotation(sum(r_be.T @ r_vb.T @ s["ee_r"]
                             for s, (_, r_be) in zip(samples, fk_poses)))
    with_c = [rotation_angle((r_vb @ r_be @ c).T @ s["ee_r"])
              for s, (_, r_be) in zip(samples, fk_poses)]
    return dict(r_vb=r_vb, p_vb=p_vb, fk_poses=fk_poses,
                arm_ee_frame_offset_deg=float(np.degrees(rotation_angle(c))),
                arm_ee_frame_offset_quaternion_xyzw=rotation_to_quat(c).tolist(),
                orientation_residual_after_offset_rms_deg=float(np.degrees(
                    np.sqrt(np.mean(np.square(with_c))))))


def analyse(samples: list[dict], fk) -> dict:
    fit = fit_fixed_transform(samples, fk)
    r_vb, p_vb = fit["r_vb"], fit["p_vb"]

    residuals = []
    rb_p, rb_r = [], []
    for s, (p_be, r_be) in zip(samples, fit["fk_poses"]):
        p_pred = p_vb + r_vb @ p_be
        r_pred = r_vb @ r_be
        dp = p_pred - s["ee_p"]
        residuals.append(dict(
            position_error_mm=float(1e3 * np.linalg.norm(dp)),
            position_error_xyz_mm=(1e3 * dp).tolist(),
            orientation_error_deg=float(np.degrees(
                rotation_angle(s["ee_r"].T @ r_pred)))))
        # 由这一帧自己的 arm_base 位姿反解 Rb_T_B = inverse(V_T_Rb)·V_T_B
        rb_p.append(s["base_r"].T @ (p_vb - s["base_p"]))
        rb_r.append(s["base_r"].T @ r_vb)

    rb_p = np.asarray(rb_p)
    p_rb_b = np.median(rb_p, axis=0)
    r_rb_b = mean_rotation(list(rb_r))
    # 逐帧解出的 Rb_T_B 在样本间的离散度，直接检验基座在采集期间没有移动
    rb_p_spread_mm = float(1e3 * np.max(np.linalg.norm(rb_p - p_rb_b, axis=1)))
    rb_r_spread_deg = float(np.degrees(max(
        rotation_angle(r_rb_b.T @ r) for r in rb_r)))

    pos = np.asarray([r["position_error_mm"] for r in residuals])
    ori = np.asarray([r["orientation_error_deg"] for r in residuals])
    q_encoder = np.asarray([s["q_encoder"] for s in samples])

    def correlations(values):
        out = {}
        for j in range(q_encoder.shape[1]):
            x = q_encoder[:, j]
            out["joint%d" % (j + 1)] = (None if len(values) < 2
                                         or np.std(x) < 1e-12
                                         or np.std(values) < 1e-12 else
                                         float(np.corrcoef(x, values)[0, 1]))
        return out

    return dict(
        vicon_from_base_link=dict(
            translation_m=p_vb.tolist(),
            rotation_matrix=r_vb.tolist(),
            quaternion_xyzw=rotation_to_quat(r_vb).tolist()),
        arm_base_from_base_link=dict(
            translation_m=p_rb_b.tolist(),
            rotation_matrix=r_rb_b.tolist(),
            quaternion_xyzw=rotation_to_quat(r_rb_b).tolist(),
            translation_spread_mm=rb_p_spread_mm,
            rotation_spread_deg=rb_r_spread_deg),
        residual=dict(
            position_rms_mm=float(np.sqrt(np.mean(pos ** 2))),
            position_max_mm=float(pos.max()),
            orientation_rms_deg=float(np.sqrt(np.mean(ori ** 2))),
            orientation_max_deg=float(ori.max()),
            position_error_vs_joint_correlation=correlations(pos),
            orientation_error_vs_joint_correlation=correlations(ori)),
        per_sample_residual=residuals,
        arm_ee_frame_check=dict(
            frame_offset_deg=fit["arm_ee_frame_offset_deg"],
            frame_offset_quaternion_xyzw=fit["arm_ee_frame_offset_quaternion_xyzw"],
            residual_after_offset_rms_deg=fit[
                "orientation_residual_after_offset_rms_deg"],
            note="Re_R_E = I is only assumed, never fitted; these numbers say "
                 "how far the assumption is from the measurement.  The fit "
                 "uses positions only, so a large offset here does not enter "
                 "the result -- it invalidates using arm_ee orientation."),
        bridge_parameters=dict(
            rigid_from_base_translation_m=[float(v) for v in p_rb_b],
            rigid_from_base_quaternion_xyzw=[
                float(v) for v in rotation_to_quat(r_rb_b)]),
    )


def describe_plan(q0: np.ndarray, waypoints: np.ndarray, node_fk) -> dict:
    poses = [node_fk(q) for q in waypoints]
    p = np.asarray([x[0] for x in poses])
    r = [x[1] for x in poses]
    centred = p - p.mean(axis=0)
    sv = np.linalg.svd(centred, compute_uv=False)
    legs = [(q0, waypoints[0])] + [(waypoints[i - 1], waypoints[i])
                                   for i in range(1, len(waypoints))]
    legs.append((waypoints[-1], q0))
    spans = np.array([np.max(np.abs(b - a)) for a, b in legs])
    times = np.array([segment_time(a, b) for a, b in legs])
    angles = [np.degrees(rotation_angle(r[0].T @ x)) for x in r]
    return dict(
        n_points=len(waypoints),
        limit_margin_min_rad=float(np.min([limit_margin(q).min() for q in waypoints])),
        max_segment_span_rad=float(spans.max()),
        peak_commanded_speed_rad_s=float(np.max(QUINTIC_PEAK * spans / times)),
        ee_extent_mm=(1e3 * (p.max(axis=0) - p.min(axis=0))).tolist(),
        ee_principal_extent_mm=(1e3 * sv / math.sqrt(len(p))).tolist(),
        arc_non_straightness=float(sv[1] / sv[0]),
        arc_non_planarity=float(sv[2] / sv[0]),
        orientation_span_deg=float(max(angles)),
        estimated_minutes=float(sum(times)
                                + len(waypoints) *
                                (SETTLE_SEC + MIN_CAPTURE_PAIRS / 120.0)) / 60.0,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--points", type=int, default=N_POINTS,
                    help="采样点数，必须在 12~20 之间")
    ap.add_argument("--base-topic", default="/vrpn/arm_base/pose")
    ap.add_argument("--ee-topic", default="/vrpn/arm_ee/pose")
    ap.add_argument("--hold-after-sec", type=float, default=3.0,
                    help="收尾时继续发【重力前馈+PD】的秒数，之后交给驱动的"
                         "同款重力保位兜底，不会退回纯 PD")
    ap.add_argument("--start-pose", default="",
                    help="先按同样的限速与守卫走到这个位形，再从它生成圆弧。"
                         "留空则直接用当前位姿。六个逗号分隔的弧度值")
    ap.add_argument("--plan-only", action="store_true",
                    help="只读当前位姿并打印轨迹计划，不发任何指令")
    ap.add_argument("--output-dir", default="")
    ap.add_argument("--calibration-yaml", default=str(DEFAULT_CANDIDATE_YAML),
                    help="FK 使用的标定文件，默认是 arm6_calibration_candidate.yaml")
    args = ap.parse_args()

    if not 12 <= args.points <= 20:
        print("--points must be between 12 and 20", file=sys.stderr)
        return 2
    if rclpy is None:
        print("rclpy unavailable: %s" % _IMPORT_ERROR, file=sys.stderr)
        return 2

    rclpy.init()
    node = BaseExtrinsicCalibrator(args.base_topic, args.ee_topic,
                                   args.calibration_yaml)
    result: dict = {}
    try:
        node.wait_ready()
        q0 = node.q.copy()
        if args.start_pose:
            q_start = np.array([float(v) for v in args.start_pose.split(",")],
                               dtype=float)
            if q_start.shape != (N_ARM,):
                raise Abort("--start-pose needs %d values" % N_ARM)
            if float(np.min(limit_margin(q_start))) < MIN_LIMIT_MARGIN:
                raise Abort("--start-pose violates the %.2f rad limit margin"
                            % MIN_LIMIT_MARGIN)
            if not args.plan_only:
                node.get_logger().info(
                    "moving to the requested start pose %s"
                    % np.round(q_start, 4).tolist())
                node.move(q0, q_start, segment_time(q0, q_start))
                node.hold(q_start, SETTLE_SEC, settled=True)
                node.peak_speed[:] = 0.0
                q0 = node.q.copy()
            else:
                q0 = q_start
        waypoints = plan_waypoints(q0, args.points)
        plan = describe_plan(q0, waypoints, node.fk)
        node.get_logger().info("start pose (rad): %s"
                               % np.round(q0, 4).tolist())
        node.get_logger().info(
            "plan: %d points, limit margin >= %.3f rad, peak commanded speed "
            "%.4f rad/s, EE extent %s mm, arc s2/s1 %.2f s3/s1 %.2f, "
            "orientation span %.0f deg, ~%.1f min"
            % (plan["n_points"], plan["limit_margin_min_rad"],
               plan["peak_commanded_speed_rad_s"],
               np.round(plan["ee_extent_mm"], 0).tolist(),
               plan["arc_non_straightness"], plan["arc_non_planarity"],
               plan["orientation_span_deg"], plan["estimated_minutes"]))

        for k, q in enumerate(waypoints):
            margin = limit_margin(q)
            if float(np.min(margin)) < MIN_LIMIT_MARGIN:
                raise Abort("planned point %d violates the %.2f rad limit "
                            "margin" % (k + 1, MIN_LIMIT_MARGIN))
        if plan["arc_non_straightness"] < 0.15:
            raise Abort("planned arc is too close to a straight line "
                        "(s2/s1 = %.3f)" % plan["arc_non_straightness"])

        if args.plan_only:
            print(json.dumps(dict(start_pose_rad=q0.tolist(), plan=plan,
                                  waypoints_rad=waypoints.tolist()), indent=2))
            return 0

        samples = []
        failed_points = []
        q_prev = q0
        for k, q_target in enumerate(waypoints, 1):
            node.move(q_prev, q_target, segment_time(q_prev, q_target))
            try:
                sample = node.capture(q_target)
            except CaptureFailure as exc:
                failed_points.append(dict(point=k, reason=str(exc)))
                node.get_logger().warning("point %d/%d failed, continue: %s"
                                          % (k, len(waypoints), exc))
            else:
                samples.append(sample)
                node.get_logger().info(
                    "point %d/%d captured: %d paired frames, spread p %.2f / %.2f mm, "
                    "spread R %.3f / %.3f deg, dropped gap %d/%d latency %d/%d"
                    % (k, len(waypoints), sample["valid_pairs"],
                       1e3 * sample["base_spread_m"], 1e3 * sample["ee_spread_m"],
                       sample["base_orientation_spread_deg"],
                       sample["ee_orientation_spread_deg"],
                       sample["base_dropped_gap"], sample["ee_dropped_gap"],
                       sample["base_dropped_latency"], sample["ee_dropped_latency"]))
            q_prev = q_target

        node.get_logger().info("returning to the start pose")
        node.move(q_prev, q0, segment_time(q_prev, q0))
        # 用驱动自己的阻尼交回控制权，切到它的 gravity_hold 兜底时臂的受力不变
        node.kd = KD_HANDOVER.copy()
        node.hold(q0, max(0.0, args.hold_after_sec), settled=True)

        if not samples:
            raise Abort("all planned static points failed to collect 200 valid pairs")
        result = analyse(samples, node.fk)
        result["plan"] = plan
        result["failed_points"] = failed_points
        result["assumptions"] = dict(
            arm_ee_equals_link6=True,
            end_effector_extrinsic_fitted=False,
            q_fk="joint_encoder + arm6_calibration_candidate.zero_offset_rad",
            calibration_yaml=node.calibration_yaml,
            gravity_feedforward="g(arm6.urdf) at q_fk, no first_moment_ratio "
                                "(already baked into the URDF inertias)",
            vicon_axes="+y up; URDF joint1 axis +x")
        result["samples"] = [dict(
            q_encoder_rad=s["q_encoder"].tolist(),
            q_command_rad=s["q_command"].tolist(),
            arm_base_position_m=s["base_p"].tolist(),
            arm_base_quaternion_xyzw=rotation_to_quat(s["base_r"]).tolist(),
            arm_ee_position_m=s["ee_p"].tolist(),
            arm_ee_quaternion_xyzw=rotation_to_quat(s["ee_r"]).tolist(),
            arm_base_frames=s["base_frames"], arm_ee_frames=s["ee_frames"],
            valid_pairs=s["valid_pairs"],
            joint_frames=s["joint_frames"],
            arm_base_spread_mm=1e3 * s["base_spread_m"],
            arm_ee_spread_mm=1e3 * s["ee_spread_m"],
            arm_base_orientation_spread_deg=s["base_orientation_spread_deg"],
            arm_ee_orientation_spread_deg=s["ee_orientation_spread_deg"],
            arm_base_gap_max_sec=s["base_gap_max_sec"],
            arm_ee_gap_max_sec=s["ee_gap_max_sec"],
            arm_base_dropped_gap=s["base_dropped_gap"],
            arm_ee_dropped_gap=s["ee_dropped_gap"],
            arm_base_dropped_latency=s["base_dropped_latency"],
            arm_ee_dropped_latency=s["ee_dropped_latency"],
            pair_time_delta_max_sec=s["pair_time_delta_max_sec"],
            window_sec=s["window_sec"]) for s in samples]
        result["vicon_stream"] = dict(
            worst_source_clock_gap_sec=dict(node.worst_gap),
            static_capture_gap_filter_sec=MAX_VICON_GAP,
            static_capture_pair_max_dt_sec=PAIR_MAX_DT,
            static_capture_latency_filter_sec=LATENCY_SPIKE_SEC,
            static_capture_min_valid_pairs=MIN_CAPTURE_PAIRS,
            static_capture_timeout_sec=CAPTURE_TIMEOUT_SEC,
            dynamic_movement_gap_abort_threshold_sec=MAX_VICON_GAP,
            link_down_abort_threshold_sec=MAX_VICON_SILENCE)
        return 0
    except Abort as exc:
        node.get_logger().error("ABORTED: %s" % exc)
        return 3
    except KeyboardInterrupt:
        node.get_logger().warning("interrupted; driver keeps the gravity hold")
        return 4
    finally:
        if result:
            out_dir = (Path(args.output_dir) if args.output_dir
                       else Path(__file__).resolve().parent / "data"
                       / "vicon_base_extrinsic")
            out_dir.mkdir(parents=True, exist_ok=True)
            path = out_dir / ("base_extrinsic_%s.json"
                              % time.strftime("%Y%m%d_%H%M%S"))
            path.write_text(json.dumps(result, indent=2))
            print(json.dumps(dict(
                residual=result["residual"],
                arm_base_from_base_link=result["arm_base_from_base_link"],
                bridge_parameters=result["bridge_parameters"]), indent=2))
            print("result -> %s" % path)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
