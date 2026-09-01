#!/usr/bin/env python3
"""把 Vicon/VRPN 的刚体位姿转成 ``FloatingBaseState``。

VRPN listener 给每个位姿打的是 vrpn_TRACKERCB.msg_time，即【Vicon 服务器自己的
帧时间】。该时钟与臂端计算机不同步（实测差 12 h 时区加约 4.6 s 真实偏斜），但它的
【差值】是准的：帧间隔实测 8.017 ms、σ = 0.474 ms，而本地到达间隔在 WiFi 上从
0.09 ms 散到 180 ms。

所以这里把时间戳的两个职责分开：

* **求导**用源时钟差。拿真实 8 ms 的位移去除以 0.09 ms 的到达间隔，速度被放大约
  90 倍 —— 曾被当成「Vicon 噪声」归咎于网络的部分，大半来自这一处（线速度噪声
  30 倍，角速度峰值 66 倍）。
* **发布的时间戳**用本地时钟，因为控制器的陈旧性判据要拿它跟自己的 now 比。取的是
  【采集】时刻映射到本地时间的结果（经 arrival − source 的滑动窗最小值），不是到达
  时刻：按到达打戳，一个在网络里飞了 180 ms 的样本会被报成 0 ms 新鲜，而按采集打戳
  报成 180 ms —— 后者才是那道判据存在的意义。

实测数据与出处见 docs/VICON_TIMESTAMP_JITTER.md。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from dummy_interface.msg import FloatingBaseState
from geometry_msgs.msg import AccelStamped, PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray

if __package__ in (None, ""):  # 直接按脚本运行（install/ 下的入口就是这样）
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from task_space_smc.core.base_state_fusion import (
    AssembledFrame, DerivativeConsistencyMonitor, FrameAssembler,
    SourceClockAligner, shift_twist_accel)


def _normalize_quat_xyzw(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(4)
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid quaternion")
    return q / norm


def _quat_multiply_xyzw(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def _quat_conjugate_xyzw(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]])


def _rotation_matrix_xyzw(q: np.ndarray) -> np.ndarray:
    x, y, z, w = _normalize_quat_xyzw(q)
    return np.array([
        [1.0 - 2.0 * (y*y + z*z), 2.0 * (x*y - z*w), 2.0 * (x*z + y*w)],
        [2.0 * (x*y + z*w), 1.0 - 2.0 * (x*x + z*z), 2.0 * (y*z - x*w)],
        [2.0 * (x*z - y*w), 2.0 * (y*z + x*w), 1.0 - 2.0 * (x*x + y*y)],
    ])


def _rotation_vector_xyzw(q: np.ndarray) -> np.ndarray:
    q = _normalize_quat_xyzw(q)
    # q 与 −q 表示同一旋转，取点积为正的那一支，保证增量走短程。
    if q[3] < 0.0:
        q = -q
    sin_half = float(np.linalg.norm(q[:3]))
    if sin_half < 1e-9:
        return 2.0 * q[:3]
    angle = 2.0 * math.atan2(sin_half, float(q[3]))
    return q[:3] * (angle / sin_half)


class ViconFloatingBaseBridge(Node):
    def __init__(self):
        super().__init__("vicon_floating_base_bridge")
        self.declare_parameter("pose_topic", "/vrpn/base/pose")
        self.declare_parameter("output_topic", "/floating_base/vicon")
        self.declare_parameter("world_from_vicon_translation_m", [0.0, 0.0, 0.0])
        self.declare_parameter("world_from_vicon_quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("rigid_from_base_translation_m", [0.0, 0.0, 0.0])
        self.declare_parameter("rigid_from_base_quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("zero_on_first_pose", True)
        self.declare_parameter("filter_time_constant_sec", 0.04)
        self.declare_parameter("max_sample_gap_sec", 0.10)
        self.declare_parameter("use_source_timestamp", True)
        self.declare_parameter("clock_offset_window_sec", 5.0)
        self.declare_parameter("input_queue_depth", 200)
        self.declare_parameter("stats_interval_sec", 10.0)
        # Vicon 原生 twist/accel 与 arm_ee。缺省话题名与 vrpn_listener 的命名一致。
        self.declare_parameter("twist_topic", "")
        self.declare_parameter("accel_topic", "")
        self.declare_parameter("ee_pose_topic", "")
        # 末端的 twist/accel 与位姿【同源同帧】，走和基座完全一样的一套：
        # 各自一个 SourceClockAligner，再由共用的 FrameAssembler 按精确源戳
        # 装配成整帧。
        self.declare_parameter("ee_twist_topic", "")
        self.declare_parameter("ee_accel_topic", "")
        self.declare_parameter("ee_output_topic", "/vicon/arm_ee")
        self.declare_parameter("diagnostics_topic", "/floating_base/vicon_diagnostics")
        self.declare_parameter("prefer_vicon_twist", True)
        self.declare_parameter("prefer_vicon_linear_accel", True)
        # 角速度默认【不采信 Vicon】（2026-08-06 由 true 改为 false）。
        #
        # vrpn_listener 现在把角速度按 0 发布（vel_quat 语义未判定）。而
        # "上游发 0" + "这道对照闸"是个危险组合：
        #     判据 |cand-ref| <= tol*max(|ref|,floor)，cand=0
        #     => |ω_ref| <= floor 时退化成 |ω_ref| <= tol*floor = 0.1 rad/s
        # 于是慢速转动（<= 5.73 °/s，正是保持 demo 的工作区间）时，硬零会被
        # 闸门当成真实测量放行，控制器就看不到基座角速度，ω×r 与 ω×(ω×r)
        # 一起消失。闸门是为"真实但可能错标度的值"设计的，挡不住主动发零。
        self.declare_parameter("prefer_vicon_angular_velocity", True)
        self.declare_parameter("angular_cross_check_tol", 0.5)
        self.declare_parameter("angular_cross_check_floor_rad_s", 0.2)
        self.declare_parameter("angular_accel_cross_check_floor_rad_s2", 2.0)
        # 角加速度同理默认关闭（2026-08-06 由 true 改为 false）。同一道闸、
        # 同样的硬零放行区间（floor=2.0 rad/s^2 => 放行 |α| <= 1.0 rad/s^2）。
        # acc_quat 的可用性完全继承 vel_quat：实测
        # acc_quat[1:4] = 120*Δ(vel_quat[1:4]) 是精确恒等式。
        self.declare_parameter("prefer_vicon_angular_accel", True)
        # Base 与末端的角量选择不可共用：用户要求 Base 用 Vicon，而 arm_ee
        # 姿态及其导数仍由 FK/本地差分提供。
        self.declare_parameter("prefer_ee_vicon_angular_velocity", False)
        self.declare_parameter("prefer_ee_vicon_angular_accel", False)
        self.declare_parameter("max_cross_channel_skew_sec", 0.004)
        # 整帧装配的完成超时。缺省 3 ms：大于实测的最大到达跨度 1.232 ms
        # （中位 0.158 ms、p95 0.424 ms），小于半个帧周期 4 ms。
        # 某条通道再也不来时由它兜底，绝不会卡住不发。
        self.declare_parameter("frame_completion_timeout_sec", 0.003)
        self.declare_parameter("accel_hold_sec", 0.05)
        self.declare_parameter("consistency_linear_tol_m_s", 0.05)
        self.declare_parameter("consistency_angular_tol_rad_s", 0.5)
        self.declare_parameter("consistency_window_sec", 0.5)
        self.declare_parameter("consistency_fail_sec", 2.0)

        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.t_wv = np.asarray(
            self.get_parameter("world_from_vicon_translation_m").value, dtype=float).reshape(3)
        self.q_wv = _normalize_quat_xyzw(
            self.get_parameter("world_from_vicon_quaternion_xyzw").value)
        self.t_rb = np.asarray(
            self.get_parameter("rigid_from_base_translation_m").value, dtype=float).reshape(3)
        self.q_rb = _normalize_quat_xyzw(
            self.get_parameter("rigid_from_base_quaternion_xyzw").value)
        self.zero_on_first_pose = bool(self.get_parameter("zero_on_first_pose").value)
        self.tau_filter = max(0.0, float(self.get_parameter("filter_time_constant_sec").value))
        self.max_gap = max(1e-3, float(self.get_parameter("max_sample_gap_sec").value))
        self.use_source_timestamp = bool(
            self.get_parameter("use_source_timestamp").value)
        self.clock_offset_window = max(
            0.5, float(self.get_parameter("clock_offset_window_sec").value))
        self.stats_interval = max(
            0.0, float(self.get_parameter("stats_interval_sec").value))

        self.prev_t: float | None = None
        self.prev_p: np.ndarray | None = None
        self.prev_q: np.ndarray | None = None
        self.prev_v = np.zeros(6)
        self.prev_a = np.zeros(6)
        self.zero_p: np.ndarray | None = None
        self.zero_q: np.ndarray | None = None
        self._last_q_vr = np.array([0.0, 0.0, 0.0, 1.0])

        # 时钟对齐实现在 core/base_state_fusion.SourceClockAligner 里，不在这里
        # 重写一份 —— twist/accel/arm_ee 三条新通道各自持有一个同类实例，
        # 保证四条流用的是【同一套】源时钟方案而不是四套相似的。
        self._pose_clock = SourceClockAligner(
            self.clock_offset_window, self.use_source_timestamp)
        self._offset_logged = False

        self.twist_topic = str(self.get_parameter("twist_topic").value)
        self.accel_topic = str(self.get_parameter("accel_topic").value)
        self.ee_pose_topic = str(self.get_parameter("ee_pose_topic").value)
        self.ee_twist_topic = str(self.get_parameter("ee_twist_topic").value)
        self.ee_accel_topic = str(self.get_parameter("ee_accel_topic").value)
        self.prefer_vicon_twist = bool(self.get_parameter("prefer_vicon_twist").value)
        self.prefer_vicon_linear_accel = bool(
            self.get_parameter("prefer_vicon_linear_accel").value)
        self.prefer_vicon_angular_velocity = bool(
            self.get_parameter("prefer_vicon_angular_velocity").value)
        self.angular_cross_check_tol = float(
            self.get_parameter("angular_cross_check_tol").value)
        self.angular_cross_check_floor = float(
            self.get_parameter("angular_cross_check_floor_rad_s").value)
        self.angular_accel_cross_check_floor = float(
            self.get_parameter("angular_accel_cross_check_floor_rad_s2").value)
        self.prefer_vicon_angular_accel = bool(
            self.get_parameter("prefer_vicon_angular_accel").value)
        self.prefer_ee_vicon_angular_velocity = bool(
            self.get_parameter("prefer_ee_vicon_angular_velocity").value)
        self.prefer_ee_vicon_angular_accel = bool(
            self.get_parameter("prefer_ee_vicon_angular_accel").value)
        self.max_channel_skew = max(
            0.0, float(self.get_parameter("max_cross_channel_skew_sec").value))
        self.accel_hold_sec = max(
            0.0, float(self.get_parameter("accel_hold_sec").value))

        self._twist_clock = SourceClockAligner(
            self.clock_offset_window, self.use_source_timestamp)
        self._accel_clock = SourceClockAligner(
            self.clock_offset_window, self.use_source_timestamp)
        self._ee_clock = SourceClockAligner(
            self.clock_offset_window, self.use_source_timestamp)
        self._ee_twist_clock = SourceClockAligner(
            self.clock_offset_window, self.use_source_timestamp)
        self._ee_accel_clock = SourceClockAligner(
            self.clock_offset_window, self.use_source_timestamp)
        # ---- 整帧装配。基座与末端【共用同一个类】，理由与 SourceClockAligner
        # 和 _r_ctrl_from_vicon 相同：两份相似但不同的实现正是这个项目反复踩过
        # 的坑。装配器只把没配置的通道排除在"齐全"条件之外，所以只订阅 pose
        # 时额外延迟严格为 0。
        self.frame_timeout = max(
            0.0, float(self.get_parameter("frame_completion_timeout_sec").value))
        self._frame_wait = 0.0
        self._ee_frame_wait = 0.0
        self._build_assemblers()

        # arm_ee 的本地差分状态。它有两个职责，缺一不可：Vicon 通道缺帧时的
        # 回退值，以及【对照闸的裁判】—— 没有裁判就只能盲信 vel_quat，而
        # ||vel_quat|| = 3.850（不是单位四元数）说明它的语义还没被验证过。
        self._ee_prev_t: float | None = None
        self._ee_prev_p: np.ndarray | None = None
        self._ee_prev_q: np.ndarray | None = None
        self._ee_prev_v = np.zeros(6)
        self._ee_prev_a = np.zeros(6)
        self._ee_last_good_accel: np.ndarray | None = None
        self._ee_last_good_accel_time: float | None = None
        self._ee_used_vicon_twist = False
        self._ee_used_vicon_accel = False
        self._ee_channel_skew = 0.0
        self._ee_vicon_angular_ok = False
        self._ee_angular_reject = 0
        self._ee_vicon_angular_accel_ok = False
        self._ee_angular_accel_reject = 0
        self._ee_withheld = 0
        self._ee_startup_logged = False
        self._last_good_accel: np.ndarray | None = None
        self._last_good_accel_time: float | None = None
        self._used_vicon_twist = False
        self._used_vicon_accel = False
        self._channel_skew = 0.0
        self._consistency = DerivativeConsistencyMonitor(
            linear_tol=float(self.get_parameter("consistency_linear_tol_m_s").value),
            angular_tol=float(self.get_parameter("consistency_angular_tol_rad_s").value),
            window_sec=float(self.get_parameter("consistency_window_sec").value),
            fail_sec=float(self.get_parameter("consistency_fail_sec").value),
        )
        self._consistency_gap = (0.0, 0.0)
        self._withheld = 0
        self.prev_v_valid = False
        self._startup_logged = False
        self._vicon_angular_ok = False
        self._angular_reject = 0
        self._vicon_angular_accel_ok = False
        self._angular_accel_reject = 0

        # 现场数据验收用的诊断量
        self._stats_start: float | None = None
        self._stats_count = 0
        self._stats_dropped = 0
        self._stats_latency_sum = 0.0
        self._stats_latency_max = 0.0
        self._stats_windows = 0

        queue_depth = max(1, int(self.get_parameter("input_queue_depth").value))
        self.pub = self.create_publisher(FloatingBaseState, self.output_topic, 10)
        # WiFi 是成串投递的：实测最小到达间隔 0.087 ms，即几十帧落在同一个调度片里。
        # 队列深度取 10 时，2.4 GHz 下有 4.5%、5 GHz 下有 14% 的帧是在【本节点内】
        # 被丢掉的 —— 网络本身已经完好送达。
        self.sub = self.create_subscription(
            PoseStamped, self.pose_topic, self._pose_cb, queue_depth)

        # 辅助通道全部走同样的深队列：QA_record 里深度 10 在【节点内部】丢掉
        # 4.5%(2.4G)/14%(5G) 的帧，那是在网络已经把它们完整送到之后。
        self.twist_sub = self.accel_sub = self.ee_sub = None
        if self.twist_topic:
            self.twist_sub = self.create_subscription(
                TwistStamped, self.twist_topic, self._twist_cb, queue_depth)
        if self.accel_topic:
            self.accel_sub = self.create_subscription(
                AccelStamped, self.accel_topic, self._accel_cb, queue_depth)
        self.ee_pub = None
        self.ee_twist_sub = self.ee_accel_sub = None
        if self.ee_pose_topic:
            # arm_ee 必须经【同一个】桥换算：zero_on_first_pose 用第一帧基座位姿
            # 定义了控制器世界系，另起一个节点会落在不同的世界系里。
            self.ee_sub = self.create_subscription(
                PoseStamped, self.ee_pose_topic, self._ee_cb, queue_depth)
            # 发 FloatingBaseState 而不是三个话题：一条消息一个时间戳同时带
            # 位姿/速度/加速度，下游就不必再面对跨话题时间对齐 —— 那正是这个
            # 项目在基座上花了大力气才解决掉的问题，没必要在末端重来一遍。
            self.ee_pub = self.create_publisher(
                FloatingBaseState,
                str(self.get_parameter("ee_output_topic").value), 10)
            if self.ee_twist_topic:
                self.ee_twist_sub = self.create_subscription(
                    TwistStamped, self.ee_twist_topic, self._ee_twist_cb,
                    queue_depth)
            if self.ee_accel_topic:
                self.ee_accel_sub = self.create_subscription(
                    AccelStamped, self.ee_accel_topic, self._ee_accel_cb,
                    queue_depth)
        self.diag_pub = self.create_publisher(
            Float64MultiArray, str(self.get_parameter("diagnostics_topic").value), 10)
        # Read-only timing probe: records the exact local time immediately before
        # each bridge output publish. It is separate from control data.
        self.timing_pub = self.create_publisher(
            Float64MultiArray, "/floating_base/vicon_timing", 200)

        self.get_logger().info(
            "Vicon base bridge: %s -> %s (zero_on_first_pose=%s, "
            "source_timestamp=%s, queue=%d)"
            % (self.pose_topic, self.output_topic, self.zero_on_first_pose,
               self.use_source_timestamp, queue_depth))
        self.get_logger().info(
            "  aux channels: twist=%s accel=%s arm_ee=%s | prefer vicon "
            "twist=%s lin_accel=%s ang_accel=%s"
            % (self.twist_topic or "-", self.accel_topic or "-",
               self.ee_pose_topic or "-", self.prefer_vicon_twist,
               self.prefer_vicon_linear_accel, self.prefer_vicon_angular_accel))
        self.get_logger().info(
            "  arm_ee aux: twist=%s accel=%s（末端通路【不加杠杆臂】：Vicon 报的"
            "就是标记簇原点，而控制点就是这个原点）"
            % (self.ee_twist_topic or "-", self.ee_accel_topic or "-"))
        self.get_logger().info(
            "  整帧装配：基座必需通道 %s，末端 %s，完成超时 %.1f ms"
            % (sorted(self._base_frames.channels),
               sorted(self._ee_frames.channels), 1e3 * self.frame_timeout))

        # 超时兜底的定时器。没有它，"某条通道再也不来"要等下一帧 pose
        # （8 ms）才会被冲出去；有了它，等待上界就是 frame_completion_timeout。
        # 周期取超时的一半，保证判定的时间分辨率够。
        self._flush_timer = None
        if self.frame_timeout > 0.0:
            self._flush_timer = self.create_timer(
                max(0.0005, 0.5 * self.frame_timeout), self._flush_frames)

    def _build_assemblers(self) -> None:
        """按【当前配置的话题】建装配器。

        没配置的通道不进 ``channels``，于是"齐全"就不含它 —— 只订阅 pose 时
        额外延迟严格为 0。这是回归底线，也是把这一步拆成独立方法的理由：
        测试要能在改完话题名之后重建。
        """
        self._base_frames = FrameAssembler(
            "pose",
            ["pose"] + (["twist"] if self.twist_topic else [])
                     + (["accel"] if self.accel_topic else []),
            timeout_sec=self.frame_timeout)
        self._ee_frames = FrameAssembler(
            "pose",
            ["pose"] + (["twist"] if self.ee_twist_topic else [])
                     + (["accel"] if self.ee_accel_topic else []),
            timeout_sec=self.frame_timeout)

    def _transform_pose(self, msg: PoseStamped) -> tuple[np.ndarray, np.ndarray]:
        p_vr = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q_vr = _normalize_quat_xyzw(np.array([
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        ]))
        # twist/accel 的杠杆臂要用【刚体系】的姿态，不是变换后的 base_link 姿态
        self._last_q_vr = q_vr
        r_wv = _rotation_matrix_xyzw(self.q_wv)
        r_vr = _rotation_matrix_xyzw(q_vr)
        p_wb = self.t_wv + r_wv @ (p_vr + r_vr @ self.t_rb)
        q_wb = _normalize_quat_xyzw(_quat_multiply_xyzw(
            _quat_multiply_xyzw(self.q_wv, q_vr), self.q_rb))
        if self.zero_on_first_pose:
            if self.zero_p is None:
                self.zero_p, self.zero_q = p_wb.copy(), q_wb.copy()
                self.get_logger().info("Captured the initial Vicon pose as the controller world origin")
            q_inv = _quat_conjugate_xyzw(self.zero_q)
            p_wb = _rotation_matrix_xyzw(q_inv) @ (p_wb - self.zero_p)
            q_wb = _normalize_quat_xyzw(_quat_multiply_xyzw(q_inv, q_wb))
        return p_wb, q_wb

    def _sync_pose_clock(self) -> SourceClockAligner:
        """Push the live parameter values into the aligner before each use.

        两个参数在运行中可被改写（测试就直接赋值 ``clock_offset_window`` /
        ``use_source_timestamp``），所以每次取用时同步一次，而不是只在构造时读。
        """
        self._pose_clock.window_sec = max(0.5, float(self.clock_offset_window))
        self._pose_clock.enabled = bool(self.use_source_timestamp)
        return self._pose_clock

    def _source_time(self, msg: PoseStamped) -> float | None:
        """Vicon frame time carried by the VRPN message, or ``None`` if absent.

        ``vrpn_listener`` copies ``vrpn_TRACKERCB.msg_time`` straight into the
        header.  A zero (or negative) stamp means nobody filled it in, which is
        the one case where arrival time is genuinely the best we have.
        """
        return self._sync_pose_clock().source_time(
            msg.header.stamp.sec, msg.header.stamp.nanosec)

    def _clock_offset(self, arrival: float, source: float) -> float:
        """Sliding-window minimum of ``arrival - source``.

        The lowest-latency sample in the window defines the offset, so the
        result is a clock offset rather than a latency average.  Two alternating
        windows keep this O(1) and let the estimate follow clock drift instead
        of latching onto one lucky early packet.
        """
        return self._sync_pose_clock().offset(arrival, source)

    def _r_ctrl_from_vicon(self) -> np.ndarray:
        """VRPN 世界系 -> 控制器世界系的旋转。

        Vicon 原生 twist/accel 是在 ``vicon_world`` 里给的；本地差分出来的量
        天然就在控制器世界系里（它是对已经变换+归零过的位姿做差分）。两者
        不是同一个系，直接混用等于悄悄换了坐标系，而且一致性对照会变成跨
        坐标系相减 —— 那道闸本身就错了。这个 bug 真实发生过，所以基座与末端
        两条通路共用这一个函数，而不是各写一遍。

            R_ctrl_from_vicon = R(conj(zero_q)) @ R_wv    （开启 zero_on_first_pose）
                              = R_wv                       （未开启）
        """
        r = _rotation_matrix_xyzw(self.q_wv)
        if self.zero_on_first_pose and self.zero_q is not None:
            r = _rotation_matrix_xyzw(_quat_conjugate_xyzw(self.zero_q)) @ r
        return r

    def _cross_check(self, candidate, reference, tol, floor):
        """粗量级对照：``|cand - ref| <= tol * max(|ref|, floor)``。

        必须在【转进控制器世界系之后】调用。相对判据而不是绝对判据，是因为
        参考量本身随运动幅度变化两个数量级；floor 保证静止时不会除以零。
        """
        gap = float(np.linalg.norm(np.asarray(candidate) - np.asarray(reference)))
        scale = max(float(np.linalg.norm(reference)), float(floor))
        return gap <= float(tol) * scale, gap

    @staticmethod
    def _frame_key(msg) -> tuple[int, int]:
        """整帧装配的键：**精确的** ``(sec, nanosec)`` 整数对。

        实测同一 Vicon 帧的 pose/twist/accel 三条话题 header.stamp 逐位相同
        （3000/3000），所以整数相等就是最强的同帧判据。用浮点比较反而要引入
        一个本不需要的容差，而任何容差都有把相邻帧误配在一起的风险。
        """
        return (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))

    def _submit_aux(self, assembler, clock, channel, msg, value, on_frame):
        """辅助通道（twist/accel）的公共提交路径。"""
        arrival = self.get_clock().now().nanoseconds * 1e-9
        aligned = clock.align(
            arrival, msg.header.stamp.sec, msg.header.stamp.nanosec)
        if not aligned.used_source:
            # stamp=0 就是 vrpn_listener 未修复时的现状（实测 5156/5156）。
            # 没有源时刻就无法与位姿同帧对齐，只能整条丢弃并回退到本地差分 ——
            # 这正是"修复前也能部署"的那条路径。这条通道因此永远不会 submit，
            # 也就永远不会进入装配器的必需集合，帧靠 pose 一到就齐全。
            return
        if not np.all(np.isfinite(value)):
            return
        for frame in assembler.submit(channel, self._frame_key(msg),
                                      value, arrival):
            on_frame(frame, arrival)

    def _twist_cb(self, msg: TwistStamped) -> None:
        self._submit_aux(
            self._base_frames, self._twist_clock, "twist", msg,
            np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                      msg.twist.angular.x, msg.twist.angular.y,
                      msg.twist.angular.z]),
            self._emit_base_frame)

    def _accel_cb(self, msg: AccelStamped) -> None:
        self._submit_aux(
            self._base_frames, self._accel_clock, "accel", msg,
            np.array([msg.accel.linear.x, msg.accel.linear.y, msg.accel.linear.z,
                      msg.accel.angular.x, msg.accel.angular.y,
                      msg.accel.angular.z]),
            self._emit_base_frame)

    def _ee_twist_cb(self, msg: TwistStamped) -> None:
        self._submit_aux(
            self._ee_frames, self._ee_twist_clock, "twist", msg,
            np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                      msg.twist.angular.x, msg.twist.angular.y,
                      msg.twist.angular.z]),
            self._emit_ee_frame)

    def _ee_accel_cb(self, msg: AccelStamped) -> None:
        self._submit_aux(
            self._ee_frames, self._ee_accel_clock, "accel", msg,
            np.array([msg.accel.linear.x, msg.accel.linear.y, msg.accel.linear.z,
                      msg.accel.angular.x, msg.accel.angular.y,
                      msg.accel.angular.z]),
            self._emit_ee_frame)

    def _flush_frames(self) -> None:
        """定时器驱动的超时兜底。

        某条通道再也不来时，光靠"下一条消息到达"来推进会等到下一帧 pose
        （8 ms）。有了它，等待上界就是 frame_completion_timeout_sec。
        """
        arrival = self.get_clock().now().nanoseconds * 1e-9
        for frame in self._base_frames.flush_expired(arrival):
            self._emit_base_frame(frame, arrival)
        for frame in self._ee_frames.flush_expired(arrival):
            self._emit_ee_frame(frame, arrival)

    def _ee_vicon_twist_accel(self, twist, accel, local_v, local_a):
        """末端的 Vicon 原生 twist/accel，转进控制器世界系后与本地差分二选一。

        ``twist`` / ``accel`` 由 :class:`FrameAssembler` 按【精确源时间戳】
        装配好后传进来，所以这里不再做任何配对，源错位按构造恒为 0。

        与基座那条通路【唯一】的结构差别：**不加杠杆臂**。Vicon 的 arm_ee
        刚体报的 pose/twist 就在标记簇原点，而控制点就是这个标记原点，两点
        重合，w x r 恒等于零。config 里 arm_ee_from_link6 的 17.9 mm 是给
        【FK 一侧】用的（把 FK 从 link6 挪到标记点，实现在
        EeFeedbackEstimator.control_point_* 里），在这里再搬一次就是搬两次。

        返回 ``(v, a, used_twist, used_accel, skew)``。
        """
        v = local_v.copy()
        a = local_a.copy()
        used_twist = used_accel = False
        skew_sec = 0.0          # 同帧装配：源错位按构造为 0

        if twist is None and accel is None:
            return v, a, used_twist, used_accel, skew_sec

        r_ctrl_from_vicon = self._r_ctrl_from_vicon()

        # 角速度：转进控制器系【之后】再与 arm_ee 位姿的本地差分比对。
        # arm_ee 相对 link6 差 114 度，但那是一个【常量】旋转，常量旋转不改变
        # 世界系角速度 —— 所以位姿差分出来的 ω 即使姿态本身不可用，作为裁判
        # 依然成立。
        omega = local_v[3:]
        self._ee_vicon_angular_ok = False
        if twist is not None and self.prefer_ee_vicon_angular_velocity:
            candidate = r_ctrl_from_vicon @ twist[3:]
            ok, gap = self._cross_check(
                candidate, local_v[3:], self.angular_cross_check_tol,
                self.angular_cross_check_floor)
            if ok:
                omega = candidate
                self._ee_vicon_angular_ok = True
            else:
                self._ee_angular_reject += 1
                self.get_logger().warning(
                    "arm_ee 的 Vicon 角速度与位姿差分不符（|Δ|=%.3f rad/s，"
                    "参考 %.3f）：本拍回退本地差分。"
                    % (gap, float(np.linalg.norm(local_v[3:]))),
                    throttle_duration_sec=5.0)

        alpha = local_a[3:]
        self._ee_vicon_angular_accel_ok = False
        if accel is not None and self.prefer_ee_vicon_angular_accel:
            cand_a = r_ctrl_from_vicon @ accel[3:]
            ok_a, gap_a = self._cross_check(
                cand_a, local_a[3:], self.angular_cross_check_tol,
                self.angular_accel_cross_check_floor)
            if ok_a:
                alpha = cand_a
                self._ee_vicon_angular_accel_ok = True
            else:
                self._ee_angular_accel_reject += 1
                self.get_logger().warning(
                    "arm_ee 的 Vicon 角加速度与位姿二阶差分不符"
                    "（|Δ|=%.3f rad/s^2，参考 %.3f）：本拍回退本地差分。"
                    % (gap_a, float(np.linalg.norm(local_a[3:]))),
                    throttle_duration_sec=5.0)

        if twist is not None and self.prefer_vicon_twist:
            # 无杠杆臂 —— 直接换系，不调用 shift_twist_accel。
            v = np.concatenate([r_ctrl_from_vicon @ twist[:3], omega])
            used_twist = True
        if accel is not None and self.prefer_vicon_linear_accel:
            a = np.concatenate([r_ctrl_from_vicon @ accel[:3], alpha])
            used_accel = True
        return v, a, used_twist, used_accel, skew_sec

    def _ee_cb(self, msg: PoseStamped) -> None:
        """arm_ee 位姿：换进控制器世界系后交给装配器。

        只做世界系变换，【不】乘 rigid_from_base —— 那是基座刚体到 base_link
        的外参，与末端刚体无关。

        这里【不再】就地去找同帧 twist/accel：实测它们恒晚于 pose 到达，
        那样找 100% 找不到。改为交给 FrameAssembler，等本帧的通道齐了再发。
        """
        if self.ee_pub is None:
            return
        arrival = self.get_clock().now().nanoseconds * 1e-9
        aligned = self._ee_clock.align(
            arrival, msg.header.stamp.sec, msg.header.stamp.nanosec)
        p_vr = np.array([msg.pose.position.x, msg.pose.position.y,
                         msg.pose.position.z])
        if not np.all(np.isfinite(p_vr)):
            return
        try:
            q_vr = _normalize_quat_xyzw(np.array([
                msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w]))
        except ValueError:
            return
        if self.zero_on_first_pose and self.zero_p is None:
            return          # 世界系原点还没被基座定义，先不发
        r_wv = _rotation_matrix_xyzw(self.q_wv)
        p_w = self.t_wv + r_wv @ p_vr
        q_w = _quat_multiply_xyzw(self.q_wv, q_vr)
        if self.zero_on_first_pose:
            q_inv = _quat_conjugate_xyzw(self.zero_q)
            p_w = _rotation_matrix_xyzw(q_inv) @ (p_w - self.zero_p)
            q_w = _normalize_quat_xyzw(_quat_multiply_xyzw(q_inv, q_w))

        pose_sample = {
            "p_w": p_w, "q_w": q_w,
            "sample_time": aligned.sample_time,
            "stamp_local": aligned.stamp_local,
            "stamp_source": float(msg.header.stamp.sec)
                + 1e-9 * float(msg.header.stamp.nanosec),
            "orientation": msg.pose.orientation,
        }
        if not aligned.used_source:
            # 没有源时刻就无法与任何通道同帧对齐，等下去毫无意义：立即处理，
            # 保持改造前的降级行为（辅助通道自然缺席，回退本地差分）。
            self._emit_ee_frame(
                AssembledFrame(key=(0, 0), samples={"pose": pose_sample},
                               complete=False, wait_sec=0.0, missing=()),
                arrival)
            return
        for frame in self._ee_frames.submit(
                "pose", self._frame_key(msg), pose_sample, arrival):
            self._emit_ee_frame(frame, arrival)

    def _emit_ee_frame(self, frame, arrival: float) -> None:
        """装配好的 arm_ee 整帧 -> 一条 FloatingBaseState。"""
        pose_sample = frame.samples.get("pose")
        if pose_sample is None:
            return
        p_w = pose_sample["p_w"]
        q_w = pose_sample["q_w"]
        sample_time = pose_sample["sample_time"]
        self._ee_frame_wait = frame.wait_sec

        # ---- 本地差分。差的是【已经转进控制器世界系】的 p/q，所以结果天然
        # 与转换后的 Vicon 原生量同系，对照才有意义。
        local_v = np.zeros(6)
        local_a = np.zeros(6)
        derivatives_ok = False
        if (self._ee_prev_t is not None and self._ee_prev_p is not None
                and self._ee_prev_q is not None):
            dt = sample_time - self._ee_prev_t
            if 1e-4 <= dt <= self.max_gap:
                derivatives_ok = True
                raw_v = np.concatenate([
                    (p_w - self._ee_prev_p) / dt,
                    _rotation_vector_xyzw(_quat_multiply_xyzw(
                        q_w, _quat_conjugate_xyzw(self._ee_prev_q))) / dt,
                ])
                beta = 1.0 if self.tau_filter <= 0.0 else dt / (self.tau_filter + dt)
                local_v = self._ee_prev_v + beta * (raw_v - self._ee_prev_v)
                raw_a = (local_v - self._ee_prev_v) / dt
                local_a = self._ee_prev_a + beta * (raw_a - self._ee_prev_a)
        self._ee_prev_t, self._ee_prev_p, self._ee_prev_q = \
            sample_time, p_w.copy(), q_w.copy()
        self._ee_prev_v, self._ee_prev_a = local_v.copy(), local_a.copy()

        v, a, self._ee_used_vicon_twist, self._ee_used_vicon_accel, \
            self._ee_channel_skew = self._ee_vicon_twist_accel(
                frame.samples.get("twist"), frame.samples.get("accel"),
                local_v, local_a)

        # 缺数据【绝不发零】：0 是下游任何守卫都识别不出来的合法值。短暂缺失
        # 保持上一有效加速度，超过 accel_hold_sec 就整条不发 —— 由运行器既有
        # 且已测的陈旧性守卫接管（core/base_state.py 的 latest 抛 stale）。
        valid = self._ee_used_vicon_accel or derivatives_ok
        if valid:
            self._ee_last_good_accel = a.copy()
            self._ee_last_good_accel_time = sample_time
        elif (self._ee_last_good_accel is not None
              and sample_time - (self._ee_last_good_accel_time or 0.0)
              <= self.accel_hold_sec):
            a = self._ee_last_good_accel.copy()
        else:
            self._ee_withheld += 1
            if not self._ee_startup_logged:
                # 刚起来还没有第二帧，导数无从谈起。等一帧（约 8 ms）即可。
                self._ee_startup_logged = True
                self.get_logger().info(
                    "等待第二帧以建立 arm_ee 导数估计（起步期，非故障）")
            return

        out = FloatingBaseState()
        # 采集时刻，不是装配完成时刻 —— 下游的陈旧性守卫要比的是"这个测量
        # 有多老"，装配那 0.16 ms 的等待本来就该算进年龄里。
        out.header.stamp = Time(
            nanoseconds=int(round(pose_sample["stamp_local"] * 1e9))).to_msg()
        out.header.frame_id = "world"
        out.pose.position.x, out.pose.position.y, out.pose.position.z = \
            [float(x) for x in p_w]
        # 姿态原样透传（VRPN 原生），仅供记录残差：arm_ee 相对 link6 差 114 度，
        # 且扣掉常量后的残差两次标定间差 4 倍，控制回路永不使用它。差分用的是
        # 上面换过系的 q_w，两者刻意分开 —— 换系会改变记录下来的残差数值，而
        # 那个数值是标定时定下的口径。
        out.pose.orientation = pose_sample["orientation"]
        out.twist.linear.x, out.twist.linear.y, out.twist.linear.z = \
            [float(x) for x in v[:3]]
        out.twist.angular.x, out.twist.angular.y, out.twist.angular.z = \
            [float(x) for x in v[3:]]
        out.acceleration.linear.x, out.acceleration.linear.y, \
            out.acceleration.linear.z = [float(x) for x in a[:3]]
        out.acceleration.angular.x, out.acceleration.angular.y, \
            out.acceleration.angular.z = [float(x) for x in a[3:]]
        publish_time = self.get_clock().now().nanoseconds * 1e-9
        self.timing_pub.publish(Float64MultiArray(data=[
            2.0, float(pose_sample["stamp_source"]),
            float(out.header.stamp.sec + 1e-9 * out.header.stamp.nanosec),
            float(publish_time),
        ]))
        self.ee_pub.publish(out)

    def _vicon_twist_accel(self, twist, accel, q_vr, local_v, local_a):
        """Vicon 原生 twist/accel，已搬到 base_link 并转进控制器世界系。

        ``twist`` / ``accel`` 由 :class:`FrameAssembler` 按【精确源时间戳】
        装配好后传进来。此前这里用 ``pair_by_source_time`` 去缓冲区现找，
        100% 配不上（同帧 twist/accel 恒晚于 pose 到达），所以这四个
        ``prefer_vicon_*`` 开关一直是装饰品。

        返回 ``(v, a, used_twist, used_accel, skew)``；任一通道拿不到就退回
        对应的本地差分值。
        """
        v = local_v.copy()
        a = local_a.copy()
        used_twist = used_accel = False
        skew_sec = 0.0          # 同帧装配：源错位按构造为 0

        if twist is None and accel is None:
            return v, a, used_twist, used_accel, skew_sec

        # ---- 坐标系换算。下面两处的参考系选择是判据成立的前提，不可省略。
        # 世界系换算见 _r_ctrl_from_vicon 的 docstring（末端通路共用同一个）。
        #
        # 杠杆臂是【基座独有】的：它是「标记簇原点 -> base_link」，用【基座】外参
        # t_rb（33.1 mm，不是 arm_ee_from_link6 的 17.9 mm）。t_rb 表达在【刚体系】里，
        # 所以要先经 R(q_vr) 转到 vicon_world，再转进控制器系：
        #     r_ctrl = R_ctrl_from_vicon @ R(q_vr) @ t_rb
        # 与 _transform_pose 里 p_wb 的杠杆项逐项对应。此处不能用 q_wb —— 它已经乘过
        # q_rb(85.3°) 与归零旋转，会给杠杆臂多转 85°。
        r_ctrl_from_vicon = self._r_ctrl_from_vicon()
        r_world = r_ctrl_from_vicon @ _rotation_matrix_xyzw(q_vr) @ self.t_rb

        # 角速度默认取 Vicon，但要先过一道【与位姿差分的对照】。
        #
        # 实测本机原始字段（arm_base 静止）：
        #     vel_quat    = [0.000000, -0.026500, 0.178300, -0.033800]  ||q|| != 1
        #     vel_quat_dt = 1.000000000     <- vrpn_Tracker 构造函数的缺省值
        # 字段【是有数据的】、也确实跟着刚体走；没有确立的是它的单位与约定
        # （四元数不归一、dt 还是库默认值）。所以采信它、但用本地差分当裁判：
        # 两者粗量级不符就这一拍回退本地差分并记录，而不是盲信也不是一刀切禁用。
        omega = local_v[3:]
        self._vicon_angular_ok = False
        if twist is not None and self.prefer_vicon_angular_velocity:
            # 必须转进控制器系【之后】再与本地差分比：跨坐标系相减出来的
            # 一致性判据本身没有意义。
            # listener 发布的槽 1..3 是刚体系角速度。先用同帧刚体姿态把
            # 它转回 Vicon 世界系，再转进控制器世界系。
            candidate = (r_ctrl_from_vicon @ _rotation_matrix_xyzw(q_vr)
                         @ twist[3:])
            reference = local_v[3:]
            gap = float(np.linalg.norm(candidate - reference))
            scale = max(float(np.linalg.norm(reference)), self.angular_cross_check_floor)
            if gap <= self.angular_cross_check_tol * scale:
                omega = candidate
                self._vicon_angular_ok = True
            else:
                self._angular_reject += 1
                self.get_logger().warning(
                    "Vicon 角速度与位姿差分不符（|Δ|=%.3f rad/s，参考 %.3f）："
                    "本拍回退本地差分。vel_quat 的单位/约定仍待核实。"
                    % (gap, float(np.linalg.norm(reference))),
                    throttle_duration_sec=5.0)
        # 角加速度同样过一道对照。它直接进 compute_task_bias_acceleration 变
        # 力矩：QA_record 量过，16 rad/s^2 的伪角加速度在 0.3 m 力臂上就是
        # 4.8 m/s^2 的伪末端需求，所以宁可回退也不能放一个错的量进去。
        alpha = local_a[3:]
        self._vicon_angular_accel_ok = False
        if accel is not None and self.prefer_vicon_angular_accel:
            cand_a = (r_ctrl_from_vicon @ _rotation_matrix_xyzw(q_vr)
                      @ accel[3:])
            ref_a = local_a[3:]
            gap_a = float(np.linalg.norm(cand_a - ref_a))
            scale_a = max(float(np.linalg.norm(ref_a)),
                          self.angular_accel_cross_check_floor)
            if gap_a <= self.angular_cross_check_tol * scale_a:
                alpha = cand_a
                self._vicon_angular_accel_ok = True
            else:
                self._angular_accel_reject += 1
                self.get_logger().warning(
                    "Vicon 角加速度与位姿二阶差分不符（|Δ|=%.3f rad/s^2，"
                    "参考 %.3f）：本拍回退本地差分。"
                    % (gap_a, float(np.linalg.norm(ref_a))),
                    throttle_duration_sec=5.0)

        if twist is not None and self.prefer_vicon_twist:
            v_lin, _ = shift_twist_accel(r_ctrl_from_vicon @ twist[:3],
                                         omega, np.zeros(3), np.zeros(3), r_world)
            v = np.concatenate([v_lin, omega])
            used_twist = True
        if accel is not None and self.prefer_vicon_linear_accel:
            _, a_lin = shift_twist_accel(np.zeros(3), omega,
                                         r_ctrl_from_vicon @ accel[:3],
                                         alpha, r_world)
            a_ang = alpha if self.prefer_vicon_angular_accel else local_a[3:]
            a = np.concatenate([a_lin, a_ang])
            used_accel = True
        return v, a, used_twist, used_accel, skew_sec

    def _publish_diagnostics(self, pose_age, twist_age, accel_age, degraded):
        msg = Float64MultiArray()
        msg.data = [
            float(pose_age), float(twist_age), float(accel_age),
            float(self._channel_skew),
            1.0 if self._used_vicon_twist else 0.0,
            1.0 if self._used_vicon_accel else 0.0,
            1.0 if degraded else 0.0,
            float(self._consistency_gap[0]), float(self._consistency_gap[1]),
            float(self._withheld),
            1.0 if self._vicon_angular_ok else 0.0,
            float(self._angular_reject),
            1.0 if self._vicon_angular_accel_ok else 0.0,
            float(self._angular_accel_reject),
            # ---- 索引 14 起是 arm_ee 通路。追加在尾部而不是插进中间：
            # 已有的看板/脚本按位置读前 14 个。
            1.0 if self._ee_used_vicon_twist else 0.0,
            1.0 if self._ee_used_vicon_accel else 0.0,
            float(self._ee_channel_skew),
            1.0 if self._ee_vicon_angular_ok else 0.0,
            float(self._ee_angular_reject),
            1.0 if self._ee_vicon_angular_accel_ok else 0.0,
            float(self._ee_angular_accel_reject),
            float(self._ee_withheld),
            # ---- 索引 22/23：整帧装配的实际等待（秒）。这是这次改动引入的
            # 全部额外延迟，实测应在 0.2 ms 量级。它取代了"错位 8 ms 配不上"
            # 的老状态，所以是这条修复最直接的可观测量。
            float(self._frame_wait),
            float(self._ee_frame_wait),
            # 索引 24/25：装配超时放行的次数。正常链路应长期为 0；持续增长
            # 说明某条通道时断时续。
            float(self._base_frames.timeouts),
            float(self._ee_frames.timeouts),
        ]
        self.diag_pub.publish(msg)

    def _publish_stats(self, arrival: float) -> None:
        if self.stats_interval <= 0.0:
            return
        if self._stats_start is None:
            self._stats_start = arrival
            return
        span = arrival - self._stats_start
        if span < self.stats_interval or self._stats_count == 0:
            return
        # 第一个窗口是【预热】，不要当验收依据：节点起来时 DDS 发现和 Python
        # 预热会卡住约 1 s，深队列如实缓存那一段，于是这批帧的年龄天然很大。
        # 它们确实很旧，运行器按年龄丢掉就对了 —— 但拿它做验收会误判链路。
        warmup = " (预热窗口，不作验收依据)" if self._stats_windows == 0 else ""
        self.get_logger().info(
            "Vicon %.1f Hz over %.0fs | transport latency mean %.1f ms, max %.1f ms "
            "| source-clock gaps %d%s"
            % (self._stats_count / span, span,
               1e3 * self._stats_latency_sum / self._stats_count,
               1e3 * self._stats_latency_max, self._stats_dropped, warmup))
        self._stats_windows += 1
        self._stats_start = arrival
        self._stats_count = 0
        self._stats_dropped = 0
        self._stats_latency_sum = 0.0
        self._stats_latency_max = 0.0

    def _pose_cb(self, msg: PoseStamped) -> None:
        arrival = self.get_clock().now().nanoseconds * 1e-9
        try:
            p, q = self._transform_pose(msg)
        except (ValueError, FloatingPointError) as exc:
            self.get_logger().warning("Discarded invalid Vicon pose: %s" % exc)
            return
        if not (np.all(np.isfinite(p)) and np.all(np.isfinite(q))):
            self.get_logger().warning("Discarded non-finite Vicon pose")
            return

        # sample_time 是求导所用的时钟：有 Vicon 帧时钟就用它，否则退回到达时间。
        # stamp_local 是同一时刻在本机时钟下的表达，控制器的陈旧性判据比的是它。
        source = self._source_time(msg)
        if source is None:
            sample_time = arrival
            stamp_local = arrival
            latency = 0.0
        else:
            sample_time = source
            offset = self._clock_offset(arrival, source)
            stamp_local = source + offset
            latency = max(0.0, arrival - stamp_local)
            if not self._offset_logged:
                self._offset_logged = True
                self.get_logger().info(
                    "Vicon source clock in use: offset %.3f s (%.4f h) from local"
                    % (offset, offset / 3600.0))

        pose_sample = {
            "p": p, "q": q, "q_vr": self._last_q_vr.copy(),
            "sample_time": sample_time, "stamp_local": stamp_local,
            "stamp_source": float(msg.header.stamp.sec)
                + 1e-9 * float(msg.header.stamp.nanosec),
            "latency": latency,
        }
        if source is None:
            # 没有源时刻 = 无法与任何辅助通道同帧对齐，等下去毫无意义。
            self._emit_base_frame(
                AssembledFrame(key=(0, 0), samples={"pose": pose_sample},
                               complete=False, wait_sec=0.0, missing=()),
                arrival)
            return
        for frame in self._base_frames.submit(
                "pose", self._frame_key(msg), pose_sample, arrival):
            self._emit_base_frame(frame, arrival)

    def _emit_base_frame(self, frame, arrival: float) -> None:
        """装配好的基座整帧 -> 一条 FloatingBaseState。

        触发点是"本帧的所有已订阅通道都齐了"，不是"pose 到了"。实测 accel
        恒为最后一个到达的通道，但这里【不依赖】那个顺序 —— 顺序变了也不会
        坏，装配器只看齐不齐。
        """
        pose_sample = frame.samples.get("pose")
        if pose_sample is None:
            return
        p = pose_sample["p"]
        q = pose_sample["q"]
        sample_time = pose_sample["sample_time"]
        stamp_local = pose_sample["stamp_local"]
        latency = pose_sample["latency"]
        self._frame_wait = frame.wait_sec

        v = np.zeros(6)
        a = np.zeros(6)
        derivatives_ok = False
        if self.prev_t is not None and self.prev_p is not None and self.prev_q is not None:
            dt = sample_time - self.prev_t
            if 1e-4 <= dt <= self.max_gap:
                derivatives_ok = True
                raw_v = np.concatenate([
                    (p - self.prev_p) / dt,
                    _rotation_vector_xyzw(_quat_multiply_xyzw(q, _quat_conjugate_xyzw(self.prev_q))) / dt,
                ])
                alpha = 1.0 if self.tau_filter <= 0.0 else dt / (self.tau_filter + dt)
                v = self.prev_v + alpha * (raw_v - self.prev_v)
                raw_a = (v - self.prev_v) / dt
                a = self.prev_a + alpha * (raw_a - self.prev_a)
            else:
                # 按源时钟判定，这里只对真实丢帧触发（实测 2.8%），
                # 不会被投递抖动误触发。
                self._stats_dropped += 1
                self.get_logger().warning(
                    "Vicon sample gap %.3fs; reset derivative estimates" % dt)

        self.prev_t, self.prev_p, self.prev_q = sample_time, p.copy(), q.copy()
        self.prev_v, self.prev_a = v.copy(), a.copy()

        # 优先用 Vicon 原生 twist/accel；本地差分退居"缺帧回退 + 一致性诊断"。
        local_v, local_a = v.copy(), a.copy()
        v, a, self._used_vicon_twist, self._used_vicon_accel, self._channel_skew = \
            self._vicon_twist_accel(
                frame.samples.get("twist"), frame.samples.get("accel"),
                pose_sample["q_vr"], local_v, local_a)
        if self._used_vicon_twist:
            self._consistency_gap = self._consistency.update(
                sample_time, v[:3], local_v[:3], v[3:], local_v[3:])

        # 缺加速度【绝不】喂零：0 是一个下游任何守卫都识别不出来的合法值。
        # 短暂缺失保持上一有效值，超过 accel_hold_sec 就整条不发 —— 让运行器
        # 既有且已测的陈旧性守卫（base_state.py:139 抛错 -> 控制环不发指令）
        # 自动做正确的事，不需要给 FloatingBaseState 加 validity 字段。
        accel_valid = self._used_vicon_accel or derivatives_ok
        if accel_valid:
            self._last_good_accel = a.copy()
            self._last_good_accel_time = sample_time
        elif (self._last_good_accel is not None
              and sample_time - (self._last_good_accel_time or 0.0) <= self.accel_hold_sec):
            a = self._last_good_accel.copy()
        else:
            self._withheld += 1
            if self.prev_v_valid:
                # 有过数据又丢了 —— 这是真空档，值得一条警告。
                self.get_logger().warning(
                    "基座加速度缺失超过 %.3f s：整条 FloatingBaseState 不发，"
                    "由运行器的陈旧性守卫接管" % self.accel_hold_sec)
            elif not self._startup_logged:
                # 刚起来还没有第二帧，导数无从谈起。等一帧（约 8 ms）即可，
                # 拿零当测量值发下去才是错的。
                self._startup_logged = True
                self.get_logger().info(
                    "等待第二帧以建立基座导数估计（起步期，非故障）")
            self._publish_diagnostics(latency, math.inf, math.inf, True)
            return
        self.prev_v_valid = True

        if self._consistency.failed:
            self._withheld += 1
            self.get_logger().warning(
                "Vicon twist 与本地差分持续不一致（线 %.3f m/s、角 %.3f rad/s）："
                "无法判断哪一路可信，停发基座状态"
                % self._consistency_gap)
            self._publish_diagnostics(latency, math.inf, math.inf, True)
            return

        self._stats_count += 1
        self._stats_latency_sum += latency
        self._stats_latency_max = max(self._stats_latency_max, latency)
        self._publish_stats(arrival)
        out = FloatingBaseState()
        out.header.stamp = Time(
            nanoseconds=int(round(stamp_local * 1e9))).to_msg()
        out.header.frame_id = "world"
        out.pose.position.x, out.pose.position.y, out.pose.position.z = [float(x) for x in p]
        out.pose.orientation.x, out.pose.orientation.y = float(q[0]), float(q[1])
        out.pose.orientation.z, out.pose.orientation.w = float(q[2]), float(q[3])
        out.twist.linear.x, out.twist.linear.y, out.twist.linear.z = [float(x) for x in v[:3]]
        out.twist.angular.x, out.twist.angular.y, out.twist.angular.z = [float(x) for x in v[3:]]
        out.acceleration.linear.x, out.acceleration.linear.y, out.acceleration.linear.z = [float(x) for x in a[:3]]
        out.acceleration.angular.x, out.acceleration.angular.y, out.acceleration.angular.z = [float(x) for x in a[3:]]
        publish_time = self.get_clock().now().nanoseconds * 1e-9
        self.timing_pub.publish(Float64MultiArray(data=[
            1.0, float(frame.samples["pose"]["stamp_source"]),
            float(out.header.stamp.sec + 1e-9 * out.header.stamp.nanosec),
            float(publish_time),
        ]))
        self.pub.publish(out)

        # 同帧装配后"通道年龄"退化成"这一帧有没有这条通道"：有就是 0（同一
        # 个源时刻），没有就是 inf。真正值得看的是装配等了多久 —— 那个数在
        # 诊断数组尾部（索引 22）。
        twist_age = 0.0 if "twist" in frame.samples else math.inf
        accel_age = 0.0 if "accel" in frame.samples else math.inf
        self._publish_diagnostics(latency, twist_age, accel_age, False)


def main(args=None):
    rclpy.init(args=args)
    node = ViconFloatingBaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
