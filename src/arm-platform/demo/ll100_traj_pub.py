#!/usr/bin/env python3
"""LL100-LH420 trajectory publisher for goal-based NMPC demos.

It publishes one /goal_pose waypoint at a time and advances when
/ll100_nmpc_goal/reached reports success.
"""

import math

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


PUBLISH_RATE_HZ = 50.0
SUPPORTED_PATH_MODES = (
    "lower_obstacle_circle",
    "avoidance_sweep",
    "front_wing_arc",
    "circle",
)

LL100_CIRCLE_CENTER = np.array([0.70, 0.0, 0.10], dtype=float)
LL100_CIRCLE_RADIUS = 0.08
LL100_LOWER_OBSTACLE_CIRCLE_CENTER = np.array([0.70, 0.02, -0.21], dtype=float)
LL100_LOWER_OBSTACLE_CIRCLE_RADIUS = 0.08
# front_wing_arc = 右桨盘【下方】的扫掠弧线避障演示。两端点都在桨盘下方(z=-0.22),中段沿 z
# 抬升到 z≈-0.14(进入 3cm 斥力避障区)→ 手臂想升向桨盘、被避障顶在离桨 ~30mm 处,抬升段出现
# 可见跟踪偏移(那就是避障在起作用)。全程 viol=0、真臂不碰桨。
# 前提:front_obstacle_capsule=True(前桨按连杆 capsule 建模)+ 斥力场默认开(见 demo 默认参数)。
LL100_FRONT_WING_ARC_LEFT = np.array([0.62, -0.24, -0.22], dtype=float)   # 桨下方·外侧端
LL100_FRONT_WING_ARC_RIGHT = np.array([0.54, -0.06, -0.22], dtype=float)  # 桨下方·内侧端
LL100_FRONT_WING_ARC_RISE = 0.08  # 中段沿 z 抬向桨盘(升到 z≈-0.14,进避障区)


def _lerp(a: np.ndarray, b: np.ndarray, alpha: float) -> np.ndarray:
    return (1.0 - alpha) * a + alpha * b


def _resample_polyline(points, count: int) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    if len(pts) < 2:
        return np.repeat(pts[:1], count, axis=0)
    segment_lengths = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    total = float(np.sum(segment_lengths))
    if total < 1e-12:
        return np.repeat(pts[:1], count, axis=0)
    distances = np.concatenate([[0.0], np.cumsum(segment_lengths)])
    samples = np.linspace(0.0, total, count, endpoint=False)
    result = []
    seg = 0
    for distance in samples:
        while seg + 1 < len(segment_lengths) and distance > distances[seg + 1]:
            seg += 1
        length = max(segment_lengths[seg], 1e-12)
        alpha = (distance - distances[seg]) / length
        result.append(_lerp(pts[seg], pts[seg + 1], alpha))
    return np.array(result, dtype=float)


def _build_path(mode: str, count: int) -> np.ndarray:
    if mode == "circle":
        points = []
        for i in range(count):
            theta = 2.0 * math.pi * i / count
            points.append(
                LL100_CIRCLE_CENTER
                + np.array(
                    [
                        0.0,
                        LL100_CIRCLE_RADIUS * math.cos(theta),
                        LL100_CIRCLE_RADIUS * math.sin(theta),
                    ],
                    dtype=float,
                )
            )
        return np.array(points, dtype=float)
    if mode == "lower_obstacle_circle":
        points = []
        for i in range(count):
            theta = 2.0 * math.pi * i / count
            points.append(
                LL100_LOWER_OBSTACLE_CIRCLE_CENTER
                + np.array(
                    [
                        LL100_LOWER_OBSTACLE_CIRCLE_RADIUS * math.cos(theta),
                        LL100_LOWER_OBSTACLE_CIRCLE_RADIUS * math.sin(theta),
                        0.0,
                    ],
                    dtype=float,
                )
            )
        return np.array(points, dtype=float)
    if mode == "avoidance_sweep":
        # 前部下沉段(原 x~0.42, z~0.10)会把臂带进近奇异区:实测该处 manip 最低、跟踪误差最大。
        # 因此把前段外推到 x>=0.49、抬到 z~0.14,使整圈停留在高可操作度区(实测 manip 0.008+),
        # 仍是机身/安装架前方的扫掠演示,但远离奇异、关节速度需求更低。
        points = [
            [0.50, -0.18, 0.15],
            [0.49, -0.07, 0.14],
            [0.49, 0.00, 0.14],
            [0.49, 0.07, 0.14],
            [0.50, 0.18, 0.15],
            [0.56, 0.12, 0.16],
            [0.58, 0.00, 0.17],
            [0.56, -0.12, 0.16],
            [0.50, -0.18, 0.15],
        ]
        return _resample_polyline(points, count)
    if mode == "front_wing_arc":
        # 桨下扫掠弧:两端点都在桨盘下方,中段沿 z 抬向桨盘、进入斥力避障区 → 避障把手臂顶在
        # 离桨 ~30mm 处(抬升段出现可见跟踪偏移=避障在起作用)。既是桨下弧线、又真验证避障。
        pts = []
        for i in range(count):
            phase = 2.0 * i / max(1, count)                  # 0..2
            tau = phase if phase <= 1.0 else 2.0 - phase      # 0..1..0
            s = 0.5 * (1.0 - math.cos(math.pi * tau))          # 余弦缓入缓出
            p = (1.0 - s) * LL100_FRONT_WING_ARC_LEFT + s * LL100_FRONT_WING_ARC_RIGHT
            p = p.copy()
            p[2] += LL100_FRONT_WING_ARC_RISE * math.sin(math.pi * s)   # 中段抬向桨盘(激活避障)
            pts.append(p)
        return np.array(pts, dtype=float)
    raise ValueError(f"path_mode 必须是 {SUPPORTED_PATH_MODES}, 当前是 {mode!r}")


class LL100TrajPub(Node):
    def __init__(self):
        super().__init__("ll100_traj_pub")
        self.declare_parameter("path_mode", "lower_obstacle_circle")
        self.declare_parameter("period_sec", 60.0)
        self.declare_parameter("waypoint_count", 60)
        self.declare_parameter("waypoint_timeout_sec", 10.0)
        self.declare_parameter("frame_id", "base_link")

        self.path_mode = str(self.get_parameter("path_mode").value)
        period_sec = float(self.get_parameter("period_sec").value)
        waypoint_count = int(self.get_parameter("waypoint_count").value)
        self.waypoint_timeout_sec = float(self.get_parameter("waypoint_timeout_sec").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        if self.path_mode not in SUPPORTED_PATH_MODES:
            raise ValueError(f"path_mode 必须是 {SUPPORTED_PATH_MODES}, 当前是 {self.path_mode!r}")
        if period_sec <= 0.0:
            raise ValueError("period_sec 必须 > 0")
        if waypoint_count < 2:
            raise ValueError("waypoint_count 必须 >= 2")
        if self.waypoint_timeout_sec <= 0.0:
            raise ValueError("waypoint_timeout_sec 必须 > 0")

        dt = 1.0 / PUBLISH_RATE_HZ
        count = max(20, waypoint_count)
        self.path = _build_path(self.path_mode, count)
        self.path_index = 0
        self.last_advance_time = self.get_clock().now()
        self.startup_delay_steps = int(PUBLISH_RATE_HZ)
        self.step_count = 0

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.path_marker_pub = self.create_publisher(MarkerArray, "/ll100_traj_pub/markers", 10)
        self.create_subscription(Bool, "/ll100_nmpc_goal/reached", self._reached_callback, 10)
        self.timer = self.create_timer(dt, self.tick)
        self.path_marker_timer = self.create_timer(0.5, self._publish_path_markers)

        self.get_logger().info(
            "LL100 轨迹发布节点启动: "
            f"path_mode={self.path_mode}, points={len(self.path)}, "
            f"period_hint={period_sec:.1f}s, "
            f"waypoint_timeout={self.waypoint_timeout_sec:.1f}s"
        )

    def _reached_callback(self, msg: Bool):
        if msg.data:
            self._advance("reached")

    def tick(self):
        self.step_count += 1
        if self.step_count < self.startup_delay_steps:
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_advance_time).nanoseconds * 1e-9
        if elapsed > self.waypoint_timeout_sec:
            self.get_logger().warn(
                f"超时 {elapsed:.1f}s 未收到 reached,自动推进 path_index={self.path_index}",
                throttle_duration_sec=5.0,
            )
            self._advance("timeout")

        self._publish_current()

    def _advance(self, reason: str):
        self.path_index = (self.path_index + 1) % len(self.path)
        self.last_advance_time = self.get_clock().now()
        self.get_logger().info(
            f"{reason} 推进到路径点 {self.path_index}/{len(self.path)}: "
            f"{self.path[self.path_index].tolist()}"
        )

    def _publish_current(self):
        pt = self.path[self.path_index]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(pt[0])
        msg.pose.position.y = float(pt[1])
        msg.pose.position.z = float(pt[2])
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def _publish_path_markers(self):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = now
        line.ns = "planned_path"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.005
        line.color.g = 1.0
        line.color.a = 0.8
        line.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in self.path]
        markers.markers.append(line)

        pt = self.path[self.path_index]
        cur = Marker()
        cur.header.frame_id = self.frame_id
        cur.header.stamp = now
        cur.ns = "current_waypoint"
        cur.id = 1
        cur.type = Marker.SPHERE
        cur.action = Marker.ADD
        cur.pose.position.x = float(pt[0])
        cur.pose.position.y = float(pt[1])
        cur.pose.position.z = float(pt[2])
        cur.pose.orientation.w = 1.0
        cur.scale.x = cur.scale.y = cur.scale.z = 0.04
        cur.color.r = 1.0
        cur.color.g = 1.0
        cur.color.a = 1.0
        markers.markers.append(cur)

        self.path_marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = LL100TrajPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
