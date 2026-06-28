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
    "front_propeller_line",
    "circle",
    "windylab",
)

# Lightweight front-propeller-line anchors. Keep x at the propeller reference
# plane; _build_path() adds the forward offset explicitly.
_FRONT_PROP_LEFT_CENTER = np.array([0.0, 0.230, 0.0], dtype=float)
_FRONT_PROP_RIGHT_CENTER = np.array([0.0, -0.230, 0.0], dtype=float)


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


def _stroke_text_points(text: str, x: float, y0: float, z0: float, height: float, spacing: float):
    glyphs = {
        "w": [(0, 1), (0.18, 0), (0.36, 0.55), (0.54, 0), (0.72, 1)],
        "i": [(0.36, 0), (0.36, 0.72), (0.36, 0.95), (0.36, 1.0)],
        "n": [(0, 0), (0, 1), (0.55, 1), (0.55, 0)],
        "d": [(0.55, 0), (0.55, 1), (0.05, 1), (0, 0.75), (0, 0.25), (0.05, 0), (0.55, 0)],
        "y": [(0, 1), (0.28, 0.45), (0.55, 1), (0.28, 0.45), (0.20, 0)],
        "l": [(0.28, 1), (0.28, 0), (0.55, 0)],
        "a": [(0, 0), (0.0, 0.85), (0.55, 0.85), (0.55, 0), (0.55, 0.45), (0.0, 0.45)],
        "b": [(0, 0), (0, 1), (0.48, 1), (0.55, 0.75), (0.48, 0.55), (0, 0.55),
              (0.48, 0.55), (0.55, 0.25), (0.48, 0), (0, 0)],
    }
    width = height * 0.55
    pen_y = y0
    points = []
    last = None
    for char in text:
        raw = glyphs.get(char.lower())
        if raw is None:
            pen_y += width + spacing
            continue
        char_points = [
            np.array([x, pen_y + width * px, z0 + height * pz], dtype=float)
            for px, pz in raw
        ]
        if last is not None:
            points.append(last)
            points.append(char_points[0])
        points.extend(char_points)
        last = char_points[-1]
        pen_y += width + spacing
    return points


def _build_path(mode: str, count: int) -> np.ndarray:
    if mode == "circle":
        center = np.array([0.7, 0.0, 0.10], dtype=float)
        radius_y = 0.08
        radius_z = 0.06
        points = []
        for i in range(count):
            theta = 2.0 * math.pi * i / count
            points.append(center + np.array([0.0, radius_y * math.cos(theta), radius_z * math.sin(theta)]))
        return np.array(points, dtype=float)
    if mode == "lower_obstacle_circle":
        center = np.array([0.7, 0.02, -0.21], dtype=float)
        radius = 0.08
        points = []
        for i in range(count):
            theta = 2.0 * math.pi * i / count
            points.append(center + np.array([radius * math.cos(theta), radius * math.sin(theta), 0.0]))
        return np.array(points, dtype=float)
    if mode == "windylab":
        points = _stroke_text_points("windylab", x=0.52, y0=-0.22, z0=0.04, height=0.10, spacing=0.018)
        return _resample_polyline(points, count)
    if mode == "avoidance_sweep":
        points = [
            [0.48, -0.20, 0.12],
            [0.42, -0.08, 0.02],
            [0.36, 0.00, -0.02],
            [0.42, 0.08, 0.02],
            [0.48, 0.20, 0.12],
            [0.56, 0.12, 0.16],
            [0.58, 0.00, 0.18],
            [0.56, -0.12, 0.16],
            [0.48, -0.20, 0.12],
        ]
        return _resample_polyline(points, count)
    if mode == "front_propeller_line":
        left_center = _FRONT_PROP_LEFT_CENTER
        right_center = _FRONT_PROP_RIGHT_CENTER
        prop_center = 0.5 * (left_center + right_center)
        x = float(prop_center[0] + 0.315)
        z = float(prop_center[2] + 0.22)
        y_extension = 0.02
        start = np.array([x, min(left_center[1], right_center[1]) - y_extension, z], dtype=float)
        end = np.array([x, max(left_center[1], right_center[1]) + y_extension, z], dtype=float)
        return _resample_polyline([start, end, start], count)
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
        if line.points:
            line.points.append(line.points[0])
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
