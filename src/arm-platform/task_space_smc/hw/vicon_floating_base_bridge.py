#!/usr/bin/env python3
"""Convert a Vicon/VRPN rigid-body pose into ``FloatingBaseState``.

The VRPN listener publishes pose messages with timestamps originating at the
Vicon server.  That clock need not be synchronized with the arm computer, so
this bridge intentionally timestamps its output at local receipt time.  It
also derives the twist and acceleration from consecutive poses in that same
clock domain, which gives the controller one coherent state sample.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from dummy_interface.msg import FloatingBaseState
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


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
    # q and -q represent the same rotation; use the shortest increment.
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

        self.prev_t: float | None = None
        self.prev_p: np.ndarray | None = None
        self.prev_q: np.ndarray | None = None
        self.prev_v = np.zeros(6)
        self.prev_a = np.zeros(6)
        self.zero_p: np.ndarray | None = None
        self.zero_q: np.ndarray | None = None

        self.pub = self.create_publisher(FloatingBaseState, self.output_topic, 10)
        self.sub = self.create_subscription(PoseStamped, self.pose_topic, self._pose_cb, 10)
        self.get_logger().info(
            "Vicon base bridge: %s -> %s (zero_on_first_pose=%s)"
            % (self.pose_topic, self.output_topic, self.zero_on_first_pose))

    def _transform_pose(self, msg: PoseStamped) -> tuple[np.ndarray, np.ndarray]:
        p_vr = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q_vr = _normalize_quat_xyzw(np.array([
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        ]))
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

    def _pose_cb(self, msg: PoseStamped) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        try:
            p, q = self._transform_pose(msg)
        except (ValueError, FloatingPointError) as exc:
            self.get_logger().warning("Discarded invalid Vicon pose: %s" % exc)
            return
        if not (np.all(np.isfinite(p)) and np.all(np.isfinite(q))):
            self.get_logger().warning("Discarded non-finite Vicon pose")
            return

        v = np.zeros(6)
        a = np.zeros(6)
        if self.prev_t is not None and self.prev_p is not None and self.prev_q is not None:
            dt = now - self.prev_t
            if 1e-4 <= dt <= self.max_gap:
                raw_v = np.concatenate([
                    (p - self.prev_p) / dt,
                    _rotation_vector_xyzw(_quat_multiply_xyzw(q, _quat_conjugate_xyzw(self.prev_q))) / dt,
                ])
                alpha = 1.0 if self.tau_filter <= 0.0 else dt / (self.tau_filter + dt)
                v = self.prev_v + alpha * (raw_v - self.prev_v)
                raw_a = (v - self.prev_v) / dt
                a = self.prev_a + alpha * (raw_a - self.prev_a)
            else:
                self.get_logger().warning(
                    "Vicon sample gap %.3fs; reset derivative estimates" % dt)

        self.prev_t, self.prev_p, self.prev_q = now, p.copy(), q.copy()
        self.prev_v, self.prev_a = v.copy(), a.copy()
        out = FloatingBaseState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "world"
        out.pose.position.x, out.pose.position.y, out.pose.position.z = [float(x) for x in p]
        out.pose.orientation.x, out.pose.orientation.y = float(q[0]), float(q[1])
        out.pose.orientation.z, out.pose.orientation.w = float(q[2]), float(q[3])
        out.twist.linear.x, out.twist.linear.y, out.twist.linear.z = [float(x) for x in v[:3]]
        out.twist.angular.x, out.twist.angular.y, out.twist.angular.z = [float(x) for x in v[3:]]
        out.acceleration.linear.x, out.acceleration.linear.y, out.acceleration.linear.z = [float(x) for x in a[:3]]
        out.acceleration.angular.x, out.acceleration.angular.y, out.acceleration.angular.z = [float(x) for x in a[3:]]
        self.pub.publish(out)


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
