#!/usr/bin/env python3
"""前桨避障 ACADOS NMPC demo。

这个脚本给定一个末端目标点,滚动调用 MpcPinocchioIK 的 ACADOS 避障后端,
并在 RViz 中发布前桨避障圆盘、机械臂关键点和目标点 marker。
"""

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK


JOINT_COUNT = 7
PUBLISH_RATE_HZ = 50.0
BASE_FRAME = "base_link"


def _quat_from_z_axis(normal: np.ndarray):
    n = np.asarray(normal, dtype=float)
    norm = np.linalg.norm(n)
    if norm <= 0.0:
        return 0.0, 0.0, 0.0, 1.0
    n = n / norm
    z_axis = np.array([0.0, 0.0, 1.0], dtype=float)
    dot = float(np.clip(z_axis @ n, -1.0, 1.0))
    if dot > 1.0 - 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    if dot < -1.0 + 1e-9:
        return 1.0, 0.0, 0.0, 0.0
    axis = np.cross(z_axis, n)
    axis = axis / np.linalg.norm(axis)
    angle = math.acos(dot)
    s = math.sin(0.5 * angle)
    return float(axis[0] * s), float(axis[1] * s), float(axis[2] * s), float(math.cos(0.5 * angle))


class MoveArmObstacleMpcDemo(Node):
    def __init__(self):
        super().__init__("move_arm_obstacle_mpc_demo")
        self.declare_parameter("target_position", [0.35, 0.0, 0.12])
        self.declare_parameter("terminal_tolerance", 0.01)
        self.declare_parameter("horizon_steps", 8)
        self.declare_parameter("max_joint_step", 0.08)
        self.declare_parameter("lock_orientation", False)
        self.declare_parameter("orientation_target", "initial")
        self.declare_parameter("orientation_weight", 2.0)
        self.declare_parameter("terminal_orientation_weight", 20.0)
        self.declare_parameter("obstacle_weight", 2e4)
        self.declare_parameter("obstacle_safety_margin", 0.04)
        self.declare_parameter("obstacle_propeller_radius", 0.27)
        self.declare_parameter("obstacle_disk_half_thickness", 0.02)

        target_position = np.array(self.get_parameter("target_position").value, dtype=float)
        if target_position.shape != (3,):
            raise ValueError("target_position 必须是长度为 3 的数组,例如 [0.35, 0.0, 0.12]")
        self.target_position = target_position
        self.terminal_tolerance = float(self.get_parameter("terminal_tolerance").value)
        self.horizon_steps = int(self.get_parameter("horizon_steps").value)
        self.max_joint_step = float(self.get_parameter("max_joint_step").value)
        self.lock_orientation = bool(self.get_parameter("lock_orientation").value)
        self.orientation_target = self.get_parameter("orientation_target").value
        self.orientation_weight = float(self.get_parameter("orientation_weight").value)
        self.terminal_orientation_weight = float(self.get_parameter("terminal_orientation_weight").value)
        self.obstacle_weight = float(self.get_parameter("obstacle_weight").value)
        self.obstacle_safety_margin = float(self.get_parameter("obstacle_safety_margin").value)
        self.obstacle_propeller_radius = float(self.get_parameter("obstacle_propeller_radius").value)
        self.obstacle_disk_half_thickness = float(self.get_parameter("obstacle_disk_half_thickness").value)

        if self.horizon_steps < 1:
            raise ValueError("horizon_steps 必须 >= 1")
        if self.max_joint_step <= 0.0:
            raise ValueError("max_joint_step 必须 > 0")
        if self.orientation_target not in ("initial", "vertical_down"):
            raise ValueError("orientation_target 必须是 'initial' 或 'vertical_down'")

        cfg = MpcIkConfig(
            horizon_steps=self.horizon_steps,
            control_dt=1.0 / PUBLISH_RATE_HZ,
            max_joint_step=self.max_joint_step,
            success_tolerance=self.terminal_tolerance,
            solver_backend="acados",
            acados_build_dir="/tmp/windylab_acados_mpc_ik_obstacle",
            orientation_weight=self.orientation_weight if self.lock_orientation else 0.0,
            terminal_orientation_weight=self.terminal_orientation_weight if self.lock_orientation else 0.0,
            obstacle_avoidance_enabled=True,
            obstacle_weight=self.obstacle_weight,
            obstacle_safety_margin=self.obstacle_safety_margin,
            obstacle_propeller_radius=self.obstacle_propeller_radius,
            obstacle_disk_half_thickness=self.obstacle_disk_half_thickness,
        )
        self.mpc_ik = MpcPinocchioIK(config=cfg)
        self.q_last = np.clip(pin.neutral(self.mpc_ik.model), self.mpc_ik.q_lower, self.mpc_ik.q_upper)
        self.locked_rotation = self._orientation_target_rotation(self.q_last)
        self.warm_start = None
        self.dt = 1.0 / PUBLISH_RATE_HZ
        self.t = 0.0
        self.trajectory_start_time_sec = None

        self.command_pub = self.create_publisher(JointState, "/student/joint_command", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/obstacle_mpc/markers", 10)
        self.timer = self.create_timer(self.dt, self.tick)

        disks = self.mpc_ik.obstacle_disks()
        self.get_logger().info(
            "前桨避障 ACADOS NMPC 启动: "
            f"target={self.target_position.tolist()}, horizon={self.horizon_steps}, "
            f"max_joint_step={self.max_joint_step:.3f}, obstacle_disks={len(disks)}, "
            f"obstacle_weight={self.obstacle_weight:.1f}, safety_margin={self.obstacle_safety_margin:.3f}, "
            f"lock_orientation={self.lock_orientation}, orientation_target={self.orientation_target}"
        )

    def _orientation_target_rotation(self, q0: np.ndarray) -> np.ndarray:
        if self.orientation_target == "vertical_down":
            return np.array(
                [
                    [0.0, 0.0, 1.0],
                    [0.0, 1.0, 0.0],
                    [-1.0, 0.0, 0.0],
                ],
                dtype=float,
            )
        _, current_rotation = self.mpc_ik.forward(q0)
        return current_rotation

    def tick(self):
        q_current = self.q_last.copy()
        target_rotation = self.locked_rotation if self.lock_orientation else None
        q_next, ok, info = self.mpc_ik.step(
            self.target_position,
            q_current,
            warm_start=self.warm_start,
            target_rotation=target_rotation,
        )
        self.warm_start = info["trajectory"]

        command_usable = bool(info.get("command_usable", ok))
        dq_cmd = np.zeros(JOINT_COUNT)
        if command_usable:
            self.q_last = q_next
            dq_cmd = (q_next - q_current) / self.dt
        else:
            self.get_logger().warn(
                "避障 NMPC 命令不可用,保持上一帧: "
                f"terminal_error={info['terminal_error']:.4f}, "
                f"min_margin={info.get('min_obstacle_margin')}, "
                f"violation={info.get('obstacle_violation_max')}, "
                f"status={info['status']}, message={info['message']}",
                throttle_duration_sec=1.0,
            )

        if command_usable and not bool(info.get("tracking_ok", ok)):
            self.get_logger().warn(
                "避障 NMPC 尚未到达目标,继续滚动靠近: "
                f"terminal_error={info['terminal_error']:.4f}, "
                f"min_margin={info.get('min_obstacle_margin')}, "
                f"violation={info.get('obstacle_violation_max')}",
                throttle_duration_sec=2.0,
            )

        msg = JointState()
        msg.header.stamp = self._trajectory_stamp(self.t)
        msg.name = [f"joint{i + 1}" for i in range(JOINT_COUNT)]
        msg.position = self.q_last.tolist()
        msg.velocity = dq_cmd.tolist()
        self.command_pub.publish(msg)
        self._publish_markers(self.q_last)
        self.t += self.dt

    def _publish_markers(self, q: np.ndarray):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        disks = self.mpc_ik.obstacle_disks()
        for i, disk in enumerate(disks):
            marker = Marker()
            marker.header.frame_id = BASE_FRAME
            marker.header.stamp = now
            marker.ns = "front_propeller_disks"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(disk[0])
            marker.pose.position.y = float(disk[1])
            marker.pose.position.z = float(disk[2])
            qx, qy, qz, qw = _quat_from_z_axis(disk[3:6])
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            diameter = 2.0 * float(disk[6] + self.obstacle_safety_margin)
            marker.scale.x = diameter
            marker.scale.y = diameter
            marker.scale.z = 2.0 * float(self.obstacle_disk_half_thickness)
            marker.color.r = 1.0
            marker.color.g = 0.15
            marker.color.b = 0.05
            marker.color.a = 0.28
            markers.markers.append(marker)

        keypoints = self.mpc_ik.obstacle_keypoint_positions(q)
        marker = Marker()
        marker.header.frame_id = BASE_FRAME
        marker.header.stamp = now
        marker.ns = "arm_keypoints"
        marker.id = 1000
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035
        marker.scale.y = 0.035
        marker.scale.z = 0.035
        marker.color.r = 0.1
        marker.color.g = 0.45
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in keypoints.values()]
        markers.markers.append(marker)

        target = Marker()
        target.header.frame_id = BASE_FRAME
        target.header.stamp = now
        target.ns = "target"
        target.id = 2000
        target.type = Marker.SPHERE
        target.action = Marker.ADD
        target.pose.position.x = float(self.target_position[0])
        target.pose.position.y = float(self.target_position[1])
        target.pose.position.z = float(self.target_position[2])
        target.pose.orientation.w = 1.0
        target.scale.x = 0.05
        target.scale.y = 0.05
        target.scale.z = 0.05
        target.color.r = 0.0
        target.color.g = 0.85
        target.color.b = 0.2
        target.color.a = 1.0
        markers.markers.append(target)

        self.marker_pub.publish(markers)

    def _trajectory_stamp(self, trajectory_time_sec: float) -> Time:
        if self.trajectory_start_time_sec is None:
            self.trajectory_start_time_sec = self.get_clock().now().nanoseconds * 1e-9
        stamp_sec = self.trajectory_start_time_sec + trajectory_time_sec
        sec = math.floor(stamp_sec)
        nanosec = int(round((stamp_sec - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        return Time(sec=int(sec), nanosec=nanosec)


def main():
    rclpy.init()
    node = MoveArmObstacleMpcDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
