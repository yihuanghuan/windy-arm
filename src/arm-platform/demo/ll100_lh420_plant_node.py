#!/usr/bin/env python3
"""ROS plant node for LL100-LH420 NMPC closed-loop tests.

The plant mirrors the SmoothPositionController velocity-feedforward branch used
by student_arm_node and by benchmark_nmpc_solvers.py / tune_ll100_nmpc.py.
It publishes the LL100 display joint order on /joint_states and consumes the
six arm joints from /student/joint_command.
"""

import csv
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


PUBLISH_RATE_HZ = 50.0
ARM_JOINT_NAMES = ["A_Joint", "B_Joint", "C_Joint", "D_Joint", "E_Joint", "F_Joint"]
DISPLAY_JOINT_NAMES = [
    "rot1_Joint",
    "rot2_Joint",
    "rot3_Joint",
    "rot4_Joint",
    *ARM_JOINT_NAMES,
]

# SmoothPositionController velocity-feedforward defaults.
VFF_GAIN = 0.7
VFF_TAU = 0.15
VFF_FB_VLIM = 0.4
VFF_MAX_ACCEL = 5.0

FALLBACK_Q_LOWER = np.array([-4.0, -3.1415, -1.5708, -4.0, -2.0, -4.0], dtype=float)
FALLBACK_Q_UPPER = np.array([4.0, 2.0, 1.5708, 4.0, 2.0, 4.0], dtype=float)


def _load_neutral_and_limits() -> Tuple[np.ndarray, np.ndarray, np.ndarray, str]:
    """Load the same neutral/home and limits as the NMPC demo without building a solver."""
    try:
        import pinocchio as pin
        from mpc_pinocchio_ik import PACKAGE_ROOT

        urdf = str(PACKAGE_ROOT / "config" / "ll100_lh420_nmpc.urdf")
        model = pin.buildModelFromUrdf(urdf)
        q_lower = np.asarray(model.lowerPositionLimit, dtype=float).reshape(len(ARM_JOINT_NAMES))
        q_upper = np.asarray(model.upperPositionLimit, dtype=float).reshape(len(ARM_JOINT_NAMES))
        q0 = np.clip(np.asarray(pin.neutral(model), dtype=float), q_lower, q_upper)
        return q0, q_lower, q_upper, urdf
    except Exception as exc:  # pragma: no cover - only used on misconfigured ROS hosts.
        q0 = np.zeros(len(ARM_JOINT_NAMES), dtype=float)
        return q0, FALLBACK_Q_LOWER.copy(), FALLBACK_Q_UPPER.copy(), f"fallback:{exc}"


def _plant_step(
    q_plant: np.ndarray,
    q_cmd: np.ndarray,
    qdot_ref: np.ndarray,
    last_v: np.ndarray,
    max_velocity: float,
    q_lower: np.ndarray,
    q_upper: np.ndarray,
    dt: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """One SmoothPositionController velocity-feedforward integration step."""
    v_ff = np.clip(qdot_ref, -max_velocity, max_velocity)
    v_fb = np.clip((q_cmd - q_plant) / VFF_TAU, -VFF_FB_VLIM, VFF_FB_VLIM)
    v_raw = np.clip(VFF_GAIN * v_ff + v_fb, -max_velocity, max_velocity)
    accel = VFF_MAX_ACCEL * dt
    v_cmd = np.clip(v_raw, last_v - accel, last_v + accel)
    q_new = np.clip(q_plant + v_cmd * dt, q_lower, q_upper)
    return q_new, (q_new - q_plant) / dt


class LL100LH420PlantNode(Node):
    def __init__(self):
        super().__init__("ll100_lh420_plant_node")
        self.declare_parameter("publish_rate", PUBLISH_RATE_HZ)
        self.declare_parameter("max_velocity", 0.5)
        self.declare_parameter("record_path", "")

        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.max_velocity = float(self.get_parameter("max_velocity").value)
        record_path = str(self.get_parameter("record_path").value)
        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be > 0")
        if self.max_velocity < 0.0:
            raise ValueError("max_velocity must be >= 0")
        self.dt = 1.0 / self.publish_rate

        self.q, self.q_lower, self.q_upper, urdf_source = _load_neutral_and_limits()
        self.last_v = np.zeros(len(ARM_JOINT_NAMES), dtype=float)
        self.latest_cmd_q: Optional[np.ndarray] = None
        self.latest_cmd_dq: Optional[np.ndarray] = None
        self.command_seq = 0
        self.last_command_time_sec: Optional[float] = None
        self.start_time_sec: Optional[float] = None

        self.record_file = None
        self.record_writer = None
        if record_path:
            path = Path(record_path).expanduser().resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.record_file = path.open("w", newline="")
            self.record_writer = csv.writer(self.record_file)
            self.record_writer.writerow(
                [
                    "t_sec",
                    "ros_time_sec",
                    "command_seq",
                    "command_age_sec",
                    *[f"q_{name}" for name in ARM_JOINT_NAMES],
                    *[f"v_{name}" for name in ARM_JOINT_NAMES],
                    *[f"cmd_q_{name}" for name in ARM_JOINT_NAMES],
                    *[f"cmd_v_{name}" for name in ARM_JOINT_NAMES],
                ]
            )
            self.record_file.flush()

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.sub = self.create_subscription(
            JointState, "/student/joint_command", self._command_callback, 10
        )
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            "LL100-LH420 plant started: "
            f"publish_rate={self.publish_rate:.1f}Hz, max_velocity={self.max_velocity:.3f}, "
            f"q0={self.q.tolist()}, limits=({self.q_lower.tolist()}, {self.q_upper.tolist()}), "
            f"urdf={urdf_source}, record_path={record_path or '<disabled>'}"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _command_callback(self, msg: JointState) -> None:
        if msg.position is None or len(msg.position) == 0:
            self.get_logger().warn("Rejected empty /student/joint_command", throttle_duration_sec=2.0)
            return

        name_to_index = {name: i for i, name in enumerate(msg.name)}
        if all(name in name_to_index for name in ARM_JOINT_NAMES):
            indices = [name_to_index[name] for name in ARM_JOINT_NAMES]
        elif len(msg.position) >= len(ARM_JOINT_NAMES):
            indices = list(range(len(ARM_JOINT_NAMES)))
        else:
            self.get_logger().warn(
                f"Rejected command: expected {len(ARM_JOINT_NAMES)} arm positions, got {len(msg.position)}",
                throttle_duration_sec=2.0,
            )
            return

        q_cmd = np.asarray([msg.position[i] for i in indices], dtype=float)
        if not np.all(np.isfinite(q_cmd)):
            self.get_logger().warn("Rejected command: position contains NaN/Inf", throttle_duration_sec=2.0)
            return
        q_cmd = np.clip(q_cmd, self.q_lower, self.q_upper)

        if len(msg.velocity) >= max(indices) + 1:
            dq_cmd = np.asarray([msg.velocity[i] for i in indices], dtype=float)
            dq_cmd = np.where(np.isfinite(dq_cmd), dq_cmd, 0.0)
        else:
            dq_cmd = np.zeros(len(ARM_JOINT_NAMES), dtype=float)

        self.latest_cmd_q = q_cmd
        self.latest_cmd_dq = dq_cmd
        self.command_seq += 1
        self.last_command_time_sec = self._now_sec()

    def tick(self) -> None:
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9
        if self.start_time_sec is None:
            self.start_time_sec = now_sec
        t_sec = now_sec - self.start_time_sec

        if self.latest_cmd_q is not None and self.latest_cmd_dq is not None:
            self.q, self.last_v = _plant_step(
                self.q,
                self.latest_cmd_q,
                self.latest_cmd_dq,
                self.last_v,
                self.max_velocity,
                self.q_lower,
                self.q_upper,
                self.dt,
            )
        else:
            self.last_v = np.zeros_like(self.last_v)

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = DISPLAY_JOINT_NAMES
        msg.position = [0.0, 0.0, 0.0, 0.0, *self.q.tolist()]
        msg.velocity = [0.0, 0.0, 0.0, 0.0, *self.last_v.tolist()]
        self.pub.publish(msg)

        self._record_tick(t_sec, now_sec)

    def _record_tick(self, t_sec: float, now_sec: float) -> None:
        if self.record_writer is None:
            return
        if self.latest_cmd_q is None or self.latest_cmd_dq is None:
            cmd_q = [float("nan")] * len(ARM_JOINT_NAMES)
            cmd_dq = [float("nan")] * len(ARM_JOINT_NAMES)
            command_age = float("nan")
        else:
            cmd_q = self.latest_cmd_q.tolist()
            cmd_dq = self.latest_cmd_dq.tolist()
            command_age = (
                now_sec - self.last_command_time_sec
                if self.last_command_time_sec is not None
                else float("nan")
            )
        self.record_writer.writerow(
            [
                t_sec,
                now_sec,
                self.command_seq,
                command_age,
                *self.q.tolist(),
                *self.last_v.tolist(),
                *cmd_q,
                *cmd_dq,
            ]
        )
        self.record_file.flush()

    def close_record(self) -> None:
        if self.record_file is not None:
            self.record_file.flush()
            self.record_file.close()
            self.record_file = None
            self.record_writer = None

    def destroy_node(self):
        self.close_record()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = LL100LH420PlantNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_record()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
