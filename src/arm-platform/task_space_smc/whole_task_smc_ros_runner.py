#!/usr/bin/env python3
"""ROS/RViz whole-task runner for the floating-base task-space SMC."""

from __future__ import annotations

import csv
import json
import math
import sys
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml

if __package__ in (None, ""):
    package_dir = Path(__file__).resolve().parent
    if (package_dir / "__init__.py").exists():
        sys.path = [p for p in sys.path if Path(p or ".").resolve() != package_dir]
        sys.path.insert(0, str(package_dir.parent))
    else:
        sys.path.insert(0, str(package_dir))

from task_space_smc.actuator_adapter import ActuatorAdapter, ActuatorLimits
from task_space_smc.model import DEFAULT_ARM_JOINT_NAMES, FloatingBaseArmModel
from task_space_smc.offline_simulation import controller_from_config, default_config_path
from task_space_smc.pose_error import PoseState
from task_space_smc.quaternion_math import (
    quat_to_rotation,
    quaternion_log_error,
    rotation_to_quat,
    wxyz_to_ros_xyzw,
)
from task_space_smc.redundancy_resolution import resolve_acceleration
from task_space_smc.trajectory_reference import (
    AnalyticBaseSample,
    base_sample,
    drawing_segments,
    reference_sample,
    traj,
    whole_period,
)

try:
    import rclpy
    from dummy_interface.msg import FloatingBaseState, MotorControl
    from geometry_msgs.msg import Point, TransformStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import JointState
    from std_msgs.msg import ColorRGBA, Float64MultiArray
    from tf2_ros import TransformBroadcaster
    from visualization_msgs.msg import Marker, MarkerArray
except Exception:  # pragma: no cover
    rclpy = None
    Node = object


class WholeTaskSmcRosRunner(Node):
    def __init__(self):
        super().__init__("whole_task_smc_ros_runner")
        self.declare_parameter("config_path", default_config_path())
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("mode", "simulation")
        self.declare_parameter("trajectory_shape", "whole")
        self.declare_parameter("duration_sec", 0.0)
        self.declare_parameter("start_delay_sec", 3.0)
        self.declare_parameter("derivative_dt_sec", 0.005)
        self.declare_parameter("hardware_enabled", False)
        self.declare_parameter("publish_joint_states_in_sim", True)
        self.declare_parameter(
            "data_dir",
            "/home/windylab/code/windylab_ws/src/arm-platform/task_space_smc/data",
        )
        self.declare_parameter("marker_sample_count", 4000)
        self.declare_parameter("visualization_rate_hz", 30.0)
        self.declare_parameter("control_rate_hz_override", 0.0)
        self.declare_parameter("initial_joint_positions", [0.0] * 7)

        self.config_path = str(self.get_parameter("config_path").value)
        with open(self.config_path, "r", encoding="utf-8") as f:
            self.config = yaml.safe_load(f)
        self.shape = str(self.get_parameter("trajectory_shape").value)
        self.mode = str(self.get_parameter("mode").value)
        if self.mode not in ("simulation", "internal_simulation", "hardware"):
            raise ValueError(
                "mode must be simulation, internal_simulation, or hardware")
        self.internal_plant = self.mode == "internal_simulation"
        self.simulated_base = self.mode in ("simulation", "internal_simulation")
        self.derivative_dt = float(self.get_parameter("derivative_dt_sec").value)
        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.start_delay_sec = max(
            0.0, float(self.get_parameter("start_delay_sec").value))
        if self.duration_sec <= 0.0:
            self.duration_sec = whole_period() if self.shape == "whole" else 20.0
        self.data_dir = Path(str(self.get_parameter("data_dir").value))
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        urdf_path = self.get_parameter("urdf_path").value or None
        self.model = FloatingBaseArmModel(urdf_path=urdf_path)
        self.controller = controller_from_config(self.config)
        self.q = np.asarray(self.config["nullspace"]["q_nominal"], dtype=float).reshape(7).copy()
        self.qd = np.zeros(7)
        self.last_joint_stamp = None
        self.has_joint_feedback = self.internal_plant
        self.feedback_generation = 0
        self.last_command_feedback_generation = -1
        self.feedback_effort = np.zeros(7)
        self.start_time = None
        self.last_update_time = None
        self.last_hold_publish_time = None
        self.last_reference_publish_time = None
        self.sim_time = 0.0
        self.done = False
        self.rng = np.random.default_rng(
            int(self.config.get("simulation_injection", {}).get("seed", 20260730))
        )
        self.base_state_history = []
        self.motor_command_history = deque()
        self.last_injection = {
            "tau_actual": np.zeros(7),
            "tau_disturbance": np.zeros(7),
            "gust_active": 0,
            "gust_envelope": 0.0,
            "base_gust_offset": np.zeros(3),
        }

        limits = ActuatorLimits(
            torque=np.asarray(self.config["limits"]["torque"], dtype=float).reshape(7),
            torque_rate=np.asarray(self.config["limits"]["torque_rate"], dtype=float).reshape(7),
            current=np.asarray(self.config["limits"]["current"], dtype=float).reshape(7),
        )
        self.adapter = ActuatorAdapter(
            joint_names=list(DEFAULT_ARM_JOINT_NAMES),
            limits=limits,
            current_unit="torque_nm",
            command_timeout_sec=float(self.config["safety"]["command_timeout_sec"]),
            joint_state_timeout_sec=float(self.config["safety"]["joint_state_timeout_sec"]),
            base_state_timeout_sec=float(self.config["safety"]["base_state_timeout_sec"]),
            hardware_enabled=(
                self.mode == "simulation"
                or bool(self.get_parameter("hardware_enabled").value)
            ),
        )
        approach = self.config.get("approach", {})
        self.approach_initial_offset = np.asarray(
            approach.get(
                "initial_joint_offset_rad",
                [0.25, -0.20, 0.20, 0.25, -0.20, 0.15, -0.15],
            ),
            dtype=float,
        ).reshape(7)
        self.approach_kp = np.asarray(
            approach.get("kp", [60.0, 60.0, 60.0, 50.0, 50.0, 40.0, 40.0]),
            dtype=float,
        ).reshape(7)
        self.approach_kd = np.asarray(
            approach.get("kd", [16.0, 16.0, 16.0, 14.0, 14.0, 12.0, 12.0]),
            dtype=float,
        ).reshape(7)
        if self.simulated_base:
            self.track_start_q = self._solve_initial_pose(self.q)
            initial_joint_positions = np.asarray(
                self.get_parameter("initial_joint_positions").value,
                dtype=float,
            ).reshape(7)
            if np.any(np.abs(initial_joint_positions) > 1e-12):
                self.q = np.asarray(
                    initial_joint_positions, dtype=float).reshape(7)
            else:
                self.q = self.track_start_q + (
                    self.approach_initial_offset
                    if self.start_delay_sec > 0.0 else np.zeros(7)
                )
            self.q = np.clip(
                self.q,
                self.model.arm_lower_position_limit,
                self.model.arm_upper_position_limit,
            )
            self.approach_initial_q = self.q.copy()
        else:
            self.track_start_q = self.q.copy()
            self.approach_initial_q = self.q.copy()

        self.command_pub = self.create_publisher(MotorControl, "/student/joint_effort_command", 10)
        self.base_truth_pub = self.create_publisher(
            FloatingBaseState, "/floating_base/truth", 10)
        self.joint_pub = (
            self.create_publisher(JointState, "/joint_states", 10)
            if self.internal_plant else None
        )
        static_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reference_marker_pub = self.create_publisher(
            MarkerArray, "/task_space_smc/reference_markers", static_qos)
        self.marker_pub = self.create_publisher(MarkerArray, "/task_space_smc/markers", 10)
        self.tracking_pub = self.create_publisher(
            Float64MultiArray, "/task_space_smc/tracking", 10)
        self.torque_pub = self.create_publisher(
            Float64MultiArray, "/task_space_smc/joint_torque", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

        self.metrics_path = self.data_dir / (
            f"whole_task_smc_trajectory_{self.run_stamp}.csv")
        self.metrics_file = open(self.metrics_path, "w", newline="", encoding="utf-8")
        vector_fields = (
            [f"desired_{axis}_m" for axis in "xyz"]
            + [f"actual_{axis}_m" for axis in "xyz"]
            + [f"error_{axis}_m" for axis in "xyz"]
            + [f"base_{axis}_m" for axis in "xyz"]
            + [f"base_q_{name}" for name in ("w", "x", "y", "z")]
            + [f"base_est_{axis}_m" for axis in "xyz"]
            + [f"base_est_q_{name}" for name in ("w", "x", "y", "z")]
            + [f"base_gust_{axis}_m" for axis in "xyz"]
            + [f"joint_{i + 1}_rad" for i in range(7)]
            + [f"joint_{i + 1}_velocity_rad_s" for i in range(7)]
            + [f"joint_{i + 1}_torque_nm" for i in range(7)]
            + [f"joint_{i + 1}_tau_actual_nm" for i in range(7)]
            + [f"joint_{i + 1}_tau_disturbance_nm" for i in range(7)]
        )
        self.metrics_writer = csv.DictWriter(
            self.metrics_file,
            fieldnames=[
                "t", "pos_error_m", "rot_error_rad", "tau_max_abs",
                "sigma_min", "damping", "nullspace_leakage", "enabled",
                "pen_down", "stroke_id", "simulation_injection_enabled",
                "gust_active", "gust_envelope", "tau_actual_max_abs",
                "tau_disturbance_max_abs",
            ] + vector_fields,
        )
        self.metrics_writer.writeheader()
        self.error_samples = []
        self.rot_samples = []
        self.actual_path_segments = []
        self.actual_stroke_id = None
        self.reference_markers = None
        rate_override = float(
            self.get_parameter("control_rate_hz_override").value)
        self.runtime_control_rate_hz = (
            rate_override if rate_override > 0.0
            else float(self.config["control_rate_hz"])
        )
        self.visualization_rate_hz = max(
            1.0, float(self.get_parameter("visualization_rate_hz").value))
        self.visualization_stride = max(
            1,
            int(round(
                self.runtime_control_rate_hz
                / self.visualization_rate_hz
            )),
        )
        self._publish_static_path()
        period = 1.0 / self.runtime_control_rate_hz
        self.control_period = period
        self.timer = self.create_timer(period, self._control_loop)
        self.get_logger().info(
            f"Whole-task SMC runner mode={self.mode}, shape={self.shape}, "
            f"start_delay={self.start_delay_sec:.3f}s, duration={self.duration_sec:.3f}s, "
            f"config={self.config_path}, metrics={self.metrics_path}"
        )
        if self.simulated_base and self._inj_enabled():
            self.get_logger().info(
                "simulation_injection enabled: base noise/delay, payload error, "
                "friction, backlash, motor torque error/delay, gust torque"
            )
            if not self.internal_plant:
                self.get_logger().warning(
                    "Shared-chain simulation currently injects base truth and "
                    "base-estimator uncertainty only; actuator disturbances "
                    "remain available in mode=internal_simulation.")

    def _solve_initial_pose(self, q_seed: np.ndarray) -> np.ndarray:
        if getattr(self, "simulated_base", False):
            bs = self._simulation_base_sample(0.0)
        else:
            bs = base_sample(0.0, self.shape, 0.0, self.derivative_dt)
        ref = reference_sample(0.0, self.shape, self.derivative_dt)
        q = np.asarray(q_seed, dtype=float).reshape(7).copy()
        q_lower = self.model.model.lowerPositionLimit[7:].copy()
        q_upper = self.model.model.upperPositionLimit[7:].copy()
        scale = np.diag([1.0, 1.0, 1.0, 0.12, 0.12, 0.12])
        for _ in range(250):
            q_full = self.model.make_q_full(bs.position_w, bs.quaternion_wxyz, q)
            kin = self.model.compute_kinematics(q_full, self.model.make_v_full(bs.velocity_lwa, np.zeros(7)))
            err = np.concatenate([
                ref.position_w - kin.ee_position,
                pin.log3(ref.rotation_w @ kin.ee_rotation.T),
            ])
            if np.linalg.norm(err[:3]) < 5e-5 and np.linalg.norm(err[3:]) < 2e-3:
                break
            J = scale @ kin.J_arm_lwa
            y = scale @ err
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), y)
            q = np.clip(q + 0.45 * dq, q_lower, q_upper)
        return q

    def destroy_node(self):
        if getattr(self, "metrics_file", None):
            self.metrics_file.close()
        super().destroy_node()

    def _joint_state_cb(self, msg):
        if self.internal_plant:
            return
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        for j, name in enumerate(DEFAULT_ARM_JOINT_NAMES):
            if name not in name_to_index:
                return
            i = name_to_index[name]
            self.q[j] = msg.position[i]
            self.qd[j] = msg.velocity[i] if i < len(msg.velocity) else 0.0
            self.feedback_effort[j] = (
                msg.effort[i] if i < len(msg.effort) else 0.0)
        self.last_joint_stamp = self.get_clock().now().nanoseconds * 1e-9
        self.has_joint_feedback = True
        self.feedback_generation += 1

    @staticmethod
    def _point(p):
        point = Point()
        point.x, point.y, point.z = [float(x) for x in p]
        return point

    @staticmethod
    def _color(r, g, b, a=1.0):
        c = ColorRGBA()
        c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
        return c

    def _line_marker(self, marker_id, ns, points, color, scale=0.003):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = float(scale)
        marker.color = color
        marker.points = [self._point(p) for p in points]
        return marker

    def _sphere_marker(self, marker_id, ns, p, color, scale=0.012):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = self._point(p)
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = float(scale)
        marker.color = color
        return marker

    def _publish_static_path(self):
        markers = []
        for i, segment in enumerate(drawing_segments(self.shape, int(self.get_parameter("marker_sample_count").value))):
            markers.append(self._line_marker(
                i,
                "whole_reference_path",
                segment,
                self._color(0.1, 1.0, 0.1, 0.9),
                0.006,
            ))
        self.reference_markers = MarkerArray(markers=markers)
        self.reference_marker_pub.publish(self.reference_markers)

    def _publish_base_tf(self, bs):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id = "base_link"
        msg.transform.translation.x = float(bs.position_w[0])
        msg.transform.translation.y = float(bs.position_w[1])
        msg.transform.translation.z = float(bs.position_w[2])
        q_xyzw = wxyz_to_ros_xyzw(bs.quaternion_wxyz)
        msg.transform.rotation.x = float(q_xyzw[0])
        msg.transform.rotation.y = float(q_xyzw[1])
        msg.transform.rotation.z = float(q_xyzw[2])
        msg.transform.rotation.w = float(q_xyzw[3])
        self.tf_broadcaster.sendTransform(msg)

    def _publish_base_truth(self, bs):
        if not self.simulated_base or self.internal_plant:
            return
        msg = FloatingBaseState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(bs.position_w[0])
        msg.pose.position.y = float(bs.position_w[1])
        msg.pose.position.z = float(bs.position_w[2])
        q_xyzw = wxyz_to_ros_xyzw(bs.quaternion_wxyz)
        msg.pose.orientation.x = float(q_xyzw[0])
        msg.pose.orientation.y = float(q_xyzw[1])
        msg.pose.orientation.z = float(q_xyzw[2])
        msg.pose.orientation.w = float(q_xyzw[3])
        msg.twist.linear.x = float(bs.velocity_lwa[0])
        msg.twist.linear.y = float(bs.velocity_lwa[1])
        msg.twist.linear.z = float(bs.velocity_lwa[2])
        msg.twist.angular.x = float(bs.velocity_lwa[3])
        msg.twist.angular.y = float(bs.velocity_lwa[4])
        msg.twist.angular.z = float(bs.velocity_lwa[5])
        msg.acceleration.linear.x = float(bs.acceleration_lwa[0])
        msg.acceleration.linear.y = float(bs.acceleration_lwa[1])
        msg.acceleration.linear.z = float(bs.acceleration_lwa[2])
        msg.acceleration.angular.x = float(bs.acceleration_lwa[3])
        msg.acceleration.angular.y = float(bs.acceleration_lwa[4])
        msg.acceleration.angular.z = float(bs.acceleration_lwa[5])
        self.base_truth_pub.publish(msg)

    def _inj(self, *keys, default=None):
        node = self.config.get("simulation_injection", {})
        for key in keys:
            if not isinstance(node, dict) or key not in node:
                return default
            node = node[key]
        return node

    def _inj_enabled(self):
        return bool(self._inj("enabled", default=True))

    def _inj_vec(self, *keys, default, n=7):
        return np.asarray(self._inj(*keys, default=default), dtype=float).reshape(n)

    def _small_rotation(self, rotvec):
        angle = float(np.linalg.norm(rotvec))
        if angle < 1e-12:
            return np.eye(3)
        return pin.exp3(np.asarray(rotvec, dtype=float).reshape(3))

    def _simulation_base_sample(self, t):
        nominal = base_sample(t, self.shape, t, self.derivative_dt)
        if not self._inj_enabled():
            self.last_injection["gust_active"] = 0
            self.last_injection["gust_envelope"] = 0.0
            self.last_injection["base_gust_offset"] = np.zeros(3)
            return nominal

        amp = self._inj_vec(
            "base_motion",
            "translation_amplitude_m",
            default=self._inj("translation_amplitude_m", default=[0.0, 0.0, 0.0]),
            n=3,
        )
        freq = self._inj_vec(
            "base_motion",
            "translation_frequency_hz",
            default=self._inj("translation_frequency_hz", default=[0.0, 0.0, 0.0]),
            n=3,
        )
        phase = self._inj_vec(
            "base_motion",
            "translation_phase_rad",
            default=self._inj("translation_phase_rad", default=[0.0, 0.0, 0.0]),
            n=3,
        )
        wt = 2.0 * math.pi * freq * float(t) + phase
        p_inj = amp * np.sin(wt)
        v_inj = amp * (2.0 * math.pi * freq) * np.cos(wt)
        a_inj = -amp * (2.0 * math.pi * freq) ** 2 * np.sin(wt)

        rpy_amp = np.deg2rad(self._inj_vec(
            "base_motion",
            "rpy_amplitude_deg",
            default=self._inj("rpy_amplitude_deg", default=[0.0, 0.0, 0.0]),
            n=3,
        ))
        rpy_freq = self._inj_vec(
            "base_motion",
            "rpy_frequency_hz",
            default=self._inj("rpy_frequency_hz", default=[0.0, 0.0, 0.0]),
            n=3,
        )
        rpy_phase = self._inj_vec(
            "base_motion",
            "rpy_phase_rad",
            default=self._inj("rpy_phase_rad", default=[0.0, 0.0, 0.0]),
            n=3,
        )
        rpy = rpy_amp * np.sin(2.0 * math.pi * rpy_freq * float(t) + rpy_phase)
        rpy_d = rpy_amp * (2.0 * math.pi * rpy_freq) * np.cos(
            2.0 * math.pi * rpy_freq * float(t) + rpy_phase
        )
        rpy_dd = -rpy_amp * (2.0 * math.pi * rpy_freq) ** 2 * np.sin(
            2.0 * math.pi * rpy_freq * float(t) + rpy_phase
        )

        gust_cfg = self._inj("gust", default={}) or {}
        gust_amp = float(gust_cfg.get("base_amplitude_m", 0.0))
        gust_profile = str(gust_cfg.get("profile", "gust"))
        gust_seed = int(gust_cfg.get("seed", self._inj("seed", default=20260730)))
        gust_period = float(gust_cfg.get("period_sec", 4.0))
        gust_width = float(gust_cfg.get("width_sec", 0.8))
        g0 = traj.base_disturbance_sample(
            float(t),
            gust_amp,
            seed=gust_seed,
            profile=gust_profile,
            gust_period_sec=gust_period,
            gust_width_sec=gust_width,
        )
        h = max(float(self.derivative_dt), 1e-4)
        gm = traj.base_disturbance_sample(
            float(t) - h,
            gust_amp,
            seed=gust_seed,
            profile=gust_profile,
            gust_period_sec=gust_period,
            gust_width_sec=gust_width,
        ).total_offset
        gp = traj.base_disturbance_sample(
            float(t) + h,
            gust_amp,
            seed=gust_seed,
            profile=gust_profile,
            gust_period_sec=gust_period,
            gust_width_sec=gust_width,
        ).total_offset
        p_gust = np.asarray(g0.total_offset, dtype=float).reshape(3)
        v_gust = (gp - gm) / (2.0 * h)
        a_gust = (gp - 2.0 * p_gust + gm) / (h * h)
        self.last_injection["gust_active"] = int(bool(g0.event_active))
        self.last_injection["gust_envelope"] = float(g0.event_strength)
        self.last_injection["base_gust_offset"] = p_gust.copy()

        R_nom = np.asarray(nominal.rotation_wb, dtype=float).reshape(3, 3)
        R_true = R_nom @ self._small_rotation(rpy)
        return AnalyticBaseSample(
            position_w=nominal.position_w + p_inj + p_gust,
            quaternion_wxyz=rotation_to_quat(R_true),
            rotation_wb=R_true,
            velocity_lwa=nominal.velocity_lwa + np.concatenate([v_inj + v_gust, rpy_d]),
            acceleration_lwa=nominal.acceleration_lwa + np.concatenate([a_inj + a_gust, rpy_dd]),
        )

    def _noisy_delayed_base_sample(self, t, true_bs):
        if not (self.simulated_base and self._inj_enabled()):
            return true_bs
        self.base_state_history.append((float(t), true_bs))
        max_delay = max(0.0, float(self._inj("base_estimator", "delay_sec", default=0.0)))
        keep_after = float(t) - max(max_delay, 0.0) - 1.0
        while len(self.base_state_history) > 2 and self.base_state_history[1][0] < keep_after:
            self.base_state_history.pop(0)

        target = float(t) - max_delay
        delayed = self.base_state_history[0][1]
        for stamp, sample in self.base_state_history:
            if stamp <= target:
                delayed = sample
            else:
                break

        pos_std = float(self._inj("base_estimator", "position_noise_std_m", default=0.0))
        att_std = math.radians(float(self._inj("base_estimator", "attitude_noise_std_deg", default=0.0)))
        vel_std = float(self._inj("base_estimator", "linear_velocity_noise_std_m_s", default=0.0))
        gyro_std = float(self._inj("base_estimator", "angular_velocity_noise_std_rad_s", default=0.0))
        acc_std = float(self._inj("base_estimator", "linear_acceleration_noise_std_m_s2", default=0.0))
        alpha_std = float(self._inj("base_estimator", "angular_acceleration_noise_std_rad_s2", default=0.0))

        R = quat_to_rotation(delayed.quaternion_wxyz) @ self._small_rotation(
            self.rng.normal(0.0, att_std, 3)
        )
        return AnalyticBaseSample(
            position_w=delayed.position_w + self.rng.normal(0.0, pos_std, 3),
            quaternion_wxyz=rotation_to_quat(R),
            rotation_wb=R,
            velocity_lwa=delayed.velocity_lwa
            + np.concatenate([
                self.rng.normal(0.0, vel_std, 3),
                self.rng.normal(0.0, gyro_std, 3),
            ]),
            acceleration_lwa=delayed.acceleration_lwa
            + np.concatenate([
                self.rng.normal(0.0, acc_std, 3),
                self.rng.normal(0.0, alpha_std, 3),
            ]),
        )

    def _delayed_motor_command(self, tau_cmd, t):
        if not (self.internal_plant and self._inj_enabled()):
            return np.asarray(tau_cmd, dtype=float).reshape(7)
        delay = max(0.0, float(self._inj("motor", "drive_delay_sec", default=0.0)))
        tau_cmd = np.asarray(tau_cmd, dtype=float).reshape(7).copy()
        self.motor_command_history.append((float(t), tau_cmd))
        keep_after = float(t) - delay - 1.0
        while len(self.motor_command_history) > 2 and self.motor_command_history[1][0] < keep_after:
            self.motor_command_history.popleft()
        target = float(t) - delay
        delayed = self.motor_command_history[0][1]
        for stamp, cmd in self.motor_command_history:
            if stamp <= target:
                delayed = cmd
            else:
                break
        return delayed.copy()

    def _payload_disturbance_torque(self, q_full, kin):
        mass = float(self._inj("payload", "mass_error_kg", default=0.0))
        if abs(mass) < 1e-12:
            return np.zeros(7)
        gravity = np.asarray(self._inj("payload", "gravity_w_m_s2", default=[0.0, 0.0, -9.81]), dtype=float).reshape(3)
        force = mass * gravity
        return kin.J_arm_lwa[:3, :].T @ force

    def _gust_joint_torque(self, t):
        gust_cfg = self._inj("gust", default={}) or {}
        amp = self._inj_vec("gust", "joint_torque_amplitude_nm", default=[0.0] * 7, n=7)
        if not np.any(np.abs(amp) > 0.0):
            return np.zeros(7)
        period = float(gust_cfg.get("period_sec", 4.0))
        width = float(gust_cfg.get("width_sec", 0.8))
        if period <= 1e-9 or width <= 1e-9:
            return np.zeros(7)
        k = math.floor(float(t) / period)
        phase = float(t) - k * period
        if phase < 0.0 or phase >= width:
            return np.zeros(7)
        envelope = 0.5 * (1.0 - math.cos(2.0 * math.pi * phase / width))
        rng = np.random.default_rng([int(gust_cfg.get("seed", self._inj("seed", default=20260730))), int(k), 0x746175])
        direction = rng.uniform(-1.0, 1.0, 7)
        direction /= np.maximum(np.linalg.norm(direction), 1e-12)
        return envelope * amp * direction

    def _actual_plant_torque(self, tau_cmd, t, q_full, kin):
        if not (self.internal_plant and self._inj_enabled()):
            self.last_injection["tau_actual"] = np.asarray(tau_cmd, dtype=float).reshape(7).copy()
            self.last_injection["tau_disturbance"] = np.zeros(7)
            return self.last_injection["tau_actual"]

        tau_delayed = self._delayed_motor_command(tau_cmd, t)
        gain = self._inj_vec("motor", "torque_gain", default=[1.0] * 7, n=7)
        bias = self._inj_vec("motor", "torque_bias_nm", default=[0.0] * 7, n=7)
        noise_std = self._inj_vec("motor", "torque_noise_std_nm", default=[0.0] * 7, n=7)
        ripple = self._inj_vec("motor", "torque_ripple_amplitude_nm", default=[0.0] * 7, n=7)
        ripple_freq = self._inj_vec("motor", "torque_ripple_frequency_hz", default=[0.0] * 7, n=7)
        ripple_phase = self._inj_vec("motor", "torque_ripple_phase_rad", default=[0.0] * 7, n=7)
        tau_motor = (
            gain * tau_delayed
            + bias
            + ripple * np.sin(2.0 * math.pi * ripple_freq * float(t) + ripple_phase)
            + self.rng.normal(0.0, noise_std, 7)
        )

        deadband = self._inj_vec("backlash", "torque_deadband_nm", default=[0.0] * 7, n=7)
        tau_transmitted = np.sign(tau_motor) * np.maximum(np.abs(tau_motor) - deadband, 0.0)

        viscous = self._inj_vec("friction", "viscous_nm_per_rad_s", default=[0.0] * 7, n=7)
        coulomb = self._inj_vec("friction", "coulomb_nm", default=[0.0] * 7, n=7)
        stribeck = max(1e-6, float(self._inj("friction", "stribeck_velocity_rad_s", default=0.03)))
        tau_friction = viscous * self.qd + coulomb * np.tanh(self.qd / stribeck)

        tau_payload = self._payload_disturbance_torque(q_full, kin)
        tau_gust = self._gust_joint_torque(t)
        tau_actual = tau_transmitted - tau_friction + tau_payload + tau_gust
        self.last_injection["tau_actual"] = tau_actual.copy()
        self.last_injection["tau_disturbance"] = (tau_actual - np.asarray(tau_cmd, dtype=float).reshape(7)).copy()
        return tau_actual

    def _publish_joint_states(self):
        if self.joint_pub is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = list(DEFAULT_ARM_JOINT_NAMES)
        msg.position = self.q.tolist()
        msg.velocity = self.qd.tolist()
        msg.effort = [0.0] * 7
        self.joint_pub.publish(msg)
        self.last_joint_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _publish_runtime_markers(self, desired, actual_position):
        markers = [
            self._sphere_marker(5000, "current_target", desired.position, self._color(0.0, 0.9, 0.2, 1.0), 0.014),
            self._sphere_marker(5001, "current_actual", actual_position, self._color(0.9, 0.1, 0.1, 1.0), 0.014),
        ]
        for marker_id, segment in enumerate(self.actual_path_segments):
            if len(segment) < 2:
                continue
            markers.append(
                self._line_marker(
                    marker_id,
                    "whole_actual_path",
                    segment,
                    self._color(0.95, 0.15, 0.1, 0.9),
                    0.008,
                )
            )
        self.marker_pub.publish(MarkerArray(markers=markers))

    def _append_actual_path_point(self, point, pen_down, stroke_id):
        if not pen_down:
            self.actual_stroke_id = None
            return
        stroke_id = int(stroke_id)
        if self.actual_stroke_id != stroke_id:
            self.actual_path_segments.append([])
            self.actual_stroke_id = stroke_id
        self.actual_path_segments[-1].append(
            np.asarray(point, dtype=float).copy())

    def _publish_diagnostics(self, t, ref, actual_position, motor_command):
        tracking = Float64MultiArray()
        tracking.data = [
            float(t),
            *np.asarray(ref.position_w, dtype=float).tolist(),
            *np.asarray(actual_position, dtype=float).tolist(),
            *np.asarray(ref.position_w - actual_position, dtype=float).tolist(),
        ]
        self.tracking_pub.publish(tracking)
        torque = Float64MultiArray()
        torque.data = np.asarray(motor_command, dtype=float).tolist()
        self.torque_pub.publish(torque)

    def _publish_motor_command(self, motor):
        msg = MotorControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = self.q.tolist()
        msg.velocity = [0.0] * 7
        msg.current = motor.command.tolist()
        msg.p = [0.0] * 7
        msg.d = [0.0] * 7
        msg.enable = 1 if motor.enabled else 0
        self.command_pub.publish(msg)
        if not self.internal_plant:
            self.last_command_feedback_generation = self.feedback_generation

    def _run_approach(self, elapsed, now):
        duration = max(self.start_delay_sec, self.control_period)
        phase = float(np.clip(elapsed / duration, 0.0, 1.0))
        blend = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
        blend_d = (
            30.0 * phase**2 - 60.0 * phase**3 + 30.0 * phase**4
        ) / duration
        blend_dd = (
            60.0 * phase - 180.0 * phase**2 + 120.0 * phase**3
        ) / (duration * duration)
        delta_q = self.track_start_q - self.approach_initial_q
        q_des = self.approach_initial_q + blend * delta_q
        qd_des = blend_d * delta_q
        qdd_des = blend_dd * delta_q

        bs = self._simulation_base_sample(0.0)
        bs.velocity_lwa = np.zeros(6)
        bs.acceleration_lwa = np.zeros(6)
        self._publish_base_tf(bs)
        self._publish_base_truth(bs)
        q_full = self.model.make_q_full(
            bs.position_w, bs.quaternion_wxyz, self.q)
        v_full = self.model.make_v_full(bs.velocity_lwa, self.qd)
        qdd_cmd = (
            qdd_des
            + self.approach_kp * (q_des - self.q)
            + self.approach_kd * (qd_des - self.qd)
        )
        tau = self.model.compute_rnea_arm_torque(
            q_full, v_full, bs.acceleration_lwa, qdd_cmd)
        motor = self.adapter.torque_to_motor_command(
            tau,
            now,
            self.last_joint_stamp or now,
            now,
            self.control_period,
        )
        self._publish_motor_command(motor)
        if self.internal_plant:
            qdd = self.model.compute_arm_forward_dynamics(
                q_full, v_full, bs.acceleration_lwa, motor.command)
            self.qd = self.qd + qdd * self.control_period
            q_next = self.q + self.qd * self.control_period
            lower_hit = q_next < self.model.arm_lower_position_limit
            upper_hit = q_next > self.model.arm_upper_position_limit
            self.q = np.clip(
                q_next,
                self.model.arm_lower_position_limit,
                self.model.arm_upper_position_limit,
            )
            self.qd[lower_hit & (self.qd < 0.0)] = 0.0
            self.qd[upper_hit & (self.qd > 0.0)] = 0.0

        if (
            self.last_hold_publish_time is None
            or now - self.last_hold_publish_time
            >= 1.0 / self.visualization_rate_hz
        ):
            if self.internal_plant:
                self._publish_joint_states()
            q_visual = self.model.make_q_full(
                bs.position_w, bs.quaternion_wxyz, self.q)
            kin = self.model.compute_kinematics(
                q_visual,
                self.model.make_v_full(bs.velocity_lwa, self.qd),
            )
            ref = reference_sample(0.0, self.shape, self.derivative_dt)
            desired = PoseState(
                position=ref.position_w,
                quaternion_wxyz=ref.quaternion_wxyz,
                linear_velocity=np.zeros(3),
                angular_velocity=np.zeros(3),
            )
            self._publish_runtime_markers(desired, kin.ee_position)
            self.last_hold_publish_time = now

    def _publish_start_hold(self):
        if self.simulated_base:
            bs = self._simulation_base_sample(0.0)
        else:
            bs = base_sample(0.0, self.shape, 0.0, self.derivative_dt)
        self._publish_base_tf(bs)
        self._publish_base_truth(bs)
        if self.internal_plant:
            self._publish_joint_states()
        q_full = self.model.make_q_full(
            bs.position_w, bs.quaternion_wxyz, self.q)
        v_full = self.model.make_v_full(bs.velocity_lwa, self.qd)
        kin = self.model.compute_kinematics(q_full, v_full)
        ref = reference_sample(0.0, self.shape, self.derivative_dt)
        desired = PoseState(
            position=ref.position_w,
            quaternion_wxyz=ref.quaternion_wxyz,
            linear_velocity=np.zeros(3),
            angular_velocity=np.zeros(3),
        )
        self._publish_runtime_markers(desired, kin.ee_position)

    def _control_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.done:
            return
        if not self.internal_plant and not self.has_joint_feedback:
            if (
                self.last_hold_publish_time is None
                or now - self.last_hold_publish_time >= 1.0
            ):
                self.get_logger().warning(
                    "Waiting for /joint_states from student_arm_node")
                self.last_hold_publish_time = now
            return
        if (
            not self.internal_plant
            and self.feedback_generation
            <= self.last_command_feedback_generation
        ):
            return
        if self.start_time is None:
            self.start_time = now
            self.last_update_time = now
        elapsed = now - self.start_time
        if elapsed < self.start_delay_sec:
            self.last_update_time = now
            if (
                self.reference_markers is not None
                and (
                    self.last_reference_publish_time is None
                    or now - self.last_reference_publish_time >= 1.0
                )
            ):
                self.reference_marker_pub.publish(self.reference_markers)
                self.last_reference_publish_time = now
            if self.simulated_base:
                self._run_approach(elapsed, now)
            elif (
                self.last_hold_publish_time is None
                or now - self.last_hold_publish_time
                >= 1.0 / self.visualization_rate_hz
            ):
                self._publish_start_hold()
                self.last_hold_publish_time = now
            return
        if self.simulated_base:
            t = self.sim_time
            dt = self.control_period
        else:
            t = elapsed - self.start_delay_sec
            dt = max(1e-5, now - self.last_update_time)
        self.last_update_time = now
        if t > self.duration_sec:
            self._publish_motor_command(
                self.adapter.zero(
                    list(DEFAULT_ARM_JOINT_NAMES), now, "trajectory complete"))
            self.done = True
            self._write_summary()
            return

        if self.simulated_base:
            bs_true = self._simulation_base_sample(t)
        else:
            bs_true = base_sample(t, self.shape, t, self.derivative_dt)
        bs_est = self._noisy_delayed_base_sample(t, bs_true)
        self._publish_base_tf(bs_true)
        self._publish_base_truth(bs_true)
        q_full_true = self.model.make_q_full(bs_true.position_w, bs_true.quaternion_wxyz, self.q)
        v_full_true = self.model.make_v_full(bs_true.velocity_lwa, self.qd)
        kin_true = self.model.compute_kinematics(q_full_true, v_full_true)
        q_full_ctrl = self.model.make_q_full(bs_est.position_w, bs_est.quaternion_wxyz, self.q)
        v_full_ctrl = self.model.make_v_full(bs_est.velocity_lwa, self.qd)
        kin_ctrl = self.model.compute_kinematics(q_full_ctrl, v_full_ctrl)
        ref = reference_sample(t, self.shape, self.derivative_dt)
        desired = PoseState(
            position=ref.position_w,
            quaternion_wxyz=ref.quaternion_wxyz,
            linear_velocity=ref.linear_velocity_w,
            angular_velocity=ref.angular_velocity_w,
            linear_acceleration=ref.linear_acceleration_w,
            angular_acceleration=ref.angular_acceleration_w,
        )
        actual = PoseState(
            position=kin_ctrl.ee_position,
            quaternion_wxyz=kin_ctrl.ee_quaternion_wxyz,
            linear_velocity=kin_ctrl.ee_velocity_lwa[:3],
            angular_velocity=kin_ctrl.ee_velocity_lwa[3:],
        )
        smc = self.controller.compute(desired, actual)
        b6 = self.model.compute_task_bias_acceleration(q_full_ctrl, v_full_ctrl, bs_est.acceleration_lwa)
        rr = resolve_acceleration(
            kin_ctrl.J_arm_lwa,
            self.model.compute_arm_mass_block(q_full_ctrl),
            smc.nu - b6,
            self.q,
            self.qd,
            self.config["nullspace"]["q_nominal"],
            self.config["nullspace"]["kp"],
            self.config["nullspace"]["kd"],
            float(self.config["task"]["characteristic_length"]),
            float(self.config["singularity"]["sigma_warn"]),
            float(self.config["singularity"]["sigma_critical"]),
            float(self.config["singularity"]["damping_min"]),
            float(self.config["singularity"]["damping_max"]),
            self.config["nullspace"].get("joint_acceleration_weight", None),
        )
        tau = self.model.compute_rnea_arm_torque(q_full_ctrl, v_full_ctrl, bs_est.acceleration_lwa, rr.qdd_cmd)
        motor = self.adapter.torque_to_motor_command(tau, now, self.last_joint_stamp or now, now, dt)
        recorded_q = self.q.copy()
        recorded_qd = self.qd.copy()
        self._publish_motor_command(motor)

        if self.internal_plant:
            tau_actual = self._actual_plant_torque(motor.command, t, q_full_true, kin_true)
            qdd = self.model.compute_arm_forward_dynamics(
                q_full_true, v_full_true, bs_true.acceleration_lwa, tau_actual)
            self.qd = self.qd + qdd * dt
            q_next = self.q + self.qd * dt
            lower_hit = q_next < self.model.arm_lower_position_limit
            upper_hit = q_next > self.model.arm_upper_position_limit
            self.q = np.clip(q_next, self.model.arm_lower_position_limit, self.model.arm_upper_position_limit)
            self.qd[lower_hit & (self.qd < 0.0)] = 0.0
            self.qd[upper_hit & (self.qd > 0.0)] = 0.0
            if bool(self.get_parameter("publish_joint_states_in_sim").value):
                self._publish_joint_states()
        else:
            tau_actual = self.feedback_effort.copy()
            self.last_injection["tau_actual"] = tau_actual.copy()
            self.last_injection["tau_disturbance"] = (
                tau_actual - motor.command)

        pos_error = float(np.linalg.norm(ref.position_w - kin_true.ee_position))
        rot_error = float(np.linalg.norm(quaternion_log_error(ref.quaternion_wxyz, kin_true.ee_quaternion_wxyz)))
        self.error_samples.append(pos_error)
        self.rot_samples.append(rot_error)
        error_xyz = ref.position_w - kin_true.ee_position
        tau_disturbance = self.last_injection["tau_disturbance"]
        row = {
            "t": t,
            "pos_error_m": pos_error,
            "rot_error_rad": rot_error,
            "tau_max_abs": float(np.max(np.abs(motor.command))),
            "sigma_min": rr.sigma_min,
            "damping": rr.damping,
            "nullspace_leakage": rr.nullspace_leakage_norm,
            "enabled": int(motor.enabled),
            "pen_down": int(ref.pen_down),
            "stroke_id": int(ref.stroke_id),
            "simulation_injection_enabled": int(
                self.simulated_base and self._inj_enabled()),
            "gust_active": int(self.last_injection["gust_active"]),
            "gust_envelope": float(self.last_injection["gust_envelope"]),
            "tau_actual_max_abs": float(np.max(np.abs(tau_actual))),
            "tau_disturbance_max_abs": float(np.max(np.abs(tau_disturbance))),
        }
        for axis, value in zip("xyz", ref.position_w):
            row[f"desired_{axis}_m"] = float(value)
        for axis, value in zip("xyz", kin_true.ee_position):
            row[f"actual_{axis}_m"] = float(value)
        for axis, value in zip("xyz", error_xyz):
            row[f"error_{axis}_m"] = float(value)
        for axis, value in zip("xyz", bs_true.position_w):
            row[f"base_{axis}_m"] = float(value)
        for name, value in zip(("w", "x", "y", "z"), bs_true.quaternion_wxyz):
            row[f"base_q_{name}"] = float(value)
        for axis, value in zip("xyz", bs_est.position_w):
            row[f"base_est_{axis}_m"] = float(value)
        for name, value in zip(("w", "x", "y", "z"), bs_est.quaternion_wxyz):
            row[f"base_est_q_{name}"] = float(value)
        for axis, value in zip("xyz", self.last_injection["base_gust_offset"]):
            row[f"base_gust_{axis}_m"] = float(value)
        for i in range(7):
            row[f"joint_{i + 1}_rad"] = float(recorded_q[i])
            row[f"joint_{i + 1}_velocity_rad_s"] = float(recorded_qd[i])
            row[f"joint_{i + 1}_torque_nm"] = float(motor.command[i])
            row[f"joint_{i + 1}_tau_actual_nm"] = float(tau_actual[i])
            row[f"joint_{i + 1}_tau_disturbance_nm"] = float(tau_disturbance[i])
        self.metrics_writer.writerow(row)
        if len(self.error_samples) % 50 == 0:
            self.metrics_file.flush()
        self._publish_diagnostics(t, ref, kin_true.ee_position, motor.command)
        if len(self.error_samples) % self.visualization_stride == 0:
            self._append_actual_path_point(
                kin_true.ee_position, ref.pen_down, ref.stroke_id)
            self._publish_runtime_markers(desired, kin_true.ee_position)
        if (
            self.reference_markers is not None
            and len(self.error_samples)
            % max(1, int(round(self.runtime_control_rate_hz))) == 0
        ):
            self.reference_marker_pub.publish(self.reference_markers)
        if self.simulated_base:
            self.sim_time += self.control_period

    def _write_summary(self):
        self.metrics_file.flush()
        errors = np.asarray(self.error_samples, dtype=float)
        rots = np.asarray(self.rot_samples, dtype=float)
        summary = {
            "samples": int(errors.size),
            "position_rms_m": float(np.sqrt(np.mean(errors * errors))) if errors.size else float("nan"),
            "position_peak_m": float(np.max(errors)) if errors.size else float("nan"),
            "orientation_rms_rad": float(np.sqrt(np.mean(rots * rots))) if rots.size else float("nan"),
            "orientation_peak_rad": float(np.max(rots)) if rots.size else float("nan"),
            "pass_1mm_rms_2mm_peak": bool(errors.size and np.sqrt(np.mean(errors * errors)) < 0.001 and np.max(errors) < 0.002),
            "metrics_csv": str(self.metrics_path),
        }
        path = self.data_dir / f"whole_task_smc_summary_{self.run_stamp}.json"
        summary["summary_json"] = str(path)
        try:
            from task_space_smc.plot_whole_task_smc import generate_report
            summary["plots"] = generate_report(
                self.metrics_path,
                self.data_dir / f"whole_task_smc_report_{self.run_stamp}",
            )
        except Exception as exc:
            summary["plot_error"] = str(exc)
            self.get_logger().warning(f"Could not generate report plots: {exc}")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)
        self.get_logger().info(json.dumps(summary, indent=2))


def main(args=None):
    if rclpy is None:
        raise RuntimeError("rclpy is not available")
    rclpy.init(args=args)
    node = WholeTaskSmcRosRunner()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok() and not node.done:
            node._write_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
