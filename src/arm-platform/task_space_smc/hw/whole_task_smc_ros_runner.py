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


    _here = Path(__file__).resolve().parent
    _root = _here
    while (_root / "__init__.py").exists():
        _root = _root.parent
    sys.path = [p for p in sys.path if Path(p or ".").resolve() != _here]
    sys.path.insert(0, str(_root))

from task_space_smc.core.actuator_adapter import (
    ActuatorAdapter, ActuatorLimits, uniform_acceleration_torque_scale)
from task_space_smc.core.approach_reference import TaskSpaceApproach
from task_space_smc.core.base_state import BaseState, RosFeedbackBaseState
from task_space_smc.core.handover_blend import HandoverBlend, trajectory_clock
from task_space_smc.core.loop_delay import (
    CommandDelayBuffer, FeedbackDelayModel, resolve_simulation_delay)
from task_space_smc.core.state_estimator import resolve_estimator
from task_space_smc.core.velocity_filter import JointVelocityFilter
from task_space_smc.core.friction_feedforward import (
    FrictionFeedforwardConfig, approach_torque_stiffness, residual_deadband,
    stiction_feedforward)
from task_space_smc.core.model import (
    DEFAULT_ARM_JOINT_NAMES, FloatingBaseArmModel, load_arm6_calibration)
from task_space_smc.core.config_loader import controller_from_config, default_config_path
from task_space_smc.core.pose_error import PoseState
from task_space_smc.core.quaternion_math import (
    quat_to_rotation,
    quaternion_log_error,
    rotation_to_quat,
    wxyz_to_ros_xyzw,
)
from task_space_smc.core.redundancy_resolution import (
    FeasibilityLimits, LimitAvoidance, pen_axis_task, resolve_acceleration)
from task_space_smc.core.trajectory_reference import (
    AnalyticBaseSample,
    base_sample,
    drawing_segments,
    reference_sample,
    traj,
    whole_period,
)


METRICS_FLUSH_INTERVAL = 1000


def tune_gc_for_realtime():
    import gc
    gc.collect()
    gc.freeze()


MAX_PUBLISHED_PATH_POINTS = 600


def decimate_path(points, max_points: int):
    n = len(points)
    limit = max(2, int(max_points))
    if n <= limit:
        return points

    idx = np.linspace(0, n - 1, limit - 1).astype(int)
    out = [points[i] for i in idx]
    if idx[-1] != n - 1:
        out.append(points[n - 1])
    return out


def trajectory_period(shape: str) -> float:
    if shape == "whole":
        return whole_period()
    return float(traj.ee_trajectory_period(shape))

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
except Exception:
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
        self.declare_parameter("initial_joint_positions", [0.0] * 6)
        self.declare_parameter("base_state_mode", "analytic")
        self.declare_parameter("external_base_topic", "/floating_base/vicon")

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
        self.base_state_mode = str(self.get_parameter("base_state_mode").value)
        if self.base_state_mode not in ("analytic", "vicon"):
            raise ValueError("base_state_mode must be analytic or vicon")
        if self.base_state_mode == "vicon" and self.mode != "hardware":
            raise ValueError("base_state_mode=vicon is supported only with mode=hardware")
        self.uses_external_base = self.base_state_mode == "vicon"
        self.external_base = RosFeedbackBaseState(
            timeout_sec=float(self.config.get("safety", {}).get(
                "base_state_timeout_sec", 0.1)))
        self._external_base_initialized = not self.uses_external_base
        self.derivative_dt = float(self.get_parameter("derivative_dt_sec").value)
        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.start_delay_sec = max(
            0.0, float(self.get_parameter("start_delay_sec").value))
        if self.duration_sec <= 0.0:


            self.duration_sec = trajectory_period(self.shape)
        self.data_dir = Path(str(self.get_parameter("data_dir").value))
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        urdf_path = self.get_parameter("urdf_path").value or None


        ee_frame = self.config.get("ee_frame") or None
        self.model = FloatingBaseArmModel(
            urdf_path=urdf_path, ee_frame_name=ee_frame)
        self.get_logger().info(
            f"model: {Path(self.model.urdf_path).name} "
            f"n_arm={self.model.n_arm} ee={self.model.ee_frame_name}")
        self.controller = controller_from_config(self.config)

        self.n_arm = self.model.n_arm
        self.arm_joint_names = list(self.model.arm_joint_names)


        _ns = self.config.get("nullspace") or {}
        _ns_on = bool(_ns.get("enabled", True)) and "q_nominal" in _ns


        self._ns_q_nominal_is_track_start = (
            _ns_on and isinstance(_ns.get("q_nominal"), str)
            and _ns["q_nominal"].strip() == "track_start")
        if self._ns_q_nominal_is_track_start:
            self.ns_q_nominal = np.zeros(self.n_arm)
        else:
            self.ns_q_nominal = (np.asarray(_ns["q_nominal"], dtype=float)
                                 if _ns_on else np.zeros(self.n_arm))
        self.ns_kp = (np.asarray(_ns["kp"], dtype=float)
                      if _ns_on else np.zeros(self.n_arm))
        self.ns_kd = (np.asarray(_ns["kd"], dtype=float)
                      if _ns_on else np.zeros(self.n_arm))
        self.ns_accel_weight = (_ns.get("joint_acceleration_weight")
                                if _ns_on else None)

        self.q_lower = self.model.model.lowerPositionLimit[7:].copy()
        self.q_upper = self.model.model.upperPositionLimit[7:].copy()
        _task = self.config.get("task", {})
        self.orientation_mode = str(_task.get("orientation_mode", "full"))
        self.pen_axis_local = np.asarray(
            _task.get("pen_axis_local", [1.0, 0.0, 0.0]), dtype=float)
        self.reference_offset_local = np.asarray(
            _task.get("reference_offset_local_m", [0.0, 0.0, 0.0]),
            dtype=float,
        ).reshape(3)
        _la = _task.get("limit_avoidance") or {}
        self.limit_avoidance = None
        if float(_la.get("gain", 0.0)) > 0.0:
            self.limit_avoidance = LimitAvoidance(
                lower=np.asarray(self.q_lower, dtype=float),
                upper=np.asarray(self.q_upper, dtype=float),
                gain=float(_la["gain"]),
                activate_rad=float(_la.get("activate_rad", 0.5)))


        _init = self.config.get("initial_pose_rad")
        if _init is None:
            _init = self.config.get("nullspace", {}).get("q_nominal")
            if isinstance(_init, str):
                _init = None
        if _init is None or len(_init) != self.n_arm:
            raise RuntimeError(
                f"config must provide initial_pose_rad with {self.n_arm} values "
                f"(nullspace is disabled for the 6-joint arm)")
        self.q = np.asarray(_init, dtype=float).reshape(self.n_arm).copy()
        self.qd = np.zeros(self.n_arm)
        self._qd_raw = np.zeros(self.n_arm)
        self.last_joint_stamp = None
        self.has_joint_feedback = self.internal_plant
        self.feedback_generation = 0
        self.last_command_feedback_generation = -1
        self.feedback_effort = np.zeros(self.n_arm)


        self.q_ctrl = self.q
        self.qd_ctrl = self.qd
        self.effort_ctrl = self.feedback_effort
        self.start_time = None
        self.last_update_time = None
        self._watchdog_tripped = False
        self._watchdog_trip_time = None
        self.watchdog_reason = ""


        _wd = (self.config.get("watchdog") or {})
        self.stop_on_watchdog = bool(_wd.get("stop_on_trip", True))
        self.watchdog_hold_sec = float(_wd.get("hold_sec", 1.0))
        self._watchdog_hold_q = None
        self.last_position_error = None
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
            "tau_actual": np.zeros(self.n_arm),
            "tau_disturbance": np.zeros(self.n_arm),
            "gust_active": 0,
            "gust_envelope": 0.0,
            "base_gust_offset": np.zeros(3),
        }


        _limits_cfg = self.config.get("limits", {})
        _joint_velocity = _limits_cfg.get("joint_velocity")
        self.feasibility_velocity = (
            np.asarray(_joint_velocity, dtype=float).reshape(self.n_arm)
            if _joint_velocity is not None else None)
        self.feasibility_brake_safety = float(
            _limits_cfg.get("joint_brake_safety", 0.0))
        self.feasibility_torque = np.asarray(
            _limits_cfg["torque"], dtype=float).reshape(self.n_arm)


        self.torque_saturation_mode = str(
            _limits_cfg.get("torque_saturation", "per_axis"))

        limits = ActuatorLimits(
            torque=np.asarray(self.config["limits"]["torque"], dtype=float).reshape(self.n_arm),
            torque_rate=np.asarray(self.config["limits"]["torque_rate"], dtype=float).reshape(self.n_arm),
            current=np.asarray(self.config["limits"]["current"], dtype=float).reshape(self.n_arm),
        )
        self.adapter = ActuatorAdapter(
            joint_names=list(self.arm_joint_names),
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
        ).reshape(self.n_arm)
        self.approach_kp = np.asarray(
            approach.get("kp", [60.0, 60.0, 60.0, 50.0, 50.0, 40.0, 40.0]),
            dtype=float,
        ).reshape(self.n_arm)
        self.approach_kd = np.asarray(
            approach.get("kd", [16.0, 16.0, 16.0, 14.0, 14.0, 12.0, 12.0]),
            dtype=float,
        ).reshape(self.n_arm)


        self.approach_mode = str(approach.get("mode", "joint_space")).strip().lower()
        if self.approach_mode not in ("joint_space", "task_space"):
            raise ValueError(
                f"approach.mode = {self.approach_mode!r} 不合法，"
                "可选 ['joint_space', 'task_space']")


        _guard = approach.get("workspace_guard") or {}
        self.approach_guard = dict(
            enabled=bool(_guard.get("enabled", True)),
            desk_plane_z=float(_guard.get("desk_plane_z", 0.0)),
            desk_edge_x=float(_guard.get("desk_edge_x", 0.02)),
            desk_clearance=float(_guard.get("desk_clearance", 0.08)),
            fence_y=float(_guard.get("fence_y", 0.40)),
            fence_height=float(_guard.get("fence_height", 0.331)),
            fence_clearance=float(_guard.get("fence_clearance", 0.03)),
            max_linear_speed=float(_guard.get("max_linear_speed", 0.15)),
            max_linear_accel=float(_guard.get("max_linear_accel", 0.60)),
        )


        self.task_approach = None
        self.task_approach_rejected = False
        self.approach_friction = self._build_approach_friction(approach)


        _tf = dict((self.config.get("tracking") or {}).get(
            "friction_feedforward", {}) or {})


        if _tf.get("stiction_nm") is None:
            _tf["stiction_nm"] = self.approach_friction.stiction_nm.tolist()
        self.tracking_friction = self._build_approach_friction(
            {"friction_feedforward": _tf}, quiet=True)
        if self.tracking_friction.enabled:
            self.get_logger().info(
                "跟踪段静摩擦前馈: 开启 scale=%.2f κ=%.4f rad (%.2f°)  f_s=%s Nm"
                % (self.tracking_friction.scale,
                   self.tracking_friction.velocity_epsilon
                   / max(self.tracking_friction.error_gain, 1e-9),
                   np.degrees(self.tracking_friction.velocity_epsilon
                              / max(self.tracking_friction.error_gain, 1e-9)),
                   np.round(self.tracking_friction.stiction_nm, 3).tolist()))

        _vf = (self.config.get("feedback") or {}).get("velocity_lowpass_hz", 0.0)
        self.velocity_filter = JointVelocityFilter(_vf, self.n_arm)
        if self.velocity_filter.enabled:
            self.get_logger().info(
                "关节速度反馈低通: 截止 %s Hz，在 25 Hz 处引入相位滞后 %s°"
                % (np.round(self.velocity_filter.cutoff_hz, 1).tolist(),
                   np.round(self.velocity_filter.phase_lag_deg(25.0), 1).tolist()))
            self.get_logger().warn(
                "⚠️ 降噪与相位裕度是【对冲】的：实测该噪声为白噪声，压得动就得把"
                "截止压低，而失稳频率 25.8 Hz 正是最付不起相位的地方。若失稳源于"
                "回路负裕度而非噪声放大，本滤波会让它更早发散 —— 以实测为准。")

        handover_cfg = self.config.get("handover", {}) or {}
        self.handover_blend = HandoverBlend(
            float(handover_cfg.get("reference_blend_sec", 0.0)))


        self.freeze_trajectory_during_blend = bool(
            handover_cfg.get("freeze_trajectory_during_blend", False))
        if self.simulated_base:
            self.track_start_q = self._solve_initial_pose(self.q)
            initial_joint_positions = np.asarray(
                self.get_parameter("initial_joint_positions").value,
                dtype=float,
            ).reshape(self.n_arm)
            if np.any(np.abs(initial_joint_positions) > 1e-12):
                self.q = np.asarray(
                    initial_joint_positions, dtype=float).reshape(self.n_arm)
            else:
                self.q = self.track_start_q + (
                    self.approach_initial_offset
                    if self.start_delay_sec > 0.0 else np.zeros(self.n_arm)
                )
            self.q = np.clip(
                self.q,
                self.model.arm_lower_position_limit,
                self.model.arm_upper_position_limit,
            )
            self.approach_initial_q = self.q.copy()
        else:


            self.track_start_q = self._solve_initial_pose(self.q)
            self.approach_initial_q = self.q.copy()
            jump = float(np.max(np.abs(self.track_start_q - self.approach_initial_q)))
            self.get_logger().info(
                f"approach: 需移动 {np.degrees(jump):.1f}° "
                f"(峰值速度约 {1.875 * jump / max(self.start_delay_sec, 1e-3):.2f} rad/s)")
        self._resolve_track_start_nullspace_nominal()

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
        if self.uses_external_base:
            self.create_subscription(
                FloatingBaseState,
                str(self.get_parameter("external_base_topic").value),
                self._external_base_cb,
                10,
            )

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
            + [f"joint_{i + 1}_rad" for i in range(self.n_arm)]
            + [f"joint_{i + 1}_velocity_rad_s" for i in range(self.n_arm)]
            + [f"joint_{i + 1}_torque_nm" for i in range(self.n_arm)]
            + [f"joint_{i + 1}_tau_actual_nm" for i in range(self.n_arm)]
            + [f"joint_{i + 1}_tau_disturbance_nm" for i in range(self.n_arm)]


            + [f"approach_q_des_{i + 1}_rad" for i in range(self.n_arm)]
            + [f"approach_q_err_{i + 1}_rad" for i in range(self.n_arm)]


            + [f"joint_{i + 1}_q_meas_rad" for i in range(self.n_arm)]
            + [f"joint_{i + 1}_qd_meas_rad_s" for i in range(self.n_arm)]
            + [f"joint_{i + 1}_est_acc_rad_s2" for i in range(self.n_arm)]
        )
        self.metrics_writer = csv.DictWriter(
            self.metrics_file,
            fieldnames=[
                "t", "phase", "pos_error_m", "rot_error_rad", "tau_max_abs",
                "sigma_min", "damping", "nullspace_leakage",
                "feasibility_scale", "nullspace_scale", "torque_scale",
                "enabled",
                "pen_down", "stroke_id", "simulation_injection_enabled",
                "gust_active", "gust_envelope", "tau_actual_max_abs",
                "tau_disturbance_max_abs", "approach_tau_friction_max_nm",
                "handover_blend",


                "trajectory_clock",


                "estimator_predict_clamp_hits",
                "estimator_hold_count",
            ] + vector_fields,
        )
        self.metrics_writer.writeheader()
        self.error_samples = []
        self.approach_rows_written = 0
        self.handover_q = None
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


        _plan = resolve_simulation_delay(
            self.config.get("simulation"), self.control_period, self.mode)
        self.command_delay = _plan.command_delay
        self.feedback_delay = _plan.feedback
        for _msg in _plan.messages:
            self.get_logger().info(_msg)
        for _msg in _plan.warnings:
            self.get_logger().warning("⚠️ " + _msg)


        _est_plan = resolve_estimator(
            self.config.get("estimator"), self.n_arm, self.control_period,
            self.mode, feedback_delay_enabled=self.feedback_delay.enabled)
        self.estimator = _est_plan.estimator
        for _msg in _est_plan.messages:
            self.get_logger().info(_msg)
        for _msg in _est_plan.warnings:
            self.get_logger().warning("⚠️ " + _msg)


        self._q_meas = self.q
        self._qd_meas = self.qd


        self._estimator_last_time = None


        self._refresh_controller_feedback()


        tune_gc_for_realtime()
        self.timer = self.create_timer(period, self._control_loop)
        self.get_logger().info(
            f"Whole-task SMC runner mode={self.mode}, shape={self.shape}, "
            f"start_delay={self.start_delay_sec:.3f}s, duration={self.duration_sec:.3f}s, "
            f"config={self.config_path}, metrics={self.metrics_path}"
        )
        if self.uses_external_base:
            self.get_logger().warning(
                "Vicon base feedback enabled: waiting for a fresh %s message; "
                "no torque command will be sent before it arrives"
                % str(self.get_parameter("external_base_topic").value))
        if (self.freeze_trajectory_during_blend
                and self.handover_blend.duration_sec > 0.0):
            self.get_logger().info(
                "拉入期间冻结轨迹时钟: t<%.1fs 参考量停在轨迹起点，"
                "t>=%.1fs 起轨迹时钟从 0 走（总时长需 >= blend_sec + 轨迹周期）"
                % (self.handover_blend.duration_sec,
                   self.handover_blend.duration_sec))
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

    def _trajectory_clock(self, t: float) -> float:
        return trajectory_clock(
            t,
            self.handover_blend.duration_sec,
            self.freeze_trajectory_during_blend,
        )

    def _reference_sample(self, t: float):
        """Return the desired pose of the configured end-effector frame.

        ``reference_offset_local_m`` translates the nominal drawing point in
        its local frame.  It lets a configuration whose physical tool has
        been removed (for example ``link7`` -> ``link6``) move each station
        inward without altering the shared global trajectory definition.
        """
        ref = reference_sample(t, self.shape, self.derivative_dt)
        if not np.any(self.reference_offset_local):
            return ref
        offset_w = ref.rotation_w @ self.reference_offset_local
        omega_w = ref.angular_velocity_w
        alpha_w = ref.angular_acceleration_w
        ref.position_w = ref.position_w + offset_w
        ref.linear_velocity_w = (
            ref.linear_velocity_w + np.cross(omega_w, offset_w))
        ref.linear_acceleration_w = (
            ref.linear_acceleration_w
            + np.cross(alpha_w, offset_w)
            + np.cross(omega_w, np.cross(omega_w, offset_w)))
        return ref

    def _solve_initial_pose(self, q_seed: np.ndarray,
                            base_state: BaseState | None = None) -> np.ndarray:
        if base_state is not None:
            bs = base_state
        elif getattr(self, "simulated_base", False):
            bs = self._simulation_base_sample(0.0)
        else:
            bs = base_sample(0.0, self.shape, 0.0, self.derivative_dt)
        ref = self._reference_sample(0.0)
        q_lower = self.model.model.lowerPositionLimit[7:].copy()
        q_upper = self.model.model.upperPositionLimit[7:].copy()
        seed0 = np.asarray(q_seed, dtype=float).reshape(self.n_arm).copy()
        pen_des = np.asarray(ref.rotation_w, dtype=float).reshape(3, 3) @ self.pen_axis_local
        pen_des = pen_des / max(np.linalg.norm(pen_des), 1e-12)
        scale = np.diag([1.0, 1.0, 1.0, 0.12, 0.12])
        v_zero = self.model.make_v_full(bs.velocity_lwa, np.zeros(self.n_arm))
        rng = np.random.default_rng(20260802)

        def _refine(seed):
            q = seed.copy()
            for _ in range(250):
                q_full = self.model.make_q_full(bs.position_w, bs.quaternion_wxyz, q)
                kin = self.model.compute_kinematics(q_full, v_zero)
                pen = kin.ee_rotation @ self.pen_axis_local
                e_pos = ref.position_w - kin.ee_position
                e_pen = np.cross(pen / max(np.linalg.norm(pen), 1e-12), pen_des)
                if np.linalg.norm(e_pos) < 5e-5 and np.linalg.norm(e_pen) < 2e-3:
                    break
                P = np.eye(3) - np.outer(pen_des, pen_des)
                B = np.linalg.svd(P)[0][:, :2].T
                J = scale @ np.vstack([kin.J_arm_lwa[:3, :], B @ kin.J_arm_lwa[3:, :]])
                y = scale @ np.concatenate([e_pos, B @ e_pen])
                dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(5), y)
                q = np.clip(q + 0.45 * dq, q_lower, q_upper)
            return q, float(np.linalg.norm(e_pos))

        best = None
        for k in range(60):
            seed = seed0 if k == 0 else q_lower + (q_upper - q_lower) * rng.random(self.n_arm)
            q, err = _refine(seed)
            if err > 1e-4:
                continue
            margin = float(min(np.min(q - q_lower), np.min(q_upper - q)))
            disp = float(np.max(np.abs(q - seed0)))
            score = margin - 0.15 * disp
            if best is None or score > best[0]:
                best = (score, q, margin, disp)
        if best is None:
            self.get_logger().warn(
                "初始位形 5D 求解未收敛，退回种子位形（approach 段将不移动）")
            return seed0
        self.get_logger().info(
            f"初始位形: 限位余量 {best[2]:.3f} rad，需移动 {np.degrees(best[3]):.1f}°")
        return best[1]

    def destroy_node(self):
        if getattr(self, "metrics_file", None):
            self.metrics_file.close()
        super().destroy_node()

    def _joint_state_cb(self, msg):
        if self.internal_plant:
            return
        name_to_index = {name: i for i, name in enumerate(msg.name)}


        for j, name in enumerate(self.arm_joint_names):
            if name not in name_to_index:
                return
            i = name_to_index[name]
            self.q[j] = msg.position[i]
            self._qd_raw[j] = msg.velocity[i] if i < len(msg.velocity) else 0.0
            self.feedback_effort[j] = (
                msg.effort[i] if i < len(msg.effort) else 0.0)


        self.qd = self.velocity_filter(self._qd_raw, self.control_period)
        self.last_joint_stamp = self.get_clock().now().nanoseconds * 1e-9
        if not self.has_joint_feedback:


            self.estimator.reset()
            self._estimator_last_time = None
        self.has_joint_feedback = True
        self.feedback_generation += 1

    def _external_base_cb(self, msg):
        q_xyzw = np.array([
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        ], dtype=float)
        norm = float(np.linalg.norm(q_xyzw))
        vector = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
            msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z,
            msg.acceleration.linear.x, msg.acceleration.linear.y,
            msg.acceleration.linear.z, msg.acceleration.angular.x,
            msg.acceleration.angular.y, msg.acceleration.angular.z,
        ], dtype=float)
        if not np.all(np.isfinite(vector)) or not np.isfinite(norm) or norm < 1e-9:
            self.get_logger().warning("Ignoring invalid external base-state sample")
            return
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        self.external_base.update(BaseState(
            position_w=vector[:3].copy(),
            quaternion_wxyz=np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]) / norm,
            rotation_wb=quat_to_rotation(
                np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]) / norm),
            velocity_lwa=vector[3:9].copy(),
            acceleration_lwa=vector[9:15].copy(),
            stamp=float(stamp),
        ))

    def _base_for_control(self, now: float) -> BaseState:
        """Return the current base state in the controller world frame."""
        if self.uses_external_base:
            return self.external_base.latest(now)
        if self.simulated_base:
            return self._simulation_base_sample(0.0)
        return base_sample(0.0, self.shape, 0.0, self.derivative_dt)

    def _initialize_external_base(self, base: BaseState) -> None:
        if self._external_base_initialized:
            return
        self.track_start_q = self._solve_initial_pose(self.q, base)
        self.approach_initial_q = self.q.copy()
        jump = float(np.max(np.abs(self.track_start_q - self.approach_initial_q)))
        self._resolve_track_start_nullspace_nominal()
        self._external_base_initialized = True
        self.get_logger().info(
            "Vicon base accepted; current pose initialised. approach needs %.1f° max joint motion"
            % np.degrees(jump))

    def _estimator_dt(self, now):
        if self.simulated_base:
            return self.control_period
        if now is None:
            return None
        prev = self._estimator_last_time
        self._estimator_last_time = now
        return None if prev is None else (now - prev)

    def _refresh_controller_feedback(self, now=None):
        q_d, qd_d, self.effort_ctrl = self.feedback_delay(
            self.q, self.qd, self.feedback_effort)
        self._q_meas, self._qd_meas = q_d, qd_d
        if not self.estimator.enabled:


            self.q_ctrl, self.qd_ctrl = q_d, qd_d
            return
        self.q_ctrl, self.qd_ctrl = self.estimator(
            q_d, qd_d, self._estimator_dt(now))

    def _fill_estimator_row(self, row):
        row["estimator_predict_clamp_hits"] = float(
            self.estimator.predict_clamp_hits)
        row["estimator_hold_count"] = float(self.estimator.hold_count)
        est_acc = self.estimator.acceleration_estimate
        for i in range(self.n_arm):
            row[f"joint_{i + 1}_q_meas_rad"] = float(self._q_meas[i])
            row[f"joint_{i + 1}_qd_meas_rad_s"] = float(self._qd_meas[i])
            row[f"joint_{i + 1}_est_acc_rad_s2"] = float(est_acc[i])

    def _push_estimator_command(self, q_full_ctrl, v_full_ctrl, base_accel,
                                tau_cmd, tau_extra, dt, tau_bias=None):
        if not self.estimator.wants_commanded_acceleration:
            return
        if tau_bias is None:
            tau_bias = self.model.compute_rnea_arm_torque(
                q_full_ctrl, v_full_ctrl, base_accel, np.zeros(self.n_arm))
        net = (np.asarray(tau_cmd, dtype=float)
               - np.asarray(tau_bias, dtype=float)
               - np.asarray(tau_extra, dtype=float))
        try:
            qdd_eff = np.linalg.solve(
                self.model.compute_arm_mass_block(q_full_ctrl), net)
        except np.linalg.LinAlgError:


            return
        self.estimator.push_command(qdd_eff, dt)

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

    def _drawing_segments(self):
        sample_count = int(self.get_parameter("marker_sample_count").value)
        if not np.any(self.reference_offset_local):
            return drawing_segments(self.shape, sample_count)

        period = trajectory_period(self.shape)
        segments = []
        current = []
        stroke_id = None
        for t in np.linspace(0.0, period, max(2, sample_count), endpoint=False):
            ref = self._reference_sample(float(t))
            if not ref.pen_down:
                if len(current) >= 2:
                    segments.append(np.asarray(current))
                current = []
                stroke_id = None
                continue
            if stroke_id is not None and ref.stroke_id != stroke_id:
                if len(current) >= 2:
                    segments.append(np.asarray(current))
                current = []
            current.append(ref.position_w)
            stroke_id = ref.stroke_id
        if len(current) >= 2:
            segments.append(np.asarray(current))
        return segments

    def _publish_static_path(self):
        markers = []
        for i, segment in enumerate(self._drawing_segments()):
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

    def _inj_vec(self, *keys, default, n=None):


        if n is None:
            n = self.n_arm
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
            return np.asarray(tau_cmd, dtype=float).reshape(self.n_arm)
        delay = max(0.0, float(self._inj("motor", "drive_delay_sec", default=0.0)))
        tau_cmd = np.asarray(tau_cmd, dtype=float).reshape(self.n_arm).copy()
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
            return np.zeros(self.n_arm)
        gravity = np.asarray(self._inj("payload", "gravity_w_m_s2", default=[0.0, 0.0, -9.81]), dtype=float).reshape(3)
        force = mass * gravity
        return kin.J_arm_lwa[:3, :].T @ force

    def _gust_joint_torque(self, t):
        gust_cfg = self._inj("gust", default={}) or {}
        amp = self._inj_vec("gust", "joint_torque_amplitude_nm", default=[0.0] * self.n_arm)
        if not np.any(np.abs(amp) > 0.0):
            return np.zeros(self.n_arm)
        period = float(gust_cfg.get("period_sec", 4.0))
        width = float(gust_cfg.get("width_sec", 0.8))
        if period <= 1e-9 or width <= 1e-9:
            return np.zeros(self.n_arm)
        k = math.floor(float(t) / period)
        phase = float(t) - k * period
        if phase < 0.0 or phase >= width:
            return np.zeros(self.n_arm)
        envelope = 0.5 * (1.0 - math.cos(2.0 * math.pi * phase / width))
        rng = np.random.default_rng([int(gust_cfg.get("seed", self._inj("seed", default=20260730))), int(k), 0x746175])
        direction = rng.uniform(-1.0, 1.0, self.n_arm)
        direction /= np.maximum(np.linalg.norm(direction), 1e-12)
        return envelope * amp * direction

    def _actual_plant_torque(self, tau_cmd, t, q_full, kin):
        if not (self.internal_plant and self._inj_enabled()):
            self.last_injection["tau_actual"] = np.asarray(tau_cmd, dtype=float).reshape(self.n_arm).copy()
            self.last_injection["tau_disturbance"] = np.zeros(self.n_arm)
            return self.last_injection["tau_actual"]

        tau_delayed = self._delayed_motor_command(tau_cmd, t)
        gain = self._inj_vec("motor", "torque_gain", default=[1.0] * self.n_arm)
        bias = self._inj_vec("motor", "torque_bias_nm", default=[0.0] * self.n_arm)
        noise_std = self._inj_vec("motor", "torque_noise_std_nm", default=[0.0] * self.n_arm)
        ripple = self._inj_vec("motor", "torque_ripple_amplitude_nm", default=[0.0] * self.n_arm)
        ripple_freq = self._inj_vec("motor", "torque_ripple_frequency_hz", default=[0.0] * self.n_arm)
        ripple_phase = self._inj_vec("motor", "torque_ripple_phase_rad", default=[0.0] * self.n_arm)
        tau_motor = (
            gain * tau_delayed
            + bias
            + ripple * np.sin(2.0 * math.pi * ripple_freq * float(t) + ripple_phase)
            + self.rng.normal(0.0, noise_std, self.n_arm)
        )

        deadband = self._inj_vec("backlash", "torque_deadband_nm", default=[0.0] * self.n_arm)
        tau_transmitted = np.sign(tau_motor) * np.maximum(np.abs(tau_motor) - deadband, 0.0)

        viscous = self._inj_vec("friction", "viscous_nm_per_rad_s", default=[0.0] * self.n_arm)
        coulomb = self._inj_vec("friction", "coulomb_nm", default=[0.0] * self.n_arm)
        stribeck = max(1e-6, float(self._inj("friction", "stribeck_velocity_rad_s", default=0.03)))
        tau_friction = viscous * self.qd + coulomb * np.tanh(self.qd / stribeck)

        tau_payload = self._payload_disturbance_torque(q_full, kin)
        tau_gust = self._gust_joint_torque(t)
        tau_actual = tau_transmitted - tau_friction + tau_payload + tau_gust
        self.last_injection["tau_actual"] = tau_actual.copy()
        self.last_injection["tau_disturbance"] = (tau_actual - np.asarray(tau_cmd, dtype=float).reshape(self.n_arm)).copy()
        return tau_actual

    def _publish_joint_states(self):
        if self.joint_pub is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = list(self.arm_joint_names)
        msg.position = self.q.tolist()
        msg.velocity = self.qd.tolist()
        msg.effort = [0.0] * self.n_arm
        self.joint_pub.publish(msg)
        self.last_joint_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _publish_runtime_markers(self, desired, actual_position):
        if self.marker_pub.get_subscription_count() == 0:
            return
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
                    decimate_path(segment, MAX_PUBLISHED_PATH_POINTS),
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

    def _watchdog_trip(self, now):
        if self._watchdog_tripped:
            return True


        raw_vlim = (self.config.get("limits") or {}).get("joint_velocity")
        if raw_vlim is None:
            vlim = np.full(self.n_arm, 5.0, dtype=float)
            vlim[:min(3, self.n_arm)] = 3.0
        else:
            vlim = np.asarray(raw_vlim, dtype=float).reshape(self.n_arm)
        reasons = []


        qd_seen = np.asarray(self._qd_meas, dtype=float)
        if np.any(np.abs(qd_seen) > 1.2 * vlim):
            j = int(np.argmax(np.abs(qd_seen) / vlim))
            reasons.append(
                f"joint{j+1} 速度 {qd_seen[j]:+.2f} rad/s 超包线 "
                f"{vlim[j]:.1f} 的 1.2 倍")
        if self.last_position_error is not None and self.last_position_error > 0.05:
            reasons.append(
                f"跟踪误差 {1e3 * self.last_position_error:.1f} mm > 50 mm"
                "（对混合后的参考量，非对真实轨迹）")
        if reasons:
            self._watchdog_tripped = True
            self._watchdog_trip_time = now
            self.watchdog_reason = "；".join(reasons)
            self.get_logger().error(
                "⚠️ 安全看门狗触发，停止跟踪并保持位置：" + self.watchdog_reason
                + ("（保位 %.1f s 后收尾退出）" % self.watchdog_hold_sec
                   if self.stop_on_watchdog else ""))
        return self._watchdog_tripped

    def _publish_motor_command(self, motor):
        if getattr(self, "_watchdog_tripped", False):


            msg = MotorControl()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = self._watchdog_hold_q.tolist()
            msg.velocity = [0.0] * self.n_arm
            msg.current = [0.0] * self.n_arm
            msg.p = [30.0] * self.n_arm
            msg.d = [1.0] * self.n_arm
            msg.enable = 1
            self.command_pub.publish(msg)
            if not self.internal_plant:
                self.last_command_feedback_generation = self.feedback_generation
            return
        msg = MotorControl()
        msg.header.stamp = self.get_clock().now().to_msg()


        msg.position = np.asarray(self.q_ctrl, dtype=float).tolist()
        msg.velocity = [0.0] * self.n_arm


        msg.current = np.asarray(
            self.command_delay(motor.command), dtype=float).tolist()
        msg.p = [0.0] * self.n_arm
        msg.d = [0.0] * self.n_arm
        msg.enable = 1 if motor.enabled else 0
        self.command_pub.publish(msg)
        if not self.internal_plant:
            self.last_command_feedback_generation = self.feedback_generation

    def _resolve_track_start_nullspace_nominal(self):
        if not getattr(self, "_ns_q_nominal_is_track_start", False):
            return
        self.ns_q_nominal = np.asarray(
            self.track_start_q, dtype=float).reshape(self.n_arm).copy()
        self.get_logger().info(
            "零空间姿态目标 q_nominal := 轨迹起点位形 %s°"
            % np.round(np.degrees(self.ns_q_nominal), 2).tolist())

    def _approach_inertia_diagonal(self):
        if getattr(self, "simulated_base", False):
            bs = self._simulation_base_sample(0.0)
        else:
            bs = base_sample(0.0, self.shape, 0.0, self.derivative_dt)
        q_full = self.model.make_q_full(
            bs.position_w, bs.quaternion_wxyz, self.q)
        M = np.asarray(pin.crba(self.model.model, self.model.data, q_full))
        return np.diag(M)[-self.n_arm:].copy()

    def _build_approach_friction(self, approach, quiet=False):
        cfg = dict(approach.get("friction_feedforward", {}) or {})
        enabled = bool(cfg.get("enabled", False))

        stiction = cfg.get("stiction_nm")
        source = "配置显式给定"
        if stiction is None:
            calibration = load_arm6_calibration()
            stiction = calibration.get("stiction_nm")
            source = calibration.get("source") or "标定文件缺失"
        if stiction is None or len(stiction) < self.n_arm:
            if enabled:
                raise ValueError(
                    f"approach.friction_feedforward.enabled=true 但拿不到 {self.n_arm} "
                    f"个关节的 stiction_nm（来源: {source}）。静摩擦是实测量，"
                    "不能用默认值糊弄 —— 先跑 phase2_torque_probe.py 标定。")
            stiction = [0.0] * self.n_arm

        friction = FrictionFeedforwardConfig(
            np.asarray(stiction, dtype=float).ravel()[:self.n_arm],
            enabled=enabled,
            scale=float(cfg.get("scale", 0.8)),
            velocity_epsilon=float(cfg.get("velocity_epsilon", 0.02)),
            error_gain=float(cfg.get("error_gain", 10.0)),
        )

        if not enabled or quiet:
            return friction


        stiffness = approach_torque_stiffness(
            self._approach_inertia_diagonal(), self.approach_kp)
        before = residual_deadband(
            FrictionFeedforwardConfig(
                friction.stiction_nm, enabled=False, scale=friction.scale),
            stiffness)
        after = residual_deadband(friction, stiffness)
        self.get_logger().info(
            "approach 静摩擦前馈: 开启 scale=%.2f  f_s=%s Nm (%s)"
            % (friction.scale, np.round(friction.stiction_nm, 3).tolist(), source))
        self.get_logger().info(
            "  当前位形力矩刚度 %s Nm/rad，死区 %s° -> %s°"
            % (np.round(stiffness, 2).tolist(),
               np.round(np.degrees(before), 2).tolist(),
               np.round(np.degrees(after), 2).tolist()))
        self.get_logger().info(
            "  最差关节 j%d: %.2f° -> %.2f°"
            % (int(before.argmax()) + 1,
               np.degrees(before.max()), np.degrees(after.max())))
        if self.mode != "hardware":


            self.get_logger().warn(
                "approach 静摩擦前馈在 mode=%s 下开启 —— 该被控对象【没有摩擦"
                "模型】（internal 走 compute_arm_forward_dynamics，ROS 仿真走 "
                "torque_sim_arm.cc，两者都只加载 armature）。零误差处前馈为零，"
                "所以不会把仿真跑坏（A/B 实测 RMS 4.4 µm 无差别），但这一段仿真"
                "【验证不了这个前馈】—— 它要补偿的东西在仿真里不存在。"
                % self.mode)
        return friction

    def _write_approach_row(self, elapsed, bs, q_des, tau_friction, motor):
        q_full = self.model.make_q_full(
            bs.position_w, bs.quaternion_wxyz, self.q)
        v_full = self.model.make_v_full(bs.velocity_lwa, self.qd)
        kin = self.model.compute_kinematics(q_full, v_full)
        ref = self._reference_sample(0.0)
        error_xyz = ref.position_w - kin.ee_position
        rot_error = float(np.linalg.norm(quaternion_log_error(
            ref.quaternion_wxyz, kin.ee_quaternion_wxyz)))

        row = {
            "t": float(elapsed - self.start_delay_sec),
            "phase": 0,
            "pos_error_m": float(np.linalg.norm(error_xyz)),
            "rot_error_rad": rot_error,
            "tau_max_abs": float(np.max(np.abs(motor.command))),
            "sigma_min": float("nan"),
            "damping": float("nan"),
            "nullspace_leakage": float("nan"),
            "feasibility_scale": float("nan"),
            "nullspace_scale": float("nan"),
            "torque_scale": float("nan"),
            "enabled": int(motor.enabled),
            "pen_down": 0,
            "stroke_id": -1,
            "simulation_injection_enabled": 0,
            "gust_active": 0,
            "gust_envelope": 0.0,
            "tau_actual_max_abs": float(np.max(np.abs(self.effort_ctrl)))
            if not self.internal_plant else float("nan"),

            "approach_tau_friction_max_nm": float(
                np.max(np.abs(tau_friction))),
            "handover_blend": float("nan"),
            "tau_disturbance_max_abs": float("nan"),

            "trajectory_clock": 0.0,
        }
        for axis, value in zip("xyz", ref.position_w):
            row[f"desired_{axis}_m"] = float(value)
        for axis, value in zip("xyz", kin.ee_position):
            row[f"actual_{axis}_m"] = float(value)
        for axis, value in zip("xyz", error_xyz):
            row[f"error_{axis}_m"] = float(value)
        for axis, value in zip("xyz", bs.position_w):
            row[f"base_{axis}_m"] = float(value)
        for name, value in zip(("w", "x", "y", "z"), bs.quaternion_wxyz):
            row[f"base_q_{name}"] = float(value)
        for axis, value in zip("xyz", bs.position_w):
            row[f"base_est_{axis}_m"] = float(value)
        for name, value in zip(("w", "x", "y", "z"), bs.quaternion_wxyz):
            row[f"base_est_q_{name}"] = float(value)
        for axis in "xyz":
            row[f"base_gust_{axis}_m"] = 0.0
        for i in range(self.n_arm):

            row[f"joint_{i + 1}_rad"] = float(self.q_ctrl[i])
            row[f"joint_{i + 1}_velocity_rad_s"] = float(self.qd_ctrl[i])
            row[f"joint_{i + 1}_torque_nm"] = float(motor.command[i])
            row[f"joint_{i + 1}_tau_actual_nm"] = (
                float(self.effort_ctrl[i]) if not self.internal_plant
                else float("nan"))
            row[f"joint_{i + 1}_tau_disturbance_nm"] = float("nan")
            row[f"approach_q_des_{i + 1}_rad"] = float(q_des[i])
            row[f"approach_q_err_{i + 1}_rad"] = float(q_des[i] - self.q_ctrl[i])
        self._fill_estimator_row(row)
        self.metrics_writer.writerow(row)
        self.approach_rows_written += 1
        if self.approach_rows_written % METRICS_FLUSH_INTERVAL == 0:
            self.metrics_file.flush()

    def _plan_task_space_approach(self, kin_ctrl) -> bool:


        J_chk = kin_ctrl.J_arm_lwa
        if self.orientation_mode == "pen_axis":
            J_chk, _, _ = pen_axis_task(
                J_chk, np.zeros(J_chk.shape[0]),
                kin_ctrl.ee_rotation, self.pen_axis_local)
        sigma_min = float(np.linalg.svd(J_chk, compute_uv=False)[-1])
        sigma_crit = float(self.config["singularity"]["sigma_critical"])
        sigma_warn = float(self.config["singularity"]["sigma_warn"])
        if sigma_min < sigma_crit:
            self.get_logger().error(
                "起点位形奇异，拒绝就位（不发任何力矩）：sigma_min = %.5f < "
                "sigma_critical %.5f。当前 j5 = %.2f°。\n"
                "  先用位置模式把 joint5 挪出腕部奇异，再起 SMC：\n"
                "    python3 task_space_smc/hw/wrist_prepose.py --target-deg 40"
                % (sigma_min, sigma_crit, np.degrees(self.q_ctrl[4])
                   if self.n_arm >= 5 else float("nan")))
            self.task_approach_rejected = True
            return False
        if sigma_min < sigma_warn:
            self.get_logger().warning(
                "起点位形接近奇异：sigma_min = %.5f < sigma_warn %.5f"
                "（当前 j5 = %.2f°，建议 >= 40°）"
                % (sigma_min, sigma_warn,
                   np.degrees(self.q_ctrl[4]) if self.n_arm >= 5 else float("nan")))
        ref0 = self._reference_sample(0.0)
        self.task_approach = TaskSpaceApproach(
            kin_ctrl.ee_position, kin_ctrl.ee_rotation,
            ref0.position_w, ref0.rotation_w, self.start_delay_sec)
        plan = self.task_approach.plan()
        self.get_logger().info(
            "就位（任务空间 SMC）: 末端直线 %.1f mm、姿态 %.2f°，%.1f s 走完；"
            "峰值速度 %.4f m/s、峰值加速度 %.3f m/s²"
            % (1000.0 * plan.distance_m, plan.rotation_deg, plan.duration_sec,
               plan.peak_linear_speed, plan.peak_linear_accel))
        g = self.approach_guard
        if not g["enabled"]:
            self.get_logger().warning("就位护栏已关闭（approach.workspace_guard.enabled=false）")
            return True
        bad = []
        if plan.peak_linear_speed > g["max_linear_speed"]:
            bad.append("峰值速度 %.4f > %.4f m/s（加大 start_delay_sec）"
                       % (plan.peak_linear_speed, g["max_linear_speed"]))
        if plan.peak_linear_accel > g["max_linear_accel"]:
            bad.append("峰值加速度 %.3f > %.3f m/s²（加大 start_delay_sec）"
                       % (plan.peak_linear_accel, g["max_linear_accel"]))


        for p in self.task_approach.sample_positions(101):
            on_desk_side = p[0] < g["desk_edge_x"]
            if on_desk_side and p[2] < g["desk_plane_z"] + g["desk_clearance"]:
                bad.append("直线经过 (%.0f, %.0f, %.0f) mm：−x 侧有桌面，"
                           "离台面不足 %.0f mm"
                           % (1000*p[0], 1000*p[1], 1000*p[2],
                              1000*g["desk_clearance"]))
                break
            if (on_desk_side and p[1] > g["fence_y"] - g["fence_clearance"]
                    and p[2] < g["fence_height"]):
                bad.append("直线经过 (%.0f, %.0f, %.0f) mm：撞围栏"
                           "（y>%.0f mm 且 z<%.0f mm）"
                           % (1000*p[0], 1000*p[1], 1000*p[2],
                              1000*(g["fence_y"] - g["fence_clearance"]),
                              1000*g["fence_height"]))
                break
        if bad:
            self.get_logger().error(
                "就位轨迹被护栏拒绝，不发任何力矩（电机将由驱动的指令超时保位）：\n  "
                + "\n  ".join(bad))
            self.task_approach_rejected = True
            return False
        return True

    def _run_task_space_approach(self, elapsed, now):
        if getattr(self, "task_approach_rejected", False):
            return
        bs = self._base_for_control(now)
        if not self.uses_external_base:
            bs.velocity_lwa = np.zeros(6)
            bs.acceleration_lwa = np.zeros(6)
        self._publish_base_tf(bs)
        self._publish_base_truth(bs)
        q_full = self.model.make_q_full(
            bs.position_w, bs.quaternion_wxyz, self.q)
        v_full = self.model.make_v_full(bs.velocity_lwa, self.qd)
        q_full_ctrl = self.model.make_q_full(
            bs.position_w, bs.quaternion_wxyz, self.q_ctrl)
        v_full_ctrl = self.model.make_v_full(bs.velocity_lwa, self.qd_ctrl)
        kin_ctrl = self.model.compute_kinematics(q_full_ctrl, v_full_ctrl)
        if self.task_approach is None and not self._plan_task_space_approach(kin_ctrl):
            return
        desired = self.task_approach.sample(elapsed)
        actual = PoseState(
            position=kin_ctrl.ee_position,
            quaternion_wxyz=kin_ctrl.ee_quaternion_wxyz,
            linear_velocity=kin_ctrl.ee_velocity_lwa[:3],
            angular_velocity=kin_ctrl.ee_velocity_lwa[3:],
        )
        (tau, tau_bias, smc, rr, torque_scale,
         tau_friction) = self._task_space_torque(
            desired, actual, kin_ctrl, q_full_ctrl, v_full_ctrl, bs)


        self.last_position_error = float(np.linalg.norm(smc.position_error))
        if self._watchdog_hold_q is None:
            self._watchdog_hold_q = np.asarray(self._q_meas, dtype=float).copy()
        if not self._watchdog_trip(now):
            self._watchdog_hold_q = np.asarray(self._q_meas, dtype=float).copy()


        self._finish_approach_tick(
            elapsed, now, bs, q_full, v_full, q_full_ctrl, v_full_ctrl,
            tau, np.full(self.n_arm, np.nan), tau_friction)

    def _run_approach(self, elapsed, now):
        if self.approach_mode == "task_space":
            return self._run_task_space_approach(elapsed, now)
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

        bs = self._base_for_control(now)
        if not self.uses_external_base:
            bs.velocity_lwa = np.zeros(6)
            bs.acceleration_lwa = np.zeros(6)
        self._publish_base_tf(bs)
        self._publish_base_truth(bs)


        q_full = self.model.make_q_full(
            bs.position_w, bs.quaternion_wxyz, self.q)
        v_full = self.model.make_v_full(bs.velocity_lwa, self.qd)
        q_full_ctrl = self.model.make_q_full(
            bs.position_w, bs.quaternion_wxyz, self.q_ctrl)
        v_full_ctrl = self.model.make_v_full(bs.velocity_lwa, self.qd_ctrl)
        qdd_cmd = (
            qdd_des
            + self.approach_kp * (q_des - self.q_ctrl)
            + self.approach_kd * (qd_des - self.qd_ctrl)
        )
        tau = self.model.compute_rnea_arm_torque(
            q_full_ctrl, v_full_ctrl, bs.acceleration_lwa, qdd_cmd)


        tau_friction = stiction_feedforward(
            self.approach_friction, qd_des, q_des - self.q_ctrl)
        tau = tau + tau_friction
        self._finish_approach_tick(
            elapsed, now, bs, q_full, v_full, q_full_ctrl, v_full_ctrl,
            tau, q_des, tau_friction)
    def _finish_approach_tick(self, elapsed, now, bs, q_full, v_full,
                              q_full_ctrl, v_full_ctrl, tau, q_des,
                              tau_friction):
        motor = self.adapter.torque_to_motor_command(
            tau,
            now,
            self.last_joint_stamp or now,
            bs.stamp,
            self.control_period,
        )
        self._publish_motor_command(motor)


        self._push_estimator_command(
            q_full_ctrl, v_full_ctrl, bs.acceleration_lwa,
            motor.command, tau_friction, self.control_period)
        self._write_approach_row(elapsed, bs, q_des, tau_friction, motor)
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
            ref = self._reference_sample(0.0)
            desired = PoseState(
                position=ref.position_w,
                quaternion_wxyz=ref.quaternion_wxyz,
                linear_velocity=np.zeros(3),
                angular_velocity=np.zeros(3),
            )
            self._publish_runtime_markers(desired, kin.ee_position)
            self.last_hold_publish_time = now

    def _publish_start_hold_markers(self):
        try:
            bs = self._base_for_control(
                self.get_clock().now().nanoseconds * 1e-9)
        except RuntimeError:
            return
        self._publish_base_tf(bs)

    def _publish_start_hold(self):
        try:
            bs = self._base_for_control(
                self.get_clock().now().nanoseconds * 1e-9)
        except RuntimeError:
            return
        self._publish_base_tf(bs)
        self._publish_base_truth(bs)
        if self.internal_plant:
            self._publish_joint_states()
        q_full = self.model.make_q_full(
            bs.position_w, bs.quaternion_wxyz, self.q)
        v_full = self.model.make_v_full(bs.velocity_lwa, self.qd)
        kin = self.model.compute_kinematics(q_full, v_full)
        ref = self._reference_sample(0.0)
        desired = PoseState(
            position=ref.position_w,
            quaternion_wxyz=ref.quaternion_wxyz,
            linear_velocity=np.zeros(3),
            angular_velocity=np.zeros(3),
        )
        self._publish_runtime_markers(desired, kin.ee_position)

    def _feasibility_limits(self, dt):
        if self.feasibility_velocity is None:
            return None
        return FeasibilityLimits(
            velocity=self.feasibility_velocity,
            lower=self.q_lower,
            upper=self.q_upper,
            dt=float(dt),
            torque=(self.feasibility_torque
                    if self.feasibility_brake_safety > 0.0 else None),
            brake_safety=self.feasibility_brake_safety,
        )

    def _task_space_torque(self, desired, actual, kin_ctrl,
                           q_full_ctrl, v_full_ctrl, bs_est):
        smc = self.controller.compute(desired, actual)
        b6 = self.model.compute_task_bias_acceleration(q_full_ctrl, v_full_ctrl, bs_est.acceleration_lwa)


        J_task = kin_ctrl.J_arm_lwa
        rhs_task = smc.nu - b6
        if self.orientation_mode == "pen_axis":
            J_task, rhs_task, _ = pen_axis_task(
                J_task, rhs_task, kin_ctrl.ee_rotation, self.pen_axis_local)
        rr = resolve_acceleration(
            J_task,
            self.model.compute_arm_mass_block(q_full_ctrl),
            rhs_task,
            self.q_ctrl,
            self.qd_ctrl,
            self.ns_q_nominal,
            self.ns_kp,
            self.ns_kd,
            float(self.config["task"]["characteristic_length"]),
            float(self.config["singularity"]["sigma_warn"]),
            float(self.config["singularity"]["sigma_critical"]),
            float(self.config["singularity"]["damping_min"]),
            float(self.config["singularity"]["damping_max"]),
            self.ns_accel_weight,
            limit_avoidance=self.limit_avoidance,


            feasibility=None,
        )
        tau = self.model.compute_rnea_arm_torque(q_full_ctrl, v_full_ctrl, bs_est.acceleration_lwa, rr.qdd_cmd)

        tau_bias = self.model.compute_rnea_arm_torque(
            q_full_ctrl, v_full_ctrl, bs_est.acceleration_lwa,
            np.zeros(self.n_arm))


        torque_scale = 1.0
        if self.torque_saturation_mode == "uniform":
            torque_scale = uniform_acceleration_torque_scale(
                tau_bias, tau - tau_bias, self.feasibility_torque)
            tau = tau_bias + torque_scale * (tau - tau_bias)


        tau_track_friction = np.zeros(self.n_arm)
        if self.tracking_friction.enabled:
            J6 = kin_ctrl.J_arm_lwa
            twist_ref = np.concatenate([
                desired.linear_velocity, desired.angular_velocity])
            err_task = np.concatenate([
                smc.position_error, smc.orientation_error])
            qd_ref = np.linalg.lstsq(J6, twist_ref, rcond=None)[0]
            dq_err = np.linalg.lstsq(J6, err_task, rcond=None)[0]
            tau_track_friction = stiction_feedforward(
                self.tracking_friction, qd_ref, dq_err)
        tau = tau + tau_track_friction
        return tau, tau_bias, smc, rr, torque_scale, tau_track_friction

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
        if self.uses_external_base:
            try:
                external_base = self.external_base.latest(now)
            except RuntimeError as exc:
                if (
                    self.last_hold_publish_time is None
                    or now - self.last_hold_publish_time >= 1.0
                ):
                    self.get_logger().warning(
                        "Waiting for fresh Vicon base state: %s" % exc)
                    self.last_hold_publish_time = now
                return
            self._initialize_external_base(external_base)
        if (
            not self.internal_plant
            and self.feedback_generation
            <= self.last_command_feedback_generation
        ):
            return


        self._refresh_controller_feedback(now)

        if (self._watchdog_tripped and self.stop_on_watchdog
                and self._watchdog_trip_time is not None
                and now - self._watchdog_trip_time >= self.watchdog_hold_sec):
            self.get_logger().error(
                "看门狗触发后已保位 %.1f s，收尾退出（运行时长 %.2f s）"
                % (self.watchdog_hold_sec,
                   now - (self.start_time if self.start_time else now)))
            self.done = True
            self._write_summary()
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


            self._run_approach(elapsed, now)
            if (
                self.last_hold_publish_time is None
                or now - self.last_hold_publish_time
                >= 1.0 / self.visualization_rate_hz
            ):
                self._publish_start_hold_markers()
                self.last_hold_publish_time = now
            return
        if self.handover_q is None:


            self.handover_q = np.asarray(self.q_ctrl, dtype=float).copy()
            shortfall = self.track_start_q - self.handover_q
            self.get_logger().info(
                "交接: 欠量 %s°，最大 %.2f°（j%d）"
                % (np.round(np.degrees(shortfall), 2).tolist(),
                   np.degrees(np.abs(shortfall).max()),
                   int(np.abs(shortfall).argmax()) + 1))
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
                    list(self.arm_joint_names), now, "trajectory complete"))
            self.done = True
            self._write_summary()
            return

        if self.uses_external_base:
            bs_true = external_base
        elif self.simulated_base:
            bs_true = self._simulation_base_sample(t)
        else:
            bs_true = base_sample(t, self.shape, t, self.derivative_dt)
        bs_est = self._noisy_delayed_base_sample(t, bs_true)
        self._publish_base_tf(bs_true)
        self._publish_base_truth(bs_true)
        q_full_true = self.model.make_q_full(bs_true.position_w, bs_true.quaternion_wxyz, self.q)
        v_full_true = self.model.make_v_full(bs_true.velocity_lwa, self.qd)
        kin_true = self.model.compute_kinematics(q_full_true, v_full_true)


        q_full_ctrl = self.model.make_q_full(bs_est.position_w, bs_est.quaternion_wxyz, self.q_ctrl)
        v_full_ctrl = self.model.make_v_full(bs_est.velocity_lwa, self.qd_ctrl)
        kin_ctrl = self.model.compute_kinematics(q_full_ctrl, v_full_ctrl)


        traj_t = self._trajectory_clock(t)
        ref = self._reference_sample(traj_t)


        if not self.handover_blend.captured:


            self.handover_blend.capture(
                t, kin_ctrl.ee_position, kin_ctrl.ee_rotation,
                ref.position_w, ref.rotation_w,
                pen_axis_local=(self.pen_axis_local
                                if self.orientation_mode == "pen_axis"
                                else None),


                actual_linear_velocity=kin_ctrl.ee_velocity_lwa[:3],
                reference_linear_velocity=ref.linear_velocity_w)
            if self.handover_blend.enabled:
                self.get_logger().info(
                    "交接偏置: 位置 %.1f mm、速度 %.4f m/s、姿态 %.1f°，"
                    "%.1f s 内衰减到零（自身贡献加速度峰值 %.4f m/s²）"
                    % (1000.0 * np.linalg.norm(
                        self.handover_blend.delta_position),
                       np.linalg.norm(self.handover_blend.delta_velocity),
                       np.degrees(np.linalg.norm(
                           self.handover_blend.delta_rotvec)),
                       self.handover_blend.duration_sec,
                       self.handover_blend.peak_extra_acceleration()))
        (blend_p, blend_R, blend_v, blend_w, blend_a,
         blend_wd) = self.handover_blend.apply(
            t, ref.position_w, ref.rotation_w,
            ref.linear_velocity_w, ref.angular_velocity_w,
            ref.linear_acceleration_w, ref.angular_acceleration_w)
        desired = PoseState(
            position=blend_p,
            quaternion_wxyz=rotation_to_quat(blend_R),
            linear_velocity=blend_v,
            angular_velocity=blend_w,
            linear_acceleration=blend_a,
            angular_acceleration=blend_wd,
        )
        actual = PoseState(
            position=kin_ctrl.ee_position,
            quaternion_wxyz=kin_ctrl.ee_quaternion_wxyz,
            linear_velocity=kin_ctrl.ee_velocity_lwa[:3],
            angular_velocity=kin_ctrl.ee_velocity_lwa[3:],
        )
        (tau, tau_bias, smc, rr, torque_scale,
         tau_track_friction) = self._task_space_torque(
            desired, actual, kin_ctrl, q_full_ctrl, v_full_ctrl, bs_est)


        if self._watchdog_hold_q is None:
            self._watchdog_hold_q = np.asarray(self._q_meas, dtype=float).copy()
        if not self._watchdog_trip(now):
            self._watchdog_hold_q = np.asarray(self._q_meas, dtype=float).copy()
        motor = self.adapter.torque_to_motor_command(
            tau, now, self.last_joint_stamp or now, bs_est.stamp, dt)


        recorded_q = self.q_ctrl.copy()
        recorded_qd = self.qd_ctrl.copy()
        self._publish_motor_command(motor)


        self._push_estimator_command(
            q_full_ctrl, v_full_ctrl, bs_est.acceleration_lwa,
            motor.command, tau_track_friction, dt, tau_bias=tau_bias)

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


            tau_actual = np.asarray(self.effort_ctrl, dtype=float).copy()
            self.last_injection["tau_actual"] = tau_actual.copy()
            self.last_injection["tau_disturbance"] = (
                tau_actual - motor.command)


        pos_error = float(np.linalg.norm(ref.position_w - kin_true.ee_position))


        self.last_position_error = float(
            np.linalg.norm(blend_p - kin_true.ee_position))
        rot_error = float(np.linalg.norm(quaternion_log_error(ref.quaternion_wxyz, kin_true.ee_quaternion_wxyz)))
        self.error_samples.append(pos_error)
        self.rot_samples.append(rot_error)
        error_xyz = ref.position_w - kin_true.ee_position
        tau_disturbance = self.last_injection["tau_disturbance"]
        row = {
            "t": t,
            "phase": 1,
            "pos_error_m": pos_error,
            "rot_error_rad": rot_error,
            "tau_max_abs": float(np.max(np.abs(motor.command))),
            "sigma_min": rr.sigma_min,
            "damping": rr.damping,
            "nullspace_leakage": rr.nullspace_leakage_norm,
            "feasibility_scale": rr.feasibility_scale,
            "nullspace_scale": rr.nullspace_scale,
            "torque_scale": torque_scale,
            "enabled": int(motor.enabled),
            "pen_down": int(ref.pen_down),
            "stroke_id": int(ref.stroke_id),
            "simulation_injection_enabled": int(
                self.simulated_base and self._inj_enabled()),
            "gust_active": int(self.last_injection["gust_active"]),
            "gust_envelope": float(self.last_injection["gust_envelope"]),
            "tau_actual_max_abs": float(np.max(np.abs(tau_actual))),
            "tau_disturbance_max_abs": float(np.max(np.abs(tau_disturbance))),
            "approach_tau_friction_max_nm": float("nan"),

            "handover_blend": float(self.handover_blend.scale(t)),

            "trajectory_clock": float(traj_t),
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
        for i in range(self.n_arm):
            row[f"joint_{i + 1}_rad"] = float(recorded_q[i])
            row[f"joint_{i + 1}_velocity_rad_s"] = float(recorded_qd[i])
            row[f"joint_{i + 1}_torque_nm"] = float(motor.command[i])
            row[f"joint_{i + 1}_tau_actual_nm"] = float(tau_actual[i])
            row[f"joint_{i + 1}_tau_disturbance_nm"] = float(tau_disturbance[i])

            row[f"approach_q_des_{i + 1}_rad"] = float("nan")
            row[f"approach_q_err_{i + 1}_rad"] = float("nan")
        self._fill_estimator_row(row)
        self.metrics_writer.writerow(row)


        if len(self.error_samples) % METRICS_FLUSH_INTERVAL == 0:
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


            "approach": {
                "rows_logged": int(self.approach_rows_written),
                "start_q_rad": np.asarray(
                    self.approach_initial_q, dtype=float).tolist(),
                "track_start_q_rad": np.asarray(
                    self.track_start_q, dtype=float).tolist(),
                "commanded_travel_rad": np.abs(
                    np.asarray(self.track_start_q, dtype=float)
                    - np.asarray(self.approach_initial_q, dtype=float)).tolist(),
                "handover_q_rad": (
                    np.asarray(self.handover_q, dtype=float).tolist()
                    if self.handover_q is not None else None),
                "handover_shortfall_rad": (
                    (np.asarray(self.track_start_q, dtype=float)
                     - np.asarray(self.handover_q, dtype=float)).tolist()
                    if self.handover_q is not None else None),
                "friction_feedforward_enabled": bool(
                    self.approach_friction.enabled),
                "friction_feedforward_scale": float(
                    self.approach_friction.scale),
            },


            "handover_blend": {
                "duration_sec": float(self.handover_blend.duration_sec),

                "freeze_trajectory_during_blend": bool(
                    self.freeze_trajectory_during_blend),
                "delta_position_m": np.asarray(
                    self.handover_blend.delta_position, dtype=float).tolist(),
                "delta_position_norm_m": float(np.linalg.norm(
                    self.handover_blend.delta_position)),
                "delta_rotation_rad": float(np.linalg.norm(
                    self.handover_blend.delta_rotvec)),
                "delta_velocity_norm_m_s": float(np.linalg.norm(
                    self.handover_blend.delta_velocity)),
                "peak_extra_acceleration_m_s2": float(
                    self.handover_blend.peak_extra_acceleration()),
            },
        }
        path = self.data_dir / f"whole_task_smc_summary_{self.run_stamp}.json"
        summary["summary_json"] = str(path)
        try:
            from task_space_smc.sim.plot_whole_task_smc import generate_report
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
