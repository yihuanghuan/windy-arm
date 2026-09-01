#!/usr/bin/env python3
"""浮动基座任务空间 SMC 的整任务运行器（ROS / RViz）。

每个控制拍的链路：

    反馈 (q, q̇)  --估计器-->  (q̂, q̇̂)                 见 core/state_estimator.py
    基座位姿      --FK-->      末端位姿与雅可比 J        见 core/model.py
    (期望, 实际)  --SMC-->     任务加速度 ν = [a, α]     见 core/task_space_smc.py
    ν            --分配-->    关节加速度 q̈             两条通路，见下
    q̈            --RNEA-->   关节力矩 τ               见 core/model.py
    τ            --适配器-->  电机指令                  见 core/actuator_adapter.py

由 ``task.solver`` 选择加速度分配通路：

* ``qp``（_task_space_torque_qp）：以力矩为决策变量解带约束 QP，力矩上限与
  变化率是求解器内的硬约束，后端 clamp 只当通信兜底。见 core/task_qp.py。
* 缺省（_task_space_torque）：动力学一致伪逆 + 零空间投影，力矩在后端统一
  缩放。见 core/redundancy_resolution.py。

两条通路共有的两个换算：末端偏置加速度 b6 要从任务加速度里扣掉（y = ν − b6），
控制点是 arm_ee 标记原点而非 link6，故 J 与 b6 都要沿力臂搬点。
"""

from __future__ import annotations

import csv
import json
import math
import sys
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import NamedTuple

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
    ros_xyzw_to_wxyz,
    wxyz_to_ros_xyzw,
)
from task_space_smc.core.ee_estimator import (
    EeExtrinsic, EeFeedbackConfig, EeFeedbackEstimator)

# CSV 只能存数字，所以把状态字符串编码成整数。0 永远是"正常"。
_QP_STATUS_CODES = {"optimal": 0.0, "max_iter": 1.0,
                    "infeasible": 2.0, "numerical": 3.0}
_EE_MODE_CODES = {"fk_only": 0.0, "vicon_direct": 1.0,
                  "vicon_bias_corrected": 2.0}
# 速度来源在 CSV 里存成码而不是字符串（DictWriter 那一行全是数）。
# fk 与 fk_stale/fk_rejected 分开编码：三者都是"这一拍走了 FK"，但成因不同 ——
# 配置就是 fk / 配了 vicon 但没数据 / 配了 vicon 但对照没过。
_EE_VELOCITY_SOURCE_CODES = {"fk": 0.0, "vicon": 1.0,
                             "fk_stale": 2.0, "fk_rejected": 3.0}


class _EeSample(NamedTuple):
    """``/vicon/arm_ee`` 最新一帧。全部字段同一个 ``stamp``。"""

    position: np.ndarray
    stamp: float
    rotation: np.ndarray | None
    linear_velocity: np.ndarray | None
    angular_velocity: np.ndarray | None
    angular_accel: np.ndarray | None
from task_space_smc.core.task_qp import (
    HoldQPLimits, HoldQPWeights, saturate_task_acceleration, solve_hold_qp)
from task_space_smc.core.redundancy_resolution import (
    FeasibilityLimits, LimitAvoidance, pen_axis_task, resolve_acceleration)
from task_space_smc.core.trajectory_reference import (
    AnalyticBaseSample,
    base_sample,
    drawing_segments,
    reference_sample,
    set_hold_reference,
    traj,
    whole_period,
)


METRICS_FLUSH_INTERVAL = 1000


def _deep_update(target: dict, update: dict) -> None:
    """Recursively apply a simulation-only configuration overlay."""
    for key, value in update.items():
        if value is None:
            target.pop(key, None)
            continue
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _deep_update(target[key], value)
        else:
            target[key] = value


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
    from geometry_msgs.msg import Point, PoseStamped, TransformStamped
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
        self.declare_parameter("use_fixed_initial_joint_positions", False)
        self.declare_parameter("base_state_mode", "analytic")
        self.declare_parameter("external_base_topic", "/floating_base/vicon")
        self.declare_parameter("hold_capture_measured", True)
        self.declare_parameter("hold_target_position_m", [0.0, 0.0, 0.0])
        self.declare_parameter("simulation_handover_enabled", True)
        self.declare_parameter("simulation_profile_name", "")
        self.declare_parameter(
            "timing_diagnostics_topic", "/task_space_smc/timing_callbacks")

        self.config_path = str(self.get_parameter("config_path").value)
        with open(self.config_path, "r", encoding="utf-8") as f:
            self.config = yaml.safe_load(f)
        self.shape = str(self.get_parameter("trajectory_shape").value)
        self.mode = str(self.get_parameter("mode").value)
        if self.mode not in ("simulation", "internal_simulation", "hardware"):
            raise ValueError(
                "mode must be simulation, internal_simulation, or hardware")
        profile_name = str(self.get_parameter("simulation_profile_name").value)
        if self.mode == "simulation" and profile_name:
            profiles = self.config.pop("simulation_profiles", {}) or {}
            profile = profiles.get(profile_name)
            if not isinstance(profile, dict):
                raise RuntimeError(
                    "simulation profile %r is unavailable in %s"
                    % (profile_name, self.config_path))
            overrides = profile.get("controller_overrides", {})
            if not isinstance(overrides, dict):
                raise RuntimeError("controller_overrides must be a mapping")
            _deep_update(self.config, overrides)
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


        # hold：末端在世界系里定点保持。目标位姿要么在这里静态给定，要么在
        # 接管的第一拍由实测末端位姿捕获（缺省，见 _capture_hold_reference）。
        self.is_hold_shape = self.shape == "hold"
        _hold_target = np.asarray(
            self.get_parameter("hold_target_position_m").value,
            dtype=float).reshape(-1)
        self.hold_capture_measured = (
            self.is_hold_shape
            and bool(self.get_parameter("hold_capture_measured").value))
        if self.is_hold_shape and not self.hold_capture_measured:
            if _hold_target.size != 3 or not np.any(_hold_target):
                raise RuntimeError(
                    "hold_capture_measured=false 时必须给出非零的 "
                    "hold_target_position_m（世界系 x,y,z，单位 m）")
            set_hold_reference(_hold_target)
            self.get_logger().info(
                "保持目标（静态指定）: %s m"
                % np.round(_hold_target, 4).tolist())
        self._hold_captured = not self.hold_capture_measured
        self.derivative_dt = float(self.get_parameter("derivative_dt_sec").value)
        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.start_delay_sec = max(
            0.0, float(self.get_parameter("start_delay_sec").value))


        # 保持目标 = 当前位姿时，就位段无事可做：它要走的直线长度恒为零。
        # 更要紧的是就位段会在目标【捕获之前】读参考量，读到的是兜底缺省值，
        # 于是机械臂会先跑向一个谁也没要求的点。直接取消它。
        if self.hold_capture_measured and self.start_delay_sec > 0.0:
            self.get_logger().warning(
                "hold + 捕获实测位姿：忽略 start_delay_sec=%.2f，就位段取消"
                "（目标就是当前位姿，没有要走的距离）" % self.start_delay_sec)
            self.start_delay_sec = 0.0
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
        self.get_logger().info(
            "标定模型: source=%s zero_offset_rad=%s first_moment_ratio=%s"
            % (self.model.calibration_source,
               np.round(self.model.zero_offset_rad, 9).tolist(),
               np.round(self.model.first_moment_ratio, 9).tolist()))
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
        self._setup_ee_feedback()
        _task = self.config.get("task", {})
        self.orientation_mode = str(_task.get("orientation_mode", "full"))
        # 只有 task.solver == "qp" 才走受约束 QP。其余四份配置（whole6 /
        # whole6_hw / hwtest / whole）没有这个键，继续走 resolve_acceleration，
        # 一行不改，既有测试也不动。
        self.use_task_qp = str(_task.get("solver", "")).lower() == "qp"
        self.max_linear_task_accel = float(_task.get("max_linear_task_accel", 0.0))
        self.max_angular_task_accel = float(_task.get("max_angular_task_accel", 0.0))
        _qp = self.config.get("qp", {}) or {}
        _qw = _qp.get("weights", {}) or {}
        self.qp_weights = HoldQPWeights(
            position=float(_qw.get("position", 1.0e4)),
            orientation=float(_qw.get("orientation", 1.0)),
            joint_centering=float(_qw.get("joint_centering", 1.0e-2)),
            smoothness=float(_qw.get("smoothness", 1.0e-2)),
            regularization=float(_qw.get("regularization", 1.0e-6)),
            qdd_box_slack=float(_qw.get("qdd_box_slack", 1.0e8)),
            characteristic_length=float(_task.get("characteristic_length", 0.2)),
        )
        self.qp_qdd_max = np.asarray(
            _qp.get("qdd_max", [40.0] * self.n_arm), dtype=float).reshape(self.n_arm)
        self.qp_max_iterations = int(_qp.get("max_iterations", 40))
        self.qp_fallback_hold_ticks = int(_qp.get("fallback_hold_ticks", 5))
        self._qdd_prev = np.zeros(self.n_arm)
        self._tau_prev_sent = np.zeros(self.n_arm)
        self._tau_prev_stamp = None
        self._qp_fail_streak = 0
        self._qp_fallback_ticks = 0
        self._backend_clamp_hits = 0
        self._ee_residuals = []
        self._ee_orientation_residuals = []
        self._ee_degraded_ticks = 0
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
        self._calibration_self_check_done = False


        self.q_ctrl = self.q
        self.qd_ctrl = self.qd
        self.effort_ctrl = self.feedback_effort
        # Hardware must calculate its approach target only after the first real
        # joint-state sample arrives.  The config pose is a target, not a sensor
        # measurement, and using it here makes a successful prepose invisible.
        self._hardware_start_initialized = self.internal_plant
        self._skip_task_space_approach = False
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
        if _tf.get("positive_nm") is None and self.approach_friction.directional:
            _tf["positive_nm"] = self.approach_friction.positive_nm.tolist()
        if _tf.get("negative_nm") is None and self.approach_friction.directional:
            _tf["negative_nm"] = self.approach_friction.negative_nm.tolist()
        self.tracking_friction = self._build_approach_friction(
            {"friction_feedforward": _tf}, quiet=True)
        if self.tracking_friction.enabled:
            self.get_logger().info(
                "跟踪段摩擦前馈: 开启 scale=%.2f κ=%.4f rad (%.2f°)  "
                "F+=%s Nm F-=%s Nm"
                % (self.tracking_friction.scale,
                   self.tracking_friction.velocity_epsilon
                   / max(self.tracking_friction.error_gain, 1e-9),
                   np.degrees(self.tracking_friction.velocity_epsilon
                              / max(self.tracking_friction.error_gain, 1e-9)),
                   np.round(self.tracking_friction.positive_nm, 3).tolist(),
                   np.round(self.tracking_friction.negative_nm, 3).tolist()))

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
        if (self.mode == "simulation"
                and not bool(self.get_parameter(
                    "simulation_handover_enabled").value)):
            handover_cfg = {}
        if self.hold_capture_measured:


            # 捕获实测位姿后位置与姿态偏置【构造上】就是零，没有要拉的量。
            # 但 handover 还会把交接瞬间的末端速度偏差积分进参考量
            # （bv 峰值约 0.1·T，T=30 s 时是 3 s 的位移当量）——
            # 而保持段捕获时基座往往正在动，末端速度并不为零。
            # 留着它等于让"保持"的目标先顺着捕获瞬间的速度漂出去几十厘米。
            handover_cfg = {}
            self.get_logger().info(
                "hold + 捕获实测位姿：交接混合已关闭"
                "（位姿偏置恒为零，速度偏置不该积分进保持目标）")
        self.handover_blend = HandoverBlend(
            float(handover_cfg.get("reference_blend_sec", 0.0)))


        self.freeze_trajectory_during_blend = bool(
            handover_cfg.get("freeze_trajectory_during_blend", False))
        if self.simulated_base:
            self.track_start_q = self._track_start_pose(self.q)
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
            configured_start = np.asarray(
                self.get_parameter("initial_joint_positions").value,
                dtype=float,
            ).reshape(self.n_arm)
            use_fixed_start = bool(self.get_parameter(
                "use_fixed_initial_joint_positions").value)
            if use_fixed_start and np.any(np.abs(configured_start) > 1e-12):
                if (np.any(configured_start < self.model.arm_lower_position_limit)
                        or np.any(configured_start > self.model.arm_upper_position_limit)):
                    raise ValueError("fixed initial joint position exceeds model limits")
                self.track_start_q = configured_start.copy()
                self.get_logger().info(
                    "固定 hwtest 起点: %s°" %
                    np.round(np.degrees(self.track_start_q), 2).tolist())
            else:
                # Placeholder only.  The real target and jump are computed in
                # _control_loop after _joint_state_cb supplies measured q.
                self.track_start_q = configured_start.copy()
            self.approach_initial_q = self.q.copy()
            if self.internal_plant:
                self._hardware_start_initialized = True
                jump = float(np.max(np.abs(self.track_start_q - self.approach_initial_q)))
                if not self.hold_capture_measured:
                    self.get_logger().info(
                        f"approach: 需移动 {np.degrees(jump):.1f}° "
                        f"(峰值速度约 {1.875 * jump / max(self.start_delay_sec, 1e-3):.2f} rad/s)")
            else:
                self._hardware_start_initialized = False
                self.get_logger().info(
                    "硬件等待首帧 /joint_states 后再确定 approach 起点")
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
        self.timing_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("timing_diagnostics_topic").value),
            200,
        )
        if self.ee_estimator.config.mode != "fk_only":
            # 深队列同 QA_record 的教训：这条流与基座位姿走同一条 WiFi，
            # 会被压成串投递，浅队列会在节点内部丢掉已经送达的帧。
            #
            # 类型是 FloatingBaseState 而不是 PoseStamped：位姿/速度/加速度
            # 共用一个时间戳，下游就不必再做跨话题时间对齐。
            self.create_subscription(
                FloatingBaseState, self.ee_topic, self._ee_state_cb, 200)
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
            # 验收要能把"Vicon 说的末端"和"FK 说的末端"逐拍摆在一起看。
            + [f"ee_vicon_{axis}_m" for axis in "xyz"]
            + [f"ee_fk_{axis}_m" for axis in "xyz"]
            + [f"ee_bias_{axis}_m" for axis in "xyz"]
            # 速度也要能逐拍对照。Vicon 速度的尺度和坐标系在有真实运动的数据
            # 出现之前无法验证，这两组列就是那份数据。
            + [f"ee_vicon_vel_{axis}" for axis in "xyz"]
            + [f"ee_fk_vel_{axis}" for axis in "xyz"]
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
                # QP 路径的可观测量。qp_fallback 与 backend_clamp_hits 在正常
                # 运行中必须恒为 0；非零就说明求解器没在真正管住力矩。
                "qp_status", "qp_iterations", "qp_fallback",
                "qp_position_residual_m_s2", "qp_orientation_residual_rad_s2",
                "qp_qdd_slack_max", "task_accel_saturation",
                "backend_clamp_hits",
                # 末端反馈：Vicon 与 FK 各自的值、二者残差、数据年龄、是否降级。
                "ee_mode", "ee_residual_norm_m", "ee_orientation_residual_rad",
                "ee_vicon_age_s", "ee_degraded",
                # 这一拍的线速度/角速度实际取自谁（编码见
                # _EE_VELOCITY_SOURCE_CODES）。配了 vicon 却整段是 2/3，就说明
                # 要么链路没通、要么对照没过 —— 从数据上直接看得出来。
                "ee_vel_source", "ee_omega_source",
                "enabled",
                "pen_down", "stroke_id", "simulation_injection_enabled",
                "gust_active", "gust_envelope", "tau_actual_max_abs",
                "tau_disturbance_max_abs", "approach_tau_friction_max_nm",
                "handover_blend",


                "trajectory_clock",


                "estimator_predict_clamp_hits",
                "estimator_hold_count",
                "j5_q_ref_rad", "j5_qdot_motor_rad_s", "j5_leso_z1_rad",
                "j5_leso_z2_rad_s", "j5_leso_z3_raw_rad_s2",
                "j5_leso_z3_filtered_rad_s2", "j5_M55_kg_m2",
                "j5_tau_smc_nm", "j5_tau_dist_nm", "j5_tau_final_nm",
                "j5_s_norm", "j5_torque_saturation", "watchdog_flag",
                # Diagnostic only: qdd_actual is derived from filtered LESO z2;
                # it never enters the controller.
                "qdd_commanded_j5", "qdd_actual_est_j5",
                "acceleration_residual_j5",
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

        # Independent J5 residual-disturbance path.  It is disabled by
        # default so observer-only tests cannot alter the nominal SMC torque.
        _dc = self.config.get("disturbance_compensation") or {}
        _dc_j5 = _dc.get("joint5") or {}
        self.disturbance_compensation_enabled = bool(_dc.get("enabled", False))
        self.j5_z3_lpf_hz = max(0.0, float(_dc_j5.get("z3_lpf_hz", 5.0)))
        self.j5_disturbance_gain = float(_dc_j5.get("gain", 0.0))
        self.j5_disturbance_limit_nm = max(
            0.0, float(_dc_j5.get("torque_limit_nm", 0.2)))
        self.observer_settle_sec = max(0.0, float(
            _dc.get("observer_settle_sec", 0.0)))
        self.compensation_ramp_sec = max(0.0, float(
            _dc.get("compensation_ramp_sec", 0.0)))
        if not all(np.isfinite([
                self.j5_z3_lpf_hz, self.j5_disturbance_gain,
                self.j5_disturbance_limit_nm, self.observer_settle_sec,
                self.compensation_ramp_sec])) or self.j5_disturbance_gain < 0.0:
            raise ValueError("disturbance_compensation.joint5 参数必须有限且 gain >= 0")
        self._j5_z3_filtered = 0.0
        self._j5_z2_diag_prev = float("nan")
        self._j5_z2_diag_filtered = float("nan")
        self._j5_qdd_actual_diag = float("nan")
        self._j5_qdd_commanded_diag = float("nan")
        self._j5_acceleration_residual_diag = float("nan")
        self._j5_diag_lpf_hz = 20.0
        self._last_j5_q_ref = float("nan")
        self._j5_saturation_ticks = 0
        self._j5_disturbance_diag = {
            "z3_raw": float("nan"), "z3_filtered": float("nan"),
            "m55": float("nan"), "tau_dist": 0.0,
            "tau_smc": float("nan"), "tau_final": float("nan"),
            "q_ref": float("nan"), "s_norm": float("nan"),
            "saturated": 0.0,
        }
        self.get_logger().info(
            "J5 扰动观测/补偿: enabled=%s lpf=%.2f Hz gain=%.3f limit=%.3f Nm settle=%.1fs ramp=%.1fs"
            % (self.disturbance_compensation_enabled, self.j5_z3_lpf_hz,
               self.j5_disturbance_gain, self.j5_disturbance_limit_nm,
               self.observer_settle_sec, self.compensation_ramp_sec))


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
        if self.mode == "simulation" and profile_name:
            self.get_logger().info(
                "simulation profile applied: %s (%s)" % (profile_name, self.config_path))
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

    def _track_start_pose(self, q_seed: np.ndarray,
                          base_state: BaseState | None = None) -> np.ndarray:
        """跟踪段起点位形。

        保持目标 = 当前位姿时不解 IK：目标位姿【就是】当前位形正解出来的那个，
        当前位形本身就是解，再去搜一遍只会因为"限位余量更大"而挑到另一个解，
        凭空给出一段不该存在的位移。
        """
        if self.hold_capture_measured:
            return np.asarray(q_seed, dtype=float).reshape(self.n_arm).copy()
        return self._solve_initial_pose(q_seed, base_state)

    def _capture_hold_reference(self, kin_ctrl) -> None:
        """把控制器当前看到的末端位姿定为世界系保持目标。

        用 ``kin_ctrl``（估计器/动捕视角）而不是真值，是为了让参考量与反馈量在
        同一个视角下相减 —— 这样第一拍的任务空间误差【严格为零】，交接不产生
        任何跳变，之后所有偏差都只来自基座运动。
        """
        rotation = np.asarray(kin_ctrl.ee_rotation, dtype=float).reshape(3, 3)


        # _reference_sample 会把 reference_offset_local 加回去。这里先减掉，
        # 保证 _reference_sample(0) 正好落在实测末端上。
        position = (np.asarray(kin_ctrl.ee_position, dtype=float).reshape(3)
                    - rotation @ self.reference_offset_local)
        set_hold_reference(position, rotation)
        self._hold_captured = True
        self.track_start_q = np.asarray(self.q_ctrl, dtype=float).copy()
        self._resolve_track_start_nullspace_nominal()
        self.get_logger().info(
            "保持目标（实测捕获）: 位置 [%.4f, %.4f, %.4f] m，"
            "关节 %s°"
            % (*np.asarray(kin_ctrl.ee_position, dtype=float).reshape(3),
               np.round(np.degrees(self.track_start_q), 2).tolist()))

    def _reject_start_configuration(self, now: float, reason: str) -> None:
        """拒绝起跑：不发 SMC 力矩，改为原地位置保持，保位后收尾退出。

        复用看门狗那条通路，因为"停止跟踪并原地保持"这件事只该有一个实现。
        注意【不能】发零力矩：协议没有失能帧，零力矩等于失力，实测 joint2
        失力后下沉 14.6°。
        """
        self._watchdog_tripped = True
        self._watchdog_trip_time = now
        self.watchdog_reason = reason
        self._watchdog_hold_q = np.asarray(self._q_meas, dtype=float).copy()
        self.get_logger().error("⚠️ 起点位形被拒绝，原地保持位置：" + reason)

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
        self._publish_timing_callback(1.0, stamp)
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
        self.track_start_q = self._track_start_pose(self.q, base)
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

    def _log_calibration_self_check(self, base_state: BaseState) -> None:
        """Read-only proof that candidate calibration reaches model dynamics."""
        if self._calibration_self_check_done:
            return
        q_encoder = np.asarray(self.q_ctrl, dtype=float).copy()
        q_model = self.model.model_joint_angles(q_encoder)
        q_full = self.model.make_q_full(
            base_state.position_w, base_state.quaternion_wxyz, q_encoder)
        v_full = self.model.make_v_full(base_state.velocity_lwa, self.qd_ctrl)
        gravity_original = self.model.gravity_arm_torque(q_full)
        gravity_scaled = self.model.scale_gravity_arm_torque(gravity_original)
        tau_rnea = self.model.compute_rnea_arm_torque(
            q_full, v_full, base_state.acceleration_lwa,
            np.zeros(self.n_arm))
        f_pos = (self.tracking_friction.positive_nm
                 if self.tracking_friction.directional
                 else self.tracking_friction.stiction_nm)
        f_neg = (self.tracking_friction.negative_nm
                 if self.tracking_friction.directional
                 else self.tracking_friction.stiction_nm)
        self.get_logger().info(
            "candidate 只读自检: q_encoder=%s zero_offset=%s q_model=%s"
            % (np.round(q_encoder, 6).tolist(),
               np.round(self.model.zero_offset_rad, 6).tolist(),
               np.round(q_model, 6).tolist()))
        self.get_logger().info(
            "candidate 只读自检: gravity_original=%s gravity_scaled=%s "
            "F+=%s Nm F-=%s Nm final_rnea=%s Nm"
            % (np.round(gravity_original, 6).tolist(),
               np.round(gravity_scaled, 6).tolist(),
               np.round(f_pos, 6).tolist(), np.round(f_neg, 6).tolist(),
               np.round(tau_rnea, 6).tolist()))
        self._calibration_self_check_done = True

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
        # This is the model-implied acceleration from the command that was
        # actually sent after adapter conversion/saturation.  It is logged for
        # LESO sign diagnostics and is not fed back into the controller here.
        self._j5_qdd_commanded_diag = float(qdd_eff[4])
        self.estimator.push_command(qdd_eff, dt)

    def _update_j5_acceleration_diagnostic(self, dt):
        """Estimate J5 acceleration from filtered LESO z2 for diagnostics only."""
        z2 = float(self.estimator.leso.velocity_estimate[4])
        if not np.isfinite(z2) or not np.isfinite(dt) or dt <= 0.0:
            self._j5_qdd_actual_diag = float("nan")
            self._j5_acceleration_residual_diag = float("nan")
            return
        alpha = 1.0 - math.exp(-2.0 * math.pi * self._j5_diag_lpf_hz * dt)
        if not np.isfinite(self._j5_z2_diag_filtered):
            self._j5_z2_diag_filtered = z2
        self._j5_z2_diag_filtered += alpha * (z2 - self._j5_z2_diag_filtered)
        if np.isfinite(self._j5_z2_diag_prev):
            self._j5_qdd_actual_diag = (
                (self._j5_z2_diag_filtered - self._j5_z2_diag_prev) / dt)
        else:
            self._j5_qdd_actual_diag = float("nan")
        self._j5_z2_diag_prev = self._j5_z2_diag_filtered
        self._j5_acceleration_residual_diag = (
            self._j5_qdd_actual_diag - self._j5_qdd_commanded_diag
            if np.isfinite(self._j5_qdd_actual_diag)
            and np.isfinite(self._j5_qdd_commanded_diag) else float("nan"))

    def _j5_disturbance_torque(self, q_full_ctrl, dt, t, tau_smc, smc):
        """Compute the optional, clipped J5 residual-disturbance torque."""
        z3 = float(self.estimator.leso.disturbance_estimate[4])
        valid_dt = np.isfinite(dt) and 0.0 < dt <= 0.05
        if valid_dt and np.isfinite(z3):
            if self.j5_z3_lpf_hz > 0.0:
                alpha = 1.0 - math.exp(-2.0 * math.pi * self.j5_z3_lpf_hz * dt)
                self._j5_z3_filtered += alpha * (z3 - self._j5_z3_filtered)
            else:
                self._j5_z3_filtered = z3
        else:
            self._j5_z3_filtered = 0.0

        try:
            m55 = float(self.model.compute_arm_mass_block(q_full_ctrl)[4, 4])
        except (ValueError, np.linalg.LinAlgError, IndexError):
            m55 = float("nan")
        tau_dist = 0.0
        clipped = False
        ramp = 0.0
        if self.compensation_ramp_sec <= 0.0:
            ramp = float(t >= self.observer_settle_sec)
        else:
            ramp = float(np.clip(
                (t - self.observer_settle_sec) / self.compensation_ramp_sec,
                0.0, 1.0))
        effective_gain = self.j5_disturbance_gain * ramp
        if (self.disturbance_compensation_enabled and valid_dt
                and np.isfinite(z3) and np.isfinite(self._j5_z3_filtered)
                and np.isfinite(m55) and m55 > 0.0
                and np.isfinite(effective_gain)):
            raw = -effective_gain * m55 * self._j5_z3_filtered
            if np.isfinite(raw):
                tau_dist = float(np.clip(
                    raw, -self.j5_disturbance_limit_nm,
                    self.j5_disturbance_limit_nm))
                clipped = abs(raw) > self.j5_disturbance_limit_nm

        self._j5_disturbance_diag.update({
            "z3_raw": z3, "z3_filtered": self._j5_z3_filtered,
            "m55": m55, "tau_dist": tau_dist,
            "tau_smc": float(tau_smc[4]),
            "tau_final": float(tau_smc[4] + tau_dist),
            "q_ref": float(self._last_j5_q_ref),
            "s_norm": float(np.linalg.norm(np.concatenate([
                smc.position_sliding, smc.orientation_sliding]))),
            "saturated": float(clipped),
        })
        return tau_dist

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
        if self.is_hold_shape:


            # 保持目标是一个点，没有静态路径可画；它由运行期的绿球标出。
            return []
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
        positive = cfg.get("positive_nm")
        negative = cfg.get("negative_nm")
        source = "配置显式给定"
        if stiction is None:
            calibration = load_arm6_calibration()
            stiction = calibration.get("stiction_nm")
            cal_pos = calibration.get("kinetic_friction_pos_nm")
            cal_neg = calibration.get("kinetic_friction_neg_nm")
            if cal_pos is not None and cal_neg is not None:
                base = stiction or [0.0] * self.n_arm
                positive = []
                negative = []
                for i in range(self.n_arm):
                    p = cal_pos[i] if i < len(cal_pos) else None
                    n = cal_neg[i] if i < len(cal_neg) else None
                    b = base[i] if i < len(base) and base[i] is not None else 0.0
                    if p is None or n is None or float(p) < 0.0 or float(n) < 0.0:
                        positive.append(float(b))
                        negative.append(float(b))
                    else:
                        positive.append(float(p))
                        negative.append(float(n))
                stiction = np.maximum(positive, negative).tolist()
            source = calibration.get("source") or "标定文件缺失"
        if stiction is None or len(stiction) < self.n_arm:
            if enabled:
                raise ValueError(
                    f"approach.friction_feedforward.enabled=true 但拿不到 {self.n_arm} "
                    f"个关节的 stiction_nm（来源: {source}）。静摩擦是实测量，"
                    "不能用默认值糊弄 —— 先跑 phase2_torque_probe.py 标定。")
            stiction = [0.0] * self.n_arm
        stiction = np.asarray([
            0.0 if v is None else float(v)
            for v in list(stiction)[:self.n_arm]
        ], dtype=float)
        if positive is not None and negative is not None:
            positive = [
                stiction[i] if positive[i] is None else float(positive[i])
                for i in range(self.n_arm)
            ]
            negative = [
                stiction[i] if negative[i] is None else float(negative[i])
                for i in range(self.n_arm)
            ]
        else:
            positive = None
            negative = None

        friction = FrictionFeedforwardConfig(
            stiction,
            positive_nm=positive,
            negative_nm=negative,
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
            "approach 摩擦前馈: 开启 scale=%.2f  F+=%s Nm F-=%s Nm (%s)"
            % (friction.scale,
               np.round(friction.positive_nm, 3).tolist(),
               np.round(friction.negative_nm, 3).tolist(), source))
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
            # 就位段不跑 QP、也不用 Vicon 末端；这些列必须显式补 NaN，
            # 否则 csv.DictWriter 直接抛 ValueError。
            "qp_status": float("nan"),
            "qp_iterations": float("nan"),
            "qp_fallback": float("nan"),
            "qp_position_residual_m_s2": float("nan"),
            "qp_orientation_residual_rad_s2": float("nan"),
            "qp_qdd_slack_max": float("nan"),
            "task_accel_saturation": float("nan"),
            "backend_clamp_hits": float(self._backend_clamp_hits),
            "ee_mode": float("nan"),
            "ee_residual_norm_m": float("nan"),
            "ee_orientation_residual_rad": float("nan"),
            "ee_vicon_age_s": float("nan"),
            "ee_degraded": float("nan"),
            "ee_vel_source": float("nan"),
            "ee_omega_source": float("nan"),
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
            "qdd_commanded_j5": float("nan"),
            "qdd_actual_est_j5": float("nan"),
            "acceleration_residual_j5": float("nan"),
        }
        for axis, value in zip("xyz", ref.position_w):
            row[f"desired_{axis}_m"] = float(value)
        for axis, value in zip("xyz", kin.ee_position):
            row[f"actual_{axis}_m"] = float(value)
        for axis, value in zip("xyz", error_xyz):
            row[f"error_{axis}_m"] = float(value)
        for axis in "xyz":
            row[f"ee_vicon_{axis}_m"] = float("nan")
            row[f"ee_fk_{axis}_m"] = float("nan")
            row[f"ee_bias_{axis}_m"] = float("nan")
            row[f"ee_vicon_vel_{axis}"] = float("nan")
            row[f"ee_fk_vel_{axis}"] = float("nan")
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

    def _start_singularity_reason(self, kin_ctrl) -> str:
        """起点位形奇异门。返回拒绝理由；空串表示放行（可能带 warn 日志）。

        任务空间 SMC 正是被腕部奇异卡住的那一方：sigma_min 趋零时伪逆照样给
        得出一个数，但那个数与期望的末端运动无关，机械臂会往任意方向窜。
        """
        J_chk = kin_ctrl.J_arm_lwa
        if self.orientation_mode == "pen_axis":
            J_chk, _, _ = pen_axis_task(
                J_chk, np.zeros(J_chk.shape[0]),
                kin_ctrl.ee_rotation, self.pen_axis_local)
        sigma_min = float(np.linalg.svd(J_chk, compute_uv=False)[-1])
        sigma_crit = float(self.config["singularity"]["sigma_critical"])
        sigma_warn = float(self.config["singularity"]["sigma_warn"])
        j5_deg = (np.degrees(self.q_ctrl[4]) if self.n_arm >= 5
                  else float("nan"))
        if sigma_min < sigma_crit:
            return (
                "sigma_min = %.5f < sigma_critical %.5f。当前 j5 = %.2f°。\n"
                "  先用位置模式把 joint5 挪出腕部奇异，再起 SMC：\n"
                "    python3 task_space_smc/hw/wrist_prepose.py --target-deg 40"
                % (sigma_min, sigma_crit, j5_deg))
        if sigma_min < sigma_warn:
            self.get_logger().warning(
                "起点位形接近奇异：sigma_min = %.5f < sigma_warn %.5f"
                "（当前 j5 = %.2f°，建议 >= 40°）"
                % (sigma_min, sigma_warn, j5_deg))
        return ""

    def _plan_task_space_approach(self, kin_ctrl) -> bool:
        reason = self._start_singularity_reason(kin_ctrl)
        if reason:
            self.get_logger().error(
                "起点位形奇异，拒绝就位（不发任何力矩）：" + reason)
            self.task_approach_rejected = True
            return False
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
        if self.approach_mode == "task_space" and not self._skip_task_space_approach:
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
            getattr(bs, "stamp", now),
            self.control_period,
        )
        self._publish_motor_command(motor)


        self._push_estimator_command(
            q_full_ctrl, v_full_ctrl, bs.acceleration_lwa,
            motor.command, tau_friction, self.control_period)
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

    @staticmethod
    def _load_ee_extrinsic(config_path):
        """``arm_ee_from_link6`` from ``vicon_extrinsics.yaml``, or ``None``.

        Rotation is deliberately *not* read into control.  arm_ee sits 114.39
        deg (run 1) / 113.54 deg (run 2) from link6, and the residual after
        removing that constant is 9.97 deg in one run and 43.55 deg in the
        other, so no single fixed rotation explains the channel.  The key is
        parsed only so the logged orientation residual can use it.
        """
        base = Path(config_path).resolve().parent
        path = base / "vicon_extrinsics.yaml"
        if not path.exists():
            return None
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        block = data.get("arm_ee_from_link6") or {}
        translation = block.get("translation_m")
        if not isinstance(translation, list) or len(translation) != 3:
            return None
        return EeExtrinsic(np.asarray(translation, dtype=float))

    def _setup_ee_feedback(self):
        cfg = self.config.get("ee_feedback", {}) or {}
        mode = str(cfg.get("mode", "fk_only"))
        extrinsic = self._load_ee_extrinsic(self.config_path)
        if mode != "fk_only" and extrinsic is None:
            # 与 launch 的 load_base_extrinsic 同一条原则：外参缺失是硬错误，
            # 绝不静默退化成单位阵。
            raise RuntimeError(
                "ee_feedback.mode=%s 需要 vicon_extrinsics.yaml 的 "
                "arm_ee_from_link6.translation_m，但没读到" % mode)
        self.ee_estimator = EeFeedbackEstimator(
            EeFeedbackConfig(
                mode=mode,
                apply_control_point_offset=bool(
                    cfg.get("apply_control_point_offset", True)),
                velocity_source=str(cfg.get("velocity_source", "fk")),
                angular_velocity_source=str(
                    cfg.get("angular_velocity_source", "fk")),
                velocity_cross_check_tol=float(
                    cfg.get("velocity_cross_check_tol", 0.5)),
                velocity_cross_check_floor_m_s=float(
                    cfg.get("velocity_cross_check_floor_m_s", 0.05)),
                angular_cross_check_tol=float(
                    cfg.get("angular_cross_check_tol", 0.5)),
                angular_cross_check_floor_rad_s=float(
                    cfg.get("angular_cross_check_floor_rad_s", 0.2)),
                bias_bandwidth_hz=float(cfg.get("bias_bandwidth_hz", 0.2)),
                bias_init=str(cfg.get("bias_init", "first_sample")),
                max_bias_m=float(cfg.get("max_bias_m", 0.02)),
                max_bias_rate_m_s=float(cfg.get("max_bias_rate_m_s", 0.01)),
                innovation_reject_m=float(cfg.get("innovation_reject_m", 0.05)),
                innovation_reject_hold_sec=float(
                    cfg.get("innovation_reject_hold_sec", 0.5)),
                max_age_sec=float(cfg.get("max_age_sec", 0.15)),
                hold_after_sec=float(cfg.get("hold_after_sec", 1.0)),
                require_vicon_at_capture=bool(
                    cfg.get("require_vicon_at_capture", True)),
            ),
            extrinsic)
        self.ee_topic = str(cfg.get("topic", "/vicon/arm_ee"))
        self._ee_sample = None          # _EeSample
        self._ee_estimate = None

    def _ee_state_cb(self, msg):
        """``/vicon/arm_ee``（FloatingBaseState）：一条消息一个时间戳带三样。

        速度/加速度绝不单独取时间戳 —— 它们与位姿同帧，这正是选这个消息类型
        而不是拆三个话题的理由。
        """
        position = np.array([msg.pose.position.x, msg.pose.position.y,
                             msg.pose.position.z])
        if not np.all(np.isfinite(position)):
            return
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self._publish_timing_callback(2.0, stamp)
        quaternion = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                               msg.pose.orientation.z, msg.pose.orientation.w])
        norm = float(np.linalg.norm(quaternion))
        rotation = None
        if np.isfinite(norm) and norm > 1e-6:
            rotation = quat_to_rotation(ros_xyzw_to_wxyz(quaternion / norm))
        if self._ee_sample is not None and stamp < self._ee_sample.stamp:
            return                      # 与 RosFeedbackBaseState 同规则：丢乱序样本

        def _vec3(v):
            out = np.array([v.x, v.y, v.z], dtype=float)
            # 非有限值整条置 None 而不是补零：0 是估计器的对照闸识别不出来的
            # 合法值，None 会走"没数据"分支并被计数。
            return out if np.all(np.isfinite(out)) else None

        self._ee_sample = _EeSample(
            position=position,
            stamp=stamp,
            rotation=rotation,
            linear_velocity=_vec3(msg.twist.linear),
            angular_velocity=_vec3(msg.twist.angular),
            angular_accel=_vec3(msg.acceleration.angular),
        )

    def _publish_timing_callback(self, stream_code: float,
                                 source_stamp: float) -> None:
        """Read-only callback timing probe; it does not enter control math."""
        msg = Float64MultiArray()
        arrival = self.get_clock().now().nanoseconds * 1e-9
        msg.data = [float(stream_code), float(source_stamp), float(arrival)]
        self.timing_pub.publish(msg)

    def _update_ee_feedback(self, now, kin_ctrl, dt):
        """把 Vicon arm_ee 与编码器 FK 融合成本拍使用的末端状态。

        ``fk_only`` 模式下返回 ``None``，调用点原样使用 kin_ctrl —— 于是这条
        改动对其余四份配置完全透明。
        """
        if self.ee_estimator.config.mode == "fk_only":
            return None
        sample = self._ee_sample
        estimate = self.ee_estimator.update(
            now,
            kin_ctrl.ee_position, kin_ctrl.ee_rotation,
            kin_ctrl.ee_velocity_lwa[:3], kin_ctrl.ee_velocity_lwa[3:],
            dt,
            vicon_position=(sample.position if sample else None),
            vicon_stamp=(sample.stamp if sample else None),
            vicon_rotation=(sample.rotation if sample else None),
            vicon_linear_velocity=(sample.linear_velocity if sample else None),
            vicon_angular_velocity=(sample.angular_velocity if sample else None),
            vicon_angular_accel=(sample.angular_accel if sample else None),
        )
        self._ee_estimate = estimate
        if estimate.vicon_position is not None:
            self._ee_residuals.append(
                float(np.linalg.norm(estimate.position_residual)))
        if np.isfinite(estimate.orientation_residual_rad):
            self._ee_orientation_residuals.append(
                float(estimate.orientation_residual_rad))
        if estimate.degraded:
            self._ee_degraded_ticks += 1
        return estimate

    def _hold_qp_limits(self, dt):
        return HoldQPLimits(
            torque=self.feasibility_torque,
            torque_rate=np.asarray(
                self.config["limits"]["torque_rate"], dtype=float).reshape(self.n_arm),
            qdd_max=self.qp_qdd_max,
            dt=float(dt),
            q_lower=self.q_lower,
            q_upper=self.q_upper,
            velocity=self.feasibility_velocity,
            brake_safety=self.feasibility_brake_safety,
        )

    def _torque_slew_reference(self, now):
        """上一拍【实际发出】的力矩，作为力矩变化率箱的中心。

        必须镜像 adapter 的归零规则（actuator_adapter.py:103-104）：停拍超过
        command_timeout_sec 后 adapter 把 _last_tau 归零。两边基准一旦错开，
        adapter 就会削掉一条 QP 认为合法的指令 —— 后端裁剪又回到控制策略里。
        """
        if (self._tau_prev_stamp is None
                or now - self._tau_prev_stamp > self.adapter.command_timeout_sec):
            return np.zeros(self.n_arm)
        return self._tau_prev_sent

    def _task_space_torque_qp(self, desired, actual, kin_ctrl,
                              q_full_ctrl, v_full_ctrl, bs_est, dt, now):
        """受约束 QP 版本：力矩硬约束在求解器内，后端 clamp 只当通信兜底。

        与 ``_task_space_torque`` 并存，由 ``task.solver: qp`` 选择；其余四份
        配置仍走原来的伪逆 + 后端 clip 路径，一行不改。
        """
        smc = self.controller.compute(desired, actual)
        b6 = self.model.compute_task_bias_acceleration(
            q_full_ctrl, v_full_ctrl, bs_est.acceleration_lwa)

        # 控制点是 arm_ee 标记原点，不是 link6：Jacobian 与偏置加速度都要跟着搬。
        J_task = self.ee_estimator.jacobian(kin_ctrl.J_arm_lwa, kin_ctrl.ee_rotation)
        b6 = self.ee_estimator.bias_acceleration(
            b6, kin_ctrl.ee_velocity_lwa[3:], kin_ctrl.ee_rotation)

        nu, task_scale = saturate_task_acceleration(
            smc.nu, self.max_linear_task_accel, self.max_angular_task_accel)
        y_task = nu - b6

        M_aa = self.model.compute_arm_mass_block(q_full_ctrl)
        tau_bias = self.model.compute_rnea_arm_torque(
            q_full_ctrl, v_full_ctrl, bs_est.acceleration_lwa, np.zeros(self.n_arm))

        # 摩擦前馈必须并进仿射常数项 c，而不是等力矩算完再加：加在外面会顶破
        # QP 保证的上限，后端再削一刀 —— 那正是要消灭的抖振源。
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
        c_affine = tau_bias + tau_track_friction

        result = solve_hold_qp(
            J_task, M_aa, c_affine, y_task,
            self.q_ctrl, self.qd_ctrl, self._qdd_prev,
            self._torque_slew_reference(now),
            self.ns_q_nominal, self.ns_kp, self.ns_kd,
            self.qp_weights, self._hold_qp_limits(dt),
            limit_avoidance=self.limit_avoidance,
            max_iterations=self.qp_max_iterations,
            sigma_warn=float(self.config["singularity"]["sigma_warn"]),
            sigma_critical=float(self.config["singularity"]["sigma_critical"]),
            damping_min=float(self.config["singularity"]["damping_min"]),
            damping_max=float(self.config["singularity"]["damping_max"]),
        )
        result.task_accel_scale = task_scale
        return result, smc, tau_bias, tau_track_friction

    def _task_space_torque(self, desired, actual, kin_ctrl,
                           q_full_ctrl, v_full_ctrl, bs_est):
        smc = self.controller.compute(desired, actual)
        qd_ref = np.linalg.lstsq(
            kin_ctrl.J_arm_lwa,
            np.concatenate([desired.linear_velocity, desired.angular_velocity]),
            rcond=None)[0]
        self._last_j5_q_ref = float(self.q_ctrl[4] + qd_ref[4] * self.control_period)
        b6 = self.model.compute_task_bias_acceleration(q_full_ctrl, v_full_ctrl, bs_est.acceleration_lwa)

        # 任务右端 y = ν − b6：末端加速度 ẍ = J·q̈ + b6，要求 ẍ = ν 即得该式。
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
        if not self.internal_plant and not self._hardware_start_initialized:
            configured_start = np.asarray(
                self.get_parameter("initial_joint_positions").value,
                dtype=float,
            ).reshape(self.n_arm)
            use_fixed_start = bool(self.get_parameter(
                "use_fixed_initial_joint_positions").value)
            self.approach_initial_q = np.asarray(self.q_ctrl, dtype=float).copy()
            if use_fixed_start and np.any(np.abs(configured_start) > 1e-12):
                self.track_start_q = configured_start.copy()
            else:
                self.track_start_q = self._track_start_pose(self.approach_initial_q)
            self._resolve_track_start_nullspace_nominal()
            self._hardware_start_initialized = True
            jump = float(np.max(np.abs(self.track_start_q - self.approach_initial_q)))
            # A successful prepose already placed the arm at the configured
            # start.  Do not re-solve a task-space orientation path here: with
            # a small joint error that path can choose a large J6 motion.
            self._skip_task_space_approach = jump <= np.deg2rad(5.0)
            if not self.hold_capture_measured:
                self.get_logger().info(
                    "硬件首帧起点: 当前实测 q=%s°，目标=%s°，需移动 %.1f°"
                    % (np.round(np.degrees(self.approach_initial_q), 2).tolist(),
                       np.round(np.degrees(self.track_start_q), 2).tolist(),
                       np.degrees(jump)))
                if self._skip_task_space_approach:
                    self.get_logger().info(
                        "硬件已完成 prepose：小误差交接改用关节空间，不重新规划任务空间姿态路径")
        if self.uses_external_base:
            self._log_calibration_self_check(external_base)

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
        self._update_j5_acceleration_diagnostic(dt)
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

        # true 用真值基座与原始关节反馈，仅供记录与画图；ctrl 用估计出来的基座与
        # 经估计器处理过的关节量，是控制律实际看到的那一份，两者不可混用。
        q_full_ctrl = self.model.make_q_full(bs_est.position_w, bs_est.quaternion_wxyz, self.q_ctrl)
        v_full_ctrl = self.model.make_v_full(bs_est.velocity_lwa, self.qd_ctrl)
        kin_ctrl = self.model.compute_kinematics(q_full_ctrl, v_full_ctrl)


        # hold 从第一帧接管，没有就位段，所以起点奇异门在这里查。
        # 保持段的起点位形是【操作员摆出来的】，不是配置里挑好的，更不能不查。
        if not self._hold_captured:
            self._update_ee_feedback(now, kin_ctrl, dt)
            reason = ("" if self._watchdog_tripped
                      else self._start_singularity_reason(kin_ctrl))
            if reason or self._watchdog_tripped:
                if reason:
                    self._reject_start_configuration(
                        now, "起点位形奇异：" + reason)
                self._publish_motor_command(None)
                return
            if not self.ee_estimator.ready_for_capture():
                # 还没收到过有效 arm_ee 就捕获，等于静默退化成 fk_only 并把
                # 保持目标钉在一个错的地方。宁可等，也不要悄悄换数据源。
                #
                # 这【不是】故障，所以不触发看门狗、也不发指令 —— 与顶部
                # "还没有 /joint_states" 那道门同一种处理：本拍直接跳过，
                # 由驱动自己的指令超时保位。调用 _publish_motor_command(None)
                # 在这里是错的：它只在看门狗已latch时才有 None 分支。
                self.get_logger().warning(
                    "等待 %s 的第一帧有效数据后再捕获保持目标" % self.ee_topic,
                    throttle_duration_sec=1.0)
                return
            self._capture_hold_reference(kin_ctrl)

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
        # 就位和 handover 只负责安全接管，不作为正式轨迹数据或 RViz
        # 运行轨迹的一部分。正式记录从 handover 结束、trajectory_clock=0
        # 的第一拍开始，避免把交接直线误认为圆轨迹跟踪误差。
        tracking_started = (
            not self.handover_blend.enabled
            or t >= self.handover_blend.duration_sec
        )
        desired = PoseState(
            position=blend_p,
            quaternion_wxyz=rotation_to_quat(blend_R),
            linear_velocity=blend_v,
            angular_velocity=blend_w,
            linear_acceleration=blend_a,
            angular_acceleration=blend_wd,
        )
        ee = self._update_ee_feedback(now, kin_ctrl, dt)
        if ee is not None and ee.hold_required:
            self._reject_start_configuration(
                now, "末端反馈降级：" + ee.degrade_reason)
            self._publish_motor_command(None)
            return
        actual = PoseState(
            position=(ee.position if ee is not None else kin_ctrl.ee_position),
            # 姿态永远来自 FK。ee.rotation 也等于 FK，这里直接用 kin_ctrl 的
            # 四元数是为了不多做一次矩阵->四元数转换。
            quaternion_wxyz=kin_ctrl.ee_quaternion_wxyz,
            # 两条速度由估计器选源（fk / vicon，各自可配、各自有对照闸）。
            # fk_only 模式下 ee 是 None，仍旧原样用 kin_ctrl。
            linear_velocity=(ee.linear_velocity if ee is not None
                             else kin_ctrl.ee_velocity_lwa[:3]),
            angular_velocity=(ee.angular_velocity if ee is not None
                              else kin_ctrl.ee_velocity_lwa[3:]),
        )
        qp_result = None
        if self.use_task_qp:
            qp_result, smc, tau_bias, tau_track_friction = \
                self._task_space_torque_qp(
                    desired, actual, kin_ctrl, q_full_ctrl, v_full_ctrl,
                    bs_est, dt, now)
            if qp_result.ok:
                self._qp_fail_streak = 0
                tau, rr, torque_scale = qp_result.tau, None, 1.0
                self._qdd_prev = qp_result.qdd.copy()
            else:
                # Tier 1：回落到既有且已测的伪逆路径，这次把 feasibility 真正传
                # 进去，再夹回 QP 用的同一个力矩箱 —— 不同算法，所以 QP 的 bug
                # 不会同时打掉两条路；夹到以 tau_prev ± rate*dt 为心的箱子里，
                # 输出与上一拍连续。
                self._qp_fail_streak += 1
                self._qp_fallback_ticks += 1
                self.get_logger().warning(
                    "QP 未收敛（%s），本拍回落到伪逆路径" % qp_result.status)
                (tau, tau_bias, smc, rr, torque_scale,
                 tau_track_friction) = self._task_space_torque(
                    desired, actual, kin_ctrl, q_full_ctrl, v_full_ctrl, bs_est)
                lo, hi = qp_result.tau_box
                tau = np.clip(tau, lo, hi)
                if self._qp_fail_streak >= self.qp_fallback_hold_ticks:
                    # Tier 2：连续失败转位置保持。【绝不】发零力矩 ——
                    # 实测 joint2 失力后下沉 14.6°。
                    self._reject_start_configuration(
                        now, "QP 连续 %d 拍未收敛" % self._qp_fail_streak)
                    self._publish_motor_command(None)
                    return
        else:
            (tau, tau_bias, smc, rr, torque_scale,
             tau_track_friction) = self._task_space_torque(
                desired, actual, kin_ctrl, q_full_ctrl, v_full_ctrl, bs_est)

        # Apply only after the existing SMC/QP calculation and before the
        # adapter's existing whole-arm torque saturation.
        tau_smc_before_disturbance = np.asarray(tau, dtype=float).copy()
        tau_dist_j5 = self._j5_disturbance_torque(
            q_full_ctrl, dt, t, tau_smc_before_disturbance, smc)
        tau[4] += tau_dist_j5


        if self._watchdog_hold_q is None:
            self._watchdog_hold_q = np.asarray(self._q_meas, dtype=float).copy()
        if not self._watchdog_trip(now):
            self._watchdog_hold_q = np.asarray(self._q_meas, dtype=float).copy()
        motor = self.adapter.torque_to_motor_command(
            tau, now, self.last_joint_stamp or now,
            getattr(bs_est, "stamp", now), dt)
        if motor.enabled:
            motor_cmd = np.asarray(motor.command, dtype=float)
            limit_hit = (np.any(np.abs(motor_cmd)
                                >= self.feasibility_torque - 1e-6))
            self._j5_disturbance_diag["tau_final"] = float(motor_cmd[4])
            self._j5_disturbance_diag["saturated"] = float(
                self._j5_disturbance_diag["saturated"] or limit_hit)
            if self._j5_disturbance_diag["saturated"]:
                self._j5_saturation_ticks += 1

        # 后端 clamp 只是【通信兜底】，正常运行必须恒为 0。真削到了就说明
        # 求解器的力矩箱与 adapter 的不一致（多半是 tau_prev 基准错开），
        # 那正是后端裁剪重新变成控制策略的第一征兆。
        if motor.enabled and self.use_task_qp:
            clamped = float(np.max(np.abs(np.asarray(motor.command) - tau)))
            if clamped > 1e-6:
                self._backend_clamp_hits += 1
                self.get_logger().warning(
                    "后端 clamp 被触发（最大 %.4f N*m）：QP 与 adapter 的力矩箱"
                    "不一致，正常运行不该出现" % clamped,
                    throttle_duration_sec=2.0)
        if motor.enabled:
            self._tau_prev_sent = np.asarray(motor.command, dtype=float).copy()
            self._tau_prev_stamp = now


        recorded_q = self.q_ctrl.copy()
        recorded_qd = self.qd_ctrl.copy()
        self._publish_motor_command(motor)


        self._push_estimator_command(
            q_full_ctrl, v_full_ctrl, bs_est.acceleration_lwa,
            motor.command, np.asarray(tau_track_friction, dtype=float),
            dt, tau_bias=tau_bias)

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
        if not tracking_started:
            return
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
            # QP 路径没有伪逆那套量（阻尼逆、零空间投影泄漏、缩放系数），
            # 用 NaN 明确表示"这一路不产生这个量"，而不是填 0 冒充测量值。
            "sigma_min": (rr.sigma_min if rr is not None
                          else (qp_result.sigma_min if qp_result else float("nan"))),
            "damping": (rr.damping if rr is not None
                        else (qp_result.damping if qp_result else float("nan"))),
            "nullspace_leakage": (rr.nullspace_leakage_norm if rr is not None
                                  else float("nan")),
            "feasibility_scale": (rr.feasibility_scale if rr is not None
                                  else float("nan")),
            "nullspace_scale": (rr.nullspace_scale if rr is not None
                                else float("nan")),
            "qp_status": _QP_STATUS_CODES.get(
                qp_result.status if qp_result else "", float("nan")),
            "qp_iterations": (qp_result.iterations if qp_result else float("nan")),
            "qp_fallback": (0.0 if (qp_result is None or qp_result.ok) else 1.0),
            "qp_position_residual_m_s2": (
                float(np.linalg.norm(qp_result.position_residual))
                if qp_result else float("nan")),
            "qp_orientation_residual_rad_s2": (
                float(np.linalg.norm(qp_result.orientation_residual))
                if qp_result else float("nan")),
            "qp_qdd_slack_max": (float(np.max(qp_result.qdd_slack))
                                 if qp_result else float("nan")),
            "task_accel_saturation": (float(min(qp_result.task_accel_scale))
                                      if qp_result else float("nan")),
            "backend_clamp_hits": float(self._backend_clamp_hits),
            "ee_mode": _EE_MODE_CODES.get(self.ee_estimator.config.mode, float("nan")),
            "ee_residual_norm_m": (
                float(np.linalg.norm(self._ee_estimate.position_residual))
                if self._ee_estimate is not None else float("nan")),
            "ee_orientation_residual_rad": (
                self._ee_estimate.orientation_residual_rad
                if self._ee_estimate is not None else float("nan")),
            "ee_vicon_age_s": (self._ee_estimate.vicon_age
                               if self._ee_estimate is not None else float("nan")),
            "ee_degraded": (1.0 if (self._ee_estimate is not None
                                    and self._ee_estimate.degraded) else 0.0),
            "ee_vel_source": (
                _EE_VELOCITY_SOURCE_CODES.get(
                    self._ee_estimate.linear_velocity_source, float("nan"))
                if self._ee_estimate is not None else float("nan")),
            "ee_omega_source": (
                _EE_VELOCITY_SOURCE_CODES.get(
                    self._ee_estimate.angular_velocity_source, float("nan"))
                if self._ee_estimate is not None else float("nan")),
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
            "j5_q_ref_rad": self._j5_disturbance_diag["q_ref"],
            "j5_qdot_motor_rad_s": float(self._qd_meas[4]),
            "j5_leso_z1_rad": float(self.estimator.leso.position_estimate[4]),
            "j5_leso_z2_rad_s": float(self.estimator.leso.velocity_estimate[4]),
            "j5_leso_z3_raw_rad_s2": self._j5_disturbance_diag["z3_raw"],
            "j5_leso_z3_filtered_rad_s2": self._j5_disturbance_diag["z3_filtered"],
            "j5_M55_kg_m2": self._j5_disturbance_diag["m55"],
            "j5_tau_smc_nm": self._j5_disturbance_diag["tau_smc"],
            "j5_tau_dist_nm": self._j5_disturbance_diag["tau_dist"],
            "j5_tau_final_nm": self._j5_disturbance_diag["tau_final"],
            "j5_s_norm": self._j5_disturbance_diag["s_norm"],
            "j5_torque_saturation": self._j5_disturbance_diag["saturated"],
            "watchdog_flag": float(self._watchdog_tripped),
            "qdd_commanded_j5": self._j5_qdd_commanded_diag,
            "qdd_actual_est_j5": self._j5_qdd_actual_diag,
            "acceleration_residual_j5": self._j5_acceleration_residual_diag,
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
        _ee = self._ee_estimate
        for i, axis in enumerate("xyz"):
            row[f"ee_vicon_{axis}_m"] = (
                float(_ee.vicon_position[i])
                if (_ee is not None and _ee.vicon_position is not None)
                else float("nan"))
            row[f"ee_fk_{axis}_m"] = (float(_ee.fk_position[i])
                                      if _ee is not None else float("nan"))
            row[f"ee_bias_{axis}_m"] = (float(_ee.bias[i])
                                        if _ee is not None else float("nan"))
            row[f"ee_vicon_vel_{axis}"] = (
                float(_ee.vicon_linear_velocity[i])
                if (_ee is not None and _ee.vicon_linear_velocity is not None)
                else float("nan"))
            row[f"ee_fk_vel_{axis}"] = (
                float(_ee.fk_linear_velocity[i])
                if (_ee is not None and _ee.fk_linear_velocity is not None)
                else float("nan"))
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
            # 验收硬指标：这两个数不为 0，就说明后端裁剪重新参与了控制策略、
            # 或者求解器根本没在正常工作。它们比位置 RMS 更早暴露问题。
            "backend_clamp_total": int(self._backend_clamp_hits),
            "watchdog": bool(self._watchdog_tripped),
            "watchdog_reason": self.watchdog_reason,
            "j5_torque_saturation_ticks": int(self._j5_saturation_ticks),
            "qp_enabled": bool(self.use_task_qp),
            "qp_fallback_ticks": int(self._qp_fallback_ticks),
            "ee_mode": self.ee_estimator.config.mode,
            "ee_bias_final_m": (
                [float(v) for v in self._ee_estimate.bias]
                if self._ee_estimate is not None else None),
            "ee_residual_rms_m": (
                float(np.sqrt(np.mean(np.square(self._ee_residuals))))
                if self._ee_residuals else None),
            "ee_orientation_residual_deg_p95": (
                float(np.degrees(np.percentile(self._ee_orientation_residuals, 95)))
                if self._ee_orientation_residuals else None),
            "ee_degraded_ticks": int(self._ee_degraded_ticks),
            # 速度选源的成绩单。配了 vicon 但 rejects/stale 高，说明这一路
            # 没真的在用 —— 不看这几个数就无法判断"接上了"是不是错觉。
            "ee_velocity_source": self.ee_estimator.config.velocity_source,
            "ee_angular_velocity_source":
                self.ee_estimator.config.angular_velocity_source,
            "ee_velocity_reject_ticks": int(
                self.ee_estimator.linear_velocity_rejects),
            "ee_velocity_stale_ticks": int(
                self.ee_estimator.linear_velocity_stale),
            "ee_angular_velocity_reject_ticks": int(
                self.ee_estimator.angular_velocity_rejects),
            "ee_angular_velocity_stale_ticks": int(
                self.ee_estimator.angular_velocity_stale),
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
        # Ctrl-C 是结束不定时长保持运行的正常方式，汇总必须在它之后仍能写出。
        # 这里不能用 rclpy.ok() 作条件：SIGINT 先让 context 失效，走到这里时
        # ok() 已经是 False，汇总 JSON 与报告图会全部丢失。
        # _write_summary 只碰文件和 numpy，context 已死也能跑完。
        if not node.done:
            try:
                node._write_summary()
            except Exception as exc:  # never mask the shutdown itself
                print("could not write the summary: %s" % exc, file=sys.stderr)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
