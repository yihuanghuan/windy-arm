#!/usr/bin/env python3
"""LL100-LH420 NMPC path demo with front-propeller and body obstacle avoidance."""

import csv
import math
import time
from pathlib import Path

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from ll100_traj_pub import SUPPORTED_PATH_MODES, _build_path as _build_ll100_path
from mpc_pinocchio_ik import (
    DEFAULT_LL100_URDF,
    MpcIkConfig,
    MpcPinocchioIK,
    PACKAGE_ROOT,
)


JOINT_NAMES = ["A_Joint", "B_Joint", "C_Joint", "D_Joint", "E_Joint", "F_Joint"]
BASE_FRAME = "base_link"
LL100_NMPC_URDF = str(PACKAGE_ROOT / "config" / "ll100_lh420_nmpc.urdf")
DEFAULT_LL100_BODY_OBSTACLE_BOXES = (
    # [cx, cy, cz, hx, hy, hz], all in LL100 base_link frame.
    (0.05, 0.0, -0.12, 0.30, 0.24, 0.09),   # lower deck / central body
    (-0.05, 0.0, 0.00, 0.24, 0.18, 0.08),    # rear upper body volume
    (0.31, 0.0, -0.13, 0.09, 0.12, 0.05),    # arm mount support block
)


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


PUBLISH_RATE_HZ = 50.0
MAX_TRAIL_POINTS = 2500
LATENCY_WARMUP_SEC = 5.0
LATENCY_LOG_INTERVAL_TICKS = 500
DEFAULT_MAX_JOINT_STEP = 0.010  # 硬件派生 = max_velocity(0.5)·control_dt(0.02),非自由调参项
DEFAULT_PATH_PREVIEW_STRIDE = 4
# 各场景 NMPC 调参预设来自 README §4.2 反抖动调参表,由 tune_ll100_nmpc.py 产出。
SCENARIO_TUNED_PRESETS = {
    "lower_obstacle_circle": dict(
        horizon_steps=6,
        position_weight=252.0,
        terminal_position_weight=1200.0,
        orientation_weight=0.21,
        terminal_orientation_weight=3.0,
        joint_delta_weight=0.112,
        joint_delta_change_weight=0.35,
        current_posture_weight=0.01,
        center_posture_weight=0.005,
    ),
    "avoidance_sweep": dict(
        horizon_steps=8,
        position_weight=252.0,
        terminal_position_weight=2800.0,
        orientation_weight=0.105,
        terminal_orientation_weight=1.05,
        joint_delta_weight=0.224,
        joint_delta_change_weight=0.70,
        current_posture_weight=0.0085,
        center_posture_weight=0.0043,
    ),
    "circle": dict(
        horizon_steps=4,
        position_weight=120.0,
        terminal_position_weight=1020.0,
        orientation_weight=0.30,
        terminal_orientation_weight=2.1,
        joint_delta_weight=0.224,
        joint_delta_change_weight=0.70,
        current_posture_weight=0.007,
        center_posture_weight=0.0035,
    ),
    "front_wing_arc": dict(
        horizon_steps=6,
        position_weight=180.0,
        terminal_position_weight=1200.0,
        orientation_weight=0.30,
        terminal_orientation_weight=3.0,
        joint_delta_weight=0.16,
        joint_delta_change_weight=0.50,
        current_posture_weight=0.002,
        center_posture_weight=0.0085,
    ),
}
# 无预设场景(single_target 等)回退到原全局默认,保持改前行为。
GLOBAL_NMPC_DEFAULTS = dict(
    horizon_steps=8,
    position_weight=120.0,
    terminal_position_weight=1200.0,
    orientation_weight=0.3,
    terminal_orientation_weight=3.0,
    joint_delta_weight=0.08,
    joint_delta_change_weight=0.25,
    current_posture_weight=0.005,
    center_posture_weight=0.01,
)
NMPC_TUNED_PARAM_NAMES = tuple(GLOBAL_NMPC_DEFAULTS.keys())


def _resolve_nmpc_param(name: str, raw, path_mode: str):
    if raw is not None and raw > 0:
        return raw
    preset = SCENARIO_TUNED_PRESETS.get(path_mode, GLOBAL_NMPC_DEFAULTS)
    return preset.get(name, GLOBAL_NMPC_DEFAULTS[name])


# 分段避障半径:关节(电机)粗、连杆(杆身)细。数值来自各连杆 STL 网格绕真实杆轴(相邻 frame 原点
# 连线)的最大垂距(=包络上界),再留 ~1-2mm 缓冲。frame 链 = A,B,C,D,E,F(6 个 → 6 关节半径/5 段半径)。
DEFAULT_CAPSULE_JOINT_RADII = [0.052, 0.062, 0.058, 0.050, 0.049, 0.040]  # A,B,C,D,E,F 关节(电机)
DEFAULT_CAPSULE_LINK_RADII = [0.045, 0.035, 0.043, 0.016, 0.035]          # A→B,B→C,C→D,D→E,E→F 杆身
# 中心线胶囊点(最精确的臂避障模型,优先于上面的关节连线采样)。每条 = (frame, x, y, z, radius):
# (x,y,z) 是球心在该 frame 局部坐标下的位置——取自连杆 STL 网格沿关节轴的横截面质心,使圆心贴住真实
# 杆中心线(不再落在关节连线上);radius = 覆盖该处网格的球半径(细杆细/电机粗,含无缝覆盖膨胀,
# 采样间距 30mm、R=√(局部粗细²+15²)+6mm)。共 38 点、全程无缝。由网格离线生成(见 scratchpad 脚本)。
CAPSULE_CENTERLINE_POINTS = (
    # A_Link
    # ("A_Link", -0.00042, -0.00101, -0.04925, 0.0298),
    # ("A_Link", -0.00037, -0.00416, -0.00308, 0.0592),
    ("A_Link", +0.01171, -0.00023, +0.00756, 0.04),
    # ("A_Link", +0.02404, +0.01110, +0.01370, 0.0781),
    # ("A_Link", +0.03001, +0.04512, +0.03552, 0.0732),
    # ("A_Link", +0.03129, +0.06386, +0.05199, 0.0636),
    # ("A_Link", +0.03375, +0.06799, +0.05483, 0.0650),
    # B_Link
    # ("B_Link", +0.00067, -0.01710, -0.00594, 0.0619),
    ("B_Link", +0.00044, -0.00245, -0.00858, 0.05),
    # ("B_Link", -0.00015, +0.01696, -0.01146, 0.0644),
    # ("B_Link", -0.00059, +0.04107, -0.00659, 0.0646),
    ("B_Link", -0.00111, +0.08214, +0.00228, 0.0405),
    # ("B_Link", -0.00012, +0.12458, +0.00194, 0.0295),
    # ("B_Link", -0.00012, +0.12519, +0.00188, 0.0296),
    ("B_Link", +0.00004, +0.18057, -0.00091, 0.06),
    # ("B_Link", -0.00007, +0.21268, +0.00177, 0.0311),
    # ("B_Link", +0.00064, +0.23314, -0.00994, 0.0555),
    # ("B_Link", +0.00045, +0.24989, -0.01431, 0.0520),
    # ("B_Link", -0.00008, +0.26489, -0.01675, 0.0507),
    # C_Link
    # ("C_Link", -0.00036, -0.00477, -0.00080, 0.0557),
    # ("C_Link", +0.00021, +0.00855, -0.00178, 0.0586),
    ("C_Link", -0.00708, +0.04951, -0.01685, 0.07),
    # ("C_Link", -0.00919, +0.05994, -0.01908, 0.0493),
    # ("C_Link", -0.01335, +0.07643, -0.02116, 0.0505),
    # D_Link
    # ("D_Link", +0.00003, +0.00001, -0.00111, 0.0319),
    # ("D_Link", +0.00014, +0.00033, -0.00097, 0.0319),
    ("D_Link", -0.00005, -0.00001, +0.06566, 0.0248),
    ("D_Link", +0.00000, +0.00068, +0.12460, 0.0389),
    # ("D_Link", -0.00901, -0.00534, +0.14562, 0.0564),
    # ("D_Link", -0.00519, -0.00016, +0.15930, 0.0546),
    # ("D_Link", -0.00219, +0.00389, +0.16926, 0.0474),
    # E_Link
    # ("E_Link", -0.00391, +0.00023, +0.01097, 0.0309),
    ("E_Link", +0.01872, +0.00203, +0.00453, 0.0457),
    # ("E_Link", +0.04022, +0.00498, -0.02285, 0.0562),
    ("E_Link", +0.06485, +0.00128, -0.02248, 0.0516),
    # ("E_Link", +0.07330, -0.00012, -0.02188, 0.0478),
    # F_Link
    ("F_Link", +0.00004, -0.00001, -0.00189, 0.0223),
)


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


def _build_path(
    mode: str,
    period: float,
    dt: float,
    obstacle_disks: np.ndarray = None,
    single_target: np.ndarray = None,
) -> np.ndarray:
    if mode == "single_target":
        if single_target is None:
            raise ValueError("single_target 模式需要提供 single_target_position")
        point = np.asarray(single_target, dtype=float).reshape(3)
        # 单点路径:整个时域参考退化成同一个固定目标点,到点后保持。
        return point.reshape(1, 3)
    count = max(20, int(round(period / dt)))
    if mode in ("circle", "lower_obstacle_circle", "avoidance_sweep", "front_wing_arc"):
        return _build_ll100_path(mode, count)
    raise ValueError(
        f"path_mode 必须是 {SUPPORTED_PATH_MODES} 或 'single_target'"
    )


class MoveLL100LH420NmpcDemo(Node):
    def __init__(self):
        super().__init__("move_ll100_lh420_nmpc_demo")
        self.declare_parameter("path_mode", "lower_obstacle_circle")
        self.declare_parameter("single_target_position", [0.50, 0.0, 0.10])
        self.declare_parameter("period_sec", 20.0)  # 调参定稿:20s/圈达 ~5mm
        self.declare_parameter("terminal_tolerance", 0.025)
        self.declare_parameter("horizon_steps", 0)
        self.declare_parameter("max_joint_step", DEFAULT_MAX_JOINT_STEP)
        self.declare_parameter("path_preview_stride", DEFAULT_PATH_PREVIEW_STRIDE)
        self.declare_parameter("position_weight", -1.0)
        self.declare_parameter("terminal_position_weight", -1.0)
        self.declare_parameter("lock_orientation", True)
        self.declare_parameter("orientation_target", "vertical_down")
        self.declare_parameter("orientation_weight", -1.0)
        self.declare_parameter("terminal_orientation_weight", -1.0)
        self.declare_parameter("obstacle_avoidance_enabled", True)
        self.declare_parameter("obstacle_safety_margin", 0.005)  # 桨叶:小正余量,保证实际不接触
        self.declare_parameter("obstacle_disk_half_thickness", 0.02)
        self.declare_parameter("body_obstacle_enabled", True)
        self.declare_parameter("body_obstacle_margin", 0.005)  # 机身:小正余量,保证实际不接触
        self.declare_parameter("capsule_radius", 0.018)  # 分段半径留空时的回退标量;默认走下面分段半径
        self.declare_parameter("capsule_joint_radii", DEFAULT_CAPSULE_JOINT_RADII)  # 各 frame 关节(电机)半径
        self.declare_parameter("capsule_link_radii", DEFAULT_CAPSULE_LINK_RADII)    # 各段连杆(杆身)半径
        self.declare_parameter("capsule_samples_per_segment", 3)  # 每段点数=此值+2 → 5 点/段,减小长杆采样缝
        self.declare_parameter("front_obstacle_capsule", True)      # 前桨按连杆 capsule 建模(含粗细),非仅关键点
        self.declare_parameter("obstacle_repulsion_margin", 0.03)   # 斥力场阈值 m;>0 启用,进此距离内被推离障碍
        self.declare_parameter("obstacle_repulsion_weight", 1.0e4)  # 斥力强度;默认打开(NMPC 主动留净空、不贴墙)
        self.declare_parameter("advance_on_feedback", False)
        self.declare_parameter("start_from_nearest_path_point", True)
        self.declare_parameter("hold_start_until_reached", True)
        self.declare_parameter("target_switch_tolerance", 0.005)  # 启动贴紧首点(5mm)
        self.declare_parameter("gentle_start", True)  # 启动阶段用贴近末端的 approach carrot,避免猛冲
        self.declare_parameter("approach_speed", 0.05)  # carrot 沿当前末端→起点直线推进速度,m/s
        self.declare_parameter("approach_lead", 0.04)  # carrot 最多领先实际末端距离,m;跟不上则暂停推进
        self.declare_parameter("start_reach_timeout_sec", 8.0)  # 启动到位等待超时,s
        self.declare_parameter("start_relaxed_tolerance", 0.02)  # 超时后启动滚动跟踪的放宽阈值,m
        self.declare_parameter("min_manip_warn", 0.0085)  # 可操作度低于此值时提示接近奇异
        self.declare_parameter("stuck_error_epsilon", 0.002)
        self.declare_parameter("stuck_steps_before_advance", 120)
        self.declare_parameter("stuck_advance_points", 10)
        self.declare_parameter("command_filter_enabled", True)  # 输出命令限速/限加速度,抑制在线闭环反向抽动
        self.declare_parameter("command_velocity_limit", 0.35)  # rad/s,略低于底层 max_velocity=0.5
        self.declare_parameter("command_accel_limit", 1.5)      # rad/s^2,比底层 5.0 更柔顺
        self.declare_parameter("current_posture_weight", -1.0)
        self.declare_parameter("center_posture_weight", -1.0)
        self.declare_parameter("joint_delta_weight", -1.0)         # 一阶平滑(惩罚 Δq),NMPC 代价权重
        self.declare_parameter("joint_delta_change_weight", -1.0)  # 二阶平滑(惩罚 ΔΔq/抑抖),NMPC 代价权重
        self.declare_parameter("metrics_log_sec", 0.0)
        self.declare_parameter("latency_profiling", False)
        # RViz marker 发布频率 Hz。可视化不需要 50Hz,降频可省下每 tick 重建
        # MarkerArray(含尾迹最多 2500 个 Point)的 CPU;<=0 完全关闭,>=50 每 tick 发。
        self.declare_parameter("marker_publish_hz", 10.0)
        # 求解器选择:只部署 acados(SQP_RTI + HPIPM/qpOASES) 与 GRAMPC。
        self.declare_parameter("solver_backend", "acados")
        self.declare_parameter("acados_nlp_solver_type", "SQP_RTI")
        self.declare_parameter("acados_qp_solver", "FULL_CONDENSING_QPOASES")
        self.declare_parameter("acados_qp_solver_iter_max", 500)
        self.declare_parameter("acados_qp_warm_start", 1)

        self.path_mode = str(self.get_parameter("path_mode").value)
        self.single_target_position = np.array(
            self.get_parameter("single_target_position").value, dtype=float
        )
        self.period_sec = float(self.get_parameter("period_sec").value)
        self.terminal_tolerance = float(self.get_parameter("terminal_tolerance").value)
        self.max_joint_step = float(self.get_parameter("max_joint_step").value)
        self.path_preview_stride = int(self.get_parameter("path_preview_stride").value)
        self.lock_orientation = bool(self.get_parameter("lock_orientation").value)
        self.orientation_target = str(self.get_parameter("orientation_target").value)
        self.obstacle_avoidance_enabled = bool(self.get_parameter("obstacle_avoidance_enabled").value)
        self.obstacle_safety_margin = float(self.get_parameter("obstacle_safety_margin").value)
        self.obstacle_disk_half_thickness = float(self.get_parameter("obstacle_disk_half_thickness").value)
        self.body_obstacle_enabled = bool(self.get_parameter("body_obstacle_enabled").value)
        self.body_obstacle_margin = float(self.get_parameter("body_obstacle_margin").value)
        self.capsule_radius = float(self.get_parameter("capsule_radius").value)
        self.capsule_joint_radii = [float(x) for x in self.get_parameter("capsule_joint_radii").value]
        self.capsule_link_radii = [float(x) for x in self.get_parameter("capsule_link_radii").value]
        self.capsule_samples_per_segment = int(self.get_parameter("capsule_samples_per_segment").value)
        self.front_obstacle_capsule = bool(self.get_parameter("front_obstacle_capsule").value)
        self.obstacle_repulsion_margin = float(self.get_parameter("obstacle_repulsion_margin").value)
        self.obstacle_repulsion_weight = float(self.get_parameter("obstacle_repulsion_weight").value)
        self.advance_on_feedback = bool(self.get_parameter("advance_on_feedback").value)
        self.start_from_nearest_path_point = bool(self.get_parameter("start_from_nearest_path_point").value)
        self.hold_start_until_reached = bool(self.get_parameter("hold_start_until_reached").value)
        self.target_switch_tolerance = float(self.get_parameter("target_switch_tolerance").value)
        self.gentle_start = bool(self.get_parameter("gentle_start").value)
        self.approach_speed = float(self.get_parameter("approach_speed").value)
        self.approach_lead = float(self.get_parameter("approach_lead").value)
        self.start_reach_timeout_sec = float(self.get_parameter("start_reach_timeout_sec").value)
        self.start_relaxed_tolerance = float(self.get_parameter("start_relaxed_tolerance").value)
        self.min_manip_warn = float(self.get_parameter("min_manip_warn").value)
        self.stuck_error_epsilon = float(self.get_parameter("stuck_error_epsilon").value)
        self.stuck_steps_before_advance = int(self.get_parameter("stuck_steps_before_advance").value)
        self.stuck_advance_points = int(self.get_parameter("stuck_advance_points").value)
        self.command_filter_enabled = bool(self.get_parameter("command_filter_enabled").value)
        self.command_velocity_limit = float(self.get_parameter("command_velocity_limit").value)
        self.command_accel_limit = float(self.get_parameter("command_accel_limit").value)
        raw_nmpc_params = {
            "horizon_steps": int(self.get_parameter("horizon_steps").value),
            "position_weight": float(self.get_parameter("position_weight").value),
            "terminal_position_weight": float(self.get_parameter("terminal_position_weight").value),
            "orientation_weight": float(self.get_parameter("orientation_weight").value),
            "terminal_orientation_weight": float(self.get_parameter("terminal_orientation_weight").value),
            "joint_delta_weight": float(self.get_parameter("joint_delta_weight").value),
            "joint_delta_change_weight": float(self.get_parameter("joint_delta_change_weight").value),
            "current_posture_weight": float(self.get_parameter("current_posture_weight").value),
            "center_posture_weight": float(self.get_parameter("center_posture_weight").value),
        }
        resolved_nmpc_params = {
            name: _resolve_nmpc_param(name, raw_nmpc_params[name], self.path_mode)
            for name in NMPC_TUNED_PARAM_NAMES
        }
        self.horizon_steps = int(resolved_nmpc_params["horizon_steps"])
        self.position_weight = float(resolved_nmpc_params["position_weight"])
        self.terminal_position_weight = float(resolved_nmpc_params["terminal_position_weight"])
        self.orientation_weight = float(resolved_nmpc_params["orientation_weight"])
        self.terminal_orientation_weight = float(resolved_nmpc_params["terminal_orientation_weight"])
        self.joint_delta_weight = float(resolved_nmpc_params["joint_delta_weight"])
        self.joint_delta_change_weight = float(resolved_nmpc_params["joint_delta_change_weight"])
        self.current_posture_weight = float(resolved_nmpc_params["current_posture_weight"])
        self.center_posture_weight = float(resolved_nmpc_params["center_posture_weight"])
        preset_name = self.path_mode if self.path_mode in SCENARIO_TUNED_PRESETS else "GLOBAL_NMPC_DEFAULTS"
        user_overrides = [
            f"{name}={raw_nmpc_params[name]:g}"
            for name in NMPC_TUNED_PARAM_NAMES
            if raw_nmpc_params[name] > 0
        ]
        resolved_text = ", ".join(
            f"{name}={resolved_nmpc_params[name]:g}" for name in NMPC_TUNED_PARAM_NAMES
        )
        self.get_logger().info(
            f"NMPC 调参预设: path_mode={self.path_mode}, preset={preset_name}, "
            f"user_overrides=[{', '.join(user_overrides) if user_overrides else '无'}], "
            f"resolved=[{resolved_text}]"
        )
        self.metrics_log_sec = float(self.get_parameter("metrics_log_sec").value)
        self.latency_profiling = bool(self.get_parameter("latency_profiling").value)
        marker_publish_hz = float(self.get_parameter("marker_publish_hz").value)
        if marker_publish_hz <= 0.0:
            self._marker_stride = 0  # 0 = 不发布 marker
        else:
            self._marker_stride = max(1, int(round(PUBLISH_RATE_HZ / marker_publish_hz)))
        self._marker_tick_count = 0
        self.solver_backend = str(self.get_parameter("solver_backend").value)
        self.acados_nlp_solver_type = str(self.get_parameter("acados_nlp_solver_type").value)
        self.acados_qp_solver = str(self.get_parameter("acados_qp_solver").value)
        self.acados_qp_solver_iter_max = int(self.get_parameter("acados_qp_solver_iter_max").value)
        self.acados_qp_warm_start = int(self.get_parameter("acados_qp_warm_start").value)

        if self.solver_backend not in ("acados", "grampc"):
            raise ValueError("solver_backend 只支持 'acados' 或 'grampc'")
        if self.acados_nlp_solver_type != "SQP_RTI":
            raise ValueError("acados_nlp_solver_type 只部署 'SQP_RTI'")
        if self.acados_qp_solver not in ("PARTIAL_CONDENSING_HPIPM", "FULL_CONDENSING_QPOASES"):
            raise ValueError(
                "acados_qp_solver 只支持 'PARTIAL_CONDENSING_HPIPM' 或 'FULL_CONDENSING_QPOASES'"
            )
        if self.acados_qp_solver_iter_max < 1:
            raise ValueError("acados_qp_solver_iter_max 必须 >= 1")
        if self.acados_qp_warm_start not in (0, 1, 2):
            raise ValueError("acados_qp_warm_start 只支持 0/1/2")
        if self.period_sec <= 0.0:
            raise ValueError("period_sec 必须 > 0")
        if self.horizon_steps < 1:
            raise ValueError("horizon_steps 必须 >= 1")
        if self.max_joint_step <= 0.0:
            raise ValueError("max_joint_step 必须 > 0")
        if self.path_preview_stride < 1:
            raise ValueError("path_preview_stride 必须 >= 1")
        if self.command_velocity_limit <= 0.0:
            raise ValueError("command_velocity_limit 必须 > 0")
        if self.command_accel_limit <= 0.0:
            raise ValueError("command_accel_limit 必须 > 0")
        if self.orientation_target not in ("initial", "vertical_down"):
            raise ValueError("orientation_target 必须是 'initial' 或 'vertical_down'")
        if self.path_mode == "single_target" and self.single_target_position.shape != (3,):
            raise ValueError("single_target_position 必须是长度为 3 的数组,例如 [0.50, 0.0, 0.10]")
        if not Path(LL100_NMPC_URDF).exists():
            raise FileNotFoundError(f"找不到 LL100 NMPC URDF: {LL100_NMPC_URDF}")

        self.dt = 1.0 / PUBLISH_RATE_HZ
        cfg = MpcIkConfig(
            horizon_steps=self.horizon_steps,
            control_dt=self.dt,
            max_joint_step=self.max_joint_step,
            success_tolerance=self.terminal_tolerance,
            solver_backend=self.solver_backend,
            acados_build_dir=f"/tmp/windylab_acados_ll100_lh420_path_{self.path_mode}",
            acados_nlp_solver_type=self.acados_nlp_solver_type,
            acados_qp_solver=self.acados_qp_solver,
            acados_qp_solver_iter_max=self.acados_qp_solver_iter_max,
            acados_qp_warm_start=self.acados_qp_warm_start,
            position_weight=self.position_weight,
            terminal_position_weight=self.terminal_position_weight,
            orientation_weight=self.orientation_weight if self.lock_orientation else 0.0,
            terminal_orientation_weight=self.terminal_orientation_weight if self.lock_orientation else 0.0,
            obstacle_avoidance_enabled=self.obstacle_avoidance_enabled,
            # 安全第一:桨叶避障只走硬约束(共享求解器已定为 hard-only,不再传 hard/soft 开关)
            obstacle_safety_margin=self.obstacle_safety_margin,
            obstacle_disk_half_thickness=self.obstacle_disk_half_thickness,
            obstacle_ll100_urdf_path=DEFAULT_LL100_URDF,
            obstacle_coordinate_mode="ll100_base",
            obstacle_keypoint_frames=("A_Link", "B_Link", "C_Link", "D_Link", "E_Link", "F_Link"),
            body_obstacle_enabled=self.body_obstacle_enabled,
            # 安全第一:机身避障只走硬约束(共享求解器已定为 hard-only,不再传 hard/soft 开关)
            body_obstacle_margin=self.body_obstacle_margin,
            body_obstacle_boxes=DEFAULT_LL100_BODY_OBSTACLE_BOXES,
            capsule_radius=self.capsule_radius,
            capsule_joint_radii=tuple(self.capsule_joint_radii),
            capsule_link_radii=tuple(self.capsule_link_radii),
            capsule_points=CAPSULE_CENTERLINE_POINTS,
            capsule_samples_per_segment=self.capsule_samples_per_segment,
            front_obstacle_capsule=self.front_obstacle_capsule,
            obstacle_repulsion_margin=self.obstacle_repulsion_margin,
            obstacle_repulsion_weight=self.obstacle_repulsion_weight,
            capsule_frame_sequence=("A_Link", "B_Link", "C_Link", "D_Link", "E_Link", "F_Link"),
            current_posture_weight=self.current_posture_weight,
            center_posture_weight=self.center_posture_weight,
            joint_delta_weight=self.joint_delta_weight,
            joint_delta_change_weight=self.joint_delta_change_weight,
        )
        # 三种部署目标:acados(qpOASES 默认 / HPIPM,走 MpcPinocchioIK)、GRAMPC(GrampcRunner)。
        if self.solver_backend == "grampc":
            from grampc_ll100 import GrampcRunner
            self.mpc_ik = GrampcRunner(urdf_path=LL100_NMPC_URDF, ee_frame="F_Link", config=cfg)
        else:
            self.mpc_ik = MpcPinocchioIK(urdf_path=LL100_NMPC_URDF, ee_frame="F_Link", config=cfg)
        self.path = _build_path(
            self.path_mode,
            self.period_sec,
            self.dt,
            self.mpc_ik.obstacle_disks(),
            single_target=self.single_target_position,
        )
        self.q_last = np.clip(pin.neutral(self.mpc_ik.model), self.mpc_ik.q_lower, self.mpc_ik.q_upper)
        self._command_filter_initialized = False
        self._last_command_velocity = np.zeros(len(JOINT_NAMES), dtype=float)
        self.q_feedback = None
        self.path_index_initialized = False
        self.locked_rotation = self._orientation_target_rotation(self.q_last)
        self.warm_start = None
        self.t = 0.0
        self.path_index = 0
        self.path_time_offset_index = 0
        self.path_time_tracking_started = not self.hold_start_until_reached
        self.path_time_start_sec = None
        self.path_time_start_index = 0
        self.best_current_target_error = float("inf")
        self.stuck_steps = 0
        self.trajectory_start_time_sec = None
        self._approach_carrot = None
        self._start_wait_start_time_sec = None
        self.actual_trail = []
        self._last_feedback_perf_counter = None
        self._latency_profile_start_perf_counter = None
        self._latency_rows = []
        self._latency_csv_file = None
        self._latency_csv_writer = None
        self._latency_csv_path = None
        self._latency_finalized = False
        if self.latency_profiling:
            data_dir = Path(__file__).resolve().parent / "data"
            data_dir.mkdir(parents=True, exist_ok=True)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            self._latency_csv_path = data_dir / f"latency_{timestamp}.csv"
            self._latency_csv_file = self._latency_csv_path.open("w", newline="")
            self._latency_csv_writer = csv.writer(self._latency_csv_file)
            self._latency_csv_writer.writerow(
                ["tick", "elapsed_sec", "feedback_age_sec", "compute_sec", "end_to_end_sec"]
            )
            self.get_logger().info(
                "Latency profiling enabled: "
                f"warmup={LATENCY_WARMUP_SEC:.1f}s, csv={self._latency_csv_path}"
            )

        self.command_pub = self.create_publisher(JointState, "/student/joint_command", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/ll100_nmpc_path/markers", 10)
        self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self._joint_state_callback, 10)
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            "LL100 路径避障 NMPC 启动: "
            f"path_mode={self.path_mode}, period={self.period_sec:.2f}s, "
            f"horizon={self.horizon_steps}, max_joint_step={self.max_joint_step:.4f}, "
            f"path_preview_stride={self.path_preview_stride}, "
            f"lock_orientation={self.lock_orientation}, orientation_target={self.orientation_target}, "
            f"advance_on_feedback={self.advance_on_feedback}, start_nearest={self.start_from_nearest_path_point}, "
            f"hold_start={self.hold_start_until_reached}, "
            f"command_filter={self.command_filter_enabled}(v={self.command_velocity_limit:.2f}, "
            f"a={self.command_accel_limit:.2f}), "
            f"front_obstacle={self.obstacle_avoidance_enabled}(hard-only), "
            f"body_obstacle={self.body_obstacle_enabled}(hard-only)"
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

    def _joint_state_callback(self, msg: JointState):
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        if not all(name in name_to_index for name in JOINT_NAMES):
            # LL100 rotor publisher also publishes /joint_states with only rot1~rot4.
            # Ignore those partial messages; the student_arm_node message has A~F.
            return
        q = np.array([msg.position[name_to_index[name]] for name in JOINT_NAMES], dtype=float)
        if q.shape == (len(JOINT_NAMES),) and np.all(np.isfinite(q)):
            self.q_feedback = np.clip(q, self.mpc_ik.q_lower, self.mpc_ik.q_upper)
            if self.latency_profiling:
                self._last_feedback_perf_counter = time.perf_counter()

    def _record_latency_sample(self, t0: float, t_pub: float, feedback_time: float):
        if self._latency_profile_start_perf_counter is None:
            self._latency_profile_start_perf_counter = t_pub
            return
        elapsed = t_pub - self._latency_profile_start_perf_counter
        if elapsed < LATENCY_WARMUP_SEC:
            return

        feedback_age = t0 - feedback_time
        compute = t_pub - t0
        end_to_end = t_pub - feedback_time
        tick = len(self._latency_rows) + 1
        row = (tick, elapsed, feedback_age, compute, end_to_end)
        self._latency_rows.append(row)
        if self._latency_csv_writer is not None:
            self._latency_csv_writer.writerow(row)
        if tick % LATENCY_LOG_INTERVAL_TICKS == 0:
            self._log_latency_stats(prefix=f"latency profiling {tick} ticks")
            if self._latency_csv_file is not None:
                self._latency_csv_file.flush()

    def _latency_summary(self, values):
        arr = np.asarray(values, dtype=float) * 1e3
        return {
            "mean": float(np.mean(arr)),
            "p50": float(np.percentile(arr, 50)),
            "p95": float(np.percentile(arr, 95)),
            "max": float(np.max(arr)),
        }

    def _log_latency_stats(self, prefix: str):
        if not self._latency_rows:
            self.get_logger().info(
                f"[{prefix}] no post-warmup latency samples yet "
                f"(warmup={LATENCY_WARMUP_SEC:.1f}s)"
            )
            return
        labels = ("feedback_age", "compute", "end_to_end")
        columns = list(zip(*[(r[2], r[3], r[4]) for r in self._latency_rows]))
        parts = []
        for label, values in zip(labels, columns):
            s = self._latency_summary(values)
            parts.append(
                f"{label}: mean={s['mean']:.3f}ms p50={s['p50']:.3f}ms "
                f"p95={s['p95']:.3f}ms max={s['max']:.3f}ms"
            )
        self.get_logger().info(f"[{prefix}] samples={len(self._latency_rows)}; " + "; ".join(parts))

    def _finalize_latency_profiling(self):
        if self._latency_finalized or not self.latency_profiling:
            return
        self._latency_finalized = True
        self._log_latency_stats(prefix="latency profiling final")
        if self._latency_csv_file is not None:
            self._latency_csv_file.flush()
            self._latency_csv_file.close()
            self.get_logger().info(f"Latency profiling CSV written: {self._latency_csv_path}")

    def destroy_node(self):
        self._finalize_latency_profiling()
        return super().destroy_node()

    def _target_sequence(self) -> np.ndarray:
        offsets = self.path_preview_stride * np.arange(self.horizon_steps)
        if self.advance_on_feedback:
            indices = (self.path_index + offsets) % len(self.path)
            return self.path[indices]
        if not self.path_time_tracking_started:
            if self.gentle_start and self._approach_carrot is not None:
                return np.repeat(self._approach_carrot.reshape(1, 3), self.horizon_steps, axis=0)
            return np.repeat(self.path[self.path_index].reshape(1, 3), self.horizon_steps, axis=0)
        start = (self.path_time_start_index + self._time_path_index()) % len(self.path)
        indices = (start + offsets) % len(self.path)
        return self.path[indices]

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tracking_elapsed_sec(self) -> float:
        if self.path_time_start_sec is None:
            return 0.0
        return max(0.0, self._now_sec() - self.path_time_start_sec)

    def _time_path_index(self) -> int:
        if self.path_time_tracking_started and self.path_time_start_sec is not None:
            elapsed = self._tracking_elapsed_sec()
            return int(round((elapsed % self.period_sec) / self.dt)) % len(self.path)
        return int(round((self.t % self.period_sec) / self.dt)) % len(self.path)

    def _initialize_path_index(self, actual_pos: np.ndarray):
        if self.start_from_nearest_path_point:
            distances = np.linalg.norm(self.path - actual_pos.reshape(1, 3), axis=1)
            nearest_index = int(np.argmin(distances))
            self.path_index = nearest_index
            if self.path_mode == "front_wing_arc" and distances[nearest_index] > self.target_switch_tolerance:
                center_candidates = np.array([len(self.path) // 4, 3 * len(self.path) // 4], dtype=int)
                candidate_distances = distances[center_candidates]
                self.path_index = int(center_candidates[int(np.argmin(candidate_distances))])
            self.path_time_offset_index = (self.path_index - self._time_path_index()) % len(self.path)
            self.best_current_target_error = float(distances[self.path_index])
            self.get_logger().info(
                "初始化弧线路径并入点: "
                f"path_index={self.path_index}, nearest_index={nearest_index}, "
                f"error={self.best_current_target_error:.4f}, "
                f"time_offset={self.path_time_offset_index}"
            )
        self.path_index_initialized = True

    def _advance_to_next_path_index(self, steps: int = 1):
        next_index = (self.path_index + max(1, int(steps))) % len(self.path)
        if next_index < self.path_index:
            self.actual_trail.clear()
        self.path_index = next_index
        self.best_current_target_error = float("inf")
        self.stuck_steps = 0

    def _start_time_tracking(self, error: float, log_prefix: str):
        self.path_time_start_sec = self._now_sec()
        self.path_time_start_index = self.path_index
        self.path_time_offset_index = self.path_index
        self.t = 0.0
        self.path_time_tracking_started = True
        self._start_wait_start_time_sec = None
        self.get_logger().info(
            f"{log_prefix}: "
            f"path_index={self.path_index}, error={error:.4f}, "
            f"wall_clock_tracking=True"
        )

    def _start_wait_elapsed_sec(self) -> float:
        now_sec = self._now_sec()
        if self._start_wait_start_time_sec is None:
            self._start_wait_start_time_sec = now_sec
        return now_sec - self._start_wait_start_time_sec

    def _update_approach_carrot(self, actual_pos: np.ndarray):
        if not self.gentle_start or self.path_time_tracking_started:
            return
        if self._approach_carrot is None:
            self._approach_carrot = actual_pos.copy()
        lead_limit = max(self.approach_lead, 0.0)
        current_lead = float(np.linalg.norm(self._approach_carrot - actual_pos))
        if current_lead > lead_limit:
            return
        target = self.path[self.path_index]
        delta = target - self._approach_carrot
        distance = float(np.linalg.norm(delta))
        if distance <= 1e-12:
            self._approach_carrot = target.copy()
            return
        step = min(max(self.approach_speed, 0.0) * self.dt, distance, max(lead_limit - current_lead, 0.0))
        if step <= 0.0:
            return
        self._approach_carrot = self._approach_carrot + (step / distance) * delta

    def _filtered_command(self, q_desired: np.ndarray, q_current: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Limit the published command's velocity and acceleration to avoid online tick-to-tick reversals."""
        if not self.command_filter_enabled:
            dq_cmd = (q_desired - q_current) / self.dt
            self.q_last = np.clip(q_desired, self.mpc_ik.q_lower, self.mpc_ik.q_upper)
            self._last_command_velocity = np.clip(dq_cmd, -self.command_velocity_limit, self.command_velocity_limit)
            self._command_filter_initialized = True
            return self.q_last.copy(), dq_cmd

        if not self._command_filter_initialized:
            self.q_last = q_current.copy()
            self._last_command_velocity = np.zeros(len(JOINT_NAMES), dtype=float)
            self._command_filter_initialized = True

        v_des = (q_desired - self.q_last) / self.dt
        v_des = np.clip(v_des, -self.command_velocity_limit, self.command_velocity_limit)
        dv = self.command_accel_limit * self.dt
        v_cmd = np.clip(v_des, self._last_command_velocity - dv, self._last_command_velocity + dv)
        q_cmd = np.clip(self.q_last + v_cmd * self.dt, self.mpc_ik.q_lower, self.mpc_ik.q_upper)
        # Recompute actual velocity after joint-limit clipping.
        v_cmd = (q_cmd - self.q_last) / self.dt
        self.q_last = q_cmd
        self._last_command_velocity = v_cmd
        return q_cmd.copy(), v_cmd.copy()

    def _advance_path_index_if_ready(self, actual_pos: np.ndarray):
        if not self.path_index_initialized:
            self._initialize_path_index(actual_pos)
        if not self.advance_on_feedback:
            if self.path_time_tracking_started and self.path_time_start_sec is None:
                target = self.path[self.path_index]
                error = float(np.linalg.norm(actual_pos - target))
                self._start_time_tracking(error, "启动按真实时间推进")
                return
            if not self.path_time_tracking_started:
                target = self.path[self.path_index]
                error = float(np.linalg.norm(actual_pos - target))
                if not self.hold_start_until_reached or error <= self.target_switch_tolerance:
                    self._start_time_tracking(error, "已进入弧线并启动按时间推进")
                    return
                if self.gentle_start:
                    wait_elapsed = self._start_wait_elapsed_sec()
                    if (
                        wait_elapsed > self.start_reach_timeout_sec
                        and error <= self.start_relaxed_tolerance
                    ):
                        self._start_time_tracking(
                            error,
                            "启动到位超时,以放宽阈值启动滚动跟踪",
                        )
                        return
                    self.get_logger().warn(
                        "启动等待尚未到位: "
                        f"error={error:.4f}, "
                        f"target_tol={self.target_switch_tolerance:.4f}, "
                        f"relaxed_tol={self.start_relaxed_tolerance:.4f}",
                        throttle_duration_sec=2.0,
                    )
            return
        target = self.path[self.path_index]
        error = float(np.linalg.norm(actual_pos - target))
        if error <= self.target_switch_tolerance:
            self._advance_to_next_path_index()
            return
        if error < self.best_current_target_error - self.stuck_error_epsilon:
            self.best_current_target_error = error
            self.stuck_steps = 0
            return
        self.stuck_steps += 1
        if self.stuck_steps >= self.stuck_steps_before_advance:
            self.get_logger().warn(
                "当前路径点误差长时间没有继续下降,自动推进到下一个点: "
                f"path_index={self.path_index}, error={error:.4f}, "
                f"best={self.best_current_target_error:.4f}",
                throttle_duration_sec=2.0,
            )
            self._advance_to_next_path_index(self.stuck_advance_points)

    def tick(self):
        t0 = time.perf_counter() if self.latency_profiling else None
        if self.q_feedback is None:
            self.get_logger().warn("尚未收到 /joint_states 的 A_Joint~F_Joint 反馈,暂不发布路径命令", throttle_duration_sec=2.0)
            return
        feedback_time = self._last_feedback_perf_counter
        q_current = self.q_feedback.copy()
        actual_pos, _ = self.mpc_ik.forward(q_current)
        self._advance_path_index_if_ready(actual_pos)
        self._update_approach_carrot(actual_pos)
        targets = self._target_sequence()
        jac = pin.computeFrameJacobian(
            self.mpc_ik.model,
            self.mpc_ik.data,
            q_current,
            self.mpc_ik.frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )[:3, :]
        manip = math.sqrt(max(float(np.linalg.det(jac @ jac.T)), 0.0))
        if manip < self.min_manip_warn:
            self.get_logger().warn(
                "接近奇异: "
                f"manip={manip:.6f}, threshold={self.min_manip_warn:.6f}",
                throttle_duration_sec=2.0,
            )
        target_rotation = self.locked_rotation if self.lock_orientation else None
        q_next, ok, info = self.mpc_ik.step(
            targets,
            q_current,
            warm_start=self.warm_start,
            target_rotation=target_rotation,
        )
        self.warm_start = info["trajectory"]

        command_usable = bool(info.get("command_usable", ok))
        dq_cmd = np.zeros(len(JOINT_NAMES))
        if command_usable:
            _, dq_cmd = self._filtered_command(q_next, q_current)
        else:
            self._last_command_velocity = np.zeros(len(JOINT_NAMES), dtype=float)
            self.get_logger().warn(
                "路径 NMPC 命令不可用,保持上一帧: "
                f"terminal_error={info['terminal_error']:.4f}, "
                f"min_margin={info.get('min_obstacle_margin')}, "
                f"body_margin={info.get('min_body_obstacle_margin')}, "
                f"status={info['status']}, message={info['message']}",
                throttle_duration_sec=1.0,
            )

        if command_usable and not bool(info.get("tracking_ok", ok)):
            self.get_logger().warn(
                "路径 NMPC 正在滚动跟踪: "
                f"terminal_error={info['terminal_error']:.4f}, "
                f"min_margin={info.get('min_obstacle_margin')}, "
                f"body_margin={info.get('min_body_obstacle_margin')}",
                throttle_duration_sec=2.0,
            )

        msg = JointState()
        if self.path_time_tracking_started and self.path_time_start_sec is not None and not self.advance_on_feedback:
            self.t = self._tracking_elapsed_sec()
        msg.header.stamp = self._trajectory_stamp(self.t)
        msg.name = JOINT_NAMES
        msg.position = self.q_last.tolist()
        msg.velocity = dq_cmd.tolist()
        t_pub = time.perf_counter() if self.latency_profiling else None
        self.command_pub.publish(msg)
        if (
            self.latency_profiling
            and t0 is not None
            and t_pub is not None
            and feedback_time is not None
        ):
            self._record_latency_sample(t0, t_pub, feedback_time)

        self.actual_trail.append(actual_pos.copy())
        if len(self.actual_trail) > MAX_TRAIL_POINTS:
            self.actual_trail.pop(0)
        self._marker_tick_count += 1
        if self._marker_stride and self._marker_tick_count % self._marker_stride == 0:
            self._publish_markers(targets[0], actual_pos, q_current)
        if self.metrics_log_sec > 0.0:
            self.get_logger().info(
                f"[metrics] mode={self.path_mode} "
                f"terminal_error={info['terminal_error']:.4f} "
                f"manip={manip:.6f} "
                f"min_margin={info.get('min_obstacle_margin')} "
                f"body_margin={info.get('min_body_obstacle_margin')} "
                f"violation={info.get('obstacle_violation_max')} "
                f"usable={command_usable} tracking_ok={info.get('tracking_ok')}",
                throttle_duration_sec=self.metrics_log_sec,
            )
        if self.advance_on_feedback:
            self.t += self.dt

    def _publish_markers(self, current_target: np.ndarray, actual_pos: np.ndarray, q_actual: np.ndarray):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        for i, disk in enumerate(self.mpc_ik.obstacle_disks()):
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
            marker.scale.x = marker.scale.y = diameter
            marker.scale.z = 2.0 * float(self.obstacle_disk_half_thickness)
            marker.color.r = 1.0
            marker.color.g = 0.15
            marker.color.b = 0.05
            marker.color.a = 0.25
            markers.markers.append(marker)

        for i, box in enumerate(self.mpc_ik.body_obstacle_boxes()):
            marker = Marker()
            marker.header.frame_id = BASE_FRAME
            marker.header.stamp = now
            marker.ns = "body_obstacle_boxes"
            marker.id = 500 + i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(box[0])
            marker.pose.position.y = float(box[1])
            marker.pose.position.z = float(box[2])
            marker.pose.orientation.w = 1.0
            expanded_half = box[3:] + self.body_obstacle_margin + self.capsule_radius
            marker.scale.x = float(2.0 * expanded_half[0])
            marker.scale.y = float(2.0 * expanded_half[1])
            marker.scale.z = float(2.0 * expanded_half[2])
            marker.color.r = 0.7
            marker.color.g = 0.15
            marker.color.b = 1.0
            marker.color.a = 0.20
            markers.markers.append(marker)

        markers.markers.append(self._line_marker("expected_path", 1000, self.path, (0.0, 0.9, 0.15, 0.9), 0.01, now))
        markers.markers.append(self._line_marker("actual_path", 1001, self.actual_trail, (0.1, 0.45, 1.0, 0.95), 0.012, now))
        markers.markers.append(self._sphere_marker("current_target", 1100, current_target, (0.0, 1.0, 0.2, 1.0), 0.045, now))
        markers.markers.append(self._sphere_marker("current_actual", 1101, actual_pos, (0.1, 0.45, 1.0, 1.0), 0.04, now))

        # 逐采样点画球:每个点用它自己的避障半径(关节粗/连杆细),SPHERE_LIST 只能统一 scale,故用独立球。
        capsule_samples = self.mpc_ik.capsule_sample_positions(q_actual)
        capsule_radii = self.mpc_ik.capsule_sample_radii()
        for idx, (p, r) in enumerate(zip(capsule_samples, capsule_radii)):
            sphere = Marker()
            sphere.header.frame_id = BASE_FRAME
            sphere.header.stamp = now
            sphere.ns = "capsule_samples"
            sphere.id = 1500 + idx
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(p[0])
            sphere.pose.position.y = float(p[1])
            sphere.pose.position.z = float(p[2])
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = float(2.0 * r)
            sphere.color.r = 0.1
            sphere.color.g = 0.8
            sphere.color.b = 1.0
            sphere.color.a = 0.25
            markers.markers.append(sphere)
        self.marker_pub.publish(markers)

    def _line_marker(self, ns, marker_id, points, color, width, stamp):
        marker = Marker()
        marker.header.frame_id = BASE_FRAME
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(width)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]
        return marker

    def _sphere_marker(self, ns, marker_id, point, color, diameter, stamp):
        marker = Marker()
        marker.header.frame_id = BASE_FRAME
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.pose.position.z = float(point[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = float(diameter)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

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
    node = MoveLL100LH420NmpcDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
