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
    # 直接当脚本跑时把【包的父目录】放进 sys.path。2026-08-04 目录分层后
    # 模块下沉到 core/ sim/ hw/，原来写死的 package_dir.parent 会指到
    # task_space_smc/ 本身（少上溯一级），import task_space_smc.* 全部失败。
    # 改成逐级上溯直到走出带 __init__.py 的目录，对任意层深都成立。
    _here = Path(__file__).resolve().parent
    _root = _here
    while (_root / "__init__.py").exists():
        _root = _root.parent
    sys.path = [p for p in sys.path if Path(p or ".").resolve() != _here]
    sys.path.insert(0, str(_root))

from task_space_smc.core.actuator_adapter import (
    ActuatorAdapter, ActuatorLimits, uniform_acceleration_torque_scale)
from task_space_smc.core.approach_reference import TaskSpaceApproach
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


# CSV 每多少帧 flush 一次。见 _write_row 里的注释。
METRICS_FLUSH_INTERVAL = 1000


def tune_gc_for_realtime():
    """把分代 GC 调成不会在控制周期里造成停顿。

    症状（第 20 次上机实测，hw05，已修掉可视化标记之后）：剩余 49 次丢拍
    随时间单调增长，每 10 s 分别是 3/3/2/10/13/18，但不绑定任何 stride。
    机制：运行器每帧往 ``error_samples`` 和 ``actual_path_segments`` 追加
    numpy 数组，堆里的对象数线性增长，Python 分代 GC 的第 2 代回收要遍历
    全部存活对象，于是停顿时间随运行时长增长。

    做法：``gc.freeze()`` 把【初始化阶段】产生的对象（pinocchio 模型、ROS 节点、
    配置字典等，全都是整个运行期存活的）移出分代统计，第 2 代不再遍历它们。

    ⚠️【2026-08-04 实测否定：不要抬高 gc 阈值】
    第 21 次上机试过 ``gc.set_threshold(14000, 50, 50)``（默认 700/10/10），
    结果【显著变差】：
        丢拍 49 次 (0.41%) -> 245 次 (2.20%)，最大间隙 21.3 ms -> 63.1 ms，
        并且 joint6 冲到 −7.31 rad/s 触发看门狗。
    机制：抬高阈值让回收变【稀疏但每次更长】—— 第 0 代要扫的对象多了 20 倍。
    实时回路要的是"频繁而短"，不是"稀疏而长"。63 ms 的停顿等于控制器瞎 12 拍，
    恢复时一次性大修正把腕部甩出去。

    ⚠️ 也不要用 ``gc.disable()``：控制律里有循环引用（numpy 视图、ROS 消息），
    彻底关掉会让内存持续增长，长时间运行更糟。

    必须在【初始化完成后、控制回路启动前】调用。
    """
    import gc
    gc.collect()
    gc.freeze()


# 单条已走轨迹折线最多发布多少个点。见 _publish_runtime_markers 的 docstring：
# 实测每个 ROS Point 约 0.75 µs，600 点 -> 约 0.45 ms，在 5 ms 控制周期里可接受，
# 而且【不再随时间增长】。600 点足够看清一条 25 s 周期的轨迹形状。
MAX_PUBLISHED_PATH_POINTS = 600


def decimate_path(points, max_points: int):
    """把路径均匀抽稀到 ``max_points`` 以内，保留首尾。

    用抽稀而不是截断：截断会丢掉早期轨迹，而可视化要的正是整体走向。
    抽稀后可视范围不变，只是分辨率降低。

    ``max_points < 2`` 无意义（折线至少要两个点），按 2 处理。
    """
    n = len(points)
    limit = max(2, int(max_points))
    if n <= limit:
        return points
    # 均匀取 limit-1 个点再补上最后一个，保证首尾都在
    idx = np.linspace(0, n - 1, limit - 1).astype(int)
    out = [points[i] for i in idx]
    if idx[-1] != n - 1:
        out.append(points[n - 1])
    return out


def trajectory_period(shape: str) -> float:
    """该形状跑一个完整周期的时长（秒）。"""
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
        self.declare_parameter("initial_joint_positions", [0.0] * 6)

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
            # duration_sec=0 表示"跑一个完整周期"，周期【按形状查】。
            # 原来写死 else 20.0 —— 加了 hwtest（周期 12.57s）之后那个 20
            # 既不是它的周期也不是整数圈，会停在 1.6 圈上。
            self.duration_sec = trajectory_period(self.shape)
        self.data_dir = Path(str(self.get_parameter("data_dir").value))
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        urdf_path = self.get_parameter("urdf_path").value or None
        # 末端 frame 显式来自配置，不靠自动探测。
        #
        # 自动探测的规则是"最后一个【驱动】关节的子连杆"。arm6_tool.urdf 把
        # joint7 改成了 fixed，于是探测结果是 link6 —— 恰好是必须避开的那个
        # 点：末端落在腕转轴上时 q6 转动不移动末端，只贡献 5D 任务的 2 行姿态
        # 行，位置行拿不到它，接近奇异时掉秩。取笔尖 link7 则 q6 有 91mm 力臂。
        # 实测沿轨迹 100 点：6D 姿态可达 43% -> 100%，5D 的 σ最差 0.0082 -> 0.0194。
        ee_frame = self.config.get("ee_frame") or None
        self.model = FloatingBaseArmModel(
            urdf_path=urdf_path, ee_frame_name=ee_frame)
        self.get_logger().info(
            f"model: {Path(self.model.urdf_path).name} "
            f"n_arm={self.model.n_arm} ee={self.model.ee_frame_name}")
        self.controller = controller_from_config(self.config)
        # 关节数由模型推导（nv-6），不写死 —— 6 关节实物没有 joint7。
        self.n_arm = self.model.n_arm
        self.arm_joint_names = list(self.model.arm_joint_names)
        # 6 关节做 6D 任务时零空间秩为 0，投影恒为零；配置里
        # nullspace.enabled=false 时这些键不存在，传中性值即可。
        _ns = self.config.get("nullspace") or {}
        _ns_on = bool(_ns.get("enabled", True)) and "q_nominal" in _ns
        # q_nominal 支持字符串哨兵 "track_start"：解析成【本次 IK 解出的轨迹
        # 起点位形】，在 _solve_initial_pose 之后回填（见下方回填处）。
        #
        # ⚠️ 为什么需要它：写死的 q_nominal 一旦与实际工作位形不符，零空间姿态
        # 项就变成一个持续的扰动源，全部压在冗余方向最"自由"的那个关节上。
        # 2026-08-03 第 9 次上机实测：配置里那组 7 关节时代的旧值与 hwtest 轨迹
        # 起点在 joint4 上差 105°，在交接位形处向 joint6 注入 -4.63 rad/s²
        # （其余关节均 <0.8），持续 0.41 s 即 -1.9 rad/s 漂移 —— 任务项一路对抗
        # 它，对抗就是振荡，joint6 最终超速触发看门狗。改成轨迹起点位形后
        # 注入量降 19 倍（范数 4.75 -> 0.25）。
        self._ns_q_nominal_is_track_start = (
            _ns_on and isinstance(_ns.get("q_nominal"), str)
            and _ns["q_nominal"].strip() == "track_start")
        if self._ns_q_nominal_is_track_start:
            self.ns_q_nominal = np.zeros(self.n_arm)      # 稍后回填
        else:
            self.ns_q_nominal = (np.asarray(_ns["q_nominal"], dtype=float)
                                 if _ns_on else np.zeros(self.n_arm))
        self.ns_kp = (np.asarray(_ns["kp"], dtype=float)
                      if _ns_on else np.zeros(self.n_arm))
        self.ns_kd = (np.asarray(_ns["kd"], dtype=float)
                      if _ns_on else np.zeros(self.n_arm))
        self.ns_accel_weight = (_ns.get("joint_acceleration_weight")
                                if _ns_on else None)
        # 关节限位取自模型（[7:] 是 FreeFlyer 在 nq 中的偏移，不是关节数）
        self.q_lower = self.model.model.lowerPositionLimit[7:].copy()
        self.q_upper = self.model.model.upperPositionLimit[7:].copy()
        _task = self.config.get("task", {})
        self.orientation_mode = str(_task.get("orientation_mode", "full"))
        self.pen_axis_local = np.asarray(
            _task.get("pen_axis_local", [1.0, 0.0, 0.0]), dtype=float)
        _la = _task.get("limit_avoidance") or {}
        self.limit_avoidance = None
        if float(_la.get("gain", 0.0)) > 0.0:
            self.limit_avoidance = LimitAvoidance(
                lower=np.asarray(self.q_lower, dtype=float),
                upper=np.asarray(self.q_upper, dtype=float),
                gain=float(_la["gain"]),
                activate_rad=float(_la.get("activate_rad", 0.5)))
        # 零空间在 6 关节 6D 任务下秩为 0、已停用，nullspace.q_nominal 不再存在。
        # 起始位形优先取 initial_pose_rad；退化到零位是危险的，
        # 其雅可比 sigma_min = 0.00001（cond 326263），起步即奇异。
        _init = self.config.get("initial_pose_rad")
        if _init is None:
            _init = self.config.get("nullspace", {}).get("q_nominal")
            if isinstance(_init, str):        # "track_start" 哨兵不能当初始位形
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
        # 【真值 vs 控制器看到的那一份】
        #
        # self.q / self.qd / self.feedback_effort 是 plant 真值（shared-chain
        # 模式下就是 /joint_states 送回来的原始量；internal 模式下是本节点自己
        # 积分出来的状态）。self.q_ctrl / self.qd_ctrl / self.effort_ctrl 是经过
        # 反馈通路模型（传输延迟 + 速度额外滞后 + 位置量化）之后、**控制器实际
        # 看到**的那一份，由 _refresh_controller_feedback() 每个控制周期刷新。
        #
        # 反馈模型关闭时三者是**同一个对象**（不是拷贝），逐位直通 ——
        # 没写那几个键的配置行为逐字节不变。
        #
        # 分工（详见 loop_delay.py 模块头「建模位置」一节）：
        #   控制律 / 看门狗 / CSV 的 joint_* 列 / 发回电机的位置回显  -> _ctrl
        #   plant 积分 / plant 侧摩擦 / pos_error 等评价指标           -> 真值
        self.q_ctrl = self.q
        self.qd_ctrl = self.qd
        self.effort_ctrl = self.feedback_effort
        self.start_time = None
        self.last_update_time = None
        self._watchdog_tripped = False
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

        # 【可行性盒】：把 plant 的两个运动学硬夹告诉控制器。
        # torque_sim_arm.cc:306 逐轴夹速度、:309-314 夹位置，控制器原先
        # 完全不知道，于是命令一个必被夹掉的加速度、方向被转掉。
        # 配 limits.joint_velocity 后 resolve_acceleration 会做统一标量缩放。
        _limits_cfg = self.config.get("limits", {})
        _joint_velocity = _limits_cfg.get("joint_velocity")
        self.feasibility_velocity = (
            np.asarray(_joint_velocity, dtype=float).reshape(self.n_arm)
            if _joint_velocity is not None else None)
        self.feasibility_brake_safety = float(
            _limits_cfg.get("joint_brake_safety", 0.0))
        self.feasibility_torque = np.asarray(
            _limits_cfg["torque"], dtype=float).reshape(self.n_arm)
        # per_axis = 改动前的行为（ActuatorAdapter 的 np.clip）
        # uniform  = 只缩放加速度需求、全额保留偏置补偿，保持力矩方向
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
        # 就位段用哪套控制器。2026-08-04 新增 task_space：
        #   joint_space（缺省）关节空间五次插值 + 关节 PD + RNEA。已验证基线，
        #                     不写这一项时行为逐位不变。
        #   task_space        末端空间直线+测地线插值，用与跟踪段【完全相同】的
        #                     Lagrange-SMC 执行。实物默认应当用它，理由见
        #                     core/approach_reference.py 模块头。
        self.approach_mode = str(approach.get("mode", "joint_space")).strip().lower()
        if self.approach_mode not in ("joint_space", "task_space"):
            raise ValueError(
                f"approach.mode = {self.approach_mode!r} 不合法，"
                "可选 ['joint_space', 'task_space']")
        # 末端直线的工作台护栏。缺省值是本台实测：桌面 z=0 与基座安装面同平面，
        # x < DESK_EDGE_X 的一侧有桌面，围栏沿 −x 延伸到 y≈0.40 m、高 0.331 m。
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
        # 就位轨迹在【第一个 approach 拍】才建 —— 硬件模式下 __init__ 里 self.q
        # 还是配置的 initial_pose_rad，不是实物反馈（回调不可能在 __init__ 返回
        # 前触发）。用那个假位形规划出来的直线是错的。
        self.task_approach = None
        self.task_approach_rejected = False
        self.approach_friction = self._build_approach_friction(approach)
        # 跟踪段静摩擦前馈。复用 approach 的同一组实测静摩擦（标定文件是同一份），
        # 只有 enabled/scale/torque_epsilon 独立配置 —— 两段的驱动信号不同，
        # 拐点量纲也不同（approach 是 rad/s，跟踪段是 Nm），不能共用一个数。
        _tf = dict((self.config.get("tracking") or {}).get(
            "friction_feedforward", {}) or {})
        # ⚠️【2026-08-04】跟踪段【必须】能用自己的一组摩擦值，不能沿用 approach 的静摩擦。
        # approach 是从静止起步 -> 要克服【静】摩擦；跟踪段关节一直在动 -> 面对的是
        # 【动】摩擦，而动摩擦只有静摩擦的 18~86%。
        # 第 18 次上机数据辨识：
        #     实测启动力矩 [0.706, 0.767, 0.781, 0.077, 0.064, 0.127] Nm
        #     标定 stiction [0.725, 1.775, 0.625, 0.125, 0.100, 0.100] Nm  <- j2 高 2.3 倍
        #     运动中动摩擦 [0.365, 0.369, 0.538, 0.034, 0.018, 0.022] Nm
        # 用静摩擦值做跟踪段前馈 => 折合有效 scale j2=1.85 / j4=1.29 / j5=1.24，
        # 【越过了模块的 scale<1 硬约束】，正是 docstring 设计选择 3 说的自激条件。
        # 实测后果：末端 1~3 Hz 带内抖动 0.90 mm RMS，而参考轨迹同带只有 0.052 mm。
        # 没写 stiction_nm 时仍回落到 approach 那一组（保持既有行为）。
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
        # 关节速度反馈低通。默认全零 = 直通，已有基线行为逐字节不变。
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
        # 交接偏置衰减时长。0 = 关闭（默认），已有仿真基线行为不变。
        handover_cfg = self.config.get("handover", {}) or {}
        self.handover_blend = HandoverBlend(
            float(handover_cfg.get("reference_blend_sec", 0.0)))
        # 【拉入期间冻结轨迹时钟】。见 handover_blend.trajectory_clock 的说明：
        # 开启后 t<blend_sec 期间轨迹停在起点，t>=blend_sec 起轨迹时钟才从 0 走。
        #
        # ⚠️ 代码侧的缺省是 **False**，不是 True。理由：
        # smc_params_hwtest_simbest.yaml 是【已验证基线】且 reference_blend_sec=3.0，
        # 若代码缺省为 True，它在不改一个字的情况下行为就变了（轨迹整体推迟 3 s）。
        # 缺省取 False 保证"没写这个键的配置逐位不变"，需要冻结的配置显式写 true
        # （smc_params_hwtest.yaml 已写）。
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
            # 硬件模式：起始位形来自【实物反馈】，不能像仿真那样随便设。
            # 但 track_start_q 仍必须解到【轨迹起点】——
            #
            # ⚠️ 这里原来是 track_start_q = self.q.copy()，即 approach 段位移为零、
            # 机械臂原地不动，等 start_delay 一到直接使能跟踪。那时末端距参考起点
            # 可达 144 mm（实测），SMC(λ=72,k=102) 会立刻索要巨大力矩 —— 在实机上
            # 就是一次猛冲。approach 段存在的全部意义就是避免这个。
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
            # approach 段诊断。2026-08-03 之前 approach 段一行都不记，末端误差
            # 只能事后从串口抓包反推，而抓包里的位置字段又是实测值的回显
            # （_publish_motor_command 的 msg.position = self.q），等于无从查证。
            + [f"approach_q_des_{i + 1}_rad" for i in range(self.n_arm)]
            + [f"approach_q_err_{i + 1}_rad" for i in range(self.n_arm)]
            # ------------------------------------------------------------
            # 估计器【入口】的两列 + 它用的加速度。出口是既有的
            # joint_N_rad / joint_N_velocity_rad_s（语义不变 = 控制器看到的
            # 那一份）。入口/出口成对是最小充分集，事后可算的量：
            #
            #  * 【核心判据】互相关滞后：xcorr(joint_N_qd_meas_rad_s,
            #    joint_N_velocity_rad_s) 的峰值拍数 = 估计器相对电机自报把速度
            #    提前了几拍。路线 A 必须精确为 0（两列逐位相同 —— 这是这套日志
            #    的自检项）；B/C/D 应为负。另有一路不依赖任何真值的独立复核：
            #    拿 diff(joint_N_q_meas_rad)/dt 当零偏参考微分器再做一次。
            #  * 估计噪声：两列各做 >5 Hz 高通（本轨迹 5 Hz 以上无内容）取 std。
            #  * 预测量偏差：joint_N_rad − joint_N_q_meas_rad 就是位置预测增量
            #    本身，不需要额外列。
            #  * joint_N_est_acc_rad_s2（LESO 的 z3）是【摩擦/扰动残余的在线
            #    估计】。今晚误差分解里最大的一项就是动摩擦残余 2.429 mm，
            #    这一列是第一次能逐帧看到它，而不是靠闭环反解。非 LESO 关节写
            #    NaN —— 那里根本没估这个量，写 0 会被读成"扰动为零"。
            #
            # ⚠️ 列是【无条件】加的，不随配置开关变。配置相关的 CSV schema 会
            # 让事后脚本必须分支，比多几列贵得多。
            # 成本实测：DictWriter 边际 0.206 µs/列/帧，20 列 = 4.1 µs/帧
            # = 5 ms 预算的 0.08%。对照今晚真正造成丢拍的两件事（可视化标记每拍
            # 重建折线、每 50 帧 flush），那是数百微秒到毫秒量级 —— 不在一个
            # 数量级上。
            # ------------------------------------------------------------
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
                # 该帧实际喂给 reference_sample 的轨迹时间。冻结开启时它在
                # [0, blend_sec) 恒为 0、之后是 t-blend_sec；关闭时恒等于 t。
                # 事后核对 desired_* 用的到底是哪一时刻的轨迹，只看这一列即可。
                "trajectory_clock",
                # 估计器的两个【累计】计数。选累计而非每帧布尔：clamp / hold 是
                # "发生了就必须知道"的稀有事件，计数列的单调性一眼可读，每帧
                # 布尔要扫全表。⚠️ clamp_hits 一直在涨 = 预测量在被夹，说明配置
                # 里写的 predict_*_sec 并没有真正生效。
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

        # 仿真侧回路延迟。实测实物【位置路 10 ms / 速度路 25 ms】（见
        # loop_delay.py 模块头），而仿真 plant 是 step_on_effort_command 锁步、
        # 零延迟 —— 不建模就永远复现不出腕部失稳。
        #
        # 分段模型建在【反馈侧】：位置路与速度路滞后不同，指令侧只有一个 τ，
        # 区分不了。旧的 loop_delay_sec（指令侧单块）仍能用，两者同写以分段为准。
        # ⚠️ 只在【非 hardware】模式生效：实物自带这些延迟与量化，叠加即双重计入。
        _plan = resolve_simulation_delay(
            self.config.get("simulation"), self.control_period, self.mode)
        self.command_delay = _plan.command_delay
        self.feedback_delay = _plan.feedback
        for _msg in _plan.messages:
            self.get_logger().info(_msg)
        for _msg in _plan.warnings:
            self.get_logger().warning("⚠️ " + _msg)

        # ------------------------------------------------------------------
        # 关节状态估计器（换速度来源 / Smith 预测）。
        #
        # 位置：**FeedbackDelayModel 的【下游】**，见 state_estimator.py 模块头。
        # 一句话：放上游的话，仿真里估计器会看到无延迟、无量化的完美真值，
        # 与实物不同构 —— 等于让仿真替方案作弊。串联到下游之后，仿真与实物
        # 走【同一条代码路径】，估计器内部一个 mode 分支都不需要。
        #
        # ⚠️ 与 feedback_delay 的门控方向【相反】：延迟模型只在非 hardware 生效
        # （实物自带延迟，叠加即双重计入）；估计器两种模式都生效。
        #
        # ⚠️ 缺省全关 = 逐位直通。没写 estimator 段的配置（whole6 /
        # hwtest_simbest）行为逐字节不变，且 `q_ctrl is self.q` 的别名关系保留。
        # ------------------------------------------------------------------
        _est_plan = resolve_estimator(
            self.config.get("estimator"), self.n_arm, self.control_period,
            self.mode, feedback_delay_enabled=self.feedback_delay.enabled)
        self.estimator = _est_plan.estimator
        for _msg in _est_plan.messages:
            self.get_logger().info(_msg)
        for _msg in _est_plan.warnings:
            self.get_logger().warning("⚠️ " + _msg)
        # 估计器【入口】的那一份（= 仿真里延迟量化后 / 实物里裸 /joint_states）。
        # CSV 记入口与出口两端，事后才能算互相关滞后与噪声对比 —— 只记一端的话，
        # "速度路是否真的提前了 3 拍"这个判别实验的核心问题就答不了。
        self._q_meas = self.q
        self._qd_meas = self.qd
        # 估计器自己的时间基准。不复用 self.last_update_time：那个量在
        # approach / 跟踪两条分支里的更新时机不同，而观测器要的是"距上次
        # 推进观测器过了多久"，两者在提前 return 的周期上会分叉。
        self._estimator_last_time = None
        # __init__ 里 self.q 被 _solve_initial_pose 重新绑定过，而 q_ctrl 是在那
        # 之前设的别名 —— 先同步一次，免得"控制循环开跑前有人读 q_ctrl"时拿到
        # 旧数组。延迟管道里因此多一份起步位形，只影响预热的前几拍（管道本来
        # 就保持起步位形），稳态滞后不变。
        self._refresh_controller_feedback()
        # 初始化产生的长寿对象在这一刻全部就位，冻结它们再起控制回路。
        # 放在 create_timer 之前 —— 之后就可能已经在跑控制了。
        tune_gc_for_realtime()
        self.timer = self.create_timer(period, self._control_loop)
        self.get_logger().info(
            f"Whole-task SMC runner mode={self.mode}, shape={self.shape}, "
            f"start_delay={self.start_delay_sec:.3f}s, duration={self.duration_sec:.3f}s, "
            f"config={self.config_path}, metrics={self.metrics_path}"
        )
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
        """墙钟时间 -> 该帧实际喂给 ``reference_sample`` 的轨迹时间。

        关闭冻结时是恒等映射，逐位等价于改动前的 ``reference_sample(t)``。
        推导与代价见 ``handover_blend.trajectory_clock``。
        """
        return trajectory_clock(
            t,
            self.handover_blend.duration_sec,
            self.freeze_trajectory_during_blend,
        )

    def _solve_initial_pose(self, q_seed: np.ndarray) -> np.ndarray:
        """解出轨迹起点的关节位形。

        两处必须与主控制律【口径一致】，否则解出来的分支控制器根本不会走：

        1. 【用 5D 笔轴误差，不是 6D 完整姿态】。任务本身放弃了绕笔轴的自转
           （orientation_mode=pen_axis），这里若按 6D 解就多约束了一维，会
           把解逼到劣质分支。实测从近零位形出发，6D 解要求 joint6 转 229°
           且路径上限位余量降到 0.000；换 5D 多起点后余量 1.176 rad。

        2. 【多起点并按限位余量选分支】。单用当前位形做种子只会收敛到最近的
           局部解，而"最近"不等于"能走"。approach 段是开环插值过去的，
           路径上任何一点触限都会在实机上出事。
        """
        if getattr(self, "simulated_base", False):
            bs = self._simulation_base_sample(0.0)
        else:
            bs = base_sample(0.0, self.shape, 0.0, self.derivative_dt)
        ref = reference_sample(0.0, self.shape, self.derivative_dt)
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
            score = margin - 0.15 * disp          # 余量优先，位移次之
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
        # 关节名取自【模型】而非写死的 7 关节表。6 关节的 /joint_states 里
        # 没有 joint7，用 DEFAULT_ARM_JOINT_NAMES 会在第 7 次迭代直接 return，
        # feedback_generation 永不递增 —— runner 认为「没有新反馈」而不发指令，
        # plant 又要收到指令才步进，两边死锁。RViz 里表现为模型散开
        # （TF 只在控制循环里发布，循环没跑就没有 world 帧）。
        for j, name in enumerate(self.arm_joint_names):
            if name not in name_to_index:
                return
            i = name_to_index[name]
            self.q[j] = msg.position[i]
            self._qd_raw[j] = msg.velocity[i] if i < len(msg.velocity) else 0.0
            self.feedback_effort[j] = (
                msg.effort[i] if i < len(msg.effort) else 0.0)
        # 速度低通放在这里（而非控制循环）：反馈到达即滤，dt 用名义控制周期。
        # 未启用时 JointVelocityFilter 是逐位直通。
        self.qd = self.velocity_filter(self._qd_raw, self.control_period)
        self.last_joint_stamp = self.get_clock().now().nanoseconds * 1e-9
        if not self.has_joint_feedback:
            # ⚠️ 第一帧【真实】反馈到达：把估计器的历史整个丢掉重新起算。
            # __init__ 里那次 _refresh_controller_feedback 是拿 config 的
            # initial_pose_rad 预热的（hardware 分支不重绑 self.q，回调也不可能
            # 在 __init__ 返回前触发）。不 reset 的话第一拍裸差分给出
            #     qd = (q_实测 − q_配置) / 0.005
            # 实测：起步差 0.1° -> j1 假速度 0.349 rad/s、末端 0.138 m/s；
            #       起步差 1.0° -> 3.491 rad/s，恰好卡在包线 3.6 之下【不触发】。
            # 而本架构 start_delay_sec:=0，第一拍就执行 handover_blend.capture()，
            # 那个假末端速度会被锁进参考量：Δv 的系数是 B(u)·T ≈ 5.9 s，0.1° 的
            # 起步差就外扩 0.82 m 的参考位置 —— 正是第 7/10 次上机的死法。
            # ⚠️ 仿真结构上查不出来：simulation 分支在 __init__ 里已把 self.q 重绑
            # 成 plant 起步位形，预热喂的就是真值，裸差分恒为 0。
            self.estimator.reset()
            self._estimator_last_time = None
        self.has_joint_feedback = True
        self.feedback_generation += 1

    def _estimator_dt(self, now):
        """本拍喂给估计器的采样间隔 [s]。

        口径与控制律保持一致：
          * ``simulated_base``（仿真 plant 是 step_on_effort_command 锁步，一条
            指令走一步）-> 用 ``control_period``。此时墙钟间隔与信号真正推进的
            时间无关，喂墙钟 dt 会让观测器的时间与 plant 的时间对不上。
          * 实物 -> 墙钟差。⚠️ 必须用【实际 dt】而不是标称周期：合成信号实测
            （真速度 0.020 rad/s，一次长拍）
                长拍      跟 dt 变   固定标称 dt
                2×5ms     0.0193     0.0192  rad/s 峰值误差
                3×5ms     0.0192     0.0289
                4×5ms     0.0192     0.0313
            固定 dt 的误差随长拍长度线性增长，跟 dt 变的则平掉。而实测丢拍
            0.08~0.83%、最大间隙 15.4 ms，正落在这个区间。
            夹紧与真掉线判定由 JointLESO 内部做（它的增益是 dt 的闭式函数，
            ZOH 极点 exp(−ω_o·dt) 对任意 dt 恒在 (0,1)，无条件稳定）。
        """
        if self.simulated_base:
            return self.control_period
        if now is None:
            return None
        prev = self._estimator_last_time
        self._estimator_last_time = now
        return None if prev is None else (now - prev)

    def _refresh_controller_feedback(self, now=None):
        """把 plant 真值经反馈通路模型 + 估计器变成【控制器看到的那一份】。

        ⚠️ 必须**每个控制周期恰好调用一次**，而不是放在 _joint_state_cb 里。
        延迟的单位是"拍"，一拍 = 一个控制周期；若按反馈回调推进，回调频率
        与控制频率一有偏差（plant 自走时钟、丢帧、重发），实际滞后的秒数就
        偏离配置值，而这正是本次要测的量。当前 launch 是
        step_on_effort_command 锁步、两者 1:1，但不能依赖这个巧合。

        估计器接在延迟模型的【下游】—— 见 state_estimator.py 模块头。它同样
        必须每周期恰好一次：空转周期（等反馈、等 plant）里同一个 q 被反复喂
        进去，创新量恒为 0，等于把观测器停摆一拍，实际带宽被悄悄压低。

        两者都关闭时全部原样返回入参对象（同一个 ndarray），逐位直通。
        """
        q_d, qd_d, self.effort_ctrl = self.feedback_delay(
            self.q, self.qd, self.feedback_effort)
        self._q_meas, self._qd_meas = q_d, qd_d
        if not self.estimator.enabled:
            # 禁用路径在这里就返回，不去算 dt、不碰 np —— 实测 0.09 µs/拍
            # 靠的就是这一条。
            self.q_ctrl, self.qd_ctrl = q_d, qd_d
            return
        self.q_ctrl, self.qd_ctrl = self.estimator(
            q_d, qd_d, self._estimator_dt(now))

    def _fill_estimator_row(self, row):
        """把估计器的入口两列 + 加速度估计 + 两个累计计数写进 CSV 行。

        approach 与跟踪两段共用 —— approach 段的估计器状态同样要能事后查，
        它是交接位形的来源。
        """
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
        """把本拍【已发出】的指令折算成有效关节加速度，喂给估计器。

        ⚠️ 必须在 ``_publish_motor_command`` 之【后】调用。缓冲里记的是"已经发
        出去"的指令；本拍还没发的指令进了缓冲，预测器就用到了未来信息，在实物
        上表现为凭空多出的超前。

        ⚠️ 为什么用 ``motor.command`` 反解，而不是直接拿 ``torque_scale·rr.qdd_cmd``：
        后者只在 ``limits.torque_saturation: uniform`` 下精确。hwtest 用的是缺省
        的 ``per_axis`` —— 那条路上 torque_scale 恒为 1.0，饱和由 ActuatorAdapter
        逐轴 clip（外加力矩变化率限幅）完成，rr.qdd_cmd 完全不知道。于是预测器
        会以为机械臂在按未饱和的指令走，**而饱和恰恰发生在误差最大、最需要预测
        正确的时刻**。从 motor.command 反解则两种模式都对。
            τ = M·q̈_cmd + h，τ_bias = h  =>  q̈_eff = M⁻¹(τ_cmd − τ_bias − τ_ff)

        ⚠️ 为什么要减掉 ``tau_extra``（静摩擦前馈）：它按设计是去抵消真实摩擦
        的，净加速度贡献理想为零；算进去等于预测一段不会发生的运动。它抵消不
        完的那部分残差已经包含在等效扰动 d_eff=0.411 m/s² 里了
        （j2 侧 τ_ff−τ_fric ≤ 0.65 Nm -> 3.33 rad/s² -> 25 ms 内 1.04 mrad
          ≈ 末端 0.31 mm，恰是限幅 0.005 rad 的 21%，说明限幅不会误伤合法修正）。

        ``wants_commanded_acceleration`` 为 False（缺省配置、路线 A/B/C-none）
        时整段跳过 —— 一次 RNEA + CRBA + 6×6 solve 约 5.5 µs 全都不花。
        """
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
            # 惯量矩阵奇异本身就是别处的故障；这里宁可少补一拍，也不要把
            # NaN 送进预测缓冲（那会一直污染到 horizon 走完为止）。
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

    def _inj_vec(self, *keys, default, n=None):
        # n 默认取关节数。注意本文件里的 [7:] 是 FreeFlyer 在 nq 中的偏移
        # （base 占 7 维），不是关节数，那些必须保留。
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
        """发布运行时可视化标记。

        ⚠️【2026-08-04 修复：这段曾经把控制周期挤爆】
        原实现每 visualization_stride 拍（30 Hz 可视化 / 200 Hz 控制 = 每 7 拍）
        就把【整条】已走轨迹重建成 LINE_STRIP 发一次，而轨迹点数每次 +1、
        没有上限。实测每个 ROS Point 约 0.75 µs，60 s 累计 1660 点时单次重建
        约 3.1 ms，加 DDS 序列化占掉 5 ms 预算的一大半。

        第 19 次上机实测（hw04，60 s）：控制周期中位 5.002 ms 但 p99 = 10.014 ms
        —— 整拍丢失，124 次（1.07%）。丢拍数随时间单调增长：0-10 s 零次，
        最后 5 s 有 52 次。且【52.4% 的丢拍落在 idx%7==0 上，随机只应 14.3%】
        （stride 取 6/8 无富集，21%/16.7% 与 10.5%/12.5%）。
        操作者的观察是"一顿一顿的移动，像发信号发一下停一下"—— 就是这个。

        两道防线：
        * 无订阅者直接返回。rviz:=false 时这整段是纯浪费。
        * 有订阅者时对路径【均匀抽稀】到上限，成本封顶而不改变可视范围
          （抽稀保留首尾与整体走向，截断则会丢掉早期轨迹）。
        """
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
        """运行时安全看门狗：越界就【立即停并保持位置】。

        为什么需要它：上机前我做的全是【运动学】校验（限位余量、连杆高度、
        围栏），验不出硬件模式特有的代码路径。实测有一次 approach 段在硬件
        模式下根本不驱动机械臂（只发可视化），跟踪一使能就全力冲 ——
        关节速度 28.17 rad/s（上限 3/5）、力矩饱和 100%、j4 冲到 233°，
        靠操作者断电才停住。

        判据取【物理量】而不是精度指标：速度超包线 1.2 倍、或位置误差超
        50 mm。这两条在正常运行时都不可能触发（实测仿真峰值 0.03 mm、
        速度占包线不到 30%），一旦触发必定是控制链出了结构性问题。
        """
        if self._watchdog_tripped:
            return True
        vlim = np.asarray(
            (self.config.get("limits") or {}).get(
                "joint_velocity", [3.0, 3.0, 3.0, 5.0, 5.0, 5.0]),
            dtype=float).reshape(self.n_arm)
        reasons = []
        # 用【测量速度】而不是真值：实物上看门狗手里就只有 /joint_states 回来的
        # 那一份。速度路滞后 25 ms 意味着仿真里的看门狗也要晚 25 ms 才看到超速
        # —— 这个滞后本身就是要复现的现象之一，给它真值会让仿真的看门狗比实机
        # 灵敏，触发时刻对不上。延迟模型关闭时 _qd_meas is self.qd。
        #
        # ⚠️⚠️ 这里必须是 self._qd_meas，【不是】self.qd_ctrl。2026-08-04 三个
        # 独立评审同时指出：估计器接进来之后 qd_ctrl 变成 FD/LESO/Smith 的输出，
        # 看门狗就成了"被测对象自己判自己"。本机没有软件急停通道（协议无使能/
        # 失能帧），这是唯一的自动兜底，不能有第二个失效点。三条已复现的失效路径：
        #   1) hold_on_invalid（缺省 True）：反馈出现 NaN 时估计器返回上一拍的
        #      拷贝，看门狗从此看到一份【冻结的健康速度】且不自愈；实测 j4 真实
        #      12.0 rad/s（阈值 6.0）时看门狗读到 0.000 rad/s。同时这也绕开了
        #      actuator_adapter 里"指令含 NaN 就归零"的既有保护。
        #   2) Smith 限幅 ±[0.2,0.2,0.2,0.5,0.5,0.5] rad/s 可以往【不安全方向】
        #      修正：实测 j4 真实 5.90 rad/s 被压成 5.40，阈值被偏置 8.3%。
        #   3) leso.py 的 resync（raw_dt > 0.05 s）返回掉线前的 z1 —— 正是 63 ms
        #      GC 停顿那种时刻，看门狗在最该起作用的一拍失明。
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
            self.get_logger().error(
                "⚠️ 安全看门狗触发，停止跟踪并保持位置：" + "；".join(reasons))
        return self._watchdog_tripped

    def _publish_motor_command(self, motor):
        if getattr(self, "_watchdog_tripped", False):
            # 保持位置：位置环用默认增益，力矩前馈清零。
            # 【不能发零力矩】—— MIT 模式下那是彻底失力，机械臂会垮。
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
        # p/d 全零（纯力矩模式），这个位置字段只是【测量值回显】，不构成控制。
        # 用 q_ctrl 与实物一致：实物这里发的就是刚收到的测量值。
        msg.position = np.asarray(self.q_ctrl, dtype=float).tolist()
        msg.velocity = [0.0] * self.n_arm
        # 仿真延迟（旧的指令侧单块模型）：plant 收到的是 N 拍之前的力矩。
        # 分段模型改建在反馈侧，此处 delay_steps=0、逐位直通；实物模式下同样为 0。
        msg.current = np.asarray(
            self.command_delay(motor.command), dtype=float).tolist()
        msg.p = [0.0] * self.n_arm
        msg.d = [0.0] * self.n_arm
        msg.enable = 1 if motor.enabled else 0
        self.command_pub.publish(msg)
        if not self.internal_plant:
            self.last_command_feedback_generation = self.feedback_generation

    def _resolve_track_start_nullspace_nominal(self):
        """把 q_nominal 的 "track_start" 哨兵回填成 IK 解出的轨迹起点位形。

        必须在 _solve_initial_pose 之后调用。非哨兵配置下是无操作，
        已有基线（whole6 写死 6 元素向量）行为逐字节不变。
        """
        if not getattr(self, "_ns_q_nominal_is_track_start", False):
            return
        self.ns_q_nominal = np.asarray(
            self.track_start_q, dtype=float).reshape(self.n_arm).copy()
        self.get_logger().info(
            "零空间姿态目标 q_nominal := 轨迹起点位形 %s°"
            % np.round(np.degrees(self.ns_q_nominal), 2).tolist())

    def _approach_inertia_diagonal(self):
        """当前位形下手臂惯量矩阵的对角 [kgm²]，用于把 kp 折成力矩刚度。

        随位形变化，所以只用于启动时的诊断打印，不进控制律。
        """
        if getattr(self, "simulated_base", False):
            bs = self._simulation_base_sample(0.0)
        else:
            bs = base_sample(0.0, self.shape, 0.0, self.derivative_dt)
        q_full = self.model.make_q_full(
            bs.position_w, bs.quaternion_wxyz, self.q)
        M = np.asarray(pin.crba(self.model.model, self.model.data, q_full))
        return np.diag(M)[-self.n_arm:].copy()

    def _build_approach_friction(self, approach, quiet=False):
        """装配 approach 段的静摩擦前馈。

        默认关闭 —— 只有实机配置（smc_params_hwtest.yaml）显式打开，已有仿真
        基线的行为逐字节不变。静摩擦值优先取配置里的显式覆盖，否则读
        config/arm6_calibration.yaml 的阶段 2 实测值。

        ``quiet=True`` 供跟踪段复用：装配逻辑与静摩擦来源完全相同，但下面那段
        死区诊断是 approach 专有的（依赖 approach.kp 折出的力矩刚度），对跟踪段
        没有意义，打出来只会误导。
        """
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

        # approach.kp 是【加速度增益】(1/s²)，力矩刚度是 M_ii(q)·kp。腕部连杆
        # 惯量只有 0.011 kgm²，直接把 kp 当 Nm/rad 会把刚度高估 80 倍，从而把
        # 死区低估同样的倍数。见 friction_feedforward 模块 docstring。
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
            # 两个仿真对象都没有摩擦：runner 自带积分器走
            # compute_arm_forward_dynamics（只有 armature），ROS 仿真的
            # torque_sim_arm.cc 同样只加载 armature。零误差处前馈为零所以不会
            # 发散，但运动中它是一份没有对手的额外力矩。
            self.get_logger().warn(
                "approach 静摩擦前馈在 mode=%s 下开启 —— 该被控对象【没有摩擦"
                "模型】（internal 走 compute_arm_forward_dynamics，ROS 仿真走 "
                "torque_sim_arm.cc，两者都只加载 armature）。零误差处前馈为零，"
                "所以不会把仿真跑坏（A/B 实测 RMS 4.4 µm 无差别），但这一段仿真"
                "【验证不了这个前馈】—— 它要补偿的东西在仿真里不存在。"
                % self.mode)
        return friction

    def _write_approach_row(self, elapsed, bs, q_des, tau_friction, motor):
        """把 approach 段也记进同一个 CSV。

        2026-08-03 之前这一段一行都不记，CSV 从交接时刻才开始，于是"approach
        到底走到哪、离目标多远"只能事后从串口抓包反推 —— 而抓包里的位置字段
        是实测值的回显（``msg.position = self.q``），根本不含 q_des。那次上机
        因此花了很久才定位到 approach 被静摩擦卡在死区里。

        时间轴用 ``elapsed - start_delay_sec``：approach 段是负的，交接点恰好
        是 t=0，与跟踪段拼起来单调连续。``phase`` 列区分两段。
        """
        q_full = self.model.make_q_full(
            bs.position_w, bs.quaternion_wxyz, self.q)
        v_full = self.model.make_v_full(bs.velocity_lwa, self.qd)
        kin = self.model.compute_kinematics(q_full, v_full)
        ref = reference_sample(0.0, self.shape, self.derivative_dt)
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
            # 上机时要看的关键量：前馈有没有把指令力矩推过静摩擦阈值。
            "approach_tau_friction_max_nm": float(
                np.max(np.abs(tau_friction))),
            "handover_blend": float("nan"),
            "tau_disturbance_max_abs": float("nan"),
            # approach 段的参考量本来就取 reference_sample(0.0)，轨迹时钟即 0。
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
            # joint_* 列记【控制器看到的那一份】，与跟踪段以及实物 CSV 同口径。
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
        """规划末端直线 + 测地线，并过一遍工作台护栏。返回是否放行。

        必须在【第一个 approach 拍】调用，不能在 __init__ 里 —— 硬件模式下
        __init__ 时 self.q 还是配置的 initial_pose_rad，用它规划出来的直线
        和实物实际所在的位置无关。
        """
        # ⚠️ 先查【起点是不是奇异】。任务空间就位要把末端速度经伪逆映射成关节
        # 速度，起点掉秩的话这个映射本身就没有意义 —— 阻尼伪逆会给出一个数，
        # 但那个数与期望的末端运动无关，机械臂会往任意方向窜。
        # 本机的典型情况：上电/失力后机械臂落回休息位形，j5≈0 是【腕部奇异】
        # （URDF 里 joint4 与 joint6 都是 axis="1 0 0"，joint5 是 "0 0 1" 夹在
        # 中间；j5=0 时 j4 与 j6 转轴共线，6D 雅可比掉秩）。
        # 实测 j5=0.17° 时 sigma_min = 0.00019，比 sigma_critical(0.012) 低 63 倍，
        # 条件数 9326。j5=40° 才回到 0.0425（高于 sigma_warn 0.03）。
        # 这一步【不能】由任务空间 SMC 自己完成 —— 它正是被奇异卡住的那一方。
        # 只能先用关节空间的位置模式把 j5 挪开，即 hw/wrist_prepose.py。
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
        ref0 = reference_sample(0.0, self.shape, self.derivative_dt)
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
        # 末端直线 vs 桌面 / 围栏。⚠️ 只查【末端点】—— 连杆本体的扫掠体积没有
        # 查，因为沿途的关节解并不存在（这条轨迹是末端空间规划的，逐点 IK 才
        # 有关节角）。所以这道护栏是必要条件不是充分条件。
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
        """就位段：末端空间直线 + 测地线，用与跟踪段【同一个】SMC 执行。

        与 joint_space 模式的区别只在"力矩怎么算出来"：
          * 规划在末端空间 -> 走的是可以事先检查的直线，不是关节插值扫出来的
            未知曲线；
          * 执行用 self.controller（Lagrange-SMC）+ resolve_acceleration，与跟踪
            段逐字相同 -> 交接处不换控制器；
          * 力矩模式 -> 压得住静摩擦，没有位置模式那 2°(j3)/10°(j2) 的到不了位。
        """
        if getattr(self, "task_approach_rejected", False):
            return                      # 护栏已拒绝：不发力矩，驱动超时保位
        bs = self._simulation_base_sample(0.0)
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
        # ⚠️ 就位段【也要】看门狗。joint_space 模式当年没有，是因为它的关节 PD
        # 增益远低于跟踪段；现在就位用的是跟踪段那一套 SMC 增益，同样能索要
        # 饱和力矩，兜底不能只在跟踪段有。
        # 位置判据喂任务空间误差 —— 参考量从当前位姿起步，正常情况下这个量
        # 始终是跟踪误差量级（mm），一旦跑飞立刻超 50 mm。
        self.last_position_error = float(np.linalg.norm(smc.position_error))
        if self._watchdog_hold_q is None:
            self._watchdog_hold_q = np.asarray(self._q_meas, dtype=float).copy()
        if not self._watchdog_trip(now):
            self._watchdog_hold_q = np.asarray(self._q_meas, dtype=float).copy()
        # CSV 的 approach_q_des_* 是关节空间插值的目标，任务空间模式下不存在
        # 对应量（沿途没有关节解）。写 NaN 而不是拿 q_ctrl 冒充 —— 后者会让
        # approach_q_err_* 恒为 0，看上去"完美跟踪"，是假信号。
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

        bs = self._simulation_base_sample(0.0)
        bs.velocity_lwa = np.zeros(6)
        bs.acceleration_lwa = np.zeros(6)
        self._publish_base_tf(bs)
        self._publish_base_truth(bs)
        # 真值（喂正向动力学 = plant）与控制器看到的那一份（喂 PD + RNEA）分开。
        # 反馈模型关闭时 q_ctrl is self.q，两者是同一个数组，逐位等价于改动前。
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
        # 静摩擦前馈叠加在【逆动力学之后】：它补偿的是关节里的摩擦力矩，
        # 不是一项加速度需求，不能经过 RNEA 的惯量矩阵。
        tau_friction = stiction_feedforward(
            self.approach_friction, qd_des, q_des - self.q_ctrl)
        tau = tau + tau_friction
        self._finish_approach_tick(
            elapsed, now, bs, q_full, v_full, q_full_ctrl, v_full_ctrl,
            tau, q_des, tau_friction)
    def _finish_approach_tick(self, elapsed, now, bs, q_full, v_full,
                              q_full_ctrl, v_full_ctrl, tau, q_des,
                              tau_friction):
        """approach 段的收尾：下发指令、喂估计器、记 CSV、推进内部 plant、
        发可视化。

        2026-08-04 从 ``_run_approach`` 里抽出来，让关节空间与任务空间两种
        就位模式共用同一套收尾动作 —— 两者只在"力矩怎么算出来"上不同，
        记录口径必须完全一致，否则两种模式的 CSV 没法互相对照。
        """
        motor = self.adapter.torque_to_motor_command(
            tau,
            now,
            self.last_joint_stamp or now,
            now,
            self.control_period,
        )
        self._publish_motor_command(motor)
        # ⚠️ approach 段【也要】喂估计器。否则 30 s 的 approach 期间模型输入全是
        # 零，交接那一刻观测器正处在最差状态 —— 而交接恰好是当前 RMS
        # 2.95~3.17 mm 的起点，也是历次上机最脆弱的时刻。
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
            ref = reference_sample(0.0, self.shape, self.derivative_dt)
            desired = PoseState(
                position=ref.position_w,
                quaternion_wxyz=ref.quaternion_wxyz,
                linear_velocity=np.zeros(3),
                angular_velocity=np.zeros(3),
            )
            self._publish_runtime_markers(desired, kin.ee_position)
            self.last_hold_publish_time = now

    def _publish_start_hold_markers(self):
        """只发可视化，不碰关节状态 —— approach 已在驱动机械臂。"""
        if self.simulated_base:
            bs = self._simulation_base_sample(0.0)
        else:
            bs = base_sample(0.0, self.shape, 0.0, self.derivative_dt)
        self._publish_base_tf(bs)

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

    def _feasibility_limits(self, dt):
        """按当前控制周期构造可行性盒；未配置 limits.joint_velocity 时返回 None
        （行为退回改动前）。dt 用【本周期实际步长】—— plant 是 lockstep
        （student_arm_node.cc:307 step_on_effort_command），一条指令走一步。"""
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
        """任务空间参考量 -> 关节力矩。**就位段与跟踪段共用这一条路径。**

        2026-08-04 抽出来的。此前就位段走的是另一套控制器（关节空间五次插值
        + 关节 PD + RNEA，用 approach_kp/kd 那组增益），跟踪段才是 Lagrange-SMC
        —— 交接处等于两套控制器换手，``handover_blend`` 要吸收的不只是跟踪
        残差，还有两套控制器稳态点的差。现在两段共用同一个控制器、同一组增益，
        交接处只剩跟踪误差量级的残差。

        返回 ``(tau, tau_bias, smc, rr, torque_scale, tau_track_friction)``。
        看门狗、饱和记录、指令下发留在各自的调用方 —— 两段的收尾动作不同。
        """
        smc = self.controller.compute(desired, actual)
        b6 = self.model.compute_task_bias_acceleration(q_full_ctrl, v_full_ctrl, bs_est.acceleration_lwa)
        # 5D 任务（位置 + 笔轴方向）把零空间秩从 0 提到 1，那个自由度用于
        # 限位规避。6D 任务下本轨迹只有 35% 可达，5D 下 98%/100%。
        J_task = kin_ctrl.J_arm_lwa
        rhs_task = smc.nu - b6
        if self.orientation_mode == "pen_axis":
            J_task, rhs_task, _ = pen_axis_task(
                J_task, rhs_task, kin_ctrl.ee_rotation, self.pen_axis_local)
        rr = resolve_acceleration(
            J_task,
            self.model.compute_arm_mass_block(q_full_ctrl),
            rhs_task,
            self.q_ctrl,                      # 零空间/限位规避也只有测量值可用
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
            # ⚠️【已停用】饱和感知解算。161 个单元测试全绿，但真实 ROS
            # 8 次重复 RMS 245.7mm（对照：不用它是 2.43mm），坏 100 倍。
            # 单元测试只验了孤立时刻的盒约束，验不出闭环稳定性 ——
            # 这正是"调参与验证必须跑真实 ROS"那条规则要防的情况。
            # 实现保留在 redundancy_resolution.py 里待查，不要直接删。
            feasibility=None,
        )
        tau = self.model.compute_rnea_arm_torque(q_full_ctrl, v_full_ctrl, bs_est.acceleration_lwa, rr.qdd_cmd)
        # 零加速度力矩（重力+科氏）。既是统一缩放的基准，也是静摩擦前馈的驱动量。
        tau_bias = self.model.compute_rnea_arm_torque(
            q_full_ctrl, v_full_ctrl, bs_est.acceleration_lwa,
            np.zeros(self.n_arm))
        # 力矩饱和的处理方式：逐轴限幅（旧）还是保方向的统一缩放（新）。
        # 逐轴限幅在多轴同时饱和时会把解出的加速度打散成 M⁻¹(τ_clip − h)，
        # 与任务加速度无关 —— 末端因此被饱和力矩主动甩出而不是滞后。
        torque_scale = 1.0
        if self.torque_saturation_mode == "uniform":
            torque_scale = uniform_acceleration_torque_scale(
                tau_bias, tau - tau_bias, self.feasibility_torque)
            tau = tau_bias + torque_scale * (tau - tau_bias)
        # 跟踪段静摩擦前馈。加在【饱和缩放之后】：缩放是为了保住任务加速度
        # 的方向，而这一项补的是摩擦，不属于任务加速度，不该被一起缩掉。
        #
        # ⚠️ 驱动量必须取【参考速度 + 误差】，不能取净力矩 τ − τ_bias。
        # 2026-08-04 第 17 次上机实测：用净力矩驱动时前馈抬高净力矩、净力矩
        # 又抬高前馈 —— 瞬时正反馈，操作者观察到剧烈抖振并当场断电。
        # 参考速度与误差都要经过机械臂动力学才变化，构不成瞬时环路；
        # approach 段用的就是这个结构，第 16 次实测它把机械臂精确开到了
        # 轨迹起点（每轴差 0.8° 以内）。
        tau_track_friction = np.zeros(self.n_arm)
        if self.tracking_friction.enabled:
            J6 = kin_ctrl.J_arm_lwa                     # 6×n，与误差同为 6 维
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
        if (
            not self.internal_plant
            and self.feedback_generation
            <= self.last_command_feedback_generation
        ):
            return
        # 本周期确定要跑控制了 —— 推进反馈通路模型 + 估计器一拍。放在这里
        # （而不是上面任何一个提前 return 之前）是为了让"拍"与"控制周期"严格
        # 一一对应：等反馈、等 plant 的空转周期不消耗延迟管道、也不推进观测器。
        self._refresh_controller_feedback(now)
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
            # 【approach 段必须驱动机械臂，硬件模式也一样】。
            #
            # 这里原来是 `if self.simulated_base: _run_approach() else:
            # _publish_start_hold()` —— 硬件模式只发 RViz 可视化，不发任何
            # 力矩，电机全靠 student_arm_node 的 hold 锁存维持。等 start_delay
            # 一到跟踪直接使能，此时末端距参考起点可达 144mm，SMC(λ=72,k=102)
            # 立刻全力冲。
            #
            # 实机实测后果（2026-08-02，13s 记录）：使能起点 t=0、approach 完全
            # 跳过、关节速度峰值 28.17 rad/s（电机上限 3/5）、力矩饱和占比
            # 100%、j4 冲到 233°。操作者断电中止。
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
            # approach 结束、跟踪接管的那一帧。这个位形是判断 approach 成败的
            # 唯一依据 —— 记下来，别再靠事后从抓包反推。
            # 用测量值：实物上这条日志的来源就是测量值，两边要能直接对读。
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
        # 【控制器看到的那一份】：基座用估计值，手臂用经反馈通路模型（传输延迟
        # + 速度额外滞后 + 位置量化）之后的 q_ctrl/qd_ctrl。模型关闭时
        # q_ctrl is self.q，与改动前逐位相同。
        q_full_ctrl = self.model.make_q_full(bs_est.position_w, bs_est.quaternion_wxyz, self.q_ctrl)
        v_full_ctrl = self.model.make_v_full(bs_est.velocity_lwa, self.qd_ctrl)
        kin_ctrl = self.model.compute_kinematics(q_full_ctrl, v_full_ctrl)
        # 【轨迹时钟】。冻结开启时拉入期间轨迹停在起点，t>=blend_sec 才从 0 走。
        #
        # 取舍：ref 的【全部】下游都用冻结后的这一份，只有 blend 自身的 τ 和
        # CSV 的 "t" 列仍用墙钟 t。逐条理由：
        #
        # * handover_blend.capture 的参考位姿/参考速度 —— 必须冻结后的。
        #   Δp/Δv 的定义是"实际量 − 【参考量】"，参考量既然是冻结的那一份，
        #   偏置就得对它取，否则 τ=0 处 e 与 ė 不再恒等于零，整个机制失效。
        #   （t=0 时冻结前后同为 reference_sample(0.0)，此处两者数值相同，
        #   写成冻结后的是为了语义正确，不依赖"capture 一定发生在 t=0"。）
        #
        # * pos_error/rot_error 指标与 CSV 的 desired_* 列 —— 必须冻结后的。
        #   这不是"混合窗内怎么记"的口味问题：冻结开启后 t>blend_sec 的
        #   参考量是 reference_sample(t-blend_sec)，若指标仍拿 reference_sample(t)
        #   去比，比的是圆上【差了 45 s 相位】的另一个点，整段跟踪的 RMS/peak
        #   全部作废，而不只是拉入段。desired_* 同理，画出来会与实际轨迹错位。
        #   （沿用既有约定：desired_* 记的是【未加偏置】的轨迹参考，偏置后的
        #   那一份由 handover_blend 列 + Δp 还原，两条口径都在 CSV 里可查。）
        #
        # * ref.pen_down / ref.stroke_id —— 也用冻结后的，即"轨迹说什么就是
        #   什么"。冻结期间轨迹停在 t=0，笔状态就是起笔状态，RViz 里那条 157 mm
        #   的拉入直线会并进第 0 笔。选它是因为 CSV 的 pen_down/stroke_id 与
        #   desired_* 必须出自【同一个时刻】的同一个 sample，否则事后分段对不上；
        #   要把拉入段摘出来，用新增的 trajectory_clock==0 或 handover_blend>0
        #   两列任选其一即可，信息没有丢。
        #
        # * bs_true/bs_est（基座）不冻结 —— 基座运动是环境/被控对象的属性，不是
        #   末端参考。hwtest 基座本来就固定（MB_BASE_RADIUS=0），冻结与否无差别；
        #   而 whole 任务的 blend_sec=0，冻结整体是空操作。不去动它，避免把
        #   已验证的 whole 基线卷进来。
        traj_t = self._trajectory_clock(t)
        ref = reference_sample(traj_t, self.shape, self.derivative_dt)
        # 交接偏置衰减。第一帧把参考量整体搬到当前实际位姿上，使 SMC 看到的
        # 初始误差恒为 0，然后平滑衰减掉。不这样做的话，approach 留下的几十
        # 毫米残差乘以 k·λ=7344 就是几百 m/s² 的需求，而该位形执行器上限只有
        # 48.8 m/s² —— 2026-08-03 上机正是在交接后 55 ms 被看门狗按住的。
        # 用 kin_ctrl（控制器实际看到的那一份），不是 kin_true。
        if not self.handover_blend.captured:
            # 5D 任务下只补偿笔轴指向之差 —— 绕笔轴的自转是自由量，不是误差。
            # 按 6D 取会把它当偏置塞进参考量（实测虚报 120.8°）。
            self.handover_blend.capture(
                t, kin_ctrl.ee_position, kin_ctrl.ee_rotation,
                ref.position_w, ref.rotation_w,
                pen_axis_local=(self.pen_axis_local
                                if self.orientation_mode == "pen_axis"
                                else None),
                # 速度也必须匹配：approach 插值末端速度为零，但机械臂滞后于
                # 插值。2026-08-03 实测交接瞬间 e≡0 而末端仍以 0.180 m/s 运动，
                # (K+Λ)·|ė| = 31.4 m/s² 独占全部需求，看门狗 70 ms 触发。
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
        # 保持位置的目标用【测量值】—— 实物上看门狗手里就只有测量值。
        # ⚠️ 必须是 _q_meas 不是 q_ctrl：路线 D 下 q_ctrl 是 Smith 预测位置
        # （逐关节最多偏 predict_clamp_rad = 0.005 rad），而触发之后这个位形会以
        # p=30 发给电机当保位目标 —— 兜底不能建立在"预测对不对"上面。
        if self._watchdog_hold_q is None:
            self._watchdog_hold_q = np.asarray(self._q_meas, dtype=float).copy()
        if not self._watchdog_trip(now):
            self._watchdog_hold_q = np.asarray(self._q_meas, dtype=float).copy()
        motor = self.adapter.torque_to_motor_command(tau, now, self.last_joint_stamp or now, now, dt)
        # CSV 的 joint_*_rad / joint_*_velocity_rad_s 记【控制器看到的那一份】，
        # 与实物 CSV 同口径（实物上也只有测量值）。真值只进 pos_error 等指标。
        recorded_q = self.q_ctrl.copy()
        recorded_qd = self.qd_ctrl.copy()
        self._publish_motor_command(motor)
        # 拍尾：把【已发出】的这条指令折算成有效关节加速度喂给估计器。
        # tau_bias 上面已经算过，直接复用，省一次 RNEA。
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
            # 力矩反馈与位置走同一帧串口报文，共享 transport 延迟（诊断量，
            # 不进控制律）。模型关闭时 effort_ctrl is feedback_effort。
            tau_actual = np.asarray(self.effort_ctrl, dtype=float).copy()
            self.last_injection["tau_actual"] = tau_actual.copy()
            self.last_injection["tau_disturbance"] = (
                tau_actual - motor.command)

        # 【指标】：对【未加偏置的轨迹参考】的误差，取的是 traj_t 这一刻的轨迹
        # （冻结开启时拉入段就是轨迹起点）。交接混合窗内它按设计不为零，评分要
        # 从 τ > handover.reference_blend_sec 之后开始算。
        pos_error = float(np.linalg.norm(ref.position_w - kin_true.ee_position))
        # 【看门狗】：对【控制器实际被要求跟踪的那条参考】的误差。
        #
        # ⚠️ 两者不能混用。交接偏置衰减会把参考量整体平移到实际位姿上，混合
        # 窗内对真实轨迹的误差【按设计】就等于偏置量（2026-08-03 第 7 次上机
        # 实测 36.3 mm）。拿它去撞 50 mm 的包线，看门狗会把正常工作误判成故障
        # —— 那次就是这样在 t=0.35 s 被打断的。
        self.last_position_error = float(
            np.linalg.norm(blend_p - kin_true.ee_position))
        rot_error = float(np.linalg.norm(quaternion_log_error(ref.quaternion_wxyz, kin_true.ee_quaternion_wxyz)))
        self.error_samples.append(pos_error)
        self.rot_samples.append(rot_error)
        error_xyz = ref.position_w - kin_true.ee_position
        tau_disturbance = self.last_injection["tau_disturbance"]
        row = {
            "t": t,
            "phase": 1,                       # 1 = 跟踪，0 = approach
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
            # s(τ)：1 = 参考量完全搬到实际位姿上，0 = 已回到真实轨迹
            "handover_blend": float(self.handover_blend.scale(t)),
            # 该帧实际喂给 reference_sample 的轨迹时间（冻结关闭时 == t）
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
            # 跟踪段没有 approach 参考，留 NaN，别让绘图把 0 画成"误差已归零"
            row[f"approach_q_des_{i + 1}_rad"] = float("nan")
            row[f"approach_q_err_{i + 1}_rad"] = float("nan")
        self._fill_estimator_row(row)
        self.metrics_writer.writerow(row)
        # 【2026-08-04】原来每 50 帧 flush 一次 = 每 0.25 s 一次磁盘写。
        # 第 20 次上机实测：剩余丢拍里 idx%50==0 占 12.2%，随机只应 2.0%
        # —— 富集 6 倍，是丢拍的第二来源（第一是可视化标记，已修）。
        # 放到 1000 帧（5 s）一次：崩溃时最多丢 5 s 数据，而正常结束路径
        # （_finish）本来就会 flush + close，代价可以接受。
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
            # approach 段的目标与实际落点。之前这两个数只存在于日志文本里，
            # 事后无法用程序核对 approach 是"目标解错了"还是"没走到"。
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
            # 交接偏置衰减。[0, blend_sec] 这段参考量是被平移过的，那期间
            # 对【真实轨迹】的误差按设计就不为零，评分要从这之后开始算。
            "handover_blend": {
                "duration_sec": float(self.handover_blend.duration_sec),
                # 拉入期间是否冻结轨迹时钟（冻结时轨迹整体推迟 duration_sec 开始）
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
