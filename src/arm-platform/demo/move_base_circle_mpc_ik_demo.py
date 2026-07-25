#!/usr/bin/env python3
"""基座晃动场景下的 ACADOS MPC 末端轨迹跟踪 demo。"""

import csv
import dataclasses
import gc
import json
import math
import os
from datetime import datetime
from pathlib import Path

from geometry_msgs.msg import Point, TransformStamped
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

import moving_base_trajectory_config as traj
from dart_noise import bounded_dart_command
from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK
from online_path_time_scaler import OnlinePathTimeScaler, TimeScalingConfig
from pinocchio_ik import PinocchioIK
from second_order_mpc_ik import SecondOrderMpcConfig, SecondOrderMpcIK

JOINT_COUNT = 7
PUBLISH_RATE_HZ = 50.0
JOINT_LIMIT_WARN_MARGIN_DEG = 5.0
# LINE_LIST stores two geometry points per sampled segment.  The old 2000-point
# limit retained only about 20 seconds at 50 Hz, so early strokes disappeared
# before one ~76.6-second ``whole`` cycle finished.  12000 keeps a complete
# cycle (including margin) while pen-up samples still split strokes.
MAX_MARKER_POINTS = 12000
RESET_TOLERANCE_RAD = math.radians(2.0)
RESET_HOLD_SEC = 1.0
RECOMMENDED_CONTROLLER_MAX_VELOCITY_RAD_S = 1.0
# 阶段1C RTI per-iter 诊断在 loop CSV 里预留的迭代列数。当前 acados_rti_iterations=3,
# 完整覆盖。**若未来把 RTI 次数提高到 >MAX_RTI_LOG,超出部分的 per-iter status/terminal_error
# 不会进 CSV(会被截断)——届时需同步调大本常量,勿误以为 per-iter 列已记录全部迭代。**
MAX_RTI_LOG = 3

WORLD_FRAME = traj.WORLD_FRAME
BASE_FRAME = traj.BASE_FRAME


class MoveBaseCircleMpcIkDemo(Node):
    def __init__(self):
        super().__init__('move_base_circle_mpc_ik_demo')
        # moving-base 生产值: h=2/step=0.02;阶段2C+3 实测后降低姿态权重并固定名义姿态。
        self.declare_parameter('solver_backend', 'acados')
        self.declare_parameter('solver_model', 'second_order')
        self.declare_parameter('horizon_steps', 2)
        self.declare_parameter('max_joint_step', 0.02)
        self.declare_parameter('terminal_tolerance', 0.01)
        self.declare_parameter('orientation_weight', 0.09375)
        self.declare_parameter('terminal_orientation_weight', 1.25)
        self.declare_parameter('acados_rti_iterations', 3)
        self.declare_parameter('position_weight', 50.0)
        self.declare_parameter('terminal_position_weight', 500.0)
        self.declare_parameter('joint_delta_weight', 0.08)
        self.declare_parameter('joint_delta_change_weight', 0.25)
        self.declare_parameter('current_posture_weight', 0.01)
        self.declare_parameter('center_posture_weight', 0.002)
        self.declare_parameter('second_order_horizon_steps', 10)
        self.declare_parameter('second_order_position_weight', 5000.0)
        self.declare_parameter('second_order_terminal_position_weight', 500.0)
        self.declare_parameter('second_order_velocity_weight', 0.02)
        self.declare_parameter('second_order_acceleration_weight', 0.0)
        self.declare_parameter('second_order_current_posture_weight', 0.01)
        self.declare_parameter('second_order_center_posture_weight', 0.002)
        self.declare_parameter('second_order_max_velocity', 1.0)
        self.declare_parameter('second_order_max_acceleration', 5.0)
        self.declare_parameter('second_order_min_jacobian_sigma', 0.05)
        # 二阶 dq 前馈逆增益补偿: 发布 dq_next/该值(clip 到 ±max_velocity)。
        # 1.0=关(现状);0.7=补偿执行器 velocity_feedforward_gain 衰减。
        # 依据: second_order_offline_loop.py 离线闭环 —— tuned 配置 P95 35.1->0.28mm。
        self.declare_parameter('second_order_dq_ff_compensation', 1.0)
        # 全速时延补偿(离线/真机 h=10 时延实验归因):
        #   reference_lead: 二阶参考额外前移(默认继承 reference_lead_steps;2=多补一拍求解/发布时延)
        #   state_prediction: 用 dq 把 x0 从反馈时刻前推到求解时刻(反馈陈旧补偿,上限 2*dt)
        self.declare_parameter('second_order_reference_lead_steps', -1)
        self.declare_parameter('second_order_state_prediction', False)
        # x0 速度来源: measured=实测 dq(默认) | commanded=上一拍已发布的模型速度
        # (位置仍实测闭环;用于高速下测速噪声/延迟污染双积分器状态的对照实验)。
        self.declare_parameter('second_order_velocity_source', 'commanded')
        # 控制频率(优化①): 默认 50Hz 生产;100Hz 实验须配 max_joint_step:=0.01
        # (= max_velocity * dt,越界会告警)。求解 2-6ms,10ms 周期可容纳。
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('trajectory_shape', traj.EE_TRAJECTORY_SHAPE)
        self.declare_parameter('trajectory_time_scale', 1.0)
        self.declare_parameter('base_motion_time_scale', 1.0)
        # 阶段3: current 保持旧行为; reset_nominal 用复位姿态作为全臂固定软锚点。
        self.declare_parameter('posture_reference_mode', 'reset_nominal')
        # 阶段2E:只缩放末端图形路径时间，不缩放真实基座时间。默认关闭，先 shadow/实验。
        self.declare_parameter('online_time_scaling_enabled', False)
        self.declare_parameter('time_scaling_min_scale', 0.0)
        self.declare_parameter('time_scaling_velocity_reserve', 0.15)
        self.declare_parameter('time_scaling_slow_error_m', 0.010)
        self.declare_parameter('time_scaling_stop_error_m', 0.030)
        self.declare_parameter('command_fk_step_warn_m', 0.006)
        # 阶段1E: 期望运动感知守卫。
        #   legacy_fixed_step : 生产默认(旧 6mm 固定阈值守卫,行为与阶段1E之前逐拍一致);
        #   expected_shadow   : 拦截仍完全用 legacy;额外只记录 expected-residual 诊断,
        #                       绝不改变 q/dq/warm_start/状态机;
        #   expected_residual : 新守卫主动拦截/投影(真实 A/B 验收通过前不得设为生产默认)。
        # 阈值来源: analyze_phase1e_guard.py 对阶段1D 三轮真实 D0(gain=0.7)数据的推导
        # (data/ab_runs_1e/phase1e_d0_residual_analysis.json):
        #   r_abs = ceil(max(2×P99.9=6.74mm, 1.5×max=5.82mm, 0.5mm)) = 6.8mm;
        #   r_rel = 0.5(D0 单速工作点不可辨识速度比例项,建模裕量,多速度数据后重推);
        #   r_acc = 1.0(解析曲率覆盖项,D0 工况贡献≈0.08mm)。
        # D0 回放: 误拦截 0,最小裕量 4.27mm,max r_allow 8.23mm(20mm 注入跳变 2.4x 拦截余量)。
        self.declare_parameter('guard_mode', 'expected_residual')
        self.declare_parameter('guard_residual_abs_m', 0.0068)
        self.declare_parameter('guard_residual_relative_gain', 0.5)
        self.declare_parameter('guard_residual_accel_gain', 1.0)
        self.declare_parameter('guard_projection_enabled', True)
        self.declare_parameter('guard_projection_min_lambda', 0.05)
        self.declare_parameter('guard_projection_max_iterations', 8)
        self.declare_parameter('guard_max_consecutive_blocks', 25)
        # 生产默认值(阶段 1A/1B/1C 均已 Codex 正式验收):普通启动直接用已验收配置。
        # reference_lead_steps=1(阶段1A 参考前移)/=0 旧语义仅回归与 A/B 用。
        self.declare_parameter('reference_lead_steps', 1)
        # dq_prev_mode=measured_delta(阶段1B,x0 用上一控制周期实测执行增量)为生产默认;
        #   zero(x0=[q,0])仅回归与 A/B baseline 用。
        self.declare_parameter('dq_prev_mode', 'measured_delta')
        # rti_iteration_mode=fixed_iterations(阶段1C,status==0 跑满 acados_rti_iterations,
        #   status!=0 立即停止)为生产默认;legacy_early_exit(旧早退)仅回归与 A/B baseline 用。
        self.declare_parameter('rti_iteration_mode', 'fixed_iterations')
        # 阶段0观测参数:逐 tick 控制环诊断 CSV + 运行参数快照 JSON(只记录,不影响控制)。
        # 默认关闭,由实验命令显式开启,避免常规运行留下诊断文件。
        self.declare_parameter('record_loop_csv', False)
        # Phase 0 / DART: defaults can be supplied by the same environment-variable
        # workflow as the base-motion scan, while explicit ROS parameters still win.
        self.declare_parameter(
            'dart_noise_std',
            float(os.environ.get('MB_DART_NOISE', '0') or '0'))
        self.declare_parameter(
            'dart_noise_clip_sigma',
            float(os.environ.get('MB_DART_CLIP_SIGMA', '2.5') or '2.5'))
        self.declare_parameter(
            'dart_noise_seed',
            int(os.environ.get('MB_DART_SEED', '0') or '0'))
        # 输出目录不硬编码绝对路径:默认取本文件同级 data/;可由参数覆盖。
        self.declare_parameter(
            'loop_data_dir', str(Path(__file__).resolve().parent / 'data'))
        # 源码/构建 provenance(逐文件 SHA + git + 节点参数)由实验脚本在控制开始前
        # 采集写文件,这里只传路径。**绝不在控制 timer 内做 git/hash/param dump 阻塞操作。**
        self.declare_parameter('provenance_json', '')

        self.solver_backend = self.get_parameter('solver_backend').value
        self.solver_model = str(self.get_parameter('solver_model').value)
        self.horizon_steps = int(self.get_parameter('horizon_steps').value)
        self.max_joint_step = float(self.get_parameter('max_joint_step').value)
        self.terminal_tolerance = float(self.get_parameter('terminal_tolerance').value)
        self.orientation_weight = float(self.get_parameter('orientation_weight').value)
        self.terminal_orientation_weight = float(
            self.get_parameter('terminal_orientation_weight').value)
        self.acados_rti_iterations = int(self.get_parameter('acados_rti_iterations').value)
        self.position_weight = float(self.get_parameter('position_weight').value)
        self.terminal_position_weight = float(
            self.get_parameter('terminal_position_weight').value)
        self.joint_delta_weight = float(self.get_parameter('joint_delta_weight').value)
        self.joint_delta_change_weight = float(
            self.get_parameter('joint_delta_change_weight').value)
        self.current_posture_weight = float(
            self.get_parameter('current_posture_weight').value)
        self.center_posture_weight = float(
            self.get_parameter('center_posture_weight').value)
        self.second_order_horizon_steps = int(
            self.get_parameter('second_order_horizon_steps').value)
        self.second_order_position_weight = float(
            self.get_parameter('second_order_position_weight').value)
        self.second_order_terminal_position_weight = float(
            self.get_parameter('second_order_terminal_position_weight').value)
        self.second_order_velocity_weight = float(
            self.get_parameter('second_order_velocity_weight').value)
        self.second_order_acceleration_weight = float(
            self.get_parameter('second_order_acceleration_weight').value)
        self.second_order_current_posture_weight = float(
            self.get_parameter('second_order_current_posture_weight').value)
        self.second_order_center_posture_weight = float(
            self.get_parameter('second_order_center_posture_weight').value)
        self.second_order_max_velocity = float(
            self.get_parameter('second_order_max_velocity').value)
        self.second_order_max_acceleration = float(
            self.get_parameter('second_order_max_acceleration').value)
        self.second_order_min_jacobian_sigma = float(
            self.get_parameter('second_order_min_jacobian_sigma').value)
        self.second_order_dq_ff_compensation = float(
            self.get_parameter('second_order_dq_ff_compensation').value)
        self.second_order_reference_lead_steps = int(
            self.get_parameter('second_order_reference_lead_steps').value)
        self.second_order_state_prediction = bool(
            self.get_parameter('second_order_state_prediction').value)
        self.second_order_velocity_source = str(
            self.get_parameter('second_order_velocity_source').value)
        self.trajectory_shape = str(
            self.get_parameter('trajectory_shape').value).strip().lower()
        self.trajectory_time_scale = float(
            self.get_parameter('trajectory_time_scale').value)
        self.base_motion_time_scale = float(
            self.get_parameter('base_motion_time_scale').value)
        self.posture_reference_mode = str(
            self.get_parameter('posture_reference_mode').value)
        self.online_time_scaling_enabled = bool(
            self.get_parameter('online_time_scaling_enabled').value)
        self.time_scaler = OnlinePathTimeScaler(TimeScalingConfig(
            min_scale=float(self.get_parameter('time_scaling_min_scale').value),
            velocity_reserve=float(
                self.get_parameter('time_scaling_velocity_reserve').value),
            tracking_slow_error_m=float(
                self.get_parameter('time_scaling_slow_error_m').value),
            tracking_stop_error_m=float(
                self.get_parameter('time_scaling_stop_error_m').value),
        ))
        self._time_scaling_utilization = 0.0
        self._time_scaling_tracking_error = 0.0
        self._last_path_target_time0 = float('nan')
        self.command_fk_step_warn_m = float(
            self.get_parameter('command_fk_step_warn_m').value)
        self.guard_mode = str(self.get_parameter('guard_mode').value)
        self.guard_residual_abs_m = float(self.get_parameter('guard_residual_abs_m').value)
        self.guard_residual_relative_gain = float(
            self.get_parameter('guard_residual_relative_gain').value)
        self.guard_residual_accel_gain = float(
            self.get_parameter('guard_residual_accel_gain').value)
        self.guard_projection_enabled = bool(
            self.get_parameter('guard_projection_enabled').value)
        self.guard_projection_min_lambda = float(
            self.get_parameter('guard_projection_min_lambda').value)
        self.guard_projection_max_iterations = int(
            self.get_parameter('guard_projection_max_iterations').value)
        self.guard_max_consecutive_blocks = int(
            self.get_parameter('guard_max_consecutive_blocks').value)
        self.reference_lead_steps = int(self.get_parameter('reference_lead_steps').value)
        self.dq_prev_mode = str(self.get_parameter('dq_prev_mode').value)
        self.rti_iteration_mode = str(self.get_parameter('rti_iteration_mode').value)
        self.record_loop_csv = bool(self.get_parameter('record_loop_csv').value)
        self.dart_noise_std = float(self.get_parameter('dart_noise_std').value)
        self.dart_noise_clip_sigma = float(
            self.get_parameter('dart_noise_clip_sigma').value)
        self.dart_noise_seed = int(self.get_parameter('dart_noise_seed').value)
        self.loop_data_dir = Path(str(self.get_parameter('loop_data_dir').value))
        self.provenance_json = str(self.get_parameter('provenance_json').value)
        if self.reference_lead_steps < 0:
            raise ValueError('reference_lead_steps must be >= 0')
        if self.dq_prev_mode not in ('zero', 'measured_delta'):
            raise ValueError("dq_prev_mode must be 'zero' or 'measured_delta'")
        if self.rti_iteration_mode not in ('legacy_early_exit', 'fixed_iterations'):
            raise ValueError("rti_iteration_mode must be 'legacy_early_exit' or 'fixed_iterations'")
        if self.solver_model not in ('first_order', 'second_order', 'shadow'):
            raise ValueError(
                "solver_model must be 'first_order', 'second_order' or 'shadow'")
        if self.second_order_horizon_steps < 1:
            raise ValueError('second_order_horizon_steps must be >= 1')
        if not (0.0 < self.second_order_dq_ff_compensation <= 1.0):
            raise ValueError('second_order_dq_ff_compensation must be in (0, 1]')
        if self.second_order_reference_lead_steps < -1:
            raise ValueError('second_order_reference_lead_steps must be >= -1 (-1=继承)')
        if self.second_order_velocity_source not in ('measured', 'commanded'):
            raise ValueError("second_order_velocity_source must be 'measured'|'commanded'")
        if min(
            self.second_order_position_weight,
            self.second_order_terminal_position_weight,
            self.second_order_velocity_weight,
            self.second_order_acceleration_weight,
            self.second_order_current_posture_weight,
            self.second_order_center_posture_weight,
        ) < 0.0:
            raise ValueError('second-order weights must be non-negative')
        if self.trajectory_time_scale <= 0.0 or self.base_motion_time_scale <= 0.0:
            raise ValueError('trajectory_time_scale/base_motion_time_scale must be > 0')
        if self.trajectory_shape not in traj.SUPPORTED_EE_TRAJECTORIES:
            raise ValueError(
                f"trajectory_shape must be one of {traj.SUPPORTED_EE_TRAJECTORIES}, "
                f"got '{self.trajectory_shape}'")
        self.trajectory_period_sec = (
            traj.ee_trajectory_period(self.trajectory_shape)
            * self.trajectory_time_scale
        )
        if self.posture_reference_mode not in ('current', 'reset_nominal'):
            raise ValueError(
                "posture_reference_mode must be 'current' or 'reset_nominal'")
        if self.dart_noise_std < 0.0:
            raise ValueError('dart_noise_std must be >= 0')
        if self.dart_noise_clip_sigma <= 0.0:
            raise ValueError('dart_noise_clip_sigma must be > 0')
        # 阶段1E: 守卫参数快速校验(拼写错误/非法值 -> ValueError,不静默回退)。
        self._validate_guard_config(
            self.guard_mode, self.guard_residual_abs_m,
            self.guard_residual_relative_gain, self.guard_residual_accel_gain,
            self.guard_projection_min_lambda, self.guard_projection_max_iterations,
            self.guard_max_consecutive_blocks)

        # 阶段0观测状态:loop CSV 懒打开,guard 计数,当前 tick 的 targets[0] 时刻。
        # 必须先于 _write_params_snapshot 初始化。
        self._run_stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._loop_file = None
        self._loop_writer = None
        self._loop_rows_since_flush = 0
        self._loop_record_failed = False
        self._last_target_time0 = float('nan')
        self.guard_block_count = 0
        # 阶段1E 守卫状态。锚点=最近**被接受并发布**的命令及其有效参考时刻;
        # 与 last_command_fk_world 告警监视状态**完全分离**(后者每 tick 都更新,含 hold 帧)。
        # shadow 计算、被拒绝命令、求解失败一律不更新锚点。
        self._guard_q_last = None
        self._guard_t_last_reference = float('nan')
        self._guard_p_cmd_last = None                # 缓存 p_cmd^W(q_guard_last, t_last)
        self._guard_p_ref_last = None                # 缓存 p_desired^W(t_last)
        self._guard_R_err_last = None                # 姿态 shadow: R_ref^T R_cmd(上一接受命令)
        self._guard_uncertainty_m = 0.0              # 真机基座估计不确定性接口(解析 demo=0)
        self.guard_consecutive_blocks = 0
        self.guard_projection_count = 0
        self.guard_fault = False
        self.solver_step_violation_count = 0
        # 最近一帧 /joint_states 的时间信息:轨迹外反馈时序诊断(不进控制)。
        self._js_header_stamp_sec = float('nan')  # 消息 header.stamp(发送端时钟)
        self._js_recv_wall_sec = float('nan')     # 本机接收 wall 时间
        # 实时时钟驱动的轨迹时间:不再用 self.t += dt 的合成时间(会因阻塞/补偿tick永久漂移)。
        # 轨迹时间原点在首个 track tick 的求解起点建立;此后 t_solve/t_publish 全部由真实
        # clock 相对该原点得到。origin 在所有阻塞准备(快照)完成之后才建立。
        self.traj_origin_wall = None
        self._last_tick_wall = None
        self._nominal_tick_index = 0
        # 阶段1B:measured_delta 需要上一 tick 的实测 q 与其 header 时间戳,估算实际执行增量。
        self._prev_solve_q_measured = None
        self._prev_solve_js_stamp = float('nan')
        self._prev_u0 = None                    # 上一 tick 的 u0,用于 delta_u0
        self._dart_rng = np.random.default_rng(self.dart_noise_seed)
        self.dart_clipped_count = 0
        self.dart_track_ticks = 0
        self.second_order_warm_start = None
        self._so_last_cmd_dq = None      # commanded 速度源: 上一拍实际接受的模型速度
        self.dq_prev_fallback_count = 0
        self.dq_prev_total_track_ticks = 0

        if self.solver_backend != 'acados':
            raise ValueError("solver_backend must be 'acados'")
        if self.horizon_steps < 1:
            raise ValueError('horizon_steps must be >= 1')
        if self.max_joint_step <= 0.0:
            raise ValueError('max_joint_step must be > 0')

        self.br = TransformBroadcaster(self)
        self.pub = self.create_publisher(JointState, '/student/joint_command', 10)
        self.actual_marker_pub = self.create_publisher(Marker, '/ee_traj_marker', 10)
        self.target_marker_pub = self.create_publisher(Marker, '/ee_target_traj_marker', 10)
        self.create_subscription(JointState, '/joint_states', self._joint_states_callback, 10)

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        if not (10.0 <= self.control_rate_hz <= 200.0):
            raise ValueError('control_rate_hz must be in [10, 200]')
        self.dt = 1.0 / self.control_rate_hz
        config = MpcIkConfig(
            horizon_steps=self.horizon_steps,
            control_dt=self.dt,
            max_joint_step=self.max_joint_step,
            success_tolerance=self.terminal_tolerance,
            solver_backend=self.solver_backend,
            position_weight=self.position_weight,
            terminal_position_weight=self.terminal_position_weight,
            orientation_weight=self.orientation_weight,
            terminal_orientation_weight=self.terminal_orientation_weight,
            joint_delta_weight=self.joint_delta_weight,
            joint_delta_change_weight=self.joint_delta_change_weight,
            current_posture_weight=self.current_posture_weight,
            center_posture_weight=self.center_posture_weight,
            acados_rti_iterations=self.acados_rti_iterations,
            rti_iteration_mode=self.rti_iteration_mode,
            acados_build_dir='/tmp/windylab_acados_moving_base_mpc_ik',
        )
        self.mpc_ik = MpcPinocchioIK(config=config)
        self.second_order_ik = None
        self._second_order_config = None
        if self.solver_model in ('second_order', 'shadow'):
            self._second_order_config = SecondOrderMpcConfig(
                horizon_steps=self.second_order_horizon_steps,
                control_dt=self.dt,
                max_joint_step=self.max_joint_step,
                success_tolerance=self.terminal_tolerance,
                solver_backend=self.solver_backend,
                position_weight=self.second_order_position_weight,
                terminal_position_weight=self.second_order_terminal_position_weight,
                orientation_weight=self.orientation_weight,
                terminal_orientation_weight=self.terminal_orientation_weight,
                current_posture_weight=self.second_order_current_posture_weight,
                center_posture_weight=self.second_order_center_posture_weight,
                joint_velocity_weight=self.second_order_velocity_weight,
                joint_acceleration_weight=self.second_order_acceleration_weight,
                max_joint_velocity=self.second_order_max_velocity,
                max_joint_acceleration=self.second_order_max_acceleration,
                min_translation_jacobian_sigma=self.second_order_min_jacobian_sigma,
                acados_rti_iterations=self.acados_rti_iterations,
                rti_iteration_mode='fixed_iterations',
                acados_build_dir='/tmp/windylab_acados_moving_base_second_order',
            )
            self.second_order_ik = SecondOrderMpcIK(config=self._second_order_config)
        self.seed_ik = PinocchioIK()
        # 快照在进入 track 阶段时写(此时 ROS 图稳定,发布者计数准确)。
        self._pending_config = config
        self._snapshot_written = False

        self.q_last = None
        self.q_measured = None
        self.dq_measured = None
        self.warm_start = None
        self.last_limit_warn = None
        self.last_joint_print_time = -1.0
        self.last_feedback_warn_time = 0.0
        self.phase = 'reset'
        self.reset_hold_time = 0.0
        self.last_command_fk_world = None

        self.ee_traj_marker = self._create_marker(
            topic_ns='ee_traj_mpc', marker_id=10, color=(1.0, 0.2, 0.2), scale=0.005)
        self.ee_target_traj_marker = self._create_marker(
            topic_ns='ee_target_traj_mpc', marker_id=11, color=(0.1, 0.8, 0.1), scale=0.004)
        # LINE_LIST history state.  A pen-up sample clears the previous point,
        # which makes it impossible for RViz to connect separate strokes.
        self._marker_previous = {}

        p0 = self._ee_target_base(0.0)
        r0 = self._target_rotation_base(0.0)
        q0, ok = self.seed_ik.solve(p0, target_rotation=r0, rot_weight=0.2)
        if not ok:
            raise RuntimeError(
                f'末端起始点 {p0.tolist()} 在当前姿态约束下 IK 不可达，请调整轨迹参数')
        q_pose, pose_ok, pose_info = self.mpc_ik.solve(
            p0,
            q_init=q0,
            target_rotation=r0,
        )
        if bool(pose_info.get('command_usable', pose_ok)):
            q0 = q_pose
            self.warm_start = pose_info['trajectory']
        else:
            self.get_logger().warn(
                'MPC 起始姿态目标没有完全收敛，先使用普通 IK 起点: '
                f"terminal_error={pose_info['terminal_error']:.4f}, "
                f"orientation_error={pose_info.get('terminal_orientation_error')}")

        # 二阶求解器在进入控制 timer 前完成生成/加载和一次静态预热，避免首个 track
        # tick 因 codegen 阻塞。该结果不发布。
        if self.second_order_ik is not None:
            static_positions = np.repeat(
                p0.reshape(1, 3), self.second_order_horizon_steps, axis=0)
            static_rotations = np.repeat(
                r0.reshape(1, 3, 3), self.second_order_horizon_steps, axis=0)
            _, _, _, second_init_info = self.second_order_ik.step(
                static_positions, q0, np.zeros(JOINT_COUNT),
                target_rotation=static_rotations,
                posture_reference=q0,
            )
            if not second_init_info.get('command_usable', False):
                raise RuntimeError(
                    '二阶求解器静态预热失败: '
                    f"status={second_init_info.get('status')}, "
                    f"terminal_error={second_init_info.get('terminal_error')}")
            self.second_order_warm_start = second_init_info['trajectory']

        self.q_last = q0
        self.reset_q_target = q0.copy()
        self.posture_reference = self.reset_q_target.copy()

        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f'基座晃动 ACADOS MPC demo 启动: solver={self.solver_backend}, '
            f'model={self.solver_model}, '
            f'trajectory={self.trajectory_shape}, '
            f'period={self.trajectory_period_sec:.2f}s, '
            f'horizon={self.horizon_steps}, max_joint_step={self.max_joint_step:.3f}, '
            f'orientation_weight={self.orientation_weight:.3f}, '
            f'terminal_orientation_weight={self.terminal_orientation_weight:.3f}, '
            f'closed_loop=True, '
            f'command_fk_step_warn={self.command_fk_step_warn_m * 1000.0:.1f}mm')
        recommended_max_step = RECOMMENDED_CONTROLLER_MAX_VELOCITY_RAD_S * self.dt
        if self.max_joint_step > recommended_max_step + 1e-9:
            self.get_logger().warn(
                'MPC max_joint_step 应与控制器 max_velocity 匹配: '
                'max_joint_step <= max_velocity * dt。'
                f'当前 dt={self.dt:.3f}s, max_joint_step={self.max_joint_step:.3f}rad; '
                '若 student_arm max_velocity:=1.0，建议 max_joint_step:=0.02。')

    def _joint_states_callback(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        joint_names = [f'joint{i + 1}' for i in range(JOINT_COUNT)]
        if not all(name in name_to_pos for name in joint_names):
            return
        self.q_measured = np.array([name_to_pos[name] for name in joint_names], dtype=float)
        if len(msg.velocity) == len(msg.name):
            name_to_vel = dict(zip(msg.name, msg.velocity))
            self.dq_measured = np.array(
                [name_to_vel[name] for name in joint_names], dtype=float)
        else:
            self.dq_measured = None
        # 观测:记录该反馈的 header 时间戳(发送端时钟)与本机接收 wall 时间。
        stamp = msg.header.stamp
        self._js_header_stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        self._js_recv_wall_sec = self.get_clock().now().nanoseconds * 1e-9

    def _create_marker(self, topic_ns: str, marker_id: int, color, scale: float) -> Marker:
        marker = Marker()
        marker.header.frame_id = WORLD_FRAME
        marker.ns = topic_ns
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.color.a = 1.0
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.pose.orientation.w = 1.0
        return marker

    @staticmethod
    def _to_point(pos_world: np.ndarray) -> Point:
        point = Point()
        point.x = float(pos_world[0])
        point.y = float(pos_world[1])
        point.z = float(pos_world[2])
        return point

    def _append_segment_and_publish(
            self, marker: Marker, pub, pos_world: np.ndarray,
            pen_down: bool, stroke_id: int):
        key = (marker.ns, marker.id)
        marker.header.stamp = self.get_clock().now().to_msg()
        if not pen_down:
            self._marker_previous.pop(key, None)
            pub.publish(marker)
            return

        previous = self._marker_previous.get(key)
        if previous is not None and previous[1] == int(stroke_id):
            marker.points.extend([
                self._to_point(previous[0]),
                self._to_point(pos_world),
            ])
            while len(marker.points) > MAX_MARKER_POINTS:
                del marker.points[:2]
        self._marker_previous[key] = (
            np.asarray(pos_world, dtype=float).copy(),
            int(stroke_id),
        )
        pub.publish(marker)

    def _base_pose(self, t: float) -> np.ndarray:
        return traj.base_pose(t / float(getattr(self, 'base_motion_time_scale', 1.0)))

    def _base_rotation(
            self, t: float, path_t: float = None) -> np.ndarray:
        if path_t is None:
            path_t = t
        return traj.base_rotation(
            t / float(getattr(self, 'base_motion_time_scale', 1.0)),
            getattr(self, 'trajectory_shape', traj.EE_TRAJECTORY_SHAPE),
            path_t / float(getattr(self, 'trajectory_time_scale', 1.0)),
        )

    def _ee_target_world(self, t: float) -> np.ndarray:
        return self._ee_trajectory_sample(t).position.copy()

    def _ee_trajectory_sample(self, t: float):
        return traj.ee_trajectory_sample(
            t / float(getattr(self, 'trajectory_time_scale', 1.0)),
            getattr(self, 'trajectory_shape', traj.EE_TRAJECTORY_SHAPE),
        )

    def _ee_target_base(self, base_t: float, path_t: float = None) -> np.ndarray:
        if path_t is None:
            path_t = base_t
        base_pos_world = self._base_pose(base_t)
        base_rot_world = self._base_rotation(base_t, path_t)
        ee_pos_world = self._ee_target_world(path_t)
        return base_rot_world.T @ (ee_pos_world - base_pos_world)

    def _target_rotation_base(
            self, base_t: float, path_t: float = None) -> np.ndarray:
        if path_t is None:
            path_t = base_t
        base_rot_world = self._base_rotation(base_t, path_t)
        target_rotation_world = self._ee_trajectory_sample(path_t).rotation
        return base_rot_world.T @ target_rotation_world

    def _compute_dq_prev(self, q_now, js_stamp_now):
        """计算本 tick 传给求解器的 previous_joint_delta(上一控制周期实际执行增量)。

        measured_delta 模式:
            Δq_exec = (q_now - q_prev) * control_dt / Δt_feedback,  Δt_feedback = js_now - js_prev
        clip 到 [-max_joint_step, +max_joint_step]。异常时回退 0 并记原因。

        **历史锚点(_prev_solve_q_measured / _prev_solve_js_stamp)是否更新由本函数决定,
        经 update_anchor/anchor_q/anchor_stamp 返回,调用方据此条件更新——绝不无条件写入,
        避免用无效/时间倒退的样本污染合法锚点。** 更新规则:
          - 当前样本无效(q 或 stamp 非有限)          -> 回退0(code4),不更新锚点(不写脏样本);
          - 上一锚点无效/缺失(prev q/stamp 非有限)    -> 回退0(code1),用当前合法样本重锚;
          - Δt 非有限/非递增                          -> 回退0(code2),不污染合法锚点(保留旧锚点);
          - Δt 过旧(>5*control_dt)                    -> 回退0(code3),用当前合法样本重锚
                                                          (**前提是随后能收到连续合法样本**才在下一拍恢复);
          - 正常                                       -> measured_ok(code0),用当前合法样本更新锚点。
        注:"异常后一拍恢复"以随后收到连续合法(有限且时间递增、间隔正常)的样本为前提;
        若异常持续,则持续回退直到重新出现连续合法样本。
        返回:input(clip后)/raw(clip前)/velocity_estimate(dq_meas*dt 交叉诊断)/source(0 zero/
        1 measured_ok/2 measured_fallback)/fallback(原因码)/feedback_dt/update_anchor/anchor_q/anchor_stamp。
        任何 source==1(measured_ok)行的 input/raw 都保证全部有限。
        """
        nanv = np.full(JOINT_COUNT, np.nan)
        zero = np.zeros(JOINT_COUNT)
        vel_est = (np.asarray(self.dq_measured, dtype=float) * self.dt
                   if self.dq_measured is not None else nanv)
        cur_finite = (q_now is not None and np.all(np.isfinite(q_now))
                      and js_stamp_now is not None and np.isfinite(js_stamp_now))

        def out(input_, raw_, source, fallback, dt_fb, update, aq=None, astamp=None):
            return {'input': input_, 'raw': raw_, 'velocity_estimate': vel_est,
                    'source': source, 'fallback': fallback, 'feedback_dt': dt_fb,
                    'update_anchor': update,
                    'anchor_q': (None if aq is None else np.asarray(aq, dtype=float).copy()),
                    'anchor_stamp': astamp}

        if self.dq_prev_mode == 'zero':
            # zero 模式 dq_prev 恒 0,**不执行 measured 的完整时间戳状态机**(不做 Δt 递增/过旧判定);
            # 仅当当前样本有效时维护一个最简锚点,便于诊断,不参与任何增量计算。
            if cur_finite:
                return out(zero, zero, 0, 0, float('nan'), True, q_now, float(js_stamp_now))
            return out(zero, zero, 0, 0, float('nan'), False)

        # ---- measured_delta ----
        if not cur_finite:                       # 当前样本无效:回退,绝不写脏锚点
            return out(zero, zero, 2, 4, float('nan'), False)
        prev_finite = (self._prev_solve_q_measured is not None
                       and np.all(np.isfinite(self._prev_solve_q_measured))
                       and np.isfinite(self._prev_solve_js_stamp))
        if not prev_finite:                      # 上一锚点无效/缺失:回退,用当前合法样本重锚
            return out(zero, zero, 2, 1, float('nan'), True, q_now, float(js_stamp_now))
        dt_fb = float(js_stamp_now) - float(self._prev_solve_js_stamp)
        if not np.isfinite(dt_fb) or dt_fb <= 0.0:   # 时间非递增:不污染合法锚点(保留)
            return out(zero, zero, 2, 2, dt_fb, False)
        if dt_fb > 5.0 * self.dt:                    # 反馈过旧:回退但重锚,保证下一拍能恢复
            return out(zero, zero, 2, 3, dt_fb, True, q_now, float(js_stamp_now))
        raw = (np.asarray(q_now, dtype=float) - self._prev_solve_q_measured) * (self.dt / dt_fb)
        clipped = np.clip(raw, -self.max_joint_step, self.max_joint_step)
        # measured_ok:input/raw 必有限(q_now/prev 均有限、dt_fb 有限>0),更新锚点
        return out(clipped, raw, 1, 0, dt_fb, True, q_now, float(js_stamp_now))

    def _target_horizon_schedule(
            self, t_solve: float, horizon_steps: int = None,
            lead_override: int = None):
        """Return future base/path clocks and the local path clock rate."""
        lead = (self.reference_lead_steps if lead_override is None
                else int(lead_override))
        horizon = self.horizon_steps if horizon_steps is None else int(horizon_steps)
        offsets = self.dt * (lead + np.arange(horizon))
        base_times = t_solve + offsets
        scaling_enabled = bool(getattr(
            self, 'online_time_scaling_enabled', False))
        if scaling_enabled:
            path_times = self.time_scaler.preview(offsets)
            path_rates = np.full(horizon, float(self.time_scaler.scale))
        else:
            path_times = base_times.copy()
            path_rates = np.ones(horizon)
        return base_times, path_times, path_rates

    def _target_horizon_base(self, t_solve: float, horizon_steps: int = None,
                             update_log_times: bool = True,
                             lead_override: int = None):
        # 参考时序由 reference_lead_steps(L)控制:targets[j] = target(t_solve+(L+j)*dt)。
        #   L=0(baseline): targets 从 t_solve 开始,预测 q_k 主要对齐 target(t_solve)(一拍滞后)。
        #   L=1(阶段1A):  targets 从 t_solve+dt 开始;配合求解器 stage->target 映射
        #                  (stage k 代价用 targets[k-1]),得 q_k <-> target(t_solve+k*dt)。
        # t_solve 是本 tick 求解起点相对轨迹原点的**真实 clock** 时间(非合成 self.t)。
        # /student/joint_command 的 header.stamp = 真实发布时刻(见 _publish_joint_positions);
        # 参考时刻 t_reference=t_solve+L*dt 单独记录在 loop CSV,不用 stamp 制造时间平移。
        base_times, path_times, _ = self._target_horizon_schedule(
            t_solve, horizon_steps, lead_override)
        if update_log_times:
            self._last_target_time0 = float(base_times[0])
            self._last_path_target_time0 = float(path_times[0])
        positions = np.array([
            self._ee_target_base(float(bt), float(pt))
            for bt, pt in zip(base_times, path_times)
        ])
        rotations = np.array([
            self._target_rotation_base(float(bt), float(pt))
            for bt, pt in zip(base_times, path_times)
        ])
        return positions, rotations

    def _publish_base_tf(self, t: float):
        x, y, z = self._base_pose(t)
        path_t = self.time_scaler.path_time if self.online_time_scaling_enabled else t
        base_rot = self._base_rotation(t, path_t)
        quat = pin.Quaternion(base_rot)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = WORLD_FRAME
        tf_msg.child_frame_id = BASE_FRAME
        tf_msg.transform.translation.x = float(x)
        tf_msg.transform.translation.y = float(y)
        tf_msg.transform.translation.z = float(z)
        tf_msg.transform.rotation.x = float(quat.x)
        tf_msg.transform.rotation.y = float(quat.y)
        tf_msg.transform.rotation.z = float(quat.z)
        tf_msg.transform.rotation.w = float(quat.w)
        self.br.sendTransform(tf_msg)

    def _second_order_budget_diag(self, info: dict) -> dict:
        """二阶归因诊断: ddq0/规划内速度、加速度约束激活率/最小平移奇异值。"""
        v = np.asarray(info.get('velocity_trajectory', np.empty((0, JOINT_COUNT))))
        a = np.asarray(info.get('acceleration_trajectory', np.empty((0, JOINT_COUNT))))
        return {
            'ddq0': np.asarray(info.get('ddq0', np.full(JOINT_COUNT, np.nan)), dtype=float),
            'vel_active_ratio': (float(np.mean(
                np.abs(v) >= self.second_order_max_velocity - 1e-6)) if v.size else float('nan')),
            'acc_active_ratio': (float(np.mean(
                np.abs(a) >= self.second_order_max_acceleration - 1e-6)) if a.size else float('nan')),
            'min_sigma': float(info.get('min_translation_jacobian_sigma', float('nan'))),
        }

    def _solve_second_order(self, t_solve: float, q_current: np.ndarray):
        """以实测 q/dq 固定二阶 x0。dq 缺失/非法时不求解、不伪造。"""
        if self.second_order_ik is None:
            return None
        if self.dq_measured is None:
            return {
                'available': False,
                'reason': 'joint_states has no measured velocity',
            }
        dq_current = np.asarray(self.dq_measured, dtype=float).reshape(-1)
        if (self.second_order_velocity_source == 'commanded'
                and self._so_last_cmd_dq is not None):
            dq_current = np.asarray(self._so_last_cmd_dq, dtype=float).reshape(-1)
        if (dq_current.shape != (JOINT_COUNT,)
                or not np.all(np.isfinite(dq_current))
                or np.max(np.abs(dq_current))
                > self.second_order_max_velocity + 1e-6):
            return {
                'available': False,
                'reason': 'measured velocity invalid or outside configured bound',
            }
        so_lead = (None if self.second_order_reference_lead_steps < 0
                   else self.second_order_reference_lead_steps)
        targets, rotations = self._target_horizon_base(
            t_solve, self.second_order_horizon_steps,
            update_log_times=(self.solver_model == 'second_order'),
            lead_override=so_lead)
        q0_solve = q_current
        if self.second_order_state_prediction:
            # 反馈陈旧补偿: x0 位置用 dq 前推 age = t_solve_wall - js_stamp(钳到 [0, 2dt])。
            age = 0.0
            if (self.traj_origin_wall is not None
                    and np.isfinite(self._js_header_stamp_sec)):
                age = (self.get_clock().now().nanoseconds * 1e-9
                       - float(self._js_header_stamp_sec))
            age = float(np.clip(age, 0.0, 2.0 * self.dt))
            q0_solve = np.clip(q_current + dq_current * age,
                               self.mpc_ik.q_lower, self.mpc_ik.q_upper)
        q_next, dq_next, ok, info = self.second_order_ik.step(
            targets,
            q0_solve,
            dq_current,
            warm_start=self.second_order_warm_start,
            target_rotation=rotations,
            posture_reference=(
                self.posture_reference
                if self.posture_reference_mode == 'reset_nominal' else None),
        )
        self.second_order_warm_start = info.get('trajectory')
        return {
            'available': True,
            'q_next': q_next,
            'dq_next': dq_next,
            'ok': ok,
            'info': info,
        }

    def _publish_world_markers(self, t: float):
        q_for_marker = self.q_measured if self.q_measured is not None else self.q_last
        if q_for_marker is None:
            return
        path_t = self.time_scaler.path_time if self.online_time_scaling_enabled else t
        base_pos_world = self._base_pose(t)
        base_rot_world = self._base_rotation(t, path_t)
        ee_pos_base, _ = self.mpc_ik.forward(q_for_marker)
        ee_pos_world = base_pos_world + base_rot_world @ ee_pos_base
        target_sample = self._ee_trajectory_sample(path_t)
        self._append_segment_and_publish(
            self.ee_traj_marker,
            self.actual_marker_pub,
            ee_pos_world,
            target_sample.pen_down,
            target_sample.stroke_id,
        )
        self._append_segment_and_publish(
            self.ee_target_traj_marker,
            self.target_marker_pub,
            target_sample.position,
            target_sample.pen_down,
            target_sample.stroke_id,
        )

    def _command_fk_world(self, q: np.ndarray, t: float) -> np.ndarray:
        base_pos_world = self._base_pose(t)
        base_rot_world = self._base_rotation(t)
        ee_pos_base, _ = self.mpc_ik.forward(q)
        return base_pos_world + base_rot_world @ ee_pos_base

    def _command_fk_step(self, q: np.ndarray, t: float) -> float:
        command_fk_world = self._command_fk_world(q, t)
        if self.last_command_fk_world is None:
            return 0.0
        return float(np.linalg.norm(command_fk_world - self.last_command_fk_world))

    def _update_command_fk_monitor(self, q: np.ndarray, t: float):
        if self.phase != 'track':
            return
        command_fk_world = self._command_fk_world(q, t)
        if self.last_command_fk_world is not None:
            step = float(np.linalg.norm(command_fk_world - self.last_command_fk_world))
            if step > self.command_fk_step_warn_m:
                period = float(getattr(
                    self, 'trajectory_period_sec', traj.EE_PERIOD_SEC))
                phase_deg = (360.0 * t / period) % 360.0
                self.get_logger().warn(
                    'Command FK 单步变化偏大: '
                    f'step={step * 1000.0:.2f} mm, phase={phase_deg:.1f} deg',
                    throttle_duration_sec=1.0,
                )
        self.last_command_fk_world = command_fk_world

    # ------------------------------------------------------------- 阶段1E 守卫
    @staticmethod
    def _validate_guard_config(mode, abs_m, rel_gain, acc_gain, min_lambda,
                               max_iterations, max_consecutive):
        """守卫参数合法性(独立静态方法,便于单测)。非法 -> ValueError,不静默回退。"""
        if mode not in ('legacy_fixed_step', 'expected_shadow', 'expected_residual'):
            raise ValueError(
                "guard_mode must be 'legacy_fixed_step'|'expected_shadow'|"
                f"'expected_residual', got '{mode}'")
        if not (math.isfinite(abs_m) and abs_m > 0.0):
            raise ValueError('guard_residual_abs_m must be finite and > 0')
        if not (math.isfinite(rel_gain) and rel_gain >= 0.0):
            raise ValueError('guard_residual_relative_gain must be finite and >= 0')
        if not (math.isfinite(acc_gain) and acc_gain >= 0.0):
            raise ValueError('guard_residual_accel_gain must be finite and >= 0')
        if not (math.isfinite(min_lambda) and 0.0 < min_lambda <= 1.0):
            raise ValueError('guard_projection_min_lambda must be in (0, 1]')
        if int(max_iterations) < 1:
            raise ValueError('guard_projection_max_iterations must be >= 1')
        if int(max_consecutive) < 1:
            raise ValueError('guard_max_consecutive_blocks must be >= 1')

    def _desired_world_accel(self, t: float, eps: float = 1e-3) -> np.ndarray:
        """期望轨迹世界系二阶导(中心差分;解析轨迹,确定性,无随机)。"""
        p_p = self._ee_target_world(t + eps)
        p_0 = self._ee_target_world(t)
        p_m = self._ee_target_world(t - eps)
        return (p_p - 2.0 * p_0 + p_m) / (eps * eps)

    def _guard_fk_world(self, q: np.ndarray, t: float):
        """世界系命令 FK(位置+姿态)。基座变换用**同一时刻 t**(新命令用 t_new,
        锚点缓存来自其自己的 t_last —— 绝不把两条命令放在同一基座位姿下虚假比较)。"""
        base_pos = self._base_pose(t)
        base_rot = self._base_rotation(t)
        ee_pos_base, ee_rot_base = self.mpc_ik.forward(q)
        return base_pos + base_rot @ ee_pos_base, base_rot @ ee_rot_base

    def _guard_expected_eval(self, q_candidate: np.ndarray, t_new: float) -> dict:
        """expected-residual 守卫量(纯计算,不改任何状态)。

        r_vec = Δp_cmd - Δp_ref = e_new - e_last(两种算法代数恒等,单元测试逐拍核对);
        r_allow = r_abs + r_rel*||Δp_ref|| + r_acc*||a_ref||*dt² + uncertainty(demo=0)。
        锚点缺失(首帧)=> 合法初始化: r_pos=0, r_allow=inf,不拦截。
        """
        p_cmd_new, r_cmd_new = self._guard_fk_world(q_candidate, t_new)
        p_ref_new = self._ee_target_world(t_new)
        e_new = p_cmd_new - p_ref_new
        R_err_new = self._ee_trajectory_sample(t_new).rotation.T @ r_cmd_new
        ev = {'p_cmd_new': p_cmd_new, 'p_ref_new': p_ref_new, 'e_new': e_new,
              'R_err_new': R_err_new, 't_new': float(t_new),
              'r_ori': float('nan'), 'first_anchor': self._guard_q_last is None}
        if self._guard_q_last is None:
            ev.update(r_vec=np.full(3, float('nan')), r_pos=0.0, r_allow=float('inf'),
                      allow_abs=self.guard_residual_abs_m, allow_rel=float('nan'),
                      allow_acc=float('nan'), expected_step=float('nan'),
                      command_step=float('nan'))
            return ev
        dp_ref = p_ref_new - self._guard_p_ref_last
        dp_cmd = p_cmd_new - self._guard_p_cmd_last
        r_vec = dp_cmd - dp_ref
        a_ref = self._desired_world_accel(t_new)
        allow_abs = self.guard_residual_abs_m
        allow_rel = self.guard_residual_relative_gain * float(np.linalg.norm(dp_ref))
        allow_acc = (self.guard_residual_accel_gain
                     * float(np.linalg.norm(a_ref)) * self.dt * self.dt)
        if self._guard_R_err_last is not None:
            ev['r_ori'] = float(np.linalg.norm(
                pin.log3(self._guard_R_err_last.T @ R_err_new)))
        ev.update(r_vec=r_vec, r_pos=float(np.linalg.norm(r_vec)),
                  r_allow=allow_abs + allow_rel + allow_acc + self._guard_uncertainty_m,
                  allow_abs=allow_abs, allow_rel=allow_rel, allow_acc=allow_acc,
                  expected_step=float(np.linalg.norm(dp_ref)),
                  command_step=float(np.linalg.norm(dp_cmd)))
        return ev

    def _guard_candidate_safe(self, q_candidate: np.ndarray, q_current: np.ndarray,
                              ev: dict, r_ori_baseline: float) -> bool:
        """投影候选附加安全检查: 有限性/关节上下限/相对当前实测的步长(发布速度限幅)
        /姿态残差不比原候选更差。"""
        if not np.all(np.isfinite(q_candidate)):
            return False
        if (np.any(q_candidate < self.mpc_ik.q_lower - 1e-12)
                or np.any(q_candidate > self.mpc_ik.q_upper + 1e-12)):
            return False
        if float(np.max(np.abs(q_candidate - q_current))) > self.max_joint_step + 1e-9:
            return False
        if (math.isfinite(ev['r_ori']) and math.isfinite(r_ori_baseline)
                and ev['r_ori'] > r_ori_baseline + 1e-9):
            return False
        return True

    def _guard_project(self, q_next: np.ndarray, q_current: np.ndarray, t_new: float,
                       r_ori_baseline: float):
        """确定性 λ 搜索: 在 [min_lambda, 1] 内找最大安全比例。

        先探 min_lambda(不可行 => 直接放弃,返回 None => block);再固定次数二分,
        每次评估 = 1 次 FK + 常数向量运算,迭代上限 guard_projection_max_iterations,
        最坏计算量固定,不阻塞 50Hz 控制周期。返回 (λ, q_safe, ev_safe) 或 None。
        """
        q_anchor = self._guard_q_last

        def try_lambda(lam: float):
            q_s = q_anchor + lam * (q_next - q_anchor)
            ev = self._guard_expected_eval(q_s, t_new)
            if (ev['r_pos'] <= ev['r_allow']
                    and self._guard_candidate_safe(q_s, q_current, ev, r_ori_baseline)):
                return q_s, ev
            return None

        lam_lo = self.guard_projection_min_lambda
        best = try_lambda(lam_lo)
        if best is None:
            return None
        best_lam = lam_lo
        lo, hi = lam_lo, 1.0
        for _ in range(self.guard_projection_max_iterations):
            mid = 0.5 * (lo + hi)
            cand = try_lambda(mid)
            if cand is not None:
                best, best_lam, lo = cand, mid, mid
            else:
                hi = mid
        return best_lam, best[0], best[1]

    def _guard_accept(self, q_cmd: np.ndarray, ev: dict):
        """只有**实际接受并发布**的命令才更新守卫锚点(shadow 模式=legacy 接受的命令;
        residual 模式=λ 结果)。被拒绝命令/求解失败/纯 shadow 计算不得调用。"""
        self._guard_q_last = np.asarray(q_cmd, dtype=float).copy()
        self._guard_t_last_reference = float(ev['t_new'])
        self._guard_p_cmd_last = ev['p_cmd_new']
        self._guard_p_ref_last = ev['p_ref_new']
        self._guard_R_err_last = ev['R_err_new']
        self.guard_consecutive_blocks = 0

    def _guard_enter_fault(self):
        """连续拦截达上限: 显式 fault/abort。保持位置、停止轨迹跟踪;
        **不**通过自动重锚把异常命令放行。"""
        self.guard_fault = True
        self.phase = 'guard_fault'
        self.get_logger().error(
            f'守卫连续拦截达到上限 {self.guard_max_consecutive_blocks},进入 guard_fault '
            f'安全停止: 保持当前位置,停止轨迹跟踪(不自动重锚放行异常命令)。'
            f'total_blocks={self.guard_block_count}, '
            f'projections={self.guard_projection_count}')

    def _warn_joint_limits(self, q: np.ndarray):
        margin = np.deg2rad(JOINT_LIMIT_WARN_MARGIN_DEG)
        lower_gap = q - self.mpc_ik.q_lower
        upper_gap = self.mpc_ik.q_upper - q
        warn_msgs = []
        for idx, (low, high) in enumerate(zip(lower_gap, upper_gap), start=1):
            if low <= margin:
                warn_msgs.append(
                    f'joint{idx} 接近下限，剩余 {math.degrees(max(low, 0.0)):.2f} deg')
            elif high <= margin:
                warn_msgs.append(
                    f'joint{idx} 接近上限，剩余 {math.degrees(max(high, 0.0)):.2f} deg')

        warn_text = '; '.join(warn_msgs) if warn_msgs else None
        if warn_text and warn_text != self.last_limit_warn:
            self.get_logger().warn(f'检测到关节限位风险: {warn_text}')
        self.last_limit_warn = warn_text

    def _print_joint_angles(self, t: float):
        if self.q_measured is None:
            return
        if t - self.last_joint_print_time < 0.2:
            return
        self.last_joint_print_time = t
        joint_text = ', '.join(
            f'joint{i + 1}={angle:.4f} rad ({math.degrees(angle):.2f} deg)'
            for i, angle in enumerate(self.q_measured)
        )
        self.get_logger().info(f'当前关节角: {joint_text}')

    def _publish_joint_positions(self, q: np.ndarray, dq: np.ndarray = None):
        """发布关节命令。header.stamp = 真实发布 wall 时间(不制造时间平移)。
        返回实际位置/速度、发布时间和 DART 诊断,供 loop CSV 记录。"""
        nominal_q = np.asarray(q, dtype=float).reshape(JOINT_COUNT)
        dart_sample = np.zeros(JOINT_COUNT)
        dart_applied = np.zeros(JOINT_COUNT)
        dart_clipped = False
        q_executed = nominal_q
        dart_enabled = (
            self.dart_noise_std > 0.0
            and getattr(self, 'phase', '') == 'track'
            and self.q_measured is not None
        )
        if dart_enabled:
            dart = bounded_dart_command(
                nominal_q=nominal_q,
                current_q=self.q_measured,
                q_lower=self.mpc_ik.q_lower,
                q_upper=self.mpc_ik.q_upper,
                max_joint_step=self.max_joint_step,
                noise_std=self.dart_noise_std,
                noise_clip_sigma=self.dart_noise_clip_sigma,
                rng=self._dart_rng,
            )
            q_executed = dart.executed_q
            dart_sample = dart.sampled_noise
            dart_applied = dart.applied_noise
            dart_clipped = dart.was_clipped
            self.dart_track_ticks += 1
            self.dart_clipped_count += int(dart_clipped)

        msg = JointState()
        publish_time = self.get_clock().now()
        msg.header.stamp = publish_time.to_msg()
        msg.header.frame_id = 'moving_base_track' if self.phase == 'track' else 'moving_base_reset'
        msg.name = [f'joint{i + 1}' for i in range(JOINT_COUNT)]
        msg.position = q_executed.tolist()
        if dq is None:
            msg.velocity = [0.0] * JOINT_COUNT
        else:
            dq_arr = np.asarray(dq, dtype=float).reshape(JOINT_COUNT)
            if not np.all(np.isfinite(dq_arr)):
                self.get_logger().warn(
                    'MPC 发布速度包含 NaN/Inf，非法项改为 0',
                    throttle_duration_sec=2.0,
                )
                dq_arr = np.where(np.isfinite(dq_arr), dq_arr, 0.0)
            # Keep position and velocity feed-forward consistent with the DART
            # action that was actually sent to the simulated plant.
            if dart_enabled:
                dq_arr = dq_arr + dart_applied / self.dt
            velocity_limit = self.max_joint_step / self.dt
            msg.velocity = np.clip(dq_arr, -velocity_limit, velocity_limit).tolist()
        self.pub.publish(msg)
        return (
            q_executed.copy(),
            np.asarray(msg.velocity, dtype=float),
            publish_time.nanoseconds * 1e-9,
            {
                'nominal_q': nominal_q.copy(),
                'sampled_noise': dart_sample.copy(),
                'applied_noise': dart_applied.copy(),
                'clipped': dart_clipped,
            },
        )

    def _warn_missing_joint_states(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self.last_feedback_warn_time < 2.0:
            return
        self.last_feedback_warn_time = now_sec
        self.get_logger().warn(
            '还没有收到合法 /joint_states 反馈，复位阶段会等待。'
            '请确认 /joint_states 中包含 joint1 到 joint7。',
            throttle_duration_sec=2.0,
        )

    # ------------------------------------------------------------------
    # 阶段0观测:参数快照 + 逐 tick loop CSV。只读控制变量,不改任何控制行为。
    # ------------------------------------------------------------------

    def _joint_states_graph_info(self):
        # 纯本地 rmw 图查询,微秒级,不阻塞 executor。
        try:
            infos = self.get_publishers_info_by_topic('/joint_states')
            nodes = [f'{i.node_namespace.rstrip("/")}/{i.node_name}'.replace('//', '/')
                     for i in infos]
            return {
                'joint_states_publisher_count': self.count_publishers('/joint_states'),
                'joint_states_publisher_nodes': nodes,
                'joint_command_subscriber_count': self.count_subscribers(
                    '/student/joint_command'),
            }
        except Exception as exc:
            return {'joint_states_publisher_count': -1,
                    'graph_query_error': str(exc)}

    def _load_provenance(self):
        # 只读一个由实验脚本在控制开始前预生成的小 JSON;无 git/hash/subprocess。
        if not self.provenance_json:
            return {'note': 'no provenance_json provided'}
        try:
            return json.loads(Path(self.provenance_json).read_text())
        except Exception as exc:
            return {'error': f'failed to load provenance_json: {exc}',
                    'path': self.provenance_json}

    def _write_params_snapshot(self, config):
        # 在 reset->track 切换点调用。**只做快操作**:本地图查询 + 读 provenance 小文件 +
        # 组装 dict + 写盘。绝无 git/hash/ros2 param dump 阻塞(那些由脚本预采集)。
        try:
            self.loop_data_dir.mkdir(parents=True, exist_ok=True)
            graph = self._joint_states_graph_info()
            snapshot = {
                'schema_version': 4,
                'created': datetime.now().isoformat(timespec='seconds'),
                'node': self.get_name(),
                'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '(unset=0)'),
                'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION', '(default)'),
                'publish_rate_hz': self.control_rate_hz,
                'control_dt': self.dt,
                'joint_count': JOINT_COUNT,
                'reference_lead_steps': self.reference_lead_steps,
                'solver_model': self.solver_model,
                'dq_prev_mode': self.dq_prev_mode,
                'rti_iteration_mode': self.rti_iteration_mode,
                'u0_active_threshold_rad': 0.0195,
                'accel_limit_rad_s2': 5.0,
                'max_joint_step_rad': self.max_joint_step,
                'urdf_path': self.mpc_ik.urdf_path,
                'ee_frame': self.mpc_ik.ee_frame,
                'provenance': self._load_provenance(),
                'graph': graph,
                'experiment_valid': graph.get('joint_states_publisher_count') == 1,
                'ros_params': {
                    name: self.get_parameter(name).value
                    for name in (
                        'solver_backend', 'solver_model', 'horizon_steps', 'max_joint_step',
                        'terminal_tolerance', 'orientation_weight',
                        'terminal_orientation_weight', 'acados_rti_iterations',
                        'position_weight', 'terminal_position_weight',
                        'joint_delta_weight', 'joint_delta_change_weight',
                        'current_posture_weight', 'center_posture_weight',
                        'second_order_horizon_steps', 'second_order_velocity_weight',
                        'second_order_position_weight',
                        'second_order_terminal_position_weight',
                        'second_order_acceleration_weight',
                        'second_order_current_posture_weight',
                        'second_order_center_posture_weight',
                        'second_order_max_velocity',
                        'second_order_max_acceleration',
                        'second_order_min_jacobian_sigma',
                        'second_order_dq_ff_compensation',
                        'second_order_reference_lead_steps',
                        'second_order_state_prediction',
                        'second_order_velocity_source', 'control_rate_hz',
                        'trajectory_shape', 'trajectory_time_scale',
                        'base_motion_time_scale',
                        'posture_reference_mode',
                        'online_time_scaling_enabled', 'time_scaling_min_scale',
                        'time_scaling_velocity_reserve', 'time_scaling_slow_error_m',
                        'time_scaling_stop_error_m',
                        'command_fk_step_warn_m', 'reference_lead_steps', 'dq_prev_mode',
                        'rti_iteration_mode', 'record_loop_csv', 'loop_data_dir',
                        'dart_noise_std', 'dart_noise_clip_sigma', 'dart_noise_seed',
                        'guard_mode', 'guard_residual_abs_m',
                        'guard_residual_relative_gain', 'guard_residual_accel_gain',
                        'guard_projection_enabled', 'guard_projection_min_lambda',
                        'guard_projection_max_iterations', 'guard_max_consecutive_blocks',
                    )
                },
                'guard_mode': self.guard_mode,
                'trajectory_config': {
                    'BASE_CIRCLE_CENTER': traj.BASE_CIRCLE_CENTER.tolist(),
                    'BASE_CIRCLE_RADIUS': traj.BASE_CIRCLE_RADIUS,
                    'BASE_PERIOD_SEC': traj.BASE_PERIOD_SEC,
                    'BASE_HEIGHT': traj.BASE_HEIGHT,
                    'BASE_Z_AMPLITUDE': traj.BASE_Z_AMPLITUDE,
                    'BASE_Z_PHASE': traj.BASE_Z_PHASE,
                    'BASE_MOTION_SHAPE': traj.BASE_MOTION_SHAPE,
                    'BASE_SPHERE_RADIUS': traj.BASE_SPHERE_RADIUS,
                    'BASE_ATTITUDE_AMPLITUDE_DEG': traj.BASE_ATTITUDE_AMPLITUDE_DEG,
                    'BASE_ROLL_AMPLITUDE': traj.BASE_ROLL_AMPLITUDE,
                    'BASE_PITCH_AMPLITUDE': traj.BASE_PITCH_AMPLITUDE,
                    'BASE_YAW_AMPLITUDE': traj.BASE_YAW_AMPLITUDE,
                    'EE_WORLD_CIRCLE_CENTER': traj.EE_WORLD_CIRCLE_CENTER.tolist(),
                    'EE_CIRCLE_RADIUS': traj.EE_CIRCLE_RADIUS,
                    'EE_PERIOD_SEC': traj.EE_PERIOD_SEC,
                    'EE_TRAJECTORY_SHAPE': self.trajectory_shape,
                    'EE_TRAJECTORY_PERIOD_SEC': self.trajectory_period_sec,
                    'EE_PEN_LIFT_HEIGHT': traj.EE_PEN_LIFT_HEIGHT,
                    'EE_DRAW_SPEED': traj.EE_DRAW_SPEED,
                    'EE_TRAVEL_SPEED': traj.EE_TRAVEL_SPEED,
                    'TARGET_YAW_RAD': traj.TARGET_YAW,
                    'TARGET_PITCH_RAD': traj.TARGET_PITCH,
                    'TARGET_ROLL_RAD': traj.TARGET_ROLL,
                },
                'mpc_ik_config': dataclasses.asdict(config),
                'second_order_mpc_config': (
                    dataclasses.asdict(self._second_order_config)
                    if self._second_order_config is not None else None),
            }
            path = self.loop_data_dir / f'moving_base_mpc_params_{self._run_stamp}.json'
            with path.open('w') as f:
                json.dump(snapshot, f, indent=2, ensure_ascii=False, default=str)
            self.get_logger().info(
                f'运行参数快照已保存: {path} '
                f'(publishers={graph.get("joint_states_publisher_count")}, '
                f'valid={snapshot["experiment_valid"]})')
            if not snapshot['experiment_valid']:
                self.get_logger().error(
                    '预检失败: /joint_states 发布者数 != 1 '
                    f'({graph.get("joint_states_publisher_count")}, '
                    f'nodes={graph.get("joint_states_publisher_nodes")}); '
                    '本次数据不可作为正式 A/B 证据。')
        except Exception as exc:
            self.get_logger().warn(f'参数快照保存失败(不影响控制): {exc}')

    @staticmethod
    def _loop_csv_header():
        # 时间语义(全部真实 clock,相对轨迹原点=首个 track tick 求解起点):
        #   t_solve     = 本 tick 求解开始的真实时间(相对原点)。首行=0。
        #   t_publish   = 真正调用 publisher 前采集的真实时间(相对原点)。= header.stamp 相对原点。
        #   t_reference = t_solve + reference_lead_steps*dt(命令求解所瞄的参考时刻)。
        #   t_next      = t_publish + dt(物理下一拍执行时刻,lead 无关)。
        #   timer_interval_ms = 本 tick 求解起点 - 上一 tick 求解起点(检 catch-up:<0.5dt=补偿tick)。
        #   solve_to_publish_ms = 求解到发布耗时。
        #   feedback_age_ms = **求解开始时** wall - js_recv_wall(不等发布/写盘后再算)。
        #   js_header_stamp_rel = 求解所用 /joint_states header 时间戳相对原点(actual 序列时间轴)。
        #   t_solve_wall/t_publish_wall = 绝对 wall(catch-up/时钟诊断用)。
        cols = ['t_solve', 't_publish', 't_reference', 't_next', 'reference_lead_steps',
                'path_time', 'path_scale', 'path_target_time',
                'timer_interval_ms', 'solve_to_publish_ms', 'feedback_age_ms',
                'js_header_stamp_rel', 'js_header_stamp_sec', 'js_recv_wall_sec',
                't_solve_wall', 't_publish_wall']
        # 阶段1B dq_prev 诊断:
        #   dq_prev_input = 实际传给 ACADOS x0 的 dq_prev(clip后);dq_prev_raw = clip前;
        #   dq_prev_velocity_estimate = dq_meas*control_dt(交叉诊断,非实验变量);
        #   u0_minus_dq_prev = u0 - dq_prev_input(平滑项第一步真实残差);delta_u0 = u0_k - u0_{k-1};
        #   dq_prev_source(0 zero/1 measured_ok/2 measured_fallback)/dq_prev_fallback(原因码)/
        #   feedback_delta_t_sec = js_now - js_prev。
        for prefix in ('q_meas', 'dq_meas', 'q_next', 'q_expert', 'q_pub',
                       'dq_pub', 'u0', 'u_expert',
                       'dart_noise_sample', 'dart_noise_applied', 'u_exec',
                       'dq_prev_input', 'dq_prev_raw', 'dq_prev_velocity_estimate',
                       'u0_minus_dq_prev', 'delta_u0'):
            cols += [f'{prefix}_{i + 1}' for i in range(JOINT_COUNT)]
        cols += [
            'dart_noise_std_rad', 'dart_noise_clipped',
            'dq_prev_source', 'dq_prev_fallback', 'feedback_delta_t_sec',
            'norm_u0_minus_dq_prev', 'norm_delta_u0',
            # 阶段1C RTI 诊断:mode_code(0 legacy/1 fixed)、配置迭代数、实际 nit、是否早退,
            # 及各 RTI 迭代后的 status/terminal_error(nan 补齐/截断到 MAX_RTI_LOG 列,见常量注释)。
            'rti_mode_code', 'rti_configured', 'rti_nit_actual', 'rti_early_exit',
            *[f'rti_status_{i + 1}' for i in range(MAX_RTI_LOG)],
            *[f'rti_terminal_error_{i + 1}' for i in range(MAX_RTI_LOG)],
            'u0_active_count',
            'solver_status', 'solver_success', 'rti_iters', 'solve_time_ms',
            'terminal_error_m', 'first_error_m',
            'command_usable', 'tracking_ok', 'plan_feasible',
            'guard_block', 'command_fk_step_m',
            # 阶段1E 守卫诊断。guard_mode_code: 0 legacy_fixed_step / 1 expected_shadow /
            # 2 expected_residual。legacy 模式下 expected 残差列为 nan(不计算)。
            #   guard_t_new           = 新命令有效参考时刻 t_solve+lead*dt;
            #   guard_t_last_reference= 本 tick 决策所用锚点(上一被接受命令)的参考时刻;
            #   guard_r_pos_m/r_allow_m/allow_{abs,rel,acc}_m/r_vec_{x,y,z}/margin_m
            #                         = 残差、门限及其组成项(margin=r_allow-r_pos);
            #   guard_expected_step_m/command_step_m = ||Δp_ref||/||Δp_cmd||;
            #   guard_lambda: 1=原命令通过, (0,1)=投影比例, 0=blocked, nan=未评估;
            #   guard_r_ori_rad       = 姿态残差 shadow(只记录,不拦截);
            #   jstep_*               = 关节空间诊断(六);solver_step_violation = 步长契约违反。
            # 每 tick 基座位姿(求解时刻 t_solve 的解析基座位置+姿态 rpy)
            'base_pos_x', 'base_pos_y', 'base_pos_z',
            'base_roll', 'base_pitch', 'base_yaw',
            'guard_mode_code', 'guard_t_new', 'guard_t_last_reference',
            'guard_r_pos_m', 'guard_r_allow_m',
            'guard_allow_abs_m', 'guard_allow_rel_m', 'guard_allow_acc_m',
            'guard_r_vec_x', 'guard_r_vec_y', 'guard_r_vec_z',
            'guard_expected_step_m', 'guard_command_step_m', 'guard_margin_m',
            'guard_lambda', 'guard_projected', 'guard_consecutive_blocks', 'guard_fault',
            'guard_r_ori_rad',
            'jstep_max_next_current', 'jstep_max_next_anchor', 'jstep_max_current_anchor',
            'jstep_argmax_joint', 'jstep_limit_hit', 'solver_step_violation',
            # 二阶诊断:shadow 模式只计算不控制;second_order 模式同样填这些列
            # (此时 shadow_q/dq_next 即真实发布所用的二阶解,q_delta/fk_delta 为 nan)。
            'solver_model', 'shadow_available', 'shadow_status',
            'shadow_command_usable', 'shadow_solve_time_ms',
            'shadow_first_error_m', 'shadow_terminal_error_m',
            'shadow_q_delta_norm_rad', 'shadow_fk_delta_m',
            *[f'shadow_q_next_{i + 1}' for i in range(JOINT_COUNT)],
            *[f'shadow_dq_next_{i + 1}' for i in range(JOINT_COUNT)],
            # 二阶归因补充列: 首步加速度 ddq0、规划内速度/加速度约束激活率
            # (|v|>=vmax-1e-6 / |a|>=amax-1e-6 的条目占比)、平移雅可比最小奇异值。
            *[f'so_ddq0_{i + 1}' for i in range(JOINT_COUNT)],
            'so_vel_active_ratio', 'so_acc_active_ratio', 'so_min_sigma',
        ]
        return cols

    def _append_loop_row(
        self, timing, dq_prev, q_current, dq_measured, q_next, q_published, dq_published,
        info, command_usable, tracking_ok, guard_block, command_fk_step, guard_diag=None,
        shadow_diag=None, dart_diag=None,
    ):
        if not self.record_loop_csv or self._loop_record_failed:
            return
        try:
            if self._loop_writer is None:
                self.loop_data_dir.mkdir(parents=True, exist_ok=True)
                path = self.loop_data_dir / f'moving_base_mpc_loop_{self._run_stamp}.csv'
                self._loop_file = path.open('w', newline='')
                self._loop_writer = csv.writer(self._loop_file)
                self._loop_writer.writerow(self._loop_csv_header())
                self.get_logger().info(f'控制环诊断 CSV 开始记录: {path}')

            u0 = np.asarray(q_next, dtype=float) - np.asarray(q_current, dtype=float)
            dq_meas = (
                np.asarray(dq_measured, dtype=float)
                if dq_measured is not None
                else np.full(JOINT_COUNT, np.nan)
            )
            dqp_input = np.asarray(dq_prev['input'], dtype=float)
            dqp_raw = np.asarray(dq_prev['raw'], dtype=float)
            dqp_vel = np.asarray(dq_prev['velocity_estimate'], dtype=float)
            dd = dart_diag or {}
            dart_sample = np.asarray(
                dd.get('sampled_noise', np.zeros(JOINT_COUNT)), dtype=float)
            dart_applied = np.asarray(
                dd.get('applied_noise', np.zeros(JOINT_COUNT)), dtype=float)
            q_expert = np.asarray(
                dd.get('nominal_q', q_published), dtype=float)
            u_expert = q_expert - np.asarray(q_current, dtype=float)
            u_exec = np.asarray(q_published, dtype=float) - np.asarray(q_current, dtype=float)
            u0_minus_dqp = u0 - dqp_input
            delta_u0 = (u0 - self._prev_u0 if self._prev_u0 is not None
                        else np.full(JOINT_COUNT, np.nan))
            self._prev_u0 = u0.copy()
            origin = self.traj_origin_wall
            js_rel = (float(self._js_header_stamp_sec) - origin
                      if origin is not None and np.isfinite(self._js_header_stamp_sec)
                      else float('nan'))
            row = [
                timing['t_solve'],
                timing['t_publish'],
                timing['t_reference'],
                timing['t_next'],
                int(self.reference_lead_steps),
                float(self.time_scaler.path_time),
                float(self.time_scaler.scale),
                float(self._last_path_target_time0),
                timing['timer_interval_ms'],
                timing['solve_to_publish_ms'],
                timing['feedback_age_ms'],
                js_rel,
                float(self._js_header_stamp_sec),
                float(self._js_recv_wall_sec),
                timing['t_solve_wall'],
                timing['t_publish_wall'],
            ]
            for arr in (q_current, dq_meas, q_next, q_expert, q_published,
                        dq_published, u0, u_expert,
                        dart_sample, dart_applied, u_exec,
                        dqp_input, dqp_raw, dqp_vel, u0_minus_dqp, delta_u0):
                row += [float(v) for v in np.asarray(arr, dtype=float)]
            solve_time = info.get('solve_time')
            rti_status = list(info.get('rti_status_per_iter', []))
            rti_terr = list(info.get('rti_terminal_error_per_iter', []))

            def _pad(seq, n=MAX_RTI_LOG):
                return [seq[i] if i < len(seq) else float('nan') for i in range(n)]
            row += [
                self.dart_noise_std,
                int(bool(dd.get('clipped', False))),
                int(dq_prev['source']),
                int(dq_prev['fallback']),
                float(dq_prev['feedback_dt']),
                float(np.linalg.norm(u0_minus_dqp)),
                float(np.linalg.norm(delta_u0)) if np.all(np.isfinite(delta_u0)) else float('nan'),
                1 if info.get('rti_iteration_mode') == 'fixed_iterations' else 0,
                int(info.get('rti_configured_iters', -1)),
                int(info.get('nit', -1)),
                int(bool(info.get('rti_early_exit', False))),
            ]
            row += [float(s) for s in _pad(rti_status)]      # rti_status_1..3(nan 补齐)
            row += [float(x) for x in _pad(rti_terr)]        # rti_terminal_error_1..3
            row += [
                int(np.sum(np.abs(u0) >= 0.0195)),
                int(info.get('status', -1)),
                int(bool(info.get('success', False))),
                int(info.get('nit', -1)),
                float(solve_time) * 1000.0 if solve_time is not None else float('nan'),
                float(info.get('terminal_error', float('nan'))),
                float(info.get('first_error', float('nan'))),
                int(bool(command_usable)),
                int(bool(tracking_ok)),
                int(bool(info.get('plan_feasible', False))),
                int(bool(guard_block)),
                float(command_fk_step),
            ]
            # 每 tick 基座位姿(与控制所用同一解析源)
            bp = self._base_pose(timing['t_solve'])
            brpy = pin.rpy.matrixToRpy(self._base_rotation(timing['t_solve']))
            row += [float(bp[0]), float(bp[1]), float(bp[2]),
                    float(brpy[0]), float(brpy[1]), float(brpy[2])]
            # 阶段1E 守卫诊断列(guard_diag=None 时全 nan/0,列宽恒定)。
            gd = guard_diag or {}
            ev = gd.get('ev') or {}
            r_vec = np.asarray(ev.get('r_vec', np.full(3, np.nan)), dtype=float)
            r_pos = float(ev.get('r_pos', float('nan')))
            r_allow = float(ev.get('r_allow', float('nan')))
            row += [
                int(gd.get('mode_code', 0)),
                float(gd.get('t_new', float('nan'))),
                float(gd.get('t_last_reference', float('nan'))),
                r_pos,
                r_allow,
                float(ev.get('allow_abs', float('nan'))),
                float(ev.get('allow_rel', float('nan'))),
                float(ev.get('allow_acc', float('nan'))),
                float(r_vec[0]), float(r_vec[1]), float(r_vec[2]),
                float(ev.get('expected_step', float('nan'))),
                float(ev.get('command_step', float('nan'))),
                (r_allow - r_pos) if (math.isfinite(r_allow) and math.isfinite(r_pos))
                else float('nan'),
                float(gd.get('lambda', float('nan'))),
                int(bool(gd.get('projected', False))),
                int(gd.get('consecutive', 0)),
                int(bool(gd.get('fault', False))),
                float(ev.get('r_ori', float('nan'))),
                float(gd.get('jstep_next_current', float('nan'))),
                float(gd.get('jstep_next_anchor', float('nan'))),
                float(gd.get('jstep_current_anchor', float('nan'))),
                int(gd.get('jstep_argmax', -1)),
                int(bool(gd.get('jstep_limit_hit', False))),
                int(bool(gd.get('solver_step_violation', False))),
            ]
            sd = shadow_diag or {}
            shadow_q = np.asarray(
                sd.get('q_next', np.full(JOINT_COUNT, np.nan)), dtype=float)
            shadow_dq = np.asarray(
                sd.get('dq_next', np.full(JOINT_COUNT, np.nan)), dtype=float)
            row += [
                {'first_order': 0, 'second_order': 1, 'shadow': 2}[self.solver_model],
                int(bool(sd.get('available', False))),
                int(sd.get('status', -1)),
                int(bool(sd.get('command_usable', False))),
                float(sd.get('solve_time_ms', float('nan'))),
                float(sd.get('first_error', float('nan'))),
                float(sd.get('terminal_error', float('nan'))),
                float(sd.get('q_delta_norm', float('nan'))),
                float(sd.get('fk_delta', float('nan'))),
            ]
            row += [float(v) for v in shadow_q]
            row += [float(v) for v in shadow_dq]
            shadow_ddq0 = np.asarray(
                sd.get('ddq0', np.full(JOINT_COUNT, np.nan)), dtype=float)
            row += [float(v) for v in shadow_ddq0]
            row += [
                float(sd.get('vel_active_ratio', float('nan'))),
                float(sd.get('acc_active_ratio', float('nan'))),
                float(sd.get('min_sigma', float('nan'))),
            ]
            self._loop_writer.writerow(row)
            self._loop_rows_since_flush += 1
            if self._loop_rows_since_flush >= 100:
                self._loop_file.flush()
                self._loop_rows_since_flush = 0
        except Exception as exc:
            self._loop_record_failed = True
            self.get_logger().warn(f'loop CSV 记录失败,后续 tick 停止记录(不影响控制): {exc}')

    def _close_loop_csv(self):
        if self._loop_file is not None:
            try:
                self._loop_file.flush()
                self._loop_file.close()
                fb_ratio = (100.0 * self.dq_prev_fallback_count / self.dq_prev_total_track_ticks
                            if self.dq_prev_total_track_ticks else 0.0)
                self.get_logger().info(
                    f'控制环诊断 CSV 已关闭,守卫拦截总次数: {self.guard_block_count}; '
                    f'guard_mode={self.guard_mode}, '
                    f'projections={self.guard_projection_count}, '
                    f'guard_fault={self.guard_fault}, '
                    f'solver_step_violations={self.solver_step_violation_count}; '
                    f'DART sigma={self.dart_noise_std:.6f}rad, clipped '
                    f'{self.dart_clipped_count}/{self.dart_track_ticks}; '
                    f'dq_prev_mode={self.dq_prev_mode}, fallback '
                    f'{self.dq_prev_fallback_count}/{self.dq_prev_total_track_ticks} '
                    f'({fb_ratio:.3f}%)')
            except Exception:
                pass
            self._loop_file = None
            self._loop_writer = None

    def destroy_node(self):
        gc.enable()
        self._close_loop_csv()
        super().destroy_node()

    def _run_reset_phase(self):
        self._publish_joint_positions(self.reset_q_target)
        self._print_joint_angles(0.0)
        if self.q_measured is None:
            self._warn_missing_joint_states()
            return

        reset_err = np.abs(self.q_measured - self.reset_q_target)
        if np.all(reset_err <= RESET_TOLERANCE_RAD):
            self.reset_hold_time += self.dt
            if self.reset_hold_time >= RESET_HOLD_SEC:
                # 写快照(现已全为快操作:本地图查询+读 provenance 小文件+写盘,无 git/dump 阻塞)。
                # 在建立轨迹时间原点之前完成,原点由首个 track tick 建立,不受此处影响。
                if self.record_loop_csv and not self._snapshot_written:
                    self._write_params_snapshot(self._pending_config)
                    self._snapshot_written = True
                self.phase = 'track'
                self.q_last = self.reset_q_target.copy()
                self.traj_origin_wall = None   # 首个 track tick 求解起点建立原点
                self._last_tick_wall = None
                self._nominal_tick_index = 0
                self.warm_start = None
                self.last_command_fk_world = None
                # 阶段1E: 守卫锚点/连续计数随轨迹重启清零(首帧合法重锚,不拦截)。
                # guard_fault 不在此清除 —— fault 后不会再回到 track。
                self._guard_q_last = None
                self._guard_t_last_reference = float('nan')
                self._guard_p_cmd_last = None
                self._guard_p_ref_last = None
                self._guard_R_err_last = None
                self.guard_consecutive_blocks = 0
                # 阶段1B:进入 track 时清空跨帧状态,首帧 measured_delta 自然回退 0。
                self._prev_solve_q_measured = None
                self._prev_solve_js_stamp = float('nan')
                self._prev_u0 = None
                self.time_scaler.reset(0.0)
                self._time_scaling_utilization = 0.0
                self._time_scaling_tracking_error = 0.0
                # 实时时钟卫生:控制阶段禁用 GC,消除周期性 GC 暂停造成的 solve->publish
                # 抖动尖峰(实测偶发 ~34ms)。仅本轮实验(数十秒)禁用,close 时恢复;
                # 不改任何控制逻辑(dq_prev/RTI/vff/守卫/horizon/权重/模型均不动)。
                gc.collect()
                gc.disable()
                self.get_logger().info('机械臂已到达轨迹起始点，开始 MPC 轨迹跟踪 (GC 已禁用降抖动)')
        else:
            self.reset_hold_time = 0.0

    def _apply_command_guard(self, q_next, q_current, t_solve, info, command_usable):
        """阶段1E: 求解器步长契约 + 三种 guard_mode 的决策与应用(可单测,不含发布/CSV)。

        副作用(与阶段1E之前的内联逻辑一致): 接受时更新 self.q_last / self.warm_start /
        守卫锚点;拦截时保持 q_last、warm_start=None、计数并按连续上限进入 guard_fault。
        返回 dict: command_usable(硬拒绝后可能变 False)、dq_cmd、guard_block、
        command_fk_step 及 CSV 守卫诊断字段。
        """
        dq_cmd = np.zeros(JOINT_COUNT)
        guard_block = False
        command_fk_step = float('nan')
        t_new = t_solve + self.reference_lead_steps * self.dt
        guard_ev = None
        guard_t_last_used = float('nan')
        guard_lambda = float('nan')
        guard_projected = False
        solver_step_violation = False
        jstep_next_current = float('nan')
        jstep_next_anchor = float('nan')
        jstep_current_anchor = float('nan')
        jstep_argmax = -1
        if command_usable:
            # 阶段1E(六): 求解器步长契约 max|q_next-q_current| <= max_joint_step + eps。
            jstep_vec = np.abs(np.asarray(q_next, dtype=float) - q_current)
            jstep_next_current = float(np.max(jstep_vec))
            jstep_argmax = int(np.argmax(jstep_vec)) + 1
            if self._guard_q_last is not None:
                jstep_next_anchor = float(np.max(np.abs(
                    np.asarray(q_next, dtype=float) - self._guard_q_last)))
                jstep_current_anchor = float(np.max(np.abs(
                    q_current - self._guard_q_last)))
            # NaN/inf 候选也算契约违反(nan 比较恒 False,不显式检查会静默漏过;
            # legacy 模式对 NaN 的历史行为是 step=nan>阈值为 False 而放行 —— 该历史
            # 隐患在 legacy/shadow 保持不变(仅记录),expected_residual 模式硬拒绝)。
            if (not np.all(np.isfinite(jstep_vec))
                    or jstep_next_current > self.max_joint_step + 1e-9):
                solver_step_violation = True
                self.solver_step_violation_count += 1
                self.get_logger().error(
                    '求解器输出违反步长契约: '
                    f'max|q_next-q_current|={jstep_next_current:.5f} rad > '
                    f'max_joint_step={self.max_joint_step:.5f} rad (joint{jstep_argmax})'
                    + ('; expected_residual 模式硬拒绝,保持上一帧'
                       if self.guard_mode == 'expected_residual'
                       else '; legacy/shadow 模式仅记录(拦截行为与阶段1E之前一致)'))
                if self.guard_mode == 'expected_residual':
                    # 硬拒绝: 按命令不可用处理(保持上一帧),不计入守卫拦截。
                    command_usable = False
                    self.warm_start = None
        if command_usable:
            command_fk_step = self._command_fk_step(q_next, t_solve)
            if self.guard_mode != 'expected_residual':
                # legacy_fixed_step 决策(与阶段1E之前逐拍一致)。expected_shadow 的拦截
                # 也完全走这段 legacy 逻辑,shadow 只额外计算 expected 残差诊断。
                if (
                    self.last_command_fk_world is not None
                    and command_fk_step > self.command_fk_step_warn_m
                ):
                    guard_block = True
                    self.guard_block_count += 1
                    period = float(getattr(
                        self, 'trajectory_period_sec', traj.EE_PERIOD_SEC))
                    phase_deg = (360.0 * t_solve / period) % 360.0
                    self.get_logger().warn(
                        'MPC 命令会造成 command FK 突变，保持上一帧: '
                        f'step={command_fk_step * 1000.0:.2f} mm, phase={phase_deg:.1f} deg',
                        throttle_duration_sec=1.0,
                    )
                    dq_cmd = np.zeros(JOINT_COUNT)
                    self.warm_start = None
                else:
                    self.q_last = q_next
                    dq_cmd = (q_next - q_current) / self.dt
                    self.warm_start = info['trajectory']
                if self.guard_mode == 'expected_shadow':
                    # shadow: 只诊断,不改 q/dq/warm_start/状态机;锚点只在命令被
                    # (legacy 规则)接受时更新,拒绝帧不污染。
                    guard_t_last_used = self._guard_t_last_reference
                    guard_ev = self._guard_expected_eval(
                        np.asarray(q_next, dtype=float), t_new)
                    if not guard_block:
                        self._guard_accept(np.asarray(q_next, dtype=float), guard_ev)
            else:
                # expected_residual 主动守卫: 残差超限 -> λ 投影 -> 仍不可行才 hold。
                q_cand = np.asarray(q_next, dtype=float)
                guard_t_last_used = self._guard_t_last_reference
                guard_ev = self._guard_expected_eval(q_cand, t_new)
                if guard_ev['r_pos'] <= guard_ev['r_allow']:
                    guard_lambda = 1.0                       # 正常通过,不修改命令
                    self.q_last = q_cand
                    dq_cmd = (q_cand - q_current) / self.dt
                    self.warm_start = info['trajectory']
                    self._guard_accept(q_cand, guard_ev)
                else:
                    proj = (self._guard_project(q_cand, q_current, t_new,
                                                guard_ev['r_ori'])
                            if self.guard_projection_enabled else None)
                    if proj is not None:
                        guard_lambda, q_safe, ev_safe = proj
                        guard_projected = True
                        self.guard_projection_count += 1
                        self.q_last = q_safe
                        dq_cmd = (q_safe - q_current) / self.dt
                        self.warm_start = info['trajectory']
                        self._guard_accept(q_safe, ev_safe)
                        self.get_logger().warn(
                            '守卫投影: '
                            f'r_pos={guard_ev["r_pos"] * 1000.0:.2f}mm > '
                            f'r_allow={guard_ev["r_allow"] * 1000.0:.2f}mm, '
                            f'lambda={guard_lambda:.3f}, '
                            f'投影后 r_pos={ev_safe["r_pos"] * 1000.0:.2f}mm',
                            throttle_duration_sec=1.0,
                        )
                    else:
                        guard_block = True
                        guard_lambda = 0.0
                        self.guard_block_count += 1
                        self.guard_consecutive_blocks += 1
                        dq_cmd = np.zeros(JOINT_COUNT)
                        self.warm_start = None
                        self.get_logger().warn(
                            '守卫拦截(expected_residual): '
                            f'r_pos={guard_ev["r_pos"] * 1000.0:.2f}mm > '
                            f'r_allow={guard_ev["r_allow"] * 1000.0:.2f}mm 且无安全 λ,'
                            f'保持上一帧 (连续 {self.guard_consecutive_blocks})',
                            throttle_duration_sec=1.0,
                        )
                        if self.guard_consecutive_blocks >= self.guard_max_consecutive_blocks:
                            self._guard_enter_fault()
        return {
            'command_usable': command_usable,
            'dq_cmd': dq_cmd,
            'guard_block': guard_block,
            'command_fk_step': command_fk_step,
            't_new': t_new,
            'guard_ev': guard_ev,
            'guard_t_last_used': guard_t_last_used,
            'guard_lambda': guard_lambda,
            'guard_projected': guard_projected,
            'solver_step_violation': solver_step_violation,
            'jstep_next_current': jstep_next_current,
            'jstep_next_anchor': jstep_next_anchor,
            'jstep_current_anchor': jstep_current_anchor,
            'jstep_argmax': jstep_argmax,
        }

    def _run_tracking_phase(self):
        # 纯闭环:reset 阶段已确保 q_measured 到位,这里始终用实测反馈作为求解起点。
        # 轨迹时间由**真实 clock** 驱动:求解起点取 now,相对轨迹原点得 t_solve。
        solve_wall = self.get_clock().now().nanoseconds * 1e-9
        if self.traj_origin_wall is None:
            self.traj_origin_wall = solve_wall          # 首个 track tick 建立原点
        t_solve = solve_wall - self.traj_origin_wall
        timer_interval_ms = ((solve_wall - self._last_tick_wall) * 1000.0
                             if self._last_tick_wall is not None else float('nan'))
        self._last_tick_wall = solve_wall
        # feedback_age 在**求解开始时**计算(不等发布/写盘)。
        feedback_age_ms = ((solve_wall - self._js_recv_wall_sec) * 1000.0
                           if np.isfinite(self._js_recv_wall_sec) else float('nan'))

        self._publish_base_tf(t_solve)

        q_current = self.q_measured.copy()
        if self.online_time_scaling_enabled:
            scale_dt = (timer_interval_ms * 1e-3
                        if np.isfinite(timer_interval_ms) else 0.0)
            self.time_scaler.update(
                scale_dt,
                self._time_scaling_utilization,
                self._time_scaling_tracking_error,
            )
        # 阶段1B:计算上一控制周期实际执行增量(measured_delta),或 0(zero)。
        dqp = self._compute_dq_prev(q_current, self._js_header_stamp_sec)
        self.dq_prev_total_track_ticks += 1
        if dqp['fallback'] != 0:
            self.dq_prev_fallback_count += 1
        # 历史锚点**条件更新**:仅当 _compute_dq_prev 判定应更新(当前样本合法且不会污染)时才写,
        # 绝不无条件写入无效/时间倒退样本。
        if dqp['update_anchor']:
            self._prev_solve_q_measured = dqp['anchor_q']
            self._prev_solve_js_stamp = float(dqp['anchor_stamp'])

        second_result = None
        shadow_diag = None
        dq_model_next = None
        if self.solver_model in ('first_order', 'shadow'):
            targets, rotations = self._target_horizon_base(t_solve)
            q_next, ok, info = self.mpc_ik.step(
                targets,
                q_current,
                warm_start=self.warm_start,
                target_rotation=rotations,
                previous_joint_delta=dqp['input'],
                posture_reference=(
                    self.posture_reference
                    if self.posture_reference_mode == 'reset_nominal'
                    else None
                ),
            )
            if self.solver_model == 'shadow':
                second_result = self._solve_second_order(t_solve, q_current)
                if second_result and second_result.get('available'):
                    second_info = second_result['info']
                    q_shadow = np.asarray(second_result['q_next'], dtype=float)
                    first_fk, _ = self.mpc_ik.forward(q_next)
                    shadow_fk, _ = self.mpc_ik.forward(q_shadow)
                    shadow_diag = {
                        'available': True,
                        'status': second_info.get('status', -1),
                        'command_usable': second_info.get('command_usable', False),
                        'solve_time_ms': 1000.0 * float(
                            second_info.get('solve_time', float('nan'))),
                        'first_error': second_info.get('first_error', float('nan')),
                        'terminal_error': second_info.get(
                            'terminal_error', float('nan')),
                        'q_delta_norm': float(np.linalg.norm(q_shadow - q_next)),
                        'fk_delta': float(np.linalg.norm(shadow_fk - first_fk)),
                        'q_next': q_shadow,
                        'dq_next': second_result['dq_next'],
                        **self._second_order_budget_diag(second_info),
                    }
                else:
                    shadow_diag = {'available': False}
        else:
            second_result = self._solve_second_order(t_solve, q_current)
            if second_result and second_result.get('available'):
                q_next = second_result['q_next']
                dq_model_next = second_result['dq_next']
                ok = second_result['ok']
                info = second_result['info']
                # second_order 控制模式同样把二阶诊断进 CSV(q/dq_next=真实发布所用解)。
                shadow_diag = {
                    'available': True,
                    'status': info.get('status', -1),
                    'command_usable': info.get('command_usable', False),
                    'solve_time_ms': 1000.0 * float(info.get('solve_time', float('nan'))),
                    'first_error': info.get('first_error', float('nan')),
                    'terminal_error': info.get('terminal_error', float('nan')),
                    'q_delta_norm': float('nan'),
                    'fk_delta': float('nan'),
                    'q_next': np.asarray(q_next, dtype=float),
                    'dq_next': np.asarray(dq_model_next, dtype=float),
                    **self._second_order_budget_diag(info),
                }
            else:
                q_next = self.q_last.copy()
                ok = False
                info = {
                    'success': False,
                    'status': -20,
                    'message': (
                        second_result.get('reason', 'second-order unavailable')
                        if second_result else 'second-order unavailable'),
                    'command_usable': False,
                    'tracking_ok': False,
                    'plan_feasible': False,
                    'terminal_error': float('inf'),
                    'first_error': float('inf'),
                    'trajectory': np.repeat(
                        self.q_last.reshape(1, JOINT_COUNT),
                        self.second_order_horizon_steps, axis=0),
                }

        command_usable = bool(info.get('command_usable', ok))
        tracking_ok = bool(info.get('tracking_ok', ok))
        g = self._apply_command_guard(q_next, q_current, t_solve, info, command_usable)
        command_usable = g['command_usable']
        dq_cmd = g['dq_cmd']
        guard_block = g['guard_block']
        command_fk_step = g['command_fk_step']
        # 二阶控制正常通过守卫时，发布求解器预测的 dq_{k+1}，不能用位置差分
        # （后者是周期平均速度）。若守卫投影/hold，则沿用守卫生成的安全速度。
        if (
            self.solver_model == 'second_order'
            and dq_model_next is not None
            and command_usable
            and not guard_block
            and not g['guard_projected']
            and np.allclose(self.q_last, q_next, atol=1e-10, rtol=0.0)
        ):
            dq_cmd = np.clip(
                np.asarray(dq_model_next, dtype=float)
                / self.second_order_dq_ff_compensation,
                -self.second_order_max_velocity, self.second_order_max_velocity)
        if self.solver_model == 'second_order':
            accepted = (dq_model_next is not None and command_usable
                        and not guard_block and not g['guard_projected'])
            self._so_last_cmd_dq = (np.asarray(dq_model_next, dtype=float).copy()
                                    if accepted else np.zeros(JOINT_COUNT))
        accepted_step = np.asarray(self.q_last, dtype=float) - q_current
        self._time_scaling_utilization = float(np.clip(
            np.max(np.abs(accepted_step)) / self.max_joint_step, 0.0, 2.0))
        self._time_scaling_tracking_error = float(
            info.get('first_error', info.get('terminal_error', 0.0)))
        if command_usable:
            if not tracking_ok:
                ori_text = ''
                if info.get('terminal_orientation_error') is not None:
                    ori_text = f", orientation_error={info['terminal_orientation_error']:.4f}"
                self.get_logger().warn(
                    'MPC IK 跟踪误差超过阈值，继续发布可行命令: '
                    f"terminal_error={info['terminal_error']:.4f}, status={info['status']}, "
                    f"message={info['message']}{ori_text}",
                    throttle_duration_sec=2.0,
                )
        else:
            ori_text = ''
            if info.get('terminal_orientation_error') is not None:
                ori_text = f", orientation_error={info['terminal_orientation_error']:.4f}"
            self.get_logger().warn(
                'MPC IK 命令不可用，保持上一帧: '
                f"terminal_error={info['terminal_error']:.4f}, status={info['status']}, "
                f"message={info['message']}, plan_feasible={info.get('plan_feasible')}{ori_text}",
                throttle_duration_sec=2.0,
            )
            self.warm_start = None

        q_published, dq_published, publish_wall, dart_diag = (
            self._publish_joint_positions(self.q_last, dq_cmd))
        self._warn_joint_limits(q_published)
        self._update_command_fk_monitor(q_published, t_solve)

        t_publish = publish_wall - self.traj_origin_wall
        timing = {
            't_solve': t_solve,
            't_publish': t_publish,
            't_reference': t_solve + self.reference_lead_steps * self.dt,
            't_next': t_publish + self.dt,
            'timer_interval_ms': timer_interval_ms,
            'solve_to_publish_ms': (publish_wall - solve_wall) * 1000.0,
            'feedback_age_ms': feedback_age_ms,
            't_solve_wall': solve_wall,
            't_publish_wall': publish_wall,
        }
        self._append_loop_row(
            timing,
            dqp,
            q_current=q_current,
            dq_measured=self.dq_measured,
            q_next=q_next,
            q_published=q_published,
            dq_published=dq_published,
            info=info,
            command_usable=command_usable,
            tracking_ok=tracking_ok,
            guard_block=guard_block,
            command_fk_step=command_fk_step,
            guard_diag={
                'mode_code': {'legacy_fixed_step': 0, 'expected_shadow': 1,
                              'expected_residual': 2}[self.guard_mode],
                't_new': g['t_new'],
                't_last_reference': g['guard_t_last_used'],
                'ev': g['guard_ev'],
                'lambda': g['guard_lambda'],
                'projected': g['guard_projected'],
                'consecutive': self.guard_consecutive_blocks,
                'fault': self.guard_fault,
                'jstep_next_current': g['jstep_next_current'],
                'jstep_next_anchor': g['jstep_next_anchor'],
                'jstep_current_anchor': g['jstep_current_anchor'],
                'jstep_argmax': g['jstep_argmax'],
                'jstep_limit_hit': (math.isfinite(g['jstep_next_current'])
                                    and g['jstep_next_current']
                                    >= self.max_joint_step - 1e-9),
                'solver_step_violation': g['solver_step_violation'],
            },
            shadow_diag=shadow_diag,
            dart_diag=dart_diag,
        )
        self._nominal_tick_index += 1
        self._print_joint_angles(t_solve)
        self._publish_world_markers(t_solve)

    def tick(self):
        if self.phase == 'guard_fault':
            # 阶段1E: 显式安全停止 —— 保持最后被接受的命令,零速度,不再求解/推进轨迹,
            # 不自动重锚/复位(需要人工重启才能恢复)。
            self._publish_joint_positions(self.q_last, np.zeros(JOINT_COUNT))
            return
        if self.phase == 'reset':
            self._publish_base_tf(0.0)
            self._run_reset_phase()
            return
        self._run_tracking_phase()


def main():
    rclpy.init()
    node = MoveBaseCircleMpcIkDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
