#!/usr/bin/env python3
"""基座晃动场景下的 ACADOS MPC 末端轨迹跟踪 demo。"""

import os

# 实时时钟卫生(须在 numpy 首次 import 前生效): 本进程 numpy 全是小矩阵
# (≤1500×25), OpenBLAS 16 线程扇出纯属延迟抖动源 —— 忙碌进程内单个
# lstsq/matmul 实测偶发 3~15ms(线程唤醒被调度噪声放大), 直接打爆 50Hz
# tick 预算; 单线程后同一操作 <0.5ms 且尾部消失(P99 2.96→1.38ms,
# max 15.3→1.4ms, 60s@50Hz 负载 A/B)。acados 用 BLASFEO 不受影响。
# setdefault: 外部显式设置可覆盖。
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')

from collections import deque
import csv
import dataclasses
import gc
import json
import math
import random
import time
import traceback
from datetime import datetime
from pathlib import Path

from geometry_msgs.msg import Point, TransformStamped, Vector3Stamped
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

import moving_base_trajectory_config as traj
from base_trust_estimator import (
    BaseTrustEstimator,
    TrustFeedforwardGate,
    UncertaintyAwareFeedforwardDecision,
    trust_feedforward_gain,
)
from dart_noise import bounded_dart_command
from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK
from online_path_time_scaler import (
    OnlinePathTimeScaler,
    TimeScalingConfig,
    TrustLadderConfig,
    TrustLadderStateMachine,
)
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
# 与 student_arm.launch.py max_velocity 默认一致(达妙 DM4340/RS05 规格下限 3.0)。
RECOMMENDED_CONTROLLER_MAX_VELOCITY_RAD_S = 3.0
# 阶段1C RTI per-iter 诊断在 loop CSV 里预留的迭代列数。当前 acados_rti_iterations=3,
# 完整覆盖。**若未来把 RTI 次数提高到 >MAX_RTI_LOG,超出部分的 per-iter status/terminal_error
# 不会进 CSV(会被截断)——届时需同步调大本常量,勿误以为 per-iter 列已记录全部迭代。**
MAX_RTI_LOG = 3
# trust 开销"总账"通道的蓄水池抽样容量: track 阶段 GC 关闭且可小时级连续
# 运行, 无界 append 会以 50Hz(~36 万浮点对象/小时, 10-12MB RSS/小时)累积;
# 蓄水池(Algorithm R)使单次运行内存有上限, 分位数仍无偏反映全程分布。
TRUST_COST_RESERVOIR_CAP = 20000

WORLD_FRAME = traj.WORLD_FRAME
BASE_FRAME = traj.BASE_FRAME


class MoveBaseCircleMpcIkDemo(Node):
    def __init__(self):
        super().__init__('move_base_circle_mpc_ik_demo')
        # moving-base 生产值: h=2/step=0.02;阶段2C+3 实测后降低姿态权重并固定名义姿态。
        self.declare_parameter('solver_backend', 'acados')
        self.declare_parameter('solver_model', 'second_order')
        self.declare_parameter('horizon_steps', 2)
        # max_joint_step = second_order_max_velocity * dt (2.0*0.02)。
        self.declare_parameter('max_joint_step', 0.04)
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
        # 真实 whole-task A/B: N=20 使完整控制周期 P95=26.7ms，超过
        # 50Hz 的 20ms 截止期并导致 /joint_states 排队；N=10 恢复实时基线。
        self.declare_parameter('second_order_horizon_steps', 10)
        self.declare_parameter('second_order_position_weight', 5000.0)
        self.declare_parameter('second_order_terminal_position_weight', 500.0)
        # 外部仿射结构保留作显式 A/B。2 Hz whole-task 自动调参表明，把逐级
        # 投影后的导数与原始定时位置同时作为目标会制造周期冲突，因此生产默认
        # 只启用位置/轮廓代价；零导数权重时后端会跳过无效线性化。
        self.declare_parameter('second_order_cartesian_residual_mode', 'external_affine')
        self.declare_parameter('second_order_cartesian_velocity_weight', 0.0)
        self.declare_parameter('second_order_cartesian_acceleration_weight', 0.0)
        self.declare_parameter(
            'second_order_terminal_cartesian_velocity_weight', 0.0)
        self.declare_parameter('second_order_feasible_reference_projection', False)
        self.declare_parameter('second_order_feasible_reference_damping', 1e-4)
        # 轮廓代价保留作显式 A/B。N=20 实测被调度超时污染，生产默认先回到
        # 已真实验证的 position-only N=10；待 N=10 轮廓候选完成真机 A/B 再启用。
        self.declare_parameter('second_order_contour_tracking_enabled', False)
        self.declare_parameter('second_order_contour_position_weight', 5000.0)
        self.declare_parameter('second_order_lag_position_weight', 750.0)
        self.declare_parameter('second_order_transition_position_weight', 5000.0)
        self.declare_parameter(
            'second_order_terminal_contour_position_weight', 500.0)
        self.declare_parameter('second_order_terminal_lag_position_weight', 75.0)
        self.declare_parameter(
            'second_order_terminal_transition_position_weight', 500.0)
        self.declare_parameter('reference_derivative_dt_sec', 0.005)
        self.declare_parameter('second_order_velocity_weight', 0.02)
        # qddot_ff 与笛卡尔加速度残差均保留为显式 A/B；当前 2 Hz 固定约束
        # 消融中非零权重会放大横向误差，因此生产默认暂为 0。
        self.declare_parameter('second_order_acceleration_weight', 0.0)
        self.declare_parameter('second_order_terminal_velocity_weight', 0.0)
        self.declare_parameter('second_order_current_posture_weight', 0.01)
        self.declare_parameter('second_order_center_posture_weight', 0.002)
        # 20260726: 限幅按真实电机规格放开(达妙 DM4340 3rad/s / RS05 5rad/s,
        # 峰值扭矩对本臂惯量可支撑 >>15rad/s²)。v=2.0/a=15 留出 plant 侧
        # (launch 默认 3.0/20.0) 的执行与逆变换余量。旧 1.0/5.0 使加速度约束
        # 在 60-73% 预测变量上活跃,是 2Hz 螺旋残差的饱和失真来源。
        self.declare_parameter('second_order_max_velocity', 2.0)
        self.declare_parameter('second_order_max_acceleration', 15.0)
        self.declare_parameter('second_order_min_jacobian_sigma', 0.05)
        # 二阶 dq 前馈逆增益补偿: 发布 dq_next/该值(clip 到 ±max_velocity)。
        # 1.0=关(现状);0.7=补偿执行器 velocity_feedforward_gain 衰减。
        # 依据: second_order_offline_loop.py 离线闭环 —— tuned 配置 P95 35.1->0.28mm。
        self.declare_parameter('second_order_dq_ff_compensation', 1.0)
        # 全速时延补偿(离线/真机 h=10 时延实验归因):
        #   reference_lead: 二阶参考额外前移(默认继承 reference_lead_steps;2=多补一拍求解/发布时延)
        #   state_prediction: 用时间戳状态和上一拍已执行 ddq 把 x0 前推到预计命令生效时刻。
        self.declare_parameter('second_order_reference_lead_steps', -1)
        self.declare_parameter('second_order_state_prediction', False)
        # 已发布模型速度与二阶预测状态一致；实测速度和状态外推保留作显式实验，
        # 不作为未完成闭环辨识前的生产默认。
        self.declare_parameter('second_order_velocity_source', 'commanded')
        # 时间戳驱动的命令生效时刻预测。发布延迟用历史 EWMA，反馈时间用
        # joint_states header；二者与可配置执行器延迟共同决定 x0/参考的前移量。
        # 20260726: 默认开启。实测执行链: 发布 P50=8.75ms + plant 100Hz 采样保持
        # 平均 5ms;参考时刻与 x0 均对齐到预计命令生效时刻,消除确定性 2Hz 相位差。
        self.declare_parameter('latency_compensation_enabled', True)
        self.declare_parameter('latency_nominal_publish_ms', 9.0)
        self.declare_parameter('latency_publish_ewma_alpha', 0.15)
        self.declare_parameter('latency_actuator_delay_ms', 5.0)
        self.declare_parameter('latency_prediction_max_ms', 60.0)
        # 20260726 Smith 预测器: 用已知 plant 执行律(smooth_position_controller.cc
        # 速度前馈分支)+已发布在途命令队列,把 x0 从 joint_states 时间戳前推到
        # 预计命令生效时刻。替代 second_order_state_prediction(后者用 ddq0 外推,
        # 与 plant 实际执行不一致,离线闭环证实会正反馈进加速度饱和而发散)。
        self.declare_parameter('smith_predictor_enabled', True)
        # 命令逆变换: 已知 plant 两个 10ms 子步的线性执行律,解析求发布速度 v_c,
        # 使 plant 在下一控制拍末恰好落在求解器计划点 q_next 上:
        #   v1 = (q_next-q0)/(h*(2-h/tau)),  v_c = (v1-(q_next-q0)/tau)/gain
        # 取代"直接发布区间末速度 dq_{k+1}"(后者含半拍相位超前+位置反馈项增益)。
        self.declare_parameter('command_inversion_enabled', True)
        # plant 执行律常量,须与 student_arm.launch.py 实际启动参数一致。
        self.declare_parameter('plant_velocity_feedforward_gain', 1.0)
        self.declare_parameter('plant_position_feedback_tau', 0.15)
        self.declare_parameter('plant_feedback_velocity_limit', 0.4)
        self.declare_parameter('plant_max_velocity', 3.0)
        self.declare_parameter('plant_max_acceleration', 20.0)
        self.declare_parameter('plant_control_substep_dt', 0.01)
        # --- 基座不确定性实验(第1/2级): 真值=名义+有界扰动 ---
        # 幅值/种子须与 trajectory_visualizer_demo(真值 KPI 端)一致。
        # base_belief_mode 决定本控制器"知道"多少:
        #   nominal   只知名义轨迹(第1级: 误差下界=扰动本身);
        #   truth     精确知道扰动(上界对照, 默认——amp=0 时与现状完全一致);
        #   estimated 逐拍模拟无人机状态估计: 真值(t-延迟)+高斯噪声经一阶低通,
        #             预测域内保持常值(第2级);
        #   model     estimated 同源测量(不过低通)喂 BaseTrustEstimator,
        #             预测域用 predict_horizon 的逐时刻 Δ̂(t)(第3级)。
        self.declare_parameter('base_disturbance_amplitude_m', 0.0)
        self.declare_parameter('base_disturbance_seed', 0)
        # 扰动 profile 透传 base_disturbance_offset('harmonic'/'gust'/'fault'/
        # 'wideband')。demo 内三处调用(truth 信念/估计器测量/model 测量)与
        # trajectory_visualizer_demo 真值端必须配同一 profile。
        self.declare_parameter('base_disturbance_profile', 'harmonic')
        self.declare_parameter('base_belief_mode', 'truth')
        self.declare_parameter('base_estimator_noise_std_m', 0.0)
        self.declare_parameter('base_estimator_delay_ms', 0.0)
        self.declare_parameter('base_estimator_lpf_hz', 10.0)
        self.declare_parameter('base_estimator_seed', 1)
        # --- 第3级 model 信念(BaseTrustEstimator: 谱辨识谐波模型+保形校准) ---
        # trust_* 键名与 BaseTrustEstimator.__init__ 一致。默认值=离线调参结论:
        # n_peaks=12, lam=0.995, gamma_aci=0.005, n_cal=100; 其余照抄构造器默认。
        # 特殊约定:
        #   trust_meas_delay_sec < 0  -> 继承 base_estimator_delay_ms(换算秒);
        #   trust_dt_nominal    <= 0  -> 继承控制周期 1/control_rate_hz;
        #   trust_beta_sigma0   <= 0  -> None(构造器回退 sigma0)。
        # 二者继承值会除以 base_motion_time_scale: 估计器运行在缩放时间基
        # (与 _base_pose 内 scaled_t = t/scale 的扰动时间轴一致)。
        self.declare_parameter('trust_n_freq', 6)
        self.declare_parameter('trust_fmin_hz', 0.15)
        self.declare_parameter('trust_fmax_hz', 2.5)
        self.declare_parameter('trust_lam', 0.995)
        self.declare_parameter('trust_sigma0', 2.0e-3)
        self.declare_parameter('trust_prior_std', 1.0e-2)
        self.declare_parameter('trust_meas_delay_sec', -1.0)
        self.declare_parameter('trust_dt_nominal', -1.0)
        self.declare_parameter('trust_n_peaks', 12)
        self.declare_parameter('trust_ident_period_sec', 2.0)
        self.declare_parameter('trust_buffer_sec', 30.0)
        self.declare_parameter('trust_min_ident_sec', 2.5)
        self.declare_parameter('trust_holdout_sec', 1.0)
        self.declare_parameter('trust_f_ident_min_hz', 0.08)
        self.declare_parameter('trust_f_ident_max_hz', 5.0)
        self.declare_parameter('trust_peak_snr', 3.0)
        self.declare_parameter('trust_resid_lpf_hz', 0.0)
        self.declare_parameter('trust_beta_sigma0', -1.0)
        self.declare_parameter('trust_beta_gamma_f', 0.98)
        self.declare_parameter('trust_beta_n_grid', 15)
        # E[β] 在 degraded/healthy 两个物理精度锚点间连续标定为 b∈[0,1]。
        self.declare_parameter('trust_beta_degraded', 0.6)
        self.declare_parameter('trust_beta_healthy', 0.9)
        # B0--B4 消融/部署模式:
        # b0 无扰动前馈；b1 仅谐波点预测；b2 原 β 门控；b3 联合估计但
        # 固定控制器；b4 联合估计 + 风险决策 + NMPC/调速/守卫接口。
        self.declare_parameter('uncertainty_control_mode', 'b4')
        # 统一三维状态的两工况快速残差 IMM：瞬态衰减/持续随机游走。
        self.declare_parameter('trust_regime_enabled', True)
        self.declare_parameter('trust_regime_stable_tau_sec', 0.12)
        self.declare_parameter('trust_regime_stable_process_std', 8.0e-4)
        self.declare_parameter(
            'trust_regime_persistent_process_std', 2.5e-3)
        self.declare_parameter('trust_regime_measurement_std', -1.0)
        self.declare_parameter(
            'trust_regime_stable_stay_probability', 0.985)
        self.declare_parameter(
            'trust_regime_persistent_stay_probability', 0.97)
        self.declare_parameter(
            'trust_regime_initial_persistent_probability', 0.02)
        # B4 决策参数。β 表示参数模型适用程度，不直接压低预测均值；
        # 适用度下降时通过风险膨胀使控制决策更保守。
        self.declare_parameter('uncertainty_risk_weight', 0.25)
        self.declare_parameter('uncertainty_applicability_risk_gain', 1.0)
        self.declare_parameter('uncertainty_smoothing_weight', 2.5e-5)
        self.declare_parameter('uncertainty_gain_fall_tau_sec', 0.15)
        self.declare_parameter('uncertainty_gain_rise_tau_sec', 0.45)
        self.declare_parameter('uncertainty_risk_budget_m', 0.012)
        self.declare_parameter('uncertainty_min_speed_scale', 0.3)
        self.declare_parameter(
            'uncertainty_min_position_weight_scale', 0.35)
        self.declare_parameter('uncertainty_guard_radius_gain', 1.0)
        self.declare_parameter('uncertainty_guard_radius_max_m', 0.015)
        # β 加权模型前馈: b<=low 回到 nominal，b>=high 使用完整 Δ̂，
        # 中间 smoothstep 连续门控；b 先经非对称一阶滤波，过滤健康段的
        # 孤立低脉冲；关闭开关可生成严格的未门控消融对照。
        self.declare_parameter('trust_feedforward_gating_enabled', True)
        self.declare_parameter('trust_feedforward_gate_low', 0.2)
        self.declare_parameter('trust_feedforward_gate_high', 0.5)
        self.declare_parameter('trust_feedforward_gate_fall_tau_sec', 0.3)
        self.declare_parameter('trust_feedforward_gate_rise_tau_sec', 1.0)
        self.declare_parameter('trust_n_cal', 100)
        self.declare_parameter('trust_alpha', 0.1)
        self.declare_parameter('trust_gamma_aci', 0.005)
        # 谱辨识分摊调度(实时性): 重辨识严禁在 50Hz tick 内整块执行(实测
        # 30s 缓冲 ×3 轴 ×K=12 单块 150~250ms), 由估计器切成 ≤0.6ms 原子
        # 单元跨拍推进 —— 落笔拍每拍 ≤units_per_tick 单元且墙钟 ≥budget_ms
        # 即停; 抬笔/过渡拍放宽到 pen_up 档加速收敛。
        self.declare_parameter('trust_ident_units_per_tick', 5)
        self.declare_parameter('trust_ident_budget_ms', 1.8)
        self.declare_parameter('trust_ident_units_pen_up', 8)
        self.declare_parameter('trust_ident_budget_pen_up_ms', 2.6)
        # trust 影子模式: 无论 belief 模式为何, 逐拍把 (t, e_meas, Δ̂_1step, δ,
        # b, ||θ̂||) 写独立 CSV trust_shadow_<stamp>.csv(不动现有 loop CSV 列)。
        # 扰动关闭(amp=0)时不写、不建估计器(零开销早退)。
        self.declare_parameter('trust_shadow_csv', False)
        # 信任降速阶梯(实验开关, 默认关)。仅 base_belief_mode=model 且
        # online_time_scaling_enabled=true 时生效: 把 BaseTrustEstimator 每拍的
        # (b=健康后验概率, δ=delta_1step) 送入四态状态机。先从连续健康段
        # 学习 δ 噪声基线，之后按相对基线判故障，避免固定 4mm 阈值落在
        # 2mm 测量噪声地板内；b/δ 均使用分离的降级/恢复门形成迟滞。
        # 只影响图形路径时间推进速度; 不改任何关节速度/加速度硬约束, 不动守卫。
        self.declare_parameter('trust_ladder_enabled', False)
        self.declare_parameter('ladder_b_degraded', 0.3)
        self.declare_parameter('ladder_b_recover', 0.7)
        self.declare_parameter('ladder_delta_degraded_ratio', 1.35)
        self.declare_parameter('ladder_delta_recover_ratio', 1.2)
        self.declare_parameter('ladder_min_scale', 0.3)
        self.declare_parameter('ladder_baseline_warmup_sec', 4.0)
        self.declare_parameter('ladder_baseline_tau_sec', 20.0)
        self.declare_parameter('ladder_trust_commission_sec', 1.0)
        self.declare_parameter('ladder_recover_debounce_sec', 2.0)
        # 阶梯验收使用独占 scale 控制：只让 trust cap 决定目标速度，避免与
        # velocity/error 两个普通在线缩放启发式叠加后整程变慢、改变基座/路径
        # 相位。仍经过 OnlinePathTimeScaler 的 accel/decel 斜率限制；硬约束和
        # command guard 完全不变。关掉可恢复历史 min(velocity,error,cap) 组合。
        self.declare_parameter('ladder_exclusive_scale_control', True)
        # Deadline miss 恢复：迟到拍不能继续复用只推进了一个 dt 的 warm start /
        # commanded dq。阈值 1.5*dt=30ms，只影响异常拍，不改变正常控制。
        self.declare_parameter('late_tick_recovery_enabled', True)
        self.declare_parameter('late_tick_recovery_factor', 1.5)
        # 周期误差学习前馈（实验开关）。基座扰动是已知相位的 2 Hz 周期运动；
        # 用反馈 world 末端误差在线学习 sin/cos 系数，并把补偿加到 NMPC
        # 位置参考。它不增加预测域、不改变任何关节速度/加速度硬约束。
        self.declare_parameter('periodic_error_learning_enabled', False)
        self.declare_parameter('periodic_error_learning_gain', 0.5)
        self.declare_parameter(
            'periodic_error_correction_limit_m', 0.005)
        self.declare_parameter(
            'periodic_error_update_limit_m', 0.015)
        self.declare_parameter(
            'periodic_error_min_samples_per_cycle', 8)
        # 可加载一次独立标定得到的 3x2 world 系 [sin,cos] 系数。freeze=true
        # 时只使用固定前馈，不在 whole 单次路径中被非周期绘图误差污染。
        self.declare_parameter(
            'periodic_error_initial_coefficients', [0.0] * 6)
        self.declare_parameter('periodic_error_learning_freeze', False)
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
        # 带绝对时间戳发布路径时钟。可视化必须用该 path_time 选择图形/
        # 工作面朝向；墙上时间只驱动真实基座晃动。BEST_EFFORT 避免任何
        # 可视化订阅者反压 50Hz 控制回路。
        self.declare_parameter(
            'path_progress_topic', '/moving_base/path_progress')
        # /joint_states header 到本节点 callback 的投递延迟硬门。持续异常
        # 说明控制器正在重复使用陈旧状态；继续求解会形成 2Hz 螺旋，所得
        # 数据不可作为正式实验。默认启用，0.25s 去抖覆盖健康运行的孤立尖峰。
        self.declare_parameter('transport_delay_guard_enabled', True)
        self.declare_parameter('transport_delay_threshold_ms', 30.0)
        self.declare_parameter('transport_delay_fault_duration_sec', 0.25)
        self.declare_parameter('transport_delay_min_fresh_samples', 3)
        self.declare_parameter('feedback_stale_timeout_ms', 100.0)
        self.declare_parameter('command_fk_step_warn_m', 0.006)
        # 阶段1E: 期望运动感知守卫。
        #   legacy_fixed_step : 旧 6mm 固定阈值守卫,行为与阶段1E之前逐拍一致;
        #   expected_shadow   : 拦截仍完全用 legacy;额外只记录 expected-residual 诊断,
        #                       绝不改变 q/dq/warm_start/状态机;
        #   expected_residual : 以上一接受命令为锚点的残差拦截/投影;
        #   state_safe_filter : 生产默认,以预计生效状态为锚点并提供主动 backup。
        # 阈值来源: analyze_phase1e_guard.py 对阶段1D 三轮真实 D0(gain=0.7)数据的推导
        # (data/ab_runs_1e/phase1e_d0_residual_analysis.json):
        #   r_abs = ceil(max(2×P99.9=6.74mm, 1.5×max=5.82mm, 0.5mm)) = 6.8mm;
        #   r_rel = 0.5(D0 单速工作点不可辨识速度比例项,建模裕量,多速度数据后重推);
        #   r_acc = 1.0(解析曲率覆盖项,D0 工况贡献≈0.08mm)。
        # D0 回放: 误拦截 0,最小裕量 4.27mm,max r_allow 8.23mm(20mm 注入跳变 2.4x 拦截余量)。
        # state_safe_filter: 以预测的当前状态为锚点，超限时沿当前状态投影；
        # 若 MPC 方向仍不可行，则用阻尼最小二乘主动维持 world 目标，避免 hold 死锁。
        self.declare_parameter('guard_mode', 'state_safe_filter')
        self.declare_parameter('guard_residual_abs_m', 0.0068)
        self.declare_parameter('guard_residual_relative_gain', 0.5)
        self.declare_parameter('guard_residual_accel_gain', 1.0)
        self.declare_parameter('guard_projection_enabled', True)
        self.declare_parameter('guard_projection_min_lambda', 0.05)
        self.declare_parameter('guard_projection_max_iterations', 8)
        self.declare_parameter('guard_max_consecutive_blocks', 25)
        self.declare_parameter('guard_backup_damping', 0.03)
        self.declare_parameter('guard_backup_position_gain', 0.8)
        # 生产默认值(阶段 1A/1B/1C 均已 Codex 正式验收):普通启动直接用已验收配置。
        # reference_lead_steps=1(阶段1A 参考前移)/=0 旧语义仅回归与 A/B 用。
        self.declare_parameter('reference_lead_steps', 1)
        # 基座解析位姿与末端绘图路径可使用不同的预瞄。-1=继承
        # reference_lead_steps；显式 2 可在保持末端路径 1 拍预瞄时，单独补偿
        # 2 Hz 基座/执行链相位滞后。
        self.declare_parameter('base_reference_lead_steps', -1)
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
        self.second_order_cartesian_residual_mode = str(
            self.get_parameter('second_order_cartesian_residual_mode').value)
        self.second_order_cartesian_velocity_weight = float(
            self.get_parameter('second_order_cartesian_velocity_weight').value)
        self.second_order_cartesian_acceleration_weight = float(
            self.get_parameter('second_order_cartesian_acceleration_weight').value)
        self.second_order_terminal_cartesian_velocity_weight = float(
            self.get_parameter(
                'second_order_terminal_cartesian_velocity_weight').value)
        self.second_order_feasible_reference_projection = bool(
            self.get_parameter(
                'second_order_feasible_reference_projection').value)
        self.second_order_feasible_reference_damping = float(
            self.get_parameter(
                'second_order_feasible_reference_damping').value)
        self.second_order_contour_tracking_enabled = bool(
            self.get_parameter(
                'second_order_contour_tracking_enabled').value)
        self.second_order_contour_position_weight = float(
            self.get_parameter('second_order_contour_position_weight').value)
        self.second_order_lag_position_weight = float(
            self.get_parameter('second_order_lag_position_weight').value)
        self.second_order_transition_position_weight = float(
            self.get_parameter('second_order_transition_position_weight').value)
        self.second_order_terminal_contour_position_weight = float(
            self.get_parameter(
                'second_order_terminal_contour_position_weight').value)
        self.second_order_terminal_lag_position_weight = float(
            self.get_parameter(
                'second_order_terminal_lag_position_weight').value)
        self.second_order_terminal_transition_position_weight = float(
            self.get_parameter(
                'second_order_terminal_transition_position_weight').value)
        self.reference_derivative_dt_sec = float(
            self.get_parameter('reference_derivative_dt_sec').value)
        self.second_order_velocity_weight = float(
            self.get_parameter('second_order_velocity_weight').value)
        self.second_order_acceleration_weight = float(
            self.get_parameter('second_order_acceleration_weight').value)
        self.second_order_terminal_velocity_weight = float(
            self.get_parameter('second_order_terminal_velocity_weight').value)
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
        self.latency_compensation_enabled = bool(
            self.get_parameter('latency_compensation_enabled').value)
        self.latency_nominal_publish_sec = 1e-3 * float(
            self.get_parameter('latency_nominal_publish_ms').value)
        self.latency_publish_ewma_alpha = float(
            self.get_parameter('latency_publish_ewma_alpha').value)
        self.latency_actuator_delay_sec = 1e-3 * float(
            self.get_parameter('latency_actuator_delay_ms').value)
        self.latency_prediction_max_sec = 1e-3 * float(
            self.get_parameter('latency_prediction_max_ms').value)
        self.smith_predictor_enabled = bool(
            self.get_parameter('smith_predictor_enabled').value)
        self.command_inversion_enabled = bool(
            self.get_parameter('command_inversion_enabled').value)
        self.plant_velocity_feedforward_gain = float(
            self.get_parameter('plant_velocity_feedforward_gain').value)
        self.plant_position_feedback_tau = float(
            self.get_parameter('plant_position_feedback_tau').value)
        self.plant_feedback_velocity_limit = float(
            self.get_parameter('plant_feedback_velocity_limit').value)
        self.plant_max_velocity = float(
            self.get_parameter('plant_max_velocity').value)
        self.plant_max_acceleration = float(
            self.get_parameter('plant_max_acceleration').value)
        self.plant_control_substep_dt = float(
            self.get_parameter('plant_control_substep_dt').value)
        if (self.plant_velocity_feedforward_gain <= 0.0
                or self.plant_position_feedback_tau <= 0.0
                or self.plant_feedback_velocity_limit < 0.0
                or self.plant_max_velocity <= 0.0
                or self.plant_max_acceleration <= 0.0
                or self.plant_control_substep_dt <= 0.0):
            raise ValueError('plant_* 执行律参数必须为正(见 student_arm.launch.py)')
        self.base_disturbance_amplitude_m = float(
            self.get_parameter('base_disturbance_amplitude_m').value)
        self.base_disturbance_seed = int(
            self.get_parameter('base_disturbance_seed').value)
        self.base_disturbance_profile = str(
            self.get_parameter('base_disturbance_profile').value
        ).strip().lower()
        self.base_belief_mode = str(
            self.get_parameter('base_belief_mode').value).strip().lower()
        self.base_estimator_noise_std_m = float(
            self.get_parameter('base_estimator_noise_std_m').value)
        self.base_estimator_delay_sec = 1e-3 * float(
            self.get_parameter('base_estimator_delay_ms').value)
        self.base_estimator_lpf_hz = float(
            self.get_parameter('base_estimator_lpf_hz').value)
        self.base_estimator_seed = int(
            self.get_parameter('base_estimator_seed').value)
        self.trust_n_freq = int(self.get_parameter('trust_n_freq').value)
        self.trust_fmin_hz = float(self.get_parameter('trust_fmin_hz').value)
        self.trust_fmax_hz = float(self.get_parameter('trust_fmax_hz').value)
        self.trust_lam = float(self.get_parameter('trust_lam').value)
        self.trust_sigma0 = float(self.get_parameter('trust_sigma0').value)
        self.trust_prior_std = float(
            self.get_parameter('trust_prior_std').value)
        self.trust_meas_delay_sec = float(
            self.get_parameter('trust_meas_delay_sec').value)
        self.trust_dt_nominal = float(
            self.get_parameter('trust_dt_nominal').value)
        self.trust_n_peaks = int(self.get_parameter('trust_n_peaks').value)
        self.trust_ident_period_sec = float(
            self.get_parameter('trust_ident_period_sec').value)
        self.trust_buffer_sec = float(
            self.get_parameter('trust_buffer_sec').value)
        self.trust_min_ident_sec = float(
            self.get_parameter('trust_min_ident_sec').value)
        self.trust_holdout_sec = float(
            self.get_parameter('trust_holdout_sec').value)
        self.trust_f_ident_min_hz = float(
            self.get_parameter('trust_f_ident_min_hz').value)
        self.trust_f_ident_max_hz = float(
            self.get_parameter('trust_f_ident_max_hz').value)
        self.trust_peak_snr = float(self.get_parameter('trust_peak_snr').value)
        self.trust_resid_lpf_hz = float(
            self.get_parameter('trust_resid_lpf_hz').value)
        self.trust_beta_sigma0 = float(
            self.get_parameter('trust_beta_sigma0').value)
        self.trust_beta_gamma_f = float(
            self.get_parameter('trust_beta_gamma_f').value)
        self.trust_beta_n_grid = int(
            self.get_parameter('trust_beta_n_grid').value)
        self.trust_beta_degraded = float(
            self.get_parameter('trust_beta_degraded').value)
        self.trust_beta_healthy = float(
            self.get_parameter('trust_beta_healthy').value)
        self.uncertainty_control_mode = str(
            self.get_parameter('uncertainty_control_mode').value
        ).strip().lower()
        self.trust_regime_enabled = bool(
            self.get_parameter('trust_regime_enabled').value)
        self.trust_regime_stable_tau_sec = float(
            self.get_parameter('trust_regime_stable_tau_sec').value)
        self.trust_regime_stable_process_std = float(
            self.get_parameter('trust_regime_stable_process_std').value)
        self.trust_regime_persistent_process_std = float(
            self.get_parameter(
                'trust_regime_persistent_process_std').value)
        self.trust_regime_measurement_std = float(
            self.get_parameter('trust_regime_measurement_std').value)
        self.trust_regime_stable_stay_probability = float(
            self.get_parameter(
                'trust_regime_stable_stay_probability').value)
        self.trust_regime_persistent_stay_probability = float(
            self.get_parameter(
                'trust_regime_persistent_stay_probability').value)
        self.trust_regime_initial_persistent_probability = float(
            self.get_parameter(
                'trust_regime_initial_persistent_probability').value)
        self.uncertainty_risk_weight = float(
            self.get_parameter('uncertainty_risk_weight').value)
        self.uncertainty_applicability_risk_gain = float(
            self.get_parameter(
                'uncertainty_applicability_risk_gain').value)
        self.uncertainty_smoothing_weight = float(
            self.get_parameter('uncertainty_smoothing_weight').value)
        self.uncertainty_gain_fall_tau_sec = float(
            self.get_parameter('uncertainty_gain_fall_tau_sec').value)
        self.uncertainty_gain_rise_tau_sec = float(
            self.get_parameter('uncertainty_gain_rise_tau_sec').value)
        self.uncertainty_risk_budget_m = float(
            self.get_parameter('uncertainty_risk_budget_m').value)
        self.uncertainty_min_speed_scale = float(
            self.get_parameter('uncertainty_min_speed_scale').value)
        self.uncertainty_min_position_weight_scale = float(
            self.get_parameter(
                'uncertainty_min_position_weight_scale').value)
        self.uncertainty_guard_radius_gain = float(
            self.get_parameter('uncertainty_guard_radius_gain').value)
        self.uncertainty_guard_radius_max_m = float(
            self.get_parameter('uncertainty_guard_radius_max_m').value)
        self.trust_feedforward_gating_enabled = bool(
            self.get_parameter('trust_feedforward_gating_enabled').value)
        self.trust_feedforward_gate_low = float(
            self.get_parameter('trust_feedforward_gate_low').value)
        self.trust_feedforward_gate_high = float(
            self.get_parameter('trust_feedforward_gate_high').value)
        self.trust_feedforward_gate_fall_tau_sec = float(
            self.get_parameter(
                'trust_feedforward_gate_fall_tau_sec').value)
        self.trust_feedforward_gate_rise_tau_sec = float(
            self.get_parameter(
                'trust_feedforward_gate_rise_tau_sec').value)
        self.trust_n_cal = int(self.get_parameter('trust_n_cal').value)
        self.trust_alpha = float(self.get_parameter('trust_alpha').value)
        self.trust_gamma_aci = float(
            self.get_parameter('trust_gamma_aci').value)
        self.trust_ident_units_per_tick = int(
            self.get_parameter('trust_ident_units_per_tick').value)
        self.trust_ident_budget_ms = float(
            self.get_parameter('trust_ident_budget_ms').value)
        self.trust_ident_units_pen_up = int(
            self.get_parameter('trust_ident_units_pen_up').value)
        self.trust_ident_budget_pen_up_ms = float(
            self.get_parameter('trust_ident_budget_pen_up_ms').value)
        self.trust_shadow_csv = bool(
            self.get_parameter('trust_shadow_csv').value)
        self.trust_ladder_enabled = bool(
            self.get_parameter('trust_ladder_enabled').value)
        self.ladder_b_degraded = float(
            self.get_parameter('ladder_b_degraded').value)
        self.ladder_b_recover = float(
            self.get_parameter('ladder_b_recover').value)
        self.ladder_delta_degraded_ratio = float(
            self.get_parameter('ladder_delta_degraded_ratio').value)
        self.ladder_delta_recover_ratio = float(
            self.get_parameter('ladder_delta_recover_ratio').value)
        self.ladder_min_scale = float(
            self.get_parameter('ladder_min_scale').value)
        self.ladder_baseline_warmup_sec = float(
            self.get_parameter('ladder_baseline_warmup_sec').value)
        self.ladder_baseline_tau_sec = float(
            self.get_parameter('ladder_baseline_tau_sec').value)
        self.ladder_trust_commission_sec = float(
            self.get_parameter('ladder_trust_commission_sec').value)
        self.ladder_recover_debounce_sec = float(
            self.get_parameter('ladder_recover_debounce_sec').value)
        self.ladder_exclusive_scale_control = bool(
            self.get_parameter('ladder_exclusive_scale_control').value)
        # 两个配置对象集中做完整 fail-fast 参数校验。
        trust_feedforward_gain(
            0.5, self.trust_feedforward_gate_low,
            self.trust_feedforward_gate_high)
        if (self.trust_feedforward_gate_fall_tau_sec <= 0.0
                or self.trust_feedforward_gate_rise_tau_sec <= 0.0):
            raise ValueError('trust feedforward gate 时间常数必须为正')
        if self.uncertainty_control_mode not in (
                'b0', 'b1', 'b2', 'b3', 'b4'):
            raise ValueError(
                "uncertainty_control_mode 必须是 b0/b1/b2/b3/b4")
        # 构造一次以集中校验 B4 决策参数；运行态在 reset 时重建。
        UncertaintyAwareFeedforwardDecision(
            applicability_low=self.trust_feedforward_gate_low,
            applicability_high=self.trust_feedforward_gate_high,
            risk_weight=self.uncertainty_risk_weight,
            applicability_risk_gain=(
                self.uncertainty_applicability_risk_gain),
            smoothing_weight=self.uncertainty_smoothing_weight,
            fall_tau_sec=self.uncertainty_gain_fall_tau_sec,
            rise_tau_sec=self.uncertainty_gain_rise_tau_sec,
            risk_budget_m=self.uncertainty_risk_budget_m,
            min_speed_scale=self.uncertainty_min_speed_scale,
            min_position_weight_scale=(
                self.uncertainty_min_position_weight_scale),
        )
        regime_positive = np.asarray([
            self.trust_regime_stable_tau_sec,
            self.trust_regime_stable_process_std,
            self.trust_regime_persistent_process_std,
            (self.trust_sigma0 if self.trust_regime_measurement_std <= 0.0
             else self.trust_regime_measurement_std),
            self.uncertainty_guard_radius_gain,
            self.uncertainty_guard_radius_max_m,
            self.uncertainty_applicability_risk_gain,
        ], dtype=float)
        regime_probabilities = np.asarray([
            self.trust_regime_stable_stay_probability,
            self.trust_regime_persistent_stay_probability,
            self.trust_regime_initial_persistent_probability,
        ], dtype=float)
        if (not np.all(np.isfinite(regime_positive))
                or np.any(regime_positive <= 0.0)
                or not np.all(np.isfinite(regime_probabilities))
                or not 0.0 < regime_probabilities[0] < 1.0
                or not 0.0 < regime_probabilities[1] < 1.0
                or not 0.0 <= regime_probabilities[2] <= 1.0):
            raise ValueError('two-regime/B4 uncertainty parameters are invalid')
        TrustLadderConfig(
            b_degraded=self.ladder_b_degraded,
            b_recover=self.ladder_b_recover,
            delta_degraded_ratio=self.ladder_delta_degraded_ratio,
            delta_recover_ratio=self.ladder_delta_recover_ratio,
            min_scale=self.ladder_min_scale,
            baseline_warmup_sec=self.ladder_baseline_warmup_sec,
            baseline_tau_sec=self.ladder_baseline_tau_sec,
            trust_commission_sec=self.ladder_trust_commission_sec,
            recover_debounce_sec=self.ladder_recover_debounce_sec,
        )
        if self.base_belief_mode not in (
                'nominal', 'truth', 'estimated', 'model'):
            raise ValueError(
                "base_belief_mode 必须是 'nominal'/'truth'/'estimated'/'model'")
        if (self.base_disturbance_amplitude_m < 0.0
                or self.base_estimator_noise_std_m < 0.0
                or self.base_estimator_delay_sec < 0.0
                or self.base_estimator_lpf_hz < 0.0):
            raise ValueError('base 扰动/估计器参数必须 >= 0')
        # fail-fast: 未知 profile 立即报错(合法枚举以 base_disturbance_offset
        # 为准, 探针调用与运行时同一代码路径; amp 传正值绕过其 0 幅早退)。
        traj.base_disturbance_offset(
            0.0, 1.0, self.base_disturbance_seed,
            profile=self.base_disturbance_profile)
        self.late_tick_recovery_enabled = bool(
            self.get_parameter('late_tick_recovery_enabled').value)
        self.late_tick_recovery_factor = float(
            self.get_parameter('late_tick_recovery_factor').value)
        self.periodic_error_learning_enabled = bool(
            self.get_parameter('periodic_error_learning_enabled').value)
        self.periodic_error_learning_gain = float(
            self.get_parameter('periodic_error_learning_gain').value)
        self.periodic_error_correction_limit_m = float(
            self.get_parameter(
                'periodic_error_correction_limit_m').value)
        self.periodic_error_update_limit_m = float(
            self.get_parameter('periodic_error_update_limit_m').value)
        self.periodic_error_min_samples_per_cycle = int(
            self.get_parameter(
                'periodic_error_min_samples_per_cycle').value)
        self.periodic_error_initial_coefficients = np.asarray(
            self.get_parameter(
                'periodic_error_initial_coefficients').value,
            dtype=float,
        )
        self.periodic_error_learning_freeze = bool(
            self.get_parameter('periodic_error_learning_freeze').value)
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
        if self.trust_ladder_enabled and not self.online_time_scaling_enabled:
            # 阶梯的唯一执行通道就是图形路径时间缩放; 缺通道时 fail-fast,
            # 避免"开关打开却静默无效"的假阳性实验。
            raise ValueError(
                'trust_ladder_enabled:=true 需要 '
                'online_time_scaling_enabled:=true')
        if self.trust_ladder_enabled and self.base_belief_mode != 'model':
            self.get_logger().warn(
                'trust_ladder_enabled 已开启但 base_belief_mode != model: '
                '阶梯不生效(scale cap 恒为 1)')
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
        self.path_progress_topic = str(
            self.get_parameter('path_progress_topic').value)
        self.transport_delay_guard_enabled = bool(
            self.get_parameter('transport_delay_guard_enabled').value)
        self.transport_delay_threshold_ms = float(
            self.get_parameter('transport_delay_threshold_ms').value)
        self.transport_delay_fault_duration_sec = float(
            self.get_parameter('transport_delay_fault_duration_sec').value)
        self.transport_delay_min_fresh_samples = int(
            self.get_parameter('transport_delay_min_fresh_samples').value)
        self.feedback_stale_timeout_ms = float(
            self.get_parameter('feedback_stale_timeout_ms').value)
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
        self.guard_backup_damping = float(
            self.get_parameter('guard_backup_damping').value)
        self.guard_backup_position_gain = float(
            self.get_parameter('guard_backup_position_gain').value)
        self.reference_lead_steps = int(self.get_parameter('reference_lead_steps').value)
        self.base_reference_lead_steps = int(
            self.get_parameter('base_reference_lead_steps').value)
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
        if self.base_reference_lead_steps < -1:
            raise ValueError(
                'base_reference_lead_steps must be >= -1 (-1=继承)')
        if self.dq_prev_mode not in ('zero', 'measured_delta'):
            raise ValueError("dq_prev_mode must be 'zero' or 'measured_delta'")
        if self.rti_iteration_mode not in ('legacy_early_exit', 'fixed_iterations'):
            raise ValueError("rti_iteration_mode must be 'legacy_early_exit' or 'fixed_iterations'")
        if not self.path_progress_topic:
            raise ValueError('path_progress_topic must not be empty')
        if (
            not math.isfinite(self.transport_delay_threshold_ms)
            or self.transport_delay_threshold_ms <= 0.0
        ):
            raise ValueError(
                'transport_delay_threshold_ms must be finite and > 0')
        if (
            not math.isfinite(self.transport_delay_fault_duration_sec)
            or self.transport_delay_fault_duration_sec <= 0.0
        ):
            raise ValueError(
                'transport_delay_fault_duration_sec must be finite and > 0')
        if self.transport_delay_min_fresh_samples < 1:
            raise ValueError(
                'transport_delay_min_fresh_samples must be >= 1')
        if (
            not math.isfinite(self.feedback_stale_timeout_ms)
            or self.feedback_stale_timeout_ms <= 0.0
        ):
            raise ValueError(
                'feedback_stale_timeout_ms must be finite and > 0')
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
        if self.second_order_cartesian_residual_mode not in (
                'external_affine', 'nonlinear'):
            raise ValueError(
                "second_order_cartesian_residual_mode must be "
                "'external_affine'|'nonlinear'")
        if not (
            self.latency_nominal_publish_sec >= 0.0
            and 0.0 < self.latency_publish_ewma_alpha <= 1.0
            and self.latency_actuator_delay_sec >= 0.0
            and self.latency_prediction_max_sec > 0.0
        ):
            raise ValueError('latency compensation parameters are invalid')
        if (
            not math.isfinite(self.late_tick_recovery_factor)
            or self.late_tick_recovery_factor < 1.0
        ):
            raise ValueError(
                'late_tick_recovery_factor must be finite and >= 1.0')
        if (
            not math.isfinite(self.periodic_error_learning_gain)
            or not 0.0 < self.periodic_error_learning_gain <= 1.0
            or not math.isfinite(self.periodic_error_correction_limit_m)
            or self.periodic_error_correction_limit_m <= 0.0
            or not math.isfinite(self.periodic_error_update_limit_m)
            or self.periodic_error_update_limit_m <= 0.0
            or self.periodic_error_min_samples_per_cycle < 1
            or self.periodic_error_initial_coefficients.shape != (6,)
            or not np.all(np.isfinite(
                self.periodic_error_initial_coefficients))
        ):
            raise ValueError(
                'periodic error learning requires gain in (0,1], positive '
                'finite correction/update limits, and min samples >= 1')
        if (
            self.periodic_error_learning_enabled
            and self.online_time_scaling_enabled
        ):
            raise ValueError(
                'periodic_error_learning_enabled currently requires '
                'online_time_scaling_enabled:=false')
        second_order_weights = np.asarray((
            self.second_order_position_weight,
            self.second_order_terminal_position_weight,
            self.second_order_cartesian_velocity_weight,
            self.second_order_cartesian_acceleration_weight,
            self.second_order_terminal_cartesian_velocity_weight,
            self.second_order_contour_position_weight,
            self.second_order_lag_position_weight,
            self.second_order_transition_position_weight,
            self.second_order_terminal_contour_position_weight,
            self.second_order_terminal_lag_position_weight,
            self.second_order_terminal_transition_position_weight,
            self.second_order_velocity_weight,
            self.second_order_acceleration_weight,
            self.second_order_terminal_velocity_weight,
            self.second_order_current_posture_weight,
            self.second_order_center_posture_weight,
        ), dtype=float)
        if (
            not np.all(np.isfinite(second_order_weights))
            or np.any(second_order_weights < 0.0)
        ):
            raise ValueError('second-order weights must be finite and non-negative')
        if (
            not math.isfinite(self.second_order_feasible_reference_damping)
            or self.second_order_feasible_reference_damping <= 0.0
        ):
            raise ValueError(
                'second_order_feasible_reference_damping must be finite and > 0')
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
        if (
            not math.isfinite(self.guard_backup_damping)
            or self.guard_backup_damping <= 0.0
            or not math.isfinite(self.guard_backup_position_gain)
            or not 0.0 < self.guard_backup_position_gain <= 1.0
        ):
            raise ValueError(
                'guard_backup_damping must be >0 and '
                'guard_backup_position_gain must be in (0,1]')

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
        self._guard_path_t_last_reference = float('nan')
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
        # callback 写入完整不可变样本后再原子替换引用；控制线程永远不会混用
        # 不同 JointState 消息里的 q/dq/stamp。
        self._latest_feedback_sample = None
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
        self._so_last_cmd_ddq = None     # 生效时刻状态预测: 上一拍实际接受的模型加速度
        # Smith 预测器: 最近实际发布的命令 (publish_wall_sec, q_pub, dq_pub)。
        # 只在 _publish_joint_positions 内追加,保证与 plant 收到的命令一致
        # (含 DART/守卫 hold/reset 阶段命令)。
        self._cmd_history = deque(maxlen=16)
        # /joint_states 传输延迟自检: recv_wall - header.stamp 持续 >30ms 说明
        # DDS 投递异常(典型原因: /dev/shm 残留 fastrtps 段使 SHM 传输失效,
        # 回落 UDP 心跳修复路径, 恒定 ~100ms 延迟)。陈旧反馈会让守卫大量投影、
        # 基座补偿失效, 实测把 whole RMS 从 0.27mm 打到 20mm——必须显式报警。
        self._transport_gap_bad_ticks = 0
        self._transport_gap_warned = False
        self._transport_gap_bad_duration_sec = 0.0
        self._transport_gap_distinct_bad_samples = 0
        self._transport_gap_last_bad_stamp = float('nan')
        self._transport_gap_bad_active = False
        self.transport_fault = False
        self.transport_fault_gap_ms = float('nan')
        self._fault_path_time = float('nan')
        self._params_snapshot_path = None
        # 基座不确定性实验: estimated 模式的信念偏移状态与噪声源。
        self._base_belief_offset = np.zeros(3)
        self._base_estimator_rng = np.random.default_rng(
            self.base_estimator_seed)
        self._last_reference_kinematics = {}
        self._publish_latency_ewma_sec = self.latency_nominal_publish_sec
        self._last_application_delay_sec = 0.0
        self._last_state_prediction_age_sec = 0.0
        self._last_path_update_wall = None
        self.late_tick_recovery_count = 0
        # key -> (3 world axes, sin/cos) coefficients. whole 模式按五个
        # 工作面分组，避免正面/侧面/顶部的补偿相互污染。
        self._periodic_error_coefficients = {}
        self._periodic_error_accumulators = {}
        self._last_periodic_learning_diag = {}
        self._reset_periodic_error_learning()
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
        path_progress_qos = QoSProfile(depth=8)
        path_progress_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        path_progress_qos.durability = DurabilityPolicy.VOLATILE
        self.path_progress_pub = self.create_publisher(
            Vector3Stamped, self.path_progress_topic, path_progress_qos)
        # depth=1 是控制反馈的正确语义：若 CPU 短暂繁忙，丢弃旧状态而不是
        # 排队后让 NMPC 逐条处理过期样本。
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_callback,
            1,
        )

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        if not (10.0 <= self.control_rate_hz <= 200.0):
            raise ValueError('control_rate_hz must be in [10, 200]')
        self.dt = 1.0 / self.control_rate_hz
        if not (0.0 < self.reference_derivative_dt_sec <= 0.5 * self.dt):
            raise ValueError(
                'reference_derivative_dt_sec must be in (0, 0.5*control_dt]')
        # --- 第3级 model 信念 / trust 影子通道状态 ---
        # 预测网格覆盖本拍所有 _base_pose 查询时刻: 回看 n_back 拍(反馈时刻/
        # 微分步长), 前瞻 lead+horizon+延迟补偿上限+2 拍余量。model 档的
        # _base_pose(t) 只在该网格上线性插值, 拍内绝不重复 predict_horizon。
        trust_n_back = 8
        trust_n_fwd = (
            max(self.reference_lead_steps,
                self.base_reference_lead_steps,
                self.second_order_reference_lead_steps,
                0)
            + max(self.horizon_steps, self.second_order_horizon_steps)
            + int(math.ceil(self.latency_prediction_max_sec / self.dt))
            + 2
        )
        self._trust_grid_offsets = self.dt * np.arange(
            -trust_n_back, trust_n_fwd + 1)
        self._trust_grid_n_fwd = int(trust_n_fwd)
        self._trust_grid_1step_index = trust_n_back + 1  # 网格上 t_solve+dt
        self._trust_file = None
        self._trust_writer = None
        self._trust_rows_since_flush = 0
        self._trust_record_failed = False
        self._trust_estimator = None
        self._trust_meas_rng = None
        self._trust_pred_times = None
        self._trust_pred_delta = None
        self._trust_pred_delta_raw = None
        self._trust_pred_delta_cal = float('nan')
        # amp=0 时不构造估计器(零开销); amp>0 且 model/影子开启时在此 fail-fast
        # 校验全部 trust_* 构造参数。
        self._reset_trust_estimator()
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
                cartesian_residual_mode=(
                    self.second_order_cartesian_residual_mode),
                cartesian_velocity_weight=(
                    self.second_order_cartesian_velocity_weight),
                cartesian_acceleration_weight=(
                    self.second_order_cartesian_acceleration_weight),
                terminal_cartesian_velocity_weight=(
                    self.second_order_terminal_cartesian_velocity_weight),
                feasible_reference_projection=(
                    self.second_order_feasible_reference_projection),
                feasible_reference_damping=(
                    self.second_order_feasible_reference_damping),
                contour_tracking_enabled=(
                    self.second_order_contour_tracking_enabled),
                contour_position_weight=(
                    self.second_order_contour_position_weight),
                lag_position_weight=self.second_order_lag_position_weight,
                transition_position_weight=(
                    self.second_order_transition_position_weight),
                terminal_contour_position_weight=(
                    self.second_order_terminal_contour_position_weight),
                terminal_lag_position_weight=(
                    self.second_order_terminal_lag_position_weight),
                terminal_transition_position_weight=(
                    self.second_order_terminal_transition_position_weight),
                orientation_weight=self.orientation_weight,
                terminal_orientation_weight=self.terminal_orientation_weight,
                current_posture_weight=self.second_order_current_posture_weight,
                center_posture_weight=self.second_order_center_posture_weight,
                joint_velocity_weight=self.second_order_velocity_weight,
                joint_acceleration_weight=self.second_order_acceleration_weight,
                terminal_joint_velocity_weight=(
                    self.second_order_terminal_velocity_weight),
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
        q_measured = np.array(
            [name_to_pos[name] for name in joint_names], dtype=float)
        if len(msg.velocity) == len(msg.name):
            name_to_vel = dict(zip(msg.name, msg.velocity))
            dq_measured = np.array(
                [name_to_vel[name] for name in joint_names], dtype=float)
        else:
            dq_measured = None
        # 观测:记录该反馈的 header 时间戳(发送端时钟)与本机接收 wall 时间。
        stamp = msg.header.stamp
        header_stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        recv_wall_sec = self.get_clock().now().nanoseconds * 1e-9
        sample = (
            q_measured,
            dq_measured,
            header_stamp_sec,
            recv_wall_sec,
        )
        # Python 对象引用替换在 GIL 下是原子的。先发布完整 sample，再维护旧字段
        # 以兼容 reset/可视化和已有离线测试。
        self._latest_feedback_sample = sample
        self.q_measured = q_measured
        self.dq_measured = dq_measured
        self._js_header_stamp_sec = header_stamp_sec
        self._js_recv_wall_sec = recv_wall_sec

    def _feedback_snapshot(self):
        """Return one internally consistent JointState sample for a control tick."""
        sample = getattr(self, '_latest_feedback_sample', None)
        if sample is None:
            q = getattr(self, 'q_measured', None)
            dq = getattr(self, 'dq_measured', None)
            return (
                None if q is None else np.asarray(q, dtype=float).copy(),
                None if dq is None else np.asarray(dq, dtype=float).copy(),
                float(getattr(self, '_js_header_stamp_sec', float('nan'))),
                float(getattr(self, '_js_recv_wall_sec', float('nan'))),
            )
        q, dq, stamp, recv = sample
        return (
            np.asarray(q, dtype=float).copy(),
            None if dq is None else np.asarray(dq, dtype=float).copy(),
            float(stamp),
            float(recv),
        )

    def _path_time_at_offset(
            self, t_solve: float, offset_sec: float = 0.0) -> float:
        """Map a future wall/base-clock offset to the controller path clock."""
        offset = max(0.0, float(offset_sec))
        if bool(getattr(self, 'online_time_scaling_enabled', False)):
            return float(self.time_scaler.preview([offset])[0])
        return float(t_solve) + offset

    def _publish_path_progress(
            self, stamp_wall_sec: float, t_solve: float,
            path_time: float = None, path_scale: float = None):
        """Publish the exact clock mapping used by references and base yaw.

        Vector fields are deliberately fixed and documented:
          x = path_time, y = d(path_time)/d(wall_time), z = t_solve.
        The absolute ROS stamp lets consumers interpolate path_time for each
        JointState/command stamp instead of using message arrival time.
        """
        if path_time is None:
            path_time = self._path_time_at_offset(t_solve, 0.0)
        if path_scale is None:
            path_scale = (
                float(self.time_scaler.scale)
                if self.online_time_scaling_enabled else 1.0)
        msg = Vector3Stamped()
        stamp_ns = max(0, int(round(float(stamp_wall_sec) * 1e9)))
        msg.header.stamp.sec = stamp_ns // 1_000_000_000
        msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
        msg.header.frame_id = (
            f'moving_base_path_clock_{self._run_stamp}')
        msg.vector.x = float(path_time)
        msg.vector.y = float(path_scale)
        msg.vector.z = float(t_solve)
        self.path_progress_pub.publish(msg)

    def _update_transport_delay_guard(
            self, transport_gap_ms: float, elapsed_sec: float,
            feedback_stamp_sec: float,
            feedback_age_ms: float = 0.0) -> bool:
        """Return True once stale feedback has persisted long enough to abort."""
        if not self.transport_delay_guard_enabled:
            return False
        valid_source_stamp = (
            math.isfinite(feedback_stamp_sec)
            and feedback_stamp_sec > 0.0)
        bad_transport = (
            not valid_source_stamp
            or not math.isfinite(transport_gap_ms)
            or abs(transport_gap_ms) > self.transport_delay_threshold_ms
        )
        stale_local = (
            math.isfinite(feedback_age_ms)
            and feedback_age_ms > self.feedback_stale_timeout_ms
        )
        # No callback has delivered a new sample for the configured timeout.
        # This is already a direct safety fault; waiting a second debounce
        # interval would keep solving against a known-stale state.
        if stale_local:
            return True
        bad = bad_transport or stale_local
        if not bad:
            self._transport_gap_bad_duration_sec = 0.0
            self._transport_gap_distinct_bad_samples = 0
            self._transport_gap_last_bad_stamp = float('nan')
            self._transport_gap_bad_active = False
            return False

        # The first bad observation starts the stopwatch at zero. Subsequent
        # control intervals accrue duration, so "0.25 s" means actual observed
        # persistence rather than 0.25 s including a fictitious interval
        # before the first bad sample.
        dt = float(elapsed_sec)
        if not math.isfinite(dt) or dt <= 0.0:
            dt = float(self.dt)
        if self._transport_gap_bad_active:
            self._transport_gap_bad_duration_sec += min(dt, 0.1)
        else:
            self._transport_gap_bad_active = True
            self._transport_gap_bad_duration_sec = 0.0
        if (
            valid_source_stamp
            and (
                not math.isfinite(self._transport_gap_last_bad_stamp)
                or abs(
                    float(feedback_stamp_sec)
                    - self._transport_gap_last_bad_stamp
                ) > 1e-9
            )
        ):
            self._transport_gap_distinct_bad_samples += 1
            self._transport_gap_last_bad_stamp = float(feedback_stamp_sec)
        enough_distinct = (
            self._transport_gap_distinct_bad_samples
            >= self.transport_delay_min_fresh_samples
            or not valid_source_stamp)
        return (
            self._transport_gap_bad_duration_sec
            >= self.transport_delay_fault_duration_sec
            and enough_distinct
        )

    def _transport_fault_reason(
            self, transport_gap_ms: float,
            feedback_age_ms: float,
            feedback_stamp_sec: float) -> str:
        if (
            math.isfinite(feedback_age_ms)
            and feedback_age_ms > self.feedback_stale_timeout_ms
        ):
            return 'joint_states_feedback_stale'
        if (
            not math.isfinite(feedback_stamp_sec)
            or feedback_stamp_sec <= 0.0
            or not math.isfinite(transport_gap_ms)
            or transport_gap_ms < -self.transport_delay_threshold_ms
        ):
            return 'joint_states_invalid_source_stamp'
        return 'joint_states_transport_delay'

    def _enter_transport_fault(
            self, transport_gap_ms: float,
            feedback_age_ms: float = float('nan'),
            feedback_stamp_sec: float = float('nan'),
            path_time: float = None):
        """Latch the fault, publish the last safe command, then persist it."""
        if self.transport_fault:
            return None
        self.transport_fault = True
        self.transport_fault_gap_ms = float(transport_gap_ms)
        self.phase = 'transport_fault'
        if path_time is None:
            path_time = (
                float(self.time_scaler.path_time)
                if self.online_time_scaling_enabled else 0.0)
        self._fault_path_time = max(0.0, float(path_time))
        # q_last is the last accepted/published safe command. A delayed
        # q_feedback can be 100+ ms behind and must never overwrite it.
        history = getattr(self, '_cmd_history', ())
        if (
            history
            and np.all(np.isfinite(
                np.asarray(history[-1][1], dtype=float)))
        ):
            # Includes any deliberate DART perturbation, so this is the exact
            # actuator command rather than merely the nominal accepted q.
            self.q_last = np.asarray(history[-1][1], dtype=float).copy()
        elif (
            self.q_last is None
            or not np.all(np.isfinite(np.asarray(self.q_last, dtype=float)))
        ):
            self.q_last = np.asarray(
                self.reset_q_target, dtype=float).copy()
        self.warm_start = None
        self.second_order_warm_start = None
        self._so_last_cmd_dq = None
        self._so_last_cmd_ddq = None

        # Safety output precedes JSON/file work. A slow filesystem can no
        # longer extend the lifetime of the previous moving command.
        published = self._publish_joint_positions(
            self.q_last, np.zeros(JOINT_COUNT))
        if (
            self.record_loop_csv
            and self._params_snapshot_path is None
            and hasattr(self, '_pending_config')
        ):
            self._write_params_snapshot(self._pending_config)
            self._snapshot_written = self._params_snapshot_path is not None
        reason = self._transport_fault_reason(
            transport_gap_ms, feedback_age_ms, feedback_stamp_sec)
        details = {
            'transport_gap_ms': float(transport_gap_ms),
            'feedback_age_ms': float(feedback_age_ms),
            'feedback_stamp_sec': float(feedback_stamp_sec),
            'bad_duration_sec': float(
                self._transport_gap_bad_duration_sec),
            'distinct_bad_samples': int(
                self._transport_gap_distinct_bad_samples),
            'threshold_ms': float(self.transport_delay_threshold_ms),
        }
        self._invalidate_params_snapshot(reason, details)
        self.get_logger().error(
            '正式实验硬门触发: /joint_states 投递延迟、时间戳或本地反馈'
            '年龄异常，'
            f'当前 transport_gap={transport_gap_ms:.1f}ms，'
            f'feedback_age={feedback_age_ms:.1f}ms，'
            f'持续 {self._transport_gap_bad_duration_sec:.2f}s，'
            '进入 transport_fault：保持最后安全命令、冻结路径、停止求解；'
            '本次参数快照已标记 experiment_valid=false。')
        return published

    def _invalidate_params_snapshot(self, reason: str, details: dict):
        """Persist runtime invalidation so offline analysis cannot accept it."""
        path = getattr(self, '_params_snapshot_path', None)
        if path is None:
            return
        try:
            snapshot = json.loads(Path(path).read_text())
            snapshot['experiment_valid'] = False
            snapshot['runtime_validation'] = {
                'valid': False,
                'reason': str(reason),
                'invalidated_at': datetime.now().isoformat(
                    timespec='milliseconds'),
                **dict(details),
            }
            tmp_path = Path(path).with_suffix('.json.tmp')
            with tmp_path.open('w') as stream:
                json.dump(
                    snapshot, stream, indent=2, ensure_ascii=False,
                    default=str)
            tmp_path.replace(path)
        except Exception as exc:
            self.get_logger().error(
                f'运行失效状态写入参数快照失败: {exc}')


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
        """Controller belief for the base position."""
        scaled_t = t / float(getattr(self, 'base_motion_time_scale', 1.0))
        pose = traj.base_pose(scaled_t)
        amplitude = float(getattr(self, 'base_disturbance_amplitude_m', 0.0))
        if amplitude > 0.0:
            mode = getattr(self, 'base_belief_mode', 'truth')
            if mode == 'truth':
                pose = pose + traj.base_disturbance_offset(
                    scaled_t, amplitude, self.base_disturbance_seed,
                    profile=self.base_disturbance_profile)
            elif mode == 'estimated':
                pose = pose + self._base_belief_offset
            elif (mode == 'model'
                  and getattr(self, '_trust_pred_times', None) is not None):
                # 按 t 查本拍缓存的逐时刻 Δ̂(t): 最近一次 predict_horizon 的
                # 时间网格线性插值(np.interp 越界端点保持), 拍内不重复预测。
                # 首拍缓存前(reset 阶段 TF)不加补偿 —— θ̂ 初值本就为 0。
                pose = pose + np.array([
                    np.interp(t, self._trust_pred_times,
                              self._trust_pred_delta[:, axis])
                    for axis in range(3)])
            # 'nominal': 只知名义轨迹, 不加任何补偿
        return pose

    def _base_truth_pose(self, t: float) -> np.ndarray:
        """Physical/simulated base truth used only for TF and actual markers."""
        scaled_t = t / float(getattr(self, 'base_motion_time_scale', 1.0))
        pose = traj.base_pose(scaled_t)
        amplitude = float(getattr(
            self, 'base_disturbance_amplitude_m', 0.0))
        if amplitude > 0.0:
            pose = pose + traj.base_disturbance_offset(
                scaled_t, amplitude, self.base_disturbance_seed,
                profile=self.base_disturbance_profile)
        return pose

    def _trust_estimator_enabled(self) -> bool:
        """model 信念或 trust 影子 CSV 需要估计器; amp=0 一律关闭(零开销)。"""
        return (self.base_disturbance_amplitude_m > 0.0
                and (self.base_belief_mode == 'model'
                     or self.trust_shadow_csv))

    def _make_trust_estimator(self) -> BaseTrustEstimator:
        """按 trust_* 参数构造 BaseTrustEstimator(运行在缩放时间基)。"""
        scale = float(self.base_motion_time_scale)
        meas_delay = (
            self.trust_meas_delay_sec if self.trust_meas_delay_sec >= 0.0
            else self.base_estimator_delay_sec / scale)
        dt_nominal = (
            self.trust_dt_nominal if self.trust_dt_nominal > 0.0
            else self.dt / scale)
        # 保形校准的最大前瞻组号: predict_horizon 的 h 相对**最近测量时刻**,
        # 需覆盖预测网格前瞻拍数 + 测量延迟折算拍数, 外加 2 拍余量。
        cal_horizon_steps = int(
            self._trust_grid_n_fwd
            + math.ceil(max(0.0, meas_delay) / dt_nominal) + 2)
        return BaseTrustEstimator(
            n_freq=self.trust_n_freq,
            fmin_hz=self.trust_fmin_hz,
            fmax_hz=self.trust_fmax_hz,
            lam=self.trust_lam,
            sigma0=self.trust_sigma0,
            prior_std=self.trust_prior_std,
            meas_delay_sec=meas_delay,
            dt_nominal=dt_nominal,
            n_peaks=self.trust_n_peaks,
            ident_period_sec=self.trust_ident_period_sec,
            buffer_sec=self.trust_buffer_sec,
            min_ident_sec=self.trust_min_ident_sec,
            holdout_sec=self.trust_holdout_sec,
            f_ident_min_hz=self.trust_f_ident_min_hz,
            f_ident_max_hz=self.trust_f_ident_max_hz,
            peak_snr=self.trust_peak_snr,
            resid_lpf_hz=self.trust_resid_lpf_hz,
            beta_sigma0=(None if self.trust_beta_sigma0 <= 0.0
                         else self.trust_beta_sigma0),
            beta_gamma_f=self.trust_beta_gamma_f,
            beta_n_grid=self.trust_beta_n_grid,
            beta_degraded=self.trust_beta_degraded,
            beta_healthy=self.trust_beta_healthy,
            n_cal=self.trust_n_cal,
            alpha=self.trust_alpha,
            gamma_aci=self.trust_gamma_aci,
            ident_units_per_tick=self.trust_ident_units_per_tick,
            ident_budget_sec=1e-3 * self.trust_ident_budget_ms,
            ident_units_pen_up=self.trust_ident_units_pen_up,
            ident_budget_pen_up_sec=1e-3 * self.trust_ident_budget_pen_up_ms,
            cal_horizon_steps=cal_horizon_steps,
            regime_enabled=self.trust_regime_enabled,
            regime_stable_tau_sec=self.trust_regime_stable_tau_sec,
            regime_stable_process_std=(
                self.trust_regime_stable_process_std),
            regime_persistent_process_std=(
                self.trust_regime_persistent_process_std),
            regime_measurement_std=(
                None if self.trust_regime_measurement_std <= 0.0
                else self.trust_regime_measurement_std),
            regime_stable_stay_probability=(
                self.trust_regime_stable_stay_probability),
            regime_persistent_stay_probability=(
                self.trust_regime_persistent_stay_probability),
            regime_initial_persistent_probability=(
                self.trust_regime_initial_persistent_probability),
        )

    def _reset_trust_estimator(self):
        """重建估计器与本拍预测缓存(初始化及每次进入 track 时调用)。

        测量噪声用独立但同种子的 RNG 流: model/影子通道的噪声序列与
        estimated 信念完全一致, 且开关影子 CSV 不改变 estimated 通道
        自身的噪声序列(不与 _base_estimator_rng 抢流)。
        """
        self._trust_pred_times = None
        self._trust_pred_delta = None
        self._trust_pred_delta_raw = None
        self._trust_pred_harmonic = None
        self._trust_pred_fast = None
        self._trust_pred_covariance = None
        self._trust_pred_mode_probabilities = None
        self._trust_pred_delta_cal = float('nan')
        self._trust_feedforward_gain = 0.0
        self._trust_feedforward_gate = TrustFeedforwardGate(
            low=self.trust_feedforward_gate_low,
            high=self.trust_feedforward_gate_high,
            fall_tau_sec=self.trust_feedforward_gate_fall_tau_sec,
            rise_tau_sec=self.trust_feedforward_gate_rise_tau_sec)
        self._uncertainty_decision = UncertaintyAwareFeedforwardDecision(
            applicability_low=self.trust_feedforward_gate_low,
            applicability_high=self.trust_feedforward_gate_high,
            risk_weight=self.uncertainty_risk_weight,
            applicability_risk_gain=(
                self.uncertainty_applicability_risk_gain),
            smoothing_weight=self.uncertainty_smoothing_weight,
            fall_tau_sec=self.uncertainty_gain_fall_tau_sec,
            rise_tau_sec=self.uncertainty_gain_rise_tau_sec,
            risk_budget_m=self.uncertainty_risk_budget_m,
            min_speed_scale=self.uncertainty_min_speed_scale,
            min_position_weight_scale=(
                self.uncertainty_min_position_weight_scale),
        )
        self._uncertainty_speed_cap = 1.0
        self._uncertainty_position_weight_scale = 1.0
        self._uncertainty_radius_m = 0.0
        self._uncertainty_parameter_gain = 0.0
        self._uncertainty_risk_power = 0.0
        self._guard_uncertainty_m = 0.0
        # 估计器单拍开销实测(要求: 常规 step/predict O(K), 分摊辨识块 <3ms):
        # 15s 窗口打印 P50/P99/max, 生命周期累积在结束时打总账。总账通道为
        # 有界蓄水池子样(容量 TRUST_COST_RESERVOIR_CAP, 见 _trust_all_append),
        # 计数/最大值另行精确跟踪 —— 长时间连续运行内存有上限。
        self._trust_cost_ms_win = []
        self._trust_ident_ms_win = []
        self._trust_cost_ms_all = []
        self._trust_ident_ms_all = []
        self._trust_all_n = 0
        self._trust_cost_ms_max = 0.0
        self._trust_ident_ms_max = 0.0
        self._trust_all_rng = random.Random(0)
        self._trust_cost_last_log_wall = None
        self._sched_pen_down = True     # 上一拍参考样本的落笔态(调度提示)
        # 信任降速阶梯状态随估计器一起复位: 重新学习本轮健康 δ 基线。
        self._trust_ladder = TrustLadderStateMachine(TrustLadderConfig(
            b_degraded=self.ladder_b_degraded,
            b_recover=self.ladder_b_recover,
            delta_degraded_ratio=self.ladder_delta_degraded_ratio,
            delta_recover_ratio=self.ladder_delta_recover_ratio,
            min_scale=self.ladder_min_scale,
            baseline_warmup_sec=self.ladder_baseline_warmup_sec,
            baseline_tau_sec=self.ladder_baseline_tau_sec,
            trust_commission_sec=self.ladder_trust_commission_sec,
            recover_debounce_sec=self.ladder_recover_debounce_sec,
        ))
        self._ladder_scale_cap = 1.0
        if self._trust_estimator_enabled():
            self._trust_estimator = self._make_trust_estimator()
            self._trust_meas_rng = np.random.default_rng(
                self.base_estimator_seed)
        else:
            self._trust_estimator = None
            self._trust_meas_rng = None

    def _update_base_belief(self, t_solve: float):
        """estimated/model 信念与 trust 影子通道的每拍更新。

        estimated: 延迟测量+噪声经一阶低通, 预测域内常值保持, 与离线环
                   hifi_offline_loop 的估计器模型逐式一致;
        model:     同源测量(不过低通)喂 BaseTrustEstimator.step, 并缓存
                   predict_horizon 在本拍时间网格上的逐时刻 Δ̂(t),
                   _base_pose 只做线性插值;
        trust_shadow_csv: 无论 belief 模式为何, 估计器照跑并逐拍写
                   (t, e_meas, Δ̂_1step, δ, b, ||θ̂||) 到独立 CSV。
        扰动关闭(amp=0)时直接早退, 所有新代码零开销。
        """
        if self.base_disturbance_amplitude_m <= 0.0:
            return
        scale = float(getattr(self, 'base_motion_time_scale', 1.0))
        measurement_time = (t_solve - self.base_estimator_delay_sec) / scale
        if self.base_belief_mode == 'estimated':
            measurement = traj.base_disturbance_offset(
                measurement_time, self.base_disturbance_amplitude_m,
                self.base_disturbance_seed,
                profile=self.base_disturbance_profile)
            if self.base_estimator_noise_std_m > 0.0:
                measurement = measurement + self._base_estimator_rng.normal(
                    0.0, self.base_estimator_noise_std_m, 3)
            if self.base_estimator_lpf_hz > 0.0:
                alpha = 1.0 - math.exp(
                    -2.0 * math.pi * self.base_estimator_lpf_hz * self.dt)
            else:
                alpha = 1.0
            self._base_belief_offset = (
                self._base_belief_offset
                + alpha * (measurement - self._base_belief_offset))
        if self._trust_estimator is None:
            return
        # model/影子通道共用 estimated 同源测量(真值(t-延迟)+噪声, 不过低通),
        # 与 hifi_offline_loop 的 belief='model' 测量通道逐式一致。
        e_meas = traj.base_disturbance_offset(
            measurement_time, self.base_disturbance_amplitude_m,
            self.base_disturbance_seed,
            profile=self.base_disturbance_profile)
        if self.base_estimator_noise_std_m > 0.0:
            e_meas = e_meas + self._trust_meas_rng.normal(
                0.0, self.base_estimator_noise_std_m, 3)
        # 实时性: step 内部只做 O(K) 常规更新 + 一个 ≤budget 的分摊辨识块
        # (sched_pen_down=False 的抬笔/过渡拍放宽预算); 整块重辨识严禁在
        # tick 内执行。单拍开销实测累积, 周期性打印。
        trust_t0 = time.perf_counter()
        step_diag = self._trust_estimator.step(
            t_solve / scale, e_meas,
            sched_pen_down=self._sched_pen_down)
        if self.base_belief_mode == 'model':
            # 缓存本拍预测网格: 之后所有 _base_pose(参考生成/守卫/TF/记录)
            # 共享同一 Δ̂(t) 网格插值, 拍内一致且不重复预测。
            times = t_solve + self._trust_grid_offsets
            stats = self._trust_estimator.predict_horizon_stats(
                times / scale)
            harmonic = stats['harmonic_mean']
            fast = stats['fast_mean']
            covariance = stats['covariance']
            delta_hat_raw = stats['mean']
            delta_cal = float(stats['delta'])
            parameter_gain = trust_feedforward_gain(
                step_diag['parameter_applicability'],
                self.trust_feedforward_gate_low,
                self.trust_feedforward_gate_high)
            mode = self.uncertainty_control_mode
            self._uncertainty_speed_cap = 1.0
            self._uncertainty_position_weight_scale = 1.0
            self._uncertainty_radius_m = math.sqrt(
                max(float(np.mean(np.trace(
                    covariance, axis1=1, axis2=2)))
                    + 3.0 * delta_cal * delta_cal, 0.0))
            self._uncertainty_parameter_gain = float(parameter_gain)
            self._uncertainty_risk_power = (
                self._uncertainty_radius_m ** 2)
            if mode == 'b0':
                prediction = np.zeros_like(delta_hat_raw)
                gate = 0.0
            elif mode == 'b1':
                prediction = harmonic
                gate = 1.0
            elif mode == 'b2':
                gate = (self._trust_feedforward_gate.update(
                            step_diag['trust'], self.dt)
                        if self.trust_feedforward_gating_enabled else 1.0)
                prediction = gate * harmonic
            elif mode == 'b3':
                prediction = delta_hat_raw
                gate = 1.0
            else:
                decision = self._uncertainty_decision.update(
                    harmonic, fast, covariance, delta_cal,
                    step_diag['parameter_applicability'], self.dt)
                prediction = decision['compensated_prediction']
                gate = float(decision['feedforward_gain'])
                self._uncertainty_speed_cap = float(
                    decision['speed_scale'])
                self._uncertainty_position_weight_scale = float(
                    decision['position_weight_scale'])
                self._uncertainty_radius_m = float(
                    decision['uncertainty_radius'])
                self._uncertainty_parameter_gain = float(
                    decision['parameter_applicability_gain'])
                self._uncertainty_risk_power = float(
                    decision['risk_power'])
            self._trust_pred_times = times
            self._trust_pred_delta_raw = delta_hat_raw
            self._trust_pred_harmonic = harmonic
            self._trust_pred_fast = fast
            self._trust_pred_covariance = covariance
            self._trust_pred_mode_probabilities = (
                stats['mode_probabilities'])
            self._trust_pred_delta = prediction
            self._trust_pred_delta_cal = delta_cal
            self._trust_feedforward_gain = float(gate)
            # 当前解析平台只有基座平移不确定性，world 末端位置对该平移的
            # 雅可比为 I。L∞ 保形半径转换为 L2 后进入命令安全过滤。
            self._guard_uncertainty_m = (
                min(
                    self.uncertainty_guard_radius_max_m,
                    self.uncertainty_guard_radius_gain
                    * math.sqrt(3.0) * delta_cal)
                if mode in ('b3', 'b4') else 0.0)
        trust_cost_ms = 1e3 * (time.perf_counter() - trust_t0)
        ident_ms = float(step_diag.get('ident_ms', 0.0))
        self._trust_cost_ms_win.append(trust_cost_ms)
        self._trust_ident_ms_win.append(ident_ms)
        self._trust_all_append(trust_cost_ms, ident_ms)
        self._maybe_log_trust_cost()
        if self.trust_ladder_enabled and self.base_belief_mode == 'model':
            self._update_trust_ladder(t_solve, step_diag)
        if self.trust_shadow_csv:
            if self._trust_pred_times is not None:
                delta_hat_1step = self._trust_pred_delta[
                    self._trust_grid_1step_index]
                delta_hat_raw_1step = self._trust_pred_delta_raw[
                    self._trust_grid_1step_index]
            else:
                delta_hat_raw_1step = self._trust_estimator.predict_horizon(
                    np.array([(t_solve + self.dt) / scale]))[0][0]
                delta_hat_1step = (
                    self._trust_feedforward_gain * delta_hat_raw_1step)
            self._append_trust_shadow_row(
                t_solve, e_meas, delta_hat_1step,
                delta_hat_raw_1step, step_diag)

    def _trust_all_append(self, cost_ms: float, ident_ms: float):
        """总账样本入有界蓄水池(Algorithm R, cost/ident 同索引配对替换).

        每列各自仍是全程样本的均匀子样, 结束时分位数统计无偏; 计数与
        最大值精确跟踪。track 阶段 GC 关闭, 无界 append 会持续累积
        (~36 万对象/小时), 蓄水池使单次运行内存有上限。
        """
        self._trust_all_n += 1
        if cost_ms > self._trust_cost_ms_max:
            self._trust_cost_ms_max = float(cost_ms)
        if ident_ms > self._trust_ident_ms_max:
            self._trust_ident_ms_max = float(ident_ms)
        if len(self._trust_cost_ms_all) < TRUST_COST_RESERVOIR_CAP:
            self._trust_cost_ms_all.append(cost_ms)
            self._trust_ident_ms_all.append(ident_ms)
            return
        j = self._trust_all_rng.randrange(self._trust_all_n)
        if j < TRUST_COST_RESERVOIR_CAP:
            self._trust_cost_ms_all[j] = cost_ms
            self._trust_ident_ms_all[j] = ident_ms

    def _maybe_log_trust_cost(self):
        """每 15s 打印估计器单拍实测开销(step+predict 含分摊辨识块)。

        目的: 实时性回归可观测 —— 常规路径应为亚毫秒 O(K), 分摊辨识块
        P99 应 <3ms; 若窗口 P99 异常升高, 说明分摊调度失效或机器过载。
        """
        now = time.perf_counter()
        if self._trust_cost_last_log_wall is None:
            self._trust_cost_last_log_wall = now
            return
        if now - self._trust_cost_last_log_wall < 15.0:
            return
        self._trust_cost_last_log_wall = now
        cost = np.asarray(self._trust_cost_ms_win)
        ident = np.asarray(self._trust_ident_ms_win)
        self._trust_cost_ms_win = []
        self._trust_ident_ms_win = []
        if cost.size == 0:
            return
        self.get_logger().info(
            'trust 单拍开销(15s 窗, ms): step+pred P50/P99/max = '
            f'{np.percentile(cost, 50):.2f}/{np.percentile(cost, 99):.2f}/'
            f'{cost.max():.2f}; 分摊辨识块 P99/max = '
            f'{np.percentile(ident, 99):.2f}/{ident.max():.2f} '
            f'(辨识 {self._trust_estimator.n_ident} 轮/'
            f'换库 {self._trust_estimator.n_ident_accepted})')

    def _log_trust_cost_summary(self):
        """结束时打总账: 全程估计器单拍开销与分摊辨识块分位数。

        分位数来自有界蓄水池子样(全程均匀抽样, 见 _trust_all_append),
        n 与 max 为全程精确值。
        """
        cost = np.asarray(getattr(self, '_trust_cost_ms_all', []))
        ident = np.asarray(getattr(self, '_trust_ident_ms_all', []))
        if cost.size == 0 or ident.size == 0 \
                or getattr(self, '_trust_estimator', None) is None:
            return
        self.get_logger().info(
            'trust 单拍开销总账(ms): step+pred P50/P95/P99/max = '
            f'{np.percentile(cost, 50):.2f}/{np.percentile(cost, 95):.2f}/'
            f'{np.percentile(cost, 99):.2f}/{self._trust_cost_ms_max:.2f} '
            f'(n={self._trust_all_n}); 分摊辨识块 P99/max = '
            f'{np.percentile(ident, 99):.2f}/{self._trust_ident_ms_max:.2f}; '
            f'辨识 {self._trust_estimator.n_ident} 轮/'
            f'换库 {self._trust_estimator.n_ident_accepted}')

    def _update_trust_ladder(self, t_solve: float, step_diag: dict):
        """推进健康基线 + 双迟滞四态阶梯，并更新路径 scale 上限。"""
        delta = float(step_diag['delta_1step'])
        b = float(step_diag['trust'])
        cap_prev = self._ladder_scale_cap
        state_prev = self._trust_ladder.state
        self._ladder_scale_cap = self._trust_ladder.update(
            t_solve, b, delta)
        state = self._trust_ladder.state
        baseline = self._trust_ladder.delta_baseline
        # 只在 1.0 边界穿越时打日志(启动/完全恢复), 避免降速期间逐拍刷屏。
        if cap_prev >= 1.0 and self._ladder_scale_cap < 1.0:
            self.get_logger().info(
                f'信任阶梯降速启动: cap={self._ladder_scale_cap:.3f} '
                f'(state={state}, delta={1e3*delta:.2f}mm, '
                f'baseline={1e3*baseline:.2f}mm, b={b:.3f})')
        elif cap_prev < 1.0 and self._ladder_scale_cap >= 1.0:
            self.get_logger().info(
                f'信任阶梯恢复全速 (delta={1e3*delta:.2f}mm, '
                f'baseline={1e3*baseline:.2f}mm, b={b:.3f})')
        elif state_prev == 'warming' and state == 'healthy':
            self.get_logger().info(
                f'信任阶梯健康基线就绪: delta0={1e3*baseline:.2f}mm')

    def _append_trust_shadow_row(
            self, t_solve, e_meas, delta_hat_1step,
            delta_hat_raw_1step, step_diag):
        """trust 影子 CSV 独立于 loop CSV: 懒打开, 失败只停本通道记录。"""
        if self._trust_record_failed:
            return
        try:
            if self._trust_writer is None:
                self.loop_data_dir.mkdir(parents=True, exist_ok=True)
                path = (self.loop_data_dir
                        / f'trust_shadow_{self._run_stamp}.csv')
                self._trust_file = path.open('w', newline='')
                self._trust_writer = csv.writer(self._trust_file)
                self._trust_writer.writerow([
                    't', 'e_meas_x', 'e_meas_y', 'e_meas_z',
                    'delta_hat_1step_x', 'delta_hat_1step_y',
                    'delta_hat_1step_z',
                    'delta_1step', 'trust_b', 'theta_norm',
                    'delta_hat_raw_1step_x', 'delta_hat_raw_1step_y',
                    'delta_hat_raw_1step_z', 'beta_mean',
                    'feedforward_gain', 'trust_b_filtered',
                    'ladder_delta_baseline',
                    'ladder_state', 'ladder_cap',
                    'uncertainty_control_mode',
                    'parameter_applicability_gain',
                    'persistent_probability',
                    'fast_residual_x', 'fast_residual_y', 'fast_residual_z',
                    'uncertainty_radius_m', 'uncertainty_risk_power',
                    'uncertainty_speed_cap',
                    'uncertainty_position_weight_scale',
                    'guard_uncertainty_m'])
                self.get_logger().info(f'trust 影子 CSV 开始记录: {path}')
            theta_norm = math.sqrt(sum(
                float(rls.mu @ rls.mu)
                for rls in self._trust_estimator.rls))
            self._trust_writer.writerow([
                float(t_solve),
                *[float(v) for v in np.asarray(e_meas, dtype=float)],
                *[float(v) for v in np.asarray(delta_hat_1step, dtype=float)],
                float(step_diag['delta_1step']),
                float(step_diag['trust']),
                theta_norm,
                *[float(v) for v in np.asarray(
                    delta_hat_raw_1step, dtype=float)],
                float(step_diag.get('beta_mean', float('nan'))),
                float(self._trust_feedforward_gain),
                float(self._trust_feedforward_gate.filtered_trust),
                float(self._trust_ladder.delta_baseline),
                str(self._trust_ladder.state),
                float(self._ladder_scale_cap),
                str(self.uncertainty_control_mode),
                float(self._uncertainty_parameter_gain),
                float(step_diag.get(
                    'persistent_probability', float('nan'))),
                *[float(v) for v in np.asarray(
                    step_diag.get(
                        'fast_residual_mean', np.full(3, np.nan)),
                    dtype=float)],
                float(self._uncertainty_radius_m),
                float(self._uncertainty_risk_power),
                float(self._uncertainty_speed_cap),
                float(self._uncertainty_position_weight_scale),
                float(self._guard_uncertainty_m),
            ])
            self._trust_rows_since_flush += 1
            if self._trust_rows_since_flush >= 100:
                self._trust_file.flush()
                self._trust_rows_since_flush = 0
        except Exception as exc:
            self._trust_record_failed = True
            self.get_logger().warn(
                'trust 影子 CSV 记录失败,后续 tick 停止记录(不影响控制): '
                f'{exc}\n{traceback.format_exc()}')

    def _close_trust_csv(self):
        if self._trust_file is not None:
            try:
                self._trust_file.flush()
                self._trust_file.close()
            except Exception:
                pass
            self._trust_file = None
            self._trust_writer = None

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

    def _compute_dq_prev(
            self, q_now, js_stamp_now, dq_measured_snapshot=None):
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
        dq_for_diag = (
            dq_measured_snapshot
            if dq_measured_snapshot is not None
            else getattr(self, 'dq_measured', None))
        vel_est = (np.asarray(dq_for_diag, dtype=float) * self.dt
                   if dq_for_diag is not None else nanv)
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

    def _predicted_application_delay_sec(self) -> float:
        """Expected solve-start -> actuator-application delay for this tick."""
        if not bool(getattr(self, 'latency_compensation_enabled', False)):
            return 0.0
        estimate = (
            float(getattr(self, '_publish_latency_ewma_sec', 0.0))
            + float(getattr(self, 'latency_actuator_delay_sec', 0.0))
        )
        maximum = float(getattr(
            self, 'latency_prediction_max_sec', 2.0 * self.dt))
        return float(np.clip(estimate, 0.0, maximum))

    def _feedback_age_to_solve_sec(self, solve_wall: float) -> float:
        """Age of the state sample at solve start, preferring its source stamp."""
        if np.isfinite(getattr(self, '_js_header_stamp_sec', float('nan'))):
            age = float(solve_wall) - float(self._js_header_stamp_sec)
            # Some hardware drivers publish zero or a different clock domain.
            # In that case receive time is the only comparable local clock.
            if age < 0.0 or age > 2.0:
                age = float('nan')
        else:
            age = float('nan')
        if not np.isfinite(age):
            if np.isfinite(getattr(self, '_js_recv_wall_sec', float('nan'))):
                age = float(solve_wall) - float(self._js_recv_wall_sec)
            else:
                age = 0.0
        maximum = float(getattr(
            self, 'latency_prediction_max_sec', 2.0 * self.dt))
        return float(np.clip(age, 0.0, maximum))

    def _smith_predict_state(
            self, q_feedback: np.ndarray, dq_feedback: np.ndarray,
            solve_wall: float, application_delay_sec: float):
        """用已知 plant 执行律 + 在途命令队列, 把实测状态前推到命令生效时刻。

        plant 律(smooth_position_controller.cc 速度前馈分支, 100Hz):
            v_fb = clip((q_cmd-q)/tau, ±fb_lim)
            v    = clip(clip(gain*dq_cmd + v_fb, ±vmax), v_last ± amax*h)
            q   += v*h
        与 second_order_state_prediction(用 ddq0 二阶外推)不同, 这里模拟的是
        plant 真实会执行的命令序列, 不会与执行链失配形成正反馈。
        返回 (q_pred, v_pred, prediction_age_sec)。
        """
        stamp = self._js_header_stamp_sec
        if not np.isfinite(stamp):
            stamp = self._js_recv_wall_sec
        if not np.isfinite(stamp) or not self._cmd_history:
            return (np.asarray(q_feedback, dtype=float).copy(),
                    np.asarray(dq_feedback, dtype=float).copy(), 0.0)
        t_target = float(solve_wall) + max(0.0, float(application_delay_sec))
        h = self.plant_control_substep_dt
        n = int(round((t_target - float(stamp)) / h))
        n_max = int(math.ceil(self.latency_prediction_max_sec / h)) + 2
        n = max(0, min(n, n_max))
        q = np.asarray(q_feedback, dtype=float).copy()
        v_last = np.asarray(dq_feedback, dtype=float).copy()
        history = list(self._cmd_history)
        t = float(stamp)
        for _ in range(n):
            q_cmd, dq_cmd = history[0][1], history[0][2]
            for entry in history:
                if entry[0] <= t + 1e-9:
                    q_cmd, dq_cmd = entry[1], entry[2]
            v_fb = np.clip(
                (q_cmd - q) / self.plant_position_feedback_tau,
                -self.plant_feedback_velocity_limit,
                self.plant_feedback_velocity_limit)
            v_raw = np.clip(
                self.plant_velocity_feedforward_gain * dq_cmd + v_fb,
                -self.plant_max_velocity, self.plant_max_velocity)
            step = self.plant_max_acceleration * h
            v = np.clip(v_raw, v_last - step, v_last + step)
            q = np.clip(q + v * h, self.mpc_ik.q_lower, self.mpc_ik.q_upper)
            v_last = v
            t += h
        return q, v_last, n * h

    def _predict_second_order_state(
            self, q_current: np.ndarray, dq_current: np.ndarray,
            solve_wall: float, application_delay_sec: float):
        """Propagate timestamped [q,dq] to the predicted command-application time."""
        feedback_age = self._feedback_age_to_solve_sec(solve_wall)
        prediction_age = feedback_age + max(0.0, float(application_delay_sec))
        prediction_age = float(np.clip(
            prediction_age, 0.0, self.latency_prediction_max_sec))
        ddq = np.asarray(
            self._so_last_cmd_ddq
            if self._so_last_cmd_ddq is not None
            else np.zeros(JOINT_COUNT),
            dtype=float,
        )
        if ddq.shape != (JOINT_COUNT,) or not np.all(np.isfinite(ddq)):
            ddq = np.zeros(JOINT_COUNT)
        ddq = np.clip(
            ddq,
            -self.second_order_max_acceleration,
            self.second_order_max_acceleration,
        )
        q_pred = (
            np.asarray(q_current, dtype=float)
            + prediction_age * np.asarray(dq_current, dtype=float)
            + 0.5 * prediction_age * prediction_age * ddq
        )
        dq_pred = np.asarray(dq_current, dtype=float) + prediction_age * ddq
        q_pred = np.clip(q_pred, self.mpc_ik.q_lower, self.mpc_ik.q_upper)
        dq_pred = np.clip(
            dq_pred,
            -self.second_order_max_velocity,
            self.second_order_max_velocity,
        )
        return q_pred, dq_pred, prediction_age

    def _target_horizon_schedule(
            self, t_solve: float, horizon_steps: int = None,
            lead_override: int = None, application_delay_sec: float = 0.0):
        """Return future base/path clocks and the local path clock rate."""
        lead = (self.reference_lead_steps if lead_override is None
                else int(lead_override))
        configured_base_lead = int(getattr(
            self, 'base_reference_lead_steps', -1))
        base_lead = (
            lead if configured_base_lead < 0
            else configured_base_lead
        )
        horizon = self.horizon_steps if horizon_steps is None else int(horizon_steps)
        application_delay_sec = max(0.0, float(application_delay_sec))
        base_offsets = application_delay_sec + self.dt * (
            base_lead + np.arange(horizon))
        path_offsets = application_delay_sec + self.dt * (
            lead + np.arange(horizon))
        base_times = t_solve + base_offsets
        scaling_enabled = bool(getattr(
            self, 'online_time_scaling_enabled', False))
        if scaling_enabled:
            path_times = self.time_scaler.preview(path_offsets)
            path_rates = np.full(horizon, float(self.time_scaler.scale))
        else:
            path_times = t_solve + path_offsets
            path_rates = np.ones(horizon)
        return base_times, path_times, path_rates

    def _target_horizon_base(self, t_solve: float, horizon_steps: int = None,
                             update_log_times: bool = True,
                             lead_override: int = None,
                             application_delay_sec: float = 0.0):
        # 参考时序由 reference_lead_steps(L)控制:targets[j] = target(t_solve+(L+j)*dt)。
        #   L=0(baseline): targets 从 t_solve 开始,预测 q_k 主要对齐 target(t_solve)(一拍滞后)。
        #   L=1(阶段1A):  targets 从 t_solve+dt 开始;配合求解器 stage->target 映射
        #                  (stage k 代价用 targets[k-1]),得 q_k <-> target(t_solve+k*dt)。
        # t_solve 是本 tick 求解起点相对轨迹原点的**真实 clock** 时间(非合成 self.t)。
        # /student/joint_command 的 header.stamp = 真实发布时刻(见 _publish_joint_positions);
        # 参考时刻 t_reference=t_solve+L*dt 单独记录在 loop CSV,不用 stamp 制造时间平移。
        base_times, path_times, _ = self._target_horizon_schedule(
            t_solve, horizon_steps, lead_override, application_delay_sec)
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

    @staticmethod
    def _vee(skew_matrix: np.ndarray) -> np.ndarray:
        return np.array([
            skew_matrix[2, 1],
            skew_matrix[0, 2],
            skew_matrix[1, 0],
        ], dtype=float)

    def _reference_kinematics_base(
            self, base_t: float, path_t: float = None,
            path_rate: float = 1.0) -> dict:
        """Return ideal world/base derivatives and the drawing contour tangent."""
        if path_t is None:
            path_t = base_t
        h = float(self.reference_derivative_dt_sec)
        path_step = h * float(path_rate)

        # model 信念档的 _base_pose 含 dt 网格缓存 Δ̂ 的**分段线性**插值:
        # 其二阶差分在段内恒 0, 跨节点 ≈ 斜率跳变/h; 用 h ≤ 0.5*dt 的微分
        # 步长会把 Δ̂'' 放大 ~dt/h 倍(默认 5ms 下 ≈4 倍尖峰), 且随延迟补偿
        # 漂移跨越对齐边界时在 0 与尖峰间逐拍跳变。把基座位置差分步长对齐
        # 到缓存网格间距 dt: 三点样本各落入相邻段且段内相位一致, 二阶差分
        # 退化为相邻节点离散二阶差分(节点值是谐波模型精确值)的线性混合,
        # 幅值正确、拍间一致(2Hz 分量幅值误差 <1%); 速度仍为 O(dt²) 中心
        # 差分。truth/estimated/nominal 档 _base_pose 平滑, 保持原 h。
        h_base = h
        if (getattr(self, 'base_belief_mode', '') == 'model'
                and getattr(self, '_trust_pred_times', None) is not None):
            h_base = float(self.dt)
        base_position = self._base_pose(base_t)
        base_position_minus = self._base_pose(base_t - h_base)
        base_position_plus = self._base_pose(base_t + h_base)
        base_linear_velocity_world = (
            base_position_plus - base_position_minus) / (2.0 * h_base)
        base_linear_acceleration_world = (
            base_position_plus - 2.0 * base_position + base_position_minus
        ) / (h_base * h_base)

        base_rotation = self._base_rotation(base_t, path_t)
        base_rotation_minus = self._base_rotation(
            base_t - h, path_t - path_step)
        base_rotation_plus = self._base_rotation(
            base_t + h, path_t + path_step)
        rotation_dot = (
            base_rotation_plus - base_rotation_minus) / (2.0 * h)
        rotation_ddot = (
            base_rotation_plus - 2.0 * base_rotation + base_rotation_minus
        ) / (h * h)
        omega_matrix = base_rotation.T @ rotation_dot
        omega_matrix = 0.5 * (omega_matrix - omega_matrix.T)
        alpha_matrix = base_rotation.T @ rotation_ddot
        alpha_matrix = 0.5 * (alpha_matrix - alpha_matrix.T)
        base_angular_velocity_base = self._vee(omega_matrix)
        base_angular_acceleration_base = self._vee(alpha_matrix)

        ee_sample = self._ee_trajectory_sample(path_t)
        ee_position_world = ee_sample.position
        ee_position_world_minus = self._ee_target_world(path_t - path_step)
        ee_position_world_plus = self._ee_target_world(path_t + path_step)
        ee_velocity_world = (
            ee_position_world_plus - ee_position_world_minus) / (2.0 * h)
        ee_acceleration_world = (
            ee_position_world_plus
            - 2.0 * ee_position_world
            + ee_position_world_minus
        ) / (h * h)

        relative_position_base = (
            base_rotation.T @ (ee_position_world - base_position))
        relative_velocity_base = (
            base_rotation.T @ (
                ee_velocity_world - base_linear_velocity_world)
            - np.cross(
                base_angular_velocity_base, relative_position_base)
        )
        relative_acceleration_base = (
            base_rotation.T @ (
                ee_acceleration_world - base_linear_acceleration_world)
            - 2.0 * np.cross(
                base_angular_velocity_base, relative_velocity_base)
            - np.cross(
                base_angular_acceleration_base, relative_position_base)
            - np.cross(
                base_angular_velocity_base,
                np.cross(
                    base_angular_velocity_base, relative_position_base))
        )

        tangent_base = base_rotation.T @ ee_velocity_world
        tangent_norm = float(np.linalg.norm(tangent_base))
        contour_active = bool(ee_sample.pen_down and tangent_norm > 1e-4)
        if contour_active:
            tangent_base = tangent_base / tangent_norm
        else:
            tangent_base = np.zeros(3)
        values = {
            'position_base': relative_position_base,
            'velocity_base': relative_velocity_base,
            'acceleration_base': relative_acceleration_base,
            'contour_tangent_base': tangent_base,
            'contour_active': float(contour_active),
            'ee_velocity_world': ee_velocity_world,
            'ee_acceleration_world': ee_acceleration_world,
            'base_linear_velocity_world': base_linear_velocity_world,
            'base_linear_acceleration_world': base_linear_acceleration_world,
            'base_angular_velocity_base': base_angular_velocity_base,
            'base_angular_acceleration_base': base_angular_acceleration_base,
        }
        if not all(np.all(np.isfinite(value)) for value in values.values()):
            raise RuntimeError('explicit reference kinematics contains NaN/Inf')
        return values

    def _target_horizon_base_kinematics(
            self, t_solve: float, horizon_steps: int,
            update_log_times: bool = True, lead_override: int = None,
            application_delay_sec: float = 0.0):
        base_times, path_times, path_rates = self._target_horizon_schedule(
            t_solve, horizon_steps, lead_override, application_delay_sec)
        references = [
            self._reference_kinematics_base(
                float(base_t), float(path_t), float(path_rate))
            for base_t, path_t, path_rate in zip(
                base_times, path_times, path_rates)
        ]
        if update_log_times:
            self._last_target_time0 = float(base_times[0])
            self._last_path_target_time0 = float(path_times[0])
        positions = np.array([
            reference['position_base'] for reference in references])
        if bool(getattr(
                self, 'periodic_error_learning_enabled', False)):
            positions += self._periodic_target_corrections_base(
                base_times, path_times)
        rotations = np.array([
            self._target_rotation_base(float(base_t), float(path_t))
            for base_t, path_t in zip(base_times, path_times)
        ])
        velocities = np.array([
            reference['velocity_base'] for reference in references])
        accelerations = np.array([
            reference['acceleration_base'] for reference in references])
        tangents = np.array([
            reference['contour_tangent_base'] for reference in references])
        contour_active = np.array([
            reference['contour_active'] for reference in references])
        self._last_reference_kinematics = references[0]
        return (
            positions, rotations, velocities, accelerations,
            tangents, contour_active,
        )

    def _periodic_learning_key(self, sample) -> int:
        """Return an independent coefficient-bank key for a pen-down sample."""
        if not bool(sample.pen_down) or int(sample.stroke_id) < 0:
            return -1
        if self.trajectory_shape != 'whole':
            return 0
        # whole stroke id:
        # cycle*10000 + task_index*100 + local_stroke_id.
        return (int(sample.stroke_id) % 10000) // 100

    def _periodic_learning_basis(self, base_t: float) -> np.ndarray:
        period = (
            float(traj.BASE_PERIOD_SEC)
            * float(self.base_motion_time_scale)
        )
        phase = 2.0 * math.pi * float(base_t) / period
        return np.array([math.sin(phase), math.cos(phase)], dtype=float)

    def _reset_periodic_error_learning(self):
        initial = np.asarray(
            self.periodic_error_initial_coefficients,
            dtype=float,
        ).reshape(3, 2)
        key_count = 5 if self.trajectory_shape == 'whole' else 1
        self._periodic_error_coefficients = {
            key: initial.copy() for key in range(key_count)
        }
        self._periodic_error_accumulators = {}
        self._last_periodic_learning_diag = {}

    def _periodic_correction_world(
            self, base_t: float, path_t: float):
        """Return learned world correction and its coefficient-bank key."""
        sample = self._ee_trajectory_sample(float(path_t))
        key = self._periodic_learning_key(sample)
        if key < 0:
            return np.zeros(3), key
        coefficients = self._periodic_error_coefficients.get(key)
        if coefficients is None:
            return np.zeros(3), key
        correction = (
            np.asarray(coefficients, dtype=float)
            @ self._periodic_learning_basis(base_t)
        )
        return correction, key

    def _periodic_target_corrections_base(
            self, base_times: np.ndarray,
            path_times: np.ndarray) -> np.ndarray:
        """Rotate learned world corrections into each horizon stage's base."""
        corrections = []
        for base_t, path_t in zip(base_times, path_times):
            correction_world, _ = self._periodic_correction_world(
                float(base_t), float(path_t))
            corrections.append(
                self._base_rotation(float(base_t), float(path_t)).T
                @ correction_world)
        return np.asarray(corrections, dtype=float)

    def _update_periodic_error_learning(
            self, q_feedback: np.ndarray,
            feedback_time: float) -> dict:
        """LMS update for repeatable base-phase tracking error.

        The measured error is always relative to the *nominal* drawing target.
        If the plant response is ``actual = nominal + correction + disturbance``,
        gradient descent on that error learns ``correction = -disturbance``.
        """
        diag = {
            'active': False,
            'key': -1,
            'error_world': np.full(3, np.nan),
            'error_norm': float('nan'),
            'correction_world': np.zeros(3),
            'correction_norm': 0.0,
            'coefficient_norm': 0.0,
        }
        if (
            not self.periodic_error_learning_enabled
            or not np.isfinite(feedback_time)
            or feedback_time < 0.0
        ):
            self._last_periodic_learning_diag = diag
            return diag

        path_t = float(feedback_time)
        sample = self._ee_trajectory_sample(path_t)
        key = self._periodic_learning_key(sample)
        if key < 0:
            self._last_periodic_learning_diag = diag
            return diag

        q_feedback = np.asarray(q_feedback, dtype=float)
        ee_position_base, _ = self.mpc_ik.forward(q_feedback)
        actual_world = (
            self._base_pose(feedback_time)
            + self._base_rotation(feedback_time, path_t)
            @ ee_position_base
        )
        error_world = actual_world - sample.position
        error_norm = float(np.linalg.norm(error_world))
        diag.update({
            'key': key,
            'error_world': error_world.copy(),
            'error_norm': error_norm,
        })
        if (
            not np.all(np.isfinite(error_world))
            or error_norm > self.periodic_error_update_limit_m
        ):
            self._last_periodic_learning_diag = diag
            return diag

        basis = self._periodic_learning_basis(feedback_time)
        if self.periodic_error_learning_freeze:
            coefficients = self._periodic_error_coefficients.setdefault(
                key, np.zeros((3, 2), dtype=float))
            correction_world = coefficients @ basis
            diag.update({
                'active': True,
                'correction_world': correction_world.copy(),
                'correction_norm': float(np.linalg.norm(correction_world)),
                'coefficient_norm': float(np.linalg.norm(coefficients)),
            })
            self._last_periodic_learning_diag = diag
            return diag

        period = (
            float(traj.BASE_PERIOD_SEC)
            * float(self.base_motion_time_scale)
        )
        cycle = math.floor(float(feedback_time) / period)
        accumulator = self._periodic_error_accumulators.get(key)
        if accumulator is None:
            accumulator = {
                'cycle': cycle,
                'sum': np.zeros((3, 2), dtype=float),
                'count': 0,
            }
            self._periodic_error_accumulators[key] = accumulator

        coefficients = self._periodic_error_coefficients.setdefault(
            key, np.zeros((3, 2), dtype=float))
        # Only publish a coefficient update at a base-period boundary. Within
        # one 0.5 s cycle the reference is frozen, so instantaneous feedback
        # noise cannot turn into a new 50 Hz target disturbance.
        if int(accumulator['cycle']) != cycle:
            count = int(accumulator['count'])
            if count >= self.periodic_error_min_samples_per_cycle:
                coefficients -= (
                    2.0 * self.periodic_error_learning_gain
                    * np.asarray(accumulator['sum'], dtype=float)
                    / float(count)
                )
            accumulator['cycle'] = cycle
            accumulator['sum'] = np.zeros((3, 2), dtype=float)
            accumulator['count'] = 0
        accumulator['sum'] += np.outer(error_world, basis)
        accumulator['count'] += 1
        # ||C phi|| <= ||C||_F because ||phi||=1. Clamping the Frobenius norm
        # therefore guarantees the configured world correction bound at every
        # base phase, not only at the phase of the current update.
        coefficient_norm = float(np.linalg.norm(coefficients))
        if coefficient_norm > self.periodic_error_correction_limit_m:
            coefficients *= (
                self.periodic_error_correction_limit_m / coefficient_norm)
            coefficient_norm = self.periodic_error_correction_limit_m
        correction_world = coefficients @ basis
        diag.update({
            'active': True,
            'correction_world': correction_world.copy(),
            'correction_norm': float(np.linalg.norm(correction_world)),
            'coefficient_norm': coefficient_norm,
        })
        self._last_periodic_learning_diag = diag
        return diag

    def _publish_base_tf(
            self, t: float, path_t_override: float = None):
        # TF/RobotModel represents the physical simulated base, not the
        # controller's nominal/estimated/model belief. This keeps link7 and
        # the truth-channel current-actual marker in the same world frame.
        x, y, z = self._base_truth_pose(t)
        path_t = (
            float(path_t_override)
            if path_t_override is not None
            else self.time_scaler.path_time
            if self.online_time_scaling_enabled else t)
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
            'linearization_time_ms': 1000.0 * float(
                info.get('linearization_time', float('nan'))),
            'projection_time_ms': 1000.0 * float(
                info.get('projection_time', float('nan'))),
            'total_optimize_time_ms': 1000.0 * float(
                info.get('total_optimize_time', float('nan'))),
            'projection_velocity_residual': float(
                info.get('projection_velocity_residual_rms', float('nan'))),
            'projection_acceleration_residual': float(
                info.get('projection_acceleration_residual_rms', float('nan'))),
            'projection_velocity_active_ratio': float(
                info.get('projection_velocity_active_ratio', float('nan'))),
            'projection_acceleration_active_ratio': float(
                info.get('projection_acceleration_active_ratio', float('nan'))),
            'position_weight_scale': float(
                info.get('position_weight_scale', 1.0)),
        }

    def _solve_second_order(
            self, t_solve: float, solve_wall: float, q_current: np.ndarray,
            application_delay_sec: float, dq_measured_snapshot=None):
        """Use a timestamped, application-time [q,dq] as the second-order x0."""
        if self.second_order_ik is None:
            return None
        dq_feedback = (
            dq_measured_snapshot
            if dq_measured_snapshot is not None
            else getattr(self, 'dq_measured', None))
        if dq_feedback is None:
            return {
                'available': False,
                'reason': 'joint_states has no measured velocity',
            }
        dq_current = np.asarray(dq_feedback, dtype=float).reshape(-1)
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
        q0_solve = np.asarray(q_current, dtype=float)
        dq0_solve = dq_current.copy()
        prediction_age = 0.0
        if self.smith_predictor_enabled:
            # q 用 plant 律前推;速度状态保持 commanded 语义(模型计划速度),
            # 因为逆变换发布的 dq_pub 不再等于模型 dq —— plant 实测 v_last
            # 是执行律内部量,不是双积分模型的 dq 状态。
            dq_measured_for_pred = np.asarray(dq_feedback, dtype=float)
            q0_solve, v_pred, prediction_age = self._smith_predict_state(
                q_current, dq_measured_for_pred, solve_wall,
                application_delay_sec)
            if (self.second_order_velocity_source != 'commanded'
                    or self._so_last_cmd_dq is None):
                dq0_solve = np.clip(
                    v_pred, -self.second_order_max_velocity,
                    self.second_order_max_velocity)
        elif self.second_order_state_prediction:
            q0_solve, dq0_solve, prediction_age = (
                self._predict_second_order_state(
                    q_current, dq_current, solve_wall,
                    application_delay_sec))
        (
            targets, rotations, target_velocities, target_accelerations,
            contour_tangents, contour_active,
        ) = self._target_horizon_base_kinematics(
            t_solve, self.second_order_horizon_steps,
            update_log_times=(self.solver_model == 'second_order'),
            lead_override=so_lead,
            application_delay_sec=application_delay_sec)
        q_next, dq_next, ok, info = self.second_order_ik.step(
            targets,
            q0_solve,
            dq0_solve,
            warm_start=self.second_order_warm_start,
            target_rotation=rotations,
            target_velocity=target_velocities,
            target_acceleration=target_accelerations,
            contour_tangent=contour_tangents,
            contour_active=contour_active,
            position_weight_scale=(
                self._uncertainty_position_weight_scale
                if (self.base_belief_mode == 'model'
                    and self.uncertainty_control_mode == 'b4')
                else 1.0),
            posture_reference=(
                self.posture_reference
                if self.posture_reference_mode == 'reset_nominal' else None),
        )
        self.second_order_warm_start = info.get('trajectory')
        return {
            'available': True,
            'q_next': q_next,
            'dq_next': dq_next,
            'q_control': q0_solve,
            'dq_control': dq0_solve,
            'state_prediction_age_sec': prediction_age,
            'application_delay_sec': application_delay_sec,
            'ok': ok,
            'info': info,
        }

    def _reset_second_order_plan(self):
        """Discard a solver plan after the published command diverges from it."""
        self.second_order_warm_start = None
        if self.second_order_ik is not None:
            self.second_order_ik.reset_warm_start()

    def _recover_from_late_tick(self, timer_interval_ms: float) -> bool:
        """Reset one-step model history after a control deadline miss.

        The second-order warm start and commanded velocity are advanced by one
        nominal ``dt``.  Reusing them after a long callback interval creates an
        inconsistent x0 even when the newest measured position is available.
        """
        threshold_ms = (
            float(self.late_tick_recovery_factor) * self.dt * 1000.0)
        late = bool(
            self.late_tick_recovery_enabled
            and np.isfinite(timer_interval_ms)
            and timer_interval_ms > threshold_ms)
        if not late:
            return False
        self._reset_second_order_plan()
        self._so_last_cmd_dq = None
        self._so_last_cmd_ddq = None
        self.late_tick_recovery_count += 1
        self.get_logger().warn(
            '控制周期超时，丢弃过期二阶 warm start/commanded dq，'
            f'本拍从最新实测状态重启: interval={timer_interval_ms:.2f}ms '
            f'> threshold={threshold_ms:.2f}ms',
            throttle_duration_sec=1.0,
        )
        return True

    def _publish_world_markers(
            self, t: float, path_t_override: float = None):
        q_for_marker = self.q_measured if self.q_measured is not None else self.q_last
        if q_for_marker is None:
            return
        path_t = (
            float(path_t_override)
            if path_t_override is not None
            else self.time_scaler.path_time
            if self.online_time_scaling_enabled else t)
        base_pos_world = self._base_truth_pose(t)
        base_rot_world = self._base_rotation(t, path_t)
        ee_pos_base, _ = self.mpc_ik.forward(q_for_marker)
        ee_pos_world = base_pos_world + base_rot_world @ ee_pos_base
        target_sample = self._ee_trajectory_sample(path_t)
        # 落笔态存给下一拍的辨识分摊调度(滞后一拍, 缺省按落笔保守处理):
        # 抬笔/过渡拍放宽辨识预算, 落笔拍严格 <3ms。
        self._sched_pen_down = bool(target_sample.pen_down)
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

    def _command_fk_world(
            self, q: np.ndarray, base_t: float,
            path_t: float = None) -> np.ndarray:
        if path_t is None:
            path_t = self._path_time_at_offset(base_t, 0.0)
        base_pos_world = self._base_pose(base_t)
        base_rot_world = self._base_rotation(base_t, path_t)
        ee_pos_base, _ = self.mpc_ik.forward(q)
        return base_pos_world + base_rot_world @ ee_pos_base

    def _command_fk_step(
            self, q: np.ndarray, base_t: float,
            path_t: float = None) -> float:
        command_fk_world = self._command_fk_world(q, base_t, path_t)
        if self.last_command_fk_world is None:
            return 0.0
        return float(np.linalg.norm(command_fk_world - self.last_command_fk_world))

    def _update_command_fk_monitor(
            self, q: np.ndarray, base_t: float,
            path_t: float = None):
        if self.phase != 'track':
            return
        if path_t is None:
            path_t = self._path_time_at_offset(base_t, 0.0)
        command_fk_world = self._command_fk_world(q, base_t, path_t)
        if self.last_command_fk_world is not None:
            step = float(np.linalg.norm(command_fk_world - self.last_command_fk_world))
            if step > self.command_fk_step_warn_m:
                period = float(getattr(
                    self, 'trajectory_period_sec', traj.EE_PERIOD_SEC))
                phase_deg = (360.0 * path_t / period) % 360.0
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
        if mode not in (
            'legacy_fixed_step', 'expected_shadow', 'expected_residual',
            'state_safe_filter',
        ):
            raise ValueError(
                "guard_mode must be 'legacy_fixed_step'|'expected_shadow'|"
                f"'expected_residual'|'state_safe_filter', got '{mode}'")
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

    def _desired_world_accel(
            self, path_t: float, path_rate: float = 1.0,
            eps: float = 1e-3) -> np.ndarray:
        """Expected world acceleration with respect to the wall/base clock."""
        path_step = eps * max(0.0, float(path_rate))
        p_p = self._ee_target_world(path_t + path_step)
        p_0 = self._ee_target_world(path_t)
        p_m = self._ee_target_world(path_t - path_step)
        return (p_p - 2.0 * p_0 + p_m) / (eps * eps)

    def _guard_fk_world(
            self, q: np.ndarray, base_t: float,
            path_t: float = None):
        """World command FK with independent physical-base and path clocks."""
        if path_t is None:
            path_t = base_t
        base_pos = self._base_pose(base_t)
        base_rot = self._base_rotation(base_t, path_t)
        ee_pos_base, ee_rot_base = self.mpc_ik.forward(q)
        return base_pos + base_rot @ ee_pos_base, base_rot @ ee_rot_base

    def _guard_eval_between(
            self, q_anchor: np.ndarray, base_t_anchor: float,
            q_candidate: np.ndarray, base_t_new: float,
            path_t_anchor: float = None, path_t_new: float = None,
            path_rate: float = 1.0) -> dict:
        """Evaluate command/reference increments between two explicit states.

        r_vec = Δp_cmd - Δp_ref = e_new - e_last(两种算法代数恒等,单元测试逐拍核对);
        r_allow = r_abs + r_rel*||Δp_ref|| + r_acc*||a_ref||*Δt² + uncertainty。
        """
        if path_t_anchor is None:
            path_t_anchor = base_t_anchor
        if path_t_new is None:
            path_t_new = base_t_new
        p_cmd_new, r_cmd_new = self._guard_fk_world(
            q_candidate, base_t_new, path_t_new)
        p_ref_new = self._ee_target_world(path_t_new)
        p_cmd_anchor, r_cmd_anchor = self._guard_fk_world(
            q_anchor, base_t_anchor, path_t_anchor)
        p_ref_anchor = self._ee_target_world(path_t_anchor)
        e_new = p_cmd_new - p_ref_new
        R_err_new = (
            self._ee_trajectory_sample(path_t_new).rotation.T @ r_cmd_new)
        e_anchor = p_cmd_anchor - p_ref_anchor
        R_err_anchor = (
            self._ee_trajectory_sample(path_t_anchor).rotation.T
            @ r_cmd_anchor)
        ev = {
            'p_cmd_new': p_cmd_new,
            'p_ref_new': p_ref_new,
            'e_new': e_new,
            'R_err_new': R_err_new,
            't_new': float(base_t_new),
            'path_t_new': float(path_t_new),
            'r_ori': float(np.linalg.norm(
                pin.log3(R_err_anchor.T @ R_err_new))),
            'first_anchor': False,
        }
        dp_ref = p_ref_new - p_ref_anchor
        dp_cmd = p_cmd_new - p_cmd_anchor
        r_vec = dp_cmd - dp_ref
        # Curvature allowance belongs to path progress, not physical base
        # elapsed time. Using Δpath² also remains exact when online scaling
        # changes between accepted anchors or pauses at scale=0.
        a_ref = self._desired_world_accel(path_t_new, 1.0)
        elapsed = max(float(base_t_new) - float(base_t_anchor), 0.0)
        path_elapsed = max(
            float(path_t_new) - float(path_t_anchor), 0.0)
        allow_abs = self.guard_residual_abs_m
        allow_rel = self.guard_residual_relative_gain * float(np.linalg.norm(dp_ref))
        allow_acc = (
            self.guard_residual_accel_gain
            * float(np.linalg.norm(a_ref))
            * path_elapsed * path_elapsed
        )
        ev.update(
            r_vec=r_vec,
            r_pos=float(np.linalg.norm(r_vec)),
            r_allow=(
                allow_abs + allow_rel + allow_acc
                + self._guard_uncertainty_m),
            allow_abs=allow_abs,
            allow_rel=allow_rel,
            allow_acc=allow_acc,
            expected_step=float(np.linalg.norm(dp_ref)),
            command_step=float(np.linalg.norm(dp_cmd)),
            elapsed=elapsed,
            path_elapsed=path_elapsed,
            e_anchor=e_anchor,
            error_new=float(np.linalg.norm(e_new)),
            error_anchor=float(np.linalg.norm(e_anchor)),
        )
        return ev

    def _guard_expected_eval(
            self, q_candidate: np.ndarray, base_t_new: float,
            path_t_new: float = None, path_rate: float = 1.0) -> dict:
        """Evaluate against the last accepted-command anchor (legacy modes)."""
        if path_t_new is None:
            path_t_new = base_t_new
        p_cmd_new, r_cmd_new = self._guard_fk_world(
            q_candidate, base_t_new, path_t_new)
        p_ref_new = self._ee_target_world(path_t_new)
        e_new = p_cmd_new - p_ref_new
        R_err_new = (
            self._ee_trajectory_sample(path_t_new).rotation.T @ r_cmd_new)
        ev = {'p_cmd_new': p_cmd_new, 'p_ref_new': p_ref_new, 'e_new': e_new,
              'R_err_new': R_err_new, 't_new': float(base_t_new),
              'path_t_new': float(path_t_new),
              'r_ori': float('nan'), 'first_anchor': self._guard_q_last is None}
        if self._guard_q_last is None:
            ev.update(r_vec=np.full(3, float('nan')), r_pos=0.0, r_allow=float('inf'),
                      allow_abs=self.guard_residual_abs_m, allow_rel=float('nan'),
                      allow_acc=float('nan'), expected_step=float('nan'),
                      command_step=float('nan'))
            return ev
        return self._guard_eval_between(
            self._guard_q_last,
            self._guard_t_last_reference,
            q_candidate,
            base_t_new,
            self._guard_path_t_last_reference,
            path_t_new,
            path_rate,
        )

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

    def _guard_project(
            self, q_next: np.ndarray, q_current: np.ndarray,
            base_t_new: float, r_ori_baseline: float,
            path_t_new: float = None, path_rate: float = 1.0):
        """确定性 λ 搜索: 在 [min_lambda, 1] 内找最大安全比例。

        先探 min_lambda(不可行 => 直接放弃,返回 None => block);再固定次数二分,
        每次评估 = 1 次 FK + 常数向量运算,迭代上限 guard_projection_max_iterations,
        最坏计算量固定,不阻塞 50Hz 控制周期。返回 (λ, q_safe, ev_safe) 或 None。
        """
        q_anchor = self._guard_q_last

        def try_lambda(lam: float):
            q_s = q_anchor + lam * (q_next - q_anchor)
            ev = self._guard_expected_eval(
                q_s, base_t_new, path_t_new, path_rate)
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

    def _guard_project_from_state(
            self, q_next: np.ndarray, q_current: np.ndarray,
            base_t_state: float, base_t_new: float,
            r_ori_baseline: float, path_t_state: float = None,
            path_t_new: float = None, path_rate: float = 1.0):
        """Largest safe interpolation from the predicted current state."""
        q_anchor = np.asarray(q_current, dtype=float)

        def try_lambda(lam: float):
            q_s = q_anchor + lam * (
                np.asarray(q_next, dtype=float) - q_anchor)
            ev = self._guard_eval_between(
                q_anchor, base_t_state, q_s, base_t_new,
                path_t_state, path_t_new, path_rate)
            if (
                ev['r_pos'] <= ev['r_allow']
                and self._guard_candidate_safe(
                    q_s, q_anchor, ev, r_ori_baseline)
            ):
                return q_s, ev
            return None

        # λ=0 is a valid bound for the soft safety filter.  Unlike the
        # accepted-command projection, no stale command anchor can make the
        # search geometrically infeasible before task-space constraints apply.
        best = try_lambda(0.0)
        if best is None:
            return None
        best_lam = 0.0
        lo, hi = 0.0, 1.0
        for _ in range(self.guard_projection_max_iterations):
            mid = 0.5 * (lo + hi)
            cand = try_lambda(mid)
            if cand is not None:
                best, best_lam, lo = cand, mid, mid
            else:
                hi = mid
        return best_lam, best[0], best[1]

    def _guard_backup_command(
            self, q_current: np.ndarray, base_t_new: float,
            path_t_new: float = None) -> np.ndarray:
        """Bounded damped-least-squares command that actively holds the world target."""
        if path_t_new is None:
            path_t_new = base_t_new
        q_current = np.asarray(q_current, dtype=float)
        p_hold, _ = self._guard_fk_world(
            q_current, base_t_new, path_t_new)
        target = self._ee_target_world(path_t_new)
        base_rotation = self._base_rotation(base_t_new, path_t_new)
        jacobian_base = pin.computeFrameJacobian(
            self.mpc_ik.model,
            self.mpc_ik.data,
            q_current,
            self.mpc_ik.frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )[:3, :]
        jacobian_world = base_rotation @ jacobian_base
        damping = max(float(self.guard_backup_damping), 1e-6)
        desired_step = (
            float(self.guard_backup_position_gain) * (target - p_hold))
        joint_step = jacobian_world.T @ np.linalg.solve(
            jacobian_world @ jacobian_world.T
            + damping * damping * np.eye(3),
            desired_step,
        )

        # Position, velocity and acceleration bounds are all enforced around
        # the predicted application-time state.
        velocity = np.clip(
            joint_step / self.dt,
            -self.second_order_max_velocity,
            self.second_order_max_velocity,
        )
        previous_velocity = np.asarray(
            self._so_last_cmd_dq
            if self._so_last_cmd_dq is not None
            else np.zeros(JOINT_COUNT),
            dtype=float,
        )
        dv = self.second_order_max_acceleration * self.dt
        velocity = np.clip(
            velocity,
            previous_velocity - dv,
            previous_velocity + dv,
        )
        joint_step = np.clip(
            velocity * self.dt,
            -self.max_joint_step,
            self.max_joint_step,
        )
        return np.clip(
            q_current + joint_step,
            self.mpc_ik.q_lower,
            self.mpc_ik.q_upper,
        )

    def _guard_accept(self, q_cmd: np.ndarray, ev: dict):
        """只有**实际接受并发布**的命令才更新守卫锚点(shadow 模式=legacy 接受的命令;
        residual 模式=λ 结果)。被拒绝命令/求解失败/纯 shadow 计算不得调用。"""
        self._guard_q_last = np.asarray(q_cmd, dtype=float).copy()
        self._guard_t_last_reference = float(ev['t_new'])
        self._guard_path_t_last_reference = float(
            ev.get('path_t_new', ev['t_new']))
        self._guard_p_cmd_last = ev['p_cmd_new']
        self._guard_p_ref_last = ev['p_ref_new']
        self._guard_R_err_last = ev['R_err_new']
        self.guard_consecutive_blocks = 0

    def _guard_enter_fault(self, path_time: float = None):
        """连续拦截达上限: 显式 fault/abort。保持位置、停止轨迹跟踪;
        **不**通过自动重锚把异常命令放行。"""
        self.guard_fault = True
        self.phase = 'guard_fault'
        if path_time is None:
            path_time = (
                float(self.time_scaler.path_time)
                if getattr(self, 'online_time_scaling_enabled', False)
                else 0.0)
        self._fault_path_time = max(0.0, float(path_time))
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
            # 逆变换速度可略高于模型 vmax(位置反馈项补偿),上限取 plant 实际
            # 接受的速度而非 max_joint_step/dt,否则前馈被静默削顶。
            velocity_limit = max(
                self.max_joint_step / self.dt, self.plant_max_velocity)
            msg.velocity = np.clip(dq_arr, -velocity_limit, velocity_limit).tolist()
        self.pub.publish(msg)
        publish_wall_sec = publish_time.nanoseconds * 1e-9
        self._cmd_history.append((
            publish_wall_sec,
            q_executed.copy(),
            np.asarray(msg.velocity, dtype=float).copy(),
        ))
        return (
            q_executed.copy(),
            np.asarray(msg.velocity, dtype=float),
            publish_wall_sec,
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
            provenance = self._load_provenance()
            executor_entry = (
                (provenance.get('node_params', {}) or {}).get(
                    '/student_arm_node', {})
                if isinstance(provenance, dict) else {})
            executor_params = (
                executor_entry.get('key_params', {})
                if isinstance(executor_entry, dict) else {})

            def finite_executor_value(name):
                try:
                    value = float(executor_params.get(name))
                except (TypeError, ValueError):
                    return None
                return value if math.isfinite(value) and value > 0.0 else None

            executor_vmax = finite_executor_value('max_velocity')
            executor_amax = finite_executor_value('max_acceleration')
            accel_enabled = executor_params.get(
                'enable_acceleration_limit')
            if not isinstance(accel_enabled, bool):
                accel_enabled = None
            snapshot = {
                'schema_version': 7,
                'created': datetime.now().isoformat(timespec='seconds'),
                'node': self.get_name(),
                'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '(unset=0)'),
                'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION', '(default)'),
                'fastdds_profiles_file': os.environ.get(
                    'FASTRTPS_DEFAULT_PROFILES_FILE', '(unset)'),
                'publish_rate_hz': self.control_rate_hz,
                'control_dt': self.dt,
                'joint_count': JOINT_COUNT,
                'reference_lead_steps': self.reference_lead_steps,
                'solver_model': self.solver_model,
                'dq_prev_mode': self.dq_prev_mode,
                'rti_iteration_mode': self.rti_iteration_mode,
                'u0_active_threshold_rad': 0.0195,
                # Keep actual executor, controller model and OCP constraints
                # separate.  They were historically conflated into a
                # hard-coded ``accel_limit_rad_s2=5`` field, which made
                # saturation reports wrong after the plant moved to 3/20.
                'executor_limits': {
                    'velocity_rad_s': executor_vmax,
                    'acceleration_rad_s2': executor_amax,
                    'acceleration_limit_enabled': accel_enabled,
                    'source': (
                        'provenance:/student_arm_node:key_params'
                        if executor_vmax is not None
                        or executor_amax is not None
                        else 'unavailable'),
                },
                'controller_plant_model_limits': {
                    'velocity_rad_s': self.plant_max_velocity,
                    'acceleration_rad_s2': self.plant_max_acceleration,
                },
                'solver_limits': {
                    'velocity_rad_s': self.second_order_max_velocity,
                    'acceleration_rad_s2':
                        self.second_order_max_acceleration,
                },
                'max_joint_step_rad': self.max_joint_step,
                'urdf_path': self.mpc_ik.urdf_path,
                'ee_frame': self.mpc_ik.ee_frame,
                'provenance': provenance,
                'graph': graph,
                'experiment_valid': graph.get('joint_states_publisher_count') == 1,
                'runtime_validation': {
                    'valid': True,
                    'transport_fault': False,
                },
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
                        'second_order_cartesian_residual_mode',
                        'second_order_cartesian_velocity_weight',
                        'second_order_cartesian_acceleration_weight',
                        'second_order_terminal_cartesian_velocity_weight',
                        'second_order_feasible_reference_projection',
                        'second_order_feasible_reference_damping',
                        'second_order_contour_tracking_enabled',
                        'second_order_contour_position_weight',
                        'second_order_lag_position_weight',
                        'second_order_transition_position_weight',
                        'second_order_terminal_contour_position_weight',
                        'second_order_terminal_lag_position_weight',
                        'second_order_terminal_transition_position_weight',
                        'reference_derivative_dt_sec',
                        'second_order_acceleration_weight',
                        'second_order_terminal_velocity_weight',
                        'second_order_current_posture_weight',
                        'second_order_center_posture_weight',
                        'second_order_max_velocity',
                        'second_order_max_acceleration',
                        'second_order_min_jacobian_sigma',
                        'second_order_dq_ff_compensation',
                        'second_order_reference_lead_steps',
                        'second_order_state_prediction',
                        'second_order_velocity_source', 'control_rate_hz',
                        'latency_compensation_enabled',
                        'latency_nominal_publish_ms',
                        'latency_publish_ewma_alpha',
                        'latency_actuator_delay_ms',
                        'latency_prediction_max_ms',
                        'smith_predictor_enabled',
                        'command_inversion_enabled',
                        'plant_velocity_feedforward_gain',
                        'plant_position_feedback_tau',
                        'plant_feedback_velocity_limit',
                        'plant_max_velocity',
                        'plant_max_acceleration',
                        'plant_control_substep_dt',
                        'base_disturbance_amplitude_m',
                        'base_disturbance_seed',
                        'base_disturbance_profile',
                        'base_belief_mode',
                        'base_estimator_noise_std_m',
                        'base_estimator_delay_ms',
                        'base_estimator_lpf_hz',
                        'base_estimator_seed',
                        'trust_n_freq', 'trust_fmin_hz', 'trust_fmax_hz',
                        'trust_lam', 'trust_sigma0', 'trust_prior_std',
                        'trust_meas_delay_sec', 'trust_dt_nominal',
                        'trust_n_peaks', 'trust_ident_period_sec',
                        'trust_buffer_sec', 'trust_min_ident_sec',
                        'trust_holdout_sec', 'trust_f_ident_min_hz',
                        'trust_f_ident_max_hz', 'trust_peak_snr',
                        'trust_resid_lpf_hz', 'trust_beta_sigma0',
                        'trust_beta_gamma_f', 'trust_beta_n_grid',
                        'trust_beta_degraded', 'trust_beta_healthy',
                        'uncertainty_control_mode',
                        'trust_regime_enabled',
                        'trust_regime_stable_tau_sec',
                        'trust_regime_stable_process_std',
                        'trust_regime_persistent_process_std',
                        'trust_regime_measurement_std',
                        'trust_regime_stable_stay_probability',
                        'trust_regime_persistent_stay_probability',
                        'trust_regime_initial_persistent_probability',
                        'uncertainty_risk_weight',
                        'uncertainty_applicability_risk_gain',
                        'uncertainty_smoothing_weight',
                        'uncertainty_gain_fall_tau_sec',
                        'uncertainty_gain_rise_tau_sec',
                        'uncertainty_risk_budget_m',
                        'uncertainty_min_speed_scale',
                        'uncertainty_min_position_weight_scale',
                        'uncertainty_guard_radius_gain',
                        'uncertainty_guard_radius_max_m',
                        'trust_feedforward_gating_enabled',
                        'trust_feedforward_gate_low',
                        'trust_feedforward_gate_high',
                        'trust_feedforward_gate_fall_tau_sec',
                        'trust_feedforward_gate_rise_tau_sec',
                        'trust_n_cal', 'trust_alpha', 'trust_gamma_aci',
                        'trust_shadow_csv',
                        'trust_ladder_enabled', 'ladder_b_degraded',
                        'ladder_b_recover',
                        'ladder_delta_degraded_ratio',
                        'ladder_delta_recover_ratio',
                        'ladder_min_scale',
                        'ladder_baseline_warmup_sec',
                        'ladder_baseline_tau_sec',
                        'ladder_trust_commission_sec',
                        'ladder_recover_debounce_sec',
                        'ladder_exclusive_scale_control',
                        'late_tick_recovery_enabled',
                        'late_tick_recovery_factor',
                        'periodic_error_learning_enabled',
                        'periodic_error_learning_gain',
                        'periodic_error_correction_limit_m',
                        'periodic_error_update_limit_m',
                        'periodic_error_min_samples_per_cycle',
                        'periodic_error_initial_coefficients',
                        'periodic_error_learning_freeze',
                        'trajectory_shape', 'trajectory_time_scale',
                        'base_motion_time_scale',
                        'posture_reference_mode',
                        'online_time_scaling_enabled', 'time_scaling_min_scale',
                        'time_scaling_velocity_reserve', 'time_scaling_slow_error_m',
                        'time_scaling_stop_error_m',
                        'path_progress_topic',
                        'transport_delay_guard_enabled',
                        'transport_delay_threshold_ms',
                        'transport_delay_fault_duration_sec',
                        'transport_delay_min_fresh_samples',
                        'feedback_stale_timeout_ms',
                        'command_fk_step_warn_m', 'reference_lead_steps', 'dq_prev_mode',
                        'base_reference_lead_steps',
                        'rti_iteration_mode', 'record_loop_csv', 'loop_data_dir',
                        'dart_noise_std', 'dart_noise_clip_sigma', 'dart_noise_seed',
                        'guard_mode', 'guard_residual_abs_m',
                        'guard_residual_relative_gain', 'guard_residual_accel_gain',
                        'guard_projection_enabled', 'guard_projection_min_lambda',
                        'guard_projection_max_iterations', 'guard_max_consecutive_blocks',
                        'guard_backup_damping', 'guard_backup_position_gain',
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
            self._params_snapshot_path = path
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
        #   predicted_application_delay_ms = 上一拍 EWMA 发布延迟 + 配置执行器延迟。
        #   state_prediction_age_ms = feedback source stamp 到预计命令生效时刻的总前推量。
        #   js_header_stamp_rel = 求解所用 /joint_states header 时间戳相对原点(actual 序列时间轴)。
        #   t_solve_wall/t_publish_wall = 绝对 wall(catch-up/时钟诊断用)。
        cols = ['t_solve', 't_publish', 't_reference', 't_next', 'reference_lead_steps',
                'path_time', 'path_scale', 'path_target_time',
                'timer_interval_ms', 'solve_to_publish_ms', 'feedback_age_ms',
                'transport_gap_ms', 'transport_fault',
                'late_tick_recovery',
                'predicted_application_delay_ms', 'state_prediction_age_ms',
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
            # 守卫诊断。guard_mode_code: 0 legacy / 1 shadow / 2 accepted-anchor
            # residual / 3 application-time state_safe_filter。
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
            'guard_mode_code',
            'guard_t_new', 'guard_path_t_new',
            'guard_t_last_reference', 'guard_path_t_last_reference',
            'guard_r_pos_m', 'guard_r_allow_m',
            'guard_allow_abs_m', 'guard_allow_rel_m', 'guard_allow_acc_m',
            'guard_r_vec_x', 'guard_r_vec_y', 'guard_r_vec_z',
            'guard_expected_step_m', 'guard_command_step_m', 'guard_margin_m',
            'guard_lambda', 'guard_projected', 'guard_backup',
            'guard_consecutive_blocks', 'guard_fault',
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
            'so_linearization_time_ms', 'so_projection_time_ms',
            'so_total_optimize_time_ms',
            'so_projection_velocity_residual_mps',
            'so_projection_acceleration_residual_mps2',
            'so_projection_velocity_active_ratio',
            'so_projection_acceleration_active_ratio',
            # 基座周期误差学习诊断。error 是实际 world 末端相对原始目标，
            # correction 是加到 NMPC 参考上的 world 系学习补偿。
            'periodic_learning_active', 'periodic_learning_key',
            'periodic_error_x', 'periodic_error_y', 'periodic_error_z',
            'periodic_error_norm_m',
            'periodic_correction_x', 'periodic_correction_y',
            'periodic_correction_z', 'periodic_correction_norm_m',
            'periodic_coefficient_norm_m',
        ]
        return cols

    def _append_loop_row(
        self, timing, dq_prev, q_current, dq_measured, q_next, q_published, dq_published,
        info, command_usable, tracking_ok, guard_block, command_fk_step, guard_diag=None,
        shadow_diag=None, dart_diag=None, periodic_diag=None,
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
            js_header_stamp_sec = float(timing.get(
                'js_header_stamp_sec', self._js_header_stamp_sec))
            js_recv_wall_sec = float(timing.get(
                'js_recv_wall_sec', self._js_recv_wall_sec))
            js_rel = (js_header_stamp_sec - origin
                      if origin is not None and np.isfinite(js_header_stamp_sec)
                      else float('nan'))
            row = [
                timing['t_solve'],
                timing['t_publish'],
                timing['t_reference'],
                timing['t_next'],
                int(self.reference_lead_steps),
                float(timing.get(
                    'path_time',
                    self.time_scaler.path_time
                    if self.online_time_scaling_enabled
                    else timing['t_solve'])),
                float(timing.get(
                    'path_scale',
                    self.time_scaler.scale
                    if self.online_time_scaling_enabled else 1.0)),
                float(self._last_path_target_time0),
                timing['timer_interval_ms'],
                timing['solve_to_publish_ms'],
                timing['feedback_age_ms'],
                timing.get('transport_gap_ms', float('nan')),
                int(bool(timing.get('transport_fault', False))),
                int(bool(timing.get('late_tick_recovery', False))),
                timing.get('predicted_application_delay_ms', float('nan')),
                timing.get('state_prediction_age_ms', float('nan')),
                js_rel,
                js_header_stamp_sec,
                js_recv_wall_sec,
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
            brpy = pin.rpy.matrixToRpy(self._base_rotation(
                timing['t_solve'],
                float(timing.get(
                    'path_time',
                    self.time_scaler.path_time
                    if self.online_time_scaling_enabled
                    else timing['t_solve']))))
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
                float(gd.get('path_t_new', float('nan'))),
                float(gd.get('t_last_reference', float('nan'))),
                float(gd.get(
                    'path_t_last_reference', float('nan'))),
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
                int(bool(gd.get('backup', False))),
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
                float(sd.get('linearization_time_ms', float('nan'))),
                float(sd.get('projection_time_ms', float('nan'))),
                float(sd.get('total_optimize_time_ms', float('nan'))),
                float(sd.get('projection_velocity_residual', float('nan'))),
                float(sd.get('projection_acceleration_residual', float('nan'))),
                float(sd.get('projection_velocity_active_ratio', float('nan'))),
                float(sd.get('projection_acceleration_active_ratio', float('nan'))),
            ]
            pd = periodic_diag or {}
            periodic_error = np.asarray(
                pd.get('error_world', np.full(3, np.nan)), dtype=float)
            periodic_correction = np.asarray(
                pd.get('correction_world', np.zeros(3)), dtype=float)
            row += [
                int(bool(pd.get('active', False))),
                int(pd.get('key', -1)),
                *[float(value) for value in periodic_error],
                float(pd.get('error_norm', float('nan'))),
                *[float(value) for value in periodic_correction],
                float(pd.get('correction_norm', 0.0)),
                float(pd.get('coefficient_norm', 0.0)),
            ]
            self._loop_writer.writerow(row)
            self._loop_rows_since_flush += 1
            if self._loop_rows_since_flush >= 100:
                self._loop_file.flush()
                self._loop_rows_since_flush = 0
        except Exception as exc:
            self._loop_record_failed = True
            self.get_logger().warn(
                'loop CSV 记录失败,后续 tick 停止记录(不影响控制): '
                f'{exc}\n{traceback.format_exc()}')

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
                    f'late_tick_recoveries={self.late_tick_recovery_count}; '
                    f'DART sigma={self.dart_noise_std:.6f}rad, clipped '
                    f'{self.dart_clipped_count}/{self.dart_track_ticks}; '
                    f'dq_prev_mode={self.dq_prev_mode}, fallback '
                    f'{self.dq_prev_fallback_count}/{self.dq_prev_total_track_ticks} '
                    f'({fb_ratio:.3f}%)')
            except Exception:
                pass
            self._loop_file = None
            self._loop_writer = None

    def _append_transport_fault_row(
            self, solve_wall: float, timer_interval_ms: float,
            q_feedback: np.ndarray, dq_feedback: np.ndarray,
            js_header_stamp_sec: float, js_recv_wall_sec: float,
            feedback_age_ms: float, transport_gap_ms: float,
            published):
        """Write and flush one terminal loop row with transport_fault=1."""
        if published is None or not self.record_loop_csv:
            return
        q_published, dq_published, publish_wall, dart_diag = published
        q_current = (
            np.asarray(q_feedback, dtype=float)
            if q_feedback is not None
            else np.asarray(q_published, dtype=float))
        zeros = np.zeros(JOINT_COUNT)
        dq_prev = {
            'input': zeros,
            'raw': zeros,
            'velocity_estimate': zeros,
            'source': 0,
            'fallback': 90,
            'feedback_dt': float('nan'),
        }
        origin = self.traj_origin_wall
        t_solve = (
            max(0.0, solve_wall - origin)
            if origin is not None else 0.0)
        t_publish = (
            max(0.0, publish_wall - origin)
            if origin is not None else 0.0)
        path_t = (
            self._fault_path_time
            if math.isfinite(self._fault_path_time) else 0.0)
        timing = {
            't_solve': t_solve,
            't_publish': t_publish,
            't_reference': float('nan'),
            't_next': t_publish,
            'path_time': path_t,
            'path_scale': 0.0,
            'timer_interval_ms': timer_interval_ms,
            'solve_to_publish_ms': max(
                0.0, (publish_wall - solve_wall) * 1000.0),
            'feedback_age_ms': feedback_age_ms,
            'transport_gap_ms': transport_gap_ms,
            'transport_fault': True,
            'late_tick_recovery': False,
            'predicted_application_delay_ms': float('nan'),
            'state_prediction_age_ms': float('nan'),
            'js_header_stamp_sec': js_header_stamp_sec,
            'js_recv_wall_sec': js_recv_wall_sec,
            't_solve_wall': solve_wall,
            't_publish_wall': publish_wall,
        }
        info = {
            'status': -30,
            'success': False,
            'nit': 0,
            'message': 'transport_fault',
            'command_usable': False,
            'tracking_ok': False,
            'plan_feasible': False,
            'terminal_error': float('nan'),
            'first_error': float('nan'),
            'solve_time': 0.0,
        }
        self._append_loop_row(
            timing,
            dq_prev,
            q_current=q_current,
            dq_measured=dq_feedback,
            q_next=q_published,
            q_published=q_published,
            dq_published=dq_published,
            info=info,
            command_usable=False,
            tracking_ok=False,
            # Transport freshness is an independent hard gate.  Do not make
            # the terminal row look like a command-residual guard rejection,
            # otherwise offline analysis attributes the DDS failure to 1E.
            guard_block=False,
            command_fk_step=float('nan'),
            guard_diag={
                'mode_code': {
                    'legacy_fixed_step': 0,
                    'expected_shadow': 1,
                    'expected_residual': 2,
                    'state_safe_filter': 3,
                }[self.guard_mode],
                't_new': t_solve,
                'path_t_new': path_t,
                'consecutive': self.guard_consecutive_blocks,
                'fault': bool(self.guard_fault),
            },
            dart_diag=dart_diag,
        )
        if self._loop_file is not None:
            self._loop_file.flush()
            self._loop_rows_since_flush = 0

    def destroy_node(self):
        gc.enable()
        self._log_trust_cost_summary()
        self._close_loop_csv()
        self._close_trust_csv()
        super().destroy_node()

    def _run_reset_phase(self):
        self._publish_joint_positions(self.reset_q_target)
        self._print_joint_angles(0.0)
        if self.q_measured is None:
            self._warn_missing_joint_states()
            return

        if self.transport_delay_guard_enabled:
            now_wall = self.get_clock().now().nanoseconds * 1e-9
            local_age_ms = (
                (now_wall - self._js_recv_wall_sec) * 1000.0
                if np.isfinite(self._js_recv_wall_sec)
                else float('inf'))
            source_gap_ms = (
                (self._js_recv_wall_sec - self._js_header_stamp_sec) * 1000.0
                if (
                    np.isfinite(self._js_recv_wall_sec)
                    and np.isfinite(self._js_header_stamp_sec)
                )
                else float('inf'))
            if self._update_transport_delay_guard(
                    source_gap_ms, self.dt,
                    self._js_header_stamp_sec, local_age_ms):
                published = self._enter_transport_fault(
                    source_gap_ms,
                    local_age_ms,
                    self._js_header_stamp_sec,
                    path_time=0.0)
                self._publish_path_progress(
                    now_wall, 0.0, path_time=0.0, path_scale=0.0)
                self._append_transport_fault_row(
                    now_wall, float('nan'),
                    self.q_measured, self.dq_measured,
                    self._js_header_stamp_sec, self._js_recv_wall_sec,
                    local_age_ms, source_gap_ms, published)
                return
            if (
                local_age_ms > self.feedback_stale_timeout_ms
                or abs(source_gap_ms) > self.transport_delay_threshold_ms
            ):
                self.reset_hold_time = 0.0
                self.get_logger().warn(
                    '复位姿态已到达，但 /joint_states 尚不满足新鲜度硬门：'
                    f'transport_gap={source_gap_ms:.1f}ms, '
                    f'feedback_age={local_age_ms:.1f}ms；'
                    '保持 reset，不进入正式轨迹。',
                    throttle_duration_sec=1.0,
                )
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
                self._guard_path_t_last_reference = float('nan')
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
                self._publish_latency_ewma_sec = self.latency_nominal_publish_sec
                self._last_application_delay_sec = 0.0
                self._last_state_prediction_age_sec = 0.0
                self._so_last_cmd_dq = None
                self._so_last_cmd_ddq = None
                self.late_tick_recovery_count = 0
                self._transport_gap_bad_ticks = 0
                self._transport_gap_warned = False
                self._transport_gap_bad_duration_sec = 0.0
                self._transport_gap_distinct_bad_samples = 0
                self._transport_gap_last_bad_stamp = float('nan')
                self._transport_gap_bad_active = False
                self.transport_fault = False
                self.transport_fault_gap_ms = float('nan')
                self._fault_path_time = float('nan')
                self._last_path_update_wall = None
                self._base_belief_offset = np.zeros(3)
                self._base_estimator_rng = np.random.default_rng(
                    self.base_estimator_seed)
                self._reset_trust_estimator()
                self._reset_periodic_error_learning()
                # 实时时钟卫生:控制阶段禁用 GC,消除周期性 GC 暂停造成的 solve->publish
                # 抖动尖峰(实测偶发 ~34ms)。仅本轮实验(数十秒)禁用,close 时恢复;
                # 不改任何控制逻辑(dq_prev/RTI/vff/守卫/horizon/权重/模型均不动)。
                gc.collect()
                gc.disable()
                self.get_logger().info('机械臂已到达轨迹起始点，开始 MPC 轨迹跟踪 (GC 已禁用降抖动)')
        else:
            self.reset_hold_time = 0.0

    def _apply_command_guard(
            self, q_next, q_current, t_solve, info, command_usable,
            t_state=None, path_t_solve=None, path_t_state=None,
            path_t_new=None, base_t_new=None):
        """求解器步长契约 + 四种 guard_mode 的决策与应用。

        副作用(与阶段1E之前的内联逻辑一致): 接受时更新 self.q_last / self.warm_start /
        守卫锚点。state_safe_filter 使用预计命令生效时的 q_current/t_state，
        超限时先投影，再用主动 world-target backup，只有两者都无效才 block。
        返回 dict: command_usable(硬拒绝后可能变 False)、dq_cmd、guard_block、
        command_fk_step 及 CSV 守卫诊断字段。
        """
        q_next = np.asarray(q_next, dtype=float)
        q_current = np.asarray(q_current, dtype=float)
        t_state = float(t_solve if t_state is None else t_state)
        if path_t_solve is None:
            path_t_solve = self._path_time_at_offset(t_solve, 0.0)
        dq_cmd = np.zeros(JOINT_COUNT)
        guard_block = False
        command_fk_step = float('nan')
        t_new = float(
            base_t_new
            if base_t_new is not None
            else t_state + self.dt
            if self.guard_mode == 'state_safe_filter'
            else t_solve + self.reference_lead_steps * self.dt)
        if path_t_state is None:
            path_t_state = self._path_time_at_offset(
                t_solve, max(0.0, t_state - t_solve))
        if path_t_new is None:
            path_t_new = self._path_time_at_offset(
                t_solve, max(0.0, t_new - t_solve))
        path_rate = (
            float(self.time_scaler.scale)
            if self.online_time_scaling_enabled else 1.0)
        guard_ev = None
        guard_t_last_used = float('nan')
        guard_path_t_last_used = float('nan')
        guard_lambda = float('nan')
        guard_projected = False
        guard_backup = False
        solver_step_clipped = False
        contract_lambda = 1.0
        solver_step_violation = False
        jstep_next_current = float('nan')
        jstep_next_anchor = float('nan')
        jstep_current_anchor = float('nan')
        jstep_argmax = -1
        if command_usable:
            # 阶段1E(六): 求解器步长契约 max|q_next-q_current| <= max_joint_step + eps。
            jstep_vec = np.abs(q_next - q_current)
            jstep_next_current = float(np.max(jstep_vec))
            jstep_argmax = int(np.argmax(jstep_vec)) + 1
            if self._guard_q_last is not None:
                jstep_next_anchor = float(np.max(np.abs(
                    q_next - self._guard_q_last)))
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
                    + ('; residual 模式硬拒绝,保持上一帧'
                       if self.guard_mode == 'expected_residual'
                       else '; state_safe_filter 将先裁剪再进入安全滤波'
                       if self.guard_mode == 'state_safe_filter'
                       else '; legacy/shadow 模式仅记录(拦截行为与阶段1E之前一致)'))
                if (
                    self.guard_mode == 'state_safe_filter'
                    and np.all(np.isfinite(jstep_vec))
                    and jstep_next_current > 0.0
                ):
                    contract_lambda = (
                        self.max_joint_step / jstep_next_current)
                    q_next = (
                        q_current
                        + (q_next - q_current)
                        * contract_lambda
                    )
                    q_next = np.clip(
                        q_next, self.mpc_ik.q_lower, self.mpc_ik.q_upper)
                    jstep_vec = np.abs(q_next - q_current)
                    jstep_next_current = float(np.max(jstep_vec))
                    guard_projected = True
                    solver_step_clipped = True
                    self.warm_start = None
                elif self.guard_mode in (
                    'expected_residual', 'state_safe_filter'):
                    # 硬拒绝: 按命令不可用处理(保持上一帧),不计入守卫拦截。
                    command_usable = False
                    self.warm_start = None
        if command_usable:
            command_fk_step = self._command_fk_step(
                q_next, t_solve, path_t_solve)
            if self.guard_mode not in (
                    'expected_residual', 'state_safe_filter'):
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
                    guard_path_t_last_used = (
                        self._guard_path_t_last_reference)
                    guard_ev = self._guard_expected_eval(
                        np.asarray(q_next, dtype=float), t_new,
                        path_t_new, path_rate)
                    if not guard_block:
                        self._guard_accept(np.asarray(q_next, dtype=float), guard_ev)
            elif self.guard_mode == 'expected_residual':
                # expected_residual 主动守卫: 残差超限 -> λ 投影 -> 仍不可行才 hold。
                q_cand = np.asarray(q_next, dtype=float)
                guard_t_last_used = self._guard_t_last_reference
                guard_path_t_last_used = (
                    self._guard_path_t_last_reference)
                guard_ev = self._guard_expected_eval(
                    q_cand, t_new, path_t_new, path_rate)
                if guard_ev['r_pos'] <= guard_ev['r_allow']:
                    guard_lambda = 1.0                       # 正常通过,不修改命令
                    self.q_last = q_cand
                    dq_cmd = (q_cand - q_current) / self.dt
                    self.warm_start = info['trajectory']
                    self._guard_accept(q_cand, guard_ev)
                else:
                    proj = (self._guard_project(
                                q_cand, q_current, t_new,
                                guard_ev['r_ori'], path_t_new, path_rate)
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
                            self._guard_enter_fault(path_t_solve)
            else:
                # Application-time state safety filter.  Each tick starts from
                # the predicted current state, so one rejected command cannot
                # leave a stale accepted-command anchor that poisons all later
                # feasibility checks.
                q_cand = np.asarray(q_next, dtype=float)
                guard_t_last_used = t_state
                guard_path_t_last_used = path_t_state
                guard_ev = self._guard_eval_between(
                    q_current, t_state, q_cand, t_new,
                    path_t_state, path_t_new, path_rate)
                candidate_safe = self._guard_candidate_safe(
                    q_cand, q_current, guard_ev, guard_ev['r_ori'])
                if (
                    guard_ev['r_pos'] <= guard_ev['r_allow']
                    and candidate_safe
                ):
                    guard_lambda = (
                        contract_lambda if solver_step_clipped else 1.0)
                    self.q_last = q_cand
                    dq_cmd = (q_cand - q_current) / self.dt
                    self.warm_start = (
                        None if solver_step_clipped
                        else info['trajectory'])
                    if solver_step_clipped:
                        self.guard_projection_count += 1
                    self._guard_accept(q_cand, guard_ev)
                else:
                    proj = (
                        self._guard_project_from_state(
                            q_cand, q_current, t_state, t_new,
                            guard_ev['r_ori'], path_t_state, path_t_new,
                            path_rate)
                        if self.guard_projection_enabled else None
                    )
                    # λ=0 is geometrically safe but would merely hold the
                    # predicted joint state. Prefer an active world-target
                    # backup in that case.
                    if proj is not None and proj[0] > 1e-6:
                        guard_lambda, q_safe, ev_safe = proj
                        guard_projected = True
                        self.guard_projection_count += 1
                        self.q_last = q_safe
                        dq_cmd = (q_safe - q_current) / self.dt
                        self.warm_start = None
                        self._guard_accept(q_safe, ev_safe)
                        self.get_logger().warn(
                            '状态安全滤波投影: '
                            f'r_pos={guard_ev["r_pos"] * 1000.0:.2f}mm > '
                            f'r_allow={guard_ev["r_allow"] * 1000.0:.2f}mm, '
                            f'lambda={guard_lambda:.3f}',
                            throttle_duration_sec=1.0,
                        )
                    else:
                        q_backup = self._guard_backup_command(
                            q_current, t_new, path_t_new)
                        ev_backup = self._guard_eval_between(
                            q_current, t_state, q_backup, t_new,
                            path_t_state, path_t_new, path_rate)
                        ev_hold = self._guard_eval_between(
                            q_current, t_state, q_current, t_new,
                            path_t_state, path_t_new, path_rate)
                        backup_safe = (
                            np.all(np.isfinite(q_backup))
                            and self._guard_candidate_safe(
                                q_backup, q_current, ev_backup,
                                guard_ev['r_ori'])
                            and ev_backup['error_new']
                            <= ev_hold['error_new'] + 1e-9
                        )
                        if backup_safe:
                            guard_lambda = -1.0
                            guard_projected = True
                            guard_backup = True
                            self.guard_projection_count += 1
                            self.q_last = q_backup
                            dq_cmd = (q_backup - q_current) / self.dt
                            self.warm_start = None
                            self._guard_accept(q_backup, ev_backup)
                            guard_ev = ev_backup
                            self.get_logger().warn(
                                '状态安全滤波启用主动 backup: '
                                f'error={ev_hold["error_new"] * 1000.0:.2f}'
                                f'->{ev_backup["error_new"] * 1000.0:.2f}mm, '
                                f'r={ev_backup["r_pos"] * 1000.0:.2f}mm',
                                throttle_duration_sec=1.0,
                            )
                        else:
                            guard_block = True
                            guard_lambda = 0.0
                            self.guard_block_count += 1
                            self.guard_consecutive_blocks += 1
                            # Hold the predicted current state rather than the
                            # stale last accepted command. The next tick starts
                            # from a fresh state anchor, so recovery is possible.
                            self.q_last = np.clip(
                                q_current,
                                self.mpc_ik.q_lower,
                                self.mpc_ik.q_upper)
                            dq_cmd = np.zeros(JOINT_COUNT)
                            self.warm_start = None
                            if (
                                self.guard_consecutive_blocks
                                >= self.guard_max_consecutive_blocks
                            ):
                                self._guard_enter_fault(path_t_solve)
        elif self.guard_mode == 'state_safe_filter':
            # Solver failure/NaN must not fall back to a stale accepted joint
            # command while the base keeps moving. Use the same bounded local
            # controller as the residual-rejection path, anchored at the fresh
            # application-time state.
            guard_t_last_used = t_state
            guard_path_t_last_used = path_t_state
            ev_hold = self._guard_eval_between(
                q_current, t_state, q_current, t_new,
                path_t_state, path_t_new, path_rate)
            q_backup = self._guard_backup_command(
                q_current, t_new, path_t_new)
            ev_backup = self._guard_eval_between(
                q_current, t_state, q_backup, t_new,
                path_t_state, path_t_new, path_rate)
            backup_safe = (
                np.all(np.isfinite(q_backup))
                and self._guard_candidate_safe(
                    q_backup, q_current, ev_backup, float('inf'))
                and ev_backup['error_new'] <= ev_hold['error_new'] + 1e-9
            )
            if backup_safe:
                command_usable = True
                guard_lambda = -1.0
                guard_projected = True
                guard_backup = True
                self.guard_projection_count += 1
                self.q_last = q_backup
                dq_cmd = (q_backup - q_current) / self.dt
                self.warm_start = None
                self._guard_accept(q_backup, ev_backup)
                guard_ev = ev_backup
                self.get_logger().warn(
                    'NMPC 命令不可用，状态安全滤波切换到主动 backup: '
                    f'error={ev_hold["error_new"] * 1000.0:.2f}'
                    f'->{ev_backup["error_new"] * 1000.0:.2f}mm',
                    throttle_duration_sec=1.0,
                )
            else:
                guard_block = True
                guard_lambda = 0.0
                self.guard_block_count += 1
                self.guard_consecutive_blocks += 1
                self.q_last = np.clip(
                    q_current, self.mpc_ik.q_lower, self.mpc_ik.q_upper)
                dq_cmd = np.zeros(JOINT_COUNT)
                self.warm_start = None
                guard_ev = ev_hold
                if (
                    self.guard_consecutive_blocks
                    >= self.guard_max_consecutive_blocks
                ):
                    self._guard_enter_fault(path_t_solve)
        return {
            'command_usable': command_usable,
            'dq_cmd': dq_cmd,
            'guard_block': guard_block,
            'command_fk_step': command_fk_step,
            't_new': t_new,
            'path_t_new': path_t_new,
            'guard_ev': guard_ev,
            'guard_t_last_used': guard_t_last_used,
            'guard_path_t_last_used': guard_path_t_last_used,
            'guard_lambda': guard_lambda,
            'guard_projected': guard_projected,
            'guard_backup': guard_backup,
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
        late_tick_recovery = self._recover_from_late_tick(timer_interval_ms)
        q_feedback, dq_feedback, js_header_stamp_sec, js_recv_wall_sec = (
            self._feedback_snapshot())
        if q_feedback is None:
            self._warn_missing_joint_states()
            return
        # feedback_age 在**求解开始时**计算(不等发布/写盘)。
        feedback_age_ms = ((solve_wall - js_recv_wall_sec) * 1000.0
                           if np.isfinite(js_recv_wall_sec) else float('nan'))
        transport_gap_ms = (
            (js_recv_wall_sec - js_header_stamp_sec) * 1000.0
            if (np.isfinite(js_recv_wall_sec)
                and np.isfinite(js_header_stamp_sec))
            else float('nan'))
        if (
            np.isfinite(transport_gap_ms)
            and abs(transport_gap_ms) > self.transport_delay_threshold_ms
        ):
            self._transport_gap_bad_ticks += 1
        else:
            self._transport_gap_bad_ticks = 0
        if (self._transport_gap_bad_ticks >= 50
                and not self._transport_gap_warned):
            self._transport_gap_warned = True
            self.get_logger().error(
                f'/joint_states 传输延迟异常: recv-stamp 间隔持续 '
                f'>{self.transport_delay_threshold_ms:.0f}ms '
                f'(当前 {transport_gap_ms:.1f}ms)。反馈陈旧会让'
                '守卫大量投影、跟踪严重劣化。典型原因是 DDS 共享内存残留: '
                '请停止所有 ROS 节点后执行 '
                'rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 再重启。')
        guard_dt_sec = (
            timer_interval_ms * 1e-3
            if np.isfinite(timer_interval_ms) else self.dt)
        if self._update_transport_delay_guard(
                transport_gap_ms, guard_dt_sec, js_header_stamp_sec,
                feedback_age_ms):
            fault_path_t = self._path_time_at_offset(t_solve, 0.0)
            published = self._enter_transport_fault(
                transport_gap_ms,
                feedback_age_ms,
                js_header_stamp_sec,
                path_time=fault_path_t)
            self._publish_base_tf(t_solve, fault_path_t)
            self._publish_path_progress(
                solve_wall, t_solve,
                path_time=fault_path_t,
                path_scale=0.0)
            self._append_transport_fault_row(
                solve_wall, timer_interval_ms,
                q_feedback, dq_feedback,
                js_header_stamp_sec, js_recv_wall_sec,
                feedback_age_ms, transport_gap_ms, published)
            self._publish_world_markers(t_solve, fault_path_t)
            return
        application_delay_sec = self._predicted_application_delay_sec()
        self._last_application_delay_sec = application_delay_sec

        # estimated/model 信念模式: 先更新本拍 Δ̂(model 档缓存 predict_horizon
        # 时间网格), 之后本拍所有 _base_pose(参考生成/守卫/TF/记录)共享同一
        # 信念, 不会拍内不一致; trust 影子 CSV 也在此逐拍落盘。
        self._update_base_belief(t_solve)

        q_current = q_feedback
        feedback_time = (
            js_header_stamp_sec - self.traj_origin_wall
            if np.isfinite(js_header_stamp_sec)
            else float('nan')
        )
        periodic_diag = self._update_periodic_error_learning(
            q_current, feedback_time)
        if self.online_time_scaling_enabled:
            scale_dt = (
                max(0.0, solve_wall - self._last_path_update_wall)
                if self._last_path_update_wall is not None else 0.0)
            self._last_path_update_wall = solve_wall
            b4_risk_control = (
                self.base_belief_mode == 'model'
                and self.uncertainty_control_mode == 'b4')
            exclusive_ladder = (
                self.trust_ladder_enabled
                and self.ladder_exclusive_scale_control)
            external_scale_cap = min(
                self._ladder_scale_cap
                if self.trust_ladder_enabled else 1.0,
                self._uncertainty_speed_cap
                if b4_risk_control else 1.0,
            )
            self.time_scaler.update(
                scale_dt,
                (0.0 if (exclusive_ladder or b4_risk_control)
                 else self._time_scaling_utilization),
                (0.0 if (exclusive_ladder or b4_risk_control)
                 else self._time_scaling_tracking_error),
                external_scale_cap=external_scale_cap,
            )
        # scaler 更新后再统一派生 guard/TF 的双时钟。物理基座继续按
        # wall/base clock 运动，路径目标与 whole 工作面朝向按 path clock。
        base_t_state = t_solve + application_delay_sec
        base_t_new = base_t_state + self.dt
        path_t_solve = self._path_time_at_offset(t_solve, 0.0)
        path_t_state = self._path_time_at_offset(
            t_solve, application_delay_sec)
        path_t_new = self._path_time_at_offset(
            t_solve, application_delay_sec + self.dt)
        self._publish_base_tf(t_solve)
        # 阶段1B:计算上一控制周期实际执行增量(measured_delta),或 0(zero)。
        dqp = self._compute_dq_prev(
            q_current, js_header_stamp_sec, dq_feedback)
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
        q_control = q_current
        dq_control = (
            np.asarray(dq_feedback, dtype=float)
            if dq_feedback is not None
            else np.zeros(JOINT_COUNT))
        state_prediction_age_sec = 0.0
        if self.solver_model in ('first_order', 'shadow'):
            targets, rotations = self._target_horizon_base(
                t_solve,
                application_delay_sec=application_delay_sec)
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
                second_result = self._solve_second_order(
                    t_solve, solve_wall, q_current,
                    application_delay_sec, dq_feedback)
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
            second_result = self._solve_second_order(
                t_solve, solve_wall, q_current,
                application_delay_sec, dq_feedback)
            if second_result and second_result.get('available'):
                q_next = second_result['q_next']
                dq_model_next = second_result['dq_next']
                q_control = second_result['q_control']
                dq_control = second_result['dq_control']
                state_prediction_age_sec = float(
                    second_result['state_prediction_age_sec'])
                self._last_state_prediction_age_sec = state_prediction_age_sec
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
        g = self._apply_command_guard(
            q_next, q_control, t_solve, info, command_usable,
            t_state=base_t_state,
            path_t_solve=path_t_solve,
            path_t_state=path_t_state,
            path_t_new=path_t_new,
            base_t_new=base_t_new)
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
            if self.command_inversion_enabled:
                # 命令逆变换: 解析求发布速度 v_c, 使 plant 从 x0(=q_control,
                # Smith 预测的生效时刻状态) 经 n=dt/h 个执行律子步后恰好落在
                # q_next。plant 子步: gap_{j} = gap_{j-1}*r - h*v_c, r=1-h/tau,
                # 末端 gap=0 => v_c = Δq * r^n / (tau*(1-r^n))。
                # 相比直接发布区间末速度 dq_{k+1}, 消除半拍相位超前和位置
                # 反馈项带来的 ~13% 幅值增益。
                h = self.plant_control_substep_dt
                tau = self.plant_position_feedback_tau
                n_sub = max(1, int(round(self.dt / h)))
                r = 1.0 - h / tau
                delta_q_plan = (np.asarray(q_next, dtype=float)
                                - np.asarray(q_control, dtype=float))
                v_c = delta_q_plan * (r ** n_sub) / (tau * (1.0 - r ** n_sub))
                dq_cmd = np.clip(
                    v_c / self.plant_velocity_feedforward_gain,
                    -self.plant_max_velocity, self.plant_max_velocity)
            else:
                dq_cmd = np.clip(
                    np.asarray(dq_model_next, dtype=float)
                    / self.second_order_dq_ff_compensation,
                    -self.second_order_max_velocity,
                    self.second_order_max_velocity)
        if self.solver_model == 'second_order':
            accepted = (dq_model_next is not None and command_usable
                        and not guard_block and not g['guard_projected'])
            executed_velocity = (
                np.asarray(dq_model_next, dtype=float).copy()
                if accepted
                else np.asarray(g['dq_cmd'], dtype=float).copy()
                if command_usable and not guard_block
                else np.zeros(JOINT_COUNT)
            )
            self._so_last_cmd_dq = executed_velocity
            ddq0 = np.asarray(
                info.get('ddq0', np.zeros(JOINT_COUNT)), dtype=float)
            self._so_last_cmd_ddq = (
                ddq0.copy() if accepted
                else np.clip(
                    (executed_velocity - dq_control) / self.dt,
                    -self.second_order_max_acceleration,
                    self.second_order_max_acceleration))
            # The ACADOS plan was generated for q_next/dq_next. A projected,
            # backup, held or unusable command invalidates it; reusing that plan
            # can immediately recreate the rejected jump on the next tick.
            if not accepted:
                self._reset_second_order_plan()
        accepted_step = np.asarray(self.q_last, dtype=float) - q_control
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

        first_track_tick = self._nominal_tick_index == 0
        clock_path_t = (
            self._fault_path_time
            if self.guard_fault and math.isfinite(self._fault_path_time)
            else 0.0 if first_track_tick else path_t_solve)
        clock_path_scale = (
            0.0 if self.guard_fault
            else float(self.time_scaler.scale)
            if self.online_time_scaling_enabled else 1.0)
        if not first_track_tick:
            # Publish this solve anchor before the command.  During a scale
            # transition the visualizer can then map the command stamp using
            # the current scale instead of extrapolating from the previous
            # tick and permanently recording a small orange-FK phase error.
            self._publish_path_progress(
                solve_wall, t_solve,
                path_time=clock_path_t,
                path_scale=clock_path_scale)

        q_published, dq_published, publish_wall, dart_diag = (
            self._publish_joint_positions(self.q_last, dq_cmd))
        # 首个 track tick 发布后, 把轨迹原点重锚到该次发布时刻。
        # trajectory_visualizer_demo 以第一条 moving_base_track 命令的
        # header.stamp 作为其时间原点; 若 MPC 原点停留在首拍求解起点,
        # 两节点时钟相差首拍 solve->publish 延迟(实测 6~17ms 且逐次运行不同),
        # visualizer 的定时误差 KPI 会被这个纯时钟偏移污染(~1-2mm 假滞后)。
        # 重锚后两原点按构造相等; 首拍参考瞬时前移一次(<20ms, 处于起笔前),
        # 不改变 header.stamp=真实发布时刻 的既有约定。
        if first_track_tick:
            self.traj_origin_wall = publish_wall
            # Path-time integration uses the same re-anchored origin. Without
            # this, the second tick integrates solve->solve while base_t uses
            # publish->solve, permanently leading by first solve latency.
            self._last_path_update_wall = publish_wall
            self._publish_path_progress(
                publish_wall, 0.0,
                path_time=0.0,
                path_scale=clock_path_scale)
        actual_publish_delay_sec = max(0.0, publish_wall - solve_wall)
        if self.latency_compensation_enabled:
            alpha = self.latency_publish_ewma_alpha
            self._publish_latency_ewma_sec = (
                (1.0 - alpha) * self._publish_latency_ewma_sec
                + alpha * actual_publish_delay_sec)
        self._warn_joint_limits(q_published)
        self._update_command_fk_monitor(
            q_published, t_solve, path_t_solve)

        t_publish = publish_wall - self.traj_origin_wall
        timing = {
            't_solve': t_solve,
            't_publish': t_publish,
            't_reference': float(self._last_target_time0),
            't_next': t_publish + self.dt,
            'path_time': path_t_solve,
            'path_scale': (
                0.0 if self.guard_fault
                else float(self.time_scaler.scale)
                if self.online_time_scaling_enabled else 1.0),
            'timer_interval_ms': timer_interval_ms,
            'solve_to_publish_ms': (publish_wall - solve_wall) * 1000.0,
            'feedback_age_ms': feedback_age_ms,
            'transport_gap_ms': transport_gap_ms,
            'transport_fault': self.transport_fault,
            'late_tick_recovery': late_tick_recovery,
            'predicted_application_delay_ms': application_delay_sec * 1000.0,
            'state_prediction_age_ms': state_prediction_age_sec * 1000.0,
            'js_header_stamp_sec': js_header_stamp_sec,
            'js_recv_wall_sec': js_recv_wall_sec,
            't_solve_wall': solve_wall,
            't_publish_wall': publish_wall,
        }
        self._append_loop_row(
            timing,
            dqp,
            q_current=q_current,
            dq_measured=dq_feedback,
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
                              'expected_residual': 2,
                              'state_safe_filter': 3}[self.guard_mode],
                't_new': g['t_new'],
                'path_t_new': g['path_t_new'],
                't_last_reference': g['guard_t_last_used'],
                'path_t_last_reference': g[
                    'guard_path_t_last_used'],
                'ev': g['guard_ev'],
                'lambda': g['guard_lambda'],
                'projected': g['guard_projected'],
                'backup': g['guard_backup'],
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
            periodic_diag=periodic_diag,
        )
        self._nominal_tick_index += 1
        self._print_joint_angles(t_solve)
        self._publish_world_markers(
            t_solve,
            self._fault_path_time
            if self.guard_fault and math.isfinite(self._fault_path_time)
            else None)

    def tick(self):
        if self.phase in ('guard_fault', 'transport_fault'):
            # 阶段1E: 显式安全停止 —— 保持最后被接受的命令,零速度,不再求解/推进轨迹,
            # 不自动重锚/复位(需要人工重启才能恢复)。
            self._publish_joint_positions(self.q_last, np.zeros(JOINT_COUNT))
            now_wall = self.get_clock().now().nanoseconds * 1e-9
            base_t = (
                max(0.0, now_wall - self.traj_origin_wall)
                if self.traj_origin_wall is not None else 0.0)
            frozen_path_t = (
                self._fault_path_time
                if math.isfinite(self._fault_path_time)
                else self._path_time_at_offset(base_t, 0.0))
            self._publish_base_tf(base_t, frozen_path_t)
            self._publish_path_progress(
                now_wall, base_t,
                path_time=frozen_path_t,
                path_scale=0.0)
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
        # ACADOS 控制循环采用单线程确定性调度。/joint_states 的 QoS depth=1
        # 保证求解期间只保留最新反馈，避免旧消息积压；同时避免 Python
        # MultiThreadedExecutor 中 100Hz 反馈回调争抢 GIL、打断 50Hz NMPC。
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
