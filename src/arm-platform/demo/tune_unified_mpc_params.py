#!/usr/bin/env python3
"""固定基座臂 + 浮动基座臂的闭环离线调参（不含 LL100）。

======================= T0 语义对齐(2026-07-23) =======================
本调参器的闭环评估已与 moving-base 生产 demo(move_base_circle_mpc_ik_demo.py)
的已验收语义逐项对齐:
  - 参考时序   : targets[j] = target(t_solve + (1+j)*dt)  (reference_lead_steps=1);
  - 误差对齐   : plant 推进到 t_next=t_solve+dt 后,q_actual(t_next) 对 target(t_next);
  - dq_prev    : measured_delta —— 每 tick 传 previous_joint_delta,状态机**直接复用生产
                 函数对象** MoveBaseCircleMpcIkDemo._compute_dq_prev(经 stub 适配,零拷贝实现);
  - RTI        : rti_iteration_mode="fixed_iterations", acados_rti_iterations=3(显式);
  - 执行器闭环 : 速度前馈控制律 v_cmd=clamp(0.7*clamp(dq_ref,±1.0)+clamp((q_ref-q)/0.15,±0.4),±1.0)
                 + 5.0 rad/s² 加速度限幅,以 10ms 子步积分(与 C++ student_arm_node 100Hz
                 控制周期一致;**这是与当前控制律对齐的近似闭环模型,不是真实执行器的精确模型**);
  - 守卫       : 复现生产 6mm command-FK 守卫(world 系 FK、当前基座时刻、对上一发布命令、
                 hold 后发 q_prev/dq=0、监视点随基座每 tick 更新);guard_count>0 硬淘汰;
  - 统计窗口   : 默认 2 周期预热 + 4 周期稳态,瞬态/稳态分开统计,评分只用稳态。
注意: 固定臂生产 demo(move_arm_mpc_ik_demo.py)截至 T0 仍是旧语义(lead=0/无守卫/无 dq_prev),
本调参器对固定臂场景同样按对齐后的语义评估(为其未来升级服务),守卫仅在 moving-base 场景启用
(与各自生产 demo 一致)。
=====================================================================

两种模式（`--mode`）:
  - unified  : 搜出**一套统一参数**，同时满足所有场景（默认）。
  - separate : **每个场景单独调参**，各自得到自己的最优参数。

扩展点① 场景: build_scenarios() 里加 Scenario;移动基座参数化矩阵用
  build_moving_base_matrix()(BASE_PERIOD/振幅/初相位/EE周期/是否锁姿态)。
  矩形/三角形/样条/在线时间缩放等轨迹层扩展预留在 position_fn 接口,后续阶段实现。
扩展点② 求解器: 只改 build_solver()。
"""

import argparse
import contextlib
import csv
import itertools
import json
import math
import os
import time
from dataclasses import asdict, dataclass, field, replace
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import pinocchio as pin

import moving_base_trajectory_config as mbtraj
from analyze_moving_base_run import TrajectoryModel, _quadrature_fit
from move_base_circle_mpc_ik_demo import MoveBaseCircleMpcIkDemo
from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK
from pinocchio_ik import DEFAULT_URDF, EE_FRAME, PinocchioIK
from second_order_mpc_ik import SecondOrderMpcConfig, SecondOrderMpcIK


# ----- 控制周期 / 闭环 plant -----
PUBLISH_RATE_HZ = 50.0
CONTROL_DT = 1.0 / PUBLISH_RATE_HZ          # MPC 发布周期 (s)
CTRL_SUBSTEP_DT = 0.01                       # 底层控制器周期 10ms (C++ 100Hz)
N_SUBSTEPS = max(1, int(round(CONTROL_DT / CTRL_SUBSTEP_DT)))

# 与 student_arm.launch.py 速度前馈默认值一致 (角速度控制恒开)。
PLANT_MAX_VELOCITY = 1.0
PLANT_FF_GAIN = 0.7            # 阶段1D 已证明 1.0 失稳,生产保持 0.7
PLANT_FB_TAU = 0.15
PLANT_FB_VEL_LIMIT = 0.4
PLANT_MAX_ACCEL = 5.0
PLANT_ACCEL_LIMIT = True

# ----- 生产语义常量(与 move_base_circle_mpc_ik_demo 生产默认一致) -----
REFERENCE_LEAD_STEPS = 1
RTI_ITERATION_MODE = "fixed_iterations"
ACADOS_RTI_ITERATIONS = 3
GUARD_STEP_WARN_M = 0.006       # 生产 command_fk_step_warn_m 默认
DQ_PREV_MODE = "measured_delta"

# ----- 评估阈值 -----
TERMINAL_TOLERANCE = 0.01
SINGULAR_SIGMA_THRESHOLD = 0.05
MAX_SOLVE_TIME_MS_HARD = 20.0     # 50Hz 预算 20ms, 按 p99 判定
# 真机 20260725 A/B 表明“只看 ACADOS solve_time”会错误放行 N=20：
# solver/optimize P99=15.3ms，但完整 timer P95/P99=26.7/32.9ms。离线评估至少
# 要把参考生成 + Python 封装 + solver 的 wall time 纳入，并预留 ROS 发布/记录余量。
MAX_CONTROL_COMPUTE_TIME_MS_HARD = 12.0
SOLVE_TIME_WARMUP_STEPS = 5
VEL_LIMIT_TOL = 1.03              # 实测速度硬门: <= 1.03 * max_velocity
ACCEL_LIMIT_TOL = 1.10            # 实测加速度硬门: <= 1.10 * max_accel
DQ_PREV_FALLBACK_MAX_RATIO = 0.001
JOINT_LIMIT_ACTIVE_MARGIN_RAD = 1e-3   # "接近限位"判定余量(报告/软评分,非硬门)

# ----- 饱和预算相对门(vs 生产参考参数在同场景的参考值) -----
# 依据: 阶段1D D0 已证明当前工况本身存在明显加速度限幅活动(accel near-limit ~0.8-0.9),
# 故不能要求 sat_ratio==0;门取"不超过生产参考 + 容差"。容差为绝对比例点。
# closeout 修订: 总体 ratio 定义为"该子步至少一个关节饱和",生产工况下 accel 总体≈0.99,
# "参考+0.05">1 使门形同虚设 —— 相对硬门改用 **max 每关节饱和率**(单关节子步占比),
# 总体 ratio 仅进报告/软评分。
VEL_SAT_RATIO_TOL = 0.05
ACCEL_SAT_RATIO_TOL = 0.05

# ----- 阶段1D 轨迹配置快照(D0 门限的有效工况;逐字段一致才允许附 D0 门限) -----
# 来源: data/ab_runs_1d/metrics_p1d_r{1,4,5}_d0.json 的 config_fingerprint.trajectory_config
# (三轮一致,创建 testdata fixture 时已核对)。任何字段改动都意味着"不再是阶段1D工况",
# 旧 32/38mm 门限不能静默套用。
PHASE1D_TRAJ_CONFIG_SNAPSHOT = {
    "BASE_CIRCLE_CENTER": [0.0, 0.0, 0.0],
    "BASE_CIRCLE_RADIUS": 0.02,
    "BASE_PERIOD_SEC": 1.0,
    "BASE_HEIGHT": 0.01,
    "BASE_Z_AMPLITUDE": 0.0,
    "BASE_MOTION_SHAPE": "circle_z",
    "BASE_ATTITUDE_AMPLITUDE_DEG": 0.1,
    "BASE_ROLL_AMPLITUDE": 0.0017453292519943296,
    "BASE_PITCH_AMPLITUDE": 0.0017453292519943296,
    "BASE_YAW_AMPLITUDE": 0.0017453292519943296,
    "EE_WORLD_CIRCLE_CENTER": [0.3, 0.01, 0.1],
    "EE_CIRCLE_RADIUS": 0.08,
    "EE_PERIOD_SEC": 4.0,
    "TARGET_YAW_RAD": 1.5707963267948966,
    "TARGET_PITCH_RAD": 1.5707963267948966,
    "TARGET_ROLL_RAD": 0.0,
}


def _traj_config_matches_phase1d(cfg: dict) -> Tuple[bool, List[str]]:
    """逐字段(含 TARGET_*)比较当前场景轨迹配置与阶段1D快照。返回 (匹配?, 不一致字段)。"""
    diffs = []
    for key, snap_val in PHASE1D_TRAJ_CONFIG_SNAPSHOT.items():
        cur = cfg.get(key)
        if isinstance(snap_val, list):
            same = (cur is not None and len(cur) == len(snap_val)
                    and all(math.isclose(a, b, rel_tol=0.0, abs_tol=1e-12)
                            for a, b in zip(cur, snap_val)))
        elif isinstance(snap_val, str):
            same = cur is not None and str(cur) == snap_val
        else:
            same = cur is not None and math.isclose(cur, snap_val, rel_tol=0.0, abs_tol=1e-12)
        if not same:
            diffs.append(f"{key}: 当前={cur} 快照={snap_val}")
    return (not diffs), diffs

# ----- moving-base 生产场景误差硬门(来源: 阶段1D D0 真实基线 × 安全余量) -----
# 真实 D0 稳态基线(data/ab_runs_1d/metrics_p1d_r{1,4,5}_d0.json, 3 轮, 实测核对):
#   actual P95        = 20.07 / 20.35 / 20.80 mm -> 最差 20.80
#   actual max        = 22.17 / 25.14 / 25.10 mm -> 最差 25.14 (steady.max_mm)
#   cmd@next_tick P95 = 19.63 / 20.36 / 20.98 mm -> 最差 20.98
# 门限 = 最差真实基线 × LIMIT_SAFETY_MARGIN(1.5), 向上取整到 mm:
LIMIT_SAFETY_MARGIN = 1.5
MB_POS_P95_LIMIT_M = 0.032     # 20.80mm*1.5=31.2 -> 32mm
MB_POS_MAX_LIMIT_M = 0.038     # 25.14mm*1.5=37.7 -> 38mm
MB_CMD_P95_LIMIT_M = 0.032     # 20.98mm*1.5=31.5 -> 32mm
# 姿态: 1D 真实数据未记录稳态姿态误差;取 T0 离线生产基线 steady_ori_err_max≈0.20 rad × 2。
MB_ORI_LIMIT_RAD = 0.40
MB_LIMIT_SOURCE = ("phase1D D0 real steady baseline (metrics_p1d_r{1,4,5}_d0.json) "
                   f"x margin {LIMIT_SAFETY_MARGIN}; ori: offline T0 baseline(0.20rad) x2")

# ----- 搜索网格 (max_joint_step 受 max_velocity*dt=0.02 约束) -----
COARSE_HORIZON_STEPS = [4, 8, 12]
COARSE_MAX_JOINT_STEP = [0.012, 0.016, 0.020]
COARSE_ORIENTATION_WEIGHT = [0.5, 1.0, 2.0]
COARSE_TERMINAL_ORIENTATION_WEIGHT = [5.0, 10.0, 20.0]

FINE_HORIZON_DELTA = [-2, 0, 2]
FINE_MAX_STEP_DELTA = [-0.004, 0.0, 0.004]
FINE_WEIGHT_SCALE = [0.75, 1.0, 1.25]

MAX_JOINT_STEP_LIMIT = PLANT_MAX_VELOCITY * CONTROL_DT  # 0.02

BUILD_ROOT = Path("/tmp/windylab_acados_unified_tune")


# ======================================================================
# 评估上下文
# ======================================================================
@dataclass
class Context:
    model: object
    data: object
    frame_id: int
    q_lower: np.ndarray
    q_upper: np.ndarray
    warmup_periods: float
    steady_periods: float


def make_context(warmup_periods: float, steady_periods: float) -> Context:
    model = pin.buildModelFromUrdf(DEFAULT_URDF)
    data = model.createData()
    frame_id = model.getFrameId(EE_FRAME)
    return Context(model, data, frame_id,
                   model.lowerPositionLimit.copy(), model.upperPositionLimit.copy(),
                   warmup_periods, steady_periods)


def fk(ctx: Context, q: np.ndarray):
    pin.framesForwardKinematics(ctx.model, ctx.data, q)
    oMf = ctx.data.oMf[ctx.frame_id]
    return oMf.translation.copy(), oMf.rotation.copy()


def orientation_error(current_rotation: np.ndarray, target_rotation: np.ndarray) -> np.ndarray:
    skew = target_rotation @ current_rotation.T - current_rotation @ target_rotation.T
    return 0.5 * np.array([skew[2, 1], skew[0, 2], skew[1, 0]], dtype=float)


def min_singular_value(ctx: Context, q: np.ndarray) -> float:
    J = pin.computeFrameJacobian(ctx.model, ctx.data, q, ctx.frame_id,
                                 pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    sv = np.linalg.svd(J[:3, :], compute_uv=False)
    return float(np.min(sv))


# ======================================================================
# 闭环 plant: 速度前馈控制器 (smooth_position_controller.cc 前馈分支的对齐近似)
# ======================================================================
class FeedforwardPlant:
    """逐关节、限速 + 位置反馈 + 加速度限制的位置跟踪器。

    记录每个控制 tick 的 v_ff/v_fb/v_cmd(取子步末值)与速度/加速度饱和子步计数,
    供执行匹配与预算分析。gain 可参数化(默认生产 0.7;仅用于离线可信度对照,
    不改生产 launch 默认)。
    """

    def __init__(self, q0: np.ndarray, q_lower: np.ndarray, q_upper: np.ndarray,
                 gain: float = PLANT_FF_GAIN):
        self.q = q0.astype(float).copy()
        self.v_last = np.zeros_like(self.q)
        self.q_lower = q_lower
        self.q_upper = q_upper
        self.gain = float(gain)
        # 最近一次 advance 的诊断
        self.last_v_ff = np.zeros_like(self.q)
        self.last_v_fb = np.zeros_like(self.q)
        self.last_v_cmd = np.zeros_like(self.q)
        self.vel_sat_substeps = 0
        self.accel_sat_substeps = 0
        self.total_substeps = 0
        # closeout: 每关节速度/加速度饱和子步计数
        self.vel_sat_per_joint = np.zeros(len(self.q), dtype=int)
        self.accel_sat_per_joint = np.zeros(len(self.q), dtype=int)
        # closeout: 关节位置撞限诊断(clip=真实撞限;active=接近限位 margin 内)
        self.joint_clip_substeps = 0
        self.joint_clip_per_joint = np.zeros(len(self.q), dtype=int)
        self.joint_active_substeps = 0
        self.joint_active_per_joint = np.zeros(len(self.q), dtype=int)
        self.last_tick_clipped = False       # 本 tick(advance 调用)是否发生位置 clip

    def advance(self, q_cmd: np.ndarray, dq_ff: np.ndarray) -> np.ndarray:
        self.last_tick_clipped = False
        for _ in range(N_SUBSTEPS):
            v_ff = np.clip(dq_ff, -PLANT_MAX_VELOCITY, PLANT_MAX_VELOCITY)
            v_fb = np.clip((q_cmd - self.q) / PLANT_FB_TAU,
                           -PLANT_FB_VEL_LIMIT, PLANT_FB_VEL_LIMIT)
            raw_sum = self.gain * v_ff + v_fb
            v_raw = np.clip(raw_sum, -PLANT_MAX_VELOCITY, PLANT_MAX_VELOCITY)
            vel_sat = np.abs(raw_sum) >= PLANT_MAX_VELOCITY - 1e-12
            if np.any(vel_sat):
                self.vel_sat_substeps += 1
                self.vel_sat_per_joint += vel_sat.astype(int)
            if PLANT_ACCEL_LIMIT:
                step = PLANT_MAX_ACCEL * CTRL_SUBSTEP_DT
                v_cmd = np.clip(v_raw, self.v_last - step, self.v_last + step)
                acc_sat = np.abs(v_raw - self.v_last) > step + 1e-12
                if np.any(acc_sat):
                    self.accel_sat_substeps += 1
                    self.accel_sat_per_joint += acc_sat.astype(int)
            else:
                v_cmd = v_raw
            q_unclipped = self.q + v_cmd * CTRL_SUBSTEP_DT
            q_new = np.clip(q_unclipped, self.q_lower, self.q_upper)
            clipped = np.abs(q_new - q_unclipped) > 1e-12
            if np.any(clipped):
                self.joint_clip_substeps += 1
                self.joint_clip_per_joint += clipped.astype(int)
                self.last_tick_clipped = True
            active = ((q_new <= self.q_lower + JOINT_LIMIT_ACTIVE_MARGIN_RAD)
                      | (q_new >= self.q_upper - JOINT_LIMIT_ACTIVE_MARGIN_RAD))
            if np.any(active):
                self.joint_active_substeps += 1
                self.joint_active_per_joint += active.astype(int)
            v_cmd = (q_new - self.q) / CTRL_SUBSTEP_DT
            self.q = q_new
            self.v_last = v_cmd
            self.total_substeps += 1
            self.last_v_ff, self.last_v_fb, self.last_v_cmd = v_ff, v_fb, v_cmd
        return self.q.copy()


# ======================================================================
# dq_prev: 直接复用生产函数对象(stub 适配),不复制第二套实现
# ======================================================================
class DqPrevAdapter:
    """经 stub 调用 MoveBaseCircleMpcIkDemo._compute_dq_prev(生产函数对象本体)。

    离线仿真中 js 时间戳 = t_solve(理想反馈);锚点按生产 update_anchor 决策条件更新。
    """

    def __init__(self, max_joint_step: float, mode: str = DQ_PREV_MODE):
        stub = object.__new__(MoveBaseCircleMpcIkDemo)
        stub.dq_prev_mode = mode
        stub.dt = CONTROL_DT
        stub.max_joint_step = max_joint_step
        stub.dq_measured = None
        stub._prev_solve_q_measured = None
        stub._prev_solve_js_stamp = float("nan")
        self.stub = stub
        self.fallback_count = 0
        self.tick_count = 0
        # closeout: 显式记录 (tick, fallback_code),用于结构性首帧回退的精确判定。
        self.fallback_events: List[Tuple[int, int]] = []

    def compute(self, q_now: np.ndarray, js_stamp: float) -> np.ndarray:
        tick = self.tick_count
        r = MoveBaseCircleMpcIkDemo._compute_dq_prev(self.stub, q_now, js_stamp)
        self.tick_count += 1
        if r["fallback"] != 0:
            self.fallback_count += 1
            self.fallback_events.append((tick, int(r["fallback"])))
        if r["update_anchor"]:
            self.stub._prev_solve_q_measured = r["anchor_q"]
            self.stub._prev_solve_js_stamp = float(r["anchor_stamp"])
        return r["input"]

    def nonstructural_fallback_count(self) -> int:
        """非结构性 fallback 数:只有"第0个tick 且 code==1(尚无历史锚点)"才算结构性
        并排除;NaN(4)/非递增(2)/过旧(3)以及任何非首帧的 code1 全部计入。"""
        structural = 1 if (self.fallback_events
                           and self.fallback_events[0] == (0, 1)) else 0
        return len(self.fallback_events) - structural


# ======================================================================
# 场景定义
# ======================================================================
@dataclass
class Scenario:
    name: str
    demo: str
    period_sec: float
    lock_orientation: bool
    position_fn: Callable[[float], np.ndarray]      # base 系目标位置
    rotation_fn: Optional[Callable[[float], np.ndarray]]
    demo_extra_args: List[str] = field(default_factory=list)
    q_seed: np.ndarray = field(default=None)
    # T0: 守卫与 world 变换(与各自生产 demo 一致;固定臂 demo 无守卫)
    guard_enabled: bool = False
    base_pose_fn: Optional[Callable[[float], np.ndarray]] = None      # world 系基座位置
    base_rot_fn: Optional[Callable[[float], np.ndarray]] = None       # world 系基座姿态
    base_period_sec: Optional[float] = None                            # 基座主频(残差分析)
    contour_active_fn: Optional[Callable[[float], bool]] = None        # None=整段落笔
    # closeout: 评估周期 = 联合重复周期(moving-base=lcm(EE,BASE);固定臂=自身周期)。
    evaluation_period_sec: Optional[float] = None   # None -> period_sec
    ee_period_sec: Optional[float] = None
    # closeout: 独立相位(基座晃动 vs 末端画图的相对相位覆盖)。
    base_phase_offset_sec: float = 0.0
    ee_phase_offset_sec: float = 0.0
    # closeout: 绝对误差硬门(正式搜索必配,缺失即快速失败;credibility 只报告)。
    pos_p95_limit_m: Optional[float] = None
    pos_max_limit_m: Optional[float] = None
    cmd_p95_limit_m: Optional[float] = None
    orientation_limit_rad: Optional[float] = None   # 仅锁姿态场景需要
    limit_source: str = ""
    # closeout 修订: 场景轨迹配置指纹(moving-base 场景由工厂填;credibility JSON 导出用)。
    traj_config: Optional[dict] = None

    def eval_period(self) -> float:
        return self.evaluation_period_sec if self.evaluation_period_sec else self.period_sec

    def has_required_limits(self) -> bool:
        base_ok = (self.pos_p95_limit_m is not None and self.pos_max_limit_m is not None
                   and self.cmd_p95_limit_m is not None)
        if self.lock_orientation:
            return base_ok and self.orientation_limit_rad is not None
        return base_ok


# 固定基座轨迹 (取自 move_arm_mpc_ik_demo.py 常量)
FIXED_CIRCLE_CENTER = np.array([0.35, 0.0, 0.15])
FIXED_CIRCLE_RADIUS = 0.08
FIXED_CIRCLE_PERIOD = 8.0
FIXED_HCIRCLE_PERIOD = 8.0
LINE_A = np.array([0.35, -0.15, 0.15])
LINE_B = np.array([0.35, 0.15, 0.25])
LINE_PERIOD = 6.0


def _vertical_circle(t: float) -> np.ndarray:
    w = 2.0 * math.pi / FIXED_CIRCLE_PERIOD
    return FIXED_CIRCLE_CENTER + FIXED_CIRCLE_RADIUS * np.array(
        [0.0, math.cos(w * t), math.sin(w * t)])


def _horizontal_circle(t: float) -> np.ndarray:
    w = 2.0 * math.pi / FIXED_HCIRCLE_PERIOD
    return FIXED_CIRCLE_CENTER + FIXED_CIRCLE_RADIUS * np.array(
        [math.cos(w * t), math.sin(w * t), 0.0])


def _line(t: float) -> np.ndarray:
    s = 0.5 * (1.0 - math.cos(2.0 * math.pi * t / LINE_PERIOD))
    return LINE_A + s * (LINE_B - LINE_A)


def make_moving_base_scenario(
    name: str = "moving_base_circle",
    base_period_sec: float = None,
    base_radius_m: float = None,
    base_z_amplitude_m: float = None,
    ee_period_sec: float = None,
    base_phase_offset_sec: float = 0.0,
    ee_phase_offset_sec: float = 0.0,
    with_orientation: bool = True,
    phase_offset_sec: float = None,   # 已弃用: 联合时间平移别名(=同时设两个 offset)
    attach_production_limits: bool = None,
) -> Scenario:
    """参数化移动基座场景工厂(场景矩阵接口)。

    数值默认取自 moving_base_trajectory_config(生产配置);轨迹重建复用
    analyze_moving_base_run.TrajectoryModel。
    独立相位: base_phase_offset_sec 只平移基座(pose/rotation),ee_phase_offset_sec
    只平移末端 world 圆;两者同时等量设置才等价于旧的联合时间平移。
    phase_offset_sec 为**弃用别名**(与新参数同时给出会报错)。
    评估周期 = lcm(EE_PERIOD, BASE_PERIOD)(联合重复周期)。
    attach_production_limits: None=仅生产工况(默认参数)自动附上 1D 基线门限;
                              True/False 显式控制。
    """
    if phase_offset_sec is not None:
        if base_phase_offset_sec != 0.0 or ee_phase_offset_sec != 0.0:
            raise ValueError("phase_offset_sec(弃用) 不能与 base/ee_phase_offset_sec 同时使用")
        base_phase_offset_sec = ee_phase_offset_sec = float(phase_offset_sec)
    cfg = {
        "BASE_CIRCLE_CENTER": mbtraj.BASE_CIRCLE_CENTER.tolist(),
        "BASE_CIRCLE_RADIUS": base_radius_m if base_radius_m is not None else mbtraj.BASE_CIRCLE_RADIUS,
        "BASE_PERIOD_SEC": base_period_sec if base_period_sec is not None else mbtraj.BASE_PERIOD_SEC,
        "BASE_HEIGHT": mbtraj.BASE_HEIGHT,
        "BASE_Z_AMPLITUDE": (float(base_z_amplitude_m) if base_z_amplitude_m is not None
                             else float(getattr(mbtraj, "BASE_Z_AMPLITUDE", 0.0))),
        "BASE_Z_PHASE": float(getattr(mbtraj, "BASE_Z_PHASE", 0.0)),
        "BASE_MOTION_SHAPE": str(getattr(mbtraj, "BASE_MOTION_SHAPE", "circle_z")),
        "BASE_SPHERE_RADIUS": float(getattr(mbtraj, "BASE_SPHERE_RADIUS", 0.0)),
        "BASE_ATTITUDE_AMPLITUDE_DEG": mbtraj.BASE_ATTITUDE_AMPLITUDE_DEG,
        "BASE_ROLL_AMPLITUDE": mbtraj.BASE_ROLL_AMPLITUDE,
        "BASE_PITCH_AMPLITUDE": mbtraj.BASE_PITCH_AMPLITUDE,
        "BASE_YAW_AMPLITUDE": mbtraj.BASE_YAW_AMPLITUDE,
        "EE_WORLD_CIRCLE_CENTER": mbtraj.EE_WORLD_CIRCLE_CENTER.tolist(),
        "EE_CIRCLE_RADIUS": mbtraj.EE_CIRCLE_RADIUS,
        "EE_PERIOD_SEC": ee_period_sec if ee_period_sec is not None else mbtraj.EE_PERIOD_SEC,
        # 姿态目标常量也属于工况指纹(D0 门限有效性检查用;TrajectoryModel 不读取)。
        # 键名与 collect_provenance 的 config_fingerprint.trajectory_config 对齐。
        "TARGET_YAW_RAD": float(mbtraj.TARGET_YAW),
        "TARGET_PITCH_RAD": float(mbtraj.TARGET_PITCH),
        "TARGET_ROLL_RAD": float(mbtraj.TARGET_ROLL),
    }
    tm = TrajectoryModel(cfg)
    bo = float(base_phase_offset_sec)
    eo = float(ee_phase_offset_sec)

    # 独立相位数学定义: 基座量用 t+bo,末端 world 目标用 t+eo;
    # base 系目标 = R_b(t+bo)^T (p_ee_w(t+eo) − p_b(t+bo));姿态目标用 t+bo。
    def position_fn(t: float) -> np.ndarray:
        tb, te = t + bo, t + eo
        return tm.base_rotation(tb).T @ (tm.ee_target_world(te) - tm.base_pose(tb))

    def rotation_fn(t: float) -> np.ndarray:
        return tm.base_rotation(t + bo).T @ mbtraj.target_rotation_world()

    is_production = (base_period_sec is None and base_radius_m is None
                     and base_z_amplitude_m is None
                     and ee_period_sec is None and bo == 0.0 and eo == 0.0
                     and with_orientation)
    # closeout 修订: D0 门限只对阶段1D轨迹工况有效 —— 当前 cfg 必须与快照逐字段一致
    # 才能附 D0 门限;显式 attach=True 但工况不一致 => 报错;auto 模式不一致 => 不附
    # 并打印警告(防止改了 mbtraj 常量后旧门限被静默套用)。
    snapshot_ok, snapshot_diffs = _traj_config_matches_phase1d(cfg)
    if not with_orientation:
        # D0 门限源自锁姿态工况;pos-only 是不同优化问题(ow/tow=0),门限语义失效。
        snapshot_ok = False
        snapshot_diffs = snapshot_diffs + ["with_orientation: 当前=False 快照工况=True(锁姿态)"]
    if attach_production_limits is True and not snapshot_ok:
        raise ValueError(
            "attach_production_limits=True 但轨迹配置与阶段1D快照不一致,D0 门限不适用: "
            + "; ".join(snapshot_diffs))
    if attach_production_limits is None:
        attach = is_production and snapshot_ok
        if is_production and not snapshot_ok:
            print(f"[limits] 警告: 场景 {name} 调用形式为生产默认,但轨迹配置与阶段1D快照"
                  f"不一致({'; '.join(snapshot_diffs)}),不附 D0 门限。")
    else:
        attach = bool(attach_production_limits)
    return Scenario(
        name=name, demo="move_base_circle_mpc_ik_demo.py",
        period_sec=tm.ee_period, lock_orientation=with_orientation,
        position_fn=position_fn,
        rotation_fn=rotation_fn if with_orientation else None,
        demo_extra_args=[],
        guard_enabled=True,
        base_pose_fn=lambda t: tm.base_pose(t + bo),
        base_rot_fn=lambda t: tm.base_rotation(t + bo),
        base_period_sec=tm.base_period,
        evaluation_period_sec=tm.repetition_period(),   # 联合重复周期
        ee_period_sec=tm.ee_period,
        base_phase_offset_sec=bo,
        ee_phase_offset_sec=eo,
        pos_p95_limit_m=MB_POS_P95_LIMIT_M if attach else None,
        pos_max_limit_m=MB_POS_MAX_LIMIT_M if attach else None,
        cmd_p95_limit_m=MB_CMD_P95_LIMIT_M if attach else None,
        orientation_limit_rad=(MB_ORI_LIMIT_RAD if (attach and with_orientation) else None),
        limit_source=MB_LIMIT_SOURCE if attach else "",
        traj_config=dict(cfg),
    )


def build_moving_base_matrix(
    base_periods: Sequence[float] = (8.0, 4.0, 2.0, 1.0, 0.75, 0.5),
    base_radii: Sequence[float] = (0.01, 0.02, 0.04),
    base_phases: Sequence[float] = (0.0,),
    ee_phases: Sequence[float] = (0.0,),
    ee_periods: Sequence[float] = (4.0,),
    orientation_modes: Sequence[bool] = (True, False),
) -> List[Scenario]:
    """场景矩阵接口(closeout: base/EE 独立相位)。正式搜索前须为各场景配置误差门限。"""
    out = []
    for bp, br, bph, eph, ep, om in itertools.product(
            base_periods, base_radii, base_phases, ee_phases, ee_periods, orientation_modes):
        tag = (f"mb_T{bp:g}_A{br:g}_bph{bph:g}_eph{eph:g}_ee{ep:g}_"
               f"{'ori' if om else 'pos'}")
        out.append(make_moving_base_scenario(
            tag, bp, br, ep, base_phase_offset_sec=bph, ee_phase_offset_sec=eph,
            with_orientation=om))
    return out


def build_scenarios(ctx: Context) -> List[Scenario]:
    """默认场景集(与旧版同名),moving_base 改用参数化工厂(数值=生产配置)。"""
    SCENARIO = [
        Scenario("circle", "move_arm_mpc_ik_demo.py", FIXED_CIRCLE_PERIOD,
                 False, _vertical_circle, None, ["trajectory_mode:=circle"]),
        Scenario("horizontal_circle", "move_arm_horizontal_circle_mpc_ik_demo.py",
                 FIXED_HCIRCLE_PERIOD, True, _horizontal_circle, None, ["lock_orientation:=true"]),
        Scenario("line", "move_arm_mpc_ik_demo.py", LINE_PERIOD,
                 False, _line, None, ["trajectory_mode:=line"]),
        make_moving_base_scenario(),
    ]
    seed_ik = PinocchioIK()
    for sc in SCENARIO:
        p0 = sc.position_fn(0.0)
        if sc.lock_orientation and sc.rotation_fn is not None:
            q0, ok = seed_ik.solve(p0, target_rotation=sc.rotation_fn(0.0), rot_weight=0.2)
        else:
            q0, ok = seed_ik.solve(p0)
        if not ok:
            raise RuntimeError(f"场景 {sc.name} 起点 {p0.tolist()} IK 不可达")
        sc.q_seed = np.clip(q0, ctx.q_lower, ctx.q_upper)
        if sc.lock_orientation and sc.rotation_fn is None:
            _, locked = fk(ctx, sc.q_seed)
            sc.rotation_fn = (lambda _t, R=locked: R)
    return SCENARIO


# ======================================================================
# 参数集 / 结果
# ======================================================================
@dataclass(frozen=True, order=True)
class ParamSet:
    horizon_steps: int
    max_joint_step: float
    orientation_weight: float
    terminal_orientation_weight: float
    position_weight: float = 50.0
    terminal_position_weight: float = 500.0
    joint_delta_weight: float = 0.08
    joint_delta_change_weight: float = 0.25
    current_posture_weight: float = 0.01
    center_posture_weight: float = 0.002


@dataclass(frozen=True, order=True)
class SecondOrderWeightSet:
    """二阶 NMPC 的纯权重候选，不混入时域、约束或求解器结构参数。"""

    position_weight: float
    terminal_position_weight: float
    joint_velocity_weight: float
    joint_acceleration_weight: float
    current_posture_weight: float
    center_posture_weight: float

    @classmethod
    def from_config(cls, config: SecondOrderMpcConfig) -> "SecondOrderWeightSet":
        return cls(
            position_weight=float(config.position_weight),
            terminal_position_weight=float(config.terminal_position_weight),
            joint_velocity_weight=float(config.joint_velocity_weight),
            joint_acceleration_weight=float(config.joint_acceleration_weight),
            current_posture_weight=float(config.current_posture_weight),
            center_posture_weight=float(config.center_posture_weight),
        )

    def apply(self, config: SecondOrderMpcConfig) -> SecondOrderMpcConfig:
        """复制 config 并替换权重；不修改调用方持有的基线配置。"""
        lag_ratio = (
            config.lag_position_weight / config.contour_position_weight
            if config.contour_position_weight > 0.0 else 0.0)
        terminal_lag_ratio = (
            config.terminal_lag_position_weight
            / config.terminal_contour_position_weight
            if config.terminal_contour_position_weight > 0.0 else 0.0)
        return replace(
            config,
            position_weight=self.position_weight,
            terminal_position_weight=self.terminal_position_weight,
            contour_position_weight=self.position_weight,
            transition_position_weight=self.position_weight,
            lag_position_weight=self.position_weight * lag_ratio,
            terminal_contour_position_weight=self.terminal_position_weight,
            terminal_transition_position_weight=self.terminal_position_weight,
            terminal_lag_position_weight=(
                self.terminal_position_weight * terminal_lag_ratio),
            joint_velocity_weight=self.joint_velocity_weight,
            joint_acceleration_weight=self.joint_acceleration_weight,
            current_posture_weight=self.current_posture_weight,
            center_posture_weight=self.center_posture_weight,
        )


@dataclass
class ScenarioMetrics:
    name: str
    # ---- 稳态窗口(评分/硬门用) ----
    steady_pos_err_mean: float
    steady_pos_err_rms: float
    steady_pos_err_p95: float
    steady_pos_err_max: float
    steady_cmd_err_rms: float
    steady_cmd_err_p95: float
    steady_ori_err_rms: float
    steady_ori_err_max: float
    steady_base_freq_resid_rms: float      # 主基座频率残差(无基座场景=0)
    steady_contour_err_rms: float          # 落笔路径法向误差；未启用轮廓评估时为 0
    steady_contour_err_p95: float
    steady_lag_err_rms: float              # 落笔路径切向误差
    steady_exec_gap_joint_rms: float
    steady_exec_gap_cart_rms: float
    steady_ctrl_delta_rms: float           # ||Δq_pub|| 平滑度
    # ---- 瞬态(报告用) ----
    transient_pos_err_rms: float
    transient_pos_err_max: float
    # ---- 硬门计数 ----
    guard_count: int
    fallback_count: int
    solver_failure_count: int
    tracking_bad_count: int
    steady_tracking_bad_count: int
    singular_count: int
    vel_violation_count: int
    accel_violation_count: int
    dq_prev_fallback_ratio: float
    min_sigma: float
    # ---- 预算 ----
    vel_peak: float
    accel_peak: float
    vel_sat_ratio: float                   # 总体口径: 该子步至少一关节饱和(报告/软评分)
    accel_sat_ratio: float
    vel_sat_per_joint: dict                # 每关节饱和子步比例(dict,进 credibility JSON)
    accel_sat_per_joint: dict
    # closeout 修订: 每关节口径标量(相对硬门用 max —— 总体口径在生产工况 accel≈0.99,
    # "参考+0.05">1 会让门形同虚设)。
    vel_sat_mean_joint_ratio: float
    vel_sat_max_joint_ratio: float
    accel_sat_mean_joint_ratio: float
    accel_sat_max_joint_ratio: float
    accel_sat_worst_joint: str
    # ---- 关节位置撞限诊断 ----
    joint_limit_clip_count: int            # 发生位置 clip 的子步数(>0 硬淘汰)
    joint_limit_clip_ratio: float
    joint_limit_active_count: int          # 接近限位(margin 内)子步数(报告+软评分)
    joint_limit_active_ratio: float
    joint_limit_clip_per_joint: dict
    # ---- 周期/窗口记录 ----
    ee_period_sec: float
    base_period_sec: float
    evaluation_period_sec: float
    warmup_sec: float
    steady_sec: float
    hold_then_jump_max: float              # 守卫 hold 后首个放行命令的关节跳变
    mean_solve_time_ms: float
    steady_mean_solve_time_ms: float       # 评分用(稳态窗口)
    p95_solve_time_ms: float
    p99_solve_time_ms: float
    max_solve_time_ms: float
    mean_control_compute_time_ms: float
    p95_control_compute_time_ms: float
    p99_control_compute_time_ms: float
    max_control_compute_time_ms: float
    score: float


@dataclass
class TrialResult:
    scenario_set: str
    phase: str
    params: ParamSet
    aggregate_score: float
    feasible: bool
    reject_reason: str
    scenarios: List[ScenarioMetrics]


# ======================================================================
# 求解器工厂
# ======================================================================
def build_solver(params: ParamSet, lock_orientation: bool):
    return MpcPinocchioIK(config=_build_config(params, lock_orientation))


def _build_config(params: ParamSet, lock_orientation: bool) -> MpcIkConfig:
    # 显式写全生产语义:fixed RTI + 生产权重默认(与 moving-base demo 一致)。
    return MpcIkConfig(
        horizon_steps=params.horizon_steps,
        control_dt=CONTROL_DT,
        max_joint_step=params.max_joint_step,
        success_tolerance=TERMINAL_TOLERANCE,
        solver_backend="acados",
        position_weight=params.position_weight,
        terminal_position_weight=params.terminal_position_weight,
        orientation_weight=params.orientation_weight if lock_orientation else 0.0,
        terminal_orientation_weight=params.terminal_orientation_weight if lock_orientation else 0.0,
        joint_delta_weight=params.joint_delta_weight,
        joint_delta_change_weight=params.joint_delta_change_weight,
        current_posture_weight=params.current_posture_weight,
        center_posture_weight=params.center_posture_weight,
        acados_rti_iterations=ACADOS_RTI_ITERATIONS,
        rti_iteration_mode=RTI_ITERATION_MODE,
        acados_build_dir=str(_build_dir(params)),
    )


def _build_dir(params: ParamSet) -> Path:
    step_mm = int(round(params.max_joint_step * 1000.0))
    return BUILD_ROOT / f"hz{int(round(PUBLISH_RATE_HZ))}_h{params.horizon_steps}_s{step_mm}"


# ======================================================================
# 评估(T0 重写核心)
# ======================================================================
def rms(values: np.ndarray) -> float:
    if values.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(values * values)))


def _p(values: np.ndarray, q: float) -> float:
    return float(np.percentile(values, q)) if values.size else 0.0


def target_times(t_solve: float, horizon: int, dt: Optional[float] = None,
                 lead: int = REFERENCE_LEAD_STEPS) -> np.ndarray:
    """参考时序(生产定义): targets[j] = target(t_solve + (lead+j)*dt)。"""
    if dt is None:
        dt = CONTROL_DT
    return t_solve + dt * (lead + np.arange(horizon))


def evaluate_scenario(scenario: Scenario, solver, ctx: Context, horizon: int,
                      plant_gain: float = PLANT_FF_GAIN,
                      trace: Optional[list] = None,
                      dq_ff_compensation: float = 1.0,
                      guard_rule: str = 'legacy',
                      so_velocity_source: str = 'measured',
                      activate_contour: bool = False) -> ScenarioMetrics:
    q_seed = scenario.q_seed.copy()
    plant = FeedforwardPlant(q_seed, ctx.q_lower, ctx.q_upper, gain=plant_gain)
    lock = scenario.lock_orientation and scenario.rotation_fn is not None
    second_order = isinstance(solver, SecondOrderMpcIK)
    max_joint_step = (
        solver.config.max_joint_velocity * CONTROL_DT
        if second_order else solver.config.max_joint_step)
    velocity_limit = (
        solver.config.max_joint_velocity
        if second_order else max_joint_step / CONTROL_DT)
    dq_prev_adapter = DqPrevAdapter(max_joint_step)

    # world FK(守卫/基座残差): 固定臂场景基座= identity
    def fk_world(q: np.ndarray, t: float) -> np.ndarray:
        pos, _ = fk(ctx, q)
        if scenario.base_pose_fn is None:
            return pos
        return scenario.base_pose_fn(t) + scenario.base_rot_fn(t) @ pos

    def target_world(t: float) -> np.ndarray:
        target = scenario.position_fn(t)
        if scenario.base_pose_fn is None:
            return target
        return scenario.base_pose_fn(t) + scenario.base_rot_fn(t) @ target

    def contour_tangent_base(t: float) -> np.ndarray:
        """Match the production contour definition: world drawing tangent in base axes."""
        derivative_h = min(0.005, 0.5 * CONTROL_DT)
        tangent_world = (
            target_world(t + derivative_h) - target_world(t - derivative_h)
        ) / (2.0 * derivative_h)
        norm = float(np.linalg.norm(tangent_world))
        if norm < 1e-8:
            return np.zeros(3)
        tangent_world /= norm
        if scenario.base_rot_fn is None:
            return tangent_world
        return scenario.base_rot_fn(t).T @ tangent_world

    # 起始条件对齐生产 __init__: 先用 MPC solve 精化起点(同时使 solver._last_plan
    # 被本场景的起始解覆盖 —— 既复刻生产 track 首拍的内部 warm 状态,也消除跨场景污染)。
    solver._last_plan = None
    if second_order:
        solver._last_velocity_plan = None
        solver._last_acceleration_plan = None
    else:
        try:
            p0 = scenario.position_fn(0.0)
            r0 = scenario.rotation_fn(0.0) if lock else None
            q_pose, pose_ok, pose_info = solver.solve(
                p0, q_init=q_seed, target_rotation=r0)
            if bool(pose_info.get("command_usable", pose_ok)):
                q_seed = np.clip(q_pose, ctx.q_lower, ctx.q_upper)
                plant.q = q_seed.copy()
        except Exception:
            pass                              # 精化失败:保持原 IK 种子
    q_nominal = q_seed.copy()
    warm_start = None
    q_last = q_seed.copy()                   # 已发布命令(守卫 hold 保持它)
    last_command_fk_world = None             # 守卫监视点(每 tick 用当前 t 更新)
    prev_q_pub = q_seed.copy()
    prev_v_actual = np.zeros_like(q_seed)
    in_hold = False
    er_anchor = None                     # expected_residual 离线守卫锚点 (p_cmd_w, p_ref_w)
    so_last_cmd_dq = None                # commanded 速度源: 上一拍接受的模型速度

    # closeout: 评估周期 = 联合重复周期(moving-base=lcm(EE,BASE)),保证预热/稳态窗口
    # 覆盖足够的基座周期(0.75s 基座 → 12s 联合周期,如实拉长,不退回按 EE 周期统计)。
    eval_period = scenario.eval_period()
    warmup_steps = int(round(ctx.warmup_periods * eval_period / CONTROL_DT))
    steady_steps = int(round(ctx.steady_periods * eval_period / CONTROL_DT))
    total_steps = warmup_steps + steady_steps

    S = {k: [] for k in ("pos_err", "cmd_err", "ori_err", "err_vec_w",
                         "contour_err", "lag_err", "contour_active",
                         "exec_gap_j", "exec_gap_c", "ctrl_delta", "sigma",
                         "vel_peak", "accel_peak", "solve_ms", "solve_ms_steady",
                         "control_compute_ms",
                         "steady_flag")}
    guard_count = fallback = solver_fail = tracking_bad = singular = 0
    steady_tracking_bad = 0
    vel_violation = accel_violation = 0
    hold_then_jump_max = 0.0

    for k in range(total_steps):
        t_solve = k * CONTROL_DT
        t_next = t_solve + CONTROL_DT
        q_current = plant.q.copy()

        # dq_prev: 生产状态机(理想反馈时间戳 = t_solve)
        dq_prev = dq_prev_adapter.compute(q_current, t_solve)

        control_compute_start = time.perf_counter()
        # 参考时序: targets[j] = target(t_solve + (1+j)*dt)
        times = target_times(t_solve, horizon)
        targets = np.array([scenario.position_fn(float(ti)) for ti in times])
        rotations = (np.array([scenario.rotation_fn(float(ti)) for ti in times])
                     if lock else None)

        solve_start = time.perf_counter()
        if second_order:
            # 与生产的显式参考接口一致。场景 position_fn 已经是 base 系目标，
            # 因此这里直接中心差分即可；调参器没有落笔语义，轮廓模式保持 inactive。
            derivative_h = min(0.005, 0.5 * CONTROL_DT)
            target_velocities = np.array([
                (
                    scenario.position_fn(float(ti) + derivative_h)
                    - scenario.position_fn(float(ti) - derivative_h)
                ) / (2.0 * derivative_h)
                for ti in times
            ])
            target_accelerations = np.array([
                (
                    scenario.position_fn(float(ti) + derivative_h)
                    - 2.0 * scenario.position_fn(float(ti))
                    + scenario.position_fn(float(ti) - derivative_h)
                ) / (derivative_h * derivative_h)
                for ti in times
            ])
            contour_tangents = (
                np.array([contour_tangent_base(float(ti)) for ti in times])
                if activate_contour else None)
            contour_active = (
                np.array([
                    float(
                        np.linalg.norm(tangent) > 1e-8
                        and (
                            scenario.contour_active_fn(float(ti))
                            if scenario.contour_active_fn is not None
                            else True
                        )
                    )
                    for tangent, ti in zip(contour_tangents, times)
                ])
                if activate_contour else None)
            dq0 = (so_last_cmd_dq if (so_velocity_source == 'commanded'
                                      and so_last_cmd_dq is not None)
                   else plant.v_last.copy())
            q_next, dq_next, ok, info = solver.step(
                targets, q_current, dq0,
                warm_start=warm_start,
                target_rotation=rotations,
                target_velocity=target_velocities,
                target_acceleration=target_accelerations,
                contour_tangent=contour_tangents,
                contour_active=contour_active,
                posture_reference=q_nominal)
        else:
            q_next, ok, info = solver.step(
                targets, q_current, warm_start=warm_start,
                target_rotation=rotations,
                previous_joint_delta=dq_prev,
                posture_reference=q_nominal)
            dq_next = (q_next - q_current) / CONTROL_DT
        # 逆增益前馈补偿(阶段: 二阶接口修复): 发布 dq_next/comp,clip 到执行器速度限。
        dq_pub_candidate = (np.clip(dq_next / dq_ff_compensation,
                                    -PLANT_MAX_VELOCITY, PLANT_MAX_VELOCITY)
                           if (second_order and dq_ff_compensation != 1.0)
                           else dq_next)
        solve_wall = time.perf_counter() - solve_start
        control_compute_wall = time.perf_counter() - control_compute_start
        if k >= SOLVE_TIME_WARMUP_STEPS:
            ms = 1000.0 * float(info.get("solve_time", solve_wall))
            S["solve_ms"].append(ms)
            S["control_compute_ms"].append(1000.0 * control_compute_wall)
            if k >= warmup_steps:
                S["solve_ms_steady"].append(ms)

        command_usable = bool(info.get("command_usable", ok))
        if not bool(info.get("tracking_ok", ok)):
            tracking_bad += 1
            if k >= warmup_steps:
                steady_tracking_bad += 1
        if int(info.get("status", 0)) != 0 or not bool(info.get("success", True)):
            solver_fail += 1

        # ---- 守卫(逐行复刻生产 _run_tracking_phase;仅 guard_enabled 场景) ----
        dq_cmd = np.zeros_like(q_current)
        guard_block = False
        if command_usable:
            if scenario.guard_enabled and guard_rule == 'expected_residual':
                # 阶段1E 新守卫(离线简化版: 不做 λ 投影,超限即 block)。
                t_new = t_solve + CONTROL_DT
                p_cmd_new = fk_world(q_next, t_new)
                p_ref_new = (scenario.base_pose_fn(t_new)
                             + scenario.base_rot_fn(t_new) @ scenario.position_fn(t_new))
                blocked_1e = False
                if er_anchor is not None:
                    dp_ref = p_ref_new - er_anchor[1]
                    eps = 1e-3
                    pr = lambda tt: (scenario.base_pose_fn(tt)
                                     + scenario.base_rot_fn(tt) @ scenario.position_fn(tt))
                    a_ref = (pr(t_new + eps) - 2.0 * p_ref_new + pr(t_new - eps)) / (eps * eps)
                    r_allow = (0.0068 + 0.5 * np.linalg.norm(dp_ref)
                               + 1.0 * np.linalg.norm(a_ref) * CONTROL_DT * CONTROL_DT)
                    r_pos = np.linalg.norm((p_cmd_new - er_anchor[0]) - dp_ref)
                    blocked_1e = r_pos > r_allow
                if blocked_1e:
                    guard_block = True
                    guard_count += 1
                    warm_start = None
                else:
                    q_last = q_next
                    dq_cmd = dq_pub_candidate.copy()
                    warm_start = info.get("trajectory")
                    er_anchor = (p_cmd_new, p_ref_new)
            elif scenario.guard_enabled:
                new_fk_w = fk_world(q_next, t_solve)
                step_m = (np.linalg.norm(new_fk_w - last_command_fk_world)
                          if last_command_fk_world is not None else 0.0)
                if last_command_fk_world is not None and step_m > GUARD_STEP_WARN_M:
                    guard_block = True
                    guard_count += 1
                    # 生产: hold 时置 warm_start=None(注: 两侧求解器内部都会以
                    # _last_plan 兜底,该置空实际是 no-op —— 与生产共享同一潜在行为,
                    # 对齐成立;此处忠实复刻,不"修正"生产语义)。
                    warm_start = None
                else:
                    q_last = q_next
                    dq_cmd = dq_pub_candidate.copy()
                    warm_start = info.get("trajectory")
            else:
                q_last = q_next
                dq_cmd = dq_pub_candidate.copy()
                warm_start = info.get("trajectory")
        else:
            fallback += 1
            warm_start = None                  # 生产: 不可用时保持 q_last, dq=0

        if second_order:
            so_last_cmd_dq = (dq_next.copy() if (command_usable and not guard_block)
                              else np.zeros_like(q_current))
        # hold-then-jump 统计: 从 hold 恢复的首个放行命令的关节跳变
        if not guard_block and in_hold:
            hold_then_jump_max = max(hold_then_jump_max,
                                     float(np.max(np.abs(q_last - prev_q_pub))))
        in_hold = guard_block

        # 生产监视点更新: 每 tick 用当前 t_solve 对**已发布命令**求 world FK
        if scenario.guard_enabled:
            last_command_fk_world = fk_world(q_last, t_solve)

        # 发布(生产 velocity 限幅 = max_joint_step/dt)
        q_pub = q_last.copy()
        dq_pub = np.clip(dq_cmd, -velocity_limit, velocity_limit)

        # ---- plant 推进到 t_next;误差对 target(t_next)(时序对齐) ----
        q_actual = plant.advance(q_pub, dq_pub)
        v_actual = plant.v_last.copy()

        pos_a, rot_a = fk(ctx, q_actual)
        tgt_next = scenario.position_fn(t_next)
        e_base = pos_a - tgt_next
        S["pos_err"].append(float(np.linalg.norm(e_base)))
        # world 误差向量(基座残差分析): e_w = R(t_next) @ e_base(固定臂 R=I)
        if scenario.base_rot_fn is not None:
            e_world = scenario.base_rot_fn(t_next) @ e_base
        else:
            e_world = e_base.copy()
        S["err_vec_w"].append(e_world)
        contour_is_active = bool(
            activate_contour
            and (
                scenario.contour_active_fn(t_next)
                if scenario.contour_active_fn is not None else True
            )
        )
        S["contour_active"].append(contour_is_active)
        if contour_is_active:
            tangent_base = contour_tangent_base(t_next)
            tangent_world = (
                scenario.base_rot_fn(t_next) @ tangent_base
                if scenario.base_rot_fn is not None else tangent_base)
            lag_error = float(np.dot(e_world, tangent_world))
            contour_error = e_world - lag_error * tangent_world
            S["contour_err"].append(float(np.linalg.norm(contour_error)))
            S["lag_err"].append(abs(lag_error))
        else:
            S["contour_err"].append(0.0)
            S["lag_err"].append(0.0)
        # command-FK 误差(base 系范数 == world 系范数)
        pos_c, _ = fk(ctx, q_pub)
        S["cmd_err"].append(float(np.linalg.norm(pos_c - tgt_next)))
        if lock:
            S["ori_err"].append(float(np.linalg.norm(
                orientation_error(rot_a, scenario.rotation_fn(t_next)))))
        # 执行匹配
        S["exec_gap_j"].append(float(np.linalg.norm(q_actual - q_pub)))
        S["exec_gap_c"].append(float(np.linalg.norm(pos_a - pos_c)))
        # 控制增量平滑度(发布命令)
        S["ctrl_delta"].append(float(np.linalg.norm(q_pub - prev_q_pub)))
        prev_q_pub = q_pub.copy()
        # 预算
        S["vel_peak"].append(float(np.max(np.abs(v_actual))))
        accel = np.max(np.abs(v_actual - prev_v_actual)) / CONTROL_DT
        S["accel_peak"].append(float(accel))
        if np.max(np.abs(v_actual)) > PLANT_MAX_VELOCITY * VEL_LIMIT_TOL:
            vel_violation += 1
        if accel > PLANT_MAX_ACCEL * ACCEL_LIMIT_TOL:
            accel_violation += 1
        prev_v_actual = v_actual
        sigma = min_singular_value(ctx, q_actual)
        S["sigma"].append(sigma)
        if sigma < SINGULAR_SIGMA_THRESHOLD:
            singular += 1
        S["steady_flag"].append(k >= warmup_steps)

        if trace is not None:
            trace.append({
                "t_solve": t_solve, "t_next": t_next, "steady": k >= warmup_steps,
                "q_pub": q_pub.tolist(), "dq_pub": dq_pub.tolist(),
                "q_actual": q_actual.tolist(), "dq_actual": v_actual.tolist(),
                "v_ff": plant.last_v_ff.tolist(), "v_fb": plant.last_v_fb.tolist(),
                "v_cmd": plant.last_v_cmd.tolist(),
                "pos_err": S["pos_err"][-1], "cmd_err": S["cmd_err"][-1],
                "exec_gap_j": S["exec_gap_j"][-1], "exec_gap_c": S["exec_gap_c"][-1],
                "guard_block": bool(guard_block), "dq_prev": dq_prev.tolist(),
                "joint_clip": bool(plant.last_tick_clipped),
                "solver_status": int(info.get("status", -1)), "nit": int(info.get("nit", -1)),
            })

    steady = np.asarray(S["steady_flag"], dtype=bool)
    pos = np.asarray(S["pos_err"]); cmd = np.asarray(S["cmd_err"])
    ori = np.asarray(S["ori_err"]) if S["ori_err"] else np.zeros(0)
    # 姿态误差 steady 掩码(lock 时逐 tick 记录,与 steady 同长)
    ori_sd = ori[steady] if ori.size == steady.size else ori
    egj = np.asarray(S["exec_gap_j"]); egc = np.asarray(S["exec_gap_c"])
    contour = np.asarray(S["contour_err"]); lag = np.asarray(S["lag_err"])
    contour_active_mask = np.asarray(S["contour_active"], dtype=bool)
    contour_steady = steady & contour_active_mask
    cd = np.asarray(S["ctrl_delta"]); sg = np.asarray(S["sigma"])
    vp = np.asarray(S["vel_peak"]); ap = np.asarray(S["accel_peak"])
    st = np.asarray(S["solve_ms"]); st_sd = np.asarray(S["solve_ms_steady"])
    ct = np.asarray(S["control_compute_ms"])

    # 主基座频率残差(稳态): 对 world 误差向量各轴做正交拟合
    base_resid = 0.0
    if scenario.base_period_sec:
        ts = (np.arange(total_steps) + 1) * CONTROL_DT
        ev = np.asarray(S["err_vec_w"])
        f = 1.0 / scenario.base_period_sec
        amps = [
            _quadrature_fit(ts[steady], ev[steady, ax], f)[0] for ax in range(3)
        ]
        base_resid = math.sqrt(sum(a * a for a in amps) / 2.0)

    m = ScenarioMetrics(
        name=scenario.name,
        steady_pos_err_mean=float(np.mean(pos[steady])) if steady.any() else 0.0,
        steady_pos_err_rms=rms(pos[steady]),
        steady_pos_err_p95=_p(pos[steady], 95),
        steady_pos_err_max=float(np.max(pos[steady])) if steady.any() else 0.0,
        steady_cmd_err_rms=rms(cmd[steady]),
        steady_cmd_err_p95=_p(cmd[steady], 95),
        steady_ori_err_rms=rms(ori_sd),
        steady_ori_err_max=float(np.max(ori_sd)) if ori_sd.size else 0.0,
        steady_base_freq_resid_rms=base_resid,
        steady_contour_err_rms=rms(contour[contour_steady]),
        steady_contour_err_p95=_p(contour[contour_steady], 95),
        steady_lag_err_rms=rms(lag[contour_steady]),
        steady_exec_gap_joint_rms=rms(egj[steady]),
        steady_exec_gap_cart_rms=rms(egc[steady]),
        steady_ctrl_delta_rms=rms(cd[steady]),
        transient_pos_err_rms=rms(pos[~steady]),
        transient_pos_err_max=float(np.max(pos[~steady])) if (~steady).any() else 0.0,
        guard_count=guard_count,
        fallback_count=fallback,
        solver_failure_count=solver_fail,
        tracking_bad_count=tracking_bad,
        steady_tracking_bad_count=steady_tracking_bad,
        singular_count=singular,
        vel_violation_count=vel_violation,
        accel_violation_count=accel_violation,
        # 结构性首帧回退(tick0 且 code1=尚无历史锚点)精确排除;NaN/非递增/过旧/
        # 非首帧 code1 全部计入(见 DqPrevAdapter.nonstructural_fallback_count)。
        dq_prev_fallback_ratio=(dq_prev_adapter.nonstructural_fallback_count()
                                / max(1, dq_prev_adapter.tick_count)),
        min_sigma=float(np.min(sg)) if sg.size else 0.0,
        vel_peak=float(np.max(vp)) if vp.size else 0.0,
        accel_peak=float(np.max(ap)) if ap.size else 0.0,
        vel_sat_ratio=plant.vel_sat_substeps / max(1, plant.total_substeps),
        accel_sat_ratio=plant.accel_sat_substeps / max(1, plant.total_substeps),
        vel_sat_per_joint={f"joint{j + 1}": float(plant.vel_sat_per_joint[j]
                                                  / max(1, plant.total_substeps))
                           for j in range(len(q_seed))},
        accel_sat_per_joint={f"joint{j + 1}": float(plant.accel_sat_per_joint[j]
                                                    / max(1, plant.total_substeps))
                             for j in range(len(q_seed))},
        vel_sat_mean_joint_ratio=float(np.mean(plant.vel_sat_per_joint))
                                 / max(1, plant.total_substeps),
        vel_sat_max_joint_ratio=float(np.max(plant.vel_sat_per_joint))
                                / max(1, plant.total_substeps),
        accel_sat_mean_joint_ratio=float(np.mean(plant.accel_sat_per_joint))
                                   / max(1, plant.total_substeps),
        accel_sat_max_joint_ratio=float(np.max(plant.accel_sat_per_joint))
                                  / max(1, plant.total_substeps),
        accel_sat_worst_joint=f"joint{int(np.argmax(plant.accel_sat_per_joint)) + 1}",
        joint_limit_clip_count=int(plant.joint_clip_substeps),
        joint_limit_clip_ratio=plant.joint_clip_substeps / max(1, plant.total_substeps),
        joint_limit_active_count=int(plant.joint_active_substeps),
        joint_limit_active_ratio=plant.joint_active_substeps / max(1, plant.total_substeps),
        joint_limit_clip_per_joint={f"joint{j + 1}": int(plant.joint_clip_per_joint[j])
                                    for j in range(len(q_seed))},
        ee_period_sec=float(scenario.ee_period_sec or scenario.period_sec),
        base_period_sec=float(scenario.base_period_sec or 0.0),
        evaluation_period_sec=float(eval_period),
        warmup_sec=warmup_steps * CONTROL_DT,
        steady_sec=steady_steps * CONTROL_DT,
        hold_then_jump_max=hold_then_jump_max,
        mean_solve_time_ms=float(np.mean(st)) if st.size else 0.0,
        steady_mean_solve_time_ms=(float(np.mean(st_sd)) if st_sd.size
                                   else (float(np.mean(st)) if st.size else 0.0)),
        p95_solve_time_ms=_p(st, 95),
        p99_solve_time_ms=_p(st, 99),
        max_solve_time_ms=float(np.max(st)) if st.size else 0.0,
        mean_control_compute_time_ms=float(np.mean(ct)) if ct.size else 0.0,
        p95_control_compute_time_ms=_p(ct, 95),
        p99_control_compute_time_ms=_p(ct, 99),
        max_control_compute_time_ms=float(np.max(ct)) if ct.size else 0.0,
        score=0.0,
    )
    m.score = scenario_score(m)
    return m


def scenario_score(m: ScenarioMetrics) -> float:
    """软评分(硬门通过后才有意义): **严格只用稳态窗口指标**(tracking_bad/solve_time
    也用稳态口径,避免启动瞬态泄漏进参数排名;全程计数保留在指标里仅供报告)。"""
    return (
        1000.0 * m.steady_pos_err_rms
        + 1000.0 * m.steady_pos_err_p95
        + 500.0 * m.steady_pos_err_max          # closeout: max 进软评分(同时也有硬门)
        + 500.0 * m.steady_cmd_err_rms
        + 300.0 * m.steady_cmd_err_p95          # closeout: cmd P95 进软评分(同时也有硬门)
        + 1500.0 * m.steady_base_freq_resid_rms
        # 绘图稳定性优先：法向偏离形成“绕线/螺旋”，切向误差只代表进度滞后。
        + 2500.0 * m.steady_contour_err_rms
        + 1500.0 * m.steady_contour_err_p95
        + 250.0 * m.steady_lag_err_rms
        + 100.0 * m.steady_ori_err_rms
        + 100.0 * m.steady_ori_err_max
        + 300.0 * m.steady_exec_gap_cart_rms
        + 50.0 * m.steady_ctrl_delta_rms
        # closeout: 饱和/限位预算项 —— 误差相近时不选长期顶住物理限制的候选
        + 10.0 * m.vel_sat_ratio
        + 10.0 * m.accel_sat_ratio
        + 20.0 * m.joint_limit_active_ratio
        + 0.2 * m.steady_tracking_bad_count
        + 0.05 * m.steady_mean_solve_time_ms
    )


def hard_gate_reasons(m: "ScenarioMetrics", sc: Scenario,
                      ref: Optional["ScenarioMetrics"],
                      references_enabled: bool = False):
    """单场景硬门(可单测的纯函数)。返回 (淘汰原因列表, 备注列表)。

    绝对误差门用**场景自己的门限**(不串用);饱和相对门 vs 生产参考(无有效参考
    则跳过并备注,不造假门限)。"""
    checks = [
        (m.solver_failure_count > 0, f"solver_fail={m.solver_failure_count}"),
        (m.fallback_count > 0, f"fallback={m.fallback_count}"),
        (m.guard_count > 0, f"guard={m.guard_count}"),
        (m.singular_count > 0, f"singular={m.singular_count}"),
        # closeout: 关节位置撞限 => 硬淘汰(plant clip 不再伪装合法)。
        (m.joint_limit_clip_count > 0, f"joint_limit_clip={m.joint_limit_clip_count}"),
        # 注: 对齐 plant 的结构性限幅使 vel_violation 不可能触发(防御性保留)。
        (m.vel_violation_count > 0, f"vel_violation={m.vel_violation_count}"),
        (m.accel_violation_count > 0, f"accel_violation={m.accel_violation_count}"),
        (m.dq_prev_fallback_ratio > DQ_PREV_FALLBACK_MAX_RATIO,
         f"dq_prev_fallback={m.dq_prev_fallback_ratio:.4f}"),
        (m.p99_solve_time_ms > MAX_SOLVE_TIME_MS_HARD,
         f"slow_p99={m.p99_solve_time_ms:.1f}ms"),
        (m.p99_control_compute_time_ms > MAX_CONTROL_COMPUTE_TIME_MS_HARD,
         f"slow_control_compute_p99={m.p99_control_compute_time_ms:.1f}ms"
         f">{MAX_CONTROL_COMPUTE_TIME_MS_HARD:.1f}ms"),
    ]
    # closeout: 绝对误差硬门(场景各自的门限)。
    if sc.pos_p95_limit_m is not None and m.steady_pos_err_p95 > sc.pos_p95_limit_m:
        checks.append((True, f"pos_p95={m.steady_pos_err_p95*1000:.1f}mm"
                             f">limit={sc.pos_p95_limit_m*1000:.1f}mm"))
    if sc.pos_max_limit_m is not None and m.steady_pos_err_max > sc.pos_max_limit_m:
        checks.append((True, f"pos_max={m.steady_pos_err_max*1000:.1f}mm"
                             f">limit={sc.pos_max_limit_m*1000:.1f}mm"))
    if sc.cmd_p95_limit_m is not None and m.steady_cmd_err_p95 > sc.cmd_p95_limit_m:
        checks.append((True, f"cmd_p95={m.steady_cmd_err_p95*1000:.1f}mm"
                             f">limit={sc.cmd_p95_limit_m*1000:.1f}mm"))
    if (sc.orientation_limit_rad is not None
            and m.steady_ori_err_max > sc.orientation_limit_rad):
        checks.append((True, f"ori_max={m.steady_ori_err_max:.3f}rad"
                             f">limit={sc.orientation_limit_rad:.3f}rad"))
    notes = []
    if references_enabled:
        if ref is None:
            notes.append(f"{m.name}:NO_VALID_REFERENCE(饱和相对门跳过)")
        else:
            # closeout 修订: 相对门用 **max 每关节饱和率**(总体口径 accel≈0.99 会成空门)。
            if m.vel_sat_max_joint_ratio > ref.vel_sat_max_joint_ratio + VEL_SAT_RATIO_TOL:
                checks.append((True, f"vel_sat_max_joint={m.vel_sat_max_joint_ratio:.3f}"
                                     f">ref={ref.vel_sat_max_joint_ratio:.3f}"
                                     f"+{VEL_SAT_RATIO_TOL}"))
            if m.accel_sat_max_joint_ratio > ref.accel_sat_max_joint_ratio + ACCEL_SAT_RATIO_TOL:
                checks.append((True, f"accel_sat_max_joint={m.accel_sat_max_joint_ratio:.3f}"
                                     f">ref={ref.accel_sat_max_joint_ratio:.3f}"
                                     f"+{ACCEL_SAT_RATIO_TOL}"))
    fails = [f"{m.name}:{msg}" for bad, msg in checks if bad]
    return fails, notes


def evaluate(params: ParamSet, scenarios: Sequence[Scenario], ctx: Context,
             set_label: str,
             references: Optional[Dict[str, "ScenarioMetrics"]] = None,
             require_limits: bool = False) -> TrialResult:
    """评估一组参数。

    references: 生产参考参数(PRODUCTION_PARAMS)在各场景的指标 —— 饱和预算相对门用;
                场景无有效参考时跳过该相对门并在 reject_reason 外注明(不造假门限)。
    require_limits: 正式搜索置 True —— 场景缺必要误差门限直接抛错(快速失败),
                    credibility/单场景试跑置 False(只报告)。
    """
    if require_limits:
        missing = [sc.name for sc in scenarios if not sc.has_required_limits()]
        if missing:
            raise SystemExit(
                f"正式搜索要求每个场景配置误差硬门(pos_p95/pos_max/cmd_p95"
                f"[/orientation]),缺失: {missing}。请按真实基线+安全余量配置后再跑。")

    need = {sc.lock_orientation for sc in scenarios}
    solvers = {lockflag: build_solver(params, lockflag) for lockflag in need}

    results = [evaluate_scenario(sc, solvers[sc.lock_orientation], ctx, params.horizon_steps)
               for sc in scenarios]

    by_name = {sc.name: sc for sc in scenarios}
    feasible = True
    reasons = []
    for m in results:
        sc = by_name[m.name]
        fail_msgs, note_msgs = hard_gate_reasons(m, sc,
                                                references.get(m.name) if references is not None else None,
                                                references_enabled=references is not None)
        reasons.extend(note_msgs)
        if fail_msgs:
            feasible = False
            reasons.extend(fail_msgs)

    if feasible:
        scores = [m.score for m in results]
        aggregate = float(np.mean(scores) + np.max(scores))   # 最差场景显式进聚合
    else:
        aggregate = float("inf")

    return TrialResult(set_label, "", params, aggregate, feasible,
                       "; ".join(reasons), results)


def reference_is_valid(m: "ScenarioMetrics") -> bool:
    """生产参考在该场景是否可作为饱和相对门的基准。"""
    return (m.solver_failure_count == 0 and m.guard_count == 0
            and m.fallback_count == 0 and m.joint_limit_clip_count == 0)


def compute_references(scenarios: Sequence[Scenario], ctx: Context,
                       quiet_build: bool) -> Dict[str, "ScenarioMetrics"]:
    """先评估固定的 PRODUCTION_PARAMS,得到各场景的饱和参考;无效参考如实剔除。"""
    print(f"[reference] 用生产参数 {PRODUCTION_PARAMS} 建立各场景饱和参考…")
    with suppress_native_output(quiet_build):
        ref_trial = evaluate(PRODUCTION_PARAMS, scenarios, ctx, "reference")
    refs: Dict[str, ScenarioMetrics] = {}
    for m in ref_trial.scenarios:
        if reference_is_valid(m):
            refs[m.name] = m
            print(f"[reference] {m.name}: vel_sat_max_joint={m.vel_sat_max_joint_ratio:.3f} "
                  f"accel_sat_max_joint={m.accel_sat_max_joint_ratio:.3f} "
                  f"(容差 +{VEL_SAT_RATIO_TOL}/+{ACCEL_SAT_RATIO_TOL};"
                  f"总体口径 vel={m.vel_sat_ratio:.3f}/accel={m.accel_sat_ratio:.3f} 仅报告)")
        else:
            print(f"[reference] {m.name}: 生产参考本身无效(fail/guard/fallback/clip),"
                  f"该场景无有效饱和参考,相对门将跳过并在结果中注明。")
    return refs


# ======================================================================
# 搜索(结构沿用;T0 不运行正式搜索)
# ======================================================================
def coarse_candidates() -> List[ParamSet]:
    out = []
    for h, s, ow, tow in itertools.product(
        COARSE_HORIZON_STEPS, COARSE_MAX_JOINT_STEP,
        COARSE_ORIENTATION_WEIGHT, COARSE_TERMINAL_ORIENTATION_WEIGHT,
    ):
        if s > MAX_JOINT_STEP_LIMIT + 1e-9:
            continue
        out.append(ParamSet(h, round(s, 4), ow, tow))
    return out


def fine_candidates(top: Sequence[TrialResult], seen: Iterable[ParamSet]) -> List[ParamSet]:
    seen = set(seen)
    out = set()
    for r in top:
        p = r.params
        hs = sorted({max(1, p.horizon_steps + d) for d in FINE_HORIZON_DELTA})
        steps = sorted({round(p.max_joint_step + d, 4) for d in FINE_MAX_STEP_DELTA
                        if 0.0 < p.max_joint_step + d <= MAX_JOINT_STEP_LIMIT + 1e-9})
        ows = sorted({round(p.orientation_weight * s, 4) for s in FINE_WEIGHT_SCALE
                      if p.orientation_weight * s > 0.0})
        tows = sorted({round(p.terminal_orientation_weight * s, 4) for s in FINE_WEIGHT_SCALE
                       if p.terminal_orientation_weight * s > 0.0})
        for h, s, ow, tow in itertools.product(hs, steps, ows, tows):
            c = ParamSet(
                h, s, ow, tow,
                position_weight=p.position_weight,
                terminal_position_weight=p.terminal_position_weight,
                joint_delta_weight=p.joint_delta_weight,
                joint_delta_change_weight=p.joint_delta_change_weight,
                current_posture_weight=p.current_posture_weight,
                center_posture_weight=p.center_posture_weight,
            )
            if c not in seen:
                out.add(c)
    return sorted(out)


def maybe_limit(cands: List[ParamSet], n: Optional[int]) -> List[ParamSet]:
    if n is None or n <= 0:
        return cands
    return cands[:n]


@contextlib.contextmanager
def suppress_native_output(enabled: bool):
    if not enabled:
        yield
        return
    saved_out, saved_err = os.dup(1), os.dup(2)
    try:
        with open(os.devnull, "w") as devnull:
            os.dup2(devnull.fileno(), 1)
            os.dup2(devnull.fileno(), 2)
            yield
    finally:
        os.dup2(saved_out, 1)
        os.dup2(saved_err, 2)
        os.close(saved_out)
        os.close(saved_err)


def run_phase(phase: str, candidates: List[ParamSet], scenarios: Sequence[Scenario],
              ctx: Context, set_label: str, max_combos: Optional[int],
              quiet_build: bool,
              references: Optional[Dict[str, ScenarioMetrics]] = None) -> List[TrialResult]:
    cands = maybe_limit(candidates, max_combos)
    tag = f"[{set_label}] {phase}"
    if len(cands) < len(candidates):
        print(f"{tag} 搜索: 限制为 {len(cands)}/{len(candidates)} 组")
    else:
        print(f"{tag} 搜索: {len(cands)} 组")

    results = []
    for i, params in enumerate(cands, start=1):
        print(f"{tag} {i}/{len(cands)}: h={params.horizon_steps}, "
              f"step={params.max_joint_step:.3f}, pw={params.position_weight:.1f}, "
              f"tpw={params.terminal_position_weight:.1f}, "
              f"dw={params.joint_delta_weight:.4f}, "
              f"dcw={params.joint_delta_change_weight:.4f}, "
              f"cpw={params.current_posture_weight:.4f}, "
              f"zpw={params.center_posture_weight:.4f}", flush=True)
        with suppress_native_output(quiet_build):
            r = evaluate(params, scenarios, ctx, set_label,
                         references=references, require_limits=True)
        r.phase = phase
        results.append(r)
        if r.feasible:
            worst = max(r.scenarios, key=lambda m: m.score)
            print(f"    agg={r.aggregate_score:.3f}, worst={worst.name} "
                  f"(steady_rms={worst.steady_pos_err_rms:.4f}, "
                  f"steady_p95={worst.steady_pos_err_p95:.4f}, "
                  f"min_sigma={worst.min_sigma:.3f}, mean_ms={worst.mean_solve_time_ms:.2f})",
                  flush=True)
        else:
            print(f"    判废: {r.reject_reason}", flush=True)
    return results


def search(set_label: str, scenarios: Sequence[Scenario], ctx: Context,
           args: argparse.Namespace) -> List[TrialResult]:
    # closeout: 正式搜索前置检查 —— 每个场景必须配齐误差硬门(快速失败,不静默跳过)。
    missing = [sc.name for sc in scenarios if not sc.has_required_limits()]
    if missing:
        raise SystemExit(
            f"[{set_label}] 场景缺误差硬门配置: {missing}。正式搜索要求每个场景"
            f"按真实基线+安全余量配置 pos_p95/pos_max/cmd_p95[/orientation] 门限。")
    # 先建立生产参考(饱和相对门基准)。
    references = compute_references(scenarios, ctx, args.quiet_build)

    if getattr(args, "phase2a", False):
        return run_phase("phase2a_weights", phase2a_candidates(), scenarios, ctx,
                         set_label, args.max_combinations, args.quiet_build,
                         references=references)

    all_results: List[TrialResult] = []
    coarse_results = run_phase("coarse", coarse_candidates(), scenarios, ctx, set_label,
                               args.max_combinations, args.quiet_build,
                               references=references)
    all_results.extend(coarse_results)

    if not args.skip_fine:
        feasible = sorted([r for r in coarse_results if r.feasible],
                          key=lambda r: r.aggregate_score)[:args.fine_top]
        if feasible:
            fine = fine_candidates(feasible, seen=[r.params for r in all_results])
            all_results.extend(run_phase("fine", fine, scenarios, ctx, set_label,
                                         args.max_combinations, args.quiet_build,
                                         references=references))
        else:
            print(f"[{set_label}] 粗搜索没有可行组, 跳过细搜索。")
    return all_results


# ======================================================================
# 输出
# ======================================================================
def recommended_command(scenario: Scenario, params: ParamSet) -> str:
    parts = [f"horizon_steps:={params.horizon_steps}",
             f"max_joint_step:={params.max_joint_step:.4f}",
             f"position_weight:={params.position_weight:.4f}",
             f"terminal_position_weight:={params.terminal_position_weight:.4f}",
             f"joint_delta_weight:={params.joint_delta_weight:.4f}",
             f"joint_delta_change_weight:={params.joint_delta_change_weight:.4f}",
             f"current_posture_weight:={params.current_posture_weight:.4f}",
             f"center_posture_weight:={params.center_posture_weight:.4f}"]
    parts += scenario.demo_extra_args
    if scenario.lock_orientation:
        parts += [f"orientation_weight:={params.orientation_weight:.4f}",
                  f"terminal_orientation_weight:={params.terminal_orientation_weight:.4f}"]
    return f"python3 {scenario.demo} --ros-args " + " ".join(f"-p {p}" for p in parts)


METRIC_FIELDS = [
    "steady_pos_err_mean", "steady_pos_err_rms", "steady_pos_err_p95", "steady_pos_err_max",
    "steady_cmd_err_rms", "steady_cmd_err_p95", "steady_ori_err_rms", "steady_ori_err_max",
    "steady_base_freq_resid_rms", "steady_exec_gap_joint_rms", "steady_exec_gap_cart_rms",
    "steady_ctrl_delta_rms", "transient_pos_err_rms", "transient_pos_err_max",
    "guard_count", "fallback_count", "solver_failure_count", "tracking_bad_count",
    "steady_tracking_bad_count",
    "singular_count", "vel_violation_count", "accel_violation_count",
    "dq_prev_fallback_ratio", "min_sigma", "vel_peak", "accel_peak",
    "vel_sat_ratio", "accel_sat_ratio",
    "vel_sat_mean_joint_ratio", "vel_sat_max_joint_ratio",
    "accel_sat_mean_joint_ratio", "accel_sat_max_joint_ratio", "accel_sat_worst_joint",
    "joint_limit_clip_count", "joint_limit_clip_ratio",
    "joint_limit_active_count", "joint_limit_active_ratio",
    "ee_period_sec", "base_period_sec", "evaluation_period_sec",
    "warmup_sec", "steady_sec",
    "hold_then_jump_max",
    "mean_solve_time_ms", "steady_mean_solve_time_ms",
    "p95_solve_time_ms", "p99_solve_time_ms", "max_solve_time_ms",
    "mean_control_compute_time_ms", "p95_control_compute_time_ms",
    "p99_control_compute_time_ms", "max_control_compute_time_ms",
]


def write_results(path: Path, results: Sequence[TrialResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "scenario_set", "rank", "phase", "feasible", "aggregate_score", "reject_reason",
        "horizon_steps", "max_joint_step", "orientation_weight", "terminal_orientation_weight",
        "position_weight", "terminal_position_weight",
        "joint_delta_weight", "joint_delta_change_weight",
        "current_posture_weight", "center_posture_weight",
        "scenario", "scenario_score",
    ] + METRIC_FIELDS
    by_set: Dict[str, List[TrialResult]] = {}
    for r in results:
        by_set.setdefault(r.scenario_set, []).append(r)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for label, rs in by_set.items():
            for rank, r in enumerate(sorted(rs, key=lambda x: x.aggregate_score), start=1):
                p = r.params
                for m in r.scenarios:
                    row = {
                        "scenario_set": label, "rank": rank, "phase": r.phase,
                        "feasible": r.feasible, "aggregate_score": f"{r.aggregate_score:.6f}",
                        "reject_reason": r.reject_reason,
                        "horizon_steps": p.horizon_steps,
                        "max_joint_step": f"{p.max_joint_step:.6f}",
                        "orientation_weight": f"{p.orientation_weight:.6f}",
                        "terminal_orientation_weight": f"{p.terminal_orientation_weight:.6f}",
                        "position_weight": f"{p.position_weight:.6f}",
                        "terminal_position_weight": f"{p.terminal_position_weight:.6f}",
                        "joint_delta_weight": f"{p.joint_delta_weight:.6f}",
                        "joint_delta_change_weight": f"{p.joint_delta_change_weight:.6f}",
                        "current_posture_weight": f"{p.current_posture_weight:.6f}",
                        "center_posture_weight": f"{p.center_posture_weight:.6f}",
                        "scenario": m.name, "scenario_score": f"{m.score:.6f}",
                    }
                    for k in METRIC_FIELDS:
                        v = getattr(m, k)
                        row[k] = f"{v:.9f}" if isinstance(v, float) else v
                    writer.writerow(row)


def print_set(set_label: str, scenarios: Sequence[Scenario],
              results: Sequence[TrialResult], top_k: int) -> Optional[TrialResult]:
    feasible = sorted([r for r in results if r.feasible], key=lambda r: r.aggregate_score)
    print(f"\n===== [{set_label}] 可行参数组: {len(feasible)}/{len(results)} =====")
    if not feasible:
        print("  没有满足硬门(无守卫/无 fallback/无失败/无奇异/无限幅违规/不超时)的参数组。")
        return None
    print("  rank  agg     h  step   ow    tow    worst_scenario   steady_p95 min_sigma mean_ms")
    for rank, r in enumerate(feasible[:top_k], start=1):
        p = r.params
        worst = max(r.scenarios, key=lambda m: m.score)
        print(f"  {rank:>4d} {r.aggregate_score:>7.3f} {p.horizon_steps:>3d} "
              f"{p.max_joint_step:>6.3f} {p.orientation_weight:>5.2f} {p.terminal_orientation_weight:>6.2f} "
              f"{worst.name:>16s} {worst.steady_pos_err_p95:>10.4f} {worst.min_sigma:>8.3f} "
              f"{worst.mean_solve_time_ms:>7.2f}")
    best = feasible[0]
    print("  冠军各场景明细(稳态):")
    for m in best.scenarios:
        print(f"    {m.name:>24s} rms={m.steady_pos_err_rms*1000:5.2f}mm "
              f"p95={m.steady_pos_err_p95*1000:5.2f}mm exec_gap_c={m.steady_exec_gap_cart_rms*1000:4.2f}mm "
              f"guard={m.guard_count} fb={m.fallback_count} fail={m.solver_failure_count} "
              f"mean_ms={m.mean_solve_time_ms:.2f}")
    print("  推荐运行命令:")
    by_name = {s.name: s for s in scenarios}
    for m in best.scenarios:
        if m.name in by_name:
            print("    " + recommended_command(by_name[m.name], best.params))
    return best


# ======================================================================
# T0 可信度模式: 生产参数 × 最小代表场景 + 轨迹落盘(供与真实数据对照)
# ======================================================================
PRODUCTION_PARAMS = ParamSet(2, 0.02, 0.09375, 1.25)
BASE_PRODUCTION_PARAMS = PRODUCTION_PARAMS
PHASE2A_POSITION_SCALES = (0.5, 1.0, 2.0)
PHASE2A_DELTA_WEIGHTS = (0.04, 0.08, 0.16)
PHASE2A_DELTA_CHANGE_WEIGHTS = (0.0625, 0.125, 0.25)
POSTURE_WEIGHT_SCALES = (0.5, 1.0, 2.0)
SECOND_ORDER_WEIGHT_SCALES = (0.5, 1.0, 2.0)


def phase2a_candidates(base: Optional[ParamSet] = None) -> List[ParamSet]:
    """阶段2A：固定结构、约束和姿态权重，只筛位置与平滑权重。

    position/terminal_position 保持 1:10 比例，避免无必要的四维笛卡尔积；
    3×3×3 共 27 组，包含当前生产参数。
    """
    base = PRODUCTION_PARAMS if base is None else base
    out = []
    for ps, dw, dcw in itertools.product(
            PHASE2A_POSITION_SCALES,
            PHASE2A_DELTA_WEIGHTS,
            PHASE2A_DELTA_CHANGE_WEIGHTS):
        out.append(ParamSet(
            base.horizon_steps,
            base.max_joint_step,
            base.orientation_weight,
            base.terminal_orientation_weight,
            base.position_weight * ps,
            base.terminal_position_weight * ps,
            dw,
            dcw,
            base.current_posture_weight,
            base.center_posture_weight,
        ))
    return out


def phase2a_posture_candidates(
    base: Optional[ParamSet] = None,
    scales: Sequence[float] = POSTURE_WEIGHT_SCALES,
) -> List[ParamSet]:
    """位置/平滑阶段选出基线后，单独微调两项姿态锚权重。

    返回基线加两条单坐标扫描，不做 current×center 笛卡尔积。默认共 5 组：
    baseline、current×{0.5,2}、center×{0.5,2}。
    """
    base = PRODUCTION_PARAMS if base is None else base
    candidates = {base}
    for scale in scales:
        scale = float(scale)
        if not math.isfinite(scale) or scale < 0.0:
            raise ValueError("姿态权重 scale 必须是有限非负数")
        candidates.add(replace(
            base, current_posture_weight=base.current_posture_weight * scale))
        candidates.add(replace(
            base, center_posture_weight=base.center_posture_weight * scale))
    return sorted(candidates)


def second_order_weight_candidates(
    base_config: Optional[SecondOrderMpcConfig] = None,
    scales: Sequence[float] = SECOND_ORDER_WEIGHT_SCALES,
) -> List[SecondOrderWeightSet]:
    """生成二阶 NMPC 紧凑权重候选：局部单变量 + 少量诊断候选。

    六项权重分别为 position、terminal_position、velocity、acceleration、
    current_posture 和 center_posture。除 ±2 倍局部扫描外，位置权重额外覆盖
    ×10/100/1000，加速度权重覆盖 0；并加入两组“高位置权重+零加速度软罚”
    诊断组合。硬加速度约束始终保留。
    """
    config = SecondOrderMpcConfig() if base_config is None else base_config
    baseline = SecondOrderWeightSet.from_config(config)
    candidates = {baseline}
    fields = (
        "position_weight",
        "terminal_position_weight",
        "joint_velocity_weight",
        "joint_acceleration_weight",
        "current_posture_weight",
        "center_posture_weight",
    )
    for scale in scales:
        scale = float(scale)
        if not math.isfinite(scale) or scale < 0.0:
            raise ValueError("二阶权重 scale 必须是有限非负数")
        for name in fields:
            candidates.add(replace(
                baseline, **{name: getattr(baseline, name) * scale}))
    for scale in (10.0, 100.0, 1000.0):
        candidates.add(replace(
            baseline, position_weight=baseline.position_weight * scale))
    candidates.add(replace(baseline, joint_acceleration_weight=0.0))
    for scale in (100.0, 1000.0):
        candidates.add(replace(
            baseline,
            position_weight=baseline.position_weight * scale,
            joint_acceleration_weight=0.0,
        ))
    return sorted(candidates)


def second_order_config_candidates(
    base_config: Optional[SecondOrderMpcConfig] = None,
    scales: Sequence[float] = SECOND_ORDER_WEIGHT_SCALES,
) -> List[SecondOrderMpcConfig]:
    """把紧凑纯权重候选应用到 SecondOrderMpcConfig 的独立副本。"""
    config = SecondOrderMpcConfig() if base_config is None else base_config
    return [weights.apply(config)
            for weights in second_order_weight_candidates(config, scales)]


def production_second_order_config() -> SecondOrderMpcConfig:
    """与 moving-base demo 的二阶显式默认保持一致。"""
    return SecondOrderMpcConfig(
        horizon_steps=10,
        control_dt=CONTROL_DT,
        position_weight=5000.0,
        terminal_position_weight=500.0,
        cartesian_residual_mode="external_affine",
        cartesian_velocity_weight=0.0,
        cartesian_acceleration_weight=0.0,
        terminal_cartesian_velocity_weight=0.0,
        feasible_reference_projection=False,
        feasible_reference_damping=1e-4,
        contour_tracking_enabled=False,
        contour_position_weight=5000.0,
        lag_position_weight=750.0,
        transition_position_weight=5000.0,
        terminal_contour_position_weight=500.0,
        terminal_lag_position_weight=75.0,
        terminal_transition_position_weight=500.0,
        orientation_weight=PRODUCTION_PARAMS.orientation_weight,
        terminal_orientation_weight=PRODUCTION_PARAMS.terminal_orientation_weight,
        current_posture_weight=PRODUCTION_PARAMS.current_posture_weight,
        center_posture_weight=PRODUCTION_PARAMS.center_posture_weight,
        joint_velocity_weight=0.02,
        joint_acceleration_weight=0.0,
        terminal_joint_velocity_weight=0.0,
        max_joint_velocity=PLANT_MAX_VELOCITY,
        max_joint_acceleration=PLANT_MAX_ACCEL,
        singularity_constraint_enabled=True,
        min_translation_jacobian_sigma=SINGULAR_SIGMA_THRESHOLD,
        obstacle_avoidance_enabled=False,
        body_obstacle_enabled=False,
        rti_iteration_mode=RTI_ITERATION_MODE,
        acados_rti_iterations=ACADOS_RTI_ITERATIONS,
    )


def stable_contour_second_order_candidates(
        base_config: Optional[SecondOrderMpcConfig] = None,
) -> List[SecondOrderMpcConfig]:
    """Compact coordinate search for the 2 Hz drawing-stability problem.

    The search deliberately keeps the physical velocity/acceleration limits
    fixed.  It removes the inconsistent stage-wise derivative projection,
    restores enough lag weight to prevent the reference from running away, and
    scans only the parameters that affect geometric contour tracking.
    """
    base = production_second_order_config() if base_config is None else base_config
    common = dict(
        feasible_reference_projection=False,
        cartesian_acceleration_weight=0.0,
        joint_acceleration_weight=0.0,
    )
    candidates = [
        # Exact position-only baseline, retained as a hard A/B anchor.
        replace(
            base,
            contour_tracking_enabled=False,
            cartesian_velocity_weight=0.0,
            terminal_cartesian_velocity_weight=0.0,
            **common,
        ),
        # Current production candidate, so regression is visible in one report.
        base,
    ]

    # First restore progress/lag authority while preserving contour priority.
    for lag_ratio in (0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 1.00):
        candidates.append(replace(
            base,
            contour_tracking_enabled=True,
            contour_position_weight=5000.0,
            lag_position_weight=5000.0 * lag_ratio,
            terminal_contour_position_weight=500.0,
            terminal_lag_position_weight=500.0 * lag_ratio,
            cartesian_velocity_weight=0.0,
            terminal_cartesian_velocity_weight=0.0,
            **common,
        ))

    # Coordinate scans around the balanced lag candidate.  This avoids a large
    # Cartesian product while still exposing contour authority and derivative
    # feed-forward as independent causes.
    for contour_weight in (2500.0, 10000.0, 20000.0):
        candidates.append(replace(
            base,
            contour_tracking_enabled=True,
            contour_position_weight=contour_weight,
            lag_position_weight=0.5 * contour_weight,
            terminal_contour_position_weight=0.1 * contour_weight,
            terminal_lag_position_weight=0.05 * contour_weight,
            cartesian_velocity_weight=0.0,
            terminal_cartesian_velocity_weight=0.0,
            **common,
        ))
    for velocity_weight in (0.25, 0.5, 1.0):
        candidates.append(replace(
            base,
            contour_tracking_enabled=True,
            contour_position_weight=5000.0,
            lag_position_weight=2500.0,
            terminal_contour_position_weight=500.0,
            terminal_lag_position_weight=250.0,
            cartesian_velocity_weight=velocity_weight,
            terminal_cartesian_velocity_weight=velocity_weight,
            **common,
        ))
    unique = {
        json.dumps(asdict(cfg), sort_keys=True, default=str): cfg
        for cfg in candidates
    }
    return sorted(unique.values(), key=lambda cfg: (
        cfg.contour_tracking_enabled,
        cfg.contour_position_weight,
        cfg.lag_position_weight,
        cfg.cartesian_velocity_weight,
        cfg.feasible_reference_projection,
    ))


def second_order_tune(ctx: Context, output_dir: Path, max_combinations: Optional[int],
                      quiet_build: bool, base_period_sec: Optional[float] = None,
                      ee_period_sec: Optional[float] = None,
                      horizons: Sequence[int] = (10,),
                      dq_ff_compensation: float = 1.0,
                      guard_rule: str = 'legacy',
                      so_velocity_source: str = 'measured',
                      plant_gain: float = PLANT_FF_GAIN,
                      stable_contour: bool = False) -> Path:
    """用 T0 闭环语义评估二阶候选(权重 × horizon;发布接口可带逆增益补偿)。"""
    scenario = make_moving_base_scenario(
        name="moving_base_circle_second_order_tune",
        base_period_sec=base_period_sec,
        ee_period_sec=ee_period_sec,
        attach_production_limits=False,
    )
    seed_ik = PinocchioIK()
    q0, ok = seed_ik.solve(
        scenario.position_fn(0.0),
        target_rotation=scenario.rotation_fn(0.0),
        rot_weight=0.2)
    if not ok:
        raise RuntimeError("二阶调参场景起点 IK 不可达")
    scenario.q_seed = np.clip(q0, ctx.q_lower, ctx.q_upper)
    baseline_config = production_second_order_config()
    baseline_weights = SecondOrderWeightSet.from_config(baseline_config)
    with suppress_native_output(quiet_build):
        reference = evaluate_scenario(
            scenario, SecondOrderMpcIK(config=baseline_config), ctx,
            baseline_config.horizon_steps,
            plant_gain=plant_gain,
            so_velocity_source=so_velocity_source,
            activate_contour=stable_contour)
    candidate_configs = (
        stable_contour_second_order_candidates(baseline_config)
        if stable_contour else second_order_config_candidates(baseline_config))
    configs = [
        replace(c, horizon_steps=int(h))
        for h in horizons for c in candidate_configs
    ]
    configs = maybe_limit(configs, max_combinations)
    records = []
    output_dir.mkdir(parents=True, exist_ok=True)
    for index, config in enumerate(configs, start=1):
        print(
            f"[second-order] {index}/{len(configs)} "
            f"N={config.horizon_steps} contour={int(config.contour_tracking_enabled)} "
            f"pw={config.position_weight:g} tpw={config.terminal_position_weight:g} "
            f"cw={config.contour_position_weight:g} lw={config.lag_position_weight:g} "
            f"cv={config.cartesian_velocity_weight:g} "
            f"vw={config.joint_velocity_weight:g} aw={config.joint_acceleration_weight:g} "
            f"cpw={config.current_posture_weight:g} zpw={config.center_posture_weight:g}",
            flush=True)
        weights = SecondOrderWeightSet.from_config(config)
        if (config == baseline_config
                and dq_ff_compensation == 1.0 and guard_rule == 'legacy'):
            metrics = reference
        else:
            with suppress_native_output(quiet_build):
                solver = SecondOrderMpcIK(config=replace(
                    config,
                    acados_build_dir=f'/tmp/windylab_acados_so_tune_h{config.horizon_steps}'))
                metrics = evaluate_scenario(
                    scenario, solver, ctx, config.horizon_steps,
                    plant_gain=plant_gain,
                    dq_ff_compensation=dq_ff_compensation, guard_rule=guard_rule,
                    so_velocity_source=so_velocity_source,
                    activate_contour=stable_contour)
        failures, notes = hard_gate_reasons(
            metrics, scenario, reference, references_enabled=True)
        records.append({
            "weights": asdict(weights),
            "drawing_weights": {
                "contour_tracking_enabled": config.contour_tracking_enabled,
                "contour_position_weight": config.contour_position_weight,
                "lag_position_weight": config.lag_position_weight,
                "terminal_contour_position_weight":
                    config.terminal_contour_position_weight,
                "terminal_lag_position_weight":
                    config.terminal_lag_position_weight,
                "cartesian_velocity_weight":
                    config.cartesian_velocity_weight,
                "terminal_cartesian_velocity_weight":
                    config.terminal_cartesian_velocity_weight,
                "feasible_reference_projection":
                    config.feasible_reference_projection,
            },
            "fixed_structure": {
                "plant_gain": plant_gain,
                "stable_contour": stable_contour,
                "dq_ff_compensation": dq_ff_compensation,
                "guard_rule": guard_rule,
                "so_velocity_source": so_velocity_source,
                "horizon_steps": config.horizon_steps,
                "control_dt": config.control_dt,
                "max_joint_velocity": config.max_joint_velocity,
                "max_joint_acceleration": config.max_joint_acceleration,
                "min_translation_jacobian_sigma":
                    config.min_translation_jacobian_sigma,
            },
            "score": metrics.score,
            "feasible": not failures,
            "reasons": failures + notes,
            "metrics": asdict(metrics),
        })
    records.sort(key=lambda item: (not item["feasible"], item["score"]))
    payload = {
        "solver_model": "second_order",
        "scenario": {
            "base_period_sec": scenario.base_period_sec,
            "ee_period_sec": scenario.ee_period_sec,
            "evaluation_period_sec": scenario.eval_period(),
        },
        "candidate_policy": (
            "2Hz contour/lag/velocity coordinate search"
            if stable_contour
            else "baseline + one-weight-at-a-time x{0.5,2.0}"),
        "candidate_count": len(records),
        "results": records,
    }
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = output_dir / f"second_order_weight_tuning_{stamp}.json"
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False))
    print(f"[second-order] 结果: {path}")
    if records:
        best = records[0]
        print(f"[second-order] best feasible={best['feasible']} "
              f"score={best['score']:.4f} weights={best['weights']}")
    return path


def configure_publish_rate(rate_hz: float) -> None:
    """为离线评估切换发布频率，同时保持 1 rad/s 关节速度上限。

    50 Hz: dt=20ms, max_joint_step=0.02, horizon=2 (40ms)
    100 Hz: dt=10ms, max_joint_step=0.01, horizon=4 (仍为40ms)
    """
    global PUBLISH_RATE_HZ, CONTROL_DT, N_SUBSTEPS
    global MAX_JOINT_STEP_LIMIT, MAX_SOLVE_TIME_MS_HARD, PRODUCTION_PARAMS
    if rate_hz not in (50.0, 100.0):
        raise ValueError("publish rate 只支持 50 或 100 Hz")
    PUBLISH_RATE_HZ = float(rate_hz)
    CONTROL_DT = 1.0 / PUBLISH_RATE_HZ
    N_SUBSTEPS = max(1, int(round(CONTROL_DT / CTRL_SUBSTEP_DT)))
    MAX_JOINT_STEP_LIMIT = PLANT_MAX_VELOCITY * CONTROL_DT
    MAX_SOLVE_TIME_MS_HARD = 1000.0 * CONTROL_DT
    horizon = int(round(BASE_PRODUCTION_PARAMS.horizon_steps * rate_hz / 50.0))
    PRODUCTION_PARAMS = ParamSet(
        horizon,
        MAX_JOINT_STEP_LIMIT,
        BASE_PRODUCTION_PARAMS.orientation_weight,
        BASE_PRODUCTION_PARAMS.terminal_orientation_weight,
        BASE_PRODUCTION_PARAMS.position_weight,
        BASE_PRODUCTION_PARAMS.terminal_position_weight,
        BASE_PRODUCTION_PARAMS.joint_delta_weight,
        BASE_PRODUCTION_PARAMS.joint_delta_change_weight,
        BASE_PRODUCTION_PARAMS.current_posture_weight,
        BASE_PRODUCTION_PARAMS.center_posture_weight,
    )


def rate_compare(ctx: Context, out_dir: Path, quiet_build: bool) -> dict:
    """同一 40ms 预测时长、同一 1rad/s 速度上限下比较 50/100Hz。"""
    result = {"comparison": "50Hz_vs_100Hz", "rates": {}}
    for rate in (50.0, 100.0):
        configure_publish_rate(rate)
        scenarios = build_scenarios(ctx)
        sc = next(item for item in scenarios if item.name == "moving_base_circle")
        with suppress_native_output(quiet_build):
            solver = build_solver(PRODUCTION_PARAMS, sc.lock_orientation)
            metrics = evaluate_scenario(sc, solver, ctx, PRODUCTION_PARAMS.horizon_steps)
        result["rates"][str(int(rate))] = {
            "control_dt_sec": CONTROL_DT,
            "horizon_steps": PRODUCTION_PARAMS.horizon_steps,
            "prediction_horizon_sec": CONTROL_DT * PRODUCTION_PARAMS.horizon_steps,
            "max_joint_step_rad": PRODUCTION_PARAMS.max_joint_step,
            "steady_pos_rms_mm": metrics.steady_pos_err_rms * 1000.0,
            "steady_pos_p95_mm": metrics.steady_pos_err_p95 * 1000.0,
            "steady_cmd_p95_mm": metrics.steady_cmd_err_p95 * 1000.0,
            "steady_exec_gap_cart_rms_mm": metrics.steady_exec_gap_cart_rms * 1000.0,
            "steady_ctrl_delta_rms_rad": metrics.steady_ctrl_delta_rms,
            "guard_count": metrics.guard_count,
            "solver_failure_count": metrics.solver_failure_count,
            "p99_solve_time_ms": metrics.p99_solve_time_ms,
            "deadline_ms": 1000.0 * CONTROL_DT,
            "min_sigma": metrics.min_sigma,
            "vel_peak_rad_s": metrics.vel_peak,
            "accel_peak_rad_s2": metrics.accel_peak,
        }
    r50, r100 = result["rates"]["50"], result["rates"]["100"]
    result["delta_100_minus_50"] = {
        key: r100[key] - r50[key]
        for key in ("steady_pos_rms_mm", "steady_pos_p95_mm",
                    "steady_cmd_p95_mm", "steady_exec_gap_cart_rms_mm",
                    "p99_solve_time_ms")
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "rate_compare_50_100.json"
    path.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"结果: {path}")
    return result


def credibility_run(ctx: Context, out_dir: Path, plant_gain: float,
                    quiet_build: bool) -> dict:
    """跑生产参数 + 生产 moving-base 场景的最小闭环,导出逐 tick trace 与指标 JSON。"""
    sc = make_moving_base_scenario()
    seed_ik = PinocchioIK()
    q0, ok = seed_ik.solve(sc.position_fn(0.0), target_rotation=sc.rotation_fn(0.0),
                           rot_weight=0.2)
    if not ok:
        raise RuntimeError("credibility 场景起点 IK 不可达")
    sc.q_seed = np.clip(q0, ctx.q_lower, ctx.q_upper)

    with suppress_native_output(quiet_build):
        solver = build_solver(PRODUCTION_PARAMS, sc.lock_orientation)
    trace: list = []
    m = evaluate_scenario(sc, solver, ctx, PRODUCTION_PARAMS.horizon_steps,
                          plant_gain=plant_gain, trace=trace)

    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = f"gain{plant_gain:g}"
    trace_path = out_dir / f"tuner_t0_trace_{tag}_{stamp}.csv"
    with trace_path.open("w", newline="") as f:
        cols = ["t_solve", "t_next", "steady", "pos_err", "cmd_err",
                "exec_gap_j", "exec_gap_c", "guard_block", "solver_status", "nit"]
        jcols = [f"{p}_{i+1}" for p in ("q_pub", "dq_pub", "q_actual", "dq_actual",
                                        "v_ff", "v_fb", "v_cmd", "dq_prev") for i in range(7)]
        w = csv.writer(f)
        w.writerow(cols + jcols)
        for row in trace:
            flat = [row[c] if not isinstance(row[c], bool) else int(row[c]) for c in cols]
            for p in ("q_pub", "dq_pub", "q_actual", "dq_actual", "v_ff", "v_fb", "v_cmd", "dq_prev"):
                flat += row[p]
            w.writerow(flat)

    metrics = {k: getattr(m, k) for k in METRIC_FIELDS}
    metrics.update({
        "scenario": sc.name, "plant_gain": plant_gain,
        "params": {"horizon_steps": PRODUCTION_PARAMS.horizon_steps,
                   "max_joint_step": PRODUCTION_PARAMS.max_joint_step,
                   "orientation_weight": PRODUCTION_PARAMS.orientation_weight,
                   "terminal_orientation_weight": PRODUCTION_PARAMS.terminal_orientation_weight,
                   "position_weight": PRODUCTION_PARAMS.position_weight,
                   "terminal_position_weight": PRODUCTION_PARAMS.terminal_position_weight,
                   "joint_delta_weight": PRODUCTION_PARAMS.joint_delta_weight,
                   "joint_delta_change_weight": PRODUCTION_PARAMS.joint_delta_change_weight,
                   "current_posture_weight": PRODUCTION_PARAMS.current_posture_weight,
                   "center_posture_weight": PRODUCTION_PARAMS.center_posture_weight},
        "semantics": {"reference_lead_steps": REFERENCE_LEAD_STEPS,
                      "dq_prev_mode": DQ_PREV_MODE,
                      "rti_iteration_mode": RTI_ITERATION_MODE,
                      "acados_rti_iterations": ACADOS_RTI_ITERATIONS,
                      "guard_step_warn_m": GUARD_STEP_WARN_M,
                      "plant": {"ff_gain": plant_gain, "fb_tau": PLANT_FB_TAU,
                                "fb_vel_limit": PLANT_FB_VEL_LIMIT,
                                "max_velocity": PLANT_MAX_VELOCITY,
                                "max_accel": PLANT_MAX_ACCEL,
                                "substep_dt": CTRL_SUBSTEP_DT}},
        "warmup_periods": ctx.warmup_periods, "steady_periods": ctx.steady_periods,
        "trace_csv": str(trace_path),
        # closeout 修订: 工况指纹与每关节饱和明细进 credibility JSON(比较器元数据校验用)。
        "trajectory_config": sc.traj_config,
        "vel_sat_per_joint": m.vel_sat_per_joint,
        "accel_sat_per_joint": m.accel_sat_per_joint,
    })
    metrics_path = out_dir / f"tuner_t0_metrics_{tag}_{stamp}.json"
    metrics_path.write_text(json.dumps(metrics, indent=2, ensure_ascii=False))
    print(f"[credibility gain={plant_gain}] steady rms={m.steady_pos_err_rms*1000:.2f}mm "
          f"p95={m.steady_pos_err_p95*1000:.2f}mm guard={m.guard_count} "
          f"vel_peak={m.vel_peak:.3f} accel_peak={m.accel_peak:.2f} "
          f"exec_gap_c={m.steady_exec_gap_cart_rms*1000:.2f}mm "
          f"solve_p99={m.p99_solve_time_ms:.2f}ms")
    print(f"  trace: {trace_path}\n  metrics: {metrics_path}")
    return metrics


# ======================================================================
# main
# ======================================================================
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="闭环 MPC 调参 (T0 语义对齐版)")
    parser.add_argument("--mode", choices=("unified", "separate"), default="unified")
    parser.add_argument("--scenario", default=None)
    parser.add_argument("--list-scenarios", action="store_true")
    parser.add_argument("--warmup-periods", type=float, default=2.0,
                        help="每场景预热周期数(不计入统计)")
    parser.add_argument("--steady-periods", type=float, default=4.0,
                        help="每场景稳态统计周期数")
    parser.add_argument("--top-k", type=int, default=10)
    parser.add_argument("--output-dir", type=Path,
                        default=Path(__file__).resolve().parent / "data")
    parser.add_argument("--skip-fine", action="store_true")
    parser.add_argument("--fine-top", type=int, default=5)
    parser.add_argument("--quiet-build", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--max-combinations", type=int, default=None)
    parser.add_argument("--credibility", action="store_true",
                        help="T0 可信度模式: 生产参数×生产场景最小闭环,导出 trace/指标,不搜索")
    parser.add_argument("--phase2a", action="store_true",
                        help="阶段2A轻量筛选: 生产 moving-base 场景，搜索位置/终端位置/"
                             "关节增量/增量变化权重，固定 horizon、约束、姿态权重")
    parser.add_argument("--second-order-tune", action="store_true",
                        help="二阶 NMPC 权重筛选:固定时域/速度/加速度/奇异性约束，"
                             "对六项权重做基线+单变量扫描")
    parser.add_argument(
        "--second-order-stable-contour", action="store_true",
        help="二阶筛选改用真实轮廓切线和法向优先评分，搜索 horizon/轮廓/"
             "滞后/笛卡尔速度权重；硬件速度与加速度约束保持不变")
    parser.add_argument("--second-order-base-period", type=float, default=None,
                        help="二阶筛选使用的基座周期；低速 A/B 对齐可传 4.0")
    parser.add_argument("--second-order-horizons", type=str, default="10",
                        help="逗号分隔 horizon 候选, 如 10,16,20")
    parser.add_argument("--second-order-dq-ff-compensation", type=float, default=1.0)
    parser.add_argument("--second-order-guard-rule", choices=("legacy", "expected_residual"),
                        default="legacy")
    parser.add_argument("--second-order-velocity-source", choices=("measured", "commanded"),
                        default="measured")
    parser.add_argument("--second-order-ee-period", type=float, default=None,
                        help="二阶筛选使用的末端图形周期；低速 A/B 对齐可传 8.0")
    parser.add_argument("--rate-compare", action="store_true",
                        help="阶段2C: 同一40ms预测时长下离线比较50Hz与100Hz，不搜索参数")
    parser.add_argument("--plant-gain", type=float, default=PLANT_FF_GAIN,
                        help="离线 plant 前馈增益(仅 credibility 对照用,不改生产默认)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.warmup_periods < 0 or args.steady_periods <= 0:
        raise ValueError("warmup/steady periods 非法")

    ctx = make_context(args.warmup_periods, args.steady_periods)

    if args.second_order_tune:
        second_order_tune(
            ctx, args.output_dir / "second_order_tuning",
            args.max_combinations, args.quiet_build,
            args.second_order_base_period, args.second_order_ee_period,
            horizons=tuple(int(x) for x in args.second_order_horizons.split(',')),
            dq_ff_compensation=args.second_order_dq_ff_compensation,
            guard_rule=args.second_order_guard_rule,
            so_velocity_source=args.second_order_velocity_source,
            plant_gain=args.plant_gain,
            stable_contour=args.second_order_stable_contour)
        return 0

    if args.rate_compare:
        rate_compare(ctx, args.output_dir / "phase2c", args.quiet_build)
        return 0

    if args.credibility:
        credibility_run(ctx, args.output_dir / "tuner_t0", args.plant_gain, args.quiet_build)
        return 0

    scenarios = build_scenarios(ctx)
    available = [s.name for s in scenarios]
    if args.list_scenarios:
        print("可选场景: " + ", ".join(available))
        return 0
    if args.phase2a:
        if args.scenario and args.scenario != "moving_base_circle":
            raise SystemExit("--phase2a 固定使用 moving_base_circle，不接受其他 --scenario")
        scenarios = [s for s in scenarios if s.name == "moving_base_circle"]
    elif args.scenario:
        wanted = [n.strip() for n in args.scenario.split(",") if n.strip()]
        bad = [n for n in wanted if n not in available]
        if bad:
            raise SystemExit(f"未知场景 {bad}; 可选: {available}")
        scenarios = [s for s in scenarios if s.name in wanted]

    print(f"模式: {args.mode} | 场景: {[s.name for s in scenarios]}")
    print(f"T0 语义: lead={REFERENCE_LEAD_STEPS}, dq_prev={DQ_PREV_MODE}, "
          f"rti={RTI_ITERATION_MODE}x{ACADOS_RTI_ITERATIONS}, guard={GUARD_STEP_WARN_M*1000:.0f}mm; "
          f"plant: gain={PLANT_FF_GAIN}, tau={PLANT_FB_TAU}, max_accel={PLANT_MAX_ACCEL}")
    print(f"统计: 预热 {args.warmup_periods} 周期 + 稳态 {args.steady_periods} 周期")

    if args.mode == "unified":
        sets = [("unified", list(scenarios))]
    else:
        sets = [(s.name, [s]) for s in scenarios]

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_tag = "phase2a_weights" if args.phase2a else f"unified_mpc_tuning_{args.mode}"
    csv_path = args.output_dir / f"{output_tag}_{stamp}.csv"

    all_results: List[TrialResult] = []
    summary: List[Tuple[str, Optional[TrialResult]]] = []
    for label, scs in sets:
        res = search(label, scs, ctx, args)
        all_results.extend(res)
        best = print_set(label, scs, res, args.top_k)
        summary.append((label, best))

    write_results(csv_path, all_results)
    print(f"\n调参结果已保存: {csv_path}")

    if args.mode == "separate":
        print("\n===== separate 模式各场景最优参数汇总 =====")
        for label, best in summary:
            if best is None:
                print(f"  {label:>18s}: 无可行参数")
            else:
                p = best.params
                print(f"  {label:>18s}: h={p.horizon_steps}, step={p.max_joint_step:.4f}, "
                      f"ow={p.orientation_weight:.4f}, tow={p.terminal_orientation_weight:.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
