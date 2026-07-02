#!/usr/bin/env python3
"""LL100-LH420 避障 NMPC 的离线闭环调参器(独立于独立臂的 tune_unified_mpc_params.py)。

不启动 ROS、不画图。复用 move_ll100_lh420_nmpc_demo 的同一套 MpcIkConfig + ll100_traj_pub
的路径模式,离线 roll 出**闭环**轨迹:被控对象复刻 LL100 底层 student_arm_node 的
SmoothPositionController(固定开启速度前馈),MPC 每拍从被控对象真实状态反馈重解。

**只调 NMPC 参数**(时域 + 代价权重),硬件/避障参数一律固定、不进搜索:
  被调(9 维):horizon_steps、position_weight、terminal_position_weight、
              orientation_weight、terminal_orientation_weight、
              joint_delta_weight、joint_delta_change_weight、
              current_posture_weight、center_posture_weight
  固定(不调):max_joint_step = max_velocity·control_dt(硬件速度约束的 OCP 镜像,派生量);
              避障 obstacle_safety_margin / body_obstacle_margin / capsule_radius /
              capsule_samples_per_segment(安全,固定);path_preview_stride(追踪层)。

9 维全网格积(3^9≈2万)不可行,且改权重会触发 acados 重新编译,故采用**坐标下降式
coarse→fine**:从基线出发,逐维扫描候选、采纳更优值(coarse),再在最优附近逐维局部细化(fine)。

调参目标(综合打分,优先级 安全 > 求解时间 > 精度 > 完成时间):
  1. 安全无碰撞   —— violation 在实际经过的状态上量,作硬门(<=0.1mm 且无 fallback)
  2. 无奇异姿态   —— 末端位置 Jacobian 可操作度 manipulability,软惩罚(越大越好)
  3. 求解速度快   —— mean_solve_time_ms,轻惩罚
  4. 末端误差小   —— rms / max position error
  5. 启动直达首点 —— 先 hold 到第一个期望点(到达容差内)再开始计时跟踪
  6. 跟踪快       —— EVAL_PERIOD_SEC/圈,被控对象按底层限速跟,误差反映能否跟上

用法:
    python3 tune_ll100_nmpc.py --path-mode avoidance_sweep
    python3 tune_ll100_nmpc.py --path-mode avoidance_sweep --max-combinations 8   # 快速 smoke
    python3 tune_ll100_nmpc.py --path-mode avoidance_sweep --skip-fine            # 只跑 coarse
    python3 tune_ll100_nmpc.py --path-mode avoidance_sweep --max-velocity 1.0     # 底层更快
需先 source ROS + 工作空间 + ACADOS。
"""

import argparse
import contextlib
import csv
import os
import time
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK


PUBLISH_RATE_HZ = 50.0
CONTROL_DT = 1.0 / PUBLISH_RATE_HZ

EE_FRAME = "F_Link"
LINK_FRAMES = ("A_Link", "B_Link", "C_Link", "D_Link", "E_Link", "F_Link")
EVAL_PERIOD_SEC = 20.0                # 一圈时长 s(完成时间;安全>求解>精度>完成时间,定稿 20s)
TERMINAL_TOLERANCE = 0.02
VIOLATION_TOL = 1.0e-4                # 缓冲侵入 <= 0.1mm 视为安全(数值/离散噪声)
DEFAULT_MAX_VELOCITY = 0.5            # 底层关节限速 rad/s(= ll100_lh420_control.launch.py 默认)

# 启动直达首点(需求 5):先把末端开到第一个期望点,进容差再开始计时跟踪。
START_REACH_TOL = 0.005              # 到达首点判定容差 m(5mm,和 max 目标一致;不再当安全硬门)
MAX_HOLD_SEC = 10.0                  # 最多给多少秒到达首点,超时算到不了

# 奇异检测(需求 2):末端位置 Jacobian 可操作度 sqrt(det(J Jᵀ)),越大越远离奇异。
MANIP_EPS = 1.0e-4
MANIP_SUBSAMPLE = 15                 # 每隔多少步算一次可操作度
SINGULAR_SOFT_TARGET = 0.012        # 低于该值开始惩罚(据实测 manip 量级标定,~0.004-0.012)

# SmoothPositionController velocity-feedforward 配置默认值(= VelocityFeedforwardConfig 默认)。
VFF_GAIN = 0.7
VFF_TAU = 0.15
VFF_FB_VLIM = 0.4
VFF_MAX_ACCEL = 5.0

# ---- 固定·不调(硬件/避障/追踪层)----
FIXED_CAPSULE_SAMPLES = 3             # 避障胶囊采样密度(每段点数=此值+2 → 5 点/段;安全,固定)
FIXED_PATH_PREVIEW_STRIDE = 4          # 预测时域内目标点间隔(追踪层,固定)
FIXED_OBSTACLE_SAFETY_MARGIN = 0.005   # 桨叶:小正余量,保证实际不接触(桨叶余量)
FIXED_BODY_OBSTACLE_MARGIN = 0.005     # 机身:小正余量,保证实际不接触(机身余量)
FIXED_CAPSULE_RADIUS = 0.018           # 分段半径留空时的回退标量(几何,非缓冲)
# 分段避障半径(关节粗/连杆细,来自各连杆网格绕真实杆轴的包络);与 demo DEFAULT_CAPSULE_*_RADII 一致。
FIXED_CAPSULE_JOINT_RADII = (0.052, 0.062, 0.058, 0.050, 0.049, 0.040)  # A,B,C,D,E,F 关节(电机)
FIXED_CAPSULE_LINK_RADII = (0.045, 0.035, 0.043, 0.016, 0.035)          # A→B,B→C,C→D,D→E,E→F 杆身
FIXED_FRONT_OBSTACLE_CAPSULE = True    # 前桨按 capsule 建模(含连杆粗细),与 demo 一致
FIXED_REPULSION_MARGIN = 0.03          # 斥力场阈值 m(默认打开:NMPC 主动留净空)
FIXED_REPULSION_WEIGHT = 1.0e4         # 斥力强度(peer 甜点;运行时 cost_set 可扫)

# ---- 被调(9 维 NMPC 参数)coarse 候选 + 基线 ----
# 坐标下降从基线出发,逐维在候选里找更优;基线 = 当前 demo 定稿默认。
BASELINE = None  # 见下方(ParamSet 定义后赋值)
COARSE: Dict[str, List[float]] = {
    "horizon_steps": [6, 8, 12],
    "position_weight": [80.0, 120.0, 180.0],
    "terminal_position_weight": [800.0, 1200.0, 2000.0],
    "orientation_weight": [0.15, 0.3, 0.6],
    "terminal_orientation_weight": [1.5, 3.0, 6.0],
    "joint_delta_weight": [0.04, 0.08, 0.16],
    "joint_delta_change_weight": [0.12, 0.25, 0.5],
    "current_posture_weight": [0.002, 0.005, 0.01],
    "center_posture_weight": [0.005, 0.01, 0.03],
}
FINE_SCALE = [0.7, 0.85, 1.0, 1.2, 1.4]   # 权重维在最优附近的相对缩放
FINE_HORIZON_DELTA = [-2, 0, 2]           # horizon_steps 在最优附近的增量


def rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(values * values)))


def vertical_down_rotation() -> np.ndarray:
    return np.array(
        [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]], dtype=float
    )


@contextlib.contextmanager
def suppress_native_output(enabled: bool):
    if not enabled:
        yield
        return
    stdout_fd = os.dup(1)
    stderr_fd = os.dup(2)
    try:
        with open(os.devnull, "w") as devnull:
            os.dup2(devnull.fileno(), 1)
            os.dup2(devnull.fileno(), 2)
            yield
    finally:
        os.dup2(stdout_fd, 1)
        os.dup2(stderr_fd, 2)
        os.close(stdout_fd)
        os.close(stderr_fd)


def manipulability(mpc, q: np.ndarray) -> float:
    """末端位置 Jacobian 的可操作度 sqrt(det(J Jᵀ)),有限差分,越大越远离奇异。"""
    p0, _ = mpc.forward(q)
    n = len(q)
    jac = np.zeros((3, n), dtype=float)
    for i in range(n):
        dq = q.copy()
        dq[i] += MANIP_EPS
        pi, _ = mpc.forward(dq)
        jac[:, i] = (pi - p0) / MANIP_EPS
    return float(np.sqrt(max(np.linalg.det(jac @ jac.T), 0.0)))


@dataclass(frozen=True, order=True)
class ParamSet:
    horizon_steps: int
    position_weight: float
    terminal_position_weight: float
    orientation_weight: float
    terminal_orientation_weight: float
    joint_delta_weight: float
    joint_delta_change_weight: float
    current_posture_weight: float
    center_posture_weight: float


BASELINE = ParamSet(
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


@dataclass
class TrialResult:
    params: ParamSet
    path_mode: str
    safe: bool
    score: float
    violation_max: float
    min_margin: float
    min_manipulability: float
    reached_first: bool
    reach_time_sec: float
    fallback_count: int
    solver_failure_count: int
    rms_position_error: float
    max_position_error: float
    mean_solve_time_ms: float
    samples: int
    recommended_command: str


def _dir_tag(p: ParamSet, max_joint_step: float) -> str:
    # 只按“结构量”命名:同结构(horizon + 步长边界)的不同权重组合共用同一编译目录。
    # 权重已由求解器运行时 cost_set 写入(不进签名、不重编,见 MPC 后端 Q2),故 9 维里
    # 那 8 个权重维在同一 horizon 下全部复用已编译求解器;只有 horizon_steps 变才重编。
    return f"h{p.horizon_steps}_s{int(round(max_joint_step * 1000))}"


def build_config(p: ParamSet, backend: str, max_velocity: float) -> Tuple[MpcIkConfig, str]:
    from mpc_pinocchio_ik import DEFAULT_LL100_URDF
    from move_ll100_lh420_nmpc_demo import (
        DEFAULT_LL100_BODY_OBSTACLE_BOXES,
        LL100_NMPC_URDF,
        CAPSULE_CENTERLINE_POINTS,
    )

    # max_joint_step 是硬件速度约束的 OCP 镜像,派生量、不搜索:= max_velocity·control_dt。
    max_joint_step = max_velocity * CONTROL_DT
    # 求解器与路径无关(路径只是在线目标参数),故 build_dir 不含 path_mode,各模式共用。
    # 权重进 acados 签名 → 改权重会重新编译,故 build_dir 按整组参数唯一命名。
    build_dir = f"/tmp/windylab_acados_tune_ll100_{_dir_tag(p, max_joint_step)}"
    cfg = MpcIkConfig(
        horizon_steps=p.horizon_steps,
        control_dt=CONTROL_DT,
        max_joint_step=max_joint_step,
        success_tolerance=TERMINAL_TOLERANCE,
        solver_backend=backend,
        acados_build_dir=build_dir,
        position_weight=p.position_weight,
        terminal_position_weight=p.terminal_position_weight,
        orientation_weight=p.orientation_weight,
        terminal_orientation_weight=p.terminal_orientation_weight,
        joint_delta_weight=p.joint_delta_weight,
        joint_delta_change_weight=p.joint_delta_change_weight,
        obstacle_avoidance_enabled=True,
        obstacle_safety_margin=FIXED_OBSTACLE_SAFETY_MARGIN,
        obstacle_disk_half_thickness=0.02,
        obstacle_ll100_urdf_path=DEFAULT_LL100_URDF,
        obstacle_coordinate_mode="ll100_base",
        obstacle_keypoint_frames=LINK_FRAMES,
        body_obstacle_enabled=True,
        body_obstacle_margin=FIXED_BODY_OBSTACLE_MARGIN,
        body_obstacle_boxes=DEFAULT_LL100_BODY_OBSTACLE_BOXES,
        capsule_radius=FIXED_CAPSULE_RADIUS,
        capsule_joint_radii=FIXED_CAPSULE_JOINT_RADII,
        capsule_link_radii=FIXED_CAPSULE_LINK_RADII,
        capsule_points=CAPSULE_CENTERLINE_POINTS,
        capsule_samples_per_segment=FIXED_CAPSULE_SAMPLES,
        front_obstacle_capsule=FIXED_FRONT_OBSTACLE_CAPSULE,
        obstacle_repulsion_margin=FIXED_REPULSION_MARGIN,
        obstacle_repulsion_weight=FIXED_REPULSION_WEIGHT,
        capsule_frame_sequence=LINK_FRAMES,
        current_posture_weight=p.current_posture_weight,
        center_posture_weight=p.center_posture_weight,
    )
    return cfg, LL100_NMPC_URDF


def score_of(viol_max, fallback, solver_fail, rms_pos, max_pos, mean_ms,
             min_manip, reached_first) -> Tuple[float, bool]:
    # 硬门:安全无碰撞 + 无 fallback。不满足直接拉到很后面。
    safe = (viol_max <= VIOLATION_TOL) and (fallback == 0)  # 仅碰撞+可行性;reached_first 作质量项
    score = (
        (0.0 if safe else 1.0e5)
        + 1.0e6 * viol_max                                   # 1 安全(最高优先级)
        + 500.0 * fallback
        + 500.0 * solver_fail
        + 0.05 * mean_ms                                     # 3 求解速度(优先于精度/完成时间)
        + 1000.0 * rms_pos + 1000.0 * max_pos               # 4/6 末端误差(含跟踪滞后)
        + (2.0e4 if not reached_first else 0.0)              # 5 启动直达首点
        + 300.0 * max(0.0, SINGULAR_SOFT_TARGET - min_manip)  # 2 远离奇异(软)
    )
    return score, safe


def recommended_command(p: ParamSet, path_mode: str, max_velocity: float, period_sec: float) -> str:
    max_joint_step = max_velocity * CONTROL_DT
    return (
        "python3 move_ll100_lh420_nmpc_demo.py --ros-args "
        f"-p path_mode:={path_mode} -p period_sec:={period_sec:.1f} "
        f"-p horizon_steps:={p.horizon_steps} "
        f"-p max_joint_step:={max_joint_step:.4f} "
        f"-p position_weight:={p.position_weight:.1f} "
        f"-p terminal_position_weight:={p.terminal_position_weight:.1f} "
        f"-p orientation_weight:={p.orientation_weight:.3f} "
        f"-p terminal_orientation_weight:={p.terminal_orientation_weight:.3f} "
        f"-p joint_delta_weight:={p.joint_delta_weight:.4f} "
        f"-p joint_delta_change_weight:={p.joint_delta_change_weight:.4f} "
        f"-p current_posture_weight:={p.current_posture_weight:.4f} "
        f"-p center_posture_weight:={p.center_posture_weight:.4f} "
        "-p hold_start_until_reached:=true -p start_from_nearest_path_point:=true "
        "-p target_switch_tolerance:=0.005 "
        f"(底层 student_arm max_velocity:={max_velocity:.2f})"
    )


def _plant_step(q_plant, q_cmd, qdot_ref, last_v, max_velocity, q_lower, q_upper):
    """复刻 SmoothPositionController 速度前馈:返回 (q_new, last_v_new)。"""
    v_ff = np.clip(qdot_ref, -max_velocity, max_velocity)
    v_fb = np.clip((q_cmd - q_plant) / VFF_TAU, -VFF_FB_VLIM, VFF_FB_VLIM)
    v_raw = np.clip(VFF_GAIN * v_ff + v_fb, -max_velocity, max_velocity)
    accel = VFF_MAX_ACCEL * CONTROL_DT
    v_cmd = np.clip(v_raw, last_v - accel, last_v + accel)
    q_new = np.clip(q_plant + v_cmd * CONTROL_DT, q_lower, q_upper)
    return q_new, (q_new - q_plant) / CONTROL_DT


def evaluate(p: ParamSet, path_mode: str, backend: str,
             periods: float, quiet_build: bool, max_velocity: float,
             period_sec: float = EVAL_PERIOD_SEC) -> TrialResult:
    import pinocchio as pin
    from ll100_traj_pub import _build_path as build_ll100_path

    cfg, urdf = build_config(p, backend, max_velocity)
    with suppress_native_output(quiet_build):
        mpc = MpcPinocchioIK(urdf_path=urdf, ee_frame=EE_FRAME, config=cfg)

    count = max(20, int(round(period_sec / CONTROL_DT)))
    path = build_ll100_path(path_mode, count)
    locked_rotation = vertical_down_rotation()
    horizon = cfg.horizon_steps
    offsets = FIXED_PATH_PREVIEW_STRIDE * np.arange(horizon)

    q_plant = np.clip(pin.neutral(mpc.model), mpc.q_lower, mpc.q_upper)
    last_v = np.zeros_like(q_plant)
    warm_start = None

    actual_pos, _ = mpc.forward(q_plant)
    nearest = int(np.argmin(np.linalg.norm(path - actual_pos.reshape(1, 3), axis=1)))
    first_pt = path[nearest]

    fallback = 0
    solver_fail = 0

    def tick(targets):
        nonlocal q_plant, last_v, warm_start, fallback, solver_fail
        t0 = time.perf_counter()
        q_next, ok, info = mpc.step(targets, q_plant, warm_start=warm_start, target_rotation=locked_rotation)
        dt_ms = 1000.0 * float(info.get("solve_time", time.perf_counter() - t0))
        warm_start = info["trajectory"]
        if int(info.get("status", 0)) != 0 or not bool(info.get("success", True)):
            solver_fail += 1
        if bool(info.get("command_usable", ok)):
            q_cmd = q_next
            qdot_ref = (q_next - q_plant) / CONTROL_DT
        else:
            fallback += 1
            q_cmd = q_plant.copy()
            qdot_ref = np.zeros_like(q_plant)
        q_plant, last_v = _plant_step(q_plant, q_cmd, qdot_ref, last_v,
                                      max_velocity, mpc.q_lower, mpc.q_upper)
        return dt_ms

    # --- 阶段 1:启动直达第一个期望点(需求 5)---
    reached_first = False
    reach_steps = 0
    max_hold_steps = int(round(MAX_HOLD_SEC / CONTROL_DT))
    hold_targets = np.repeat(first_pt.reshape(1, 3), horizon, axis=0)
    for _ in range(max_hold_steps):
        tick(hold_targets)
        reach_steps += 1
        cur, _ = mpc.forward(q_plant)
        if float(np.linalg.norm(cur - first_pt)) <= START_REACH_TOL:
            reached_first = True
            break
    reach_time_sec = reach_steps * CONTROL_DT
    # 启动阶段的 fallback/solver_fail 不计入跟踪统计(只看是否到得了)
    fallback = 0
    solver_fail = 0

    # --- 阶段 2:计时跟踪一圈 ---
    steps = max(1, int(round(periods * period_sec / CONTROL_DT)))
    pos_errs: List[float] = []
    solve_ms: List[float] = []
    realized: List[np.ndarray] = []
    manips: List[float] = []
    for step in range(steps):
        base = (nearest + step) % len(path)
        dt_ms = tick(path[(base + offsets) % len(path)])
        solve_ms.append(dt_ms)
        realized.append(q_plant.copy())
        cur, _ = mpc.forward(q_plant)
        pos_errs.append(float(np.linalg.norm(cur - path[base])))
        if step % MANIP_SUBSAMPLE == 0:
            manips.append(manipulability(mpc, q_plant))

    if realized:
        diag = mpc._obstacle_diagnostics(np.asarray(realized, dtype=float))
        viol_max = float(diag.get("obstacle_violation_max") or 0.0)
        mm = diag.get("min_obstacle_margin")
        min_margin = float(mm) if mm is not None else 0.0
    else:
        viol_max = 0.0
        min_margin = 0.0

    pos = np.asarray(pos_errs, dtype=float) if pos_errs else np.zeros(1)
    rms_pos = rms(pos)
    max_pos = float(np.max(pos))
    mean_ms = float(np.mean(solve_ms)) if solve_ms else 0.0
    min_manip = float(min(manips)) if manips else 0.0
    score, safe = score_of(viol_max, fallback, solver_fail, rms_pos, max_pos,
                            mean_ms, min_manip, reached_first)
    return TrialResult(
        params=p, path_mode=path_mode, safe=safe, score=score,
        violation_max=viol_max, min_margin=min_margin,
        min_manipulability=min_manip, reached_first=reached_first,
        reach_time_sec=reach_time_sec,
        fallback_count=fallback, solver_failure_count=solver_fail,
        rms_position_error=rms_pos, max_position_error=max_pos,
        mean_solve_time_ms=mean_ms, samples=len(pos_errs),
        recommended_command=recommended_command(p, path_mode, max_velocity, period_sec),
    )


def _fine_values(name: str, base_val) -> List:
    """在 coarse 最优附近的局部细化候选。"""
    if name == "horizon_steps":
        return sorted({max(1, int(base_val) + d) for d in FINE_HORIZON_DELTA})
    return sorted({round(float(base_val) * s, 6) for s in FINE_SCALE})


def run(args: argparse.Namespace) -> int:
    cache: Dict[ParamSet, TrialResult] = {}
    order = list(COARSE.keys())
    budget = args.max_combinations

    def budget_left() -> bool:
        return budget is None or budget <= 0 or len(cache) < budget

    def evaluate_cached(p: ParamSet) -> TrialResult:
        if p not in cache:
            r = evaluate(p, args.path_mode, args.backend, args.periods, args.quiet_build,
                         args.max_velocity, args.period)
            cache[p] = r
            print(
                f"    [{len(cache)}] safe={r.safe} score={r.score:.2f} viol={r.violation_max:.6f} "
                f"manip={r.min_manipulability:.4f} reached={r.reached_first} fb={r.fallback_count} "
                f"rms={r.rms_position_error:.4f} max={r.max_position_error:.4f} "
                f"ms={r.mean_solve_time_ms:.2f} | {p}",
                flush=True,
            )
        return cache[p]

    print(f"[ll100 避障 NMPC 调参] path_mode={args.path_mode} 周期={args.period}s "
          f"max_velocity={args.max_velocity} (max_joint_step 派生={args.max_velocity * CONTROL_DT:.4f}) "
          f"坐标下降 coarse{'' if args.skip_fine else '→fine'} 预算={budget}", flush=True)

    best = BASELINE
    try:
        evaluate_cached(best)
    except Exception as exc:  # noqa: BLE001
        print(f"基线评估失败: {exc}")
        return 1

    # --- coarse:坐标下降,逐维扫描 ---
    for pass_idx in range(args.coarse_passes):
        improved = False
        print(f"  -- coarse pass {pass_idx + 1}/{args.coarse_passes} --", flush=True)
        for name in order:
            base_val = getattr(best, name)
            for val in COARSE[name]:
                if not budget_left():
                    break
                if val == base_val:
                    continue
                try:
                    r = evaluate_cached(replace(best, **{name: val}))
                except Exception as exc:  # noqa: BLE001
                    print(f"    跳过(异常): {exc}", flush=True)
                    continue
                if r.score < cache[best].score:
                    best = r.params
                    improved = True
            if not budget_left():
                break
        if not improved or not budget_left():
            break

    # --- fine:在最优附近逐维局部细化 ---
    if not args.skip_fine:
        print("  -- fine 细化 --", flush=True)
        for name in order:
            base_val = getattr(best, name)
            for val in _fine_values(name, base_val):
                if not budget_left():
                    break
                if val == base_val:
                    continue
                try:
                    r = evaluate_cached(replace(best, **{name: val}))
                except Exception as exc:  # noqa: BLE001
                    print(f"    跳过(异常): {exc}", flush=True)
                    continue
                if r.score < cache[best].score:
                    best = r.params
            if not budget_left():
                break

    results = sorted(cache.values(), key=lambda r: r.score)
    if not results:
        print("没有可用结果(检查 path_mode 是否为 ll100_traj_pub 支持的模式)")
        return 1

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = args.output_dir / f"tune_ll100_{args.path_mode}_{stamp}.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "rank", "safe", "score", "path_mode",
            "horizon_steps", "position_weight", "terminal_position_weight",
            "orientation_weight", "terminal_orientation_weight",
            "joint_delta_weight", "joint_delta_change_weight",
            "current_posture_weight", "center_posture_weight",
            "violation_max", "min_margin", "min_manipulability", "reached_first",
            "reach_time_sec", "fallback_count", "solver_failure_count",
            "rms_position_error", "max_position_error", "mean_solve_time_ms", "samples",
            "recommended_command",
        ])
        for rank, r in enumerate(results, 1):
            q = r.params
            writer.writerow([
                rank, r.safe, f"{r.score:.4f}", r.path_mode,
                q.horizon_steps, f"{q.position_weight:.3f}", f"{q.terminal_position_weight:.3f}",
                f"{q.orientation_weight:.4f}", f"{q.terminal_orientation_weight:.4f}",
                f"{q.joint_delta_weight:.4f}", f"{q.joint_delta_change_weight:.4f}",
                f"{q.current_posture_weight:.4f}", f"{q.center_posture_weight:.4f}",
                f"{r.violation_max:.8f}", f"{r.min_margin:.6f}", f"{r.min_manipulability:.6f}",
                r.reached_first, f"{r.reach_time_sec:.3f}", r.fallback_count,
                r.solver_failure_count, f"{r.rms_position_error:.6f}",
                f"{r.max_position_error:.6f}", f"{r.mean_solve_time_ms:.3f}", r.samples,
                r.recommended_command,
            ])
    print(f"\nCSV: {csv_path}  (共评估 {len(results)} 组)", flush=True)
    print(f"\nTop {min(args.top_k, len(results))}(先看 safe,再看 rms/max / manip):")
    for rank, r in enumerate(results[: args.top_k], 1):
        q = r.params
        print(
            f"  #{rank} safe={r.safe} viol={r.violation_max:.6f} manip={r.min_manipulability:.4f} "
            f"reached={r.reached_first} rms={r.rms_position_error:.4f} "
            f"max={r.max_position_error:.4f} ms={r.mean_solve_time_ms:.2f} | "
            f"h={q.horizon_steps} pw={q.position_weight:.0f} tpw={q.terminal_position_weight:.0f} "
            f"ow={q.orientation_weight:.2f} tow={q.terminal_orientation_weight:.2f} "
            f"jd={q.joint_delta_weight:.3f} jdc={q.joint_delta_change_weight:.3f} "
            f"cur={q.current_posture_weight:.3f} cen={q.center_posture_weight:.3f}"
        )
    best_result = results[0]
    print(f"\n推荐(score 最低):\n{best_result.recommended_command}")
    if not best_result.safe:
        print("⚠️ 最优候选未过安全门(碰撞 / fallback);可能需要提高 "
              "--max-velocity(周期内底层跟不上)或改路径几何。")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="LL100 避障 NMPC 离线闭环调参(只调 NMPC 参数,坐标下降 coarse→fine)")
    parser.add_argument("--path-mode", default="avoidance_sweep",
                        help="路径模式(ll100_traj_pub 支持的,如 avoidance_sweep)")
    parser.add_argument("--max-velocity", type=float, default=DEFAULT_MAX_VELOCITY,
                        help="闭环 plant 的底层关节限速 rad/s(应与 student_arm max_velocity 一致;"
                             "max_joint_step 由它派生 = max_velocity·control_dt)")
    parser.add_argument("--backend", choices=("acados", "casadi", "scipy"), default="acados")
    parser.add_argument("--period", type=float, default=EVAL_PERIOD_SEC,
                        help="一圈时长 s(完成时间;越长越精准)")
    parser.add_argument("--periods", type=float, default=1.0, help="计时跟踪的圈数")
    parser.add_argument("--coarse-passes", type=int, default=2,
                        help="coarse 坐标下降扫描遍数(每遍逐维扫描一次)")
    parser.add_argument("--skip-fine", action="store_true", help="只跑 coarse,不做局部细化")
    parser.add_argument("--top-k", type=int, default=10, help="终端打印前 K 名")
    parser.add_argument("--output-dir", type=Path,
                        default=Path("/home/windylab/code/windylab_ws/src/arm-platform/demo/data"))
    parser.add_argument("--quiet-build", action=argparse.BooleanOptionalAction, default=True,
                        help="是否静音 ACADOS 代码生成/编译输出,默认静音")
    parser.add_argument("--max-combinations", type=int, default=None,
                        help="限制最多评估多少组参数;适合快速 smoke test")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.periods <= 0.0:
        raise ValueError("--periods 必须大于 0")
    if args.top_k < 1:
        raise ValueError("--top-k 必须大于等于 1")
    if args.coarse_passes < 1:
        raise ValueError("--coarse-passes 必须大于等于 1")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
