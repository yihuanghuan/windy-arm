#!/usr/bin/env python3
"""固定基座臂 + 浮动基座臂的闭环离线调参（不含 LL100）。

两种模式（`--mode`）:
  - unified  : 搜出**一套统一参数**，同时满足所有场景（默认）。
  - separate : **每个场景单独调参**，各自得到自己的最优参数。

闭环建模: 复刻底层 SmoothPositionController 的速度前馈控制器（角速度控制默认开启后的唯一
模式）做 plant，把 MPC 命令喂给它，用 plant 真实 q_actual 作为下一拍 MPC 求解起点，误差也在
q_actual 上统计。硬约束: 无 fallback / 无求解失败 / 无奇异位姿 / 求解快。

================================ 扩展点（怎么改） ================================
① 增加一个运动场景:
   在 build_scenarios() 的 SCENARIO 列表里加一条 Scenario(...)。需要提供:
     name            场景名(也是 separate 模式的 CSV/输出标识)
     demo            该场景对应的 demo 文件名(用于打印可直接运行的推荐命令)
     period_sec      轨迹一圈/一程的周期(秒)，roll-out 时长 = periods * period_sec
     lock_orientation 是否锁末端姿态(锁则用 orientation_weight/terminal_orientation_weight)
     position_fn(t)  -> (3,) 目标末端位置(base 系)
     rotation_fn(t)  -> (3,3) 目标姿态; 不锁姿态填 None; 想"锁起点姿态"填 None 且
                       lock_orientation=True(会自动锁定起点 IK 解的姿态)
     demo_extra_args 推荐命令里额外的 -p 参数(如 ["trajectory_mode:=circle"])
   轨迹函数写成模块级普通函数即可(参考 _vertical_circle / _moving_base_position)。
   不需要改其它任何地方; unified/separate 都会自动带上新场景。

② 换一个求解算法(把 MPC 换成别的 IK/规划器):
   只改 build_solver(params, lock_orientation) 这一个工厂函数, 让它返回你的求解器对象。
   该对象只需实现 step() 合约(见 build_solver 文档):
     step(targets(H,3), q_current(N,), warm_start, target_rotation) -> (q_next(N,), ok, info)
   评估用的正运动学/雅可比/奇异性都走独立的 pinocchio 模型(Context.kin), 与求解器内部无关,
   所以换算法不必再提供 FK。若新算法的可调参数不同, 同时改 ParamSet 与 coarse/fine 候选即可。
================================================================================
"""

import argparse
import contextlib
import csv
import itertools
import math
import os
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import pinocchio as pin

import moving_base_trajectory_config as mbtraj
from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK
from pinocchio_ik import DEFAULT_URDF, EE_FRAME, PinocchioIK


# ----- 控制周期 / 闭环 plant -----
PUBLISH_RATE_HZ = 50.0
CONTROL_DT = 1.0 / PUBLISH_RATE_HZ          # MPC 发布周期 (s)
CTRL_SUBSTEP_DT = 0.01                       # 底层控制器周期 10ms
N_SUBSTEPS = max(1, int(round(CONTROL_DT / CTRL_SUBSTEP_DT)))

# 与 student_arm.launch.py 速度前馈默认值一致 (角速度控制恒开)。
PLANT_MAX_VELOCITY = 1.0
PLANT_FF_GAIN = 0.7
PLANT_FB_TAU = 0.15
PLANT_FB_VEL_LIMIT = 0.4
PLANT_MAX_ACCEL = 5.0
PLANT_ACCEL_LIMIT = True

# ----- 评估阈值 -----
TERMINAL_TOLERANCE = 0.01
# 奇异性用"位置(平移)雅可比" 3xN 的最小奇异值衡量, 即末端定位能力。
# 不用全 6xN: 这些 demo 轨迹在锁姿态时本就靠近腕部/姿态奇异(全 6D sigma~1e-3,
# 理想 IK 跟随也无法避免), 但平移雅可比始终健康(~0.15)。位置奇异才是会导致关节
# 速度发散、定位失效的"奇异位姿", 这正是要规避的。健康值约 0.14~0.17。
SINGULAR_SIGMA_THRESHOLD = 0.05
MAX_SOLVE_TIME_MS_HARD = 15.0     # 求解时间硬上限 (50Hz 预算 20ms), 按 p95 判定
SOLVE_TIME_WARMUP_STEPS = 5       # 前几步含冷启动(加载/首次 RTI), 不计入求解时间统计

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
# 评估上下文: 独立的 pinocchio 运动学(供 FK/雅可比/奇异性), 与求解器内部无关
# ======================================================================
@dataclass
class Context:
    model: object
    data: object
    frame_id: int
    q_lower: np.ndarray
    q_upper: np.ndarray
    periods: float


def make_context(periods: float) -> Context:
    model = pin.buildModelFromUrdf(DEFAULT_URDF)
    data = model.createData()
    frame_id = model.getFrameId(EE_FRAME)
    return Context(model, data, frame_id,
                   model.lowerPositionLimit.copy(), model.upperPositionLimit.copy(), periods)


def fk(ctx: Context, q: np.ndarray):
    pin.framesForwardKinematics(ctx.model, ctx.data, q)
    oMf = ctx.data.oMf[ctx.frame_id]
    return oMf.translation.copy(), oMf.rotation.copy()


def orientation_error(current_rotation: np.ndarray, target_rotation: np.ndarray) -> np.ndarray:
    """与 MpcPinocchioIK._orientation_error 一致的 skew/vee 姿态误差。"""
    skew = target_rotation @ current_rotation.T - current_rotation @ target_rotation.T
    return 0.5 * np.array([skew[2, 1], skew[0, 2], skew[1, 0]], dtype=float)


def min_singular_value(ctx: Context, q: np.ndarray) -> float:
    """末端定位能力: 平移雅可比 (3xN) 的最小奇异值。"""
    J = pin.computeFrameJacobian(ctx.model, ctx.data, q, ctx.frame_id,
                                 pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    sv = np.linalg.svd(J[:3, :], compute_uv=False)
    return float(np.min(sv))


# ======================================================================
# 闭环 plant: 速度前馈控制器 (smooth_position_controller.cc 前馈分支)
# ======================================================================
class FeedforwardPlant:
    """逐关节、限速 + 位置反馈 + 加速度限制的位置跟踪器。"""

    def __init__(self, q0: np.ndarray, q_lower: np.ndarray, q_upper: np.ndarray):
        self.q = q0.astype(float).copy()
        self.v_last = np.zeros_like(self.q)
        self.q_lower = q_lower
        self.q_upper = q_upper

    def advance(self, q_cmd: np.ndarray, dq_ff: np.ndarray) -> np.ndarray:
        for _ in range(N_SUBSTEPS):
            v_ff = np.clip(dq_ff, -PLANT_MAX_VELOCITY, PLANT_MAX_VELOCITY)
            v_fb = np.clip((q_cmd - self.q) / PLANT_FB_TAU,
                           -PLANT_FB_VEL_LIMIT, PLANT_FB_VEL_LIMIT)
            v_raw = np.clip(PLANT_FF_GAIN * v_ff + v_fb,
                            -PLANT_MAX_VELOCITY, PLANT_MAX_VELOCITY)
            if PLANT_ACCEL_LIMIT:
                step = PLANT_MAX_ACCEL * CTRL_SUBSTEP_DT
                v_cmd = np.clip(v_raw, self.v_last - step, self.v_last + step)
            else:
                v_cmd = v_raw
            q_new = np.clip(self.q + v_cmd * CTRL_SUBSTEP_DT, self.q_lower, self.q_upper)
            v_cmd = (q_new - self.q) / CTRL_SUBSTEP_DT  # 限位后重算实际速度
            self.q = q_new
            self.v_last = v_cmd
        return self.q.copy()


# ======================================================================
# 场景定义 —— 扩展点① 见文件顶部
# ======================================================================
@dataclass
class Scenario:
    name: str
    demo: str                     # 推荐命令对应的 demo 文件
    period_sec: float
    lock_orientation: bool
    position_fn: Callable[[float], np.ndarray]
    rotation_fn: Optional[Callable[[float], np.ndarray]]  # None 表示不锁姿态
    demo_extra_args: List[str] = field(default_factory=list)  # 推荐命令额外 -p 参数
    q_seed: np.ndarray = field(default=None)              # 轨迹起点 IK 种子(自动算)


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


def _moving_base_position(t: float) -> np.ndarray:
    base_pos = mbtraj.base_pose(t)
    base_rot = mbtraj.base_rotation(t)
    ee_world = mbtraj.ee_target_world(t)
    return base_rot.T @ (ee_world - base_pos)


def _moving_base_rotation(t: float) -> np.ndarray:
    base_rot = mbtraj.base_rotation(t)
    return base_rot.T @ mbtraj.target_rotation_world()


def build_scenarios(ctx: Context) -> List[Scenario]:
    """构造场景并预解算各自起点 IK 种子 (与参数无关, 只算一次)。

    >>> 扩展点①: 在下面 SCENARIO 列表里增删场景即可。<<<
    """
    SCENARIO = [
        Scenario("circle", "move_arm_mpc_ik_demo.py", FIXED_CIRCLE_PERIOD,
                 False, _vertical_circle, None, ["trajectory_mode:=circle"]),
        Scenario("horizontal_circle", "move_arm_horizontal_circle_mpc_ik_demo.py",
                 FIXED_HCIRCLE_PERIOD, True, _horizontal_circle, None, ["lock_orientation:=true"]),
        Scenario("line", "move_arm_mpc_ik_demo.py", LINE_PERIOD,
                 False, _line, None, ["trajectory_mode:=line"]),
        Scenario("moving_base_circle", "move_base_circle_mpc_ik_demo.py",
                 mbtraj.EE_PERIOD_SEC, True, _moving_base_position, _moving_base_rotation, []),
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
        # lock_orientation 但没给 rotation_fn: 锁定起点姿态 (orientation_target=initial)
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


@dataclass
class ScenarioMetrics:
    name: str
    rms_position_error: float
    max_position_error: float
    rms_orientation_error: float
    max_orientation_error: float
    rms_control_delta: float
    min_sigma: float
    singular_count: int
    tracking_bad_count: int
    fallback_count: int
    solver_failure_count: int
    mean_solve_time_ms: float
    p95_solve_time_ms: float
    max_solve_time_ms: float
    score: float


@dataclass
class TrialResult:
    scenario_set: str             # "unified" 或单场景名 (separate 模式)
    phase: str
    params: ParamSet
    aggregate_score: float
    feasible: bool
    reject_reason: str
    scenarios: List[ScenarioMetrics]


# ======================================================================
# 求解器工厂 —— 扩展点② 见文件顶部
# ======================================================================
def build_solver(params: ParamSet, lock_orientation: bool):
    """返回一个滚动 MPC/IK 求解器对象。换算法只改这里。

    返回对象必须实现:
        step(targets, q_current, warm_start=None, target_rotation=None) -> (q_next, ok, info)
      targets: (horizon, 3) 目标位置序列(base 系)
      q_current: (N,) 当前关节角(闭环下为实测反馈)
      target_rotation: None / (3,3) / (horizon,3,3)
      返回: q_next (N,) 下一拍关节命令; ok (bool); info (dict)
      info 需含(尽量): "trajectory"(下一拍 warm start), "solve_time"(秒, 可选),
                       "status"(int, 0=正常), "success"(bool),
                       "command_usable"(bool), "tracking_ok"(bool)。
    评估侧的 FK/雅可比/奇异性走 Context 的独立 pinocchio 模型, 不依赖该对象。
    """
    return MpcPinocchioIK(config=_build_config(params, lock_orientation))


def _build_config(params: ParamSet, lock_orientation: bool) -> MpcIkConfig:
    return MpcIkConfig(
        horizon_steps=params.horizon_steps,
        control_dt=CONTROL_DT,
        max_joint_step=params.max_joint_step,
        success_tolerance=TERMINAL_TOLERANCE,
        solver_backend="acados",
        orientation_weight=params.orientation_weight if lock_orientation else 0.0,
        terminal_orientation_weight=params.terminal_orientation_weight if lock_orientation else 0.0,
        acados_build_dir=str(_build_dir(params)),
    )


def _build_dir(params: ParamSet) -> Path:
    # mpc_pinocchio_ik 已把代价权重移出 acados 签名(改成运行时 cost_set 设置),同一
    # (horizon_steps, max_joint_step) 下不管锁不锁姿态、ow/tow 多少,求解器结构/签名都相同,
    # 可复用同一份编译产物。因此编译目录只按 (h, step) 区分:锁姿态场景与不锁场景共用一个,
    # 权重差异由各自 MpcPinocchioIK 实例在运行时套用。
    step_mm = int(round(params.max_joint_step * 1000.0))
    return BUILD_ROOT / f"h{params.horizon_steps}_s{step_mm}"


# ======================================================================
# 评估
# ======================================================================
def rms(values: np.ndarray) -> float:
    if values.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(values * values)))


def evaluate_scenario(scenario: Scenario, solver, ctx: Context, horizon: int) -> ScenarioMetrics:
    q_seed = scenario.q_seed.copy()
    plant = FeedforwardPlant(q_seed, ctx.q_lower, ctx.q_upper)
    lock = scenario.lock_orientation and scenario.rotation_fn is not None

    warm_start = None
    last_q_cmd = q_seed.copy()
    prev_q_actual = q_seed.copy()

    pos_errors, ori_errors, ctrl_deltas, solve_times, sigmas = [], [], [], [], []
    tracking_bad = fallback = solver_fail = singular = 0

    steps = max(1, int(round(ctx.periods * scenario.period_sec / CONTROL_DT)))
    for k in range(steps):
        t = k * CONTROL_DT
        q_current = plant.q.copy()

        times = t + CONTROL_DT * np.arange(horizon)
        targets = np.array([scenario.position_fn(float(ti)) for ti in times])
        rotations = np.array([scenario.rotation_fn(float(ti)) for ti in times]) if lock else None

        start = time.perf_counter()
        q_next, ok, info = solver.step(targets, q_current,
                                       warm_start=warm_start, target_rotation=rotations)
        wall = time.perf_counter() - start
        if k >= SOLVE_TIME_WARMUP_STEPS:   # 跳过冷启动步, 只统计稳态求解时间
            solve_times.append(1000.0 * float(info.get("solve_time", wall)))
        warm_start = info.get("trajectory")

        command_usable = bool(info.get("command_usable", ok))
        if not bool(info.get("tracking_ok", ok)):
            tracking_bad += 1
        if int(info.get("status", 0)) != 0 or not bool(info.get("success", True)):
            solver_fail += 1

        if command_usable:
            q_cmd = q_next
            dq_ff = (q_next - q_current) / CONTROL_DT
            last_q_cmd = q_cmd.copy()
        else:
            fallback += 1
            q_cmd = last_q_cmd.copy()      # 命令不可用: 保持上一帧, 无前馈
            dq_ff = np.zeros_like(q_cmd)

        # 闭环: plant 执行命令, 推进真实关节角; 指标都在真实 q_actual 上统计
        q_actual = plant.advance(q_cmd, dq_ff)
        pos, rot = fk(ctx, q_actual)
        pos_errors.append(float(np.linalg.norm(pos - scenario.position_fn(t))))
        if lock:
            ori_errors.append(float(np.linalg.norm(orientation_error(rot, scenario.rotation_fn(t)))))
        ctrl_deltas.append(float(np.linalg.norm(q_actual - prev_q_actual)))
        sigma = min_singular_value(ctx, q_actual)
        sigmas.append(sigma)
        if sigma < SINGULAR_SIGMA_THRESHOLD:
            singular += 1
        prev_q_actual = q_actual.copy()

    pos_errors = np.asarray(pos_errors)
    ori_errors = np.asarray(ori_errors)
    ctrl_deltas = np.asarray(ctrl_deltas)
    solve_times = np.asarray(solve_times)
    sigmas = np.asarray(sigmas)

    metrics = ScenarioMetrics(
        name=scenario.name,
        rms_position_error=rms(pos_errors),
        max_position_error=float(np.max(pos_errors)) if pos_errors.size else 0.0,
        rms_orientation_error=rms(ori_errors),
        max_orientation_error=float(np.max(ori_errors)) if ori_errors.size else 0.0,
        rms_control_delta=rms(ctrl_deltas),
        min_sigma=float(np.min(sigmas)) if sigmas.size else 0.0,
        singular_count=singular,
        tracking_bad_count=tracking_bad,
        fallback_count=fallback,
        solver_failure_count=solver_fail,
        mean_solve_time_ms=float(np.mean(solve_times)) if solve_times.size else 0.0,
        p95_solve_time_ms=float(np.percentile(solve_times, 95)) if solve_times.size else 0.0,
        max_solve_time_ms=float(np.max(solve_times)) if solve_times.size else 0.0,
        score=0.0,
    )
    metrics.score = scenario_score(metrics)
    return metrics


def scenario_score(m: ScenarioMetrics) -> float:
    return (
        1000.0 * m.rms_position_error
        + 1000.0 * m.max_position_error
        + 100.0 * m.rms_orientation_error
        + 100.0 * m.max_orientation_error
        + 50.0 * m.rms_control_delta
        + 0.2 * m.tracking_bad_count
        + 0.05 * m.mean_solve_time_ms
    )


def evaluate(params: ParamSet, scenarios: Sequence[Scenario], ctx: Context,
             set_label: str) -> TrialResult:
    # 只构建场景子集真正需要的求解器 (separate 模式下单场景只建一个)
    need = {sc.lock_orientation for sc in scenarios}
    solvers = {lock: build_solver(params, lock) for lock in need}

    results = [evaluate_scenario(sc, solvers[sc.lock_orientation], ctx, params.horizon_steps)
               for sc in scenarios]

    feasible = True
    reasons = []
    for m in results:
        if m.fallback_count > 0:
            feasible = False
            reasons.append(f"{m.name}:fallback={m.fallback_count}")
        if m.solver_failure_count > 0:
            feasible = False
            reasons.append(f"{m.name}:solver_fail={m.solver_failure_count}")
        if m.singular_count > 0:
            feasible = False
            reasons.append(f"{m.name}:singular={m.singular_count}")
        if m.p95_solve_time_ms > MAX_SOLVE_TIME_MS_HARD:
            feasible = False
            reasons.append(f"{m.name}:slow_p95={m.p95_solve_time_ms:.1f}ms")

    if feasible:
        scores = [m.score for m in results]
        aggregate = float(np.mean(scores) + np.max(scores))
    else:
        aggregate = float("inf")

    return TrialResult(set_label, "", params, aggregate, feasible,
                       "; ".join(reasons), results)


# ======================================================================
# 搜索
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
            c = ParamSet(h, s, ow, tow)
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
              quiet_build: bool) -> List[TrialResult]:
    cands = maybe_limit(candidates, max_combos)
    tag = f"[{set_label}] {phase}"
    if len(cands) < len(candidates):
        print(f"{tag} 搜索: 限制为 {len(cands)}/{len(candidates)} 组")
    else:
        print(f"{tag} 搜索: {len(cands)} 组")

    results = []
    for i, params in enumerate(cands, start=1):
        print(f"{tag} {i}/{len(cands)}: h={params.horizon_steps}, "
              f"step={params.max_joint_step:.3f}, ow={params.orientation_weight:.3f}, "
              f"tow={params.terminal_orientation_weight:.3f}", flush=True)
        with suppress_native_output(quiet_build):
            r = evaluate(params, scenarios, ctx, set_label)
        r.phase = phase
        results.append(r)
        if r.feasible:
            worst = max(r.scenarios, key=lambda m: m.score)
            print(f"    agg={r.aggregate_score:.3f}, worst={worst.name} "
                  f"(rms_pos={worst.rms_position_error:.4f}, max_pos={worst.max_position_error:.4f}, "
                  f"min_sigma={worst.min_sigma:.3f}, mean_ms={worst.mean_solve_time_ms:.2f})", flush=True)
        else:
            print(f"    判废: {r.reject_reason}", flush=True)
    return results


def search(set_label: str, scenarios: Sequence[Scenario], ctx: Context,
           args: argparse.Namespace) -> List[TrialResult]:
    """对给定场景子集跑一遍粗到细搜索, 返回全部 TrialResult (已打 set_label)。"""
    all_results: List[TrialResult] = []
    coarse_results = run_phase("coarse", coarse_candidates(), scenarios, ctx, set_label,
                               args.max_combinations, args.quiet_build)
    all_results.extend(coarse_results)

    if not args.skip_fine:
        feasible = sorted([r for r in coarse_results if r.feasible],
                          key=lambda r: r.aggregate_score)[:args.fine_top]
        if feasible:
            fine = fine_candidates(feasible, seen=[r.params for r in all_results])
            all_results.extend(run_phase("fine", fine, scenarios, ctx, set_label,
                                         args.max_combinations, args.quiet_build))
        else:
            print(f"[{set_label}] 粗搜索没有可行组, 跳过细搜索。")
    return all_results


# ======================================================================
# 输出
# ======================================================================
def recommended_command(scenario: Scenario, params: ParamSet) -> str:
    parts = [f"horizon_steps:={params.horizon_steps}",
             f"max_joint_step:={params.max_joint_step:.4f}"]
    parts += scenario.demo_extra_args
    if scenario.lock_orientation:
        parts += [f"orientation_weight:={params.orientation_weight:.4f}",
                  f"terminal_orientation_weight:={params.terminal_orientation_weight:.4f}"]
    return f"python3 {scenario.demo} --ros-args " + " ".join(f"-p {p}" for p in parts)


def write_results(path: Path, results: Sequence[TrialResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "scenario_set", "rank", "phase", "feasible", "aggregate_score", "reject_reason",
        "horizon_steps", "max_joint_step", "orientation_weight", "terminal_orientation_weight",
        "scenario", "scenario_score", "rms_position_error", "max_position_error",
        "rms_orientation_error", "max_orientation_error", "rms_control_delta",
        "min_sigma", "singular_count", "tracking_bad_count", "fallback_count",
        "solver_failure_count", "mean_solve_time_ms", "p95_solve_time_ms", "max_solve_time_ms",
    ]
    # 每个 scenario_set 内部各自排名
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
                    writer.writerow({
                        "scenario_set": label, "rank": rank, "phase": r.phase,
                        "feasible": r.feasible, "aggregate_score": f"{r.aggregate_score:.6f}",
                        "reject_reason": r.reject_reason,
                        "horizon_steps": p.horizon_steps,
                        "max_joint_step": f"{p.max_joint_step:.6f}",
                        "orientation_weight": f"{p.orientation_weight:.6f}",
                        "terminal_orientation_weight": f"{p.terminal_orientation_weight:.6f}",
                        "scenario": m.name, "scenario_score": f"{m.score:.6f}",
                        "rms_position_error": f"{m.rms_position_error:.9f}",
                        "max_position_error": f"{m.max_position_error:.9f}",
                        "rms_orientation_error": f"{m.rms_orientation_error:.9f}",
                        "max_orientation_error": f"{m.max_orientation_error:.9f}",
                        "rms_control_delta": f"{m.rms_control_delta:.9f}",
                        "min_sigma": f"{m.min_sigma:.6f}",
                        "singular_count": m.singular_count,
                        "tracking_bad_count": m.tracking_bad_count,
                        "fallback_count": m.fallback_count,
                        "solver_failure_count": m.solver_failure_count,
                        "mean_solve_time_ms": f"{m.mean_solve_time_ms:.6f}",
                        "p95_solve_time_ms": f"{m.p95_solve_time_ms:.6f}",
                        "max_solve_time_ms": f"{m.max_solve_time_ms:.6f}",
                    })


def print_set(set_label: str, scenarios: Sequence[Scenario],
              results: Sequence[TrialResult], top_k: int) -> Optional[TrialResult]:
    feasible = sorted([r for r in results if r.feasible], key=lambda r: r.aggregate_score)
    print(f"\n===== [{set_label}] 可行参数组: {len(feasible)}/{len(results)} =====")
    if not feasible:
        print("  没有满足硬约束(无 fallback/求解失败/奇异/超时)的参数组, 请扩大搜索范围或放宽阈值。")
        return None
    print("  rank  agg     h  step   ow    tow    worst_scenario   max_pos  min_sigma mean_ms")
    for rank, r in enumerate(feasible[:top_k], start=1):
        p = r.params
        worst = max(r.scenarios, key=lambda m: m.score)
        print(f"  {rank:>4d} {r.aggregate_score:>7.3f} {p.horizon_steps:>3d} "
              f"{p.max_joint_step:>6.3f} {p.orientation_weight:>5.2f} {p.terminal_orientation_weight:>6.2f} "
              f"{worst.name:>16s} {worst.max_position_error:>8.4f} {worst.min_sigma:>8.3f} "
              f"{worst.mean_solve_time_ms:>7.2f}")
    best = feasible[0]
    print(f"  冠军各场景明细:")
    for m in best.scenarios:
        print(f"    {m.name:>16s} rms_pos={m.rms_position_error*1000:5.2f}mm "
              f"max_pos={m.max_position_error*1000:5.2f}mm min_sigma={m.min_sigma:.3f} "
              f"fb={m.fallback_count} fail={m.solver_failure_count} mean_ms={m.mean_solve_time_ms:.2f}")
    print(f"  推荐运行命令:")
    by_name = {s.name: s for s in scenarios}
    for m in best.scenarios:
        print("    " + recommended_command(by_name[m.name], best.params))
    return best


# ======================================================================
# main
# ======================================================================
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="闭环 MPC 调参 (固定臂+浮动基座臂)")
    parser.add_argument("--mode", choices=("unified", "separate"), default="unified",
                        help="unified: 一套参数满足所有场景; separate: 每个场景单独调参")
    parser.add_argument("--scenario", default=None,
                        help="只调指定场景(逗号分隔, 如 circle 或 circle,line); 默认全部。"
                             "配合 --mode separate 可手动单独调某个场景")
    parser.add_argument("--list-scenarios", action="store_true",
                        help="打印所有可选场景名后退出")
    parser.add_argument("--periods", type=float, default=1.0, help="每场景仿真周期数")
    parser.add_argument("--top-k", type=int, default=10)
    parser.add_argument("--output-dir", type=Path,
                        default=Path("/home/windylab/code/windylab_ws/src/arm-platform/demo/data"))
    parser.add_argument("--skip-fine", action="store_true", help="只跑粗搜索")
    parser.add_argument("--fine-top", type=int, default=5)
    parser.add_argument("--quiet-build", action=argparse.BooleanOptionalAction, default=True,
                        help="静音 ACADOS 代码生成/编译输出")
    parser.add_argument("--max-combinations", type=int, default=None,
                        help="每阶段最多评估多少组, 用于 smoke test")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.periods <= 0.0:
        raise ValueError("--periods 必须大于 0")

    ctx = make_context(args.periods)
    scenarios = build_scenarios(ctx)
    available = [s.name for s in scenarios]

    if args.list_scenarios:
        print("可选场景: " + ", ".join(available))
        return 0

    # --scenario 手动指定要调的场景(子集)
    if args.scenario:
        wanted = [n.strip() for n in args.scenario.split(",") if n.strip()]
        bad = [n for n in wanted if n not in available]
        if bad:
            raise SystemExit(f"未知场景 {bad}; 可选: {available}")
        scenarios = [s for s in scenarios if s.name in wanted]

    print(f"模式: {args.mode} | 场景: {[s.name for s in scenarios]}")
    print(f"闭环 plant: max_velocity={PLANT_MAX_VELOCITY}, ff_gain={PLANT_FF_GAIN}, "
          f"fb_tau={PLANT_FB_TAU}, max_accel={PLANT_MAX_ACCEL}; "
          f"max_joint_step 上限={MAX_JOINT_STEP_LIMIT:.3f}")

    # unified: 一个集合含(选中的)全部场景; separate: 每个场景一个集合
    if args.mode == "unified":
        sets = [("unified", list(scenarios))]
    else:
        sets = [(s.name, [s]) for s in scenarios]

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = args.output_dir / f"unified_mpc_tuning_{args.mode}_{stamp}.csv"

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
