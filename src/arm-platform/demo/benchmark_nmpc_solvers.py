#!/usr/bin/env python3
"""LL100-LH420 NMPC 求解器对比基准(front_wing_arc 情景,离线确定性闭环)。

在同一条路径、同一套 MpcIkConfig 基线、同一套避障硬约束下,对比多种求解器配置的:
  - 实时求解速度:每步 solve_time 的均值 / P95 / 最坏值、实时率(solve_time < control_dt 占比)
  - 末端误差精度:被控对象实际经过状态的位置跟踪误差(RMS / 最大,**稳态口径**:
    经 warmup_frac 排除从 neutral 追上轨迹的启动暂态,否则 RMS 被暂态主导)、终端误差
  - 安全(硬门槛):obstacle_violation_max 必须为 0、最小 margin >= 0
  - 奇异(硬门槛):末端雅可比最小奇异值 sigma_min / 条件数

两档对比共用一个配置矩阵:
  A 档(完整求解器):acados-SQP_RTI / acados-全SQP / CasADi-IPOPT / SciPy-SLSQP
  B 档(固定 acados,只换 QP 后端):HPIPM / qpOASES(OSQP 需 acados 编译 OSQP 后启用)
其中 acados-SQP_RTI+HPIPM 同时属于 A、B 两档(去重,只跑一次)。

门槛逻辑(安全优先于精度):任一步 violation>0 或 sigma_min<阈值 -> 该配置标记 unsafe,
排在所有合格配置之后;合格配置之间再按求解速度 / 精度排序。

闭环 plant 复刻 LL100 底层 SmoothPositionController(速度前馈),与 tune_ll100_nmpc.py 一致,
本脚本自包含(不 import 调参脚本),避免与其并发改动耦合。

用法:
    python3 benchmark_nmpc_solvers.py --path-mode front_wing_arc
    python3 benchmark_nmpc_solvers.py --path-mode front_wing_arc --periods 0.3 --tier B
需先 source ROS + 工作空间 + ACADOS(setup_acados_env.sh)。
"""

import argparse
import contextlib
import csv
import os
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK


PUBLISH_RATE_HZ = 50.0
CONTROL_DT = 1.0 / PUBLISH_RATE_HZ

EE_FRAME = "F_Link"
LINK_FRAMES = ("A_Link", "B_Link", "C_Link", "D_Link", "E_Link", "F_Link")
EVAL_PERIOD_SEC = 30.0
TERMINAL_TOLERANCE = 0.025
VIOLATION_TOL = 1.0e-4  # 侵入 <= 0.1mm 视为安全(数值/离散噪声)
SINGULARITY_SIGMA_MIN = 1.0e-3  # 末端雅可比最小奇异值低于此值视为接近奇异
DEFAULT_MAX_VELOCITY = 0.5  # 底层关节限速 rad/s(= ll100_lh420_control.launch.py 默认)

# SmoothPositionController 速度前馈配置(= LL100 底层固定值)。
VFF_GAIN = 0.7
VFF_TAU = 0.15
VFF_FB_VLIM = 0.4
VFF_MAX_ACCEL = 5.0

# 固定的"良好基线"轨迹/避障参数(只对比求解器,这些保持不变)。
BASELINE_HORIZON = 8
BASELINE_MAX_JOINT_STEP = 0.01
BASELINE_PREVIEW_STRIDE = 4
BASELINE_BODY_MARGIN = 0.005     # 对齐 demo 定稿(小正余量;连杆粗细靠 capsule_radius)
BASELINE_CAPSULE_SAMPLES = 1     # 对齐 demo 定稿


@dataclass(frozen=True)
class SolverConfig:
    """一个待对比的求解器配置。

    status:
      active     - 参与实时对比(默认会跑)
      eliminated - 已淘汰(只记录,不再跑;见 note)
      pending    - 待启用(前置条件未满足,如 acados 未编译 OSQP / GRAMPC 未集成)
    """
    key: str
    backend: str                       # acados / casadi / scipy / grampc
    nlp_solver_type: Optional[str]     # acados 专用:SQP_RTI / SQP
    qp_solver: Optional[str]           # acados 专用 QP 后端
    tiers: Tuple[str, ...]             # 该配置属于哪些档(A / B)
    label: str
    status: str = "active"
    note: str = ""
    # 每个配置可覆盖 MpcIkConfig 字段("让每个求解器调到自己最优"),如 QP 迭代上限。
    extra_overrides: Tuple[Tuple[str, object], ...] = ()


# ===== 7 种求解器配置 =====
# active(继续对比):HPIPM-RTI / HPIPM-全SQP / qpOASES / OSQP / GRAMPC
# eliminated(弃置回退,保留不删、只记录):CasADi-IPOPT / SciPy-SLSQP —— 速度太慢,不进实时回路
# 注:OSQP 已通过 -DACADOS_WITH_OSQP=ON 重编 acados 启用;GRAMPC 已集成(grampc_ll100.py)。
def solver_matrix() -> List[SolverConfig]:
    return [
        SolverConfig("acados_sqp_rti_hpipm", "acados", "SQP_RTI",
                     "PARTIAL_CONDENSING_HPIPM", ("A", "B"), "acados SQP_RTI + HPIPM",
                     status="active"),
        SolverConfig("acados_sqp_hpipm", "acados", "SQP",
                     "PARTIAL_CONDENSING_HPIPM", ("A",), "acados 全SQP + HPIPM",
                     status="active"),
        SolverConfig("acados_sqp_rti_qpoases", "acados", "SQP_RTI",
                     "FULL_CONDENSING_QPOASES", ("B",), "acados SQP_RTI + qpOASES",
                     status="active",
                     note="active-set,放开迭代上限 + primal 热启动",
                     extra_overrides=(("acados_qp_solver_iter_max", 500),
                                      ("acados_qp_warm_start", 1))),
        SolverConfig("acados_sqp_rti_osqp", "acados", "SQP_RTI",
                     "PARTIAL_CONDENSING_OSQP", ("B",), "acados SQP_RTI + OSQP",
                     status="active",
                     note="一阶 ADMM,放开迭代上限(需很多) + primal 热启动",
                     extra_overrides=(("acados_qp_solver_iter_max", 4000),
                                      ("acados_qp_warm_start", 1))),
        SolverConfig("grampc", "grampc", None, None, ("A",), "GRAMPC (增广拉格朗日+梯度法)",
                     status="active",
                     note="pygrampc + CasADi codegen 的 C++ 问题描述(grampc_ll100.py)"),
        SolverConfig("crocoddyl", "crocoddyl", None, None, ("C",), "Crocoddyl (DDP+CSQP硬约束)",
                     status="pending",
                     note="mim_solvers CSQP + Crocoddyl Python action model;需 ABI 对齐 ROS Pinocchio"),
        SolverConfig("casadi_ipopt", "casadi", None, None, ("A",), "CasADi + IPOPT",
                     status="eliminated",
                     note="弃置回退(保留不删):~1100ms/步、实时率0%,只能离线;仍用作高精度 ground-truth 参照"),
        SolverConfig("scipy_slsqp", "scipy", None, None, ("A",), "SciPy + SLSQP",
                     status="eliminated",
                     note="弃置回退(保留不删):~45ms/步、实时率低,不进实时回路"),
    ]


@dataclass
class BenchResult:
    config: SolverConfig
    path_mode: str
    safe: bool
    # 速度
    mean_solve_ms: float
    p95_solve_ms: float
    max_solve_ms: float
    realtime_rate: float
    mean_iters: float
    # 精度
    rms_position_error: float
    max_position_error: float
    mean_terminal_error: float
    # 安全
    violation_max: float
    min_margin: float
    min_body_margin: float
    # 奇异
    min_sigma_min: float
    max_condition: float
    # 计数
    fallback_count: int
    solver_failure_count: int
    samples: int
    fail_reason: str = ""
    # 多次重复运行的聚合(--repeats>1 时填充;确定性字段跨次相同,只有耗时波动)。
    runs: int = 1
    mean_ms_std: float = 0.0  # 跨次 mean_solve_ms 的标准差(时间稳定性)


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


def vertical_down_rotation() -> np.ndarray:
    # 末端 x 轴对齐 base_link -Z(= demo 的 vertical_down)。
    return np.array(
        [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]],
        dtype=float,
    )


def build_config(sc: SolverConfig, path_mode: str) -> Tuple[MpcIkConfig, str]:
    """构造与 tune_ll100_nmpc / move_ll100_lh420_nmpc_demo 同口径的 LL100 避障 NMPC 配置。

    只有求解器相关字段(backend / nlp / qp)随 SolverConfig 变化,其余保持基线不变。
    """
    from mpc_pinocchio_ik import DEFAULT_LL100_URDF
    from move_ll100_lh420_nmpc_demo import (
        DEFAULT_LL100_BODY_OBSTACLE_BOXES,
        LL100_NMPC_URDF,
        CAPSULE_CENTERLINE_POINTS,
        DEFAULT_CAPSULE_JOINT_RADII,
        DEFAULT_CAPSULE_LINK_RADII,
    )

    build_dir = f"/tmp/windylab_acados_bench_ll100_{path_mode}"
    cfg = MpcIkConfig(
        horizon_steps=BASELINE_HORIZON,
        control_dt=CONTROL_DT,
        max_joint_step=BASELINE_MAX_JOINT_STEP,
        success_tolerance=TERMINAL_TOLERANCE,
        solver_backend=sc.backend,
        acados_build_dir=build_dir,
        acados_nlp_solver_type=sc.nlp_solver_type or "SQP_RTI",
        acados_qp_solver=sc.qp_solver or "PARTIAL_CONDENSING_HPIPM",
        position_weight=120.0,
        terminal_position_weight=1200.0,
        orientation_weight=0.3,
        terminal_orientation_weight=3.0,
        obstacle_avoidance_enabled=True,
        obstacle_safety_margin=0.005,       # 对齐 demo 定稿(小正余量;含粗细靠 front capsule)
        obstacle_disk_half_thickness=0.02,
        obstacle_ll100_urdf_path=DEFAULT_LL100_URDF,
        obstacle_coordinate_mode="ll100_base",
        obstacle_keypoint_frames=LINK_FRAMES,
        front_obstacle_capsule=True,        # 前桨按连杆 capsule 建模(含粗细),对齐 demo 定稿
        body_obstacle_enabled=True,
        body_obstacle_margin=BASELINE_BODY_MARGIN,
        body_obstacle_boxes=DEFAULT_LL100_BODY_OBSTACLE_BOXES,
        capsule_radius=0.018,
        # 中心线胶囊部署几何(对齐 demo):优先级最高,前桨/机身/斥力都改用它 → 55 硬约束。
        capsule_points=CAPSULE_CENTERLINE_POINTS,
        capsule_joint_radii=tuple(DEFAULT_CAPSULE_JOINT_RADII),
        capsule_link_radii=tuple(DEFAULT_CAPSULE_LINK_RADII),
        capsule_samples_per_segment=3,
        capsule_frame_sequence=LINK_FRAMES,
        current_posture_weight=0.005,
        center_posture_weight=0.01,
        obstacle_repulsion_margin=0.03,     # 斥力场默认开(留净空、不贴墙),对齐 demo 定稿
        obstacle_repulsion_weight=1.0e4,
    )
    # 每个求解器的个性化参数覆盖(让它跑到自己最优),在构造 MpcPinocchioIK 前生效。
    for field_name, value in sc.extra_overrides:
        setattr(cfg, field_name, value)
    return cfg, LL100_NMPC_URDF


def _singularity_metrics(mpc: MpcPinocchioIK, q: np.ndarray) -> Tuple[float, float]:
    """返回末端 6xnq 雅可比的 (sigma_min, condition_number)。"""
    import pinocchio as pin

    jac = pin.computeFrameJacobian(
        mpc.model, mpc.data, q, mpc.frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    )
    sv = np.linalg.svd(jac, compute_uv=False)
    sigma_min = float(sv[-1])
    sigma_max = float(sv[0])
    cond = float(sigma_max / sigma_min) if sigma_min > 1e-12 else float("inf")
    return sigma_min, cond


def evaluate(sc: SolverConfig, path_mode: str, periods: float,
             quiet_build: bool, max_velocity: float, warmup_frac: float = 0.5) -> BenchResult:
    import pinocchio as pin
    from ll100_traj_pub import _build_path as build_ll100_path

    cfg, urdf = build_config(sc, path_mode)
    with suppress_native_output(quiet_build):
        if sc.backend == "grampc":
            # GRAMPC 走独立运行器(pygrampc + CasADi codegen 的 C++ 问题描述)。
            # 它内部持有一个 MpcPinocchioIK(用于 FK / 避障诊断 / 奇异),并暴露
            # 与 MpcPinocchioIK 同口径的 step / forward / model / _obstacle_diagnostics。
            from grampc_ll100 import GrampcRunner

            mpc = GrampcRunner(urdf_path=urdf, ee_frame=EE_FRAME, config=cfg)
        elif sc.backend == "crocoddyl":
            from crocoddyl_ll100 import CrocoddylRunner

            mpc = CrocoddylRunner(urdf_path=urdf, ee_frame=EE_FRAME, config=cfg)
        else:
            mpc = MpcPinocchioIK(urdf_path=urdf, ee_frame=EE_FRAME, config=cfg)

    count = max(20, int(round(EVAL_PERIOD_SEC / CONTROL_DT)))
    path = build_ll100_path(path_mode, count)
    locked_rotation = vertical_down_rotation()

    # 闭环 plant 复刻 SmoothPositionController(速度前馈)。
    q_plant = np.clip(pin.neutral(mpc.model), mpc.q_lower, mpc.q_upper)
    q_cmd = q_plant.copy()
    last_v = np.zeros_like(q_plant)

    actual_pos, _ = mpc.forward(q_plant)
    nearest = int(np.argmin(np.linalg.norm(path - actual_pos.reshape(1, 3), axis=1)))

    horizon = cfg.horizon_steps
    offsets = BASELINE_PREVIEW_STRIDE * np.arange(horizon)
    warm_start = None
    steps = max(1, int(round(periods * EVAL_PERIOD_SEC / CONTROL_DT)))
    # 精度取稳态口径:排除从 neutral 姿态追上轨迹的启动暂态(front_wing_arc 约需 ~800 步/16s),
    # 否则 RMS 被暂态主导(0.75mm 会被算成 ~65mm)。warmup 取步数的 warmup_frac(默认 0.5)。
    # 安全/奇异检查覆盖全程(仅跳过 IC 沉降的前几十步):启动暂态期机械臂在运动,
    # 也可能撞障碍/进奇异,不能漏检。精度 RMS 才用大 warmup 取稳态(排除追轨迹的暂态)。
    safety_warmup = min(50, steps // 4)
    accuracy_warmup = min(steps - 1, max(int(warmup_frac * steps), safety_warmup))
    if steps - accuracy_warmup < 50:
        print(f"    ⚠ 稳态样本仅 {steps - accuracy_warmup} 个(periods 太小,暂态可能未清干净,RMS 偏大)", flush=True)

    fallback = 0
    solver_fail = 0
    pos_errs: List[float] = []
    terminal_errs: List[float] = []
    solve_ms: List[float] = []
    iters: List[int] = []
    realized: List[np.ndarray] = []
    min_sigma_min = float("inf")
    max_condition = 0.0

    for step in range(steps):
        base = (nearest + step) % len(path)
        targets = path[(base + offsets) % len(path)]
        start = time.perf_counter()
        q_next, ok, info = mpc.step(
            targets, q_plant, warm_start=warm_start, target_rotation=locked_rotation
        )
        solve_ms.append(1000.0 * float(info.get("solve_time", time.perf_counter() - start)))
        iters.append(int(info.get("nit", 0)))
        terminal_errs.append(float(info.get("terminal_error", 0.0)))
        warm_start = info["trajectory"]
        if int(info.get("status", 0)) != 0 or not bool(info.get("success", True)):
            solver_fail += 1
        if bool(info.get("command_usable", ok)):
            q_cmd = q_next
            qdot_ref = (q_next - q_plant) / CONTROL_DT
        else:
            fallback += 1
            qdot_ref = np.zeros_like(q_plant)
        v_ff = np.clip(qdot_ref, -max_velocity, max_velocity)
        v_fb = np.clip((q_cmd - q_plant) / VFF_TAU, -VFF_FB_VLIM, VFF_FB_VLIM)
        v_raw = np.clip(VFF_GAIN * v_ff + v_fb, -max_velocity, max_velocity)
        accel = VFF_MAX_ACCEL * CONTROL_DT
        v_cmd = np.clip(v_raw, last_v - accel, last_v + accel)
        q_new = np.clip(q_plant + v_cmd * CONTROL_DT, mpc.q_lower, mpc.q_upper)
        last_v = (q_new - q_plant) / CONTROL_DT
        q_plant = q_new
        # 避障(碰撞):覆盖全程(含从 neutral 追轨迹的接近段)——任何时刻都不能撞障碍。
        if step >= safety_warmup:
            realized.append(q_plant.copy())
        # 奇异 + 精度:只看稳态窗口。理由:neutral home 姿态本身就近奇异(σ_min≈8e-6),
        # 机械臂离开 home 的启动接近段会短暂近奇异(实测仅 ~3 步 σ<1e-3),这是初始条件
        # 伪信号、非求解器把臂开进奇异。稳态窗口能正确区分:三合格解稳态 σ_min≈0.04(安全)
        # vs 全SQP/OSQP 全程卡在奇异 σ_min≈1e-5(不合格)。
        if step >= accuracy_warmup:
            actual_pos, _ = mpc.forward(q_plant)
            pos_errs.append(float(np.linalg.norm(actual_pos - path[base])))
            sigma_min, cond = _singularity_metrics(mpc, q_plant)
            min_sigma_min = min(min_sigma_min, sigma_min)
            max_condition = max(max_condition, cond)

    # 在被控对象实际经过的状态上量避障安全(不是 MPC 预测 plan)。
    if realized:
        diag = mpc._obstacle_diagnostics(np.asarray(realized, dtype=float))
        viol_max = float(diag.get("obstacle_violation_max") or 0.0)
        mm = diag.get("min_obstacle_margin")
        min_margin = float(mm) if mm is not None else 0.0
        bm = diag.get("min_body_obstacle_margin")
        min_body_margin = float(bm) if bm is not None else 0.0
    else:
        viol_max = 0.0
        min_margin = 0.0
        min_body_margin = 0.0

    if min_sigma_min == float("inf"):
        min_sigma_min = 0.0

    pos = np.asarray(pos_errs, dtype=float) if pos_errs else np.zeros(1)
    term = np.asarray(terminal_errs, dtype=float) if terminal_errs else np.zeros(1)
    sms = np.asarray(solve_ms, dtype=float) if solve_ms else np.zeros(1)

    # 安全 + 奇异硬门槛。
    fail_reasons = []
    if viol_max > VIOLATION_TOL:
        fail_reasons.append(f"violation={viol_max:.2e}")
    if fallback > 0:
        fail_reasons.append(f"fallback={fallback}")
    if min_sigma_min < SINGULARITY_SIGMA_MIN:
        fail_reasons.append(f"sigma_min={min_sigma_min:.2e}")
    safe = not fail_reasons

    realtime_rate = float(np.mean(sms < (CONTROL_DT * 1000.0)))

    return BenchResult(
        config=sc, path_mode=path_mode, safe=safe,
        mean_solve_ms=float(np.mean(sms)),
        p95_solve_ms=float(np.percentile(sms, 95)),
        max_solve_ms=float(np.max(sms)),
        realtime_rate=realtime_rate,
        mean_iters=float(np.mean(iters)) if iters else 0.0,
        rms_position_error=float(np.sqrt(np.mean(pos * pos))),
        max_position_error=float(np.max(pos)),
        mean_terminal_error=float(np.mean(term)),
        violation_max=viol_max,
        min_margin=min_margin,
        min_body_margin=min_body_margin,
        min_sigma_min=min_sigma_min,
        max_condition=max_condition,
        fallback_count=fallback,
        solver_failure_count=solver_fail,
        samples=len(pos_errs),
        fail_reason="; ".join(fail_reasons),
    )


def rank_key(r: BenchResult) -> Tuple:
    # 决策优先级:安全 > 求解时间(P95) > 精度(RMS)。合格者内先比 P95 再比 RMS。
    return (0 if r.safe else 1, r.p95_solve_ms, r.rms_position_error)


def write_csv(results: List[BenchResult], csv_path: Path,
              record_configs: Optional[List[SolverConfig]] = None) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "rank", "status", "key", "label", "tiers", "backend", "nlp_solver_type", "qp_solver",
            "safe", "fail_reason", "runs",
            "mean_solve_ms", "mean_ms_std", "p95_solve_ms", "max_solve_ms", "realtime_rate", "mean_iters",
            "rms_position_error", "max_position_error", "mean_terminal_error",
            "violation_max", "min_margin", "min_body_margin",
            "min_sigma_min", "max_condition",
            "fallback_count", "solver_failure_count", "samples", "note",
        ])
        for rank, r in enumerate(results, 1):
            c = r.config
            writer.writerow([
                rank, c.status, c.key, c.label, "+".join(c.tiers), c.backend,
                c.nlp_solver_type or "", c.qp_solver or "",
                r.safe, r.fail_reason, r.runs,
                f"{r.mean_solve_ms:.3f}", f"{r.mean_ms_std:.3f}", f"{r.p95_solve_ms:.3f}", f"{r.max_solve_ms:.3f}",
                f"{r.realtime_rate:.3f}", f"{r.mean_iters:.2f}",
                f"{r.rms_position_error:.6f}", f"{r.max_position_error:.6f}",
                f"{r.mean_terminal_error:.6f}",
                f"{r.violation_max:.8f}", f"{r.min_margin:.6f}", f"{r.min_body_margin:.6f}",
                f"{r.min_sigma_min:.6f}", f"{r.max_condition:.2f}",
                r.fallback_count, r.solver_failure_count, r.samples, c.note,
            ])
        # 未运行的配置(已淘汰 / 待启用)也记录在案,指标留空。
        for c in record_configs or []:
            writer.writerow([
                "-", c.status, c.key, c.label, "+".join(c.tiers), c.backend,
                c.nlp_solver_type or "", c.qp_solver or "", "", "", "",
                "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", c.note,
            ])


def maybe_plot(results: List[BenchResult], out_dir: Path, path_mode: str, stamp: str) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # noqa: BLE001
        print(f"(跳过画图:matplotlib 不可用 - {exc})", flush=True)
        return
    labels = [r.config.key for r in results]
    x = np.arange(len(labels))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    ax1.bar(x, [r.p95_solve_ms for r in results], color="#4C72B0")
    ax1.axhline(CONTROL_DT * 1000.0, color="r", ls="--", label=f"control_dt={CONTROL_DT*1000:.0f}ms")
    ax1.set_ylabel("P95 solve time (ms)")
    ax1.set_title("求解速度 (P95)")
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, rotation=30, ha="right")
    ax1.legend()
    ax2.bar(x, [r.rms_position_error * 1000.0 for r in results], color="#55A868")
    ax2.set_ylabel("RMS position error (mm)")
    ax2.set_title("末端跟踪精度 (RMS)")
    ax2.set_xticks(x)
    ax2.set_xticklabels(labels, rotation=30, ha="right")
    fig.tight_layout()
    png = out_dir / f"benchmark_nmpc_{path_mode}_{stamp}.png"
    fig.savefig(png, dpi=120)
    print(f"图: {png}", flush=True)


def aggregate_runs(runs: List[BenchResult]) -> BenchResult:
    """把同一配置的 N 次运行聚合成一条:确定性字段(精度/安全)取首次,
    耗时取跨次统计——mean 取均值±std、P95 取最坏、最坏值取最坏、实时率取最差。"""
    base = runs[0]
    means = np.array([r.mean_solve_ms for r in runs], dtype=float)
    p95s = np.array([r.p95_solve_ms for r in runs], dtype=float)
    maxes = np.array([r.max_solve_ms for r in runs], dtype=float)
    rts = np.array([r.realtime_rate for r in runs], dtype=float)
    base.mean_solve_ms = float(means.mean())
    base.mean_ms_std = float(means.std())
    base.p95_solve_ms = float(p95s.max())   # 跨次最坏 P95(实时可靠性看最坏)
    base.max_solve_ms = float(maxes.max())  # 跨次最坏单步
    base.realtime_rate = float(rts.min())   # 跨次最差实时率
    base.runs = len(runs)
    # 确定性字段一致性自检:精度/安全跨次本应完全相同,不同则告警(可能有非确定性泄漏)。
    rms_all = {round(r.rms_position_error, 6) for r in runs}
    viol_all = {round(r.violation_max, 8) for r in runs}
    if len(rms_all) > 1 or len(viol_all) > 1:
        base.fail_reason = (base.fail_reason + f"; ⚠跨次不一致 rms={sorted(rms_all)} viol={sorted(viol_all)}").strip("; ")
    return base


def run(args: argparse.Namespace) -> int:
    matrix = solver_matrix()
    configs = matrix
    if args.tier in ("A", "B"):
        configs = [c for c in configs if args.tier in c.tiers]
    if args.only:
        # 显式指定时按名跑,无视 status(便于复跑被淘汰/待启用的配置)。
        wanted = set(args.only.split(","))
        configs = [c for c in configs if c.key in wanted]
    elif not args.include_eliminated:
        # 默认只跑 active;eliminated/pending 仅记录不跑。
        configs = [c for c in configs if c.status == "active"]
    if not configs:
        print("没有匹配的求解器配置(检查 --tier / --only / status)")
        return 1

    run_keys = {c.key for c in configs}
    # 在矩阵里但本轮没跑的配置(淘汰/待启用),作为记录行写进表格与 CSV。
    record_configs = [c for c in matrix if c.key not in run_keys and c.status != "active"]

    print(f"[NMPC 求解器对比] path_mode={args.path_mode} 运行={len(configs)} "
          f"记录={len(record_configs)} periods={args.periods} repeats={args.repeats} "
          f"warmup_frac={args.warmup_frac}(RMS 为稳态口径,已排暂态)", flush=True)
    results: List[BenchResult] = []
    for i, sc in enumerate(configs):
        print(f"  [{i+1}/{len(configs)}] {sc.label} ...", flush=True)
        run_results: List[BenchResult] = []
        for rep in range(max(1, args.repeats)):
            try:
                r = evaluate(sc, args.path_mode, args.periods, args.quiet_build,
                             args.max_velocity, args.warmup_frac)
            except Exception as exc:  # noqa: BLE001 - 单次失败不应中断整轮对比
                print(f"    跳过(异常, run {rep+1}): {exc}", flush=True)
                continue
            run_results.append(r)
            if args.repeats > 1:
                print(f"    run{rep+1}/{args.repeats}: mean_ms={r.mean_solve_ms:.2f} "
                      f"p95_ms={r.p95_solve_ms:.2f} rms={r.rms_position_error*1000:.2f}mm", flush=True)
        if not run_results:
            continue
        agg = aggregate_runs(run_results)
        results.append(agg)
        print(
            f"    ==> safe={agg.safe} {agg.fail_reason} | "
            f"mean_ms={agg.mean_solve_ms:.2f}±{agg.mean_ms_std:.2f} p95_max={agg.p95_solve_ms:.2f} "
            f"rt_min={agg.realtime_rate:.2f} rms_pos={agg.rms_position_error*1000:.2f}mm "
            f"viol={agg.violation_max:.2e} sigma_min={agg.min_sigma_min:.3e} (n={agg.runs})",
            flush=True,
        )

    if not results:
        print("没有可用结果(检查 path_mode 是否被 ll100_traj_pub 支持、ACADOS 是否就绪)")
        return 1

    results.sort(key=rank_key)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = args.output_dir / f"benchmark_nmpc_{args.path_mode}_{stamp}.csv"
    write_csv(results, csv_path, record_configs)
    print(f"\nCSV: {csv_path}", flush=True)

    # 汇总表(mean_ms 为跨次均值±std,p95/rt 为跨次最坏,n=重复次数)
    print(f"\n=== 汇总(每配置 n={args.repeats} 次;优先级 安全>求解时间(P95)>精度(RMS))===")
    header = (f"{'#':>2} {'key':<26} {'safe':>5} {'n':>3} {'mean_ms(±std)':>16} "
              f"{'p95max':>8} {'rt%':>5} {'rms_mm':>7} {'sigmin':>9} {'viol':>9}")
    print(header)
    for rank, r in enumerate(results, 1):
        mean_col = f"{r.mean_solve_ms:.2f}±{r.mean_ms_std:.2f}"
        print(f"{rank:>2} {r.config.key:<26} {str(r.safe):>5} {r.runs:>3} {mean_col:>16} "
              f"{r.p95_solve_ms:>8.2f} {r.realtime_rate*100:>5.0f} "
              f"{r.rms_position_error*1000:>7.2f} {r.min_sigma_min:>9.2e} {r.violation_max:>9.2e}")
    # 记录行:淘汰 / 待启用
    for c in record_configs:
        tag = "淘汰" if c.status == "eliminated" else "待启用"
        print(f" - {c.key:<26} [{tag}] {c.note}")

    if args.plot:
        maybe_plot(results, args.output_dir, args.path_mode, stamp)

    safe_results = [r for r in results if r.safe]
    if safe_results:
        best = safe_results[0]
        print(f"\n推荐(优先级 安全>求解时间>精度,合格中 P95 最小):{best.config.label} "
              f"(p95={best.p95_solve_ms:.2f}ms, rms={best.rms_position_error*1000:.2f}mm)")
    else:
        print("\n⚠️ 没有配置通过安全/奇异门槛;请检查基线参数或路径。")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="LL100 NMPC 求解器对比基准")
    parser.add_argument("--path-mode", default="front_wing_arc",
                        help="路径模式(ll100_traj_pub 支持的模式)")
    parser.add_argument("--tier", choices=("A", "B", "all"), default="all",
                        help="只跑 A 档(完整求解器)、B 档(QP 后端)或全部")
    parser.add_argument("--only", default=None,
                        help="逗号分隔的 SolverConfig.key,只跑指定配置(无视 status)")
    parser.add_argument("--include-eliminated", action="store_true",
                        help="连同已淘汰/待启用的配置一起跑(默认只跑 status=active)")
    parser.add_argument("--periods", type=float, default=0.3,
                        help="离线仿真路径周期数;SciPy/SLSQP 较慢,默认较小")
    parser.add_argument("--repeats", type=int, default=1,
                        help="每个配置重复运行次数;耗时取跨次均值±std/最坏,精度确定性应跨次一致")
    parser.add_argument("--warmup-frac", type=float, default=0.5,
                        help="RMS 稳态口径:跳过前 frac*steps 步排除启动暂态(需 periods≥~0.7 才够清干净)")
    parser.add_argument("--max-velocity", type=float, default=DEFAULT_MAX_VELOCITY,
                        help="闭环 plant 底层关节限速 rad/s")
    parser.add_argument("--output-dir", type=Path,
                        default=Path("/home/windylab/code/windylab_ws/src/arm-platform/demo/data"))
    parser.add_argument("--plot", action=argparse.BooleanOptionalAction, default=True,
                        help="是否输出对比柱状图(需 matplotlib)")
    parser.add_argument("--quiet-build", action=argparse.BooleanOptionalAction, default=True,
                        help="静音 ACADOS 代码生成/编译输出")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.periods <= 0.0:
        raise ValueError("--periods 必须大于 0")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
