#!/usr/bin/env python3
"""二阶运动学 NMPC 原型：状态 x=[q,dq]，控制 u=ddq。

该模块与现有一阶增量模型并存。离散动力学采用控制周期内恒加速度的精确积分：

    q[k+1]  = q[k] + dt*dq[k] + 0.5*dt^2*ddq[k]
    dq[k+1] = dq[k] + dt*ddq[k]
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import time
from typing import Optional

import numpy as np
import pinocchio as pin

from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK


@dataclass
class SecondOrderMpcConfig(MpcIkConfig):
    max_joint_velocity: float = 1.0
    max_joint_acceleration: float = 5.0
    joint_velocity_weight: float = 0.02
    joint_acceleration_weight: float = 0.01
    terminal_joint_velocity_weight: float = 0.0
    rti_iteration_mode: str = "fixed_iterations"
    singularity_constraint_enabled: bool = True
    min_translation_jacobian_sigma: float = 0.05
    acados_build_dir: str = "/tmp/windylab_acados_second_order_mpc"


class SecondOrderMpcIK(MpcPinocchioIK):
    """二阶运动学 NMPC，含位置/姿态软代价及平移雅可比奇异性硬约束。"""

    def __init__(self, *args, config: Optional[SecondOrderMpcConfig] = None, **kwargs):
        cfg = config if config is not None else SecondOrderMpcConfig()
        if cfg.horizon_steps < 1 or cfg.control_dt <= 0.0:
            raise ValueError("horizon_steps 必须 >=1 且 control_dt 必须 >0")
        if cfg.max_joint_velocity <= 0.0 or cfg.max_joint_acceleration <= 0.0:
            raise ValueError("max_joint_velocity/max_joint_acceleration 必须 > 0")
        if cfg.obstacle_avoidance_enabled or cfg.body_obstacle_enabled:
            raise ValueError("二阶原型尚未实现障碍约束，不能静默忽略")
        if cfg.rti_iteration_mode != "fixed_iterations":
            raise ValueError("二阶原型仅支持 rti_iteration_mode='fixed_iterations'")
        if cfg.min_translation_jacobian_sigma < 0.0:
            raise ValueError("min_translation_jacobian_sigma 必须 >= 0")
        super().__init__(*args, config=cfg, **kwargs)
        self.config: SecondOrderMpcConfig
        self._second_order_backend = None
        self._last_velocity_plan = None
        self._last_acceleration_plan = None

    def step(
        self,
        target_position: np.ndarray,
        q_current: np.ndarray,
        dq_current: np.ndarray,
        warm_start: Optional[np.ndarray] = None,
        posture_reference: Optional[np.ndarray] = None,
        target_rotation: Optional[np.ndarray] = None,
    ):
        q0 = np.asarray(q_current, dtype=float).reshape(-1)
        if q0.shape != (self.nq,) or not np.all(np.isfinite(q0)):
            raise ValueError(f"q_current 必须是有限 ({self.nq},) 向量")
        if np.any(q0 < self.q_lower - 1e-9) or np.any(q0 > self.q_upper + 1e-9):
            raise ValueError("q_current 超出 URDF 关节限位；不裁剪实测 x0")
        v0 = np.asarray(dq_current, dtype=float).reshape(-1)
        if v0.shape != (self.nq,) or not np.all(np.isfinite(v0)):
            raise ValueError(f"dq_current 必须是有限 ({self.nq},) 向量")
        if np.max(np.abs(v0)) > self.config.max_joint_velocity + 1e-6:
            raise ValueError("dq_current 超出 max_joint_velocity；不裁剪实测 x0 伪造可行状态")
        targets = self._target_sequence(target_position)
        if not np.all(np.isfinite(targets)):
            raise ValueError("target_position 含 NaN/Inf")
        rotations, orientation_active = self._rotation_sequence(target_rotation, q0)
        source_plan = self._last_plan if warm_start is None else warm_start
        if source_plan is None:
            q_guess = np.repeat(q0.reshape(1, self.nq),
                                self.config.horizon_steps, axis=0)
        else:
            source_plan = np.asarray(source_plan, dtype=float)
            if source_plan.shape != (self.config.horizon_steps, self.nq):
                raise ValueError(
                    f"warm_start 必须是 ({self.config.horizon_steps},{self.nq})")
            q_guess = np.vstack([source_plan[1:], source_plan[-1:]]).copy()
        q_ref = q0 if posture_reference is None else np.asarray(
            posture_reference, dtype=float).reshape(-1)
        if q_ref.shape != (self.nq,) or not np.all(np.isfinite(q_ref)):
            raise ValueError(f"posture_reference 必须是有限 ({self.nq},) 向量")
        q_ref = np.clip(q_ref, self.q_lower, self.q_upper)

        backend = self._get_second_order_backend()
        result = backend.optimize(
            q0, v0, targets, rotations, q_guess, q_ref)
        q_plan = result["q"]
        v_plan = result["dq"]
        a_plan = result["ddq"]
        q_next = q_plan[0].copy()
        dq_next = v_plan[0].copy()
        first_error = self._position_error(q_next, targets[0])
        terminal_error = self._position_error(q_plan[-1], targets[-1])
        terminal_orientation_error = None
        if orientation_active:
            _, terminal_rotation = self.forward(q_plan[-1])
            terminal_orientation_error = float(np.linalg.norm(
                self._orientation_error(terminal_rotation, rotations[-1])))
        translation_sigmas = np.array([
            self._translation_min_sigma(q) for q in q_plan])

        tol = 1e-7
        feasible = bool(
            np.all(np.isfinite(q_plan))
            and np.all(np.isfinite(v_plan))
            and np.all(np.isfinite(a_plan))
            and np.all(q_plan >= self.q_lower - tol)
            and np.all(q_plan <= self.q_upper + tol)
            and np.max(np.abs(v_plan)) <= self.config.max_joint_velocity + tol
            and np.max(np.abs(a_plan)) <= self.config.max_joint_acceleration + tol
            and (
                not self.config.singularity_constraint_enabled
                or np.min(translation_sigmas)
                >= self.config.min_translation_jacobian_sigma - 1e-6
            )
        )
        status_ok = result["status"] == 0
        tracking_ok = terminal_error <= self.config.success_tolerance
        command_usable = bool(status_ok and feasible)
        self._last_plan = q_plan.copy()
        self._last_velocity_plan = v_plan.copy()
        self._last_acceleration_plan = a_plan.copy()
        info = {
            "success": status_ok,
            "status": result["status"],
            "message": "ACADOS_SUCCESS" if status_ok else f"ACADOS_STATUS_{result['status']}",
            "tracking_ok": tracking_ok,
            "plan_feasible": feasible,
            "command_usable": command_usable,
            "first_error": first_error,
            "terminal_error": terminal_error,
            "terminal_orientation_error": terminal_orientation_error,
            "orientation_active": orientation_active,
            "min_translation_jacobian_sigma": float(np.min(translation_sigmas)),
            "solve_time": result["solve_time"],
            "nit": result["nit"],
            "trajectory": q_plan.copy(),
            "velocity_trajectory": v_plan.copy(),
            "acceleration_trajectory": a_plan.copy(),
            "ddq0": a_plan[0].copy(),
            "state_definition": "[q,dq]",
            "control_definition": "ddq",
        }
        return q_next, dq_next, bool(tracking_ok and command_usable), info

    def _translation_min_sigma(self, q: np.ndarray) -> float:
        jacobian = pin.computeFrameJacobian(
            self.model, self.data, q, self.frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        return float(np.min(np.linalg.svd(jacobian[:3, :], compute_uv=False)))

    def _get_second_order_backend(self):
        if self._second_order_backend is None:
            self._second_order_backend = _AcadosSecondOrderBackend(self)
        return self._second_order_backend


class _AcadosSecondOrderBackend:
    def __init__(self, owner: SecondOrderMpcIK):
        try:
            import casadi as ca
            from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
        except Exception as exc:
            raise RuntimeError(
                "二阶 NMPC 需要 ACADOS；请先 source scripts/setup_acados_env.sh"
            ) from exc

        self.owner = owner
        self.ca = ca
        cfg = owner.config
        nq = owner.nq
        nx, nu = 2 * nq, nq
        np_param = 3 + 9 + 2 * nq
        dt = float(cfg.control_dt)

        x = ca.MX.sym("x", nx)
        u = ca.MX.sym("u", nu)
        p = ca.MX.sym("p", np_param)
        q, dq = x[:nq], x[nq:]
        target = p[:3]
        target_rot = ca.reshape(p[3:12], 3, 3).T
        q_ref = p[12:12 + nq]
        q_center = p[12 + nq:12 + 2 * nq]

        model = AcadosModel()
        model.name = "windylab_arm_second_order_mpc"
        model.x, model.u, model.p = x, u, p
        model.disc_dyn_expr = ca.vertcat(
            q + dt * dq + 0.5 * dt * dt * u,
            dq + dt * u,
        )
        ee_pos = owner._symbolic_fk.position(q)
        ee_rot = owner._symbolic_fk.rotation(q)
        position_jacobian = ca.jacobian(ee_pos, x)[:, :nq]
        pos_error = ee_pos - target
        rot_error = owner._orientation_error_ca(ee_rot, target_rot)
        model.cost_y_expr = ca.vertcat(
            pos_error, rot_error, dq, u, q - q_ref, q - q_center)
        model.cost_y_expr_e = ca.vertcat(
            pos_error, rot_error, dq, q - q_ref, q - q_center)

        # Jp*Jp^T - sigma_min^2 I >= 0 的全部主子式非负，等价于
        # 平移雅可比最小奇异值 >= sigma_min（3x3 对称矩阵 PSD 判据）。
        if cfg.singularity_constraint_enabled:
            jp = position_jacobian
            gram_shift = ca.mtimes(jp, jp.T) - (
                cfg.min_translation_jacobian_sigma ** 2) * ca.MX.eye(3)
            minor01 = (gram_shift[0, 0] * gram_shift[1, 1]
                       - gram_shift[0, 1] * gram_shift[1, 0])
            minor02 = (gram_shift[0, 0] * gram_shift[2, 2]
                       - gram_shift[0, 2] * gram_shift[2, 0])
            minor12 = (gram_shift[1, 1] * gram_shift[2, 2]
                       - gram_shift[1, 2] * gram_shift[2, 1])
            det3 = (
                gram_shift[0, 0] * (
                    gram_shift[1, 1] * gram_shift[2, 2]
                    - gram_shift[1, 2] * gram_shift[2, 1])
                - gram_shift[0, 1] * (
                    gram_shift[1, 0] * gram_shift[2, 2]
                    - gram_shift[1, 2] * gram_shift[2, 0])
                + gram_shift[0, 2] * (
                    gram_shift[1, 0] * gram_shift[2, 1]
                    - gram_shift[1, 1] * gram_shift[2, 0])
            )
            principal_minors = ca.vertcat(
                gram_shift[0, 0], gram_shift[1, 1], gram_shift[2, 2],
                minor01, minor02, minor12,
                det3,
            )
            model.con_h_expr = principal_minors
            model.con_h_expr_e = principal_minors

        ocp = AcadosOcp()
        ocp.model = model
        ocp.solver_options.N_horizon = cfg.horizon_steps
        ocp.dims.np = np_param
        ocp.parameter_values = np.zeros(np_param)
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        # 运行阶段代价乘 dt，使相同物理时域下 50/100Hz 的累计权重可比。
        W = dt * np.diag(np.concatenate([
            np.full(3, cfg.position_weight),
            np.full(3, cfg.orientation_weight),
            np.full(nq, cfg.joint_velocity_weight),
            np.full(nq, cfg.joint_acceleration_weight),
            np.full(nq, cfg.current_posture_weight),
            np.full(nq, cfg.center_posture_weight),
        ]))
        W_e = np.diag(np.concatenate([
            np.full(3, cfg.terminal_position_weight),
            np.full(3, cfg.terminal_orientation_weight),
            np.full(nq, cfg.terminal_joint_velocity_weight),
            np.full(nq, cfg.current_posture_weight),
            np.full(nq, cfg.center_posture_weight),
        ]))
        # 生成态固定为单位阵；真实权重在 solver 创建后通过 cost_set 写入。
        # 这样纯权重调参不改变 OCP JSON，也不重复 codegen。
        ocp.cost.W = np.eye(W.shape[0])
        ocp.cost.W_e = np.eye(W_e.shape[0])
        ocp.cost.yref = np.zeros(W.shape[0])
        ocp.cost.yref_e = np.zeros(W_e.shape[0])

        ocp.constraints.idxbx = np.arange(nx)
        ocp.constraints.lbx = np.concatenate([
            owner.q_lower, np.full(nq, -cfg.max_joint_velocity)])
        ocp.constraints.ubx = np.concatenate([
            owner.q_upper, np.full(nq, cfg.max_joint_velocity)])
        ocp.constraints.idxbx_e = np.arange(nx)
        ocp.constraints.lbx_e = ocp.constraints.lbx.copy()
        ocp.constraints.ubx_e = ocp.constraints.ubx.copy()
        ocp.constraints.idxbu = np.arange(nu)
        ocp.constraints.lbu = np.full(nu, -cfg.max_joint_acceleration)
        ocp.constraints.ubu = np.full(nu, cfg.max_joint_acceleration)
        if cfg.singularity_constraint_enabled:
            ocp.constraints.lh = np.zeros(7)
            ocp.constraints.uh = np.full(7, 1e9)
            ocp.constraints.lh_e = np.zeros(7)
            ocp.constraints.uh_e = np.full(7, 1e9)
        ocp.constraints.x0 = np.zeros(nx)

        ocp.solver_options.tf = cfg.horizon_steps * dt
        ocp.solver_options.integrator_type = "DISCRETE"
        ocp.solver_options.nlp_solver_type = cfg.acados_nlp_solver_type
        ocp.solver_options.qp_solver = cfg.acados_qp_solver
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.qp_solver_iter_max = cfg.acados_qp_solver_iter_max
        ocp.solver_options.qp_solver_warm_start = cfg.acados_qp_warm_start
        ocp.solver_options.print_level = 0

        fingerprint = hashlib.sha256(json.dumps({
            "model": "double_integrator_zoh_v2_position_only",
            "urdf_sha256": hashlib.sha256(
                Path(owner.urdf_path).read_bytes()).hexdigest(),
            "ee_frame": owner.ee_frame,
            "n": cfg.horizon_steps,
            "dt": dt,
            "vmax": cfg.max_joint_velocity,
            "amax": cfg.max_joint_acceleration,
            "nlp_solver": cfg.acados_nlp_solver_type,
            "qp_solver": cfg.acados_qp_solver,
            "singularity_constraint_enabled": cfg.singularity_constraint_enabled,
            "min_translation_jacobian_sigma": cfg.min_translation_jacobian_sigma,
        }, sort_keys=True).encode()).hexdigest()[:12]
        tag = f"h{cfg.horizon_steps}_dt{int(round(dt*1000))}_{fingerprint}"
        build_dir = Path(cfg.acados_build_dir).expanduser().resolve() / tag
        build_dir.mkdir(parents=True, exist_ok=True)
        ocp.code_gen_opts.code_export_directory = str(build_dir / "c_generated_code")
        json_path = build_dir / "acados_ocp.json"
        library = (build_dir / "c_generated_code"
                   / f"libacados_ocp_solver_{model.name}.so")
        reuse = json_path.exists() and library.exists()
        try:
            self.solver = AcadosOcpSolver(
                ocp, json_file=str(json_path), generate=not reuse, build=not reuse)
        except OSError:
            self.solver = AcadosOcpSolver(
                ocp, json_file=str(json_path), generate=True, build=True)
        # 权重不改变残差维数或模型结构；运行时覆盖使调参候选共用同一份生成代码。
        for stage in range(cfg.horizon_steps):
            self.solver.cost_set(stage, "W", W)
        self.solver.cost_set(cfg.horizon_steps, "W", W_e)

    def optimize(
        self, q0, v0, targets, rotations, q_guess, q_ref,
    ):
        cfg = self.owner.config
        nq, horizon = self.owner.nq, cfg.horizon_steps
        dt = float(cfg.control_dt)
        v_guess = np.empty_like(q_guess)
        previous_q = q0
        for k in range(horizon):
            v_guess[k] = np.clip(
                (q_guess[k] - previous_q) / dt,
                -cfg.max_joint_velocity, cfg.max_joint_velocity)
            previous_q = q_guess[k]
        if (self.owner._last_acceleration_plan is not None
                and self.owner._last_acceleration_plan.shape == (horizon, nq)):
            old = self.owner._last_acceleration_plan
            a_guess = np.vstack([old[1:], old[-1:]]).copy()
        else:
            a_guess = np.empty_like(q_guess)
            previous_v = v0
            for k in range(horizon):
                a_guess[k] = np.clip(
                    (v_guess[k] - previous_v) / dt,
                    -cfg.max_joint_acceleration, cfg.max_joint_acceleration)
                previous_v = v_guess[k]

        # 从新的实测 x0 按二阶动力学重推状态初值，避免 q/v/a warm-start 互不一致。
        q_state, v_state = q0.copy(), v0.copy()
        for k in range(horizon):
            acceleration = a_guess[k]
            q_state = q_state + dt * v_state + 0.5 * dt * dt * acceleration
            v_state = v_state + dt * acceleration
            q_guess[k] = q_state
            v_guess[k] = v_state

        x0 = np.concatenate([q0, v0])
        for stage in range(horizon + 1):
            target_index = min(max(stage - 1, 0), horizon - 1)
            p = np.concatenate([
                targets[target_index], rotations[target_index].reshape(-1),
                q_ref, self.owner.q_center])
            self.solver.set(stage, "p", p)
            if stage == 0:
                x_guess = x0
            else:
                x_guess = np.concatenate([q_guess[stage - 1], v_guess[stage - 1]])
            self.solver.set(stage, "x", x_guess)
            if stage < horizon:
                self.solver.set(stage, "u", a_guess[stage])
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)

        start = time.perf_counter()
        status, nit = 0, 0
        for nit in range(1, cfg.acados_rti_iterations + 1):
            status = int(self.solver.solve())
            if status != 0:
                break
        solve_time = time.perf_counter() - start
        xs = np.array([
            self.solver.get(stage, "x") for stage in range(horizon + 1)])
        us = np.array([
            self.solver.get(stage, "u") for stage in range(horizon)])
        return {
            "q": xs[1:, :nq],
            "dq": xs[1:, nq:],
            "ddq": us,
            "status": status,
            "nit": nit,
            "solve_time": solve_time,
        }
