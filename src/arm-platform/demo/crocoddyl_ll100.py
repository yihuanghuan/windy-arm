#!/usr/bin/env python3
"""Crocoddyl/mim_solvers CSQP runner for LL100 NMPC IK.

The model mirrors the existing acados kinematic NMPC:
  state x = [q, dq_prev], control u = dq, x_next = [q + u, u].
  Costs track end-effector position/orientation and regularize u, du,
  current posture, and center posture using MpcIkConfig weights.

Obstacle hard constraints reuse MpcPinocchioIK._obstacle_margin_functions()
verbatim.  The custom ResidualModel below evaluates the same CasADi margin
and its analytic Jacobian; the action model exposes it to CSQP as g >= 0.
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np


_DEFAULT_CROCO_PREFIX = Path(
    os.environ.get(
        "CROCODDYL_ROS_HUMBLE_PREFIX",
        "/home/windylab/.local/opt/crocoddyl_ros_humble_pinocchio39",
    )
).expanduser()


def _ensure_local_crocoddyl_path() -> None:
    site = _DEFAULT_CROCO_PREFIX / "lib/python3.10/site-packages"
    if site.exists():
        site_s = str(site)
        if site_s not in sys.path:
            sys.path.insert(0, site_s)


_ensure_local_crocoddyl_path()

try:
    import crocoddyl
    import mim_solvers
except Exception as exc:  # noqa: BLE001
    raise RuntimeError(
        "CrocoddylRunner 需要 ABI 兼容的 crocoddyl + mim_solvers。"
        "本机应先 source /opt/ros/humble/setup.bash,并把本地源码安装前缀加入 PYTHONPATH。"
        "不要用 pip 安装 crocoddyl/pinocchio。"
    ) from exc


class _ObstacleMarginResidual(crocoddyl.ResidualModelAbstract):
    """Bridge MpcPinocchioIK CasADi margin(q) into a Crocoddyl residual."""

    def __init__(self, state, nu: int, nq: int, value_fn, jac_fn, nr: int, *, next_q: bool):
        self.nq = int(nq)
        self.value_fn = value_fn
        self.jac_fn = jac_fn
        self.next_q = bool(next_q)
        super().__init__(state, int(nr), int(nu), True, False, self.next_q)

    def _q_eval(self, x: np.ndarray, u: Optional[np.ndarray]) -> np.ndarray:
        q = np.asarray(x, dtype=float)[: self.nq]
        if self.next_q:
            if u is None:
                u = np.zeros(self.nq)
            q = q + np.asarray(u, dtype=float).reshape(self.nq)
        return q

    def evaluate(self, x: np.ndarray, u: Optional[np.ndarray] = None):
        q = self._q_eval(x, u)
        margin = np.asarray(self.value_fn(q), dtype=float).reshape(-1)
        jac_q = np.asarray(self.jac_fn(q), dtype=float).reshape(margin.size, self.nq)

        rx = np.zeros((margin.size, 2 * self.nq), dtype=float)
        ru = np.zeros((margin.size, self.nq), dtype=float)
        rx[:, : self.nq] = jac_q
        if self.next_q:
            ru[:, :] = jac_q
        return margin, rx, ru

    def calc(self, data, x, u=None):
        margin, _, _ = self.evaluate(x, u)
        data.r = margin

    def calcDiff(self, data, x, u=None):
        _, rx, ru = self.evaluate(x, u)
        data.Rx = rx
        data.Ru = ru


class _KinematicActionModel(crocoddyl.ActionModelAbstract):
    """Discrete kinematic action model used by CrocoddylRunner."""

    def __init__(
        self,
        runner: "CrocoddylRunner",
        *,
        terminal: bool,
        target: np.ndarray,
        target_rotation: np.ndarray,
        q_ref: np.ndarray,
    ):
        self.runner = runner
        self.nq = runner.nq
        self.terminal = bool(terminal)
        self.target = np.asarray(target, dtype=float).reshape(3)
        self.target_rotation = np.asarray(target_rotation, dtype=float).reshape(3, 3)
        self.q_ref = np.asarray(q_ref, dtype=float).reshape(self.nq)
        self._eps = float(os.environ.get("CROCODDYL_FD_EPS", "1e-6"))

        state = runner.state
        ng = 2 * self.nq + runner.margin_dim
        if not self.terminal:
            ng += 2 * self.nq
        super().__init__(state, self.nq, 1, ng, 0)
        self.g_lb = np.zeros(ng, dtype=float)
        self.g_ub = np.full(ng, np.inf, dtype=float)

        if runner.margin_dim > 0:
            self.obstacle_residual = _ObstacleMarginResidual(
                state,
                self.nq,
                self.nq,
                runner.margin_value_fn,
                runner.margin_jac_fn,
                runner.margin_dim,
                next_q=not self.terminal,
            )
        else:
            self.obstacle_residual = None

    def update_references(self, target: np.ndarray, target_rotation: np.ndarray, q_ref: np.ndarray) -> None:
        self.target = np.asarray(target, dtype=float).reshape(3)
        self.target_rotation = np.asarray(target_rotation, dtype=float).reshape(3, 3)
        self.q_ref = np.asarray(q_ref, dtype=float).reshape(self.nq)

    def _q_eval(self, x: np.ndarray, u: Optional[np.ndarray]) -> np.ndarray:
        q = np.asarray(x, dtype=float)[: self.nq]
        if not self.terminal:
            if u is None:
                u = np.zeros(self.nq)
            q = q + np.asarray(u, dtype=float).reshape(self.nq)
        return q

    def _tracking_residual(self, q: np.ndarray) -> np.ndarray:
        pos, rot = self.runner.mpc.forward(q)
        pos_error = pos - self.target
        rot_error = self.runner.mpc._orientation_error(rot, self.target_rotation)
        return np.concatenate([pos_error, rot_error])

    def _tracking_jacobian(self, q: np.ndarray) -> np.ndarray:
        base = self._tracking_residual(q)
        jac = np.zeros((base.size, self.nq), dtype=float)
        for j in range(self.nq):
            step = np.zeros(self.nq, dtype=float)
            step[j] = self._eps
            qp = np.clip(q + step, self.runner.q_lower, self.runner.q_upper)
            qm = np.clip(q - step, self.runner.q_lower, self.runner.q_upper)
            denom = qp[j] - qm[j]
            if abs(denom) < 1e-12:
                continue
            jac[:, j] = (self._tracking_residual(qp) - self._tracking_residual(qm)) / denom
        return jac

    def _cost_terms(self, x: np.ndarray, u: Optional[np.ndarray]):
        cfg = self.runner.config
        q = self._q_eval(x, u)
        track = self._tracking_residual(q)
        j_track_q = self._tracking_jacobian(q)

        rows = []
        jx_rows = []
        ju_rows = []
        weights = []

        def append_block(residual, jac_q, weight_values, *, via_next_q: bool = True):
            residual = np.asarray(residual, dtype=float).reshape(-1)
            jac_q = np.asarray(jac_q, dtype=float).reshape(residual.size, self.nq)
            jx = np.zeros((residual.size, 2 * self.nq), dtype=float)
            ju = np.zeros((residual.size, self.nq), dtype=float)
            jx[:, : self.nq] = jac_q
            if via_next_q and not self.terminal:
                ju[:, :] = jac_q
            rows.append(residual)
            jx_rows.append(jx)
            ju_rows.append(ju)
            weights.append(np.asarray(weight_values, dtype=float).reshape(residual.size))

        if self.terminal:
            append_block(track[:3], j_track_q[:3], np.full(3, cfg.terminal_position_weight))
            append_block(track[3:], j_track_q[3:], np.full(3, cfg.terminal_orientation_weight))
            append_block(q - self.q_ref, np.eye(self.nq), np.full(self.nq, cfg.current_posture_weight))
            append_block(q - self.runner.mpc.q_center, np.eye(self.nq), np.full(self.nq, cfg.center_posture_weight))
        else:
            u_vec = np.zeros(self.nq) if u is None else np.asarray(u, dtype=float).reshape(self.nq)
            dq_prev = np.asarray(x, dtype=float)[self.nq : 2 * self.nq]
            append_block(track[:3], j_track_q[:3], np.full(3, cfg.position_weight))
            append_block(track[3:], j_track_q[3:], np.full(3, cfg.orientation_weight))

            jx_zero = np.zeros((self.nq, 2 * self.nq), dtype=float)
            ju_eye = np.eye(self.nq)
            rows.append(u_vec.copy())
            jx_rows.append(jx_zero.copy())
            ju_rows.append(ju_eye.copy())
            weights.append(np.full(self.nq, cfg.joint_delta_weight, dtype=float))

            jx_du = np.zeros((self.nq, 2 * self.nq), dtype=float)
            jx_du[:, self.nq : 2 * self.nq] = -np.eye(self.nq)
            rows.append(u_vec - dq_prev)
            jx_rows.append(jx_du)
            ju_rows.append(ju_eye.copy())
            weights.append(np.full(self.nq, cfg.joint_delta_change_weight, dtype=float))

            append_block(q - self.q_ref, np.eye(self.nq), np.full(self.nq, cfg.current_posture_weight))
            append_block(q - self.runner.mpc.q_center, np.eye(self.nq), np.full(self.nq, cfg.center_posture_weight))

        residual = np.concatenate(rows) if rows else np.zeros(0, dtype=float)
        jx = np.vstack(jx_rows) if jx_rows else np.zeros((0, 2 * self.nq), dtype=float)
        ju = np.vstack(ju_rows) if ju_rows else np.zeros((0, self.nq), dtype=float)
        weight = np.concatenate(weights) if weights else np.zeros(0, dtype=float)
        return residual, jx, ju, weight

    def _constraints(self, x: np.ndarray, u: Optional[np.ndarray]):
        q = self._q_eval(x, u)
        values = []
        gx_rows = []
        gu_rows = []

        def add(residual, gx_q, gu_q=None):
            residual = np.asarray(residual, dtype=float).reshape(-1)
            gx_q = np.asarray(gx_q, dtype=float).reshape(residual.size, self.nq)
            gx = np.zeros((residual.size, 2 * self.nq), dtype=float)
            gu = np.zeros((residual.size, self.nq), dtype=float)
            gx[:, : self.nq] = gx_q
            if gu_q is not None:
                gu[:, :] = np.asarray(gu_q, dtype=float).reshape(residual.size, self.nq)
            values.append(residual)
            gx_rows.append(gx)
            gu_rows.append(gu)

        eye = np.eye(self.nq)
        if self.terminal:
            add(q - self.runner.q_lower, eye)
            add(self.runner.q_upper - q, -eye)
        else:
            u_vec = np.zeros(self.nq) if u is None else np.asarray(u, dtype=float).reshape(self.nq)
            add(q - self.runner.q_lower, eye, eye)
            add(self.runner.q_upper - q, -eye, -eye)
            add(np.full(self.nq, self.runner.max_joint_step) - u_vec, np.zeros((self.nq, self.nq)), -eye)
            add(np.full(self.nq, self.runner.max_joint_step) + u_vec, np.zeros((self.nq, self.nq)), eye)

        if self.obstacle_residual is not None:
            margin, rx, ru = self.obstacle_residual.evaluate(x, u)
            values.append(margin)
            gx_rows.append(rx)
            gu_rows.append(ru)

        if not values:
            return (
                np.zeros(0, dtype=float),
                np.zeros((0, 2 * self.nq), dtype=float),
                np.zeros((0, self.nq), dtype=float),
            )
        return np.concatenate(values), np.vstack(gx_rows), np.vstack(gu_rows)

    def calc(self, data, x, u=None):
        x = np.asarray(x, dtype=float).reshape(2 * self.nq)
        if u is None:
            u = np.zeros(self.nq, dtype=float)
        else:
            u = np.asarray(u, dtype=float).reshape(self.nq)

        if self.terminal:
            data.xnext = x.copy()
        else:
            q_next = x[: self.nq] + u
            data.xnext = np.concatenate([q_next, u])

        residual, _, _, weight = self._cost_terms(x, None if self.terminal else u)
        data.r = residual
        data.cost = float(np.sum(weight * residual * residual))
        g, _, _ = self._constraints(x, None if self.terminal else u)
        data.g = g

    def calcDiff(self, data, x, u=None):
        x = np.asarray(x, dtype=float).reshape(2 * self.nq)
        if u is None:
            u = np.zeros(self.nq, dtype=float)
        else:
            u = np.asarray(u, dtype=float).reshape(self.nq)

        if self.terminal:
            data.Fx = np.eye(2 * self.nq)
            data.Fu = np.zeros((2 * self.nq, self.nq), dtype=float)
        else:
            fx = np.zeros((2 * self.nq, 2 * self.nq), dtype=float)
            fu = np.zeros((2 * self.nq, self.nq), dtype=float)
            fx[: self.nq, : self.nq] = np.eye(self.nq)
            fu[: self.nq, :] = np.eye(self.nq)
            fu[self.nq :, :] = np.eye(self.nq)
            data.Fx = fx
            data.Fu = fu

        residual, jx, ju, weight = self._cost_terms(x, None if self.terminal else u)
        wr = weight * residual
        wj = weight.reshape(-1, 1) * jx
        data.Lx = 2.0 * (jx.T @ wr)
        data.Lxx = 2.0 * (jx.T @ wj)
        if self.terminal:
            data.Lu = np.zeros(self.nq, dtype=float)
            data.Luu = np.eye(self.nq) * 1e-9
            data.Lxu = np.zeros((2 * self.nq, self.nq), dtype=float)
        else:
            wju = weight.reshape(-1, 1) * ju
            data.Lu = 2.0 * (ju.T @ wr)
            data.Luu = 2.0 * (ju.T @ wju) + np.eye(self.nq) * 1e-9
            data.Lxu = 2.0 * (jx.T @ wju)

        g, gx, gu = self._constraints(x, None if self.terminal else u)
        data.g = g
        data.Gx = gx
        data.Gu = gu


class CrocoddylRunner:
    """MpcPinocchioIK.step-compatible Crocoddyl CSQP runner."""

    def __init__(self, urdf_path: str, ee_frame: str = "F_Link", config=None):
        from mpc_pinocchio_ik import MpcPinocchioIK

        self.mpc = MpcPinocchioIK(urdf_path=urdf_path, ee_frame=ee_frame, config=config)
        self.config = self.mpc.config
        self.nq = self.mpc.nq
        self.state = crocoddyl.StateVector(2 * self.nq)
        self.max_joint_step = float(self.config.max_joint_step)

        fns, m = self.mpc._obstacle_margin_functions()
        self.margin_dim = int(m)
        if fns is None:
            self.margin_value_fn = None
            self.margin_jac_fn = None
        else:
            self.margin_value_fn, self.margin_jac_fn = fns

        horizon = int(self.config.horizon_steps)
        q0 = np.clip(np.zeros(self.nq, dtype=float), self.q_lower, self.q_upper)
        target0 = np.zeros(3, dtype=float)
        rot0 = np.eye(3, dtype=float)
        self.running_models = [
            _KinematicActionModel(self, terminal=False, target=target0, target_rotation=rot0, q_ref=q0)
            for _ in range(horizon)
        ]
        self.terminal_model = _KinematicActionModel(
            self, terminal=True, target=target0, target_rotation=rot0, q_ref=q0
        )
        self.problem = crocoddyl.ShootingProblem(
            np.concatenate([q0, np.zeros(self.nq, dtype=float)]),
            self.running_models,
            self.terminal_model,
        )
        self.solver = mim_solvers.SolverCSQP(self.problem)
        self.solver.termination_tolerance = float(os.environ.get("CROCODDYL_TERMINATION_TOL", "1e-4"))
        self.solver.eps_abs = float(os.environ.get("CROCODDYL_EPS_ABS", "1e-8"))
        self.solver.eps_rel = float(os.environ.get("CROCODDYL_EPS_REL", "0.0"))
        self.solver.max_qp_iters = int(os.environ.get("CROCODDYL_MAX_QP_ITERS", "5000"))
        self.solver.reset_y = False
        self.solver.reset_rho = False
        max_solve_time = float(os.environ.get("CROCODDYL_MAX_SOLVE_TIME", "0.25"))
        if max_solve_time > 0.0:
            self.solver.max_solve_time = max_solve_time
        self._max_iter = int(os.environ.get("CROCODDYL_MAXITER", str(max(1, self.config.max_iter))))
        self._last_xs = None
        self._last_us = None

    # ---- MpcPinocchioIK geometry/diagnostic proxies ----
    def forward(self, q):
        return self.mpc.forward(q)

    @property
    def model(self):
        return self.mpc.model

    @property
    def data(self):
        return self.mpc.data

    @property
    def frame_id(self):
        return self.mpc.frame_id

    @property
    def q_lower(self):
        return self.mpc.q_lower

    @property
    def q_upper(self):
        return self.mpc.q_upper

    def _obstacle_diagnostics(self, plan):
        return self.mpc._obstacle_diagnostics(plan)

    def obstacle_disks(self):
        return self.mpc.obstacle_disks()

    def body_obstacle_boxes(self):
        return self.mpc.body_obstacle_boxes()

    def capsule_sample_positions(self, q):
        return self.mpc.capsule_sample_positions(q)

    def capsule_sample_radii(self):
        return self.mpc.capsule_sample_radii()

    def _initial_candidate(self, q0: np.ndarray, warm_start: Optional[np.ndarray]):
        plan = self.mpc._initial_plan(q0, warm_start)
        xs = [np.concatenate([q0, np.zeros(self.nq, dtype=float)])]
        us = []
        previous = q0
        for q in plan:
            dq = np.clip(q - previous, -self.max_joint_step, self.max_joint_step)
            q_next = np.clip(previous + dq, self.q_lower, self.q_upper)
            us.append(dq.copy())
            xs.append(np.concatenate([q_next, dq]))
            previous = q_next
        return xs, us

    def _update_problem(self, q0: np.ndarray, targets: np.ndarray, rotations: np.ndarray) -> None:
        self.problem.x0 = np.concatenate([q0, np.zeros(self.nq, dtype=float)])
        for i, model in enumerate(self.running_models):
            model.update_references(targets[i], rotations[i], q0)
        self.terminal_model.update_references(targets[-1], rotations[-1], q0)

    def _position_error(self, q: np.ndarray, target: np.ndarray) -> float:
        pos, _ = self.mpc.forward(q)
        return float(np.linalg.norm(pos - target))

    def step(
        self,
        target_position: np.ndarray,
        q_current: np.ndarray,
        warm_start: Optional[np.ndarray] = None,
        target_rotation: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, bool, Dict[str, object]]:
        q0 = self.mpc._initial_q(q_current)
        targets = self.mpc._target_sequence(target_position)
        rotations, orientation_active = self.mpc._rotation_sequence(target_rotation, q0)
        self._update_problem(q0, targets, rotations)
        xs_init, us_init = self._initial_candidate(q0, warm_start)

        start_time = time.perf_counter()
        success = bool(self.solver.solve(xs_init, us_init, self._max_iter))
        solve_time = time.perf_counter() - start_time
        max_solve_time_reached = bool(getattr(self.solver, "max_solve_time_reached", False))
        if max_solve_time_reached:
            success = False

        try:
            xs = np.asarray(self.solver.xs, dtype=float)
            plan = xs[1 : self.config.horizon_steps + 1, : self.nq].copy()
        except Exception:
            plan = np.asarray([x[: self.nq] for x in xs_init[1:]], dtype=float)
            success = False
        if plan.shape != (self.config.horizon_steps, self.nq) or not np.all(np.isfinite(plan)):
            plan = np.asarray([x[: self.nq] for x in xs_init[1:]], dtype=float)
            success = False
        plan = np.clip(plan, self.q_lower, self.q_upper)

        q_next = plan[0].copy()
        terminal_error = self._position_error(plan[-1], targets[-1])
        first_error = self._position_error(q_next, targets[0])
        plan_feasible = self.mpc._plan_feasible(q0, plan)
        tracking_ok = terminal_error <= self.config.success_tolerance
        ok = bool(tracking_ok and plan_feasible)
        command_usable = bool(plan_feasible and (success or tracking_ok))

        terminal_orientation_error = None
        if orientation_active:
            _, terminal_rotation = self.mpc.forward(plan[-1])
            terminal_orientation_error = float(
                np.linalg.norm(self.mpc._orientation_error(terminal_rotation, rotations[-1]))
            )

        try:
            nit = int(getattr(self.solver, "iter", 0))
        except Exception:
            nit = 0
        status = 0 if success else (2 if max_solve_time_reached else 1)
        info: Dict[str, object] = {
            "solve_time": solve_time,
            "terminal_error": float(terminal_error),
            "first_error": float(first_error),
            "terminal_orientation_error": terminal_orientation_error,
            "trajectory": plan.copy(),
            "target_trajectory": targets.copy(),
            "target_rotation_trajectory": rotations.copy(),
            "status": status,
            "success": bool(success),
            "command_usable": command_usable,
            "nit": nit,
            "solver_backend": "crocoddyl",
            "tracking_ok": bool(tracking_ok),
            "plan_feasible": bool(plan_feasible),
            "constraint_norm": float(getattr(self.solver, "constraint_norm", np.nan)),
            "gap_norm": float(getattr(self.solver, "gap_norm", np.nan)),
            "KKT": float(getattr(self.solver, "KKT", np.nan)),
            "max_solve_time_reached": max_solve_time_reached,
            "orientation_active": bool(orientation_active),
        }
        info.update(self.mpc._obstacle_diagnostics(plan))
        self._last_xs = [np.asarray(x, dtype=float).copy() for x in getattr(self.solver, "xs", xs_init)]
        self._last_us = [np.asarray(u, dtype=float).copy() for u in getattr(self.solver, "us", us_init)]
        self.mpc._last_plan = plan.copy()
        return q_next, ok, info


def build_runner_from_config(config, urdf_path: str, ee_frame: str = "F_Link") -> CrocoddylRunner:
    return CrocoddylRunner(urdf_path=urdf_path, ee_frame=ee_frame, config=config)
