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
    # 非零笛卡尔导数权重时，每个预测 stage 在上一条名义轨迹上用 Pinocchio
    # 重新线性化；权重全零时跳过。原始 nonlinear 图保留用于结构 A/B。
    cartesian_residual_mode: str = "external_affine"
    cartesian_velocity_weight: float = 0.0
    # 结构已合并；2 Hz 原始 a_ref 经投影后的残差仍较大，A/B 显示非零权重
    # 会增大横向误差，因此生产默认先为 0，保留显式实验开关。
    cartesian_acceleration_weight: float = 0.0
    terminal_cartesian_velocity_weight: float = 0.0
    # 绘图时优先约束几何轮廓，允许沿线方向产生小量相位滞后。抬笔/换面阶段
    # 自动退回普通定时点跟踪。
    contour_tracking_enabled: bool = True
    contour_position_weight: float = 5000.0
    lag_position_weight: float = 750.0
    transition_position_weight: float = 5000.0
    terminal_contour_position_weight: float = 500.0
    terminal_lag_position_weight: float = 75.0
    terminal_transition_position_weight: float = 500.0
    # 固定硬约束内的参考投影保留作显式实验；生产默认关闭，避免投影后的
    # 导数与未投影的定时位置参考互相冲突。
    feasible_reference_projection: bool = False
    feasible_reference_damping: float = 1e-4
    joint_velocity_weight: float = 0.02
    # 投影开启时围绕可行 qdot_ff/qddot_ff；投影关闭时参考为零。
    joint_acceleration_weight: float = 0.0
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
        if cfg.cartesian_residual_mode not in ("external_affine", "nonlinear"):
            raise ValueError(
                "cartesian_residual_mode 必须是 'external_affine' 或 'nonlinear'")
        if cfg.min_translation_jacobian_sigma < 0.0:
            raise ValueError("min_translation_jacobian_sigma 必须 >= 0")
        weights = (
            cfg.cartesian_velocity_weight,
            cfg.cartesian_acceleration_weight,
            cfg.terminal_cartesian_velocity_weight,
            cfg.contour_position_weight,
            cfg.lag_position_weight,
            cfg.transition_position_weight,
            cfg.terminal_contour_position_weight,
            cfg.terminal_lag_position_weight,
            cfg.terminal_transition_position_weight,
        )
        if not np.all(np.isfinite(weights)) or min(weights) < 0.0:
            raise ValueError("笛卡尔/轮廓权重必须有限且 >= 0")
        if (
            not np.isfinite(cfg.feasible_reference_damping)
            or cfg.feasible_reference_damping <= 0.0
        ):
            raise ValueError("feasible_reference_damping 必须有限且 > 0")
        super().__init__(*args, config=cfg, **kwargs)
        self.config: SecondOrderMpcConfig
        self._second_order_backend = None
        self._last_velocity_plan = None
        self._last_acceleration_plan = None

    def reset_warm_start(self):
        """Forget all coupled q/dq/ddq plans after an external safety override."""
        self._last_plan = None
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
        target_velocity: Optional[np.ndarray] = None,
        target_acceleration: Optional[np.ndarray] = None,
        contour_tangent: Optional[np.ndarray] = None,
        contour_active: Optional[np.ndarray] = None,
        position_weight_scale: float = 1.0,
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
        target_velocities = self._cartesian_reference_sequence(
            target_velocity, "target_velocity")
        target_accelerations = self._cartesian_reference_sequence(
            target_acceleration, "target_acceleration")
        contour_tangents = self._cartesian_reference_sequence(
            contour_tangent, "contour_tangent")
        contour_active_sequence = self._scalar_reference_sequence(
            contour_active, "contour_active")
        position_weight_scale = float(position_weight_scale)
        if (not np.isfinite(position_weight_scale)
                or not 0.0 < position_weight_scale <= 1.0):
            raise ValueError(
                "position_weight_scale 必须是 (0,1] 内的有限标量")
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
            q0, v0, targets, rotations,
            target_velocities, target_accelerations,
            contour_tangents, contour_active_sequence,
            q_guess, q_ref, position_weight_scale)
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
            "linearization_time": result["linearization_time"],
            "projection_time": result["projection_time"],
            "total_optimize_time": result["total_optimize_time"],
            "cartesian_residual_mode": result["cartesian_residual_mode"],
            "feasible_reference_projection": bool(
                self.config.feasible_reference_projection),
            "projection_velocity_residual_rms": result[
                "projection_velocity_residual_rms"],
            "projection_acceleration_residual_rms": result[
                "projection_acceleration_residual_rms"],
            "projection_velocity_active_ratio": result[
                "projection_velocity_active_ratio"],
            "projection_acceleration_active_ratio": result[
                "projection_acceleration_active_ratio"],
            "position_weight_scale": result["position_weight_scale"],
            "nit": result["nit"],
            "trajectory": q_plan.copy(),
            "velocity_trajectory": v_plan.copy(),
            "acceleration_trajectory": a_plan.copy(),
            "ddq0": a_plan[0].copy(),
            "state_definition": "[q,dq]",
            "control_definition": "ddq",
        }
        return q_next, dq_next, bool(tracking_ok and command_usable), info

    def _cartesian_reference_sequence(
        self, value: Optional[np.ndarray], name: str,
    ) -> np.ndarray:
        """Expand one Cartesian reference or validate an N-step sequence."""
        if value is None:
            return np.zeros((self.config.horizon_steps, 3), dtype=float)
        array = np.asarray(value, dtype=float)
        if array.shape == (3,):
            array = np.repeat(
                array.reshape(1, 3), self.config.horizon_steps, axis=0)
        expected = (self.config.horizon_steps, 3)
        if array.shape != expected:
            raise ValueError(
                f"{name} 形状必须是 (3,) 或 {expected}, 当前是 {array.shape}")
        if not np.all(np.isfinite(array)):
            raise ValueError(f"{name} 含 NaN/Inf")
        return array

    def _scalar_reference_sequence(
        self, value: Optional[np.ndarray], name: str,
    ) -> np.ndarray:
        if value is None:
            return np.zeros(self.config.horizon_steps, dtype=float)
        array = np.asarray(value, dtype=float)
        if array.shape == ():
            array = np.full(self.config.horizon_steps, float(array))
        expected = (self.config.horizon_steps,)
        if array.shape != expected or not np.all(np.isfinite(array)):
            raise ValueError(f"{name} 必须是有限标量或 {expected}")
        if np.any((array < 0.0) | (array > 1.0)):
            raise ValueError(f"{name} 必须在 [0,1]")
        return array

    def cartesian_affine_linearization(
        self,
        data: pin.Data,
        q: np.ndarray,
        dq: np.ndarray,
        ddq: np.ndarray,
        target_velocity: np.ndarray,
        target_acceleration: np.ndarray,
    ) -> dict:
        """Pinocchio point kinematics and their contracted first derivatives."""
        q = np.asarray(q, dtype=float).reshape(self.nq)
        dq = np.asarray(dq, dtype=float).reshape(self.nq)
        ddq = np.asarray(ddq, dtype=float).reshape(self.nq)
        target_velocity = np.asarray(target_velocity, dtype=float).reshape(3)
        target_acceleration = np.asarray(target_acceleration, dtype=float).reshape(3)
        if not all(np.all(np.isfinite(value)) for value in (
                q, dq, ddq, target_velocity, target_acceleration)):
            raise ValueError("Cartesian affine linearization input contains NaN/Inf")

        frame = self.model.frames[self.frame_id]
        reference = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        pin.computeForwardKinematicsDerivatives(
            self.model, data, q, dq, ddq)
        velocity_dq, velocity_ddq = pin.getPointVelocityDerivatives(
            self.model, data, frame.parentJoint, frame.placement, reference)
        _, acceleration_dq, acceleration_ddq, acceleration_dddq = (
            pin.getPointClassicAccelerationDerivatives(
                self.model, data, frame.parentJoint, frame.placement, reference))
        pin.updateFramePlacements(self.model, data)
        velocity = np.asarray(pin.getFrameVelocity(
            self.model, data, self.frame_id, reference).linear, dtype=float)
        acceleration = np.asarray(pin.getFrameClassicalAcceleration(
            self.model, data, self.frame_id, reference).linear, dtype=float)
        result = {
            "q_bar": q.copy(),
            "dq_bar": dq.copy(),
            "ddq_bar": ddq.copy(),
            "r_v_bar": velocity - target_velocity,
            "r_a_bar": acceleration - target_acceleration,
            "A_v": np.asarray(velocity_dq, dtype=float).copy(),
            "B_v": np.asarray(velocity_ddq, dtype=float).copy(),
            "A_a": np.asarray(acceleration_dq, dtype=float).copy(),
            "B_a": np.asarray(acceleration_ddq, dtype=float).copy(),
            "C_a": np.asarray(acceleration_dddq, dtype=float).copy(),
            "velocity": velocity,
            "acceleration": acceleration,
        }
        expected_matrix_shape = (3, self.nq)
        for name in ("A_v", "B_v", "A_a", "B_a", "C_a"):
            if result[name].shape != expected_matrix_shape:
                raise RuntimeError(
                    f"Pinocchio {name} shape {result[name].shape}, "
                    f"expected {expected_matrix_shape}")
        if not all(np.all(np.isfinite(value)) for value in result.values()):
            raise RuntimeError("Pinocchio Cartesian affine coefficients contain NaN/Inf")
        return result

    @staticmethod
    def _box_damped_least_squares(
        matrix: np.ndarray,
        target: np.ndarray,
        lower: np.ndarray,
        upper: np.ndarray,
        damping: float,
    ) -> np.ndarray:
        """Small deterministic active-set projection for a 3-by-7 task."""
        matrix = np.asarray(matrix, dtype=float)
        target = np.asarray(target, dtype=float)
        lower = np.asarray(lower, dtype=float)
        upper = np.asarray(upper, dtype=float)
        n = matrix.shape[1]
        value = np.zeros(n, dtype=float)
        free = np.ones(n, dtype=bool)
        for _ in range(n + 1):
            if np.any(free):
                fixed = ~free
                rhs = target - matrix[:, fixed] @ value[fixed]
                mf = matrix[:, free]
                lhs = mf.T @ mf + damping * np.eye(int(np.sum(free)))
                value[free] = np.linalg.solve(lhs, mf.T @ rhs)
            violation_low = lower - value
            violation_high = value - upper
            violation = np.maximum(violation_low, violation_high)
            violation[~free] = -np.inf
            index = int(np.argmax(violation))
            if not np.isfinite(violation[index]) or violation[index] <= 1e-12:
                break
            value[index] = (
                lower[index]
                if violation_low[index] >= violation_high[index]
                else upper[index]
            )
            free[index] = False
        return np.clip(value, lower, upper)

    def project_feasible_reference(
        self,
        data: pin.Data,
        q: np.ndarray,
        dq_nominal: np.ndarray,
        target_velocity: np.ndarray,
        target_acceleration: np.ndarray,
    ) -> dict:
        """Project raw Cartesian derivatives into the unchanged joint limits."""
        q = np.asarray(q, dtype=float).reshape(self.nq)
        dq_nominal = np.asarray(dq_nominal, dtype=float).reshape(self.nq)
        target_velocity = np.asarray(target_velocity, dtype=float).reshape(3)
        target_acceleration = np.asarray(target_acceleration, dtype=float).reshape(3)
        reference = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        jacobian = pin.computeFrameJacobian(
            self.model, data, q, self.frame_id, reference)[:3, :]
        vmax = float(self.config.max_joint_velocity)
        amax = float(self.config.max_joint_acceleration)
        damping = float(self.config.feasible_reference_damping)
        qdot_ff = self._box_damped_least_squares(
            jacobian, target_velocity,
            np.full(self.nq, -vmax), np.full(self.nq, vmax), damping)

        zero = np.zeros(self.nq)
        pin.forwardKinematics(self.model, data, q, qdot_ff, zero)
        pin.updateFramePlacements(self.model, data)
        jdot_qdot = np.asarray(pin.getFrameClassicalAcceleration(
            self.model, data, self.frame_id, reference).linear, dtype=float)
        dt = float(self.config.control_dt)
        lower = np.maximum(-amax, (-vmax - dq_nominal) / dt)
        upper = np.minimum(amax, (vmax - dq_nominal) / dt)
        qddot_ff = self._box_damped_least_squares(
            jacobian, target_acceleration - jdot_qdot,
            lower, upper, damping)
        velocity_feasible = jacobian @ qdot_ff
        acceleration_feasible = jacobian @ qddot_ff + jdot_qdot
        return {
            "velocity": velocity_feasible,
            "acceleration": acceleration_feasible,
            "qdot": qdot_ff,
            "qddot": qddot_ff,
            "velocity_residual": float(np.linalg.norm(
                velocity_feasible - target_velocity)),
            "acceleration_residual": float(np.linalg.norm(
                acceleration_feasible - target_acceleration)),
            "velocity_active_ratio": float(np.mean(
                np.abs(qdot_ff) >= vmax - 1e-6)),
            "acceleration_active_ratio": float(np.mean(
                (np.abs(qddot_ff - lower) <= 1e-6)
                | (np.abs(qddot_ff - upper) <= 1e-6))),
        }

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
        self.cartesian_residual_mode = cfg.cartesian_residual_mode
        if self.cartesian_residual_mode == "external_affine":
            # target/rotation, posture refs, nominal q/dq/ddq, nominal Cartesian
            # residuals, five 3xn Jacobian blocks, feasible joint refs, contour data.
            np_param = (
                3 + 9 + 2 * nq + 3 * nq + 6 + 5 * 3 * nq
                + 2 * nq + 3 + 1
            )
            self._linearization_data = [
                owner.model.createData() for _ in range(cfg.horizon_steps + 1)]
            self._projection_data = [
                owner.model.createData() for _ in range(cfg.horizon_steps + 1)]
        else:
            np_param = 3 + 9 + 3 + 3 + 2 * nq + 2 * nq + 3 + 1
            self._linearization_data = []
            self._projection_data = [
                owner.model.createData() for _ in range(cfg.horizon_steps + 1)]
        self.np_param = np_param
        dt = float(cfg.control_dt)

        x = ca.MX.sym("x", nx)
        u = ca.MX.sym("u", nu)
        p = ca.MX.sym("p", np_param)
        q, dq = x[:nq], x[nq:]
        target = p[:3]
        target_rot = ca.reshape(p[3:12], 3, 3).T
        cursor = 12
        if self.cartesian_residual_mode == "external_affine":
            q_ref = p[cursor:cursor + nq]
            cursor += nq
            q_center = p[cursor:cursor + nq]
            cursor += nq
            q_bar = p[cursor:cursor + nq]
            cursor += nq
            dq_bar = p[cursor:cursor + nq]
            cursor += nq
            ddq_bar = p[cursor:cursor + nq]
            cursor += nq
            r_v_bar = p[cursor:cursor + 3]
            cursor += 3
            r_a_bar = p[cursor:cursor + 3]
            cursor += 3

            def unpack_3xn():
                nonlocal cursor
                matrix = ca.reshape(
                    p[cursor:cursor + 3 * nq], nq, 3).T
                cursor += 3 * nq
                return matrix

            A_v = unpack_3xn()
            B_v = unpack_3xn()
            A_a = unpack_3xn()
            B_a = unpack_3xn()
            C_a = unpack_3xn()
        else:
            target_velocity = p[cursor:cursor + 3]
            cursor += 3
            target_acceleration = p[cursor:cursor + 3]
            cursor += 3
            q_ref = p[cursor:cursor + nq]
            cursor += nq
            q_center = p[cursor:cursor + nq]
            cursor += nq
        qdot_ff = p[cursor:cursor + nq]
        cursor += nq
        qddot_ff = p[cursor:cursor + nq]
        cursor += nq
        contour_tangent = p[cursor:cursor + 3]
        cursor += 3
        contour_active = p[cursor]
        cursor += 1
        if cursor != np_param:
            raise RuntimeError(
                f"second-order parameter layout ended at {cursor}, "
                f"expected {np_param}")

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
        tangent = contour_tangent / ca.sqrt(
            ca.dot(contour_tangent, contour_tangent) + 1e-12)
        tangent_projection = ca.mtimes(tangent, tangent.T)
        contour_error = contour_active * ca.mtimes(
            ca.MX.eye(3) - tangent_projection, pos_error)
        lag_error = contour_active * ca.dot(tangent, pos_error)
        timed_position_error = (1.0 - contour_active) * pos_error
        if self.cartesian_residual_mode == "external_affine":
            velocity_error = (
                r_v_bar
                + ca.mtimes(A_v, q - q_bar)
                + ca.mtimes(B_v, dq - dq_bar)
            )
            acceleration_error = (
                r_a_bar
                + ca.mtimes(A_a, q - q_bar)
                + ca.mtimes(B_a, dq - dq_bar)
                + ca.mtimes(C_a, u - ddq_bar)
            )
        else:
            ee_velocity = ca.mtimes(position_jacobian, dq)
            q_direction = ca.vertcat(dq, ca.MX.zeros(nq, 1))
            jacobian_dot_velocity = ca.jtimes(
                ee_velocity, x, q_direction)
            ee_acceleration = (
                ca.mtimes(position_jacobian, u) + jacobian_dot_velocity)
            velocity_error = ee_velocity - target_velocity
            acceleration_error = ee_acceleration - target_acceleration
        model.cost_y_expr = ca.vertcat(
            contour_error, lag_error, timed_position_error,
            rot_error, velocity_error, acceleration_error,
            dq - qdot_ff, u - qddot_ff, q - q_ref, q - q_center)
        model.cost_y_expr_e = ca.vertcat(
            contour_error, lag_error, timed_position_error,
            rot_error, velocity_error,
            dq - qdot_ff, q - q_ref, q - q_center)

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
            np.full(3, cfg.contour_position_weight),
            np.array([cfg.lag_position_weight]),
            np.full(3, cfg.transition_position_weight),
            np.full(3, cfg.orientation_weight),
            np.full(3, cfg.cartesian_velocity_weight),
            np.full(3, cfg.cartesian_acceleration_weight),
            np.full(nq, cfg.joint_velocity_weight),
            np.full(nq, cfg.joint_acceleration_weight),
            np.full(nq, cfg.current_posture_weight),
            np.full(nq, cfg.center_posture_weight),
        ]))
        W_e = np.diag(np.concatenate([
            np.full(3, cfg.terminal_contour_position_weight),
            np.array([cfg.terminal_lag_position_weight]),
            np.full(3, cfg.terminal_transition_position_weight),
            np.full(3, cfg.terminal_orientation_weight),
            np.full(3, cfg.terminal_cartesian_velocity_weight),
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
            "model": "double_integrator_zoh_v5_feasible_contouring",
            "cartesian_residual_mode": cfg.cartesian_residual_mode,
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
        self.generated_library_path = library
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
        self._W_base = W.copy()
        self._W_e_base = W_e.copy()
        self._position_weight_scale = 1.0

    def _apply_position_weight_scale(self, scale: float):
        """按末端预测风险缩放位置相关代价，不改变 OCP 结构或硬约束。"""
        scale = float(scale)
        if abs(scale - self._position_weight_scale) <= 1.0e-12:
            return
        W = self._W_base.copy()
        W_e = self._W_e_base.copy()
        # cost_y 的前 7 维依次是 3D 轮廓误差、1D 滞后误差和 3D 定时误差。
        W[:7, :7] *= scale
        W_e[:7, :7] *= scale
        for stage in range(self.owner.config.horizon_steps):
            self.solver.cost_set(stage, "W", W)
        self.solver.cost_set(self.owner.config.horizon_steps, "W", W_e)
        self._position_weight_scale = scale

    def _set_stage_parameters(
        self,
        nominal_states: np.ndarray,
        nominal_controls: np.ndarray,
        targets: np.ndarray,
        rotations: np.ndarray,
        target_velocities: np.ndarray,
        target_accelerations: np.ndarray,
        contour_tangents: np.ndarray,
        contour_active: np.ndarray,
        q_ref: np.ndarray,
    ) -> dict:
        """Project raw references, linearize every stage, then set parameters."""
        cfg = self.owner.config
        nq, horizon = self.owner.nq, cfg.horizon_steps
        if nominal_states.shape != (horizon + 1, 2 * nq):
            raise ValueError("nominal_states shape does not match horizon")
        if nominal_controls.shape != (horizon, nq):
            raise ValueError("nominal_controls shape does not match horizon")

        linearization_time = 0.0
        projection_time = 0.0
        velocity_residuals = []
        acceleration_residuals = []
        velocity_active = []
        acceleration_active = []
        affine_cost_active = any(weight > 0.0 for weight in (
            cfg.cartesian_velocity_weight,
            cfg.cartesian_acceleration_weight,
            cfg.terminal_cartesian_velocity_weight,
        ))
        for stage in range(horizon + 1):
            target_index = min(max(stage - 1, 0), horizon - 1)
            q_bar = nominal_states[stage, :nq]
            dq_bar = nominal_states[stage, nq:]
            ddq_bar = nominal_controls[min(stage, horizon - 1)]
            raw_velocity = target_velocities[target_index]
            raw_acceleration = target_accelerations[target_index]
            if cfg.feasible_reference_projection:
                started = time.perf_counter()
                feasible = self.owner.project_feasible_reference(
                    self._projection_data[stage],
                    q_bar, dq_bar, raw_velocity, raw_acceleration)
                projection_time += time.perf_counter() - started
            else:
                feasible = {
                    "velocity": raw_velocity,
                    "acceleration": raw_acceleration,
                    "qdot": np.zeros(nq),
                    "qddot": np.zeros(nq),
                    "velocity_residual": 0.0,
                    "acceleration_residual": 0.0,
                    "velocity_active_ratio": 0.0,
                    "acceleration_active_ratio": 0.0,
                }
            velocity_residuals.append(feasible["velocity_residual"])
            acceleration_residuals.append(feasible["acceleration_residual"])
            velocity_active.append(feasible["velocity_active_ratio"])
            acceleration_active.append(feasible["acceleration_active_ratio"])
            active = (
                float(contour_active[target_index])
                if cfg.contour_tracking_enabled else 0.0
            )
            tangent = np.asarray(
                contour_tangents[target_index], dtype=float).copy()
            tangent_norm = float(np.linalg.norm(tangent))
            if active <= 0.0 or tangent_norm < 1e-8:
                active = 0.0
                tangent = np.zeros(3)
            else:
                tangent /= tangent_norm

            common = [
                targets[target_index],
                rotations[target_index].reshape(-1),
            ]
            if self.cartesian_residual_mode == "external_affine":
                if affine_cost_active:
                    started = time.perf_counter()
                    terms = self.owner.cartesian_affine_linearization(
                        self._linearization_data[stage],
                        q_bar, dq_bar, ddq_bar,
                        feasible["velocity"], feasible["acceleration"])
                    linearization_time += time.perf_counter() - started
                else:
                    # These residual rows have exactly zero stage/terminal
                    # weight.  Avoid 11--21 Pinocchio derivative evaluations
                    # per tick while preserving the generated parameter layout.
                    zero_3n = np.zeros((3, nq))
                    terms = {
                        "q_bar": q_bar,
                        "dq_bar": dq_bar,
                        "ddq_bar": ddq_bar,
                        "r_v_bar": np.zeros(3),
                        "r_a_bar": np.zeros(3),
                        "A_v": zero_3n,
                        "B_v": zero_3n,
                        "A_a": zero_3n,
                        "B_a": zero_3n,
                        "C_a": zero_3n,
                    }
                p = np.concatenate(common + [
                    q_ref, self.owner.q_center,
                    terms["q_bar"], terms["dq_bar"], terms["ddq_bar"],
                    terms["r_v_bar"], terms["r_a_bar"],
                    terms["A_v"].reshape(-1),
                    terms["B_v"].reshape(-1),
                    terms["A_a"].reshape(-1),
                    terms["B_a"].reshape(-1),
                    terms["C_a"].reshape(-1),
                    feasible["qdot"], feasible["qddot"],
                    tangent, np.array([active]),
                ])
            else:
                p = np.concatenate(common + [
                    feasible["velocity"], feasible["acceleration"],
                    q_ref, self.owner.q_center,
                    feasible["qdot"], feasible["qddot"],
                    tangent, np.array([active]),
                ])
            if p.shape != (self.np_param,) or not np.all(np.isfinite(p)):
                raise RuntimeError(
                    f"stage {stage} parameter shape/value invalid: "
                    f"{p.shape}, expected {(self.np_param,)}")
            self.solver.set(stage, "p", p)
        return {
            "linearization_time": linearization_time,
            "projection_time": projection_time,
            "velocity_residual_rms": float(np.sqrt(np.mean(
                np.square(velocity_residuals)))),
            "acceleration_residual_rms": float(np.sqrt(np.mean(
                np.square(acceleration_residuals)))),
            "velocity_active_ratio": float(np.mean(velocity_active)),
            "acceleration_active_ratio": float(np.mean(acceleration_active)),
        }

    def optimize(
        self, q0, v0, targets, rotations,
        target_velocities, target_accelerations,
        contour_tangents, contour_active,
        q_guess, q_ref, position_weight_scale,
    ):
        cfg = self.owner.config
        nq, horizon = self.owner.nq, cfg.horizon_steps
        dt = float(cfg.control_dt)
        self._apply_position_weight_scale(position_weight_scale)
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
        nominal_states = np.vstack([
            x0,
            np.hstack([q_guess, v_guess]),
        ])
        overall_start = time.perf_counter()
        parameter_diag = self._set_stage_parameters(
            nominal_states, a_guess, targets, rotations,
            target_velocities, target_accelerations,
            contour_tangents, contour_active, q_ref)
        for stage in range(horizon + 1):
            if stage == 0:
                x_guess = x0
            else:
                x_guess = np.concatenate([q_guess[stage - 1], v_guess[stage - 1]])
            self.solver.set(stage, "x", x_guess)
            if stage < horizon:
                self.solver.set(stage, "u", a_guess[stage])
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)

        solver_time = 0.0
        linearization_time = parameter_diag["linearization_time"]
        projection_time = parameter_diag["projection_time"]
        status, nit = 0, 0
        for nit in range(1, cfg.acados_rti_iterations + 1):
            solve_started = time.perf_counter()
            status = int(self.solver.solve())
            solver_time += time.perf_counter() - solve_started
            if status != 0:
                break
            if nit < cfg.acados_rti_iterations:
                iteration_states = np.array([
                    self.solver.get(stage, "x")
                    for stage in range(horizon + 1)])
                iteration_controls = np.array([
                    self.solver.get(stage, "u")
                    for stage in range(horizon)])
                parameter_diag = self._set_stage_parameters(
                    iteration_states, iteration_controls,
                    targets, rotations,
                    target_velocities, target_accelerations,
                    contour_tangents, contour_active, q_ref)
                linearization_time += parameter_diag["linearization_time"]
                projection_time += parameter_diag["projection_time"]
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
            "solve_time": solver_time,
            "linearization_time": linearization_time,
            "projection_time": projection_time,
            "total_optimize_time": time.perf_counter() - overall_start,
            "cartesian_residual_mode": self.cartesian_residual_mode,
            "projection_velocity_residual_rms": parameter_diag[
                "velocity_residual_rms"],
            "projection_acceleration_residual_rms": parameter_diag[
                "acceleration_residual_rms"],
            "projection_velocity_active_ratio": parameter_diag[
                "velocity_active_ratio"],
            "projection_acceleration_active_ratio": parameter_diag[
                "acceleration_active_ratio"],
            "position_weight_scale": float(position_weight_scale),
        }
