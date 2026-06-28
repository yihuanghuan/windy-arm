#!/usr/bin/env python3
"""基于 Pinocchio 的短时域 MPC 风格位置逆解。

和 pinocchio_ik.py 使用同一份 URDF 与末端 frame,但这里不是只求一个
独立 IK 点,而是一次优化未来 N 步关节轨迹。默认只跟踪末端 3D 位置;
需要时可启用末端姿态软约束。
"""

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Dict, Optional, Tuple
import time
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
from scipy.optimize import Bounds, LinearConstraint, minimize

from pinocchio_ik import DEFAULT_URDF, EE_FRAME


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LL100_URDF = str(PACKAGE_ROOT / "config" / "ll100_lh420.urdf")
DEFAULT_OBSTACLE_KEYPOINT_FRAMES = ("link3", "link4", "link5", "link6", "link7")
DEFAULT_FRONT_PROPELLER_JOINTS = ("rot1_Joint", "rot3_Joint")


@dataclass
class MpcIkConfig:
    horizon_steps: int = 8    # 预测时域步数。每次优化 q_1 ... q_N, 实际控制时通常只执行 q_1。
    control_dt: float = 0.02  # 控制周期,用于记录/理解时域长度;当前代价主要按离散步数建模。
    max_joint_step: float = 0.08   # 相邻两步之间每个关节允许变化的最大幅度(rad),相当于简化速度约束。
    # 以下权重共同定义优化偏好:末端误差更小、关节更平滑、姿态不过分偏。
    position_weight: float = 50.0
    terminal_position_weight: float = 500.0
    orientation_weight: float = 0.0
    terminal_orientation_weight: float = 0.0
    joint_delta_weight: float = 0.08
    joint_delta_change_weight: float = 0.25
    current_posture_weight: float = 0.01
    center_posture_weight: float = 0.002
    success_tolerance: float = 2e-3
    max_iter: int = 100
    ftol: float = 1e-8
    solver_backend: str = "acados"
    acados_build_dir: str = "/tmp/windylab_acados_mpc_ik"
    acados_rti_iterations: int = 3  # SQP_RTI 每次 solve 只做一次线性化;这里允许重复几次提高终端精度。
    obstacle_avoidance_enabled: bool = False
    obstacle_weight: float = 2e4
    obstacle_safety_margin: float = 0.04
    obstacle_soft_eps: float = 1e-6
    obstacle_propeller_radius: float = 0.27
    obstacle_disk_half_thickness: float = 0.04  # 圆盘半厚度
    obstacle_ll100_urdf_path: str = DEFAULT_LL100_URDF
    obstacle_coordinate_mode: str = "arm_base"
    obstacle_keypoint_frames: Tuple[str, ...] = DEFAULT_OBSTACLE_KEYPOINT_FRAMES
    obstacle_front_propeller_joints: Tuple[str, ...] = DEFAULT_FRONT_PROPELLER_JOINTS
    body_obstacle_enabled: bool = False
    body_obstacle_weight: float = 3e4
    body_obstacle_margin: float = 0.03
    body_obstacle_boxes: Tuple[Tuple[float, float, float, float, float, float], ...] = ()
    capsule_radius: float = 0.045
    capsule_samples_per_segment: int = 3
    capsule_frame_sequence: Tuple[str, ...] = ()


class MpcPinocchioIK:
    """用于生成平滑 3D 位置 IK 命令的滚动优化器。"""

    def __init__(
        self,
        urdf_path: str = DEFAULT_URDF,
        ee_frame: str = EE_FRAME,
        config: Optional[MpcIkConfig] = None,
    ):
        self.config = config if config is not None else MpcIkConfig()
        self.urdf_path = urdf_path
        self.ee_frame = ee_frame
        # Pinocchio 直接从 URDF 建模,不手写 DH 参数,避免模型和 URDF 不一致。
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(ee_frame)
        self.nq = self.model.nq
        if self.config.obstacle_avoidance_enabled and self.config.acados_build_dir == MpcIkConfig.acados_build_dir:
            self.config.acados_build_dir = self.config.acados_build_dir + "_obstacle"
        # 关节限位来自 URDF,后面会作为优化边界/可行性检查使用。
        self.q_lower = self.model.lowerPositionLimit.copy()
        self.q_upper = self.model.upperPositionLimit.copy()
        self.q_center = 0.5 * (self.q_lower + self.q_upper)
        # 保存上一轮预测轨迹,下一轮可右移一格作为 warm start。
        self._last_plan = None
        # 从 Pinocchio/URDF 提取运动链,生成 CasADi 符号 FK。CasADi/ACADOS 后端共用它。
        self._symbolic_fk = _CasadiFkFromPinocchio(self.model, self.frame_id, self.urdf_path)
        if (
            (self.config.obstacle_avoidance_enabled or self.config.body_obstacle_enabled)
            and self.config.solver_backend != "acados"
        ):
            raise ValueError("第一版避障 NMPC 只支持 solver_backend='acados'; CasADi/SciPy 后端暂不接入避障代价。")
        self._obstacle_keypoint_frame_ids = (
            self._resolve_obstacle_keypoint_frames()
            if self.config.obstacle_avoidance_enabled
            else {}
        )
        self._obstacle_disks = self._build_front_propeller_disks() if self.config.obstacle_avoidance_enabled else np.zeros((0, 7))
        self._body_obstacle_boxes = self._resolve_body_obstacle_boxes()
        self._capsule_frame_ids = self._resolve_capsule_frames()
        self._acados_solver = None

    def forward(self, q: np.ndarray):
        """正运动学:给定关节角 q,返回末端位置和旋转矩阵。"""
        pin.framesForwardKinematics(self.model, self.data, q)
        oMf = self.data.oMf[self.frame_id]
        return oMf.translation.copy(), oMf.rotation.copy()

    def solve(
        self,
        target_position: np.ndarray,
        q_init: Optional[np.ndarray] = None,
        warm_start: Optional[np.ndarray] = None,
        target_rotation: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, bool, Dict[str, object]]:
        """优化一段短关节轨迹,返回时域最后一步的关节角。

        target_position 可以是 (3,),表示整个时域使用同一个目标点;
        也可以是 (horizon_steps, 3),表示每个预测步都有自己的未来目标点。
        target_rotation 可以是 None、(3,3) 或 (horizon_steps,3,3)。None 且姿态
        权重大于 0 时,会自动锁定 q_init 对应的初始末端姿态。
        """
        q0 = self._initial_q(q_init)
        targets = self._target_sequence(target_position)
        rotations, orientation_active = self._rotation_sequence(target_rotation, q0)
        initial_plan = self._initial_plan(q0, warm_start)

        # 求解器返回的是整段 q_1 ... q_N,solve() 对外暴露最后一步 q_N。
        result = self._optimize(q0, targets, rotations, initial_plan)
        plan = result.x.reshape(self.config.horizon_steps, self.nq)
        terminal_q = np.clip(plan[-1], self.q_lower, self.q_upper)
        terminal_error = self._position_error(terminal_q, targets[-1])
        plan_feasible = self._plan_feasible(q0, plan)
        tracking_ok = terminal_error <= self.config.success_tolerance
        # IPOPT 可能达到迭代上限但解已经足够好,因此 ok 按轨迹质量判定。
        ok = bool(tracking_ok and plan_feasible)

        info = self._make_info(
            result, plan, targets, rotations, terminal_error, orientation_active, plan_feasible, tracking_ok
        )
        self._last_plan = plan
        return terminal_q, ok, info

    def step(
        self,
        target_position: np.ndarray,
        q_current: np.ndarray,
        warm_start: Optional[np.ndarray] = None,
        target_rotation: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, bool, Dict[str, object]]:
        """滚动 MPC 用法:优化未来 N 步,但只返回第一步 q_1。"""
        q0 = self._initial_q(q_current)
        targets = self._target_sequence(target_position)
        rotations, orientation_active = self._rotation_sequence(target_rotation, q0)
        initial_plan = self._initial_plan(q0, warm_start)

        # Receding horizon:每个控制周期重新优化,只执行第一步,下一周期再滚动。
        result = self._optimize(q0, targets, rotations, initial_plan)
        plan = result.x.reshape(self.config.horizon_steps, self.nq)
        q_next = np.clip(plan[0], self.q_lower, self.q_upper)
        terminal_error = self._position_error(plan[-1], targets[-1])
        first_error = self._position_error(q_next, targets[0])
        plan_feasible = self._plan_feasible(q0, plan)
        tracking_ok = terminal_error <= self.config.success_tolerance
        ok = bool(tracking_ok and plan_feasible)

        info = self._make_info(
            result, plan, targets, rotations, terminal_error, orientation_active, plan_feasible, tracking_ok
        )
        info["first_error"] = first_error
        self._last_plan = plan
        return q_next, ok, info

    def last_plan(self) -> Optional[np.ndarray]:
        """返回上一轮预测轨迹副本,可用于可视化或外部 warm start。"""
        if self._last_plan is None:
            return None
        return self._last_plan.copy()

    def obstacle_disks(self) -> np.ndarray:
        """返回前桨避障圆盘,每行 [cx, cy, cz, nx, ny, nz, radius]。"""
        return self._obstacle_disks.copy()

    def obstacle_spheres(self) -> np.ndarray:
        """兼容旧 demo 的可视化接口:返回每个圆盘的外接球,不参与 NMPC 代价。"""
        if self._obstacle_disks.shape[0] == 0:
            return np.zeros((0, 4), dtype=float)
        radius = self._obstacle_disks[:, 6] + float(self.config.obstacle_safety_margin)
        return np.column_stack([self._obstacle_disks[:, :3], radius])

    def obstacle_keypoint_positions(self, q: np.ndarray) -> Dict[str, np.ndarray]:
        """返回机械臂避障关键点位置,用于 RViz marker 和调试。"""
        q = self._initial_q(q)
        pin.framesForwardKinematics(self.model, self.data, q)
        return {
            frame_name: self.data.oMf[frame_id].translation.copy()
            for frame_name, frame_id in self._obstacle_keypoint_frame_ids.items()
        }

    def body_obstacle_boxes(self) -> np.ndarray:
        """返回机身简化 box,每行 [cx, cy, cz, hx, hy, hz],坐标系由调用方约定。"""
        return self._body_obstacle_boxes.copy()

    def capsule_sample_positions(self, q: np.ndarray) -> np.ndarray:
        """返回 capsule 线段上的固定采样点,用于 RViz marker 和诊断。"""
        if len(self._capsule_frame_ids) < 2:
            return np.zeros((0, 3), dtype=float)
        q = self._initial_q(q)
        pin.framesForwardKinematics(self.model, self.data, q)
        frame_positions = [
            self.data.oMf[frame_id].translation.copy()
            for frame_id in self._capsule_frame_ids
        ]
        return self._capsule_samples_from_positions(frame_positions)

    def _resolve_obstacle_keypoint_frames(self) -> Dict[str, int]:
        frame_ids = {}
        for frame_name in self.config.obstacle_keypoint_frames:
            frame_id = self.model.getFrameId(frame_name)
            if frame_id >= self.model.nframes:
                raise ValueError(f"避障关键点 frame '{frame_name}' 不存在于 {self.urdf_path}")
            frame_ids[frame_name] = frame_id
        return frame_ids

    def _resolve_body_obstacle_boxes(self) -> np.ndarray:
        if not self.config.body_obstacle_enabled:
            return np.zeros((0, 6), dtype=float)
        boxes = np.array(self.config.body_obstacle_boxes, dtype=float)
        if boxes.size == 0:
            return np.zeros((0, 6), dtype=float)
        boxes = boxes.reshape(-1, 6)
        if not np.all(np.isfinite(boxes)):
            raise ValueError("body_obstacle_boxes 包含 NaN/Inf")
        if np.any(boxes[:, 3:] <= 0.0):
            raise ValueError("body_obstacle_boxes 的半尺寸 hx/hy/hz 必须为正")
        return boxes

    def _resolve_capsule_frames(self) -> Tuple[int, ...]:
        if not self.config.body_obstacle_enabled:
            return ()
        frame_ids = []
        for frame_name in self.config.capsule_frame_sequence:
            frame_id = self.model.getFrameId(frame_name)
            if frame_id >= self.model.nframes:
                raise ValueError(f"capsule frame '{frame_name}' 不存在于 {self.urdf_path}")
            frame_ids.append(frame_id)
        if self._body_obstacle_boxes.shape[0] > 0 and len(frame_ids) < 2:
            raise ValueError("启用 body obstacle 时 capsule_frame_sequence 至少需要两个 frame")
        return tuple(frame_ids)

    def _build_front_propeller_disks(self) -> np.ndarray:
        """从 LL100 整机 URDF 中提取前桨圆盘,转换到配置指定坐标系下。"""
        ll100_path = Path(self.config.obstacle_ll100_urdf_path).expanduser()
        if not ll100_path.exists():
            raise FileNotFoundError(f"找不到 LL100 URDF,无法生成前桨避障圆盘: {ll100_path}")

        joints = _read_urdf_joint_origins_and_axes(str(ll100_path))
        required = ("arm_base_Joint",) + tuple(self.config.obstacle_front_propeller_joints)
        missing = [name for name in required if name not in joints]
        if missing:
            raise ValueError(f"LL100 URDF 缺少避障所需 joint: {missing}")
        if self.config.obstacle_coordinate_mode not in ("arm_base", "ll100_base"):
            raise ValueError("obstacle_coordinate_mode 必须是 'arm_base' 或 'll100_base'")

        T_ll100_arm = _xyz_rpy_to_transform(joints["arm_base_Joint"]["xyz"], joints["arm_base_Joint"]["rpy"])
        if self.config.obstacle_coordinate_mode == "arm_base":
            T_target_ll100 = np.linalg.inv(T_ll100_arm)
        else:
            T_target_ll100 = np.eye(4)
        R_target_ll100 = T_target_ll100[:3, :3]
        disk_rows = []
        disk_radius = float(self.config.obstacle_propeller_radius)
        propeller_center_offset = 0.06 # m,桨叶平面相对电机 joint origin 沿电机轴线向下偏移。
        if disk_radius <= 0.0:
            raise ValueError("obstacle_propeller_radius 必须为正")

        for joint_name in self.config.obstacle_front_propeller_joints:
            joint = joints[joint_name]
            T_ll100_prop = _xyz_rpy_to_transform(joint["xyz"], joint["rpy"])
            axis_local = joint["axis"]
            if np.linalg.norm(axis_local) <= 0.0:
                axis_local = np.array([0.0, 0.0, 1.0])
            normal_ll100 = T_ll100_prop[:3, :3] @ (axis_local / np.linalg.norm(axis_local))
            normal_target = R_target_ll100 @ normal_ll100
            normal_norm = np.linalg.norm(normal_target)
            if normal_norm <= 0.0:
                raise ValueError(f"{joint_name} 的前桨法向量为零,请检查 URDF joint axis")
            normal_target = normal_target / normal_norm
            center_target = (T_target_ll100 @ np.array([*joint["xyz"], 1.0], dtype=float))[:3]
            center_target = center_target + propeller_center_offset * normal_target
            disk_rows.append([*center_target, *normal_target, disk_radius])

        disks = np.array(disk_rows, dtype=float)
        if not np.all(np.isfinite(disks)):
            raise ValueError("前桨避障圆盘包含非有限数值,请检查 LL100 URDF joint origin/axis")
        return disks

    def _capsule_samples_from_positions(self, frame_positions) -> np.ndarray:
        samples = []
        n_samples = max(1, int(self.config.capsule_samples_per_segment))
        # 包含端点和段内点,让 frame 原点附近和连杆中段都参与避障。
        alphas = np.linspace(0.0, 1.0, n_samples + 2)
        for p0, p1 in zip(frame_positions[:-1], frame_positions[1:]):
            p0 = np.asarray(p0, dtype=float)
            p1 = np.asarray(p1, dtype=float)
            for alpha in alphas:
                samples.append((1.0 - alpha) * p0 + alpha * p1)
        if not samples:
            return np.zeros((0, 3), dtype=float)
        return np.array(samples, dtype=float)

    def _initial_q(self, q: Optional[np.ndarray]) -> np.ndarray:
        """整理当前关节角:没有输入则用 Pinocchio neutral 位姿,并钳到限位内。"""
        if q is None:
            q = pin.neutral(self.model)
        q = np.asarray(q, dtype=float).reshape(self.nq)
        return np.clip(q, self.q_lower, self.q_upper)

    def _target_sequence(self, target_position: np.ndarray) -> np.ndarray:
        """把单个目标点扩展成 N 步目标序列,或检查外部传入的目标序列。"""
        target = np.asarray(target_position, dtype=float)
        if target.shape == (3,):
            return np.repeat(target.reshape(1, 3), self.config.horizon_steps, axis=0)
        expected = (self.config.horizon_steps, 3)
        if target.shape != expected:
            raise ValueError(f"target_position 形状必须是 (3,) 或 {expected}, 当前是 {target.shape}")
        return target

    def _rotation_sequence(self, target_rotation: Optional[np.ndarray], q0: np.ndarray):
        """整理目标姿态序列。

        姿态软约束默认关闭。若权重大于 0 且没有显式目标姿态,则锁定当前
        q0 对应的末端姿态;若显式传入目标姿态,则即使权重为 0 也保留在
        info 中,方便调用方观察姿态误差。
        """
        if target_rotation is None:
            _, current_rotation = self.forward(q0)
            rotations = np.repeat(current_rotation.reshape(1, 3, 3), self.config.horizon_steps, axis=0)
            active = self._orientation_cost_enabled()
            return rotations, active

        rotation = np.asarray(target_rotation, dtype=float)
        if rotation.shape == (3, 3):
            return np.repeat(rotation.reshape(1, 3, 3), self.config.horizon_steps, axis=0), True

        expected = (self.config.horizon_steps, 3, 3)
        if rotation.shape != expected:
            raise ValueError(f"target_rotation 形状必须是 None、(3,3) 或 {expected}, 当前是 {rotation.shape}")
        return rotation, True

    def _orientation_cost_enabled(self) -> bool:
        return bool(
            self.config.orientation_weight > 0.0
            or self.config.terminal_orientation_weight > 0.0
        )

    def _initial_plan(self, q0: np.ndarray, warm_start: Optional[np.ndarray]) -> np.ndarray:
        """生成优化初值。

        有上一轮轨迹时,把轨迹整体前移一格:旧的 q_2 作为新的 q_1,
        旧的 q_N 继续作为末尾猜测。这就是 MPC 常见的 warm start。
        """
        if warm_start is None:
            warm_start = self._last_plan

        if warm_start is None:
            plan = np.repeat(q0.reshape(1, self.nq), self.config.horizon_steps, axis=0)
        else:
            plan = np.asarray(warm_start, dtype=float)
            if plan.shape != (self.config.horizon_steps, self.nq):
                plan = np.repeat(q0.reshape(1, self.nq), self.config.horizon_steps, axis=0)
            else:
                shifted = np.vstack([plan[1:], plan[-1:]])
                plan = shifted.copy()

        return self._project_plan_step_limits(q0, plan)

    def _project_plan_step_limits(self, q0: np.ndarray, plan: np.ndarray) -> np.ndarray:
        """把初值投影到关节限位和每步变化约束内,避免求解器从坏点开始。"""
        projected = np.empty_like(plan)
        previous = q0
        for i, q in enumerate(plan):
            q = np.clip(q, self.q_lower, self.q_upper)
            dq = np.clip(q - previous, -self.config.max_joint_step, self.config.max_joint_step)
            projected[i] = np.clip(previous + dq, self.q_lower, self.q_upper)
            previous = projected[i]
        return projected

    def _optimize(
        self,
        q0: np.ndarray,
        targets: np.ndarray,
        rotations: np.ndarray,
        initial_plan: np.ndarray,
    ):
        """根据配置选择求解器后端。"""
        if self.config.solver_backend == "casadi":
            return self._optimize_casadi(q0, targets, rotations, initial_plan)
        if self.config.solver_backend == "acados":
            return self._optimize_acados(q0, targets, rotations, initial_plan)
        if self.config.solver_backend == "scipy":
            return self._optimize_scipy(q0, targets, rotations, initial_plan)
        raise ValueError("solver_backend 必须是 'casadi', 'acados' 或 'scipy'")

    def _optimize_scipy(
        self,
        q0: np.ndarray,
        targets: np.ndarray,
        rotations: np.ndarray,
        initial_plan: np.ndarray,
    ):
        """SciPy SLSQP 后端:非线性目标 + 线性步长约束 + box 边界。"""
        bounds = self._bounds()
        constraints = [self._step_constraint(q0)]

        return minimize(
            fun=lambda x: self._objective(x, q0, targets, rotations),
            x0=initial_plan.reshape(-1),
            method="SLSQP",
            bounds=bounds,
            constraints=constraints,
            options={
                "maxiter": self.config.max_iter,
                "ftol": self.config.ftol,
                "disp": False,
            },
        )

    def _optimize_casadi(
        self,
        q0: np.ndarray,
        targets: np.ndarray,
        rotations: np.ndarray,
        initial_plan: np.ndarray,
    ):
        try:
            import casadi as ca
        except Exception as exc:
            raise RuntimeError(
                "已选择 CasADi 后端,但没有安装 casadi。"
                "请安装 python3-casadi/casadi,或设置 solver_backend='scipy'。"
            ) from exc

        cfg = self.config
        opti = ca.Opti()
        # 决策变量:q 是 N x nq 的矩阵,每一行是一帧预测关节角。
        q = opti.variable(cfg.horizon_steps, self.nq)

        for i in range(cfg.horizon_steps):
            # 约束 1:每一帧关节都必须在 URDF 限位内。
            opti.subject_to(opti.bounded(ca.DM(self.q_lower), q[i, :].T, ca.DM(self.q_upper)))
            if i == 0:
                dq = q[i, :].T - ca.DM(q0)
            else:
                dq = q[i, :].T - q[i - 1, :].T
            # 约束 2:相邻两帧关节变化不能超过 max_joint_step。
            opti.subject_to(opti.bounded(-cfg.max_joint_step, dq, cfg.max_joint_step))

        cost = 0
        previous_q = ca.DM(q0)
        previous_dq = ca.DM.zeros(self.nq, 1)
        for i in range(cfg.horizon_steps):
            q_i = q[i, :].T
            # 纯符号 FK:CasADi 能看到完整计算图,可以自动求导。
            pos = self._symbolic_fk.position(q_i)
            rot = self._symbolic_fk.rotation(q_i)
            pos_error = pos - ca.DM(targets[i])
            rot_error = self._orientation_error_ca(rot, ca.DM(rotations[i]))
            dq = q_i - previous_q
            ddq = dq - previous_dq

            # 代价项 1:末端位置误差,让机械臂末端靠近目标。
            cost += cfg.position_weight * ca.sumsqr(pos_error)
            # 代价项 1b:末端姿态软约束。默认权重为 0,不影响位置-only 行为。
            cost += cfg.orientation_weight * ca.sumsqr(rot_error)
            # 代价项 2:关节每步变化越小越好,轨迹会更柔和。
            cost += cfg.joint_delta_weight * ca.sumsqr(dq)
            # 代价项 3:关节变化的变化越小越好,减少抖动/突变。
            cost += cfg.joint_delta_change_weight * ca.sumsqr(ddq)
            # 代价项 4:弱约束,避免为了够到目标而远离当前姿态太多。
            cost += cfg.current_posture_weight * ca.sumsqr(q_i - ca.DM(q0))
            # 代价项 5:弱约束,让解不要总贴着关节限位。
            cost += cfg.center_posture_weight * ca.sumsqr(q_i - ca.DM(self.q_center))

            previous_q = q_i
            previous_dq = dq

        terminal_pos = self._symbolic_fk.position(q[-1, :].T)
        terminal_rot = self._symbolic_fk.rotation(q[-1, :].T)
        terminal_error = terminal_pos - ca.DM(targets[-1])
        terminal_rot_error = self._orientation_error_ca(terminal_rot, ca.DM(rotations[-1]))
        # 终端误差权重更大:鼓励预测时域末尾真正到达目标。
        cost += cfg.terminal_position_weight * ca.sumsqr(terminal_error)
        cost += cfg.terminal_orientation_weight * ca.sumsqr(terminal_rot_error)
        opti.minimize(cost)
        # 初值来自上一轮 warm start 或当前姿态重复展开,对非线性优化很重要。
        opti.set_initial(q, initial_plan)

        opti.solver(
            "ipopt",
            {
                "expand": False,
                "print_time": False,
                "ipopt.print_level": 0,
                "ipopt.sb": "yes",
                "ipopt.max_iter": cfg.max_iter,
                "ipopt.tol": cfg.ftol,
                "ipopt.acceptable_tol": max(cfg.ftol * 10.0, 1e-7),
                # 符号 FK 已可自动求导;limited-memory 可减少 IPOPT 二阶 Hessian 开销。
                "ipopt.hessian_approximation": "limited-memory",
            },
        )

        try:
            # 正常收敛:直接读取最优解。
            sol = opti.solve()
            plan = np.array(sol.value(q), dtype=float)
            cost_value = float(sol.value(cost))
            stats = opti.stats()
            return _CasadiResult(
                x=plan.reshape(-1),
                success=bool(stats.get("success", True)),
                status=0,
                message=str(stats.get("return_status", "Solve_Succeeded")),
                nit=int(stats.get("iter_count", 0)),
                fun=cost_value,
            )
        except RuntimeError as exc:
            # IPOPT 可能达到迭代上限,但 debug.value 里仍有一条可用轨迹。
            # 上层会用终端误差和约束检查决定 ok,而不是只看 solver 状态。
            plan = np.array(opti.debug.value(q), dtype=float)
            cost_value = float(opti.debug.value(cost))
            stats = opti.stats()
            return _CasadiResult(
                x=plan.reshape(-1),
                success=False,
                status=1,
                message=str(stats.get("return_status", exc)),
                nit=int(stats.get("iter_count", 0)),
                fun=cost_value,
            )

    def _optimize_acados(
        self,
        q0: np.ndarray,
        targets: np.ndarray,
        rotations: np.ndarray,
        initial_plan: np.ndarray,
    ):
        solver = self._get_acados_solver()
        cfg = self.config
        start_time = time.perf_counter()
        x0 = np.concatenate([q0, np.zeros(self.nq)])

        for stage in range(cfg.horizon_steps + 1):
            target_index = min(max(stage - 1, 0), cfg.horizon_steps - 1)
            target = targets[target_index]
            rotation = rotations[target_index]
            p = np.concatenate([target, rotation.reshape(-1), q0, self.q_center])
            solver.set(stage, "p", p)
            solver.set(stage, "x", np.concatenate([initial_plan[min(stage, cfg.horizon_steps - 1)], np.zeros(self.nq)]))
            if stage < cfg.horizon_steps:
                if stage == 0:
                    dq_guess = initial_plan[0] - q0
                else:
                    dq_guess = initial_plan[stage] - initial_plan[stage - 1]
                solver.set(stage, "u", np.clip(dq_guess, -cfg.max_joint_step, cfg.max_joint_step))

        solver.set(0, "lbx", x0)
        solver.set(0, "ubx", x0)
        status = 0
        nit = 0
        plan = initial_plan.copy()
        terminal_error = float("inf")
        us = np.zeros((cfg.horizon_steps, self.nq))
        # SQP_RTI 一次 solve 是一次实时迭代。多调用几次可以在静态 IK 自测中
        # 继续围绕上一轮结果线性化,通常 2-3 次就能把毫米级误差压到几十微米。
        for nit in range(1, max(1, cfg.acados_rti_iterations) + 1):
            status = int(solver.solve())
            xs = np.array([solver.get(i, "x") for i in range(cfg.horizon_steps + 1)], dtype=float)
            us = np.array([solver.get(i, "u") for i in range(cfg.horizon_steps)], dtype=float)
            plan = xs[1:, :self.nq]
            terminal_error = self._position_error(plan[-1], targets[-1])
            if status != 0 or terminal_error <= cfg.success_tolerance:
                break
        solve_time = time.perf_counter() - start_time

        cost = float(np.sum(us * us))
        return _AcadosResult(
            x=plan.reshape(-1),
            success=(status == 0),
            status=status,
            message=_format_acados_status(status),
            nit=nit,
            fun=cost,
            solve_time=solve_time,
        )

    def _get_acados_solver(self):
        if self._acados_solver is None:
            self._acados_solver = _AcadosMpcBackend(self)
        return self._acados_solver

    def _bounds(self) -> Bounds:
        """SciPy 后端使用的关节上下限。"""
        lower = np.tile(self.q_lower, self.config.horizon_steps)
        upper = np.tile(self.q_upper, self.config.horizon_steps)
        return Bounds(lower, upper)

    def _step_constraint(self, q0: np.ndarray) -> LinearConstraint:
        """SciPy 后端使用的线性步长约束。

        约束形式是 -step <= q_i - q_{i-1} <= step。
        对第一步来说 q_{i-1} 是当前真实关节 q0。
        """
        n = self.nq
        horizon = self.config.horizon_steps
        rows = horizon * n
        mat = np.zeros((rows, horizon * n))
        lb = np.full(rows, -self.config.max_joint_step)
        ub = np.full(rows, self.config.max_joint_step)

        for step in range(horizon):
            for joint in range(n):
                row = step * n + joint
                col = step * n + joint
                mat[row, col] = 1.0
                if step == 0:
                    lb[row] += q0[joint]
                    ub[row] += q0[joint]
                else:
                    mat[row, col - n] = -1.0

        return LinearConstraint(mat, lb, ub)

    def _objective(
        self,
        x: np.ndarray,
        q0: np.ndarray,
        targets: np.ndarray,
        rotations: np.ndarray,
    ) -> float:
        """SciPy 后端的目标函数,和 CasADi 后端使用同一组代价项。"""
        plan = x.reshape(self.config.horizon_steps, self.nq)
        cfg = self.config

        cost = 0.0
        previous_q = q0
        previous_dq = np.zeros(self.nq)

        for i, q in enumerate(plan):
            pos, rot = self.forward(q)
            pos_error = pos - targets[i]
            rot_error = self._orientation_error(rot, rotations[i])
            dq = q - previous_q
            ddq = dq - previous_dq

            cost += cfg.position_weight * float(pos_error @ pos_error)
            cost += cfg.orientation_weight * float(rot_error @ rot_error)
            cost += cfg.joint_delta_weight * float(dq @ dq)
            cost += cfg.joint_delta_change_weight * float(ddq @ ddq)
            cost += cfg.current_posture_weight * float((q - q0) @ (q - q0))
            cost += cfg.center_posture_weight * float((q - self.q_center) @ (q - self.q_center))

            previous_q = q
            previous_dq = dq

        terminal_pos, terminal_rot = self.forward(plan[-1])
        terminal_error = terminal_pos - targets[-1]
        terminal_rot_error = self._orientation_error(terminal_rot, rotations[-1])
        cost += cfg.terminal_position_weight * float(terminal_error @ terminal_error)
        cost += cfg.terminal_orientation_weight * float(terminal_rot_error @ terminal_rot_error)
        return cost

    def _position_error(self, q: np.ndarray, target: np.ndarray) -> float:
        """计算某个关节角对应的末端位置误差范数。"""
        pos, _ = self.forward(q)
        return float(np.linalg.norm(pos - target))

    @staticmethod
    def _orientation_error(current_rotation: np.ndarray, target_rotation: np.ndarray) -> np.ndarray:
        skew = target_rotation @ current_rotation.T - current_rotation @ target_rotation.T
        return 0.5 * np.array([skew[2, 1], skew[0, 2], skew[1, 0]], dtype=float)

    @staticmethod
    def _orientation_error_ca(current_rotation, target_rotation):
        import casadi as ca

        skew = ca.mtimes(target_rotation, current_rotation.T) - ca.mtimes(current_rotation, target_rotation.T)
        return 0.5 * ca.vertcat(skew[2, 1], skew[0, 2], skew[1, 0])

    def _obstacle_residuals_ca(self, q):
        """ACADOS 避障残差:前桨圆盘和机身 box 的软约束。"""
        import casadi as ca

        residuals = []
        eps = float(self.config.obstacle_soft_eps)

        if self.config.obstacle_avoidance_enabled and self._obstacle_disks.shape[0] > 0:
            for frame_id in self._obstacle_keypoint_frame_ids.values():
                p_link = self._symbolic_fk.frame_position(q, frame_id)
                for disk in self._obstacle_disks:
                    sdf = self._disk_signed_distance_ca(p_link, ca.DM(disk), eps)
                    margin = sdf - float(self.config.obstacle_safety_margin)
                    # smooth_positive(-margin): margin<0 表示进入圆盘安全距离内。
                    residuals.append(self._smooth_positive_ca(-margin, eps))

        if self.config.body_obstacle_enabled and self._body_obstacle_boxes.shape[0] > 0:
            frame_points = [
                self._symbolic_fk.frame_position(q, frame_id)
                for frame_id in self._capsule_frame_ids
            ]
            for p0, p1 in zip(frame_points[:-1], frame_points[1:]):
                for alpha in np.linspace(0.0, 1.0, max(1, int(self.config.capsule_samples_per_segment)) + 2):
                    sample = (1.0 - float(alpha)) * p0 + float(alpha) * p1
                    for box in self._body_obstacle_boxes:
                        signed_distance = self._box_signed_distance_ca(sample, box, eps)
                        margin = (
                            signed_distance
                            - float(self.config.capsule_radius)
                            - float(self.config.body_obstacle_margin)
                        )
                        residuals.append(self._smooth_positive_ca(-margin, eps))

        if not residuals:
            return ca.MX.zeros(0, 1)
        return ca.vertcat(*residuals)

    def _front_obstacle_residual_count(self) -> int:
        if not self.config.obstacle_avoidance_enabled:
            return 0
        return len(self._obstacle_keypoint_frame_ids) * int(self._obstacle_disks.shape[0])

    def _body_obstacle_residual_count(self) -> int:
        if not self.config.body_obstacle_enabled or len(self._capsule_frame_ids) < 2:
            return 0
        samples_per_segment = max(1, int(self.config.capsule_samples_per_segment)) + 2
        return (len(self._capsule_frame_ids) - 1) * samples_per_segment * int(self._body_obstacle_boxes.shape[0])

    @staticmethod
    def _smooth_positive_ca(value, eps: float):
        import casadi as ca

        return 0.5 * (value + ca.sqrt(value * value + eps))

    @staticmethod
    def _smooth_max3_ca(values, sharpness: float = 30.0):
        import casadi as ca

        return ca.log(ca.exp(sharpness * values[0]) + ca.exp(sharpness * values[1]) + ca.exp(sharpness * values[2])) / sharpness

    @staticmethod
    def _disk_signed_distance_ca(point, disk, eps: float):
        import casadi as ca

        relative = point - disk[:3]
        normal = disk[3:6]
        d_axial = ca.dot(normal, relative)
        lateral = relative - d_axial * normal
        r = ca.sqrt(ca.sumsqr(lateral) + 1e-8)
        excess_r = MpcPinocchioIK._smooth_positive_ca(r - disk[6], eps)
        return ca.sqrt(excess_r ** 2 + d_axial ** 2 + 1e-8)

    def _box_signed_distance_ca(self, point, box: np.ndarray, eps: float):
        import casadi as ca

        center = ca.DM(box[:3])
        half = ca.DM(box[3:])
        d = point - center
        abs_d = ca.sqrt(ca.power(d, 2) + eps)
        q = abs_d - half
        outside = ca.vertcat(
            self._smooth_positive_ca(q[0], eps),
            self._smooth_positive_ca(q[1], eps),
            self._smooth_positive_ca(q[2], eps),
        )
        outside_dist = ca.sqrt(ca.sumsqr(outside) + eps)
        max_q = self._smooth_max3_ca(q)
        inside_dist = -self._smooth_positive_ca(-max_q, eps)
        return outside_dist + inside_dist

    def _obstacle_diagnostics(self, plan: np.ndarray) -> Dict[str, object]:
        front = self._front_obstacle_diagnostics(plan)
        body = self._body_obstacle_diagnostics(plan)
        active = bool(front["front_obstacle_active"] or body["body_obstacle_active"])
        margins = [
            value for value in (
                front["min_front_obstacle_margin"],
                body["min_body_obstacle_margin"],
            )
            if value is not None
        ]
        violations = [
            value for value in (
                front["front_obstacle_violation_max"],
                body["body_obstacle_violation_max"],
            )
            if value is not None
        ]
        result = {
            "obstacle_active": active,
            "min_obstacle_margin": float(min(margins)) if margins else None,
            "obstacle_violation_max": float(max(violations)) if violations else None,
        }
        result.update(front)
        result.update(body)
        return result

    def _front_obstacle_diagnostics(self, plan: np.ndarray) -> Dict[str, object]:
        if not self.config.obstacle_avoidance_enabled or self._obstacle_disks.shape[0] == 0:
            return {
                "front_obstacle_active": False,
                "min_front_obstacle_margin": None,
                "front_obstacle_violation_max": None,
            }

        min_signed_distance = float("inf")
        max_violation = 0.0
        for q in plan:
            points = self.obstacle_keypoint_positions(q)
            for p_link in points.values():
                for disk in self._obstacle_disks:
                    signed_distance = (
                        _disk_signed_distance_np(p_link, disk)
                        - float(self.config.obstacle_safety_margin)
                    )
                    min_signed_distance = min(min_signed_distance, signed_distance)
                    max_violation = max(max_violation, max(0.0, -signed_distance))

        return {
            "front_obstacle_active": True,
            "min_front_obstacle_margin": float(min_signed_distance),
            "front_obstacle_violation_max": float(max_violation),
        }

    def _body_obstacle_diagnostics(self, plan: np.ndarray) -> Dict[str, object]:
        if not self.config.body_obstacle_enabled or self._body_obstacle_boxes.shape[0] == 0:
            return {
                "body_obstacle_active": False,
                "min_body_obstacle_margin": None,
                "body_obstacle_violation_max": None,
            }

        min_margin = float("inf")
        max_violation = 0.0
        radius_with_margin = float(self.config.capsule_radius + self.config.body_obstacle_margin)
        for q in plan:
            samples = self.capsule_sample_positions(q)
            for sample in samples:
                for box in self._body_obstacle_boxes:
                    signed_distance = _box_signed_distance_np(sample, box)
                    margin = signed_distance - radius_with_margin
                    min_margin = min(min_margin, margin)
                    max_violation = max(max_violation, max(0.0, -margin))

        return {
            "body_obstacle_active": True,
            "min_body_obstacle_margin": float(min_margin),
            "body_obstacle_violation_max": float(max_violation),
        }

    def _plan_feasible(self, q0: np.ndarray, plan: np.ndarray) -> bool:
        """检查预测轨迹是否满足数值有效、关节限位和每步关节变化约束。"""
        if not np.all(np.isfinite(plan)):
            return False
        if np.any(plan < self.q_lower - 1e-6) or np.any(plan > self.q_upper + 1e-6):
            return False
        step_diffs = np.diff(np.vstack([q0, plan]), axis=0)
        return bool(np.all(np.abs(step_diffs) <= self.config.max_joint_step + 1e-6))

    def _make_info(
        self,
        result,
        plan: np.ndarray,
        targets: np.ndarray,
        rotations: np.ndarray,
        terminal_error: float,
        orientation_active: bool,
        plan_feasible: bool,
        tracking_ok: bool,
    ) -> Dict[str, object]:
        """整理诊断信息,方便 demo 打印、画图或后续调参。"""
        terminal_orientation_error = None
        if orientation_active:
            _, terminal_rotation = self.forward(plan[-1])
            terminal_orientation_error = float(
                np.linalg.norm(self._orientation_error(terminal_rotation, rotations[-1]))
            )

        solver_success = bool(result.success)
        command_usable = bool(plan_feasible and (solver_success or tracking_ok))
        info = {
            "success": solver_success,
            "status": int(result.status),
            "message": str(result.message),
            "nit": int(result.nit),
            "cost": float(result.fun),
            "solver_backend": self.config.solver_backend,
            "terminal_error": float(terminal_error),
            "terminal_orientation_error": terminal_orientation_error,
            "tracking_ok": bool(tracking_ok),
            "plan_feasible": bool(plan_feasible),
            "command_usable": command_usable,
            "trajectory": plan.copy(),
            "target_trajectory": targets.copy(),
            "target_rotation_trajectory": rotations.copy(),
        }
        if hasattr(result, "solve_time"):
            info["solve_time"] = float(result.solve_time)
        info.update(self._obstacle_diagnostics(plan))
        return info


class _CasadiResult:
    """把 CasADi/IPOPT 的结果包装成和 SciPy OptimizeResult 类似的结构。"""

    def __init__(self, x, success, status, message, nit, fun):
        self.x = x
        self.success = success
        self.status = status
        self.message = message
        self.nit = nit
        self.fun = fun


class _AcadosResult(_CasadiResult):
    def __init__(self, x, success, status, message, nit, fun, solve_time):
        super().__init__(x, success, status, message, nit, fun)
        self.solve_time = solve_time


class _CasadiFkFromPinocchio:
    """从 Pinocchio/URDF 提取串联运动链,生成 CasADi 符号 FK。

    Pinocchio 负责读 URDF、确定关节顺序和固定 placement;URDF 负责提供
    revolute joint axis。生成后的 FK 是 CasADi 表达式,不是 Callback 黑盒。
    """

    def __init__(self, model, frame_id: int, urdf_path: str):
        self.model = model
        self.frame_id = frame_id
        self.urdf_path = urdf_path
        self.axes = self._read_joint_axes(urdf_path)
        self.chain = self._joint_chain_to_frame(model, frame_id)
        self.frame_placement = model.frames[frame_id].placement
        self._frame_chain_cache = {frame_id: self.chain}
        self._frame_placement_cache = {frame_id: self.frame_placement}
        self._position_function = None
        self._rotation_function = None

    def transform(self, q):
        return self.transform_to_frame(q, self.frame_id)

    def transform_to_frame(self, q, frame_id: int):
        import casadi as ca

        T = ca.MX.eye(4)
        chain = self._chain_to_frame(frame_id)
        for joint_id in chain:
            name = self.model.names[joint_id]
            joint = self.model.joints[joint_id]
            if joint.nq != 1:
                raise ValueError(f"暂只支持 1 自由度关节, {name} 的 nq={joint.nq}")
            placement = self.model.jointPlacements[joint_id]
            axis = self.axes[name]
            T = ca.mtimes(T, self._se3_to_ca(placement))
            T = ca.mtimes(T, self._rotation_transform_ca(axis, q[joint.idx_q]))

        T = ca.mtimes(T, self._se3_to_ca(self._placement_to_frame(frame_id)))
        return T

    def position(self, q):
        T = self.transform(q)
        return T[:3, 3]

    def frame_position(self, q, frame_id: int):
        T = self.transform_to_frame(q, frame_id)
        return T[:3, 3]

    def rotation(self, q):
        T = self.transform(q)
        return T[:3, :3]

    def function(self):
        if self._position_function is None:
            import casadi as ca

            q = ca.MX.sym("q", self.model.nq)
            self._position_function = ca.Function("symbolic_fk_position", [q], [self.position(q)])
        return self._position_function

    def rotation_function(self):
        if self._rotation_function is None:
            import casadi as ca

            q = ca.MX.sym("q", self.model.nq)
            self._rotation_function = ca.Function("symbolic_fk_rotation", [q], [self.rotation(q)])
        return self._rotation_function

    def validate_against_pinocchio(self, data, samples: int = 100, seed: int = 7) -> Tuple[float, float]:
        rng = np.random.default_rng(seed)
        pos_fn = self.function()
        rot_fn = self.rotation_function()
        max_pos_error = 0.0
        max_rot_error = 0.0
        for _ in range(samples):
            q = rng.uniform(self.model.lowerPositionLimit, self.model.upperPositionLimit)
            pin.framesForwardKinematics(self.model, data, q)
            pin_pos = data.oMf[self.frame_id].translation.copy()
            pin_rot = data.oMf[self.frame_id].rotation.copy()
            ca_pos = np.array(pos_fn(q), dtype=float).reshape(3)
            ca_rot = np.array(rot_fn(q), dtype=float).reshape(3, 3)
            max_pos_error = max(max_pos_error, float(np.linalg.norm(ca_pos - pin_pos)))
            max_rot_error = max(max_rot_error, float(np.linalg.norm(ca_rot - pin_rot, ord="fro")))
        return max_pos_error, max_rot_error

    def validate_frame_positions_against_pinocchio(
        self,
        data,
        frame_ids: Tuple[int, ...],
        samples: int = 100,
        seed: int = 11,
    ) -> float:
        import casadi as ca

        rng = np.random.default_rng(seed)
        q_sym = ca.MX.sym("q", self.model.nq)
        frame_functions = [
            ca.Function(f"symbolic_fk_frame_{frame_id}", [q_sym], [self.frame_position(q_sym, frame_id)])
            for frame_id in frame_ids
        ]
        max_pos_error = 0.0
        for _ in range(samples):
            q = rng.uniform(self.model.lowerPositionLimit, self.model.upperPositionLimit)
            pin.framesForwardKinematics(self.model, data, q)
            for frame_id, fn in zip(frame_ids, frame_functions):
                pin_pos = data.oMf[frame_id].translation.copy()
                ca_pos = np.array(fn(q), dtype=float).reshape(3)
                max_pos_error = max(max_pos_error, float(np.linalg.norm(ca_pos - pin_pos)))
        return max_pos_error

    @staticmethod
    def _read_joint_axes(urdf_path: str) -> Dict[str, np.ndarray]:
        root = ET.parse(urdf_path).getroot()
        axes = {}
        for joint in root.findall("joint"):
            name = joint.get("name")
            axis_node = joint.find("axis")
            if name is None or axis_node is None:
                continue
            axes[name] = np.array([float(v) for v in axis_node.get("xyz").split()], dtype=float)
        return axes

    @staticmethod
    def _joint_chain_to_frame(model, frame_id: int):
        chain = []
        joint_id = model.frames[frame_id].parentJoint
        while joint_id > 0:
            chain.append(joint_id)
            joint_id = model.parents[joint_id]
        return list(reversed(chain))

    def _chain_to_frame(self, frame_id: int):
        if frame_id not in self._frame_chain_cache:
            self._frame_chain_cache[frame_id] = self._joint_chain_to_frame(self.model, frame_id)
        return self._frame_chain_cache[frame_id]

    def _placement_to_frame(self, frame_id: int):
        if frame_id not in self._frame_placement_cache:
            self._frame_placement_cache[frame_id] = self.model.frames[frame_id].placement
        return self._frame_placement_cache[frame_id]

    @staticmethod
    def _se3_to_ca(se3):
        import casadi as ca

        T = ca.MX.eye(4)
        T[:3, :3] = ca.DM(se3.rotation)
        T[:3, 3] = ca.DM(se3.translation)
        return T

    @staticmethod
    def _rotation_transform_ca(axis: np.ndarray, theta):
        import casadi as ca

        norm = float(np.linalg.norm(axis))
        if norm <= 0.0:
            raise ValueError("URDF joint axis 不能为零向量")
        x, y, z = (axis / norm).tolist()
        c = ca.cos(theta)
        s = ca.sin(theta)
        one_c = 1.0 - c
        R = ca.MX(3, 3)
        R[0, 0] = c + x * x * one_c
        R[0, 1] = x * y * one_c - z * s
        R[0, 2] = x * z * one_c + y * s
        R[1, 0] = y * x * one_c + z * s
        R[1, 1] = c + y * y * one_c
        R[1, 2] = y * z * one_c - x * s
        R[2, 0] = z * x * one_c - y * s
        R[2, 1] = z * y * one_c + x * s
        R[2, 2] = c + z * z * one_c

        T = ca.MX.eye(4)
        T[:3, :3] = R
        return T


class _AcadosMpcBackend:
    """ACADOS 实时 NMPC 后端。

    ACADOS 是外部可选依赖。没有安装时,只有选择 solver_backend='acados'
    才会报错,不影响 casadi/scipy 后端。
    """

    def __init__(self, owner: MpcPinocchioIK):
        try:
            import casadi as ca
            from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
        except Exception as exc:
            raise RuntimeError(
                "已选择 ACADOS 后端,但当前环境没有 acados_template。"
                "请外部安装 ACADOS Python 接口并 source scripts/setup_acados_env.sh; "
                "或改用 solver_backend='casadi' / 'scipy'。"
            ) from exc

        self.owner = owner
        self.ca = ca
        cfg = owner.config
        nq = owner.nq
        nx = 2 * nq
        nu = nq
        np_param = 3 + 9 + 2 * nq

        x = ca.MX.sym("x", nx)
        u = ca.MX.sym("u", nu)
        p = ca.MX.sym("p", np_param)
        q = x[:nq]
        dq_prev = x[nq:]
        target_pos = p[:3]
        target_rot = ca.MX(3, 3)
        for row in range(3):
            for col in range(3):
                target_rot[row, col] = p[3 + row * 3 + col]
        q_ref_offset = 3 + 9
        q_ref = p[q_ref_offset:q_ref_offset + nq]
        q_center = p[q_ref_offset + nq:q_ref_offset + 2 * nq]

        model = AcadosModel()
        model.name = "windylab_arm_position_mpc"
        model.x = x
        model.u = u
        model.p = p
        # 离散模型:控制量 u 就是一个控制周期内的关节增量 dq。
        model.disc_dyn_expr = ca.vertcat(q + u, u)

        ee_pos = owner._symbolic_fk.position(q)
        ee_rot = owner._symbolic_fk.rotation(q)
        pos_error = ee_pos - target_pos
        rot_error = owner._orientation_error_ca(ee_rot, target_rot)
        obstacle_residuals = owner._obstacle_residuals_ca(q)
        n_obstacle_residuals = int(obstacle_residuals.shape[0])
        n_front_obstacle_residuals = owner._front_obstacle_residual_count()
        n_body_obstacle_residuals = owner._body_obstacle_residual_count()
        if n_obstacle_residuals != n_front_obstacle_residuals + n_body_obstacle_residuals:
            raise RuntimeError(
                "避障残差维度不一致: "
                f"expr={n_obstacle_residuals}, front={n_front_obstacle_residuals}, body={n_body_obstacle_residuals}"
            )
        y = ca.vertcat(pos_error, rot_error, u, u - dq_prev, q - q_ref, q - q_center, obstacle_residuals)
        y_e = ca.vertcat(pos_error, rot_error, q - q_ref, q - q_center, obstacle_residuals)
        model.cost_y_expr = y
        model.cost_y_expr_e = y_e

        ocp = AcadosOcp()
        ocp.model = model
        ocp.solver_options.N_horizon = cfg.horizon_steps
        ocp.dims.np = np_param
        ocp.parameter_values = np.zeros(np_param)

        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = np.diag(np.concatenate([
            np.full(3, cfg.position_weight),
            np.full(3, cfg.orientation_weight),
            np.full(nq, cfg.joint_delta_weight),
            np.full(nq, cfg.joint_delta_change_weight),
            np.full(nq, cfg.current_posture_weight),
            np.full(nq, cfg.center_posture_weight),
            np.full(n_front_obstacle_residuals, cfg.obstacle_weight),
            np.full(n_body_obstacle_residuals, cfg.body_obstacle_weight),
        ]))
        ocp.cost.W_e = np.diag(np.concatenate([
            np.full(3, cfg.terminal_position_weight),
            np.full(3, cfg.terminal_orientation_weight),
            np.full(nq, cfg.current_posture_weight),
            np.full(nq, cfg.center_posture_weight),
            np.full(n_front_obstacle_residuals, cfg.obstacle_weight),
            np.full(n_body_obstacle_residuals, cfg.body_obstacle_weight),
        ]))
        ocp.cost.yref = np.zeros(6 + 4 * nq + n_obstacle_residuals)
        ocp.cost.yref_e = np.zeros(6 + 2 * nq + n_obstacle_residuals)

        x_min = np.concatenate([owner.q_lower, np.full(nq, -cfg.max_joint_step)])
        x_max = np.concatenate([owner.q_upper, np.full(nq, cfg.max_joint_step)])
        ocp.constraints.idxbx = np.arange(nx)
        ocp.constraints.lbx = x_min
        ocp.constraints.ubx = x_max
        ocp.constraints.idxbx_e = np.arange(nx)
        ocp.constraints.lbx_e = x_min
        ocp.constraints.ubx_e = x_max
        ocp.constraints.idxbu = np.arange(nu)
        ocp.constraints.lbu = np.full(nu, -cfg.max_joint_step)
        ocp.constraints.ubu = np.full(nu, cfg.max_joint_step)
        ocp.constraints.x0 = np.zeros(nx)

        ocp.solver_options.tf = cfg.horizon_steps * cfg.control_dt
        ocp.solver_options.integrator_type = "DISCRETE"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.print_level = 0

        build_dir = Path(cfg.acados_build_dir).expanduser().resolve()
        build_dir.mkdir(parents=True, exist_ok=True)
        code_export_directory = build_dir / "c_generated_code"
        ocp.code_gen_opts.code_export_directory = str(code_export_directory)
        json_path = build_dir / "acados_ocp.json"
        shared_library_path = code_export_directory / f"libacados_ocp_solver_{model.name}.so"
        metadata_path = build_dir / "acados_solver_metadata.json"
        signature = _acados_solver_signature(
            owner=owner,
            nx=nx,
            nu=nu,
            np_param=np_param,
            n_obstacle_residuals=n_obstacle_residuals,
            model_name=model.name,
        )
        reuse_generated_solver = _can_reuse_acados_solver(
            metadata_path=metadata_path,
            json_path=json_path,
            shared_library_path=shared_library_path,
            signature=signature,
        )
        try:
            self.solver = AcadosOcpSolver(
                ocp,
                json_file=str(json_path),
                generate=not reuse_generated_solver,
                build=not reuse_generated_solver,
            )
            if not reuse_generated_solver:
                _write_acados_solver_metadata(metadata_path, signature)
        except OSError as exc:
            if reuse_generated_solver:
                self.solver = AcadosOcpSolver(ocp, json_file=str(json_path), generate=True, build=True)
                _write_acados_solver_metadata(metadata_path, signature)
                return
            raise RuntimeError(
                "ACADOS 求解器已生成,但动态库加载失败。请先运行: "
                "source src/arm-platform/scripts/setup_acados_env.sh; "
                "或改用 solver_backend='casadi' / 'scipy'。原始错误: "
                f"{exc}"
            ) from exc

    def set(self, *args, **kwargs):
        return self.solver.set(*args, **kwargs)

    def get(self, *args, **kwargs):
        return self.solver.get(*args, **kwargs)

    def solve(self):
        return self.solver.solve()


def _read_urdf_joint_origins_and_axes(urdf_path: str) -> Dict[str, Dict[str, np.ndarray]]:
    root = ET.parse(urdf_path).getroot()
    joints = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name is None:
            continue
        origin = joint.find("origin")
        axis = joint.find("axis")
        xyz = np.zeros(3)
        rpy = np.zeros(3)
        axis_xyz = np.array([0.0, 0.0, 1.0], dtype=float)
        if origin is not None:
            if origin.get("xyz"):
                xyz = np.array([float(v) for v in origin.get("xyz").split()], dtype=float)
            if origin.get("rpy"):
                rpy = np.array([float(v) for v in origin.get("rpy").split()], dtype=float)
        if axis is not None and axis.get("xyz"):
            axis_xyz = np.array([float(v) for v in axis.get("xyz").split()], dtype=float)
        joints[name] = {"xyz": xyz, "rpy": rpy, "axis": axis_xyz}
    return joints


def _xyz_rpy_to_transform(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _rpy_to_rotation(rpy)
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return T


def _rpy_to_rotation(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = np.asarray(rpy, dtype=float)
    cr, sr = _cos_sin(roll)
    cp, sp = _cos_sin(pitch)
    cy, sy = _cos_sin(yaw)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    return Rz @ Ry @ Rx


def _cos_sin(value: float) -> Tuple[float, float]:
    return float(np.cos(value)), float(np.sin(value))


def _orthonormal_basis_from_normal(normal: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    n = np.asarray(normal, dtype=float)
    norm = np.linalg.norm(n)
    if norm <= 0.0:
        raise ValueError("法向量不能为零")
    n = n / norm
    candidate = np.array([1.0, 0.0, 0.0])
    if abs(float(candidate @ n)) > 0.9:
        candidate = np.array([0.0, 1.0, 0.0])
    u = candidate - (candidate @ n) * n
    u = u / np.linalg.norm(u)
    v = np.cross(n, u)
    v = v / np.linalg.norm(v)
    return u, v


def _box_signed_distance_np(point: np.ndarray, box: np.ndarray) -> float:
    point = np.asarray(point, dtype=float)
    box = np.asarray(box, dtype=float)
    center = box[:3]
    half = box[3:]
    q = np.abs(point - center) - half
    outside = np.maximum(q, 0.0)
    outside_dist = float(np.linalg.norm(outside))
    inside_dist = float(min(max(q[0], q[1], q[2]), 0.0))
    return outside_dist + inside_dist


def _disk_signed_distance_np(point: np.ndarray, disk: np.ndarray) -> float:
    point = np.asarray(point, dtype=float)
    disk = np.asarray(disk, dtype=float)
    rel = point - disk[:3]
    normal = disk[3:6]
    radius = float(disk[6])
    d_axial = float(np.dot(normal, rel))
    lateral = rel - d_axial * normal
    r = float(np.linalg.norm(lateral))
    excess = max(r - radius, 0.0)
    return float(np.sqrt(excess ** 2 + d_axial ** 2))


def _acados_solver_signature(
    owner: MpcPinocchioIK,
    nx: int,
    nu: int,
    np_param: int,
    n_obstacle_residuals: int,
    model_name: str,
) -> Dict[str, object]:
    cfg = owner.config
    return {
        "signature_version": 1,
        "model_name": model_name,
        "urdf_path": str(Path(owner.urdf_path).expanduser().resolve()),
        "urdf_file": _file_signature(owner.urdf_path),
        "ee_frame": owner.ee_frame,
        "nq": owner.nq,
        "q_lower": np.round(owner.q_lower, 12).tolist(),
        "q_upper": np.round(owner.q_upper, 12).tolist(),
        "nx": int(nx),
        "nu": int(nu),
        "np": int(np_param),
        "horizon_steps": int(cfg.horizon_steps),
        "control_dt": float(cfg.control_dt),
        "max_joint_step": float(cfg.max_joint_step),
        "weights": {
            "position": float(cfg.position_weight),
            "terminal_position": float(cfg.terminal_position_weight),
            "orientation": float(cfg.orientation_weight),
            "terminal_orientation": float(cfg.terminal_orientation_weight),
            "joint_delta": float(cfg.joint_delta_weight),
            "joint_delta_change": float(cfg.joint_delta_change_weight),
            "current_posture": float(cfg.current_posture_weight),
            "center_posture": float(cfg.center_posture_weight),
        },
        "solver_options": {
            "integrator_type": "DISCRETE",
            "nlp_solver_type": "SQP_RTI",
            "qp_solver": "PARTIAL_CONDENSING_HPIPM",
            "hessian_approx": "GAUSS_NEWTON",
        },
        "obstacle": {
            "enabled": bool(cfg.obstacle_avoidance_enabled),
            "weight": float(cfg.obstacle_weight),
            "safety_margin": float(cfg.obstacle_safety_margin),
            "soft_eps": float(cfg.obstacle_soft_eps),
            "propeller_radius": float(cfg.obstacle_propeller_radius),
            "disk_half_thickness": float(cfg.obstacle_disk_half_thickness),
            "coordinate_mode": cfg.obstacle_coordinate_mode,
            "ll100_urdf_path": str(Path(cfg.obstacle_ll100_urdf_path).expanduser().resolve()),
            "ll100_urdf_file": _file_signature(cfg.obstacle_ll100_urdf_path),
            "keypoint_frames": list(cfg.obstacle_keypoint_frames),
            "front_propeller_joints": list(cfg.obstacle_front_propeller_joints),
            "disk_count": int(owner._obstacle_disks.shape[0]),
            "residual_count": int(n_obstacle_residuals),
            "disks": np.round(owner._obstacle_disks, 12).tolist(),
            "body_enabled": bool(cfg.body_obstacle_enabled),
            "body_weight": float(cfg.body_obstacle_weight),
            "body_margin": float(cfg.body_obstacle_margin),
            "body_boxes": np.round(owner._body_obstacle_boxes, 12).tolist(),
            "capsule_radius": float(cfg.capsule_radius),
            "capsule_samples_per_segment": int(cfg.capsule_samples_per_segment),
            "capsule_frame_sequence": list(cfg.capsule_frame_sequence),
            "front_residual_count": int(owner._front_obstacle_residual_count()),
            "body_residual_count": int(owner._body_obstacle_residual_count()),
        },
    }


def _file_signature(path: str) -> Dict[str, object]:
    resolved = Path(path).expanduser().resolve()
    try:
        stat = resolved.stat()
    except OSError:
        return {"path": str(resolved), "exists": False}
    return {
        "path": str(resolved),
        "exists": True,
        "size": int(stat.st_size),
        "mtime_ns": int(stat.st_mtime_ns),
    }


def _can_reuse_acados_solver(
    metadata_path: Path,
    json_path: Path,
    shared_library_path: Path,
    signature: Dict[str, object],
) -> bool:
    if not metadata_path.exists() or not json_path.exists() or not shared_library_path.exists():
        return False
    try:
        with metadata_path.open("r", encoding="utf-8") as f:
            metadata = json.load(f)
    except (OSError, json.JSONDecodeError):
        return False
    return metadata.get("signature") == signature


def _write_acados_solver_metadata(metadata_path: Path, signature: Dict[str, object]) -> None:
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    with metadata_path.open("w", encoding="utf-8") as f:
        json.dump({"signature": signature}, f, indent=2, sort_keys=True)


def _format_acados_status(status: int) -> str:
    meanings = {
        0: "ACADOS_SUCCESS",
        1: "ACADOS_FAILURE",
        2: "ACADOS_MAXITER",
        3: "ACADOS_MINSTEP",
        4: "ACADOS_QP_FAILURE",
    }
    return meanings.get(status, f"ACADOS_STATUS_{status}")


def _self_test() -> int:
    """命令行自测:构造 10 个局部可达目标,检查误差、限位和步长约束。"""
    ik = MpcPinocchioIK()
    rng = np.random.default_rng(1)
    print(
        f"MPC IK 模型: 自由度={ik.nq}, 预测步数={ik.config.horizon_steps}, "
        f"周期={ik.config.control_dt}, 求解器={ik.config.solver_backend}"
    )
    print(f"每步关节变化上限: {ik.config.max_joint_step:.3f} rad")
    fk_pos_error, fk_rot_error = ik._symbolic_fk.validate_against_pinocchio(ik.data, samples=100)
    print(
        "CasADi 符号 FK 对 Pinocchio FK 最大误差: "
        f"位置={fk_pos_error:.3e} m, 旋转={fk_rot_error:.3e}"
    )
    if fk_pos_error > 1e-9 or fk_rot_error > 1e-9:
        print("符号 FK 与 Pinocchio FK 不一致,请检查 URDF joint axis / placement")
        return 1
    obstacle_check = MpcPinocchioIK(
        config=MpcIkConfig(
            solver_backend="acados",
            obstacle_avoidance_enabled=True,
            acados_build_dir=ik.config.acados_build_dir + "_obstacle_check",
        )
    )
    keypoint_error = obstacle_check._symbolic_fk.validate_frame_positions_against_pinocchio(
        obstacle_check.data,
        tuple(obstacle_check._obstacle_keypoint_frame_ids.values()),
        samples=100,
    )
    disks = obstacle_check.obstacle_disks()
    disk_normal_norms = np.linalg.norm(disks[:, 3:6], axis=1) if len(disks) > 0 else np.array([])
    print(
        "前桨避障几何检查: "
        f"避障圆盘={len(disks)}, 半径范围="
        f"{np.min(disks[:, 6]):.3f}-{np.max(disks[:, 6]):.3f} m, "
        f"关键点 FK 最大误差={keypoint_error:.3e} m"
    )
    if (
        len(disks) != 2
        or disks.shape[1] != 7
        or not np.all(np.isfinite(disks))
        or not np.allclose(disk_normal_norms, 1.0, atol=1e-9)
        or keypoint_error > 1e-9
    ):
        print("前桨避障几何或关键点符号 FK 检查失败")
        return 1

    n_ok = 0
    n_ori_better = 0
    pos_only_ik = None
    ori_ik = None
    q_current = np.clip(pin.neutral(ik.model), ik.q_lower, ik.q_upper)
    warm_start = None
    for i in range(10):
        local_delta = rng.uniform(-0.12, 0.12, size=ik.nq)
        q_target = np.clip(q_current + local_delta, ik.q_lower, ik.q_upper)
        target_pos, target_rot = ik.forward(q_target)
        q_sol, ok, info = ik.solve(target_pos, q_init=q_current, warm_start=warm_start)
        warm_start = info["trajectory"]
        step_diffs = np.diff(np.vstack([q_current, info["trajectory"]]), axis=0)
        within_step = bool(np.all(np.abs(step_diffs) <= ik.config.max_joint_step + 1e-6))
        within_limits = bool(np.all(q_sol >= ik.q_lower - 1e-6) and np.all(q_sol <= ik.q_upper + 1e-6))
        ok = bool(ok and within_step and within_limits)
        n_ok += int(ok)
        print(
            f"[{i}] 终端误差={info['terminal_error']:.3e} 可用={ok} "
            f"迭代次数={info['nit']} 状态码={info['status']} 求解器={info['solver_backend']}"
        )
        q_current = q_sol

        if pos_only_ik is None or ori_ik is None:
            pos_only_cfg = MpcIkConfig(
                solver_backend=ik.config.solver_backend,
                acados_build_dir=ik.config.acados_build_dir + "_position_check",
                acados_rti_iterations=ik.config.acados_rti_iterations,
            )
            ori_cfg = MpcIkConfig(
                solver_backend=ik.config.solver_backend,
                acados_build_dir=ik.config.acados_build_dir + "_orientation",
                acados_rti_iterations=ik.config.acados_rti_iterations,
                orientation_weight=2.0,
                terminal_orientation_weight=20.0,
            )
            pos_only_ik = MpcPinocchioIK(config=pos_only_cfg)
            ori_ik = MpcPinocchioIK(config=ori_cfg)

        if ok:
            q_seed = np.clip(q_current + rng.uniform(-0.03, 0.03, size=ik.nq), ik.q_lower, ik.q_upper)
            _, _, pos_info = pos_only_ik.solve(target_pos, q_init=q_seed)
            _, ori_ok, ori_info = ori_ik.solve(target_pos, q_init=q_seed, target_rotation=target_rot)
            pos_terminal_rot = pos_info["trajectory"][-1]
            _, pos_rot = pos_only_ik.forward(pos_terminal_rot)
            pos_ori_error = float(np.linalg.norm(pos_only_ik._orientation_error(pos_rot, target_rot)))
            ori_error = ori_info["terminal_orientation_error"]
            better = bool(ori_ok and ori_error is not None and ori_error < pos_ori_error)
            n_ori_better += int(better)
            print(
                f"    姿态软约束检查: 未启用={pos_ori_error:.3e}, "
                f"启用={ori_error:.3e}, 改善={better}"
            )

    print(f"MPC IK 自测通过 {n_ok}/10")
    print(f"姿态软约束改善检查通过 {n_ori_better}/10")
    return 0 if n_ok == 10 and n_ori_better == 10 else 1


if __name__ == "__main__":
    raise SystemExit(_self_test())
