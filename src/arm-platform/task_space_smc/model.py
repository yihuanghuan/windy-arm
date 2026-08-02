"""Pinocchio floating-base A-L1-GAMMA model wrapper."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pinocchio as pin

from .quaternion_math import quat_normalize


DEFAULT_ARM_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7")
DEFAULT_EE_FRAME = "link7"


def default_urdf_path() -> str:
    here = Path(__file__).resolve()
    source_tree_urdf = here.parent.parent / "config" / "arm.urdf"
    if source_tree_urdf.exists():
        return str(source_tree_urdf)
    try:
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(get_package_share_directory("manipulator"), "arm.urdf")
    except Exception:
        return str(source_tree_urdf)


@dataclass
class Kinematics:
    ee_position: np.ndarray
    ee_rotation: np.ndarray
    ee_quaternion_wxyz: np.ndarray
    ee_velocity_lwa: np.ndarray
    J_full_lwa: np.ndarray
    J_base_lwa: np.ndarray
    J_arm_lwa: np.ndarray


class FloatingBaseArmModel:
    """Free-flyer base plus the seven A-L1-GAMMA joints.

    q_full is [base_xyz(3), base_quat_xyzw(4), arm_q(7)] because that is the
    Pinocchio FreeFlyer convention. Public helpers accept base quaternions as
    [w, x, y, z] and convert at the boundary.
    """

    def __init__(
        self,
        urdf_path: str | None = None,
        ee_frame_name: str = DEFAULT_EE_FRAME,
        arm_joint_names: tuple[str, ...] = DEFAULT_ARM_JOINT_NAMES,
    ):
        self.urdf_path = urdf_path or default_urdf_path()
        self.model = pin.buildModelFromUrdf(self.urdf_path, pin.JointModelFreeFlyer())
        self.data = self.model.createData()
        self.ee_frame_name = ee_frame_name
        self.ee_frame_id = self.model.getFrameId(ee_frame_name)
        if self.ee_frame_id >= len(self.model.frames):
            raise ValueError(f"frame {ee_frame_name!r} was not found in {self.urdf_path}")
        self.arm_joint_names = tuple(arm_joint_names)
        if self.model.nq != 14 or self.model.nv != 13:
            raise ValueError(f"expected free-flyer 7DoF model nq=14,nv=13, got nq={self.model.nq},nv={self.model.nv}")
        self.arm_lower_position_limit = self.model.lowerPositionLimit[7:].copy()
        self.arm_upper_position_limit = self.model.upperPositionLimit[7:].copy()

    @staticmethod
    def make_q_full(base_position_w: np.ndarray, base_quat_wxyz: np.ndarray, q_arm: np.ndarray) -> np.ndarray:
        p = np.asarray(base_position_w, dtype=float).reshape(3)
        q_wxyz = quat_normalize(base_quat_wxyz)
        q_arm = np.asarray(q_arm, dtype=float).reshape(7)
        q_xyzw = np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]])
        return np.concatenate([p, q_xyzw, q_arm])

    @staticmethod
    def make_v_full(base_velocity_lwa: np.ndarray, qd_arm: np.ndarray) -> np.ndarray:
        return np.concatenate([np.asarray(base_velocity_lwa, dtype=float).reshape(6), np.asarray(qd_arm, dtype=float).reshape(7)])

    @staticmethod
    def make_a_full(base_acceleration_lwa: np.ndarray, qdd_arm: np.ndarray) -> np.ndarray:
        return np.concatenate([np.asarray(base_acceleration_lwa, dtype=float).reshape(6), np.asarray(qdd_arm, dtype=float).reshape(7)])

    def neutral_q(self) -> np.ndarray:
        return pin.neutral(self.model)

    def split_arm_q(self, q_full: np.ndarray) -> np.ndarray:
        return np.asarray(q_full, dtype=float).reshape(self.model.nq)[7:].copy()

    def compute_kinematics(self, q_full: np.ndarray, v_full: np.ndarray | None = None) -> Kinematics:
        q_full = np.asarray(q_full, dtype=float).reshape(self.model.nq)
        if v_full is None:
            v_full = np.zeros(self.model.nv)
        else:
            v_full = np.asarray(v_full, dtype=float).reshape(self.model.nv)
        pin.forwardKinematics(self.model, self.data, q_full, v_full)
        pin.updateFramePlacements(self.model, self.data)
        J_full = pin.computeFrameJacobian(
            self.model,
            self.data,
            q_full,
            self.ee_frame_id,
            pin.LOCAL_WORLD_ALIGNED,
        )
        vel = pin.getFrameVelocity(self.model, self.data, self.ee_frame_id, pin.LOCAL_WORLD_ALIGNED).vector
        oMf = self.data.oMf[self.ee_frame_id]
        q_xyzw = pin.Quaternion(oMf.rotation).coeffs()
        q_wxyz = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])
        return Kinematics(
            ee_position=oMf.translation.copy(),
            ee_rotation=oMf.rotation.copy(),
            ee_quaternion_wxyz=quat_normalize(q_wxyz),
            ee_velocity_lwa=vel.copy(),
            J_full_lwa=J_full.copy(),
            J_base_lwa=J_full[:, :6].copy(),
            J_arm_lwa=J_full[:, 6:].copy(),
        )

    def compute_mass_matrix(self, q_full: np.ndarray) -> np.ndarray:
        q_full = np.asarray(q_full, dtype=float).reshape(self.model.nq)
        M = pin.crba(self.model, self.data, q_full)
        return np.asarray((M + M.T) * 0.5)

    def compute_arm_mass_block(self, q_full: np.ndarray) -> np.ndarray:
        return self.compute_mass_matrix(q_full)[6:, 6:].copy()

    def compute_frame_jacobian(self, q_full: np.ndarray) -> np.ndarray:
        q_full = np.asarray(q_full, dtype=float).reshape(self.model.nq)
        return pin.computeFrameJacobian(
            self.model,
            self.data,
            q_full,
            self.ee_frame_id,
            pin.LOCAL_WORLD_ALIGNED,
        )

    def compute_task_bias_acceleration(self, q_full: np.ndarray, v_full: np.ndarray, a_base_lwa: np.ndarray) -> np.ndarray:
        q_full = np.asarray(q_full, dtype=float).reshape(self.model.nq)
        v_full = np.asarray(v_full, dtype=float).reshape(self.model.nv)
        a_bias = np.concatenate([np.asarray(a_base_lwa, dtype=float).reshape(6), np.zeros(7)])
        pin.forwardKinematics(self.model, self.data, q_full, v_full, a_bias)
        pin.updateFramePlacements(self.model, self.data)
        return pin.getFrameClassicalAcceleration(
            self.model,
            self.data,
            self.ee_frame_id,
            pin.LOCAL_WORLD_ALIGNED,
        ).vector.copy()

    def compute_rnea_arm_torque(
        self,
        q_full: np.ndarray,
        v_full: np.ndarray,
        a_base_lwa: np.ndarray,
        qdd_cmd: np.ndarray,
    ) -> np.ndarray:
        q_full = np.asarray(q_full, dtype=float).reshape(self.model.nq)
        v_full = np.asarray(v_full, dtype=float).reshape(self.model.nv)
        a_full = self.make_a_full(a_base_lwa, qdd_cmd)
        tau_full = pin.rnea(self.model, self.data, q_full, v_full, a_full)
        return np.asarray(tau_full[6:]).reshape(7).copy()

    def compute_arm_forward_dynamics(
        self,
        q_full: np.ndarray,
        v_full: np.ndarray,
        a_base_lwa: np.ndarray,
        tau_arm: np.ndarray,
    ) -> np.ndarray:
        M = self.compute_mass_matrix(q_full)
        n_full = pin.rnea(self.model, self.data, q_full, v_full, np.zeros(self.model.nv))
        rhs = np.asarray(tau_arm, dtype=float).reshape(7) - M[6:, :6] @ np.asarray(a_base_lwa, dtype=float).reshape(6) - n_full[6:]
        return np.linalg.solve(M[6:, 6:], rhs)
