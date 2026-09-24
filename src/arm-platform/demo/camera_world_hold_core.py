#!/usr/bin/env python3
"""World hold math. Twists use Pinocchio's [linear, angular] order."""
from pathlib import Path

import numpy as np
import pinocchio as pin


def default_urdf():
    local = Path(__file__).resolve().parents[1] / 'config' / 'arm6.urdf'
    if local.exists():
        return str(local)
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory('manipulator')) / 'arm6.urdf')


def vector(value, size):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f'expected {size} finite values')
    return result


def rotation(xyzw):
    q = vector(xyzw, 4)
    if abs(np.linalg.norm(q) - 1.0) > 0.01:
        raise ValueError('quaternion must be unit length; zero means mounting rotation is unknown')
    return pin.Quaternion(q / np.linalg.norm(q)).matrix()


def transform(xyz, xyzw):
    return pin.SE3(rotation(xyzw), vector(xyz, 3))


def clamp_norm(value, limit):
    return value * min(1.0, limit / max(np.linalg.norm(value), 1e-15))


def pose_deadzone(value, radius):
    if not np.isfinite(radius) or radius < 0:
        raise ValueError('pose deadband must be nonnegative and finite')
    norm = np.linalg.norm(value)
    return value * max(0., 1.-radius/max(norm, 1e-15))


def ordered_joints(names, positions, expected):
    if len(names) != len(positions) or len(set(names)) != len(names):
        raise ValueError('invalid JointState names/positions')
    if set(names) != set(expected):
        raise ValueError(f'expected active joints {expected}, received {list(names)}')
    table = dict(zip(names, positions))
    return vector([table[name] for name in expected], len(expected))


def add_joint6_housing_frame(model):
    """Stator frame at joint6's URDF origin, before its revolute motion."""
    name = 'joint6_housing'
    if model.existFrame(name):
        return model.getFrameId(name)
    if not model.existJointName('joint6'):
        raise ValueError('joint6 housing requires a joint6 URDF joint')
    joint = model.getJointId('joint6')
    return model.addFrame(pin.Frame(name, model.parents[joint],
                                    model.jointPlacements[joint], pin.FrameType.OP_FRAME))


class CameraKinematics:
    def __init__(self, urdf, parent_frame='link6', parent_to_camera=None):
        self.model = pin.buildModelFromUrdf(urdf)
        if self.model.nq != self.model.nv or self.model.nq not in (6, 7):
            raise ValueError('expected six/seven scalar active joints')
        self.names = list(self.model.names)[1:]
        if self.names != [f'joint{i+1}' for i in range(self.model.nq)]:
            raise ValueError('driver command order must be joint1..jointN')
        if parent_frame == 'joint6_housing':
            add_joint6_housing_frame(self.model)
        if not self.model.existFrame(parent_frame):
            raise ValueError(f'unknown parent frame {parent_frame}')
        self.parent_id = self.model.getFrameId(parent_frame)
        self.frame_id = None
        self.data = self.model.createData()
        if parent_to_camera is not None:
            self.set_camera(parent_to_camera)

    def set_camera(self, parent_to_camera):
        if self.frame_id is not None:
            raise ValueError('camera extrinsic cannot change during a run')
        parent = self.model.frames[self.parent_id]
        self.frame_id = self.model.addFrame(pin.Frame(
            'd435_camera_control', parent.parentJoint, self.parent_id,
            parent.placement * parent_to_camera, pin.FrameType.OP_FRAME))
        self.data = self.model.createData()

    def check_q(self, q):
        q = vector(q, self.model.nq)
        if np.any(q < self.model.lowerPositionLimit) or np.any(q > self.model.upperPositionLimit):
            raise ValueError('joint feedback outside URDF limits')
        return q

    def fk(self, q):
        if self.frame_id is None:
            raise ValueError('camera mounting transform unavailable')
        pin.framesForwardKinematics(self.model, self.data, self.check_q(q))
        return self.data.oMf[self.frame_id].copy()

    def jacobian(self, q):
        if self.frame_id is None:
            raise ValueError('camera mounting transform unavailable')
        return pin.computeFrameJacobian(self.model, self.data, self.check_q(q),
                                        self.frame_id, pin.LOCAL).copy()

    def step(self, q, measured, desired, dt, kp_position=1.5, kp_rotation=1.5,
             max_linear_velocity=0.05, max_angular_velocity=0.2,
             max_joint_velocity=0.3, damping_lambda=0.03,
             position_deadband=0., rotation_deadband=0., feedforward=None):
        q = self.check_q(q)
        scalars = np.array([dt, kp_position, kp_rotation, max_linear_velocity,
                            max_angular_velocity, max_joint_velocity, damping_lambda])
        if not np.isfinite(scalars).all() or np.any(scalars <= 0):
            raise ValueError('control parameters must be finite and positive')
        ep = desired.translation - measured.translation
        re = desired.rotation @ measured.rotation.T
        er = pin.log3(re)
        # Keep raw error for diagnostics. A continuous radial deadzone prevents
        # stationary measurement noise from winding up the position reference.
        ff = np.zeros(6) if feedforward is None else vector(feedforward, 6)
        feedback = np.r_[kp_position * pose_deadzone(ep, position_deadband),
                         kp_rotation * pose_deadzone(er, rotation_deadband)]
        world_twist = np.r_[clamp_norm(feedback[:3] + ff[:3],
                                      max_linear_velocity),
                            clamp_norm(feedback[3:] + ff[3:],
                                      max_angular_velocity)]
        # Both blocks refer to the controlled frame origin; only rotate axes.
        # A full SE(3) adjoint here would incorrectly add a world-origin lever arm.
        local_twist = np.r_[measured.rotation.T @ world_twist[:3],
                            measured.rotation.T @ world_twist[3:]]
        j = self.jacobian(q)
        singular_values = np.linalg.svd(j, compute_uv=False)
        dq = j.T @ np.linalg.solve(j @ j.T + damping_lambda**2 * np.eye(6), local_twist)
        # Uniform scaling retains the Cartesian direction when a joint saturates.
        dq *= min(1.0, max_joint_velocity / max(np.max(np.abs(dq)), 1e-15))
        q_cmd = np.clip(q + dq * dt, self.model.lowerPositionLimit, self.model.upperPositionLimit)
        dq = (q_cmd - q) / dt
        if not np.isfinite(np.r_[ep, er, world_twist, dq, q_cmd]).all():
            raise ValueError('non-finite control result')
        return dict(position_error=ep, rotation_error=er,
                    feedback_twist=feedback, feedforward_twist=ff,
                    rpy_error=pin.rpy.matrixToRpy(re), twist=world_twist, dq=dq, q_cmd=q_cmd,
                    jacobian_singular_values=singular_values,
                    jacobian_rank=np.asarray(np.linalg.matrix_rank(j)),
                    jacobian_condition=np.asarray(singular_values[0]/singular_values[-1]
                                                  if singular_values[-1]>1e-12 else np.inf))


class Link6Kinematics(CameraKinematics):
    """Camera on the stator measures H; encoders supply H->E. Control E in 6D."""
    def __init__(self, urdf, parent_frame='joint6_housing', parent_to_camera=None):
        if parent_frame != 'joint6_housing':
            raise ValueError('link6 reconstruction requires camera on joint6_housing')
        super().__init__(urdf, parent_frame, parent_to_camera)
        self.link6_id = self.model.getFrameId('link6')

    def set_camera(self, parent_to_camera):
        super().set_camera(parent_to_camera)
        self.housing_camera = parent_to_camera.copy()

    def link6_fk(self, q):
        pin.framesForwardKinematics(self.model, self.data, self.check_q(q))
        return self.data.oMf[self.link6_id].copy()

    def housing_link6(self, q):
        pin.framesForwardKinematics(self.model, self.data, self.check_q(q))
        return self.data.oMf[self.parent_id].inverse() * self.data.oMf[self.link6_id]

    def reconstruct(self, q, world_camera):
        if self.frame_id is None:
            raise ValueError('camera mounting transform unavailable')
        return world_camera * self.housing_camera.inverse() * self.housing_link6(q)

    def jacobian(self, q):
        return pin.computeFrameJacobian(self.model, self.data, self.check_q(q),
                                        self.link6_id, pin.LOCAL).copy()

    def world_jacobian(self, q, world_link6):
        # LOCAL J is at the E origin in E axes. R_W_E rotates both blocks.
        # Equivalent to rotating LOCAL_WORLD_ALIGNED J by R_W_B; no base observer.
        j = self.jacobian(q)
        return np.vstack((world_link6.rotation @ j[:3], world_link6.rotation @ j[3:]))

    def external_velocity(self, q, dq, world_camera, camera_twist):
        """Subtract arm self-motion at C, then shift external twist C -> E.

        All inputs describe the SAME sensor timestamp. No world/base pose
        estimator: use R_W_C solely to express the camera's LOCAL Jacobian.
        q6 contributes to J_E and measured E angular velocity, but not J_C.
        """
        dq, camera_twist = vector(dq, self.model.nv), vector(camera_twist, 6)
        world_link6 = self.reconstruct(q, world_camera)
        jc = CameraKinematics.jacobian(self, q)
        jc = np.vstack((world_camera.rotation @ jc[:3], world_camera.rotation @ jc[3:]))
        robot_camera = jc @ dq
        external_camera = camera_twist - robot_camera
        offset = world_link6.translation - world_camera.translation
        external_link6 = np.r_[external_camera[:3] + np.cross(external_camera[3:], offset),
                              external_camera[3:]]
        measured_link6 = self.world_jacobian(q, world_link6) @ dq + external_link6
        return dict(camera_twist=camera_twist, robot_camera_twist=robot_camera,
                    external_link6_twist=external_link6, measured_link6_twist=measured_link6)
