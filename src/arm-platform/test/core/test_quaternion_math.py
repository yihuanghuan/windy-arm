import math
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.append(str(Path(__file__).resolve().parents[2]))

from task_space_smc.core.quaternion_math import (
    quat_multiply,
    quat_to_rotation,
    quaternion_log_error,
    ros_xyzw_to_wxyz,
    so3_left_jacobian,
    so3_left_jacobian_inverse,
    so3_left_jacobian_inverse_dot,
    wxyz_to_ros_xyzw,
)


def quat_axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    return np.array([math.cos(angle / 2.0), *(math.sin(angle / 2.0) * axis)])


def test_quaternion_sign_invariance_and_zero_error():
    q = quat_axis_angle([0.0, 0.0, 1.0], 0.7)
    assert np.allclose(quaternion_log_error(q, q), np.zeros(3), atol=1e-12)
    e1 = quaternion_log_error(q, np.array([1.0, 0.0, 0.0, 0.0]))
    e2 = quaternion_log_error(-q, np.array([1.0, 0.0, 0.0, 0.0]))
    assert np.allclose(e1, e2, atol=1e-12)


def test_known_angle_errors_and_pin_log3_consistency():
    for angle in [1e-7, math.pi / 2.0, math.radians(179.0)]:
        qd = quat_axis_angle([1.0, 0.0, 0.0], angle)
        q = np.array([1.0, 0.0, 0.0, 0.0])
        e = quaternion_log_error(qd, q)
        assert np.allclose(e, np.array([angle, 0.0, 0.0]), atol=1e-8)
        assert np.allclose(e, pin.log3(quat_to_rotation(qd) @ quat_to_rotation(q).T), atol=1e-8)


def test_ros_quaternion_conversion_roundtrip():
    q_wxyz = quat_axis_angle([0.2, -0.3, 0.4], 0.8)
    assert np.allclose(ros_xyzw_to_wxyz(wxyz_to_ros_xyzw(q_wxyz)), q_wxyz, atol=1e-12)


def test_left_jacobian_inverse_and_derivative():
    phi = np.array([0.3, -0.2, 0.1])
    phi_dot = np.array([-0.4, 0.5, 0.2])
    assert np.allclose(so3_left_jacobian(phi) @ so3_left_jacobian_inverse(phi), np.eye(3), atol=1e-10)
    h = 1e-6
    fd = (so3_left_jacobian_inverse(phi + h * phi_dot) - so3_left_jacobian_inverse(phi - h * phi_dot)) / (2.0 * h)
    assert np.allclose(so3_left_jacobian_inverse_dot(phi, phi_dot), fd, atol=1e-7)
