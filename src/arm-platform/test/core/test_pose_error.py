import math
import sys
from pathlib import Path

import numpy as np

sys.path.append(str(Path(__file__).resolve().parents[2]))

from task_space_smc.core.pose_error import PoseState, compute_pose_error
from task_space_smc.core.quaternion_math import quat_to_rotation


def quat_axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    return np.array([math.cos(angle / 2.0), *(math.sin(angle / 2.0) * axis)])


def test_strict_orientation_rate_uses_error_rotation():
    actual_q = quat_axis_angle([0.0, 0.0, 1.0], math.radians(60.0))
    desired_q = np.array([1.0, 0.0, 0.0, 0.0])
    actual = PoseState(np.zeros(3), actual_q, np.zeros(3), np.array([1.0, 0.0, 0.0]))
    desired = PoseState(np.zeros(3), desired_q, np.zeros(3), np.zeros(3))
    err = compute_pose_error(desired, actual)
    R_err = quat_to_rotation(err.q_error_wxyz)
    assert np.allclose(err.omega_error, -R_err @ actual.angular_velocity)
    assert not np.allclose(err.omega_error, -actual.angular_velocity)
