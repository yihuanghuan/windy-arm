import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.append(str(Path(__file__).resolve().parents[1]))

from task_space_smc.base_state import SimulationInjectedBase, SinusoidSpec
from task_space_smc.model import FloatingBaseArmModel


def test_floating_base_dimensions_jacobian_and_rnea():
    model = FloatingBaseArmModel()
    q_full = model.neutral_q()
    q_full[7:] = np.array([0.1, -0.4, 0.3, 0.2, -0.1, 0.15, -0.2])
    v_full = np.zeros(model.model.nv)
    kin = model.compute_kinematics(q_full, v_full)
    assert model.model.nq == 14
    assert model.model.nv == 13
    assert kin.J_full_lwa.shape == (6, 13)
    assert kin.J_arm_lwa.shape == (6, 7)
    tau = model.compute_rnea_arm_torque(q_full, v_full, np.zeros(6), np.zeros(7))
    assert tau.shape == (7,)
    assert np.all(np.isfinite(tau))


def test_task_bias_acceleration_matches_central_difference():
    model = FloatingBaseArmModel()
    base = SimulationInjectedBase(
        SinusoidSpec.from_lists([0.01, 0.0, 0.0], [0.4, 0.0, 0.0]),
        SinusoidSpec.from_lists(np.deg2rad([2.0, 1.0, 0.0]), [0.3, 0.5, 0.0]),
    )
    bs = base.sample(0.2)
    q_arm = np.array([0.2, -0.5, 0.4, -0.2, 0.15, 0.1, -0.1])
    qd_arm = np.array([0.1, -0.05, 0.02, 0.03, -0.02, 0.04, -0.03])
    q = model.make_q_full(bs.position_w, bs.quaternion_wxyz, q_arm)
    v = model.make_v_full(bs.velocity_lwa, qd_arm)
    b6 = model.compute_task_bias_acceleration(q, v, bs.acceleration_lwa)
    a = np.concatenate([bs.acceleration_lwa, np.zeros(7)])
    h = 1e-5
    q_plus = pin.integrate(model.model, q, v * h + 0.5 * a * h * h)
    q_minus = pin.integrate(model.model, q, -v * h + 0.5 * a * h * h)
    v_plus = v + a * h
    v_minus = v - a * h
    vel_plus = model.compute_kinematics(q_plus, v_plus).ee_velocity_lwa
    vel_minus = model.compute_kinematics(q_minus, v_minus).ee_velocity_lwa
    fd = (vel_plus - vel_minus) / (2.0 * h)
    assert np.allclose(b6, fd, atol=3e-4)
