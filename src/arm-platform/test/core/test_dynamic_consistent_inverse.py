import sys
from pathlib import Path

import numpy as np

sys.path.append(str(Path(__file__).resolve().parents[2]))

from task_space_smc.core.model import FloatingBaseArmModel
from task_space_smc.core.redundancy_resolution import dynamic_consistent_inverse


def test_dynamic_consistent_inverse_full_rank_identity():
    model = FloatingBaseArmModel()
    rng = np.random.default_rng(4)
    q_full = model.neutral_q()
    for _ in range(200):
        q_full[7:] = rng.uniform([-1.0, -1.0, -0.8, -1.0, -0.8, -1.0, -1.0], [1.0, 0.8, 0.8, 1.0, 0.8, 1.0, 1.0])
        kin = model.compute_kinematics(q_full, np.zeros(model.model.nv))
        if np.linalg.matrix_rank(kin.J_arm_lwa, tol=1e-5) == 6:
            M = model.compute_arm_mass_block(q_full)
            Jbar, _, _ = dynamic_consistent_inverse(kin.J_arm_lwa, M, characteristic_length=1.0, damping=0.0)
            assert np.allclose(kin.J_arm_lwa @ Jbar, np.eye(6), atol=1e-7)
            N = np.eye(7) - Jbar @ kin.J_arm_lwa
            assert np.allclose(kin.J_arm_lwa @ N, np.zeros((6, 7)), atol=1e-7)
            return
    raise AssertionError("failed to find full-rank 6D arm Jacobian")
