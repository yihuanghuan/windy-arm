#!/usr/bin/env python3
"""The reference projector must never change or violate hardware limits."""

import numpy as np

from pinocchio_ik import PinocchioIK
from second_order_mpc_ik import SecondOrderMpcConfig, SecondOrderMpcIK


def main():
    cfg = SecondOrderMpcConfig(
        max_joint_velocity=1.0,
        max_joint_acceleration=5.0,
        feasible_reference_projection=True,
    )
    seed = PinocchioIK()
    q, ok = seed.solve(np.array([0.30, 0.01, 0.10]))
    assert ok
    solver = SecondOrderMpcIK(config=cfg)
    result = solver.project_feasible_reference(
        solver.model.createData(), q, np.zeros(7),
        np.array([0.35, -0.25, 0.30]),
        np.array([3.0, -2.5, 2.8]))
    assert np.max(np.abs(result["qdot"])) <= cfg.max_joint_velocity + 1e-12
    assert np.max(np.abs(result["qddot"])) <= cfg.max_joint_acceleration + 1e-12
    assert result["velocity_residual"] >= 0.0
    assert result["acceleration_residual"] >= 0.0

    # With dq near +vmax, positive acceleration must be tightened so the next
    # double-integrator state also satisfies the unchanged velocity bound.
    dq = np.full(7, 0.99)
    tightened = solver.project_feasible_reference(
        solver.model.createData(), q, dq,
        np.array([1.0, 1.0, 1.0]), np.array([10.0, 10.0, 10.0]))
    next_dq = dq + cfg.control_dt * tightened["qddot"]
    assert np.max(next_dq) <= cfg.max_joint_velocity + 1e-12
    assert np.min(next_dq) >= -cfg.max_joint_velocity - 1e-12
    print("ALL PASS")


if __name__ == "__main__":
    main()
