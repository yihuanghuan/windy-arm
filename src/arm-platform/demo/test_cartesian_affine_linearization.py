#!/usr/bin/env python3
"""Finite-difference regression for external task-space affine residuals."""

import numpy as np
import pinocchio as pin

from pinocchio_ik import PinocchioIK
from second_order_mpc_ik import SecondOrderMpcIK


def evaluate(solver, q, dq, ddq):
    data = solver.model.createData()
    reference = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    pin.forwardKinematics(solver.model, data, q, dq, ddq)
    pin.updateFramePlacements(solver.model, data)
    velocity = np.asarray(pin.getFrameVelocity(
        solver.model, data, solver.frame_id, reference).linear)
    acceleration = np.asarray(pin.getFrameClassicalAcceleration(
        solver.model, data, solver.frame_id, reference).linear)
    return velocity, acceleration


def central_jacobian(function, value, epsilon=1e-6):
    columns = []
    for index in range(value.size):
        offset = np.zeros_like(value)
        offset[index] = epsilon
        columns.append(
            (function(value + offset) - function(value - offset))
            / (2.0 * epsilon))
    return np.column_stack(columns)


def main():
    seed = PinocchioIK()
    q, ok = seed.solve(np.array([0.30, 0.01, 0.10]))
    assert ok
    solver = SecondOrderMpcIK()
    dq = np.linspace(-0.15, 0.12, solver.nq)
    ddq = np.linspace(0.25, -0.20, solver.nq)
    target_v = np.array([0.01, -0.02, 0.03])
    target_a = np.array([0.20, -0.10, 0.05])
    terms = solver.cartesian_affine_linearization(
        solver.model.createData(), q, dq, ddq, target_v, target_a)
    finite_differences = {
        "A_v": central_jacobian(
            lambda value: evaluate(solver, value, dq, ddq)[0], q),
        "B_v": central_jacobian(
            lambda value: evaluate(solver, q, value, ddq)[0], dq),
        "A_a": central_jacobian(
            lambda value: evaluate(solver, value, dq, ddq)[1], q),
        "B_a": central_jacobian(
            lambda value: evaluate(solver, q, value, ddq)[1], dq),
        "C_a": central_jacobian(
            lambda value: evaluate(solver, q, dq, value)[1], ddq),
    }
    for name, expected in finite_differences.items():
        error = np.max(np.abs(terms[name] - expected))
        assert error < 1e-6, (name, error)
    assert np.max(np.abs(terms["B_v"] - terms["C_a"])) < 1e-12
    print("ALL PASS")


if __name__ == "__main__":
    main()
