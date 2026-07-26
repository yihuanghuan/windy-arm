#!/usr/bin/env python3
"""二阶 NMPC 的最小结构/真实 ACADOS 回归。"""

import numpy as np

from pinocchio_ik import PinocchioIK
from second_order_mpc_ik import SecondOrderMpcConfig, SecondOrderMpcIK


def main():
    dt = 0.02
    cfg = SecondOrderMpcConfig(
        horizon_steps=10,
        control_dt=dt,
        max_joint_velocity=1.0,
        max_joint_acceleration=5.0,
        position_weight=50.0,
        terminal_position_weight=500.0,
        orientation_weight=0.1,
        terminal_orientation_weight=1.0,
        acados_rti_iterations=3,
        rti_iteration_mode="fixed_iterations",
    )
    ik = PinocchioIK()
    start_target = np.array([0.30, 0.01, 0.10])
    q0, reachable = ik.solve(start_target)
    assert reachable
    solver = SecondOrderMpcIK(config=cfg)
    moving_target = start_target + np.array([0.0, 0.005, 0.0])
    _, start_rotation = solver.forward(q0)
    q1, dq1, _, info = solver.step(
        np.repeat(moving_target.reshape(1, 3), cfg.horizon_steps, axis=0),
        q0, np.zeros(7), posture_reference=q0,
        target_rotation=start_rotation,
        target_velocity=np.repeat(
            np.array([[0.0, 0.04, 0.0]]), cfg.horizon_steps, axis=0),
        target_acceleration=np.zeros((cfg.horizon_steps, 3)),
        contour_tangent=np.repeat(
            np.array([[0.0, 1.0, 0.0]]), cfg.horizon_steps, axis=0),
        contour_active=np.ones(cfg.horizon_steps))
    assert info["status"] == 0
    assert info["command_usable"]
    assert info["trajectory"].shape == (cfg.horizon_steps, 7)
    assert info["velocity_trajectory"].shape == (cfg.horizon_steps, 7)
    assert info["acceleration_trajectory"].shape == (cfg.horizon_steps, 7)
    assert np.max(np.abs(info["velocity_trajectory"])) <= 1.0 + 1e-7
    assert np.max(np.abs(info["acceleration_trajectory"])) <= 5.0 + 1e-7
    assert np.max(np.abs(info["acceleration_trajectory"])) > 1e-5
    assert info["orientation_active"]
    assert info["min_translation_jacobian_sigma"] >= 0.05 - 1e-6
    assert info["cartesian_residual_mode"] == "external_affine"
    assert info["projection_time"] == 0.0
    assert info["linearization_time"] == 0.0
    assert info["projection_velocity_residual_rms"] >= 0.0

    # 精确离散动力学一致性。
    xs_q = np.vstack([q0, info["trajectory"]])
    xs_v = np.vstack([np.zeros(7), info["velocity_trajectory"]])
    us = info["acceleration_trajectory"]
    q_res = xs_q[1:] - xs_q[:-1] - dt * xs_v[:-1] - 0.5 * dt * dt * us
    v_res = xs_v[1:] - xs_v[:-1] - dt * us
    assert np.max(np.abs(q_res)) < 1e-7
    assert np.max(np.abs(v_res)) < 1e-7
    assert np.allclose(q1, info["trajectory"][0])
    assert np.allclose(dq1, info["velocity_trajectory"][0])
    assert np.allclose(info["ddq0"], info["acceleration_trajectory"][0])
    solver.reset_warm_start()
    assert solver._last_plan is None
    assert solver._last_velocity_plan is None
    assert solver._last_acceleration_plan is None
    print("ALL PASS")


if __name__ == "__main__":
    main()
