#!/usr/bin/env python3
"""Regression checks for world/base derivatives and contour semantics."""

import unittest
from types import SimpleNamespace

import numpy as np

from move_base_circle_mpc_ik_demo import MoveBaseCircleMpcIkDemo


class ExplicitReferenceKinematicsTest(unittest.TestCase):

    @staticmethod
    def demo_stub():
        demo = object.__new__(MoveBaseCircleMpcIkDemo)
        demo.trajectory_shape = 'whole'
        demo.trajectory_time_scale = 1.0
        demo.base_motion_time_scale = 1.0
        demo.reference_derivative_dt_sec = 0.005
        demo.reference_lead_steps = 1
        demo.horizon_steps = 10
        demo.dt = 0.02
        demo.online_time_scaling_enabled = False
        demo._last_reference_kinematics = {}
        return demo

    def test_relative_derivatives_match_direct_difference(self):
        demo = self.demo_stub()
        h = demo.reference_derivative_dt_sec
        for time_value in (1.3, 9.2, 18.0, 45.0, 61.0, 72.0):
            with self.subTest(time_value=time_value):
                reference = demo._reference_kinematics_base(time_value)
                minus = demo._ee_target_base(time_value - h)
                value = demo._ee_target_base(time_value)
                plus = demo._ee_target_base(time_value + h)
                np.testing.assert_allclose(
                    reference['velocity_base'],
                    (plus - minus) / (2.0 * h),
                    atol=2e-5, rtol=2e-4)
                np.testing.assert_allclose(
                    reference['acceleration_base'],
                    (plus - 2.0 * value + minus) / (h * h),
                    atol=2e-4, rtol=2e-4)

    def test_horizon_shapes_and_unit_tangents(self):
        demo = self.demo_stub()
        values = demo._target_horizon_base_kinematics(8.0, 10)
        expected_shapes = ((10, 3), (10, 3, 3), (10, 3),
                           (10, 3), (10, 3), (10,))
        self.assertEqual(tuple(value.shape for value in values), expected_shapes)
        tangents, active = values[-2], values[-1]
        for tangent, enabled in zip(tangents, active):
            if enabled:
                self.assertAlmostEqual(np.linalg.norm(tangent), 1.0, places=10)
        self.assertTrue(np.all(np.isfinite(np.concatenate([
            value.reshape(-1) for value in values]))))

    def test_loop_csv_schema_width(self):
        demo = self.demo_stub()

        class CaptureWriter:
            def __init__(self):
                self.rows = []

            def writerow(self, row):
                self.rows.append(row)

        writer = CaptureWriter()
        demo.record_loop_csv = True
        demo._loop_record_failed = False
        demo._loop_writer = writer
        demo._loop_rows_since_flush = 0
        demo._loop_file = None
        demo.dart_noise_std = 0.0
        demo._prev_u0 = None
        demo.traj_origin_wall = 0.0
        demo._js_header_stamp_sec = 0.0
        demo._js_recv_wall_sec = 0.0
        demo.time_scaler = SimpleNamespace(path_time=0.0, scale=1.0)
        demo._last_path_target_time0 = 1.02
        demo.solver_model = 'second_order'
        zeros = np.zeros(7)
        demo._append_loop_row(
            {
                't_solve': 1.0, 't_publish': 1.001,
                't_reference': 1.02, 't_next': 1.021,
                'timer_interval_ms': 20.0,
                'solve_to_publish_ms': 1.0,
                'feedback_age_ms': 0.5,
                't_solve_wall': 1.0, 't_publish_wall': 1.001,
            },
            {
                'input': zeros, 'raw': zeros,
                'velocity_estimate': zeros,
                'source': 1, 'fallback': 0, 'feedback_dt': 0.02,
            },
            q_current=zeros, dq_measured=zeros, q_next=zeros,
            q_published=zeros, dq_published=zeros,
            info={'status': 0, 'success': True, 'nit': 3,
                  'solve_time': 0.001, 'plan_feasible': True},
            command_usable=True, tracking_ok=True, guard_block=False,
            command_fk_step=0.0,
        )
        self.assertFalse(demo._loop_record_failed)
        self.assertEqual(len(writer.rows), 1)
        self.assertEqual(
            len(writer.rows[0]), len(demo._loop_csv_header()))


if __name__ == '__main__':
    unittest.main()
