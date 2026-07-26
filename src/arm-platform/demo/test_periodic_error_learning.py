#!/usr/bin/env python3
"""Regression checks for the phase-locked periodic-error feedforward."""

import math
import unittest
from types import SimpleNamespace

import numpy as np

import moving_base_trajectory_config as traj
from move_base_circle_mpc_ik_demo import MoveBaseCircleMpcIkDemo


class _FakeKinematics:
    @staticmethod
    def forward(q):
        return np.asarray(q, dtype=float)[:3], np.eye(3)


class PeriodicErrorLearningTest(unittest.TestCase):

    @staticmethod
    def demo_stub(shape='circle'):
        demo = object.__new__(MoveBaseCircleMpcIkDemo)
        demo.trajectory_shape = shape
        demo.trajectory_time_scale = 1.0
        demo.base_motion_time_scale = 1.0
        demo.periodic_error_learning_enabled = True
        demo.periodic_error_learning_gain = 0.5
        demo.periodic_error_correction_limit_m = 0.005
        demo.periodic_error_update_limit_m = 0.015
        demo.periodic_error_min_samples_per_cycle = 1
        demo.periodic_error_learning_freeze = False
        demo.periodic_error_initial_coefficients = np.zeros(6)
        demo._periodic_error_coefficients = {}
        demo._periodic_error_accumulators = {}
        demo._last_periodic_learning_diag = {}
        demo.mpc_ik = _FakeKinematics()
        return demo

    def test_whole_uses_independent_surface_banks(self):
        demo = self.demo_stub('whole')
        for stroke_id, expected in (
                (0, 0), (100, 1), (200, 2), (209, 2),
                (300, 3), (400, 4), (10400, 4)):
            sample = SimpleNamespace(
                pen_down=True, stroke_id=stroke_id)
            self.assertEqual(
                demo._periodic_learning_key(sample), expected)
        self.assertEqual(
            demo._periodic_learning_key(SimpleNamespace(
                pen_down=False, stroke_id=-1)),
            -1)

    def test_lms_update_opposes_measured_error(self):
        demo = self.demo_stub()
        demo._base_pose = lambda _t: np.zeros(3)
        demo._base_rotation = lambda _t, _pt=None: np.eye(3)
        demo._ee_trajectory_sample = lambda _t: SimpleNamespace(
            position=np.zeros(3), pen_down=True, stroke_id=0)
        # t=T/4 gives basis=[1, 0]. A +1 mm x error must learn a
        # negative x correction at the same base phase.
        time_value = 0.25 * traj.BASE_PERIOD_SEC
        demo._update_periodic_error_learning(
            np.array([0.001, 0.0, 0.0]), time_value)
        diag = demo._update_periodic_error_learning(
            np.array([0.001, 0.0, 0.0]),
            time_value + traj.BASE_PERIOD_SEC)
        self.assertTrue(diag['active'])
        self.assertLess(diag['correction_world'][0], 0.0)
        np.testing.assert_allclose(
            demo._periodic_error_coefficients[0][0, 0],
            -2.0 * demo.periodic_error_learning_gain * 0.001,
            atol=1e-12)

    def test_coefficient_clamp_bounds_every_phase(self):
        demo = self.demo_stub()
        demo.periodic_error_learning_gain = 1.0
        demo.periodic_error_update_limit_m = 1.0
        demo.periodic_error_correction_limit_m = 0.002
        demo._base_pose = lambda _t: np.zeros(3)
        demo._base_rotation = lambda _t, _pt=None: np.eye(3)
        demo._ee_trajectory_sample = lambda _t: SimpleNamespace(
            position=np.zeros(3), pen_down=True, stroke_id=0)
        for index in range(100):
            t = index * traj.BASE_PERIOD_SEC / 20.0
            demo._update_periodic_error_learning(
                np.array([0.1, -0.1, 0.1]), t)
        coefficients = demo._periodic_error_coefficients[0]
        self.assertLessEqual(
            np.linalg.norm(coefficients),
            demo.periodic_error_correction_limit_m + 1e-12)
        for phase in np.linspace(0.0, 2.0 * math.pi, 200):
            correction = coefficients @ np.array([
                math.sin(phase), math.cos(phase)])
            self.assertLessEqual(
                np.linalg.norm(correction),
                demo.periodic_error_correction_limit_m + 1e-12)


if __name__ == '__main__':
    unittest.main()
