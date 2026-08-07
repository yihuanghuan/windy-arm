#!/usr/bin/env python3
"""Regression tests for selectable moving-base drawing tasks."""

import unittest

import numpy as np

import moving_base_trajectory_config as trajectory


class MovingBaseTrajectoryShapesTest(unittest.TestCase):

    # `hold` 是一个不动的点（末端世界系定点保持），没有笔画可画。
    # 它照样要满足下面所有【采样】判据，只是不能要求它有可见折线。
    _SHAPES_WITHOUT_STROKES = ('hold',)

    def test_all_startup_tasks_are_finite_and_have_visible_strokes(self):
        for shape in trajectory.SUPPORTED_EE_TRAJECTORIES:
            with self.subTest(shape=shape):
                period = trajectory.ee_trajectory_period(shape)
                self.assertGreater(period, 0.0)
                segments = trajectory.sample_ee_drawing_segments(shape, 240)
                if shape in self._SHAPES_WITHOUT_STROKES:
                    self.assertEqual(segments, [])
                else:
                    self.assertTrue(segments)
                for segment in segments:
                    self.assertGreaterEqual(len(segment), 2)
                    self.assertTrue(np.all(np.isfinite(segment)))

                for time_value in np.linspace(
                        0.0, period, 101, endpoint=False):
                    sample = trajectory.ee_trajectory_sample(
                        float(time_value), shape)
                    self.assertTrue(np.all(np.isfinite(sample.position)))
                    np.testing.assert_allclose(
                        sample.rotation.T @ sample.rotation,
                        np.eye(3),
                        atol=1e-10,
                    )

    def test_standalone_circle_keeps_the_historical_path(self):
        expected_start = (
            trajectory.EE_WORLD_CIRCLE_CENTER
            + np.array([trajectory.EE_CIRCLE_RADIUS, 0.0, 0.0])
        )
        np.testing.assert_allclose(
            trajectory.ee_target_world(0.0, 'circle'),
            expected_start,
        )
        self.assertAlmostEqual(
            trajectory.base_nominal_yaw(0.0, 'circle'), 0.0)

    def test_windylab_has_ten_disconnected_visible_strokes(self):
        segments = trajectory.sample_ee_drawing_segments('windylab', 1000)
        self.assertEqual(len(segments), 10)
        self.assertAlmostEqual(
            trajectory.EE_WINDYLAB_LETTER_GAP, 0.016, places=12)
        self.assertTrue(trajectory.EE_WINDYLAB_MIRROR_X)
        # The first requested stroke ("w") starts on the right after the
        # complete word is mirrored; the last requested stroke ("b") is left.
        self.assertGreater(
            trajectory._WINDYLAB_STROKES[0][0, 0],
            trajectory._WINDYLAB_STROKES[-1][0, 0],
        )
        period = trajectory.ee_trajectory_period('windylab')
        samples = [
            trajectory.ee_trajectory_sample(float(time_value), 'windylab')
            for time_value in np.linspace(
                0.0, period, 2000, endpoint=False)
        ]
        self.assertTrue(any(not sample.pen_down for sample in samples))
        self.assertEqual(
            len({sample.stroke_id for sample in samples
                 if sample.pen_down}),
            10,
        )

    def test_whole_word_exits_from_b_without_returning_to_w(self):
        word_start = (
            trajectory._local_trajectory_period('circle')
            + trajectory._WHOLE_TRANSITION_SEC
            + trajectory._local_trajectory_period('triangle')
            + trajectory._WHOLE_TRANSITION_SEC
        )
        word_end = word_start + trajectory._WINDYLAB_ONCE_PERIOD_SEC
        sample = trajectory.ee_trajectory_sample(word_end, 'whole')
        expected_b_end = trajectory._place_local_sample(
            trajectory.EeTrajectorySample(
                trajectory._WINDYLAB_STROKES[-1][-1],
                True,
                len(trajectory._WINDYLAB_STROKES) - 1,
            ),
            'back',
        ).position
        w_start = trajectory._place_local_sample(
            trajectory.EeTrajectorySample(
                trajectory._WINDYLAB_STROKES[0][0], True, 0),
            'back',
        ).position

        self.assertFalse(sample.pen_down)
        np.testing.assert_allclose(sample.position, expected_b_end, atol=1e-8)
        self.assertGreater(
            np.linalg.norm(sample.position - w_start),
            0.05,
            'whole word must not touch the next cycle w start before transfer',
        )

    def test_quintic_smoothstep_has_zero_endpoint_velocity_and_acceleration(self):
        h = 1e-4
        for endpoint, direction in ((0.0, 1.0), (1.0, -1.0)):
            f0 = trajectory._smoothstep(endpoint)
            f1 = trajectory._smoothstep(endpoint + direction * h)
            f2 = trajectory._smoothstep(endpoint + direction * 2.0 * h)
            velocity = (f1 - f0) / h
            acceleration = (f2 - 2.0 * f1 + f0) / (h * h)
            self.assertLess(abs(velocity), 2e-6)
            self.assertLess(abs(acceleration), 0.02)

    def test_whole_task_has_five_surfaces_and_no_connecting_lines(self):
        # circle + triangle + ten windylab strokes + sine + square
        segments = trajectory.sample_ee_drawing_segments('whole', 240)
        self.assertEqual(len(segments), 14)

        circle, triangle = segments[:2]
        word = np.vstack(segments[2:12])
        sine, square = segments[12:]

        # Front/left/back/right vertical surfaces and the horizontal top.
        self.assertLess(np.ptp(circle[:, 0]), 1e-10)
        self.assertAlmostEqual(float(np.mean(circle[:, 0])), 0.30, places=8)
        self.assertLess(np.ptp(triangle[:, 1]), 1e-10)
        self.assertAlmostEqual(float(np.mean(triangle[:, 1])), 0.30, places=8)
        self.assertLess(np.ptp(word[:, 0]), 1e-10)
        self.assertAlmostEqual(float(np.mean(word[:, 0])), -0.30, places=8)
        # The requested horizontal mirror reverses the first "w" stroke on
        # the back board as well.
        self.assertLess(segments[2][-1, 1], segments[2][0, 1])
        self.assertLess(np.ptp(sine[:, 1]), 1e-10)
        self.assertAlmostEqual(float(np.mean(sine[:, 1])), -0.30, places=8)
        self.assertLess(np.ptp(square[:, 2]), 1e-10)
        self.assertAlmostEqual(float(np.mean(square[:, 2])), 0.30, places=8)

        period = trajectory.ee_trajectory_period('whole')
        pen_states = [
            trajectory.ee_pen_down(float(time_value), 'whole')
            for time_value in np.linspace(
                0.0, period, 2000, endpoint=False)
        ]
        self.assertIn(False, pen_states)


if __name__ == '__main__':
    unittest.main()
