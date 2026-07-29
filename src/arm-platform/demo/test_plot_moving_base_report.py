#!/usr/bin/env python3
"""Pure-function tests for the four-figure uncertainty report."""

import unittest

import numpy as np

from plot_moving_base_report import (
    disturbance_events,
    smooth_segments,
    trajectory_segments,
)


class MovingBaseReportTest(unittest.TestCase):

    def test_second_global_gust_is_first_visible_after_warmup(self):
        events = disturbance_events(
            'gust', start_sec=3.0, end_sec=20.0,
            base_motion_time_scale=1.0)
        self.assertEqual([event.number for event in events], [2, 3])
        self.assertEqual(
            (events[0].start_sec, events[0].peak_sec, events[0].end_sec),
            (8.0, 8.4, 8.8),
        )

    def test_segments_break_at_pen_lift_and_stroke_change(self):
        points = np.asarray([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [100.0, 0.0, 0.0],
            [101.0, 0.0, 0.0],
            [200.0, 0.0, 0.0],
            [201.0, 0.0, 0.0],
        ])
        pen_down = np.asarray([1, 1, 0, 1, 1, 1, 1], dtype=bool)
        stroke_id = np.asarray([0, 0, 0, 1, 1, 2, 2])
        segments = trajectory_segments(points, pen_down, stroke_id)
        self.assertEqual(len(segments), 3)
        np.testing.assert_array_equal(
            segments[0], points[[0, 1]])
        np.testing.assert_array_equal(
            segments[1], points[[3, 4]])
        np.testing.assert_array_equal(
            segments[2], points[[5, 6]])

    def test_display_smoothing_preserves_inputs_and_endpoints(self):
        segment = np.asarray([
            [float(index), (-1.0) ** index, 0.0]
            for index in range(15)
        ])
        original = segment.copy()
        smoothed = smooth_segments([segment], window=7)[0]
        np.testing.assert_array_equal(segment, original)
        np.testing.assert_array_equal(smoothed[0], original[0])
        np.testing.assert_array_equal(smoothed[-1], original[-1])
        self.assertLess(
            np.abs(np.diff(smoothed[:, 1], n=2)).mean(),
            np.abs(np.diff(original[:, 1], n=2)).mean(),
        )


if __name__ == '__main__':
    unittest.main()
