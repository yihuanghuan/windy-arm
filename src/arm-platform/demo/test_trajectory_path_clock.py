#!/usr/bin/env python3
"""Pure unit tests for the trajectory visualizer's synchronized path clock."""

import unittest
from collections import deque
from types import SimpleNamespace

import numpy as np
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

import moving_base_trajectory_config as moving_traj
from trajectory_visualizer_demo import TrajectoryVisualizerDemo


class _LoggerStub:
    def __init__(self):
        self.warnings = []

    def warn(self, message, **_kwargs):
        self.warnings.append(str(message))


class _IkStub:
    def __init__(self, position):
        self.position = np.asarray(position, dtype=float)

    def forward(self, _q):
        return self.position.copy(), np.eye(3)


class _PublisherStub:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _visualizer_stub(
        *, mode='required', required=True, start_time=10.0):
    visualizer = object.__new__(TrajectoryVisualizerDemo)
    visualizer.trajectory_mode = 'moving_base'
    visualizer.path_clock_mode = mode
    visualizer.path_clock_required_for_run = required
    visualizer.path_clock_max_extrapolation_sec = 0.10
    visualizer.path_progress_samples = deque(maxlen=128)
    visualizer.path_clock_wait_reported = False
    visualizer.start_time = start_time
    visualizer.samples = []
    visualizer.command_fk_samples = []
    logger = _LoggerStub()
    visualizer.get_logger = lambda: logger
    return visualizer, logger


def _path_clock_message(
        stamp_sec, base_t, path_t, scale, epoch='moving_base_path_clock'):
    whole_sec = int(stamp_sec)
    nanosec = int(round((float(stamp_sec) - whole_sec) * 1e9))
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=whole_sec, nanosec=nanosec),
            frame_id=epoch,
        ),
        vector=SimpleNamespace(
            x=float(path_t),
            y=float(scale),
            z=float(base_t),
        ),
    )


class TrajectoryPathClockTest(unittest.TestCase):

    def test_interpolates_base_path_and_scale_at_message_stamp(self):
        visualizer, _ = _visualizer_stub()
        visualizer.path_progress_samples.extend((
            (10.0, 0.0, 0.0, 1.0, 'epoch-a'),
            (10.2, 0.2, 0.1, 0.5, 'epoch-a'),
        ))

        base_t, path_t, scale = visualizer._path_clock_for_sample(10.1)

        self.assertAlmostEqual(base_t, 0.1)
        self.assertAlmostEqual(path_t, 0.05)
        self.assertAlmostEqual(scale, 0.75)

    def test_extrapolates_through_the_inclusive_100_ms_boundary(self):
        visualizer, _ = _visualizer_stub()
        visualizer.path_progress_samples.append(
            (10.0, 2.0, 0.75, 0.4, 'epoch-a'))

        base_t, path_t, scale = visualizer._path_clock_for_sample(10.1)

        self.assertAlmostEqual(base_t, 2.1)
        self.assertAlmostEqual(path_t, 0.79)
        self.assertAlmostEqual(scale, 0.4)

    def test_rejects_extrapolation_beyond_100_ms(self):
        visualizer, logger = _visualizer_stub()
        visualizer.path_progress_samples.append(
            (10.0, 2.0, 0.75, 0.4, 'epoch-a'))

        result = visualizer._path_clock_for_sample(10.100001)

        self.assertIsNone(result)
        self.assertTrue(visualizer.path_clock_wait_reported)
        self.assertEqual(len(logger.warnings), 1)
        self.assertIn('stale', logger.warnings[0])

    def test_disabled_mode_uses_identity_elapsed_clock(self):
        visualizer, _ = _visualizer_stub(
            mode='disabled', required=False, start_time=20.0)

        clock = visualizer._path_clock_for_sample(22.5)

        self.assertEqual(clock, (2.5, 2.5, 1.0))

    def test_actual_fk_uses_base_time_for_translation_and_both_clocks_for_rotation(self):
        visualizer, _ = _visualizer_stub()
        visualizer.ik = _IkStub([1.0, 0.0, 0.0])
        calls = []

        def base_pose(base_t):
            calls.append(('pose', base_t))
            return np.array([base_t, 0.0, 0.0])

        def base_rotation(base_t, path_t):
            calls.append(('rotation', base_t, path_t))
            return np.array([
                [0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ])

        visualizer._moving_base_pose = base_pose
        visualizer._moving_base_rotation = base_rotation

        actual = visualizer._actual_position(
            np.zeros(7), base_t=4.0, path_t=1.25)

        np.testing.assert_allclose(actual, [4.0, 1.0, 0.0])
        self.assertEqual(calls, [
            ('pose', 4.0),
            ('rotation', 4.0, 1.25),
        ])

    def test_epoch_change_and_time_regression_discard_old_anchors(self):
        scenarios = (
            (
                _path_clock_message(
                    30.2, 0.2, 0.15, 0.5, epoch='epoch-b'),
                'epoch change',
            ),
            (
                _path_clock_message(
                    30.1, 0.1, 0.05, 0.5, epoch='epoch-a'),
                'time regression',
            ),
        )
        for replacement, label in scenarios:
            with self.subTest(label=label):
                visualizer, _ = _visualizer_stub(start_time=None)
                visualizer.path_progress_samples.extend((
                    (30.0, 0.0, 0.0, 1.0, 'epoch-a'),
                    (30.2, 0.2, 0.1, 0.5, 'epoch-a'),
                ))

                visualizer._path_progress_cb(replacement)

                self.assertEqual(len(visualizer.path_progress_samples), 1)
                retained = visualizer.path_progress_samples[0]
                self.assertAlmostEqual(
                    retained[0],
                    visualizer._header_stamp_sec(replacement),
                )
                self.assertEqual(retained[4], replacement.header.frame_id)

    def test_reset_publishes_delete_for_all_latched_run_markers(self):
        visualizer, _ = _visualizer_stub()
        visualizer.frame_id = 'world'
        visualizer.actual_points = deque([
            (np.zeros(3), 0), (np.ones(3), 0)])
        visualizer.expected_points = deque()
        visualizer.command_fk_points = deque([
            (np.zeros(3), 0), (np.ones(3), 0)])
        visualizer.control_samples = []
        visualizer.last_recorded_actual = np.ones(3)
        visualizer.last_recorded_control = np.ones(7)
        visualizer.last_recorded_command_fk = np.ones(3)
        visualizer.current_actual = np.ones(3)
        visualizer.current_expected = np.ones(3)
        visualizer.current_base_t = 0.4
        visualizer.max_rviz_path_points = 2500
        visualizer.actual_rviz_smoothing_window = 7
        visualizer.actual_marker_segment_count = 3
        visualizer.show_command_fk_marker = False
        visualizer.show_base_disturbance = True
        visualizer.base_disturbance_amplitude_m = 0.01
        visualizer.base_disturbance_seed = 0
        visualizer.base_disturbance_profile = 'gust'
        visualizer.base_motion_time_scale = 1.0
        visualizer.warmup_reported = True
        visualizer.stale_reported = True
        visualizer.plotter = None
        visualizer.expected_path_segments = [
            np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])]
        visualizer.expected_pub = _PublisherStub()
        visualizer.actual_pub = _PublisherStub()
        visualizer.command_fk_pub = _PublisherStub()
        visualizer.current_target_pub = _PublisherStub()
        visualizer.current_actual_pub = _PublisherStub()
        visualizer.disturbance_marker_pub = _PublisherStub()
        visualizer.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time()))

        visualizer._clear_recording_buffers()
        visualizer._publish_markers()

        self.assertIsNone(visualizer.current_actual)
        self.assertIsNone(visualizer.current_expected)
        self.assertIsNone(visualizer.current_base_t)
        actual_markers = visualizer.actual_pub.messages[-1].markers
        self.assertEqual(len(actual_markers), 3)
        self.assertTrue(all(
            marker.action == Marker.DELETE for marker in actual_markers))
        self.assertEqual(
            [marker.id for marker in actual_markers], [0, 1, 2])
        for publisher in (
                visualizer.command_fk_pub,
                visualizer.current_target_pub,
                visualizer.current_actual_pub):
            self.assertEqual(publisher.messages[-1].action, Marker.DELETE)
        disturbance_markers = visualizer.disturbance_marker_pub.messages[-1]
        self.assertEqual(
            [marker.id for marker in disturbance_markers.markers],
            [10, 11, 12, 13, 14])
        self.assertTrue(all(
            marker.action == Marker.DELETE
            for marker in disturbance_markers.markers))

    def test_disturbance_markers_use_physical_base_time_and_exact_endpoints(self):
        visualizer, _ = _visualizer_stub()
        visualizer.frame_id = 'world'
        visualizer.show_base_disturbance = True
        visualizer.base_disturbance_amplitude_m = 0.01
        visualizer.base_disturbance_seed = 0
        visualizer.base_disturbance_profile = 'gust'
        visualizer.base_motion_time_scale = 2.0
        visualizer.disturbance_display_height_m = 0.12
        # Physical disturbance time is base_t / scale = 0.4s, the pulse peak.
        # No path-time field participates in marker generation.
        visualizer.current_base_t = 0.8

        markers = {
            marker.id: marker
            for marker in visualizer._base_disturbance_markers(Time())
        }

        sample = moving_traj.base_disturbance_sample(
            0.4, 0.01, seed=0, profile='gust')
        display_offset = np.array([0.0, 0.0, 0.12])
        nominal = moving_traj.base_pose(0.4) + display_offset
        truth = nominal + sample.total_offset
        background = nominal + sample.background_offset
        total_arrow = markers[12]
        event_arrow = markers[13]
        np.testing.assert_allclose(
            [total_arrow.points[0].x, total_arrow.points[0].y,
             total_arrow.points[0].z],
            nominal)
        np.testing.assert_allclose(
            [total_arrow.points[1].x, total_arrow.points[1].y,
             total_arrow.points[1].z],
            truth)
        np.testing.assert_allclose(
            [event_arrow.points[0].x, event_arrow.points[0].y,
             event_arrow.points[0].z],
            background)
        np.testing.assert_allclose(
            [event_arrow.points[1].x, event_arrow.points[1].y,
             event_arrow.points[1].z],
            truth)
        self.assertEqual(event_arrow.action, Marker.ADD)
        self.assertLessEqual(
            event_arrow.scale.z,
            0.35 * np.linalg.norm(truth - background) + 1e-12)
        self.assertIn('GUST ACTIVE 100%', markers[14].text)
        self.assertIn('|Δp|=', markers[14].text)
        self.assertIn('|event|=10.0 mm', markers[14].text)

    def test_fault_event_arrow_only_exists_in_bias_window(self):
        visualizer, _ = _visualizer_stub()
        visualizer.frame_id = 'world'
        visualizer.show_base_disturbance = True
        visualizer.base_disturbance_amplitude_m = 0.01
        visualizer.base_disturbance_seed = 0
        visualizer.base_disturbance_profile = 'fault'
        visualizer.base_motion_time_scale = 1.0
        visualizer.disturbance_display_height_m = 0.12

        visualizer.current_base_t = 5.999
        inactive = {
            marker.id: marker
            for marker in visualizer._base_disturbance_markers(Time())
        }
        self.assertEqual(inactive[13].action, Marker.DELETE)
        self.assertIn('bias inactive', inactive[14].text)

        visualizer.current_base_t = 6.0
        active = {
            marker.id: marker
            for marker in visualizer._base_disturbance_markers(Time())
        }
        self.assertEqual(active[13].action, Marker.ADD)
        self.assertIn('BIAS ACTIVE', active[14].text)

    def test_rviz_path_downsampling_preserves_every_stroke_endpoint(self):
        records = deque()
        for stroke_id in range(3):
            for index in range(1000):
                TrajectoryVisualizerDemo._append_visible_point(
                    records,
                    np.array([float(index), float(stroke_id), 0.0]),
                    True,
                    stroke_id,
                )

        segments = TrajectoryVisualizerDemo._records_to_segments(
            records, max_points=300)

        self.assertEqual(len(segments), 3)
        self.assertLessEqual(sum(len(segment) for segment in segments), 300)
        for stroke_id, segment in enumerate(segments):
            np.testing.assert_allclose(
                segment[0], [0.0, float(stroke_id), 0.0])
            np.testing.assert_allclose(
                segment[-1], [999.0, float(stroke_id), 0.0])

    def test_red_rviz_line_is_smoothed_but_never_bridges_strokes(self):
        segments = [
            np.array([
                [float(index), 0.15 * (-1.0) ** index, 0.0]
                for index in range(15)
            ]),
            np.array([
                [100.0 + float(index), 1.0 + 0.15 * (-1.0) ** index, 0.0]
                for index in range(15)
            ]),
        ]
        original_segments = [segment.copy() for segment in segments]
        smoothed = TrajectoryVisualizerDemo._smooth_rviz_segments(
            segments, window=7)
        raw_roughness = np.abs(np.diff(segments[0][:, 1], n=2)).mean()
        smooth_roughness = np.abs(np.diff(smoothed[0][:, 1], n=2)).mean()
        self.assertLess(smooth_roughness, raw_roughness)
        np.testing.assert_array_equal(smoothed[0][0], segments[0][0])
        np.testing.assert_array_equal(smoothed[0][-1], segments[0][-1])
        for segment, original in zip(segments, original_segments):
            np.testing.assert_array_equal(segment, original)

        visualizer, _ = _visualizer_stub()
        visualizer.frame_id = 'world'
        visualizer.actual_marker_segment_count = 0
        markers = visualizer._line_strip_markers(
            'trajectory_actual', smoothed, Time(),
            color=(1.0, 0.1, 0.1, 1.0), width=0.010)

        self.assertEqual(len(markers), 2)
        self.assertTrue(all(
            marker.type == Marker.LINE_STRIP for marker in markers))
        self.assertAlmostEqual(markers[0].points[-1].x, 14.0)
        self.assertAlmostEqual(markers[1].points[0].x, 100.0)


if __name__ == '__main__':
    unittest.main()
