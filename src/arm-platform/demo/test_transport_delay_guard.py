#!/usr/bin/env python3
"""Unit tests for the production /joint_states transport-delay hard guard.

The tests construct ``MoveBaseCircleMpcIkDemo`` with ``object.__new__`` so
they exercise the production guard methods without starting ROS, ACADOS, or
the simulated plant.
"""

import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from move_base_circle_mpc_ik_demo import (  # noqa: E402
    MoveBaseCircleMpcIkDemo,
)


CONTROL_DT_SEC = 0.02


def _guard_stub():
    demo = object.__new__(MoveBaseCircleMpcIkDemo)
    demo.transport_delay_guard_enabled = True
    demo.transport_delay_threshold_ms = 30.0
    demo.transport_delay_fault_duration_sec = 0.25
    demo.transport_delay_min_fresh_samples = 3
    demo.feedback_stale_timeout_ms = 100.0
    demo.dt = CONTROL_DT_SEC
    demo._transport_gap_bad_duration_sec = 0.0
    demo._transport_gap_distinct_bad_samples = 0
    demo._transport_gap_last_bad_stamp = float('nan')
    demo._transport_gap_bad_active = False
    return demo


class TransportDelayGuardTest(unittest.TestCase):
    def test_healthy_195737_envelope_never_triggers(self):
        """The observed healthy gap/age envelope must not accumulate a fault."""
        demo = _guard_stub()
        gaps_ms = (0.45, 2.73, 5.57, 9.56)
        feedback_ages_ms = (0.5, 7.0, 20.0, 31.3, 35.4)

        for tick in range(5000):
            tripped = demo._update_transport_delay_guard(
                transport_gap_ms=gaps_ms[tick % len(gaps_ms)],
                elapsed_sec=CONTROL_DT_SEC,
                feedback_stamp_sec=1000.0 + 0.01 * tick,
                feedback_age_ms=feedback_ages_ms[
                    tick % len(feedback_ages_ms)],
            )
            self.assertFalse(tripped, f'healthy tick {tick} tripped')

        self.assertFalse(demo._transport_gap_bad_active)
        self.assertEqual(demo._transport_gap_bad_duration_sec, 0.0)
        self.assertEqual(demo._transport_gap_distinct_bad_samples, 0)
        self.assertTrue(math.isnan(demo._transport_gap_last_bad_stamp))

    def test_96ms_gap_at_10hz_triggers_after_debounce(self):
        """Reproduce the 20:02 transport pattern: 50 Hz ticks, 10 Hz samples."""
        demo = _guard_stub()
        trigger_elapsed = None

        for tick in range(30):
            # One distinct source sample per five 50 Hz controller ticks.
            feedback_stamp = 2000.0 + 0.1 * (tick // 5)
            tripped = demo._update_transport_delay_guard(
                transport_gap_ms=96.0,
                elapsed_sec=CONTROL_DT_SEC,
                feedback_stamp_sec=feedback_stamp,
                # Keep local age below its independent immediate-fault gate.
                feedback_age_ms=60.0,
            )
            if tripped:
                trigger_elapsed = tick * CONTROL_DT_SEC
                break

        self.assertIsNotNone(trigger_elapsed)
        self.assertGreaterEqual(trigger_elapsed, 0.25)
        self.assertLessEqual(trigger_elapsed, 0.32)
        self.assertGreaterEqual(
            demo._transport_gap_bad_duration_sec, 0.25)
        self.assertGreaterEqual(
            demo._transport_gap_distinct_bad_samples, 3)

    def test_good_sample_resets_a_0p2s_bad_burst(self):
        demo = _guard_stub()

        # Eleven observations span exactly 0.20 s from the first bad sample.
        for tick in range(11):
            self.assertFalse(demo._update_transport_delay_guard(
                transport_gap_ms=96.0,
                elapsed_sec=CONTROL_DT_SEC,
                feedback_stamp_sec=3000.0 + 0.1 * (tick // 5),
                feedback_age_ms=20.0,
            ))
        self.assertAlmostEqual(
            demo._transport_gap_bad_duration_sec, 0.20, places=12)

        self.assertFalse(demo._update_transport_delay_guard(
            transport_gap_ms=2.0,
            elapsed_sec=CONTROL_DT_SEC,
            feedback_stamp_sec=3000.3,
            feedback_age_ms=5.0,
        ))
        self.assertFalse(demo._transport_gap_bad_active)
        self.assertEqual(demo._transport_gap_bad_duration_sec, 0.0)
        self.assertEqual(demo._transport_gap_distinct_bad_samples, 0)
        self.assertTrue(math.isnan(demo._transport_gap_last_bad_stamp))

        # A later fault episode must start from zero rather than inherit 0.20 s.
        self.assertFalse(demo._update_transport_delay_guard(
            transport_gap_ms=96.0,
            elapsed_sec=CONTROL_DT_SEC,
            feedback_stamp_sec=3000.4,
            feedback_age_ms=20.0,
        ))
        self.assertEqual(demo._transport_gap_bad_duration_sec, 0.0)
        self.assertEqual(demo._transport_gap_distinct_bad_samples, 1)

    def test_local_feedback_age_over_100ms_triggers_immediately(self):
        demo = _guard_stub()

        self.assertFalse(demo._update_transport_delay_guard(
            transport_gap_ms=2.0,
            elapsed_sec=CONTROL_DT_SEC,
            feedback_stamp_sec=4000.0,
            feedback_age_ms=100.0,
        ))
        self.assertTrue(demo._update_transport_delay_guard(
            transport_gap_ms=2.0,
            elapsed_sec=CONTROL_DT_SEC,
            feedback_stamp_sec=4000.0,
            feedback_age_ms=100.001,
        ))

    def test_enter_fault_holds_q_last_not_stale_feedback(self):
        demo = _guard_stub()
        safe_q_last = np.array(
            [0.35, -0.01, -0.25, 1.14, -1.17, 0.08, -3.04])
        stale_feedback = np.array(
            [0.42, -0.02, -0.39, 1.09, -1.14, 0.15, -3.09])
        demo.q_last = safe_q_last.copy()
        # The production fault method must not read or adopt this stale state.
        demo.q_measured = stale_feedback.copy()
        demo._latest_feedback_sample = (
            stale_feedback.copy(), np.zeros(7), 5000.0, 5000.096)
        demo._cmd_history = ()
        demo.transport_fault = False
        demo.transport_fault_gap_ms = float('nan')
        demo.phase = 'track'
        demo.online_time_scaling_enabled = False
        demo.time_scaler = SimpleNamespace(path_time=4.2)
        demo.warm_start = object()
        demo.second_order_warm_start = object()
        demo._so_last_cmd_dq = np.ones(7)
        demo._so_last_cmd_ddq = np.ones(7)
        demo.record_loop_csv = False
        demo._params_snapshot_path = None
        demo._transport_gap_bad_duration_sec = 0.26
        demo._transport_gap_distinct_bad_samples = 3

        published = []
        invalidations = []

        def publish(q, dq):
            q_copy = np.asarray(q, dtype=float).copy()
            dq_copy = np.asarray(dq, dtype=float).copy()
            published.append((q_copy, dq_copy))
            return q_copy, dq_copy, 5000.2, {}

        class Logger:
            def error(self, *args, **kwargs):
                pass

        demo._publish_joint_positions = publish
        demo._invalidate_params_snapshot = (
            lambda reason, details:
            invalidations.append((reason, dict(details))))
        demo.get_logger = lambda: Logger()

        result = demo._enter_transport_fault(
            transport_gap_ms=96.0,
            feedback_age_ms=20.0,
            feedback_stamp_sec=5000.0,
            path_time=4.2,
        )

        self.assertTrue(demo.transport_fault)
        self.assertEqual(demo.phase, 'transport_fault')
        np.testing.assert_allclose(demo.q_last, safe_q_last)
        self.assertFalse(np.allclose(demo.q_last, stale_feedback))
        self.assertEqual(len(published), 1)
        np.testing.assert_allclose(published[0][0], safe_q_last)
        np.testing.assert_array_equal(published[0][1], np.zeros(7))
        self.assertIsNotNone(result)
        self.assertEqual(
            invalidations[0][0], 'joint_states_transport_delay')
        self.assertIsNone(demo.warm_start)
        self.assertIsNone(demo.second_order_warm_start)
        self.assertIsNone(demo._so_last_cmd_dq)
        self.assertIsNone(demo._so_last_cmd_ddq)
        self.assertAlmostEqual(demo._fault_path_time, 4.2)

    def test_terminal_csv_row_does_not_masquerade_as_guard_fault(self):
        demo = _guard_stub()
        demo.record_loop_csv = True
        demo._fault_path_time = 4.2
        demo.traj_origin_wall = 100.0
        demo.guard_mode = 'state_safe_filter'
        demo.guard_consecutive_blocks = 0
        demo.guard_fault = False
        demo._loop_file = None
        captured = []
        demo._append_loop_row = (
            lambda timing, dq_prev, **kwargs:
            captured.append((dict(timing), dict(kwargs))))
        q = np.linspace(-0.3, 0.3, 7)
        published = (q.copy(), np.zeros(7), 104.24, {})

        demo._append_transport_fault_row(
            solve_wall=104.20,
            timer_interval_ms=20.0,
            q_feedback=q.copy(),
            dq_feedback=np.zeros(7),
            js_header_stamp_sec=104.10,
            js_recv_wall_sec=104.196,
            feedback_age_ms=4.0,
            transport_gap_ms=96.0,
            published=published,
        )

        self.assertEqual(len(captured), 1)
        timing, row = captured[0]
        self.assertTrue(timing['transport_fault'])
        self.assertFalse(row['guard_block'])
        self.assertFalse(row['guard_diag']['fault'])


if __name__ == '__main__':
    unittest.main()
