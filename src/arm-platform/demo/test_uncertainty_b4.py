#!/usr/bin/env python3
"""B4 两工况残差估计与风险决策的纯 numpy 回归测试。"""

from pathlib import Path
import sys
import time
import unittest

import numpy as np

DEMO_DIR = Path(__file__).resolve().parent
if str(DEMO_DIR) not in sys.path:
    sys.path.insert(0, str(DEMO_DIR))

from base_trust_estimator import (  # noqa: E402
    BaseTrustEstimator,
    TwoRegimeResidualEstimator,
    UncertaintyAwareFeedforwardDecision,
)


class TwoRegimeResidualEstimatorTest(unittest.TestCase):
    def test_persistent_bias_is_identified_and_predicted(self):
        estimator = TwoRegimeResidualEstimator(
            measurement_std=1.0e-3,
            persistent_process_std=2.0e-3,
        )
        for _ in range(100):
            estimator.step(np.zeros(3))
        bias = np.array([0.012, -0.004, 0.003])
        diagnostic = None
        for _ in range(30):
            diagnostic = estimator.step(bias)
        self.assertIsNotNone(diagnostic)
        self.assertGreater(diagnostic["persistent_probability"], 0.8)
        np.testing.assert_allclose(
            diagnostic["posterior_mean"], bias, atol=8.0e-4)
        mean, covariance, probability = estimator.predict_horizon([1, 10])
        np.testing.assert_allclose(mean[0], bias, atol=1.2e-3)
        self.assertGreater(probability[0, 1], 0.7)
        self.assertGreaterEqual(np.linalg.eigvalsh(covariance).min(), 0.0)

    def test_isolated_impulse_returns_to_transient_mode(self):
        estimator = TwoRegimeResidualEstimator(
            measurement_std=1.5e-3,
            stable_tau_sec=0.08,
        )
        for _ in range(80):
            estimator.step(np.zeros(3))
        estimator.step(np.array([0.02, 0.0, 0.0]))
        diagnostic = None
        for _ in range(80):
            diagnostic = estimator.step(np.zeros(3))
        self.assertLess(diagnostic["persistent_probability"], 0.25)
        self.assertLess(
            np.linalg.norm(diagnostic["posterior_mean"]), 8.0e-4)

    def test_mixture_covariance_contains_mode_disagreement(self):
        estimator = TwoRegimeResidualEstimator(
            measurement_std=2.0e-3,
            initial_persistent_probability=0.5,
        )
        for _ in range(8):
            estimator.step(np.array([0.006, 0.0, 0.0]))
        mean = estimator.mode_probabilities @ estimator.states
        within = np.zeros((3, 3))
        disagreement = np.zeros((3, 3))
        for weight, state, covariance in zip(
                estimator.mode_probabilities,
                estimator.states,
                estimator.covariances):
            difference = state - mean
            within += weight * covariance
            disagreement += weight * np.outer(difference, difference)
        np.testing.assert_allclose(
            estimator.covariance, within + disagreement, atol=1.0e-12)
        self.assertGreaterEqual(np.trace(disagreement), 0.0)


class UncertaintyAwareDecisionTest(unittest.TestCase):
    def test_low_parameter_applicability_does_not_remove_any_mean(self):
        decision = UncertaintyAwareFeedforwardDecision()
        harmonic = np.full((5, 3), 0.02)
        fast = np.tile(np.array([0.01, -0.003, 0.002]), (5, 1))
        covariance = np.repeat((1.0e-7 * np.eye(3))[None, :, :], 5, axis=0)
        diagnostic = decision.update(
            harmonic, fast, covariance, 5.0e-4,
            parameter_applicability=0.0, dt=0.02)
        np.testing.assert_allclose(
            diagnostic["candidate_prediction"], harmonic + fast,
            atol=1.0e-12)
        self.assertEqual(diagnostic["parameter_applicability_gain"], 0.0)
        self.assertGreater(diagnostic["parameter_risk_multiplier"], 1.0)

    def test_risk_budget_reduces_gain_speed_and_position_weight(self):
        decision = UncertaintyAwareFeedforwardDecision(
            rise_tau_sec=0.05, fall_tau_sec=0.05,
            risk_budget_m=0.01)
        harmonic = np.zeros((5, 3))
        fast = np.tile(np.array([0.012, 0.0, 0.0]), (5, 1))
        low_covariance = np.repeat(
            (1.0e-8 * np.eye(3))[None, :, :], 5, axis=0)
        for _ in range(30):
            low = decision.update(
                harmonic, fast, low_covariance, 2.0e-4, 1.0, 0.02)
        self.assertGreater(low["feedforward_gain"], 0.8)
        self.assertEqual(low["speed_scale"], 1.0)
        high_covariance = np.repeat(
            (2.0e-4 * np.eye(3))[None, :, :], 5, axis=0)
        for _ in range(30):
            high = decision.update(
                harmonic, fast, high_covariance, 0.02, 1.0, 0.02)
        self.assertLess(high["feedforward_gain"], low["feedforward_gain"])
        self.assertLess(high["speed_scale"], 1.0)
        self.assertLess(high["position_weight_scale"], 1.0)


class BaseTrustEstimatorB4Test(unittest.TestCase):
    def test_stats_interface_and_fast_loop_budget(self):
        estimator = BaseTrustEstimator(
            n_freq=3,
            n_peaks=3,
            buffer_sec=20.0,
            min_ident_sec=10.0,
            regime_enabled=True,
            cal_horizon_steps=10,
        )
        samples_ms = []
        for index in range(250):
            t = 0.02 * index
            measurement = (
                np.array([0.008, -0.003, 0.002])
                if index >= 100 else np.zeros(3))
            started = time.perf_counter()
            diagnostic = estimator.step(t, measurement)
            stats = estimator.predict_horizon_stats(
                t + 0.02 * np.arange(1, 11))
            samples_ms.append(1.0e3 * (time.perf_counter() - started))
        self.assertEqual(stats["mean"].shape, (10, 3))
        self.assertEqual(stats["covariance"].shape, (10, 3, 3))
        self.assertEqual(stats["mode_probabilities"].shape, (10, 2))
        self.assertTrue(np.all(np.isfinite(stats["mean"])))
        self.assertTrue(np.all(np.isfinite(stats["covariance"])))
        self.assertGreater(diagnostic["persistent_probability"], 0.5)
        # 该测试不含慢时标频谱任务；常规感知链必须远低于 20 ms。
        self.assertLess(float(np.percentile(samples_ms, 99)), 5.0)


if __name__ == "__main__":
    unittest.main()
