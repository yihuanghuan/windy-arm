#!/usr/bin/env python3
"""Unit checks for Phase-0 bounded DART command generation."""

import numpy as np

from dart_noise import bounded_dart_command


def _run(seed=7, std=0.002, nominal=None, current=None):
    current = np.zeros(7) if current is None else np.asarray(current, dtype=float)
    nominal = (
        np.full(7, 0.01) if nominal is None else np.asarray(nominal, dtype=float)
    )
    return bounded_dart_command(
        nominal_q=nominal,
        current_q=current,
        q_lower=np.full(7, -1.0),
        q_upper=np.full(7, 1.0),
        max_joint_step=0.02,
        noise_std=std,
        noise_clip_sigma=2.5,
        rng=np.random.default_rng(seed),
    )


def test_zero_noise_is_exactly_nominal():
    result = _run(std=0.0)
    assert np.array_equal(result.executed_q, result.nominal_q)
    assert np.array_equal(result.sampled_noise, np.zeros(7))
    assert np.array_equal(result.applied_noise, np.zeros(7))
    assert not result.was_clipped


def test_seed_is_reproducible():
    a = _run(seed=123)
    b = _run(seed=123)
    assert np.array_equal(a.sampled_noise, b.sampled_noise)
    assert np.array_equal(a.executed_q, b.executed_q)


def test_noise_respects_step_and_joint_limits():
    current = np.array([0.99, -0.99, 0.0, 0.0, 0.0, 0.0, 0.0])
    nominal = np.array([1.0, -1.0, 0.019, -0.019, 0.0, 0.0, 0.0])
    result = _run(seed=2, std=0.02, nominal=nominal, current=current)
    step = result.executed_q - current
    assert np.max(np.abs(step)) <= 0.02 + 1e-12
    assert np.all(result.executed_q <= 1.0 + 1e-12)
    assert np.all(result.executed_q >= -1.0 - 1e-12)
    assert result.was_clipped


def test_applied_noise_reconstructs_executed_command():
    result = _run(seed=99)
    assert np.allclose(
        result.nominal_q + result.applied_noise,
        result.executed_q,
        atol=1e-12,
        rtol=0.0,
    )


if __name__ == "__main__":
    test_zero_noise_is_exactly_nominal()
    test_seed_is_reproducible()
    test_noise_respects_step_and_joint_limits()
    test_applied_noise_reconstructs_executed_command()
    print("test_phase0_dart: PASS")
