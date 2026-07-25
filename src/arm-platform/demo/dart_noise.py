"""Pure helpers for safe, auditable DART command perturbations.

The expert label remains the nominal MPC increment.  The perturbed command is
only the action executed by the plant so that the next observation visits a
nearby off-expert state.
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class DartCommand:
    nominal_q: np.ndarray
    executed_q: np.ndarray
    sampled_noise: np.ndarray
    applied_noise: np.ndarray
    was_clipped: bool


def bounded_dart_command(
    nominal_q,
    current_q,
    q_lower,
    q_upper,
    max_joint_step,
    noise_std,
    noise_clip_sigma,
    rng,
):
    """Sample Gaussian DART noise and project the final position command.

    The executed command is constrained both by the robot joint limits and by
    the same per-tick joint-step contract used by the MPC.
    """
    nominal = np.asarray(nominal_q, dtype=float).reshape(-1)
    current = np.asarray(current_q, dtype=float).reshape(nominal.shape)
    lower = np.asarray(q_lower, dtype=float).reshape(nominal.shape)
    upper = np.asarray(q_upper, dtype=float).reshape(nominal.shape)
    step = float(max_joint_step)
    std = float(noise_std)
    clip_sigma = float(noise_clip_sigma)

    if not np.all(np.isfinite(nominal)) or not np.all(np.isfinite(current)):
        raise ValueError("nominal_q/current_q must be finite")
    if step <= 0.0:
        raise ValueError("max_joint_step must be > 0")
    if std < 0.0:
        raise ValueError("noise_std must be >= 0")
    if clip_sigma <= 0.0:
        raise ValueError("noise_clip_sigma must be > 0")

    sampled = (
        rng.normal(0.0, std, nominal.shape)
        if std > 0.0
        else np.zeros_like(nominal)
    )
    sampled = np.clip(sampled, -clip_sigma * std, clip_sigma * std)

    safe_lower = np.maximum(lower, current - step)
    safe_upper = np.minimum(upper, current + step)
    executed = np.clip(nominal + sampled, safe_lower, safe_upper)
    applied = executed - nominal
    was_clipped = not np.allclose(sampled, applied, atol=1e-12, rtol=0.0)
    return DartCommand(
        nominal_q=nominal.copy(),
        executed_q=executed,
        sampled_noise=sampled,
        applied_noise=applied,
        was_clipped=was_clipped,
    )
