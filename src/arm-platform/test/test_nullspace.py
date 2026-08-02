import sys
from pathlib import Path

import numpy as np

sys.path.append(str(Path(__file__).resolve().parents[1]))

from task_space_smc.redundancy_resolution import resolve_acceleration


def test_damped_nullspace_reports_leakage():
    rng = np.random.default_rng(2)
    J = rng.normal(size=(6, 7))
    M = np.eye(7)
    result = resolve_acceleration(
        J,
        M,
        np.ones(6) * 0.1,
        np.zeros(7),
        np.zeros(7),
        np.zeros(7),
        np.ones(7) * 0.2,
        np.ones(7) * 0.05,
        characteristic_length=0.2,
        sigma_warn=10.0,
        sigma_critical=0.01,
        damping_min=0.0,
        damping_max=0.05,
    )
    assert result.qdd_cmd.shape == (7,)
    assert result.N.shape == (7, 7)
    assert result.damping > 0.0
    assert result.nullspace_leakage_norm >= 0.0
