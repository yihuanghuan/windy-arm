import sys
from pathlib import Path

import numpy as np

sys.path.append(str(Path(__file__).resolve().parents[1]))

from task_space_smc.offline_simulation import default_config_path, run_offline


def test_offline_simulation_generates_finite_metrics(tmp_path):
    metrics = run_offline(default_config_path(), str(tmp_path), profile="translation", reference_mode="hold", duration=0.04, dt=0.01)
    assert metrics["steps"] == 4
    for key in ["final_position_error_m", "max_position_error_m", "final_orientation_error_rad", "min_sigma", "max_damping"]:
        assert np.isfinite(metrics[key])
    assert Path(metrics["csv"]).exists()
    assert Path(metrics["metrics_path"]).exists()
