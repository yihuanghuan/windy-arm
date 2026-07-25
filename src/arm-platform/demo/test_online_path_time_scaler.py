#!/usr/bin/env python3

import numpy as np

from online_path_time_scaler import OnlinePathTimeScaler, TimeScalingConfig


def main():
    scaler = OnlinePathTimeScaler(TimeScalingConfig(
        velocity_reserve=0.1, scale_accel_per_sec=1.0,
        scale_decel_per_sec=10.0))
    # 无压力时按真实时间推进。
    for _ in range(10):
        scaler.update(0.02, 0.0, 0.0)
    assert abs(scaler.path_time - 0.2) < 1e-12
    # 关节预算满时快速减速，路径推进量小于真实时间。
    before = scaler.path_time
    for _ in range(5):
        scaler.update(0.02, 1.0, 0.0)
    assert scaler.scale < 0.1
    assert scaler.path_time - before < 0.1
    # 大误差同样允许暂停；预瞄不改变内部状态。
    scaler.update(0.02, 0.0, 0.05)
    p0 = scaler.path_time
    preview = scaler.preview([0.02, 0.04])
    assert np.all(preview >= p0)
    assert scaler.path_time == p0
    # 压力解除后平滑恢复，不瞬跳。
    old_scale = scaler.scale
    scaler.update(0.02, 0.0, 0.0)
    assert old_scale < scaler.scale <= old_scale + 0.02 + 1e-12
    print("ALL PASS")


if __name__ == "__main__":
    main()
