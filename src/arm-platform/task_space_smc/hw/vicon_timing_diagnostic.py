#!/usr/bin/env python3
"""Read-only three-layer Vicon timing diagnostic.

Layer 1 is the first ROS message emitted by vrpn_listener.  Its header stamp is
the VRPN/Vicon source timestamp; the diagnostic callback records local arrival.
Layer 2 uses an exact read-only probe immediately before each bridge publish,
and also records an independent subscriber arrival for cross-checking.
Layer 3 consumes the optional timing probe emitted by the runner callbacks.
No command or control topic is published by this node.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from dummy_interface.msg import FloatingBaseState
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class TimingDiagnostic(Node):
    def __init__(self, output: Path):
        super().__init__("vicon_timing_diagnostic")
        self.output = output
        self.samples = {name: [] for name in (
            "raw_arm_base", "raw_arm_ee", "bridge_arm_base",
            "bridge_arm_ee", "bridge_pub_arm_base", "bridge_pub_arm_ee",
            "runner_arm_base", "runner_arm_ee")}
        self.create_subscription(PoseStamped, "/vrpn/arm_base/pose",
                                 lambda m: self._pose("raw_arm_base", m), 200)
        self.create_subscription(PoseStamped, "/vrpn/arm_ee/pose",
                                 lambda m: self._pose("raw_arm_ee", m), 200)
        self.create_subscription(FloatingBaseState, "/floating_base/vicon",
                                 lambda m: self._state("bridge_arm_base", m), 200)
        self.create_subscription(FloatingBaseState, "/vicon/arm_ee",
                                 lambda m: self._state("bridge_arm_ee", m), 200)
        self.create_subscription(Float64MultiArray, "/floating_base/vicon_timing",
                                 self._bridge_publish, 200)
        self.create_subscription(Float64MultiArray,
                                 "/task_space_smc/timing_callbacks",
                                 self._runner, 200)

    @staticmethod
    def _stamp(msg):
        return float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)

    def _append(self, name, source):
        arrival = self.get_clock().now().nanoseconds * 1e-9
        self.samples[name].append({"source": float(source),
                                   "arrival": float(arrival)})

    def _pose(self, name, msg):
        self._append(name, self._stamp(msg))

    def _state(self, name, msg):
        self._append(name, self._stamp(msg))

    def _runner(self, msg):
        if len(msg.data) < 3:
            return
        code = int(round(msg.data[0]))
        name = {1: "runner_arm_base", 2: "runner_arm_ee"}.get(code)
        if name is None:
            return
        self.samples[name].append({"source": float(msg.data[1]),
                                   "arrival": float(msg.data[2])})

    def _bridge_publish(self, msg):
        if len(msg.data) < 4:
            return
        code = int(round(msg.data[0]))
        name = {1: "bridge_pub_arm_base", 2: "bridge_pub_arm_ee"}.get(code)
        if name is None:
            return
        # source is VRPN source time; arrival is the bridge's exact publish time.
        self.samples[name].append({"source": float(msg.data[1]),
                                   "header": float(msg.data[2]),
                                   "arrival": float(msg.data[3])})

    def finish(self):
        report = {"streams": {}, "duration_sec": None}
        all_times = []
        for name, rows in self.samples.items():
            if len(rows) < 2:
                report["streams"][name] = {"count": len(rows), "error": "<2 samples"}
                continue
            rows = sorted(rows, key=lambda x: x["arrival"])
            source = np.asarray([x["source"] for x in rows])
            arrival = np.asarray([x["arrival"] for x in rows])
            dt_source = np.diff(source)
            dt_arrival = np.diff(arrival)
            valid = np.isfinite(dt_source) & (dt_source > 0.0)
            dt = dt_source[valid]
            def gap_rows(dt_values, valid_mask, key):
                rows_out = []
                for i in np.flatnonzero(valid_mask):
                    value = float(dt_values[i])
                    if value > 0.05:
                        rows_out.append({
                            "time": float(arrival[i + 1]),
                            "previous_time": float(arrival[i]),
                            "dt_ms": 1000.0 * value,
                            "source_time": float(source[i + 1]),
                            "previous_source_time": float(source[i]),
                            "kind": key,
                        })
                return rows_out

            valid_arrival = np.isfinite(dt_arrival) & (dt_arrival > 0.0)
            source_gaps = []
            for i in np.flatnonzero(valid):
                value = float(dt_source[i])
                if value > 0.05:
                    source_gaps.append({
                        "source_time": float(source[i + 1]),
                        "previous_source_time": float(source[i]),
                        "dt_ms": 1000.0 * value,
                        "arrival_time": float(arrival[i + 1]),
                    })
            arrival_gaps = gap_rows(dt_arrival, valid_arrival, "arrival_dt")
            threshold = lambda rows, limit: [g for g in rows if g["dt_ms"] > limit]
            report["streams"][name] = {
                "count": len(rows),
                "source_dt_ms": self._stats(dt * 1000.0),
                "arrival_dt_ms": self._stats(dt_arrival * 1000.0),
                "source_gaps_over_50ms": source_gaps,
                "source_gaps_over_100ms": [g for g in source_gaps if g["dt_ms"] > 100.0],
                "source_gaps_over_150ms": [g for g in source_gaps if g["dt_ms"] > 150.0],
                "arrival_gaps_over_50ms": threshold(arrival_gaps, 50.0),
                "arrival_gaps_over_100ms": threshold(arrival_gaps, 100.0),
                "arrival_gaps_over_150ms": threshold(arrival_gaps, 150.0),
            }
            all_times.extend(float(x["arrival"]) for x in rows)
        if all_times:
            report["duration_sec"] = max(all_times) - min(all_times)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2), flush=True)

    @staticmethod
    def _stats(values):
        values = np.asarray(values, dtype=float)
        return {
            "median_ms": float(np.median(values)),
            "p95_ms": float(np.percentile(values, 95)),
            "p99_ms": float(np.percentile(values, 99)),
            "max_ms": float(np.max(values)),
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-sec", type=float, default=60.0)
    parser.add_argument("--output", type=Path,
                        default=Path("/tmp/vicon_timing_diagnostic.json"))
    args = parser.parse_args()
    rclpy.init()
    node = TimingDiagnostic(args.output)
    end = time.monotonic() + args.duration_sec
    try:
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.finish()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
