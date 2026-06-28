#!/usr/bin/env bash
set -euo pipefail

CONFIG_PATH="${LIVOX_CONFIG_PATH:-/opt/windshape/manipulator/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360s_config.json}"

python3 - "$CONFIG_PATH" <<'PY'
import json
import os
import sys

path = sys.argv[1]

if not os.path.exists(path):
    raise SystemExit(f"livox config not found: {path}")

with open(path, "r", encoding="utf-8") as f:
    config = json.load(f)

lidar_configs = config.get("lidar_configs")
if not isinstance(lidar_configs, list) or not lidar_configs:
    raise SystemExit(f"lidar_configs not found or empty in: {path}")

ip = lidar_configs[0].get("ip")
if ip is None:
    raise SystemExit(f"lidar_configs[0].ip not found in: {path}")

print(ip)
PY
