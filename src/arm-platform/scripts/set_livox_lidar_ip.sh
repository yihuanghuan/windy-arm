#!/usr/bin/env bash
set -euo pipefail

CONFIG_PATH="${LIVOX_CONFIG_PATH:-/opt/windshape/manipulator/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360s_config.json}"

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <lidar_ip>"
  echo "Target: ${CONFIG_PATH}"
  exit 1
fi

LIDAR_IP="$1"

python3 - "$CONFIG_PATH" "$LIDAR_IP" <<'PY'
import json
import os
import sys
import tempfile

path = sys.argv[1]
ip = sys.argv[2]

if not os.path.exists(path):
    raise SystemExit(f"livox config not found: {path}")

with open(path, "r", encoding="utf-8") as f:
    config = json.load(f)

lidar_configs = config.get("lidar_configs")
if not isinstance(lidar_configs, list) or not lidar_configs:
    raise SystemExit(f"lidar_configs not found or empty in: {path}")

if not isinstance(lidar_configs[0], dict):
    raise SystemExit(f"lidar_configs[0] is invalid in: {path}")

lidar_configs[0]["ip"] = ip

dirname = os.path.dirname(path)
fd, tmp_path = tempfile.mkstemp(prefix=".MID360s_config.json.", dir=dirname, text=True)
try:
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        json.dump(config, f, indent=2)
        f.write("\n")
    os.chmod(tmp_path, 0o644)
    os.replace(tmp_path, path)
finally:
    if os.path.exists(tmp_path):
        os.remove(tmp_path)
PY

echo "Updated lidar ip to ${LIDAR_IP} in ${CONFIG_PATH}"
