#!/usr/bin/env bash
set -euo pipefail

CONFIG_PATH="${CAMERA_CONFIG_PATH:-/opt/windshape/manipulator/install/manipulator/share/manipulator/camera.yaml}"

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <camera_ip>"
  echo "Target: ${CONFIG_PATH}"
  exit 1
fi

CAMERA_IP="$1"

python3 - "$CONFIG_PATH" "$CAMERA_IP" <<'PY'
import json
import os
import re
import sys
import tempfile

path = sys.argv[1]
ip = sys.argv[2]

if not os.path.exists(path):
    raise SystemExit(f"camera config not found: {path}")

with open(path, "r", encoding="utf-8") as f:
    content = f.read()

new_ip = json.dumps(ip)
pattern = re.compile(r"^(\s*ip:\s*).*$", re.MULTILINE)
content, count = pattern.subn(lambda match: match.group(1) + new_ip, content, count=1)

if count != 1:
    raise SystemExit(f"ip field not found in: {path}")

dirname = os.path.dirname(path)
fd, tmp_path = tempfile.mkstemp(prefix=".camera.yaml.", dir=dirname, text=True)
try:
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(content)
    os.chmod(tmp_path, 0o644)
    os.replace(tmp_path, path)
finally:
    if os.path.exists(tmp_path):
        os.remove(tmp_path)
PY

echo "Updated camera ip to ${CAMERA_IP} in ${CONFIG_PATH}"
