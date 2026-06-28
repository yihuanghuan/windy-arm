#!/usr/bin/env bash
set -euo pipefail

CONFIG_PATH="${CAMERA_CONFIG_PATH:-/opt/windshape/manipulator/install/manipulator/share/manipulator/camera.yaml}"

python3 - "$CONFIG_PATH" <<'PY'
import os
import re
import sys

path = sys.argv[1]

if not os.path.exists(path):
    raise SystemExit(f"camera config not found: {path}")

with open(path, "r", encoding="utf-8") as f:
    content = f.read()

match = re.search(r"^\s*ip:\s*['\"]?([^'\"\n#]+)['\"]?", content, re.MULTILINE)
if not match:
    raise SystemExit(f"ip field not found in: {path}")

print(match.group(1).strip())
PY
