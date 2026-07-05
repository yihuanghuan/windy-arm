#!/usr/bin/env bash
set -euo pipefail

# 从源码安装 pygrampc + GRAMPC 子模块。默认安装到 $HOME/code/pygrampc,也可提前设置:
#   export PYGRAMPC_ROOT=/path/to/pygrampc

PYGRAMPC_ROOT="${PYGRAMPC_ROOT:-${HOME}/code/pygrampc}"
PYGRAMPC_REPO="${PYGRAMPC_REPO:-https://github.com/grampc/pygrampc.git}"

echo "[GRAMPC] install dir: ${PYGRAMPC_ROOT}"
echo "[GRAMPC] repo: ${PYGRAMPC_REPO}"

command -v git >/dev/null || { echo "[GRAMPC] 缺少 git,请先安装: sudo apt install git" >&2; exit 1; }
command -v python3 >/dev/null || { echo "[GRAMPC] 缺少 python3" >&2; exit 1; }
command -v cmake >/dev/null || { echo "[GRAMPC] 缺少 cmake,请先安装: sudo apt install cmake" >&2; exit 1; }
command -v make >/dev/null || { echo "[GRAMPC] 缺少 make,请先安装: sudo apt install build-essential" >&2; exit 1; }

if [ -d "${PYGRAMPC_ROOT}/.git" ]; then
  echo "[GRAMPC] 已存在源码目录,更新源码和 submodule..."
  git -C "${PYGRAMPC_ROOT}" pull --ff-only
  git -C "${PYGRAMPC_ROOT}" submodule update --init --recursive
elif [ -e "${PYGRAMPC_ROOT}" ]; then
  echo "[GRAMPC] 目标路径已存在但不是 git 仓库: ${PYGRAMPC_ROOT}" >&2
  echo "[GRAMPC] 请删除/移动该目录,或设置 PYGRAMPC_ROOT 到新的安装路径。" >&2
  exit 1
else
  mkdir -p "$(dirname "${PYGRAMPC_ROOT}")"
  git clone --recursive "${PYGRAMPC_REPO}" "${PYGRAMPC_ROOT}"
fi

python3 -m pip install --user scikit-build-core "pybind11==2.12.0" ninja
python3 -m pip install --user -e "${PYGRAMPC_ROOT}"

cat <<EOF

[GRAMPC] 安装完成。

下一步请在每个新终端运行:
  export PYGRAMPC_ROOT=${PYGRAMPC_ROOT}
  source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh

验证:
  python3 -c "import pygrampc; print('pygrampc ok')"
EOF
