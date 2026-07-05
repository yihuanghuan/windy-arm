#!/usr/bin/env bash
set -euo pipefail

# 从源码安装 ACADOS。默认安装到 $HOME/code/acados,也可提前设置:
#   export ACADOS_SOURCE_DIR=/path/to/acados
#
# 本仓库只部署 acados SQP_RTI + HPIPM/qpOASES:
#   - HPIPM 是 acados 默认 QP 后端
#   - qpOASES 通过 ACADOS_WITH_QPOASES=ON 启用
#   - OSQP/DAQP 显式关闭,避免安装无用后端

ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-${HOME}/code/acados}"
ACADOS_REPO="${ACADOS_REPO:-https://github.com/acados/acados.git}"
JOBS="${JOBS:-$(nproc)}"

echo "[ACADOS] install dir: ${ACADOS_SOURCE_DIR}"
echo "[ACADOS] repo: ${ACADOS_REPO}"
echo "[ACADOS] jobs: ${JOBS}"

command -v git >/dev/null || { echo "[ACADOS] 缺少 git,请先安装: sudo apt install git" >&2; exit 1; }
command -v cmake >/dev/null || { echo "[ACADOS] 缺少 cmake,请先安装: sudo apt install cmake" >&2; exit 1; }
command -v make >/dev/null || { echo "[ACADOS] 缺少 make,请先安装: sudo apt install build-essential" >&2; exit 1; }
command -v python3 >/dev/null || { echo "[ACADOS] 缺少 python3" >&2; exit 1; }

if [ -d "${ACADOS_SOURCE_DIR}/.git" ]; then
  echo "[ACADOS] 已存在源码目录,更新源码和 submodule..."
  git -C "${ACADOS_SOURCE_DIR}" pull --ff-only
  git -C "${ACADOS_SOURCE_DIR}" submodule update --recursive --init
elif [ -e "${ACADOS_SOURCE_DIR}" ]; then
  echo "[ACADOS] 目标路径已存在但不是 git 仓库: ${ACADOS_SOURCE_DIR}" >&2
  echo "[ACADOS] 请删除/移动该目录,或设置 ACADOS_SOURCE_DIR 到新的安装路径。" >&2
  exit 1
else
  mkdir -p "$(dirname "${ACADOS_SOURCE_DIR}")"
  git clone --recursive "${ACADOS_REPO}" "${ACADOS_SOURCE_DIR}"
fi

mkdir -p "${ACADOS_SOURCE_DIR}/build"
cd "${ACADOS_SOURCE_DIR}/build"

# 额外 cmake 参数,用于跨平台。例如 Jetson/ARM64 需指定 blasfeo target:
#   export ACADOS_CMAKE_ARGS="-DBLASFEO_TARGET=ARMV8A_ARM_CORTEX_A57"
# 一般 x86_64 电脑留空即可(blasfeo 自动探测)。
echo "[ACADOS] extra cmake args: ${ACADOS_CMAKE_ARGS:-<none>}"
cmake \
  -DACADOS_WITH_QPOASES=ON \
  -DACADOS_WITH_OSQP=OFF \
  -DACADOS_WITH_DAQP=OFF \
  ${ACADOS_CMAKE_ARGS:-} ..
make install -j"${JOBS}"

python3 -m pip install --user -e "${ACADOS_SOURCE_DIR}/interfaces/acados_template"

cat <<EOF

[ACADOS] 安装完成。

下一步请在每个新终端运行:
  source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh

验证:
  python3 -c "import acados_template; print('acados ok')"

如果你的工作空间不在 ~/code/windylab_ws,请使用实际路径 source setup_acados_env.sh。
EOF
