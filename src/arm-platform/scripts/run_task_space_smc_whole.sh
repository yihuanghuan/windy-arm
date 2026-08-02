#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
WS_ROOT="${WINDYLAB_WS:-$(cd "$(dirname "${SCRIPT_PATH}")/../../.." && pwd)}"
ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-/opt/ros/humble/setup.bash}"

if [ ! -f "${ROS_DISTRO_SETUP}" ]; then
  echo "ROS setup not found: ${ROS_DISTRO_SETUP}" >&2
  exit 2
fi

unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset PYTHONPATH
unset LD_LIBRARY_PATH
unset ROS_PACKAGE_PATH

set +u
source "${ROS_DISTRO_SETUP}"
set -u

CACHE="${WS_ROOT}/build/manipulator/CMakeCache.txt"
EXPECTED_CACHE_PREFIX="CMAKE_INSTALL_PREFIX:PATH=${WS_ROOT}/install/manipulator"
INSTALLED_LAUNCH="${WS_ROOT}/install/manipulator/share/manipulator/task_space_smc_whole.launch.py"
NEEDS_BUILD="${WINDYLAB_SMC_REBUILD:-0}"
CLEAN_CACHE=0
if [ ! -f "${WS_ROOT}/install/setup.bash" ] ||
    [ ! -e "${INSTALLED_LAUNCH}" ] ||
    [ ! -f "${CACHE}" ]; then
  NEEDS_BUILD=1
elif ! grep -Fxq "${EXPECTED_CACHE_PREFIX}" "${CACHE}"; then
  echo "Repairing stale CMake install prefix in ${CACHE}"
  NEEDS_BUILD=1
  CLEAN_CACHE=1
fi

cd "${WS_ROOT}"
if [ "${NEEDS_BUILD}" = "1" ]; then
  BUILD_ARGS=(
    --build-base "${WS_ROOT}/build"
    --install-base "${WS_ROOT}/install"
    --symlink-install
    --packages-select dummy_interface serial dummy_description manipulator
    --cmake-args -DCMAKE_BUILD_TYPE=Release
  )
  if [ "${CLEAN_CACHE}" = "1" ]; then
    BUILD_ARGS+=(--cmake-clean-cache)
  fi
  colcon build "${BUILD_ARGS[@]}"
else
  echo "Using existing workspace build: ${WS_ROOT}/install"
fi

set +u
source "${WS_ROOT}/install/setup.bash"
if [ -f "${WS_ROOT}/src/arm-platform/demo/setup_fastdds_udp_only.sh" ]; then
  source "${WS_ROOT}/src/arm-platform/demo/setup_fastdds_udp_only.sh" \
    "${ROS_DOMAIN_ID:-78}"
fi
set -u

PREFIX="$(ros2 pkg prefix manipulator)"
EXPECTED_PREFIX="${WS_ROOT}/install/manipulator"
if [ "${PREFIX}" != "${EXPECTED_PREFIX}" ]; then
  echo "Wrong manipulator overlay selected: ${PREFIX}" >&2
  echo "Expected: ${EXPECTED_PREFIX}" >&2
  echo "The current shell still contains an external overlay. Start a new terminal and rerun this script." >&2
  exit 3
fi

exec ros2 launch manipulator task_space_smc_whole.launch.py "$@"
