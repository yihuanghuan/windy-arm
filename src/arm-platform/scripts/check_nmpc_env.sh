#!/usr/bin/env bash
set -u

fail_count=0
warn_count=0

pass() {
  echo "[通过] $*"
}

warn() {
  warn_count=$((warn_count + 1))
  echo "[警告] $*" >&2
}

fail() {
  fail_count=$((fail_count + 1))
  echo "[失败] $*" >&2
}

check_cmd() {
  local cmd="$1"
  local hint="$2"
  if command -v "${cmd}" >/dev/null 2>&1; then
    pass "找到命令 ${cmd}: $(command -v "${cmd}")"
  else
    fail "缺少命令 ${cmd}。${hint}"
  fi
}

check_python_module() {
  local module="$1"
  local hint="$2"
  if python3 -c "import ${module}" >/dev/null 2>&1; then
    pass "Python 模块 ${module} 可导入"
  else
    fail "Python 模块 ${module} 不可导入。${hint}"
  fi
}

echo "== NMPC / ACADOS 环境检查 =="

check_cmd python3 "请安装 python3。"
check_cmd cmake "请安装: sudo apt install cmake"
check_cmd make "请安装: sudo apt install build-essential"
check_cmd git "请安装: sudo apt install git"

check_python_module numpy "请安装: python3 -m pip install --user numpy"
check_python_module scipy "请安装: sudo apt install python3-scipy 或 pip 安装 scipy"
check_python_module casadi "请安装: python3 -m pip install --user casadi"
check_python_module pinocchio "请安装: sudo apt install ros-humble-pinocchio,并 source /opt/ros/humble/setup.bash"
check_python_module acados_template "请先运行 scripts/install_acados.sh,然后 source scripts/setup_acados_env.sh"

ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-${HOME}/code/acados}"
echo "[信息] ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR}"

if [ -d "${ACADOS_SOURCE_DIR}" ]; then
  pass "ACADOS 源码目录存在"
else
  fail "ACADOS 源码目录不存在。建议运行: scripts/install_acados.sh"
fi

if [ -f "${ACADOS_SOURCE_DIR}/lib/libacados.so" ]; then
  pass "找到 libacados.so"
else
  fail "未找到 ${ACADOS_SOURCE_DIR}/lib/libacados.so。ACADOS 可能尚未 make install。"
fi

if [ -n "${AMENT_PREFIX_PATH:-}" ]; then
  pass "ROS 2 环境已 source"
else
  warn "未检测到 AMENT_PREFIX_PATH。运行 ROS demo 前请先 source /opt/ros/humble/setup.bash 和工作空间 install/setup.bash。"
fi

echo
echo "== 检查结果 =="
echo "失败: ${fail_count}, 警告: ${warn_count}"

if [ "${fail_count}" -eq 0 ]; then
  echo "NMPC 环境检查通过。"
  exit 0
fi

cat <<EOF

修复建议:
  1. source /opt/ros/humble/setup.bash
  2. cd ~/code/windylab_ws/src/arm-platform
  3. scripts/install_acados.sh
  4. source scripts/setup_acados_env.sh
  5. scripts/check_nmpc_env.sh
EOF

exit 1
