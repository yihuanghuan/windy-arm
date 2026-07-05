#!/usr/bin/env bash
# 配置外部安装的 ACADOS / GRAMPC 运行环境。
#
# 用法:
#   source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
#
# 如需自定义安装位置:
#   export ACADOS_SOURCE_DIR=/path/to/acados
#   export PYGRAMPC_ROOT=/path/to/pygrampc
#   source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh

export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-${HOME}/code/acados}"
export PYGRAMPC_ROOT="${PYGRAMPC_ROOT:-${HOME}/code/pygrampc}"

_acados_add_path_once() {
  local var_name="$1"
  local path_value="$2"
  local current_value="${!var_name:-}"

  case ":${current_value}:" in
    *":${path_value}:"*) ;;
    *)
      if [ -n "${current_value}" ]; then
        export "${var_name}=${path_value}:${current_value}"
      else
        export "${var_name}=${path_value}"
      fi
      ;;
  esac
}

if [ ! -d "${ACADOS_SOURCE_DIR}" ]; then
  echo "[ACADOS] 未找到 ACADOS_SOURCE_DIR: ${ACADOS_SOURCE_DIR}" >&2
  echo "[ACADOS] 请先运行 scripts/install_acados.sh, 或手动设置 ACADOS_SOURCE_DIR 后再 source 本脚本。" >&2
fi

if [ ! -d "${ACADOS_SOURCE_DIR}/lib" ]; then
  echo "[ACADOS] 未找到 ${ACADOS_SOURCE_DIR}/lib, ACADOS 可能尚未编译安装。" >&2
fi

if [ ! -d "${ACADOS_SOURCE_DIR}/interfaces/acados_template" ]; then
  echo "[ACADOS] 未找到 acados_template 源码目录: ${ACADOS_SOURCE_DIR}/interfaces/acados_template" >&2
fi

if [ ! -d "${PYGRAMPC_ROOT}" ]; then
  echo "[GRAMPC] 未找到 PYGRAMPC_ROOT: ${PYGRAMPC_ROOT}" >&2
  echo "[GRAMPC] 如需使用 GRAMPC,请先运行 scripts/install_grampc.sh,或手动设置 PYGRAMPC_ROOT。" >&2
fi

_acados_add_path_once LD_LIBRARY_PATH "${ACADOS_SOURCE_DIR}/lib"
_acados_add_path_once PATH "${ACADOS_SOURCE_DIR}/bin"
_acados_add_path_once PYTHONPATH "${ACADOS_SOURCE_DIR}/interfaces/acados_template"

unset -f _acados_add_path_once
