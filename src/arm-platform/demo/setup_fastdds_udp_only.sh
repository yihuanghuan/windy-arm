#!/usr/bin/env bash
# Source this file in every terminal participating in a local UDP-only ROS run.
#
# Usage:
#   source ./setup_fastdds_udp_only.sh       # ROS_DOMAIN_ID=78
#   source ./setup_fastdds_udp_only.sh 79    # explicit domain

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "[FastDDS] This script must be sourced: source $0 [ROS_DOMAIN_ID]" >&2
  exit 2
fi

if (( $# > 1 )); then
  echo "[FastDDS] usage: source ${BASH_SOURCE[0]} [ROS_DOMAIN_ID]" >&2
  return 2
fi

_fastdds_domain="${1:-78}"
if [[ ! "${_fastdds_domain}" =~ ^[0-9]+$ || ${#_fastdds_domain} -gt 3 ]] ||
    (( 10#${_fastdds_domain} > 232 )); then
  echo "[FastDDS] ROS_DOMAIN_ID must be an integer in [0, 232]: ${_fastdds_domain}" >&2
  unset _fastdds_domain
  return 2
fi

_fastdds_demo_dir="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd -P
)"
_fastdds_profile="${_fastdds_demo_dir}/fastdds_udp_only.xml"
if [[ ! -r "${_fastdds_profile}" ]]; then
  echo "[FastDDS] profile is not readable: ${_fastdds_profile}" >&2
  unset _fastdds_domain _fastdds_demo_dir _fastdds_profile
  return 2
fi

export ROS_DOMAIN_ID="$((10#${_fastdds_domain}))"
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
export FASTRTPS_DEFAULT_PROFILES_FILE="${_fastdds_profile}"

echo "[FastDDS] UDP-only localhost profile enabled: domain=${ROS_DOMAIN_ID}, profile=${FASTRTPS_DEFAULT_PROFILES_FILE}"

unset _fastdds_domain _fastdds_demo_dir _fastdds_profile
