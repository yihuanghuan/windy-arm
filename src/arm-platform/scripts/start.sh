#!/bin/bash
set -e

DDS_INTERFACE=${DDS_INTERFACE:-wlP1p1s0}
DDS_WAIT_TIMEOUT=${DDS_WAIT_TIMEOUT:-60}

BASHRC_DOMAIN_ID=$(HOME="${HOME:-/home/windshape}" bash -ic 'printf "%s" "${ROS_DOMAIN_ID:-}"' 2>/dev/null || true)
if [ -n "${BASHRC_DOMAIN_ID}" ]; then
  ROS_DOMAIN_ID="${BASHRC_DOMAIN_ID}"
fi
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-5}
export ROS_DOMAIN_ID

echo "Start robot system..."
echo "Waiting for ${DDS_INTERFACE} to become available..."

for ((i = 0; i < DDS_WAIT_TIMEOUT; i++)); do
  if [ -d "/sys/class/net/${DDS_INTERFACE}" ] && [ "$(cat "/sys/class/net/${DDS_INTERFACE}/operstate")" = "up" ] && ip -4 addr show "${DDS_INTERFACE}" | grep -q 'inet '; then
    echo "${DDS_INTERFACE} is available."
    break
  fi
  sleep 1
done

if ! [ -d "/sys/class/net/${DDS_INTERFACE}" ] || ! [ "$(cat "/sys/class/net/${DDS_INTERFACE}/operstate" 2>/dev/null || true)" = "up" ] || ! ip -4 addr show "${DDS_INTERFACE}" | grep -q 'inet '; then
  echo "${DDS_INTERFACE} did not become available within ${DDS_WAIT_TIMEOUT}s."
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source /opt/windshape/manipulator/install/setup.bash
set -u

exec ros2 launch manipulator demo.launch.py
