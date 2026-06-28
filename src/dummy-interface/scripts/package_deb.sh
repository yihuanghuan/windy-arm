#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
ROS_DISTRO=${ROS_DISTRO:-humble}
ROS_SETUP=${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}
ROS_PACKAGE_NAME=${ROS_PACKAGE_NAME:-dummy_interface}
DEB_PACKAGE_NAME=${DEB_PACKAGE_NAME:-windshape-robot-interfaces}
PACKAGE_VERSION=${PACKAGE_VERSION:-0.1.0}
PACKAGE_ARCH=${PACKAGE_ARCH:-$(dpkg --print-architecture)}
PACKAGE_MAINTAINER=${PACKAGE_MAINTAINER:-windshape <dev@windshape.ai>}
INSTALL_ROOT=${INSTALL_ROOT:-/opt/windshape/robot/install}
OUTPUT_DIR=${OUTPUT_DIR:-${PROJECT_DIR}/dist}
WORK_DIR=${WORK_DIR:-${OUTPUT_DIR}/build-workspace}
STAGING_DIR=${STAGING_DIR:-${OUTPUT_DIR}/deb-staging}
DEB_PATH=${OUTPUT_DIR}/${DEB_PACKAGE_NAME}_${PACKAGE_VERSION}_${PACKAGE_ARCH}.deb
SHA256_PATH=${DEB_PATH}.sha256

if [ ! -f "${ROS_SETUP}" ]; then
  echo "ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi

rm -rf "${WORK_DIR}" "${STAGING_DIR}"
mkdir -p "${WORK_DIR}/src/dummy-interface" "${STAGING_DIR}/DEBIAN" "${OUTPUT_DIR}"

rsync -a --delete \
  --exclude '.git' \
  --exclude 'dist' \
  --exclude 'build' \
  --exclude 'install' \
  --exclude 'log' \
  "${PROJECT_DIR}/" "${WORK_DIR}/src/dummy-interface/"

set +u
source "${ROS_SETUP}"
set -u
cd "${WORK_DIR}"
colcon build --packages-select "${ROS_PACKAGE_NAME}" --install-base "${WORK_DIR}/install"

PACKAGE_INSTALL_DIR=${WORK_DIR}/install/${ROS_PACKAGE_NAME}
if [ ! -d "${PACKAGE_INSTALL_DIR}" ]; then
  echo "Package install directory not found: ${PACKAGE_INSTALL_DIR}" >&2
  exit 1
fi

mkdir -p "${STAGING_DIR}${INSTALL_ROOT}"
rsync -a --delete "${PACKAGE_INSTALL_DIR}/" "${STAGING_DIR}${INSTALL_ROOT}/${ROS_PACKAGE_NAME}/"

mkdir -p "${STAGING_DIR}${INSTALL_ROOT}"
if [ -f "${WORK_DIR}/install/setup.bash" ]; then
  install -m 0644 "${WORK_DIR}/install/setup.bash" "${STAGING_DIR}${INSTALL_ROOT}/setup.bash"
fi
if [ -f "${WORK_DIR}/install/local_setup.bash" ]; then
  install -m 0644 "${WORK_DIR}/install/local_setup.bash" "${STAGING_DIR}${INSTALL_ROOT}/local_setup.bash"
fi

cat > "${STAGING_DIR}/DEBIAN/control" <<EOF
Package: ${DEB_PACKAGE_NAME}
Version: ${PACKAGE_VERSION}
Section: robotics
Priority: optional
Architecture: ${PACKAGE_ARCH}
Maintainer: ${PACKAGE_MAINTAINER}
Depends: ros-${ROS_DISTRO}-rosidl-default-runtime, ros-${ROS_DISTRO}-std-msgs, ros-${ROS_DISTRO}-sensor-msgs
Description: Windshape robot ROS 2 interface messages
 ROS 2 message interface package for Windshape robot components.
EOF

cat > "${STAGING_DIR}/DEBIAN/postinst" <<EOF
#!/usr/bin/env bash
set -e
exit 0
EOF

cat > "${STAGING_DIR}/DEBIAN/prerm" <<EOF
#!/usr/bin/env bash
set -e
exit 0
EOF

chmod 0755 "${STAGING_DIR}/DEBIAN/postinst" "${STAGING_DIR}/DEBIAN/prerm"
find "${STAGING_DIR}" -type d -exec chmod 0755 {} +
find "${STAGING_DIR}" -type f ! -path "${STAGING_DIR}/DEBIAN/*" -exec chmod 0644 {} +

rm -f "${DEB_PATH}" "${SHA256_PATH}"
dpkg-deb --build --root-owner-group -Zxz "${STAGING_DIR}" "${DEB_PATH}"
sha256sum "${DEB_PATH}" > "${SHA256_PATH}"

echo "Built ${DEB_PATH}"
echo "Checksum ${SHA256_PATH}"
