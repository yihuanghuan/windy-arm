#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
ROS_DISTRO=${ROS_DISTRO:-humble}
ROS_SETUP=${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}
ROS_PACKAGE_NAME=${ROS_PACKAGE_NAME:-manipulator}
DEB_PACKAGE_NAME=${DEB_PACKAGE_NAME:-windshape-manipulator}
PACKAGE_VERSION=${PACKAGE_VERSION:-0.1.0}
PACKAGE_ARCH=${PACKAGE_ARCH:-$(dpkg --print-architecture)}
PACKAGE_MAINTAINER=${PACKAGE_MAINTAINER:-windshape <dev@windshape.ai>}
INSTALL_PREFIX=${INSTALL_PREFIX:-/opt/windshape/manipulator}
OUTPUT_DIR=${OUTPUT_DIR:-${PROJECT_DIR}/dist}
WORK_DIR=${WORK_DIR:-${OUTPUT_DIR}/build-workspace}
STAGING_DIR=${STAGING_DIR:-${OUTPUT_DIR}/deb-staging}
DEB_PATH=${OUTPUT_DIR}/${DEB_PACKAGE_NAME}_${PACKAGE_VERSION}_${PACKAGE_ARCH}.deb
SHA256_PATH=${DEB_PATH}.sha256
DUMMY_INTERFACE_DIR=${DUMMY_INTERFACE_DIR:-${PROJECT_DIR}/../dummy-interface}
DUMMY_INTERFACE_GIT_URL=${DUMMY_INTERFACE_GIT_URL:-http://10.0.2.66:8000/aerial_manipulator_hub/interface/dummy-interface.git}

if [ ! -f "${ROS_SETUP}" ]; then
  echo "ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi

rm -rf "${WORK_DIR}" "${STAGING_DIR}"
mkdir -p "${WORK_DIR}/src/arm-platform" "${STAGING_DIR}/DEBIAN" "${OUTPUT_DIR}"

rsync -a --delete \
  --exclude '.git' \
  --exclude 'dist' \
  --exclude 'build' \
  --exclude 'install' \
  --exclude 'log' \
  --exclude 'docker' \
  --exclude '__pycache__' \
  --exclude '*.pyc' \
  "${PROJECT_DIR}/" "${WORK_DIR}/src/arm-platform/"

if [ -d "${DUMMY_INTERFACE_DIR}" ]; then
  mkdir -p "${WORK_DIR}/src/dummy-interface"
  rsync -a --delete \
    --exclude '.git' \
    --exclude 'dist' \
    --exclude 'build' \
    --exclude 'install' \
    --exclude 'log' \
    "${DUMMY_INTERFACE_DIR}/" "${WORK_DIR}/src/dummy-interface/"
elif command -v git >/dev/null 2>&1; then
  git clone "${DUMMY_INTERFACE_GIT_URL}" "${WORK_DIR}/src/dummy-interface"
else
  echo "dummy-interface not found and git is not available" >&2
  exit 1
fi

unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset PYTHONPATH
set +u
source "${ROS_SETUP}"
set -u
cd "${WORK_DIR}"
colcon build --packages-select dummy_interface --install-base "${WORK_DIR}/install"
set +u
source "${WORK_DIR}/install/setup.bash"
set -u
colcon build --packages-select "${ROS_PACKAGE_NAME}" --install-base "${WORK_DIR}/install" --cmake-args -DCMAKE_BUILD_TYPE=Release

PACKAGE_INSTALL_DIR=${WORK_DIR}/install/${ROS_PACKAGE_NAME}
if [ ! -d "${PACKAGE_INSTALL_DIR}" ]; then
  echo "Package install directory not found: ${PACKAGE_INSTALL_DIR}" >&2
  exit 1
fi

mkdir -p "${STAGING_DIR}${INSTALL_PREFIX}/install"
rsync -a --delete "${PACKAGE_INSTALL_DIR}/" "${STAGING_DIR}${INSTALL_PREFIX}/install/${ROS_PACKAGE_NAME}/"

for setup_file in setup.bash local_setup.bash setup.sh local_setup.sh setup.zsh local_setup.zsh; do
  if [ -f "${WORK_DIR}/install/${setup_file}" ]; then
    install -m 0644 "${WORK_DIR}/install/${setup_file}" "${STAGING_DIR}${INSTALL_PREFIX}/install/${setup_file}"
  fi
done

if [ -d "${STAGING_DIR}${INSTALL_PREFIX}/install/${ROS_PACKAGE_NAME}/lib/systemd/system" ]; then
  mkdir -p "${STAGING_DIR}/lib/systemd/system"
  rsync -a "${STAGING_DIR}${INSTALL_PREFIX}/install/${ROS_PACKAGE_NAME}/lib/systemd/system/" "${STAGING_DIR}/lib/systemd/system/"
  rm -rf "${STAGING_DIR}${INSTALL_PREFIX}/install/${ROS_PACKAGE_NAME}/lib/systemd"
fi

if [ -d "${PROJECT_DIR}/debian" ]; then
  for maintainer_script in postinst prerm postrm; do
    if [ -f "${PROJECT_DIR}/debian/${maintainer_script}" ]; then
      install -m 0755 "${PROJECT_DIR}/debian/${maintainer_script}" "${STAGING_DIR}/DEBIAN/${maintainer_script}"
    fi
  done
fi

cat > "${STAGING_DIR}/DEBIAN/control" <<EOF
Package: ${DEB_PACKAGE_NAME}
Version: ${PACKAGE_VERSION}
Section: robotics
Priority: optional
Architecture: ${PACKAGE_ARCH}
Maintainer: ${PACKAGE_MAINTAINER}
Depends: windshape-robot-interfaces, ros-${ROS_DISTRO}-rclcpp, ros-${ROS_DISTRO}-sensor-msgs, ros-${ROS_DISTRO}-std-msgs, ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-visualization-msgs, ros-${ROS_DISTRO}-serial, libeigen3-dev, libyaml-cpp0.7, libpinocchio-dev
Description: Windshape manipulator runtime
 ROS 2 manipulator control runtime for Windshape robot systems.
EOF

if [ ! -f "${STAGING_DIR}/DEBIAN/postinst" ]; then
  cat > "${STAGING_DIR}/DEBIAN/postinst" <<EOF
#!/usr/bin/env bash
set -e
if command -v systemctl >/dev/null 2>&1; then
  systemctl daemon-reload || true
fi
exit 0
EOF
fi

if [ ! -f "${STAGING_DIR}/DEBIAN/prerm" ]; then
  cat > "${STAGING_DIR}/DEBIAN/prerm" <<EOF
#!/usr/bin/env bash
set -e
exit 0
EOF
fi

if [ ! -f "${STAGING_DIR}/DEBIAN/postrm" ]; then
  cat > "${STAGING_DIR}/DEBIAN/postrm" <<EOF
#!/usr/bin/env bash
set -e
if command -v systemctl >/dev/null 2>&1; then
  systemctl daemon-reload || true
fi
exit 0
EOF
fi

chmod 0755 "${STAGING_DIR}/DEBIAN/postinst" "${STAGING_DIR}/DEBIAN/prerm" "${STAGING_DIR}/DEBIAN/postrm"
find "${STAGING_DIR}" -type d -exec chmod 0755 {} +
find "${STAGING_DIR}" -type f ! -path "${STAGING_DIR}/DEBIAN/*" -exec chmod 0644 {} +
find "${STAGING_DIR}${INSTALL_PREFIX}/install/${ROS_PACKAGE_NAME}/share/${ROS_PACKAGE_NAME}" -maxdepth 1 -type f -name '*.sh' -exec chmod 0755 {} +

rm -f "${DEB_PATH}" "${SHA256_PATH}"
dpkg-deb --build --root-owner-group -Zxz "${STAGING_DIR}" "${DEB_PATH}"
sha256sum "${DEB_PATH}" > "${SHA256_PATH}"

echo "Built ${DEB_PATH}"
echo "Checksum ${SHA256_PATH}"
