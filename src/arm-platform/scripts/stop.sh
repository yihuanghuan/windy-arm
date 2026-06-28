#!/bin/bash

set +e

source /opt/ros/humble/setup.bash >/dev/null 2>&1
source /opt/windshape/manipulator/install/setup.bash >/dev/null 2>&1

ros2 daemon stop >/dev/null 2>&1

exit 0