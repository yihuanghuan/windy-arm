#!/bin/bash

set +e

echo "Cleaning ROS2 environment..."

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f ~/windshape-ws/install/setup.bash ]; then
    source ~/windshape-ws/install/setup.bash
fi

echo "Stopping ros2 daemon..."
ros2 daemon stop >/dev/null 2>&1

patterns=(
    "ros2"
    "launch"
    "component_container"
    "component_container_mt"
    "rviz2"
    "rqt"
    "robot_state_publisher"
    "joint_state_publisher"
    "move_group"
    "static_transform_publisher"
    "tf2"
    "mavros"
    "px4.launch"
    "vrpn"
    "odom_to_mavros"
    "vrpn_to_mavros"
    "laser_mapping"
    "fast_lio"
    "livox"
    "mid360"
    "camera"
    "usb_cam"
    "v4l2_camera"
    "master_arm"
    "slave_arm"
    "arm_hardware"
    "manipulator"
    "micro_ros"
    "dds"
    "fastdds"
    "FastDDS"
    "fastrtps"
    "cyclonedds"
    "CycloneDDS"
    "rmw"
)

echo "Terminating ROS-related processes..."
current_pgid=$(ps -o pgid= -p $$ | tr -d ' ')
for pattern in "${patterns[@]}"; do
    pgrep -f "$pattern" 2>/dev/null | while read -r pid; do
        [ -z "$pid" ] && continue
        [ "$pid" = "$$" ] && continue
        [ "$pid" = "$PPID" ] && continue
        target_pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
        [ -n "$target_pgid" ] && [ "$target_pgid" = "$current_pgid" ] && continue
        kill -TERM "$pid" >/dev/null 2>&1
    done
done

sleep 2

echo "Force killing remaining ROS-related processes..."
for pattern in "${patterns[@]}"; do
    pgrep -f "$pattern" 2>/dev/null | while read -r pid; do
        [ -z "$pid" ] && continue
        [ "$pid" = "$$" ] && continue
        [ "$pid" = "$PPID" ] && continue
        target_pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
        [ -n "$target_pgid" ] && [ "$target_pgid" = "$current_pgid" ] && continue
        kill -KILL "$pid" >/dev/null 2>&1
    done
done

sleep 1

echo "Resetting ros2 daemon..."
ros2 daemon stop >/dev/null 2>&1
rm -rf ~/.ros/ros_daemon* ~/.ros/_daemon* >/dev/null 2>&1

if command -v ros2 >/dev/null 2>&1; then
    remaining_nodes=$(ros2 node list 2>/dev/null)
    if [ -n "$remaining_nodes" ]; then
        echo "Warning: remaining ROS2 nodes detected:"
        echo "$remaining_nodes"
    else
        echo "No ROS2 nodes remaining."
    fi
fi

echo "Clean done."
