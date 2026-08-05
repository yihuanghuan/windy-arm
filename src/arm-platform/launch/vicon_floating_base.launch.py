"""Start the VRPN receiver and expose one Vicon rigid body as base state."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    server = DeclareLaunchArgument("server", default_value="192.168.50.96")
    port = DeclareLaunchArgument("port", default_value="3883")
    pose_topic = DeclareLaunchArgument(
        "pose_topic", default_value="/vrpn/base/pose",
        description="VRPN listener pose topic for the base rigid body")
    output_topic = DeclareLaunchArgument(
        "output_topic", default_value="/floating_base/vicon")
    zero_on_first_pose = DeclareLaunchArgument("zero_on_first_pose", default_value="true")

    vrpn_listener = Node(
        package="vrpn_listener", executable="vrpn_listener", name="vrpn_listener",
        output="screen", parameters=[{
            "server": LaunchConfiguration("server"),
            "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
            "frame_id": "vicon_world",
            "mainloop_frequency": 200.0,
            "refresh_trackers_frequency": 1.0,
            "tracker_mainloop_frequency": 200.0,
        }])
    bridge = Node(
        package="manipulator", executable="vicon_floating_base_bridge.py",
        name="vicon_floating_base_bridge", output="screen", parameters=[{
            "pose_topic": LaunchConfiguration("pose_topic"),
            "output_topic": LaunchConfiguration("output_topic"),
            "zero_on_first_pose": ParameterValue(
                LaunchConfiguration("zero_on_first_pose"), value_type=bool),
        }])
    return LaunchDescription([server, port, pose_topic, output_topic,
                              zero_on_first_pose, vrpn_listener, bridge])
