"""Visualization only; requires an independently verified encoder feedback topic.

Hardware feedback is OFF by default. Enabling it requires physically isolated RX
wiring. Only the dedicated receiver can be started; no control driver is included.
All data and TF topics are isolated under /real_arm; no calibration is applied.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    urdf = Path(LaunchConfiguration('urdf_path').perform(context)).resolve()
    description = urdf.read_text()
    root = ET.fromstring(description)
    # Physical six-axis gamma6 identity is confirmed by the user.
    # Encoder zero offsets are deliberately NOT imported from calibration YAML.
    names = [j.get('name') for j in root.findall('joint')
             if j.get('type') != 'fixed']
    if names != [f'joint{i}' for i in range(1, 7)]:
        raise ValueError('Expected verified joint1..joint6 model')
    nodes = []
    if LaunchConfiguration('start_feedback').perform(context).lower() == 'true':
        if LaunchConfiguration('rx_isolated').perform(context).lower() != 'true':
            raise ValueError('Serial access refused: confirm physical RX-only isolation first')
        nodes.append(Node(
            package='real_arm_readonly', executable='feedback',
            output='screen', parameters=[{
                'port': LaunchConfiguration('port'), 'rx_isolated': True}]))
    return nodes + [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='real_arm_readonly_state_publisher', output='screen',
             parameters=[{'robot_description': description}],
             remappings=[('joint_states', '/real_arm/raw_joint_states'),
                         ('robot_description', '/real_arm/robot_description'),
                         ('/tf', '/real_arm/tf'), ('/tf_static', '/real_arm/tf_static')]),
        Node(package='rviz2', executable='rviz2',
             name='real_arm_readonly_rviz', output='screen',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             remappings=[('/tf', '/real_arm/tf'), ('/tf_static', '/real_arm/tf_static')],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]


def generate_launch_description():
    # Support both source-tree and the existing flat share/manipulator install.
    directory = Path(__file__).resolve().parent
    config = directory.parent / 'config'
    if not (config / 'arm6.urdf').is_file():
        config = directory if (directory / 'arm6.urdf').is_file() else directory.parent
    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value=str(config / 'arm6.urdf')),
        DeclareLaunchArgument('start_feedback', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rx_isolated', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('port', default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz_config', default_value=str(config / 'real_arm_readonly.rviz')),
        OpaqueFunction(function=_nodes),
    ])
