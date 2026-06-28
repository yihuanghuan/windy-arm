from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    pkg_share = FindPackageShare('manipulator').find('manipulator')
    config_file = os.path.join(pkg_share, 'config', 'slave_arm.yaml')

    
    return LaunchDescription([
        Node(
            package='manipulator',
            executable='arm_hardware_node',
            name='arm_hardware_node',
            output='screen',
            namespace='arm',
            parameters=[
                config_file,
                {'arm_type': 'a_l1'},
                {'arm_version': 'gamma'},
                # 串口端口名
                {'port_name': '/dev/ttyTHS3'},     
            ]
        ),
    ])
