from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def get_localization_launch(context, *args, **kwargs):
    loc_type = context.launch_configurations.get('loc_type', 'mid360')
    
    fastlio2_path = get_package_share_directory('fast_lio')
    
    vicon_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'vrpn_listener', 'vrpn_client.launch'],
        output='screen'
    )
    
    odom_to_mavros = Node(
        package="vrpn_listener",
        executable="odom_to_mavros.py",
        name="odom_to_mavros",
        output="screen",
    )
    
    vicon_to_mavros = Node(
        package="vrpn_listener",
        executable="vrpn_to_mavros.py",
        name="vrpn_to_mavros",
        output="screen",
    )
    
    lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            fastlio2_path + '/launch/mapping.launch.py'
        )
    )
    
    if loc_type == 'vicon':
        return [vicon_launch, vicon_to_mavros]
    else:
        return [
            lio_launch,
            odom_to_mavros, 
            # vicon_launch
        ]


def generate_launch_description():
    mavros_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'mavros', 'px4.launch', 'gcs_url:=udp://:14550@', 'fcu_url:=/dev/ttyTHS1:921600'],
        output='screen'
    )
    
    loc_type_arg = DeclareLaunchArgument(
        'loc_type',
        default_value='mid360',
        description='Choose localization type: vicon or mid360'
    )

    return LaunchDescription([
        loc_type_arg,
        mavros_launch,
        OpaqueFunction(function=get_localization_launch)
    ])
