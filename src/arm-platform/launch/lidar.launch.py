from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    livox_driver_path = get_package_share_directory('livox_ros_driver2')
    
    left_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            livox_driver_path + '/launch_ROS2/msg_MID360s_L_launch.py'
        )
    )
    
    right_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            livox_driver_path + '/launch_ROS2/msg_MID360s_R_launch.py'
        ),
    )

    delayed_left_lidar_launch = TimerAction(
        period=5.0,
        actions=[left_lidar_launch],
    )

    return LaunchDescription([
        right_lidar_launch,
        #delayed_left_lidar_launch,
    ])
