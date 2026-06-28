from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_share = FindPackageShare('manipulator')
    
    motor_config_path = PathJoinSubstitution([pkg_share, 'motor_config.yaml'])
    arm_config_path = PathJoinSubstitution([pkg_share, 'arm_config.yaml'])
    
    enable_collision_avoidance_arg = DeclareLaunchArgument(
        'enable_collision_avoidance',
        default_value='True',
        description='Enable collision avoidance with propellers'
    )
    
    safety_distance_arg = DeclareLaunchArgument(
        'safety_distance',
        default_value='0.05',
        description='Safety distance for collision avoidance'
    )
    
    publish_joint_state_arg = DeclareLaunchArgument(
        'publish_joint_state',
        default_value='True',
        description='Whether to publish joint states'
    )
    
    publish_joint_feedback_arg = DeclareLaunchArgument(
        'publish_joint_feedback',
        default_value='True',
        description='Whether to publish joint feedback'
    )
    
    return LaunchDescription([
        enable_collision_avoidance_arg,
        safety_distance_arg,
        publish_joint_state_arg,
        publish_joint_feedback_arg,
        Node(
            package='manipulator',
            executable='slave_arm_node',
            name='slave_arm_node',
            output='screen',
            namespace='slave',
            parameters=[
                {'arm_type': 'a_l1'},
                {'arm_version': 'gamma'},
                {'port_name': '/dev/ttyTHS3'},
                {'max_velocity': 3.0},
                {'motor_config_path': motor_config_path},
                {'arm_config_path': arm_config_path},
                {'G_GAIN_0': 1.5},
                {'G_GAIN_1': 0.5},
                {'G_GAIN_2': 1.5},
                {'MAX_TORQUE': 3.0},
                {'GRAVITY': 9.81},
                {'uav_roll': 0.0},
                {'uav_pitch': 0.0},
                {'uav_yaw': 0.0},
                {'arm_roll': 0.0},
                {'arm_pitch': 0.0},
                {'arm_yaw': 0.0},
                {'debug_info': True},
                {'debug_rate': 1.0},
                {'FORCE_FEEDBACK_THRESHOLD': 0.5},
                {'FORCE_FEEDBACK_GAIN': 0.5},
                {'publish_joint_state': LaunchConfiguration('publish_joint_state')},
                {'publish_joint_feedback': LaunchConfiguration('publish_joint_feedback')},
                {'urdf_path': PathJoinSubstitution([pkg_share, 'arm.urdf'])},
                {'enable_collision_avoidance': LaunchConfiguration('enable_collision_avoidance')},
                {'safety_distance': LaunchConfiguration('safety_distance')},
                {'cylinder1_center': [-0.15, -0.37, -0.1]}, # right cylinder
                {'cylinder1_radius': 0.27},
                {'cylinder1_height': 0.1},
                {'cylinder1_axis': [0.0, 0.0, 1.0]},
                {'cylinder2_center': [-0.15, 0.37, -0.1]}, # left cylinder
                {'cylinder2_radius': 0.27},
                {'cylinder2_height': 0.1},
                {'cylinder2_axis': [0.0, 0.0, 1.0]}
            ]
        ),
    ])