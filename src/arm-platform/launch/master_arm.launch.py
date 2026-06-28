from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('manipulator')
    
    motor_config_path = PathJoinSubstitution([pkg_share, 'motor_config.yaml'])
    arm_config_path = PathJoinSubstitution([pkg_share, 'arm_config.yaml'])
    
    publish_joint_state_arg = DeclareLaunchArgument(
        'publish_joint_state',
        default_value='False',
        description='Whether to publish joint states'
    )
    
    publish_joint_feedback_arg = DeclareLaunchArgument(
        'publish_joint_feedback',
        default_value='False',
        description='Whether to publish joint feedback'
    )
    
    master_arm_node = Node(
        package='manipulator',
        executable='master_arm_node',
        name='master_arm_node',
        output='screen',
        namespace='master',
        sigterm_timeout='20',
        sigkill_timeout='20',
        parameters=[
            {'arm_type': 'a_l1'},
            {'arm_version': 'gamma'},
            {'auto_reset': True},
            {'port_name': '/dev/ttyUSB0'},
            {'motor_config_path': motor_config_path},
            {'arm_config_path': arm_config_path},
            {'MAX_TORQUE': 3.0},
            {'GRAVITY': 9.81},
            {'debug_info': True},
            {'debug_rate': 1.0},
            {'FORCE_FEEDBACK_THRESHOLD': 0.5},
            {'FORCE_FEEDBACK_GAIN': 0.5},
            {'publish_joint_state': LaunchConfiguration('publish_joint_state')},
            {'publish_joint_feedback': LaunchConfiguration('publish_joint_feedback')},
            {'urdf_path': PathJoinSubstitution([pkg_share, 'arm.urdf'])}
        ]
    )
    
    return LaunchDescription([
        publish_joint_state_arg,
        publish_joint_feedback_arg,
        master_arm_node,
    ])