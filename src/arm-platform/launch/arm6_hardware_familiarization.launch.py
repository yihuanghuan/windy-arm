"""Real gamma6 feedback/RViz and optional gravity guidance, with one serial owner."""
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    share = Path(get_package_share_directory('manipulator'))
    mode = LaunchConfiguration('mode').perform(context)
    port = LaunchConfiguration('port_name').perform(context)
    if not port or not Path(port).exists():
        raise ValueError('pass port_name:=<verified serial device/by-id path>')
    config = yaml.safe_load(Path(LaunchConfiguration('config').perform(context)).read_text())
    urdf = share / 'arm6.urdf'
    root = ET.fromstring(urdf.read_text())
    joints = [j for j in root.findall('joint') if j.get('type') != 'fixed']
    if [j.get('name') for j in joints] != [f'joint{i}' for i in range(1, 7)]:
        raise ValueError('arm6.urdf must contain only joint1..joint6')
    # Validate policy offline before launching/opening any hardware process.
    import sys
    from ament_index_python.packages import get_package_prefix
    module_dir = str(Path(get_package_prefix('manipulator')) / 'lib' / 'manipulator')
    if module_dir not in sys.path:
        sys.path.insert(0, module_dir)
    from task_space_smc.core.manual_guidance import Guidance
    from task_space_smc.core.speed_safety import SPEED_KEYS, MANUAL_KEYS
    policy = Guidance(str(urdf), config)
    motors = yaml.safe_load((share/'motor_config.yaml').read_text())['motor_models']
    joints_config = yaml.safe_load((share/'arm_config.yaml').read_text())['arms']['a_l1_gamma6']['joints']
    for joint, limit in zip(joints_config, config['guide_manual_instant_abort_speed']):
        if limit >= motors[joint['model']]['max_velocity']:
            raise ValueError('manual instantaneous speed must be below hardware capability')
    manual_flag = LaunchConfiguration('manual_push_test').perform(context) == 'true'
    seed_params = {}
    seed_path = LaunchConfiguration('hold_command_file').perform(context)
    supported_path = LaunchConfiguration('supported_startup_file').perform(context)
    if supported_path:
        if (seed_path or mode != 'guidance' or LaunchConfiguration('start_guidance').perform(context) != 'false' or
                LaunchConfiguration('external_support_confirmed').perform(context) != 'true'):
            raise ValueError('supported startup requires guidance-capable driver without starting guidance, external support, and no old hold seed')
        from task_space_smc.core.supported_startup import propose_supported_hold
        evidence = yaml.safe_load(Path(supported_path).read_text())
        if (Path(evidence['port']).resolve() != Path(port).resolve() or
                not 0 <= time.time()-float(evidence['captured_unix_sec']) <= 15):
            raise ValueError('supported feedback must be recent and for this device')
        proposal = propose_supported_hold(evidence['samples'], policy)
        for key in ('position','current','p','d'):
            seed_params['initial_hold_'+key] = proposal['command'][key]
        seed_params['initial_hold_expected_q'] = proposal['q']
        seed_params['initial_hold_position_tolerance'] = .002
        seed_params['initial_hold_is_support_estimate'] = True
    if seed_path:
        seed = yaml.safe_load(Path(seed_path).read_text())
        if (Path(seed['port']).resolve() != Path(port).resolve() or
                not 0 <= time.time()-float(seed['captured_unix_sec']) <= 120):
            raise ValueError('hold command seed must be fresh and for this serial device')
        policy.set_hold_command(seed['command'])  # offline six-axis/gain/budget validation
        for key in ('position','current','p','d'):
            seed_params['initial_hold_'+key] = [float(x) for x in seed['command'][key]]
        seed_params['initial_hold_expected_q'] = [float(x) for x in seed['q']]
        if len(seed['q']) != 6:
            raise ValueError('seed feedback must have six axes')
    # RSP consumes raw encoder feedback. Encode only the explicit model offset
    # into this in-memory visual description; the dynamics URDF stays untouched.
    for joint, offset in zip(joints, policy.offset):
        if offset != 0.:
            origin = joint.find('origin')
            axis = [float(x) for x in joint.find('axis').get('xyz').split()]
            if origin.get('rpy') != '0 0 0' or sum(x != 0 for x in axis) != 1:
                raise ValueError('unsupported visual offset transform')
            origin.set('rpy', ' '.join(str(offset * x) for x in axis))
    driver = Node(package='manipulator', executable='student_arm_node',
                  name='student_arm_node', output='screen', parameters=[{
        'arm_type': 'a_l1', 'arm_version': 'gamma6', 'port_name': port,
        'motor_config_path': str(share / 'motor_config.yaml'),
        'arm_config_path': str(share / 'arm_config.yaml'), 'urdf_path': str(urdf),
        'familiarization_mode': True, 'read_only': mode == 'readonly',
        'enable_direct_effort_command': True, 'allow_position_handover': False,
        'step_on_effort_command': False, 'publish_joint_feedback': True,
        'wait_for_first_effort_command': mode == 'guidance',
        'control_rate_hz': float(config['rate_hz']),
        'feedback_timeout_sec': float(config['feedback_timeout_sec']),
        'phase_speed_protection': True,
        **{key: float(config[key]) for key in SPEED_KEYS},
        **{key: config[key] for key in MANUAL_KEYS},
        'guide_manual_sustained_duration': float(config['guide_manual_sustained_duration']),
        'bumpless_handover': True, 'manual_push_test': manual_flag,
        'joint_limit_margin': float(config['joint_limit_margin']),
        **{key: config[key] for key in ('no_touch_duration_sec', 'no_touch_position_tolerance',
                                       'no_touch_speed_limit','required_handover_trials')},
        **seed_params,
        'heartbeat_timeout_sec': .2, 'effort_command_timeout_sec': .05,
        'joint_lower_limits': policy.lower.tolist(), 'joint_upper_limits': policy.upper.tolist(),
        'p_gain': config['hold_p'], 'd_gain': config['hold_d'],
        'gravity_hold_enabled': mode == 'guidance',
        'gravity_hold_zero_offset_rad': config['model_zero_offset_rad'],
        'gravity_hold_torque_limit': config['torque_limits'],
    }])
    nodes = [driver,
        RegisterEventHandler(OnProcessExit(target_action=driver, on_exit=[
            EmitEvent(event=Shutdown(reason='real hardware driver exited'))])),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen', parameters=[{
                 'robot_description': ET.tostring(root, encoding='unicode'),
                 'publish_frequency': 100.0, 'use_sim_time': False}]),
        Node(package='rviz2', executable='rviz2', name='arm6_familiarization_rviz',
             arguments=['-d', str(share / 'arm6_familiarization.rviz')],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]
    if mode == 'guidance' and LaunchConfiguration('start_guidance').perform(context) == 'true':
        nodes.append(Node(package='manipulator', executable='manual_guidance.py',
                          name='arm6_manual_guidance', output='screen', parameters=[{
                              **config, 'urdf_path': str(urdf), 'hardware_enabled': True,
                              'manual_push_test': manual_flag}]))
    # Guidance exit deliberately leaves the driver's timeout hold running.
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('manipulator'))
    return LaunchDescription([
        DeclareLaunchArgument('port_name', description='Verified real-arm serial path; no guessed default'),
        DeclareLaunchArgument('mode', default_value='readonly', choices=['readonly', 'hold', 'guidance'],
                              description='readonly=no motor writes; hold=PD hold; guidance=active gravity/hand guiding'),
        DeclareLaunchArgument('config', default_value=str(share / 'arm6_familiarization.yaml')),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('manual_push_test', default_value='false', choices=['true','false'],
                              description='Capability only; explicit service and 3 stable handovers still required'),
        DeclareLaunchArgument('start_guidance', default_value='true', choices=['true','false'],
                              description='Allow repeated guidance trials with one persistent hardware driver'),
        DeclareLaunchArgument('hold_command_file', default_value='',
                              description='Fresh verified existing command for driver restart; never guessed from q'),
        DeclareLaunchArgument('supported_startup_file', default_value='',
                              description='Fresh stationary feedback for supported cold-start HOLD; not command readback'),
        DeclareLaunchArgument('external_support_confirmed', default_value='false', choices=['true','false'],
                              description='Required only for supported cold start; support stays until HOLD verified'),
        OpaqueFunction(function=_nodes),
    ])
