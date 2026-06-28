import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
import yaml


def generate_launch_description():
    manipulator_pkg_share = FindPackageShare('manipulator')
    manipulator_pkg_path = manipulator_pkg_share.find('manipulator')
    yaml_path = os.path.join(manipulator_pkg_path, 'camera.yaml')
    
    with open(yaml_path, 'r') as f:
        camera_config = yaml.safe_load(f)
    
    camera_params = camera_config['camera']

    gst_command = [
        'gst-launch-1.0',
        'v4l2src', 'device=' + camera_params['device'],
        '!', 'image/jpeg,framerate=' + str(camera_params['framerate']) + '/1,widht=640,height=360',
        '!', 'jpegdec',
        '!', 'nvvidconv',
        '!', 'nvv4l2h264enc', 'bitrate=' + str(camera_params['bitrate']), 'preset-level=1', 'insert-sps-pps=1',
        '!', 'rtph264pay', 'config-interval=1', 'pt=96',
        '!', 'udpsink', 'host=' + camera_params['ip'], 'port=' + str(camera_params['port']), 'sync=false'
    ]

    gst_process = ExecuteProcess(
        cmd=gst_command,
        output='screen',
        shell=True,
        respawn=True,
        respawn_delay=1.0,
    )

    return LaunchDescription([
        gst_process,
    ])
