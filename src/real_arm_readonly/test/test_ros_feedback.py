"""Explicit offline integration: fake bytes -> receiver node -> private TF.

Run ONLY with ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 and
REAL_ARM_OFFLINE_TEST=1. No hardware file is opened; no command is published.
"""
import math
import os
import signal
import subprocess
import time
from unittest.mock import patch

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('REAL_ARM_OFFLINE_TEST') != '1', reason='explicit isolated ROS test only')


def test_feedback_tf_and_stale_behavior(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID') == '173'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import String
    from real_arm_readonly.feedback_node import FeedbackNode
    from test_protocol import frame

    class FakeReceiver:
        def __init__(self, *args):
            self.chunks = []
        def read(self):
            return self.chunks.pop(0) if self.chunks else b''
        def close(self):
            pass

    rclpy.init()
    executor = SingleThreadedExecutor()
    monitor = Node('offline_readonly_monitor')
    received, transforms, statuses = [], [], []
    monitor.create_subscription(JointState, '/real_arm/raw_joint_states', received.append, qos_profile_sensor_data)
    monitor.create_subscription(TFMessage, '/real_arm/tf', transforms.append, 10)
    monitor.create_subscription(String, '/real_arm/feedback_status', statuses.append, 10)
    with patch('real_arm_readonly.feedback_node.Receiver', FakeReceiver):
        feedback = FeedbackNode()
    executor.add_node(monitor)
    executor.add_node(feedback)
    log = (tmp_path/'rsp.log').open('w')
    proc = subprocess.Popen([
        'ros2', 'launch', 'real_arm_readonly', 'real_arm_readonly_rviz.launch.py',
        'start_feedback:=false', 'use_rviz:=false'], stdout=log, stderr=log,
        start_new_session=True)

    def until(predicate, timeout=8):
        end = time.monotonic()+timeout
        while not predicate() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
        assert predicate()

    try:
        until(lambda: monitor.count_subscribers('/real_arm/raw_joint_states') >= 2)
        until(lambda: monitor.count_publishers('/real_arm/tf') == 1)
        for angle in (0.1, 0.6):
            # Allow DDS matching and RSP's default 20 Hz throttle to settle.
            end = time.monotonic() + 0.2
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=0.02)
            feedback.receiver.chunks.append(frame(0x21, [1]*7)+frame(0x20, [angle, 0, 0, -0.4, 0, 0, -6.5]))
            until(lambda: received and abs(received[-1].position[0]-angle) < 1e-6)
            until(lambda: any(t.child_frame_id == 'link1' and abs(t.transform.rotation.x-math.sin(angle/2)) < 1e-6
                              for msg in transforms for t in msg.transforms))
            assert received[-1].name == [f'joint{i}' for i in range(1, 7)]
            assert received[-1].position[3] == pytest.approx(0.4)
            assert list(received[-1].velocity) == [1, 1, 1, -1, -1, -1]
            assert list(received[-1].effort) == []
        until(lambda: statuses and 'STALE' in statuses[-1].data, timeout=3)
        assert len(received) == 2, 'No replaying stale joint states'
        topics = dict(monitor.get_topic_names_and_types())
        assert not any('MotorControl' in t for types in topics.values() for t in types)
        assert '/joint_states' not in topics
        assert '/student/joint_command' not in topics
        assert '/student/joint_effort_command' not in topics
        assert '/tf' not in topics
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=10)
        log.close()
        feedback.destroy_node()
        monitor.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
