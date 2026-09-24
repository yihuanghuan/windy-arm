"""Real ROS transport, synthetic sensors. Run ONLY in an isolated test domain."""
import os
from pathlib import Path
import sys
import time
import signal
import subprocess

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'demo'))
from camera_world_hold_controller import CameraWorldHold


class Rig:
    def __init__(self, publish, tmp_path, mount=True, external_driver=False):
        if os.environ.get('ROS_DOMAIN_ID') != '196' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            pytest.fail('requires ROS_DOMAIN_ID=196 ROS_LOCALHOST_ONLY=1; never test on robot domain')
        rclpy.init()
        self.driver = Node('unused_test_driver' if external_driver else 'student_arm_node')
        for key, value in dict(read_only=False, familiarization_mode=False,
                               enable_direct_effort_command=False, max_velocity=.5,
                               joint_lower_limits=[-4.,-3.14,-1.57,-4.,-2.,-4.]).items():
            self.driver.declare_parameter(key, value)
        parameters = dict(publish_command=publish, log_path=str(tmp_path/'test.csv'))
        if mount:
            # Explicit synthetic mount, never copied to production YAML.
            parameters['joint6_to_left_optical_xyzw'] = [0.,0.,0.,1.]
        self.node = CameraWorldHold(parameter_overrides=[Parameter(k, value=v) for k,v in parameters.items()])
        self.probe = Node('camera_hold_test_sensors')
        self.commands, self.shadows = [], []
        self.probe.create_subscription(JointState, '/student/joint_command', self.commands.append, 10)
        self.probe.create_subscription(JointState, '/camera_world_hold/shadow_joint_command', self.shadows.append, 10)
        self.pose_pub = self.probe.create_publisher(PoseStamped, '/slam/camera_pose', 10)
        self.q_pub = self.probe.create_publisher(JointState, '/joint_states', 10)
        self.valid_pub = self.probe.create_publisher(Bool, '/slam/state_valid', 10)
        self.tf = StaticTransformBroadcaster(self.probe)
        tf = TransformStamped()
        tf.header.stamp = self.probe.get_clock().now().to_msg()
        tf.header.frame_id = 'camera_infra1_optical_frame'
        tf.child_frame_id = 'camera_link'
        tf.transform.rotation.w = 1.
        self.tf.sendTransform(tf)
        self.client = self.probe.create_client(SetBool, '/camera_world_hold/enable')
        self.q = [.2,.6,-.7,.5,.4,-.3]
        self.x, self.valid, self.old_stamp, self.frame = 0., True, False, 'orb_map'
        self.pose_delay=0.
        self.send_pose, self.send_joints, self.send_valid = True, True, True
        if external_driver:
            self.send_joints = False
        self.timer = self.probe.create_timer(.025, self.send)
        self.executor = SingleThreadedExecutor()
        for node in [self.node, self.probe, self.driver]:
            self.executor.add_node(node)

    def send(self):
        stamp = self.probe.get_clock().now().to_msg()
        if self.send_pose:
            m = PoseStamped()
            m.header.stamp = stamp
            if self.pose_delay:
                from rclpy.duration import Duration
                m.header.stamp=(self.probe.get_clock().now()-Duration(seconds=self.pose_delay)).to_msg()
            if self.old_stamp:
                m.header.stamp.sec -= 2
            m.header.frame_id = self.frame
            m.pose.position.x = self.x
            m.pose.orientation.w = 1.
            self.pose_pub.publish(m)
        if self.send_joints:
            m = JointState()
            m.header.stamp = stamp
            m.name = self.node.kin.names[::-1]
            m.position = self.q[::-1]
            self.q_pub.publish(m)
        if self.send_valid:
            self.valid_pub.publish(Bool(data=self.valid))

    def spin(self, seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            self.executor.spin_once(timeout_sec=.01)

    def enable(self, value=True):
        future = self.client.call_async(SetBool.Request(data=value))
        until = time.monotonic()+2
        while not future.done() and time.monotonic()<until:
            self.executor.spin_once(timeout_sec=.01)
        assert future.done()
        return future.result()

    def close(self):
        self.node.close()
        self.probe.destroy_node()
        self.driver.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()


@pytest.mark.parametrize('publish', [False, True])
def test_shadow_or_position_transport_and_loss(tmp_path, publish):
    rig = Rig(publish, tmp_path)
    try:
        rig.spin(2.3)
        assert rig.node.state == 'READY'
        assert not rig.commands and not rig.shadows
        assert rig.enable().success
        rig.spin(.15)
        np.testing.assert_allclose(rig.shadows[-1].position, rig.q, atol=1e-12)
        rig.x = .015
        rig.spin(.15)
        assert rig.node.state == 'HOLD'
        assert np.max(np.abs(np.array(rig.shadows[-1].position)-rig.q)) > 1e-5
        assert max(abs(np.array(rig.shadows[-1].position)-rig.q)) <= .012+1e-10
        assert bool(rig.commands) == publish
        if publish:
            np.testing.assert_allclose(rig.commands[-1].position, rig.shadows[-1].position)
        captured = rig.node.desired.translation.copy()
        assert rig.enable().success  # repeated enable must not reset target
        np.testing.assert_array_equal(rig.node.desired.translation, captured)
        rig.valid = False
        rig.spin(.1)
        assert rig.node.desired is None
        if publish:
            np.testing.assert_allclose(rig.commands[-1].position, rig.q)
        count = len(rig.commands)
        rig.valid = True
        rig.spin(.15)
        assert rig.node.state == 'READY' and rig.node.desired is None
        assert len(rig.commands) == count
        assert rig.enable().success
        rig.spin(.08)
        expected=rig.node.kin.reconstruct(rig.node.samples['joints'][0],rig.node.samples['pose'][0])
        np.testing.assert_allclose(rig.node.desired.homogeneous,expected.homogeneous)
        # Extremely old headers still fail the broad sanity limit (2 s > 1 s).
        rig.old_stamp = True
        rig.spin(.08)
        assert rig.node.desired is None
        assert not rig.enable().success
        rig.old_stamp = False
        rig.spin(.1)
        assert rig.enable().success
        rig.send_valid = False
        rig.spin(.35)
        assert rig.node.desired is None
        rig.send_valid = True
        rig.spin(.1)
        assert rig.enable().success
        rig.send_joints = False
        rig.spin(.25)
        assert rig.node.desired is None
        rig.send_joints = True
        rig.spin(.1)
        assert rig.enable().success
        rig.frame = 'wrong_map'
        rig.spin(.08)
        assert rig.node.desired is None
        rig.frame = 'orb_map'
        rig.spin(.1)
        assert rig.enable().success
        rig.x = float('nan')
        rig.spin(.08)
        assert rig.node.desired is None
        rig.x = .015
        rig.spin(.1)
        assert rig.enable().success
        assert rig.enable(False).success
        assert rig.node.desired is None
        if not publish:
            assert not rig.commands
    finally:
        rig.close()


def test_mount_missing_refuses_hold(tmp_path):
    rig = Rig(False, tmp_path, mount=False)
    try:
        rig.spin(1.2)
        assert rig.node.state == 'READY'
        assert rig.node.kin.frame_id is None
        response = rig.enable()
        assert not response.success and 'quaternion' in response.message
        assert not rig.commands and not rig.shadows
    finally:
        rig.close()


def test_processing_latency_is_not_receive_watchdog(tmp_path):
    rig=Rig(False,tmp_path)
    try:
        rig.pose_delay=.4
        rig.spin(2.3)
        assert rig.enable().success
        rig.spin(.6)
        assert rig.node.state=='HOLD'
        assert rig.node.pose_sensor_latency()>.35
        assert rig.node.pose_receive_gap()<.2
        rig.send_pose=False
        rig.spin(.25)
        assert rig.node.desired is None
        assert not rig.enable().success
        rig.send_pose=True
        rig.spin(.1)
        assert rig.node.state=='READY' and rig.node.desired is None
        assert not rig.commands
    finally:
        rig.close()


def test_existing_cpp_position_interface(tmp_path):
    """Same C++ driver/launch as hardware, with its existing ideal servo backend."""
    assert os.environ.get('ROS_DOMAIN_ID') == '196'
    with (tmp_path/'cpp_driver.log').open('w') as log:
        process = subprocess.Popen([
            'ros2', 'launch', 'manipulator', 'student_arm6.launch.py',
            'arm_type:=sim', 'use_rviz:=false'], stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        rig = None
        try:
            rig = Rig(True, tmp_path, external_driver=True)
            rig.spin(3.0)
            assert process.poll() is None
            assert rig.node.state == 'READY'
            assert not rig.commands
            assert rig.enable().success
            rig.x = .01
            rig.spin(.8)
            assert rig.commands
            actual = rig.node.samples['joints'][0]
            assert np.max(np.abs(actual)) > 1e-4
            assert rig.node.state == 'HOLD'
            assert rig.enable(False).success
        finally:
            if rig is not None:
                rig.close()
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=8)
