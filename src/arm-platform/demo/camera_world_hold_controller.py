#!/usr/bin/env python3
"""25 Hz link6 world hold from housing-mounted camera + encoders. Shadow by default."""
import csv
import json
from pathlib import Path
import signal
import time

import numpy as np
import pinocchio as pin
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener, TransformException

from camera_world_hold_core import Link6Kinematics, default_urdf, ordered_joints, transform


class CameraWorldHold(Node):
    def __init__(self, **kwargs):
        super().__init__('camera_world_hold', **kwargs)
        defaults = dict(urdf_path=default_urdf(), parent_frame='joint6_housing', world_frame='orb_map',
                        camera_frame='camera_link', left_optical_frame='camera_infra1_optical_frame',
                        joint6_to_left_xyz=[-0.0042, 0.018, 0.037],
                        joint6_to_left_optical_xyzw=[0.0, 0.0, 0.0, 0.0],
                        read_only=False, publish_command=False, control_rate=25.0,
                        kp_position=1.5, kp_rotation=1.5, max_linear_velocity=0.05,
                        max_angular_velocity=0.2, max_joint_velocity=0.3, damping_lambda=0.03,
                        position_deadband=0., rotation_deadband=0.,
                        pose_receive_timeout=0.20, pose_header_max_age=1.0,
                        joint_timeout=0.15, validity_timeout=0.25,
                        log_path='', driver_node='/student_arm_node')
        self.cfg = {k: self.declare_parameter(k, v, ParameterDescriptor(read_only=True)).value
                    for k, v in defaults.items()}
        for key in ('control_rate', 'kp_position', 'kp_rotation', 'max_linear_velocity',
                    'max_angular_velocity', 'max_joint_velocity', 'damping_lambda',
                    'pose_receive_timeout', 'pose_header_max_age', 'joint_timeout', 'validity_timeout'):
            if not np.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(f'{key} must be positive and finite')
        if self.cfg['read_only'] and self.cfg['publish_command']:
            raise ValueError('read_only and publish_command are mutually exclusive')
        for key in ('position_deadband', 'rotation_deadband'):
            if not np.isfinite(self.cfg[key]) or self.cfg[key] < 0:
                raise ValueError(f'{key} must be nonnegative and finite')
        self.kin = Link6Kinematics(self.cfg['urdf_path'], self.cfg['parent_frame'])
        self.samples = {}
        self.desired = None
        self.state = 'WAITING'
        self.reason = 'waiting for input'
        self.mount_reason = 'joint6 -> camera mounting rotation missing'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseStamped, '/slam/camera_pose', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Bool, '/slam/state_valid', self.on_valid, qos_profile_sensor_data)
        self.command_pub = (self.create_publisher(JointState, '/student/joint_command', 1)
                            if self.cfg['publish_command'] else None)
        self.shadow_pub = self.create_publisher(JointState, '~/shadow_joint_command', 1)
        # Explicit dq output for the existing MIT position-hold adapter. This is
        # an integration rate, never motor velocity/torque feedforward.
        self.velocity_pub = self.create_publisher(JointState, '~/joint_velocity', 1)
        self.desired_pub = self.create_publisher(PoseStamped, '~/desired_pose', 1)
        self.measured_pub = self.create_publisher(PoseStamped, '~/measured_link6_pose', 1)
        self.error_pub = self.create_publisher(TwistStamped, '~/pose_error', 1)
        self.twist_pub = self.create_publisher(TwistStamped, '~/command_twist', 1)
        self.status_pub = self.create_publisher(String, '~/status', 1)
        self.create_service(SetBool, '~/enable', self.on_enable)
        self.driver = None
        self.driver_request = None
        self.driver_info = None
        if self.command_pub is not None:
            self.driver = self.create_client(GetParameters, self.cfg['driver_node'] + '/get_parameters')
        self.file = None
        if self.cfg['log_path']:
            path = Path(self.cfg['log_path']).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.file = path.open('x', newline='')
            self.csv = csv.writer(self.file)
            self.csv.writerow(['time', 'state', 'valid', 'pose_sensor_latency', 'pose_receive_gap',
                               'joint_age', 'valid_age'] +
                              [f'{group}_{axis}' for group, axes in (
                                  ('camera', ['x','y','z','qx','qy','qz','qw']),
                                  ('measured', ['x','y','z','qx','qy','qz','qw']),
                                  ('desired', ['x','y','z','qx','qy','qz','qw']),
                                  ('ep', ['x','y','z']), ('er', ['x','y','z']),
                                  ('rpy_error', ['roll','pitch','yaw']),
                                  ('twist', ['vx','vy','vz','wx','wy','wz']),
                                  ('q', self.kin.names), ('dq', self.kin.names),
                                  ('q_cmd', self.kin.names)) for axis in axes] +
                              [f'jacobian_sigma_{i+1}' for i in range(6)] +
                              ['jacobian_rank','jacobian_condition','command_published'])
        self.last_tick = time.monotonic()
        self.last_report = 0.0
        self.create_timer(1.0, self.resolve_geometry)
        self.create_timer(1.0, self.poll_driver)
        self.create_timer(1.0/self.cfg['control_rate'], self.tick)
        self.get_logger().info(f'joints={self.kin.names}; publish_command={self.cfg["publish_command"]}; '
                               'controlled_frame=link6; WAITING/READY never send motion; ~/enable captures link6 target')

    def ros_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def receive(self, key, value, msg=None):
        stamp = None if msg is None else msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples[key] = (value, time.monotonic(), stamp)

    def on_pose(self, msg):
        try:
            if msg.header.frame_id != self.cfg['world_frame']:
                raise ValueError('unexpected camera pose frame')
            p, q = msg.pose.position, msg.pose.orientation
            self.receive('pose', transform([p.x,p.y,p.z], [q.x,q.y,q.z,q.w]), msg)
        except ValueError as error:
            self.samples.pop('pose', None)
            self.stop(str(error))

    def on_joints(self, msg):
        try:
            q = ordered_joints(msg.name, msg.position, self.kin.names)
            self.receive('joints', self.kin.check_q(q), msg)
        except ValueError as error:
            self.samples.pop('joints', None)
            self.stop(str(error))

    def on_valid(self, msg):
        self.receive('valid', msg.data)
        if not msg.data:
            self.stop('SLAM invalid: re-enable manually after recovery')

    def age(self, key):
        if key not in self.samples:
            return float('inf')
        _, received, stamp = self.samples[key]
        receive_age = time.monotonic() - received
        if stamp is None:
            return receive_age
        stamp_age = self.ros_seconds() - stamp
        if stamp <= 0 or stamp_age < -0.05:
            return float('inf')
        return max(receive_age, stamp_age)

    def pose_receive_gap(self):
        return (time.monotonic()-self.samples['pose'][1]
                if 'pose' in self.samples else float('inf'))

    def pose_sensor_latency(self):
        if 'pose' not in self.samples or self.samples['pose'][2] is None or self.samples['pose'][2]<=0:
            return float('inf')
        return self.ros_seconds()-self.samples['pose'][2]

    def readiness(self):
        if self.pose_receive_gap() > self.cfg['pose_receive_timeout']:
            return False, 'pose receive gap exceeded'
        if not -.05 <= self.pose_sensor_latency() <= self.cfg['pose_header_max_age']:
            return False, 'pose header timestamp sanity limit exceeded'
        for key, timeout in (('joints', 'joint_timeout'),
                             ('valid', 'validity_timeout')):
            if self.age(key) > self.cfg[timeout]:
                return False, f'{key} missing/stale'
        if not self.samples['valid'][0]:
            return False, 'SLAM invalid'
        return True, 'inputs ready'

    def resolve_geometry(self):
        if self.kin.frame_id is not None:
            return
        try:
            # User translation refers to the left IR origin; rotation must refer
            # explicitly to the LEFT OPTICAL axes. Never assume identity.
            mount = transform(self.cfg['joint6_to_left_xyz'], self.cfg['joint6_to_left_optical_xyzw'])
            tf = self.tf_buffer.lookup_transform(self.cfg['left_optical_frame'],
                                                 self.cfg['camera_frame'], rclpy.time.Time())
            t, q = tf.transform.translation, tf.transform.rotation
            camera_mount = mount * transform([t.x,t.y,t.z], [q.x,q.y,q.z,q.w])
            self.kin.set_camera(camera_mount)
            self.mount_reason = 'camera fixed frame ready'
            self.get_logger().info(f'T_{self.cfg["parent_frame"]}_camera=\n{camera_mount.homogeneous}')
        except (ValueError, TransformException) as error:
            self.mount_reason = str(error)

    def poll_driver(self):
        if self.driver is None or not self.driver.service_is_ready():
            return
        if self.driver_request is not None:
            if not self.driver_request.done():
                return
            try:
                v = self.driver_request.result().values
                self.driver_info = (time.monotonic(),
                                    v[0].bool_value or v[1].bool_value or v[2].bool_value,
                                    len(v[3].double_array_value), v[4].double_value)
            except Exception as error:
                self.driver_info = None
                self.get_logger().warning(f'Cannot inspect position interface: {error}')
        self.driver_request = self.driver.call_async(GetParameters.Request(names=[
            'read_only', 'familiarization_mode', 'enable_direct_effort_command',
            'joint_lower_limits', 'max_velocity']))

    def driver_ready(self):
        if self.command_pub is None:
            return True
        info = self.driver_info
        return (info is not None and time.monotonic()-info[0] < 3.0 and not info[1]
                and info[2] == self.kin.model.nq and self.cfg['max_joint_velocity'] <= info[3])

    def on_enable(self, request, response):
        if not request.data:
            self.stop('disabled by operator')
            response.success, response.message = True, 'HOLD disabled; joint hold latched if previously active'
            return response
        ready, reason = self.readiness()
        if self.desired is not None:
            response.success, response.message = True, 'already HOLD; target unchanged'
        elif self.cfg['read_only'] or not ready or self.kin.frame_id is None or not self.driver_ready():
            response.success = False
            response.message = ('read_only mode' if self.cfg['read_only'] else reason if not ready
                                else self.mount_reason if self.kin.frame_id is None
                                else 'driver must use matching position mode (not guidance/direct effort) and speed limit')
        else:
            self.desired = self.kin.reconstruct(self.samples['joints'][0], self.samples['pose'][0])
            self.last_tick = time.monotonic()
            self.state, self.reason = 'HOLD', 'target captured'
            response.success, response.message = True, 'current reconstructed link6 world pose captured'
        return response

    def joint_message(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position = self.kin.names, q.tolist()
        # Position-only interface. No velocity or acceleration feedforward.
        msg.velocity = [0.0] * len(q)
        return msg

    def stop(self, reason):
        if self.desired is not None:
            if self.command_pub is not None and self.age('joints') <= self.cfg['joint_timeout']:
                self.command_pub.publish(self.joint_message(self.samples['joints'][0]))
            self.get_logger().warning(reason)
        self.desired = None
        self.state, self.reason = 'WAITING', reason

    def pose_message(self, pose):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cfg['world_frame']
        p, q = pose.translation, pin.Quaternion(pose.rotation).coeffs()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p.tolist()
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = q.tolist()
        return msg

    def publish_twist(self, publisher, values):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cfg['world_frame']
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = values[:3].tolist()
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = values[3:].tolist()
        publisher.publish(msg)

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = now-self.last_tick, now
        ready, reason = self.readiness()
        result = None
        measured = None
        published = False
        if ready and self.kin.frame_id is not None:
            measured = self.kin.reconstruct(self.samples['joints'][0], self.samples['pose'][0])
            self.measured_pub.publish(self.pose_message(measured))
        if not ready:
            self.stop(reason)
        elif self.desired is None:
            self.state, self.reason = 'READY', self.mount_reason
        else:
            try:
                if not self.driver_ready():
                    raise ValueError('position driver unavailable or incompatible')
                result = self.kin.step(self.samples['joints'][0], measured,
                                       self.desired, min(dt, 1.0/self.cfg['control_rate']),
                                       **{key: self.cfg[key] for key in (
                                           'kp_position','kp_rotation','max_linear_velocity',
                                           'max_angular_velocity','max_joint_velocity','damping_lambda',
                                           'position_deadband','rotation_deadband')})
                self.desired_pub.publish(self.pose_message(self.desired))
                self.publish_twist(self.error_pub, np.r_[result['position_error'], result['rotation_error']])
                self.publish_twist(self.twist_pub, result['twist'])
                msg = self.joint_message(result['q_cmd'])
                self.shadow_pub.publish(msg)
                rate = self.joint_message(self.samples['joints'][0])
                rate.velocity = result['dq'].tolist()
                self.velocity_pub.publish(rate)
                if self.command_pub is not None:
                    self.command_pub.publish(msg)
                    published = True
            except (ValueError, np.linalg.LinAlgError) as error:
                self.stop(str(error))
        if self.file:
            def pose_values(pose):
                return ([float('nan')]*7 if pose is None else
                        np.r_[pose.translation, pin.Quaternion(pose.rotation).coeffs()].tolist())
            row = [self.ros_seconds(), self.state, self.samples.get('valid', (False,))[0],
                   self.pose_sensor_latency(), self.pose_receive_gap(), self.age('joints'), self.age('valid')]
            row += pose_values(self.samples.get('pose', (None,))[0]) + pose_values(measured) + pose_values(self.desired)
            for key, size in (('position_error',3),('rotation_error',3),('rpy_error',3),('twist',6)):
                row += ([float('nan')]*size if result is None else result[key].tolist())
            row += self.samples.get('joints', (np.full(self.kin.model.nq, np.nan),))[0].tolist()
            for key in ('dq','q_cmd'):
                row += ([float('nan')]*self.kin.model.nq if result is None else result[key].tolist())
            row += ([float('nan')]*6 if result is None else result['jacobian_singular_values'].tolist())
            row += ([float('nan')]*2 if result is None else
                    [int(result['jacobian_rank']),float(result['jacobian_condition'])])
            self.csv.writerow(row + [published])
        if now-self.last_report >= 1.0:
            self.last_report = now
            status = dict(state=self.state, reason=self.reason, geometry=self.mount_reason,
                          controlled_frame='link6',
                          pose_sensor_latency=self.pose_sensor_latency(),
                          pose_receive_gap=self.pose_receive_gap(),
                          slam_valid=self.samples.get('valid', (False,))[0],
                          ages={k: self.age(k) for k in ('pose','joints','valid')},
                          q=self.samples.get('joints', (np.array([]),))[0].tolist())
            if 'pose' in self.samples:
                pose = self.samples['pose'][0]
                status.update(camera_position=pose.translation.tolist(),
                              camera_xyzw=pin.Quaternion(pose.rotation).coeffs().tolist())
            if measured is not None:
                pose=measured
                status.update(position=pose.translation.tolist(),
                              xyzw=pin.Quaternion(pose.rotation).coeffs().tolist(),
                              rpy=pin.rpy.matrixToRpy(pose.rotation).tolist())
            if result is not None:
                status.update({k: v.tolist() for k, v in result.items()})
                if not np.isfinite(result['jacobian_condition']):
                    status['jacobian_condition']=None
            message = json.dumps(status)
            self.status_pub.publish(String(data=message))
            self.get_logger().info(message)
            if self.file:
                self.file.flush()

    def close(self):
        self.stop('controller shutdown')
        if self.file:
            self.file.close()
        self.destroy_node()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = CameraWorldHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
