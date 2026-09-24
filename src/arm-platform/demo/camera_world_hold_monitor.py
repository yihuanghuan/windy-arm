#!/usr/bin/env python3
"""Read-only integration probe; never creates a robot command publisher."""
import json
import time

import numpy as np
import pinocchio as pin
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class Monitor(Node):
    def __init__(self):
        super().__init__('camera_world_hold_monitor')
        self.samples = {}
        self.create_subscription(PoseStamped, '/slam/camera_pose',
                                 lambda m: self.receive('pose', m), qos_profile_sensor_data)
        self.create_subscription(Bool, '/slam/state_valid',
                                 lambda m: self.receive('valid', m), qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states',
                                 lambda m: self.receive('joints', m), qos_profile_sensor_data)
        self.create_timer(1.0, self.report)

    def receive(self, key, message):
        self.samples[key] = (message, time.monotonic())

    def report(self):
        result = {'receive_age_s': {k: round(time.monotonic()-t, 3)
                                   for k, (_, t) in self.samples.items()},
                  'missing': sorted({'pose', 'valid', 'joints'} - self.samples.keys())}
        now = self.get_clock().now().nanoseconds * 1e-9
        for key in ('pose', 'joints'):
            if key in self.samples:
                msg = self.samples[key][0]
                result[key+'_stamp_age_s'] = now-msg.header.stamp.sec-msg.header.stamp.nanosec*1e-9
        if 'valid' in self.samples:
            result['slam_valid'] = self.samples['valid'][0].data
        if 'joints' in self.samples:
            msg = self.samples['joints'][0]
            result['joint_names'], result['q'] = list(msg.name), list(msg.position)
        if 'pose' in self.samples:
            msg = self.samples['pose'][0]
            p, q = msg.pose.position, msg.pose.orientation
            xyzw = np.array([q.x, q.y, q.z, q.w])
            result.update(frame=msg.header.frame_id, position=[p.x, p.y, p.z], xyzw=xyzw.tolist())
            if np.isfinite(xyzw).all() and np.linalg.norm(xyzw) > 1e-8:
                result['rpy_rad'] = pin.rpy.matrixToRpy(pin.Quaternion(xyzw/np.linalg.norm(xyzw)).matrix()).tolist()
        self.get_logger().info(json.dumps(result))


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
