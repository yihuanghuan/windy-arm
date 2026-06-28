#!/usr/bin/env python3
"""Publish zero joint states for the LL100-LH420 display model."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    "rot1_Joint",
    "rot2_Joint",
    "rot3_Joint",
    "rot4_Joint",
    "A_Joint",
    "B_Joint",
    "C_Joint",
    "D_Joint",
    "E_Joint",
    "F_Joint",
]


class LL100LH420ZeroJointStatePublisher(Node):
    def __init__(self):
        super().__init__("ll100_lh420_zero_joint_state_publisher")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(0.1, self.publish_zero_state)
        self.get_logger().info("Publishing zero joint states for LL100-LH420 display model")

    def publish_zero_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [0.0] * len(JOINT_NAMES)
        msg.velocity = [0.0] * len(JOINT_NAMES)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = LL100LH420ZeroJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
