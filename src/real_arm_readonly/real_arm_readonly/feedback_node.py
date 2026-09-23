"""Encoder telemetry only. No driver imports, control interfaces or subscriptions."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .protocol import Decoder, JointMapping, NAMES
from .receiver import Receiver


class FeedbackNode(Node):
    def __init__(self):
        super().__init__('real_arm_readonly_feedback', start_parameter_services=False)
        self.declare_parameter('port', '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
        self.declare_parameter('rx_isolated', False)
        self.declare_parameter('feedback_timeout_sec', 0.25)
        self.timeout = float(self.get_parameter('feedback_timeout_sec').value)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('feedback_timeout_sec must be finite and positive')
        # All validation occurs before opening the real port.
        self.receiver = Receiver(self.get_parameter('port').value,
                                 self.get_parameter('rx_isolated').value is True)
        self.decoder, self.mapping = Decoder(), JointMapping()
        self.pub = self.create_publisher(JointState, '/real_arm/raw_joint_states', qos_profile_sensor_data)
        self.diag = self.create_publisher(String, '/real_arm/feedback_status', 10)
        self.last_position = self.last_velocity = None
        self.velocity = []
        self.count = self.previous_count = 0
        self.previous_status = time.monotonic()
        self.fault = None
        self.poll_timer = self.create_timer(0.002, self.poll)
        self.status_timer = self.create_timer(1.0, self.status)

    def poll(self):
        if self.fault:
            return
        try:
            # Bounded drain; publish each valid position frame once, never cached q.
            for _ in range(16):
                chunk = self.receiver.read()
                if not chunk:
                    break
                for cmd, values in self.decoder.feed(chunk):
                    now = time.monotonic()
                    if cmd == 0x21:
                        self.velocity = self.mapping.velocity(values)
                        self.last_velocity = now
                    elif cmd == 0x20:
                        # Re-anchor on a gap: unwrap turns cannot be inferred across it.
                        if self.last_position is not None and now-self.last_position > self.timeout:
                            self.mapping.previous = None
                        msg = JointState()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = 'base_link'
                        msg.name = list(NAMES)
                        msg.position = self.mapping.position(values)
                        # Separate wire frames: velocity is optional and age-bounded.
                        if self.last_velocity is not None and now-self.last_velocity <= 0.025:
                            msg.velocity = self.velocity.copy()
                        msg.effort = []  # current units unverified; no false Nm claim
                        self.pub.publish(msg)
                        self.last_position = now
                        self.count += 1
        except OSError as exc:
            self.fault = str(exc)
            self.receiver.close()
            self.get_logger().error(f'Receive stopped; no reconnect or commands: {exc}')

    def status(self):
        now = time.monotonic()
        age = None if self.last_position is None else now-self.last_position
        msg = String()
        msg.data = json.dumps(dict(
            state='ERROR' if self.fault else ('STALE' if age is None or age > self.timeout else 'OK'),
            position_age_sec=age, position_frames=self.count,
            position_rate_hz=(self.count-self.previous_count)/(now-self.previous_status),
            bad_checksum=self.decoder.bad_checksum, bad_length=self.decoder.bad_length,
            nonfinite=self.decoder.nonfinite, error=self.fault,
            coordinates='encoder, gamma6 signs, no zero offsets',
            timestamp='host receive time; no firmware acquisition timestamp',
        ), allow_nan=False)
        self.diag.publish(msg)
        self.previous_count, self.previous_status = self.count, now

    def destroy_node(self):
        self.receiver.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FeedbackNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
