#!/usr/bin/env python3
"""Six-axis manual guidance over student_arm_node; never opens a serial device."""
import signal
import json
import sys
import time
from pathlib import Path

# Source-tree execution as well as CMake's lib/manipulator installation.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from dummy_interface.msg import MotorControl
from std_srvs.srv import Trigger
from std_msgs.msg import String, UInt8

from task_space_smc.core.manual_guidance import Guidance, NAMES, vector
from task_space_smc.core.speed_safety import SPEED_KEYS, MANUAL_KEYS


class ManualGuidance(Node):
    def __init__(self):
        super().__init__('arm6_manual_guidance')
        defaults = dict(urdf_path='', hardware_enabled=False,
                        model_zero_offset_rad=[0.] * 6,
                        joint_lower_limits=[-4., -3.1415, -1.5708, -4., -2., -4.],
                        joint_upper_limits=[4., 2., 1.5708, 4., 2., 4.],
                        torque_limits=[9., 9., 9., 1.6, 1.6, 1.6],
                        torque_rate_limits=[3., 3., 3., .5, .5, .5],
                        damping=[.2, .2, .2, .05, .05, .05],
                        hold_p=[30., 30., 30., 5., 5., 5.],
                        hold_d=[1., 1., 1., .1, .1, .1],
                        joint_limit_margin=.10,
                        ramp_duration_sec=3., release_duration_sec=3.,
                        feedback_timeout_sec=.05, rate_hz=100.)
        defaults.update({k: 0. for k in SPEED_KEYS})  # required from shared YAML
        defaults.update({k: [0.]*6 for k in MANUAL_KEYS})
        defaults.update(guide_manual_sustained_duration=0., manual_push_test=False,
                        no_touch_duration_sec=5., no_touch_position_tolerance=.01,
                        no_touch_speed_limit=.2, required_handover_trials=3)
        cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.enabled = cfg['hardware_enabled']
        self.policy = Guidance(cfg['urdf_path'], cfg)
        rate = float(cfg['rate_hz'])
        if not np.isfinite(rate) or not 50 <= rate <= 200:
            raise ValueError('rate_hz must be in [50,200]')
        self.period = 1. / rate
        self.q = self.dq = None
        self.received = None
        self.source_stamp = None
        self.fault = None
        self.sent = False
        self.phase = None
        self.valid_frames = 0
        self.ready = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(MotorControl, '/student/joint_effort_command', qos) if self.enabled else None
        self.speed_pub = self.create_publisher(String, '~/speed_safety', 10)
        self.speed_fault_pub = self.create_publisher(UInt8, '~/speed_fault_axes', 10) if self.enabled else None
        self.last_warning = [False] * 6
        self.diagnostics_pub = self.create_publisher(String, '~/support_diagnostics', 10)
        self.driver_command = None
        self.quiet_since = None
        self.quiet_q = None
        self.quiet_duration = float(cfg['no_touch_duration_sec'])
        self.quiet_tolerance = float(cfg['no_touch_position_tolerance'])
        self.quiet_speed = float(cfg['no_touch_speed_limit'])
        self.hold_client = self.create_client(Trigger, '/student_arm_node/operator_hold')
        self.manual_client = self.create_client(Trigger, '/student_arm_node/begin_manual_push')
        self.create_subscription(MotorControl, '/student/applied_motor_command', self.on_applied, 10)
        self.create_subscription(JointState, '/joint_states', self.on_state, qos)
        self.create_service(Trigger, '~/stop', self.stop)
        self.create_service(Trigger, '~/begin_manual_push', self.begin_manual_push)
        self.create_timer(self.period, self.tick)
        self.get_logger().info('gravity = g_arm6(q + explicit_offset), ratio=1; damping only in MIT Kd')

    def verify_driver(self):
        client = self.create_client(GetParameters, '/student_arm_node/get_parameters')
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError('student_arm_node unavailable')
        request = GetParameters.Request()
        request.names = ['arm_type', 'arm_version', 'read_only',
                         'enable_direct_effort_command', 'familiarization_mode',
                         'gravity_hold_enabled', 'gravity_hold_zero_offset_rad',
                         'gravity_hold_torque_limit', 'p_gain', 'd_gain', 'urdf_path',
                         'phase_speed_protection', *SPEED_KEYS, *MANUAL_KEYS,
                         'guide_manual_sustained_duration', 'bumpless_handover']
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.)
        response = future.result() if future.done() else None
        if response is None or len(response.values) != len(request.names):
            raise RuntimeError('cannot verify hardware driver parameters')
        v = response.values
        if (v[0].string_value != 'a_l1' or v[1].string_value != 'gamma6' or
                not v[4].bool_value or (self.enabled and (v[2].bool_value or not v[3].bool_value))):
            raise RuntimeError('requires familiarization a_l1/gamma6 driver in direct effort mode')
        if self.enabled:
            if not v[11].bool_value:
                raise RuntimeError('driver phase speed protection must be enabled')
            for key, value in zip(SPEED_KEYS, v[12:12+len(SPEED_KEYS)]):
                if value.double_value != self.policy.speed_guard.cfg[key]:
                    raise RuntimeError('driver speed threshold differs: '+key)
            for key, value in zip(MANUAL_KEYS, v[12+len(SPEED_KEYS):]):
                if not np.array_equal(value.double_array_value, self.policy.speed_guard.manual[key]):
                    raise RuntimeError('driver manual speed threshold differs: '+key)
            if v[-2].double_value != self.policy.speed_guard.manual_duration or not v[-1].bool_value:
                raise RuntimeError('driver requires matching manual duration and bumpless handover')
            if not v[5].bool_value or Path(v[10].string_value).resolve() != Path(self.policy.path):
                raise RuntimeError('driver fallback must use the same gravity model')
            for value, expected in zip(v[6:10], [self.policy.offset, self.policy.limit,
                                                 self.policy.hold_p, self.policy.hold_d]):
                if not np.array_equal(value.double_array_value, expected):
                    raise RuntimeError('driver fallback offsets/limits/gains differ from guidance')
            deadline = time.monotonic()+5.
            while self.driver_command is None and time.monotonic()<deadline:
                rclpy.spin_once(self, timeout_sec=.01)
            if self.driver_command is None:
                raise RuntimeError('no authoritative applied command; seed/hold required, refusing startup')
            prepare = self.create_client(Trigger, '/student_arm_node/prepare_guidance')
            if not prepare.wait_for_service(timeout_sec=2.):
                raise RuntimeError('driver handover service unavailable')
            future = prepare.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.)
            response = future.result() if future.done() else None
            if response is None or not response.success:
                raise RuntimeError('handover refused: '+(response.message if response else 'timeout'))
            self.policy.set_hold_command(json.loads(response.message))
        self.ready = True

    def on_applied(self, msg):
        self.driver_command = msg

    def on_state(self, msg):
        if self.fault:
            return
        try:
            if len(msg.name) != 6 or set(msg.name) != set(NAMES):
                raise ValueError('feedback names must be exactly joint1..joint6')
            order = [msg.name.index(name) for name in NAMES]
            q = vector(msg.position, 'position')[order]
            dq = vector(msg.velocity, 'velocity')[order]
            vector(msg.effort, 'effort')
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if age < -.01 or age > self.policy.timeout:
                raise ValueError('old/future JointState stamp')
            if self.source_stamp is not None and stamp <= self.source_stamp:
                raise ValueError('non-increasing JointState stamp')
            self.q, self.dq, self.source_stamp = q, dq, stamp
            self.received = time.monotonic()
            self.valid_frames += 1
        except ValueError as exc:
            self.fail(str(exc))

    def publish(self, values):
        msg = MotorControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name in ('position', 'velocity', 'current', 'p', 'd', 'enable'):
            setattr(msg, name, values[name])
        self.pub.publish(msg)

    def release_to_hold(self):
        if self.sent:
            # enable=0 requests the driver's measured-pose hold; it is NOT disable.
            self.publish(dict(position=self.q.tolist(), velocity=[0.] * 6,
                              current=[0.] * 6, p=[0.] * 6, d=[0.] * 6, enable=0))
            self.sent = False

    def fail(self, reason):
        if self.fault is None:
            self.fault = reason
            axes = self.policy.speed_guard.status.get('fault_axes', [False]*6)
            if self.speed_fault_pub and any(axes):
                self.speed_fault_pub.publish(UInt8(data=sum(1 << i for i, flag in enumerate(axes) if flag)))
            self.get_logger().error('GUIDANCE STOPPED: ' + reason + '; restart required')
            self.release_to_hold()

    def publish_speed_status(self):
        status = self.policy.speed_guard.status
        if not status:
            return
        self.speed_pub.publish(String(data=json.dumps(status)))
        for i, flag in enumerate(status['warning']):
            if flag and not self.last_warning[i]:
                self.get_logger().warning(f'SPEED WARNING joint{i+1} phase={status["phase"]} dq={status["dq"][i]:.6f}')
        self.last_warning = status['warning']

    def stop(self, request, response):
        if self.fault is None and self.sent and self.hold_client.service_is_ready():
            # Explicit healthy stop can start a later operator-authorized trial.
            # Safety faults still use the irreversible/latching enable=0 path.
            self.fault = 'operator stop'
            future = self.hold_client.call_async(Trigger.Request())
            def completed(f):
                result = f.result()
                if result is None or not result.success:
                    self.release_to_hold()
                    self.get_logger().error('operator hold failed; fault hold requested')
                else:
                    self.sent = False
                    self.get_logger().info('HOLD: operator stop acknowledged')
            future.add_done_callback(completed)
        else:
            self.fail('operator stop')
        response.success = True
        response.message = 'guidance stopped; driver hold requested, not hardware disable'
        return response

    def begin_manual_push(self, request, response):
        if (self.fault or not self.get_parameter('manual_push_test').value or
                self.phase != 'GUIDE' or self.quiet_since is None or
                time.monotonic()-self.quiet_since < self.quiet_duration or
                not self.manual_client.service_is_ready()):
            response.message = 'requires explicit flag and stable no-touch GUIDE'
            return response
        future = self.manual_client.call_async(Trigger.Request())
        def completed(f):
            result = f.result()
            if result and result.success:
                self.policy.enable_manual_push()
                self.get_logger().info('MANUAL_PUSH: operator request acknowledged by driver')
            else:
                self.get_logger().warning('manual push refused: '+(result.message if result else 'no response'))
        future.add_done_callback(completed)
        response.success = True
        response.message = 'request submitted; wait for ACTIVE MANUAL_PUSH before touching'
        return response

    def tick(self):
        if self.fault or not self.ready or self.valid_frames < 3:
            return
        try:
            if self.count_publishers('/joint_states') != 1:
                raise ValueError('expected one real JointState publisher')
            if self.enabled and self.count_publishers('/student/joint_effort_command') != 1:
                raise ValueError('competing effort publisher')
            stamp_age = self.get_clock().now().nanoseconds * 1e-9 - self.source_stamp
            values = self.policy.command(self.q, self.dq, time.monotonic(),
                                         max(stamp_age, time.monotonic() - self.received))
            self.publish_speed_status()
            self.diagnostics_pub.publish(String(data=json.dumps(values)))
            quiet = values['phase'] == 'GUIDE' and np.max(abs(self.dq)) <= self.quiet_speed
            if not quiet:
                self.quiet_since = None
            elif self.quiet_since is None or np.max(abs(self.q-self.quiet_q)) > self.quiet_tolerance:
                self.quiet_since, self.quiet_q = time.monotonic(), self.q.copy()
            if values['phase'] != self.phase:
                self.phase = values['phase']
                self.get_logger().info(('ACTIVE ' if self.enabled else 'PREVIEW ') + self.phase)
            if self.enabled:
                self.publish(values)
                self.sent = True
        except (ValueError, RuntimeError) as exc:
            self.publish_speed_status()
            self.fail(str(exc))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = False

    def interrupt(signum, frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = ManualGuidance()
        node.verify_driver()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.02)
    finally:
        if node is not None:
            node.release_to_hold()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
