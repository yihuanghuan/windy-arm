"""Exercise the real C++ driver on a pseudo terminal; never opens a real arm.

Run with the built overlay sourced and ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1.
"""
import os
from pathlib import Path
import select
import signal
import struct
import subprocess
import threading
import time
import tty

import numpy as np
import pinocchio as pin
import pytest
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from sensor_msgs.msg import JointState
from dummy_interface.msg import MotorControl
from std_srvs.srv import Trigger

ROOT = Path(__file__).resolve().parents[1]
Q = np.array([-.02, 1.2, -1.2, .1, .1, .1])
SIGNS = np.array([1., 1., 1., -1., -1., -1.])


def frame(cmd, payload):
    data = bytes([0xfe, 0x55, cmd, len(payload)]) + payload
    total = cumulative = 0
    for byte in data:
        total = (total + byte) & 255
        cumulative = (cumulative + total) & 255
    return data + bytes([total, cumulative])


class Rig:
    def __init__(self, tmp_path, readonly, phase_speed=False, bumpless=False, supported=False):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)  # prevent terminal echo before the driver opens it
        self.port = os.ttyname(self.slave)
        assert self.port.startswith('/dev/pts/')
        self.q = Q.copy()
        self.dq = np.zeros(6)
        self.feed = False
        self.positions = True
        self.tx = bytearray()
        self.running = True
        cfg = yaml.safe_load((ROOT / 'config/arm6_familiarization.yaml').read_text())
        params = dict(arm_type='a_l1', arm_version='gamma6', port_name=self.port,
                      motor_config_path=str(ROOT / 'config/motor_config.yaml'),
                      arm_config_path=str(ROOT / 'config/arm_config.yaml'),
                      urdf_path=str(ROOT / 'config/arm6.urdf'),
                      familiarization_mode=True, read_only=readonly,
                      enable_direct_effort_command=True, wait_for_first_effort_command=True,
                      publish_joint_feedback=True, gravity_hold_enabled=not readonly,
                      gravity_hold_zero_offset_rad=[0.] * 6,
                      gravity_hold_torque_limit=cfg['torque_limits'],
                      joint_lower_limits=cfg['joint_lower_limits'],
                      joint_upper_limits=cfg['joint_upper_limits'],
                      p_gain=cfg['hold_p'], d_gain=cfg['hold_d'])
        if phase_speed:
            params.update(phase_speed_protection=True)
            params.update({k: v for k, v in cfg.items() if k.startswith(('guide_', 'transition_'))})
        if bumpless:
            params.update(bumpless_handover=True, initial_hold_position=Q.tolist(),
                          initial_hold_current=[.1]*6, initial_hold_p=list(cfg['hold_p']),
                          initial_hold_d=list(cfg['hold_d']), initial_hold_expected_q=Q.tolist(),
                          manual_push_test=True,joint_limit_margin=cfg['joint_limit_margin'])
            params.update({k:v for k,v in cfg.items() if k.startswith('no_touch_') or k=='required_handover_trials'})
        if supported:
            from task_space_smc.core.manual_guidance import Guidance
            from task_space_smc.core.supported_startup import propose_supported_hold
            samples=[dict(t=i*.01,q=Q.tolist(),dq=[0.]*6,effort=[.2,-1.6,.8,.01,.01,.01]) for i in range(201)]
            proposal=propose_supported_hold(samples,Guidance(ROOT/'config/arm6.urdf',cfg))
            for k in ('position','current','p','d'):params['initial_hold_'+k]=proposal['command'][k]
            params.update(initial_hold_position_tolerance=.002,initial_hold_is_support_estimate=True)
        path = tmp_path / 'params.yaml'
        path.write_text(yaml.safe_dump({'student_arm_node': {'ros__parameters': params}}))
        self.log = (tmp_path / 'driver.log').open('w')
        exe = Path(get_package_prefix('manipulator')) / 'lib/manipulator/student_arm_node'
        self.proc = subprocess.Popen([str(exe), '--ros-args', '--params-file', str(path)],
                                     stdout=self.log, stderr=subprocess.STDOUT)
        self.thread = threading.Thread(target=self.pump, daemon=True)
        self.thread.start()
        self.node = rclpy.create_node('pty_driver_test')
        self.states = []
        self.node.create_subscription(JointState, '/joint_states', self.states.append, 10)
        self.pub = self.node.create_publisher(MotorControl, '/student/joint_effort_command', 1)

    def pump(self):
        while self.running:
            if self.feed:
                packet = frame(1, b'\x1b')
                if self.positions:
                    packet += frame(0x20, struct.pack('<7f', *(SIGNS * self.q), float('nan')))
                packet += frame(0x21, struct.pack('<7f', *(SIGNS * self.dq), 99.))
                packet += frame(0x22, struct.pack('<7f', *([0.] * 6), 99.))
                packet += frame(0x24, struct.pack('<7H', *([30] * 6), 1))
                os.write(self.master, packet)
            if select.select([self.master], [], [], .005)[0]:
                self.tx.extend(os.read(self.master, 65536))

    def spin(self, seconds, command=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert self.proc.poll() is None, 'driver exited: ' + self.log.name
            if command is not None:
                command.header.stamp = self.node.get_clock().now().to_msg()
                self.pub.publish(command)
            rclpy.spin_once(self.node, timeout_sec=.005)

    def command(self):
        msg = MotorControl()
        msg.enable = 1
        msg.position = Q.tolist()
        msg.velocity = [0.] * 6
        msg.current = [.1] * 6
        msg.p = [0.] * 6
        msg.d = [.05] * 6
        return msg

    def packets(self):
        raw = bytes(self.tx)
        frames = []
        while raw:
            assert raw[:2] == b'\xfe\xaa', raw[:10]
            length = raw[3] + 6
            if len(raw) < length:
                break
            assert raw[3] == 28
            frames.append((raw[2], struct.unpack('<7f', raw[4:32])))
            raw = raw[length:]
        return frames

    def close(self):
        self.proc.send_signal(signal.SIGINT)
        self.proc.wait(timeout=5)
        self.running = False
        self.thread.join(timeout=1)
        self.node.destroy_node()
        self.log.close()
        os.close(self.master)
        os.close(self.slave)


@pytest.fixture
def rig_factory(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID') == '94'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init()
    rigs = []

    def make(readonly, phase_speed=False, bumpless=False, supported=False):
        rig = Rig(tmp_path, readonly, phase_speed, bumpless, supported)
        rigs.append(rig)
        return rig
    yield make
    for rig in rigs:
        rig.close()
    rclpy.shutdown()


def test_supported_initial_hold_matches_estimate_on_wire(rig_factory):
    rig=rig_factory(False,phase_speed=True,bumpless=True,supported=True)
    rig.feed=True;rig.spin(.6)
    packets=rig.packets();assert packets
    ref=np.array([v for c,v in packets if c==0x10][0][:6])*SIGNS
    p=np.array([v for c,v in packets if c==0x13][0][:6])
    ff=np.array([v for c,v in packets if c==0x12][0][:6])*SIGNS
    np.testing.assert_allclose(ff,0.)
    np.testing.assert_allclose(p*(ref-Q),[.2,-1.6,.8,.01,.01,.01],atol=2e-6)


def test_supported_initial_hold_rejects_moved_pose_and_latches(rig_factory):
    rig=rig_factory(False,phase_speed=True,bumpless=True,supported=True)
    rig.q[1]+=.003;rig.feed=True;rig.spin(.6)
    assert not rig.tx
    rig.q=Q.copy();rig.spin(.2)
    assert not rig.tx


def test_readonly_real_driver_publishes_six_and_writes_nothing(rig_factory):
    rig = rig_factory(True)
    rig.spin(.4)
    assert not rig.states  # startup zero cache must not masquerade as real feedback
    assert not rig.tx
    rig.feed = True
    rig.spin(.5, rig.command())
    assert len(rig.states) > 10
    msg = rig.states[-1]
    assert msg.name == [f'joint{i}' for i in range(1, 7)]
    assert len(msg.position) == len(msg.velocity) == len(msg.effort) == 6
    np.testing.assert_allclose(msg.position, Q, atol=1e-6)
    assert not rig.tx  # even an active MotorControl publisher cannot write in readonly
    rig.positions = False
    rig.spin(.15)
    count = len(rig.states)
    rig.spin(.1)
    assert len(rig.states) == count  # heartbeat/velocity cannot renew old positions
    assert not rig.tx


def test_strict_commands_signs_timeout_hold_and_no_joint7(rig_factory):
    rig = rig_factory(False)
    rig.feed = True
    rig.spin(.5)
    assert len(rig.states) > 10 and not rig.tx
    bad = rig.command()
    bad.position.append(0.)
    rig.spin(.1, bad)
    assert not rig.tx
    bad = rig.command()
    bad.current[0] = float('nan')
    rig.spin(.1, bad)
    assert not rig.tx
    rig.spin(.15, rig.command())
    packets = rig.packets()
    assert packets
    for cmd, values in packets:
        assert values[6] == 0.  # wire padding only; no seventh MotorControl entry
        if cmd == 0x12:
            np.testing.assert_allclose(values[:6], SIGNS * .1, atol=1e-6)
        if cmd == 0x10:
            np.testing.assert_allclose(values[:6], SIGNS * Q, atol=1e-6)
    rig.spin(.15)  # producer loss → latched hold
    packets = rig.packets()
    kp = [v for c, v in packets if c == 0x13][-1]
    assert all(v > 0 for v in kp[:6])
    target = [v for c, v in packets if c == 0x10][-1]
    rig.q += .01
    rig.spin(.1, rig.command())  # timeout may not silently re-enable guidance
    packets = rig.packets()
    assert [v for c, v in packets if c == 0x10][-1] == target
    assert [v for c, v in packets if c == 0x13][-1] == kp


def test_heartbeat_with_stale_position_blocks_active_gravity(rig_factory):
    rig = rig_factory(False)
    rig.feed = True
    rig.spin(.5)
    rig.spin(.1, rig.command())
    rig.positions = False
    rig.spin(.15, rig.command())
    packets = rig.packets()
    assert [v for c, v in packets if c == 0x12][-1] == (0.,) * 7
    assert all(v > 0 for v in [v for c, v in packets if c == 0x13][-1][:6])


def test_overspeed_keeps_other_axes_support_without_rearming(rig_factory):
    rig = rig_factory(False)
    rig.feed = True
    rig.spin(.5)
    rig.spin(.1, rig.command())
    rig.dq[4] = .6
    rig.spin(.1, rig.command())
    expected = SIGNS * np.array([.1, .1, .1, .1, 0., .1])
    packets = rig.packets()
    np.testing.assert_allclose([v for c, v in packets if c == 0x12][-1][:6], expected)
    assert all(v > 0 for v in [v for c, v in packets if c == 0x13][-1][:6])
    rig.dq[4] = 0.
    rig.spin(.1, rig.command())
    np.testing.assert_allclose([v for c, v in rig.packets() if c == 0x12][-1][:6], expected)
    # Later feedback loss invalidates even previously retained support, permanently.
    rig.positions = False
    rig.spin(.1)
    assert [v for c, v in rig.packets() if c == 0x12][-1] == (0.,) * 7
    rig.positions = True
    rig.spin(.1)
    assert [v for c, v in rig.packets() if c == 0x12][-1] == (0.,) * 7


def test_installed_guidance_reaches_zero_kp_and_operator_stop_holds(rig_factory, tmp_path):
    rig = rig_factory(False, phase_speed=True, bumpless=True)
    rig.feed = True
    rig.spin(.5)
    rig.node.destroy_publisher(rig.pub)
    cfg = yaml.safe_load((ROOT / 'config/arm6_familiarization.yaml').read_text())
    cfg.update(urdf_path=str(ROOT / 'config/arm6.urdf'), hardware_enabled=True)
    path = tmp_path / 'guidance.yaml'
    path.write_text(yaml.safe_dump({'arm6_manual_guidance': {'ros__parameters': cfg}}))
    exe = Path(get_package_prefix('manipulator')) / 'lib/manipulator/manual_guidance.py'
    with (tmp_path / 'guidance.log').open('w') as log:
        proc = subprocess.Popen([str(exe), '--ros-args', '--params-file', str(path)],
                                stdout=log, stderr=subprocess.STDOUT)
        try:
            rig.spin(7.5)
            assert proc.poll() is None
            packets = rig.packets()
            kp = [v for c, v in packets if c == 0x13][-1]
            assert kp == (0.,) * 7, (tmp_path / 'guidance.log').read_text()
            kd = [v for c, v in packets if c == 0x14][-1]
            np.testing.assert_allclose(kd[:6], cfg['damping'], atol=1e-7)
            model = pin.buildModelFromUrdf(cfg['urdf_path'])
            expected_g = pin.computeGeneralizedGravity(model, model.createData(), Q)
            current = [v for c, v in packets if c == 0x12][-1]
            np.testing.assert_allclose(current[:6], SIGNS * expected_g, atol=1e-6)
            rig.dq[4] = .6
            rig.spin(.15)
            assert [v for c,v in rig.packets() if c == 0x13][-1] == (0.,)*7
            assert 'SPEED WARNING joint5' in (tmp_path / 'guidance.log').read_text()
            rig.dq[4] = 0.
            client = rig.node.create_client(Trigger, '/arm6_manual_guidance/stop')
            assert client.wait_for_service(timeout_sec=2.)
            future = client.call_async(Trigger.Request())
            rig.spin(.15)
            assert future.done() and future.result().success
            assert all(v > 0 for v in [v for c, v in rig.packets() if c == 0x13][-1][:6])
        finally:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=5)


@pytest.mark.parametrize('kind', ['sustained', 'instantaneous', 'transition'])
def test_phase_speed_driver_limits_and_selective_fault_hold(rig_factory, kind):
    rig = rig_factory(False, phase_speed=True)
    rig.feed = True
    cfg = yaml.safe_load((ROOT / 'config/arm6_familiarization.yaml').read_text())
    sustained_test_speed = cfg['guide_sustained_abort_speed'] + .05
    rig.spin(.5)
    cmd = rig.command()
    if kind == 'transition': cmd.p = [30.] * 3 + [5.] * 3
    rig.spin(.1, cmd)
    if kind != 'transition':
        rig.dq[4] = .6
        rig.spin(.12, cmd)
        assert [v for c,v in rig.packets() if c == 0x13][-1] == (0.,)*7
        rig.dq[4] = sustained_test_speed
        rig.spin(.03, cmd)
        assert [v for c,v in rig.packets() if c == 0x13][-1] == (0.,)*7
        rig.dq[4] = .7
        rig.spin(.05, cmd)
        rig.dq[4] = sustained_test_speed
        rig.spin(.03, cmd)
        assert [v for c,v in rig.packets() if c == 0x13][-1] == (0.,)*7
    rig.dq[4] = dict(sustained=sustained_test_speed, instantaneous=1.25, transition=.51)[kind]
    rig.spin(.13, cmd)
    expected = SIGNS * np.array([.1, .1, .1, .1, 0., .1])
    np.testing.assert_allclose([v for c,v in rig.packets() if c == 0x12][-1][:6], expected)
    assert all(x > 0 for x in [v for c,v in rig.packets() if c == 0x13][-1][:6])
    rig.dq[:] = 0.
    rig.spin(.1, cmd)
    np.testing.assert_allclose([v for c,v in rig.packets() if c == 0x12][-1][:6], expected)
    assert all(x > 0 for x in [v for c,v in rig.packets() if c == 0x13][-1][:6])


def test_python_fault_axis_notification_survives_feedback_race(rig_factory):
    from std_msgs.msg import UInt8
    rig = rig_factory(False, phase_speed=True)
    rig.feed = True
    pub = rig.node.create_publisher(UInt8, '/arm6_manual_guidance/speed_fault_axes', 10)
    rig.spin(.5)
    rig.spin(.1, rig.command())
    # Driver already has a newer, slow sample when Python reports an older fault.
    pub.publish(UInt8(data=1 << 4))
    rig.spin(.15, rig.command())
    expected = SIGNS * np.array([.1,.1,.1,.1,0.,.1])
    np.testing.assert_allclose([v for c,v in rig.packets() if c == 0x12][-1][:6], expected)
    assert all(x > 0 for x in [v for c,v in rig.packets() if c == 0x13][-1][:6])


def test_authoritative_first_command_and_manual_gate(rig_factory):
    rig=rig_factory(False,phase_speed=True,bumpless=True)
    rig.feed=True;rig.spin(.5)
    assert rig.tx
    manual=rig.node.create_client(Trigger,'/student_arm_node/begin_manual_push')
    assert manual.wait_for_service(timeout_sec=2.)
    future=manual.call_async(Trigger.Request());rig.spin(.1)
    assert future.done() and not future.result().success
    prepare=rig.node.create_client(Trigger,'/student_arm_node/prepare_guidance')
    assert prepare.wait_for_service(timeout_sec=2.)
    future=prepare.call_async(Trigger.Request());rig.spin(.05)
    assert future.done() and future.result().success
    import json
    seed=json.loads(future.result().message)
    assert seed['p']==[30.,30.,30.,5.,5.,5.]
    # The old dangerous q=current, FF=0 take-over is rejected and cannot rearm.
    bad=rig.command();bad.current=[0.]*6
    rig.spin(.1,bad)
    assert all(v>0 for v in [v for c,v in rig.packets() if c==0x13][-1][:6])
    np.testing.assert_allclose([v for c,v in rig.packets() if c==0x12][-1][:6],SIGNS*.1)
    future=prepare.call_async(Trigger.Request());rig.spin(.05)
    assert future.done() and not future.result().success


def test_three_warm_handovers_unlock_explicit_manual_profile(rig_factory,tmp_path):
    rig=rig_factory(False,phase_speed=True,bumpless=True)
    rig.feed=True;rig.spin(.5);rig.node.destroy_publisher(rig.pub)
    cfg=yaml.safe_load((ROOT/'config/arm6_familiarization.yaml').read_text())
    cfg.update(urdf_path=str(ROOT/'config/arm6.urdf'),hardware_enabled=True,manual_push_test=True)
    path=tmp_path/'warm_guidance.yaml'
    path.write_text(yaml.safe_dump({'arm6_manual_guidance':{'ros__parameters':cfg}}))
    exe=Path(get_package_prefix('manipulator'))/'lib/manipulator/manual_guidance.py'
    stop=rig.node.create_client(Trigger,'/arm6_manual_guidance/stop')
    manual=rig.node.create_client(Trigger,'/arm6_manual_guidance/begin_manual_push')
    for trial in range(3):
        with (tmp_path/f'warm_{trial}.log').open('w') as log:
            proc=subprocess.Popen([str(exe),'--ros-args','--params-file',str(path)],stdout=log,stderr=subprocess.STDOUT)
            try:
                rig.spin(12.)
                assert proc.poll() is None,(tmp_path/f'warm_{trial}.log').read_text()
                assert [v for c,v in rig.packets() if c==0x13][-1]==(0.,)*7
                if trial==2:
                    assert manual.wait_for_service(timeout_sec=2.)
                    f=manual.call_async(Trigger.Request());rig.spin(.15)
                    assert f.done() and f.result().success
                    rig.dq[1]=1.;rig.spin(.2)
                    assert [v for c,v in rig.packets() if c==0x13][-1]==(0.,)*7
                    # New profile permits 1.7 rad/s for longer than 120 ms.
                    rig.dq[1]=1.7;rig.spin(.2)
                    assert [v for c,v in rig.packets() if c==0x13][-1]==(0.,)*7
                    # Wrist-only instantaneous extension still permits only short bursts.
                    rig.dq[3]=2.1;rig.spin(.04)
                    # Clear the synthetic burst before decoding the accumulated TX log;
                    # decoding time must not extend a 40 ms stimulus beyond 120 ms.
                    rig.dq[3]=0.;rig.spin(.05)
                    assert [v for c,v in rig.packets() if c==0x13][-1]==(0.,)*7
                    rig.dq[1]=1.9;rig.spin(.04)
                    rig.dq[1]=1.4;rig.spin(.05)
                    assert [v for c,v in rig.packets() if c==0x13][-1]==(0.,)*7
                    rig.dq[1]=1.9;rig.spin(.2)
                    ff=[v for c,v in rig.packets() if c==0x12][-1]
                    assert ff[1]==0. and abs(ff[2])>.1
                    rig.dq[1]=0.;rig.spin(.1)
                    assert all(v>0 for v in [v for c,v in rig.packets() if c==0x13][-1][:6])
                    assert [v for c,v in rig.packets() if c==0x12][-1][1]==0.
                else:
                    assert stop.wait_for_service(timeout_sec=2.)
                    f=stop.call_async(Trigger.Request());rig.spin(.2)
                    assert f.done() and f.result().success
                    assert all(v>0 for v in [v for c,v in rig.packets() if c==0x13][-1][:6])
            finally:
                proc.send_signal(signal.SIGINT);proc.wait(timeout=5)
        rig.spin(.15)
