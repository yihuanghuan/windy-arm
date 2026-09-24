#!/usr/bin/env python3
"""Read-only RViz presentation of measured link6 hold; publishes markers only.

Robot meshes are placed using measured link6 and FK for display, without
publishing world/base TF or feeding any estimate back to the controller.
"""
import json
import time
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np
import pinocchio as pin
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from camera_world_hold_core import default_urdf, ordered_joints, transform


CYAN = (0.12, 0.85, 1.0, 1.0)
GOLD = (1.0, 0.70, 0.15, 1.0)
WHITE = (0.88, 0.94, 1.0, 1.0)


def point(p):
    return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))


def pose_tf(m):
    p, q = m.pose.position, m.pose.orientation
    return transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


class HoldView(Node):
    def __init__(self):
        super().__init__('link6_hold_visualizer')
        self.model = pin.buildModelFromUrdf(default_urdf())
        self.data = self.model.createData()
        self.names = list(self.model.names)[1:]
        self.eid = self.model.getFrameId('link6')
        self.visuals = []
        for link in ET.parse(default_urdf()).getroot().findall('link'):
            for v in link.findall('visual'):
                mesh = v.find('geometry/mesh')
                if mesh is None:
                    continue
                o = v.find('origin')
                xyz = np.fromstring(o.get('xyz', '0 0 0'), sep=' ') if o is not None else np.zeros(3)
                rpy = np.fromstring(o.get('rpy', '0 0 0'), sep=' ') if o is not None else np.zeros(3)
                self.visuals.append((self.model.getFrameId(link.get('name')),
                                     pin.SE3(pin.rpy.rpyToMatrix(rpy), xyz),
                                     mesh.get('filename')))
        self.q = self.measured = self.desired = self.camera = self.anchor = None
        self.q_time = self.pose_time = self.command_time = self.status_time = -np.inf
        self.status = {}
        self.ee_path, self.base_path = deque(maxlen=1200), deque(maxlen=1200)
        self.active_before = False
        self.pub = self.create_publisher(MarkerArray, '/link6_hold_view/markers', 1)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 1)
        self.create_subscription(PoseStamped, '/camera_world_hold/measured_link6_pose', self.on_pose, 1)
        self.create_subscription(PoseStamped, '/camera_world_hold/desired_pose', self.on_desired, 1)
        self.create_subscription(PoseStamped, '/slam/camera_pose', self.on_camera, 1)
        self.create_subscription(String, '/camera_world_hold/status', self.on_status, 1)
        self.create_subscription(JointState, '/camera_world_hold/actuator_joint_command', self.on_command, 1)
        self.create_service(Trigger, '~/clear_trails', self.clear)
        self.create_timer(.1, self.draw)

    def on_joints(self, m):
        try:
            self.q = ordered_joints(m.name, m.position, self.names)
            self.q_time = time.monotonic()
        except ValueError:
            self.q = None

    def on_pose(self, m):
        if m.header.frame_id == 'orb_map':
            self.measured, self.pose_time = pose_tf(m), time.monotonic()
            if self.anchor is None:
                self.anchor = self.measured.translation.copy()

    def on_desired(self, m):
        if m.header.frame_id == 'orb_map':
            self.desired = pose_tf(m)

    def on_camera(self, m):
        if m.header.frame_id == 'orb_map':
            self.camera = pose_tf(m)

    def on_status(self, m):
        try:
            self.status = json.loads(m.data)
            self.status_time = time.monotonic()
        except ValueError:
            self.status = {}

    def on_command(self, _):
        self.command_time = time.monotonic()

    def clear(self, _, response):
        self.ee_path.clear()
        self.base_path.clear()
        response.success, response.message = True, 'Display trails cleared; controller unchanged'
        return response

    def marker(self, ns, kind, color=WHITE):
        m = Marker()
        m.header.frame_id = 'orb_map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type = ns, len(self.markers), kind
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = color
        m.lifetime.sec = 1
        self.markers.append(m)
        return m

    def place(self, m, t):
        m.pose.position = point(t.translation)
        q = pin.Quaternion(t.rotation).coeffs()
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = map(float, q)

    def line(self, name, points, color, width=.004, kind=Marker.LINE_STRIP):
        m = self.marker(name, kind, color)
        m.scale.x = width
        m.points = [point(p) for p in points]
        return m

    def label(self, name, text, p, size=.027, color=WHITE):
        m = self.marker(name, Marker.TEXT_VIEW_FACING, color)
        m.pose.position, m.scale.z, m.text = point(p), size, text.replace(' ', '_')

    def axes(self, name, t, length, width):
        for k, c in enumerate(((1., .2, .24, 1.), (.22, 1., .45, 1.), (.26, .55, 1., 1.))):
            m = self.marker(name, Marker.ARROW, c)
            m.points = [point(t.translation), point(t.translation + t.rotation[:, k]*length)]
            m.scale.x, m.scale.y, m.scale.z = width, 2*width, 2.5*width

    def draw(self):
        self.markers = []
        now = time.monotonic()
        active = (self.status.get('state') == 'HOLD' and now-self.status_time < 2.
                  and now-self.command_time < .25)
        fresh = now-self.pose_time < .25 and now-self.q_time < .25
        anchor = np.zeros(3) if self.anchor is None else self.anchor
        if active and not self.active_before:
            self.ee_path.clear()
            self.base_path.clear()
        self.active_before = active
        mode = 'LIVE-COMPENSATION' if active and fresh else 'POSITION-HOLD / COMPENSATION-OFF'
        if not fresh:
            mode = 'WAITING / POSE UNAVAILABLE'
        self.label('title', 'LINK6-WORLD-HOLD', anchor + [0., 0., .39], .04)
        self.label('mode', mode, anchor + [0., 0., .33], .025, CYAN if active and fresh else GOLD)
        self.label('legend', 'CYAN:Measured-EE | GOLD:Target+Base-Trail',
                   anchor + [0., 0., .27], .019)
        if self.measured is not None and self.q is not None and fresh:
            pin.framesForwardKinematics(self.model, self.data, self.q)
            wb = self.measured * self.data.oMf[self.eid].inverse()
            for fid, offset, resource in self.visuals:
                m = self.marker('robot', Marker.MESH_RESOURCE, (.60, .70, .80, .92))
                self.place(m, wb * self.data.oMf[fid] * offset)
                m.scale.x = m.scale.y = m.scale.z = 1.
                m.mesh_resource = resource
            self.axes('measured_axes', self.measured, .09, .004)
            dot = self.marker('measured', Marker.SPHERE, CYAN)
            self.place(dot, self.measured)
            dot.scale.x = dot.scale.y = dot.scale.z = .021
            self.label('ee_label', 'MEASURED LINK6', self.measured.translation + [0., 0., .115], .020, CYAN)
            if active:
                self.ee_path.append(self.measured.translation.copy())
                self.base_path.append(wb.translation.copy())
            self.label('base_label', 'MOVING BASE', wb.translation + [0., 0., -.06], .019, GOLD)
            if self.desired is not None and active:
                self.axes('target_axes', self.desired, .13, .002)
                corners = [np.array([x, y, z])*.032 for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
                edges = []
                for a in corners:
                    for b in corners:
                        if np.count_nonzero(a != b) == 1:
                            edges.extend((self.desired.act(a), self.desired.act(b)))
                self.line('target_box', edges, GOLD, .0015, Marker.LINE_LIST)
                self.line('error', [self.measured.translation, self.desired.translation], (1., .25, .4, 1.), .003)
                ep = np.linalg.norm(self.desired.translation-self.measured.translation)*1000
                er = np.linalg.norm(pin.log3(self.desired.rotation @ self.measured.rotation.T))*180/np.pi
                self.label('error_readout', f'Position:{ep:.1f}mm | Rotation:{er:.1f}deg',
                           anchor + [0., 0., .21], .027)
            if self.camera is not None:
                m = self.marker('camera', Marker.CUBE, (.2, .22, .28, 1.))
                self.place(m, self.camera)
                m.scale.x, m.scale.y, m.scale.z = .028, .09, .025
        if len(self.ee_path) > 1:
            self.line('ee_trail', self.ee_path, CYAN, .003)
            self.line('base_trail', self.base_path, GOLD, .004)
        self.pub.publish(MarkerArray(markers=self.markers))


def main():
    rclpy.init()
    node = HoldView()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
