#!/usr/bin/env python3
"""Fixed-base hand-eye rotation calibration. ENTER/capture: average last 0.5 s.

No robot commands are published. Keep the base fixed throughout the session.
"""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import select
import sys
import time

import numpy as np
import pinocchio as pin
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import yaml

from camera_world_hold_core import default_urdf, ordered_joints, transform, add_joint6_housing_frame
from camera_mount_calibration_core import mean_rotation, angle_deg, solve_rotation


def save_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def apply_config(path, result):
    if not result['accepted']:
        raise ValueError('rejected calibration cannot update controller configuration')
    original = path.read_text()
    data = yaml.safe_load(original)['camera_world_hold']['ros__parameters']
    if data['parent_frame'] != result['parent_frame'] or data['camera_frame'] != 'camera_link':
        raise ValueError('configuration frames differ from calibration')
    q = result['joint6_to_left_optical_xyzw']
    lines = original.splitlines()
    indices = [i for i,line in enumerate(lines) if line.lstrip().startswith('joint6_to_left_optical_xyzw:')]
    if len(indices) != 1:
        raise ValueError('configuration must contain one mounting quaternion')
    if not path.with_suffix(path.suffix+'.before_calibration').exists():
        path.with_suffix(path.suffix+'.before_calibration').write_text(original)
    i = indices[0]
    lines[i] = '    joint6_to_left_optical_xyzw: ' + json.dumps(q)
    # Preserve all gains, limits, comments unrelated to the calibrated value.
    lines = [line for line in lines if '# UNKNOWN, deliberately invalid.' not in line]
    temporary = path.with_suffix('.tmp')
    temporary.write_text('\n'.join(lines)+'\n')
    temporary.replace(path)


class Calibration(Node):
    def __init__(self, args):
        super().__init__('camera_mount_calibration')
        self.args = args
        self.output = Path(args.output).resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        stored = self.output/'samples.json'
        if stored.exists() and not args.resume:
            raise ValueError('output already contains samples; choose a new session directory')
        if args.resume and not stored.exists():
            raise ValueError('--resume requires existing samples.json')
        self.model = pin.buildModelFromUrdf(args.urdf)
        self.names = list(self.model.names)[1:]
        self.joint_id = self.model.getJointId('joint6')
        self.cfg = yaml.safe_load(Path(args.config).read_text())['camera_world_hold']['ros__parameters']
        parent = self.cfg['parent_frame']
        if parent not in ('link6','joint6_housing') or self.cfg['camera_frame'] != 'camera_link':
            raise ValueError('tool expects link6 or joint6_housing, and camera_link')
        if parent == 'joint6_housing':
            self.frame_id = add_joint6_housing_frame(self.model)
            self.frame_definition = 'joint6 housing/stator: joint6 URDF origin BEFORE joint6 rotation; fixed to its parent joint/link5'
        else:
            self.frame_id = self.model.getFrameId('link6')
            frame = self.model.frames[self.frame_id]
            if frame.parentJoint != self.joint_id or not np.allclose(frame.placement.homogeneous,np.eye(4),atol=1e-12):
                raise ValueError('joint6/link6 transform differs; convert measured translation first')
            self.frame_definition = 'joint6 moving frame == link6; fixed placement identity'
        self.data = self.model.createData()
        self.joints, self.poses = deque(maxlen=500), deque(maxlen=200)
        self.valid = False
        self.valid_received = -np.inf
        self.samples = []
        self.finished = False
        self.last_auto = -np.inf
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer,self)
        self.create_subscription(JointState,'/joint_states',self.on_joints,qos_profile_sensor_data)
        self.create_subscription(PoseStamped,'/slam/camera_pose',self.on_pose,qos_profile_sensor_data)
        self.create_subscription(Bool,'/slam/state_valid',self.on_valid,qos_profile_sensor_data)
        self.create_service(Trigger,'~/capture',self.capture_service)
        self.create_service(Trigger,'~/solve',self.solve_service)
        self.create_service(Trigger,'~/undo',self.undo_service)
        self.create_timer(.25,self.auto_tick)
        self.create_timer(3.,self.status)
        self.last_reason = 'waiting for valid SLAM and joint states'
        self.left_camera = None
        if args.resume:
            previous=json.loads(stored.read_text())
            expected_hash=hashlib.sha256(Path(args.urdf).read_bytes()).hexdigest()
            if (previous.get('parent_frame') != parent or
                    previous['urdf_sha256'] != expected_hash or
                    previous['controlled_frame'] != 'camera_link' or
                    previous['world_frame'] != 'orb_map' or
                    not np.allclose(previous['p_parent_left'],self.cfg['joint6_to_left_xyz'])):
                raise ValueError('resume geometry/model differs from recorded session')
            self.samples=previous['samples']
            matrix=np.asarray(previous['T_left_camera'])
            self.left_camera=pin.SE3(matrix[:3,:3],matrix[:3,3])
            self.get_logger().info(f'Resumed {len(self.samples)} samples; only valid if base and SLAM map remained unchanged')
        self.get_logger().info('BASE MUST STAY FIXED. 6–10 poses, 15–45 deg changes about >=2 axes. '
                               'Keep still >=2 s. ENTER or ~/capture; s or ~/solve; u or ~/undo. '
                               f'exclude_joint5={args.exclude_joint5}')

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def on_valid(self,msg):
        self.valid,self.valid_received = msg.data,time.monotonic()
        if not msg.data:
            self.poses.clear()

    def on_joints(self,msg):
        try:
            q=ordered_joints(msg.name,msg.position,self.names)
            if np.any(q<self.model.lowerPositionLimit) or np.any(q>self.model.upperPositionLimit):
                raise ValueError('joint outside URDF limit')
            stamp=self.stamp(msg)
            if self.joints and stamp<=self.joints[-1][0]:
                self.joints.clear()
            self.joints.append((stamp,time.monotonic(),q))
        except ValueError as e:
            self.joints.clear();self.last_reason=str(e)

    def on_pose(self,msg):
        try:
            if msg.header.frame_id!='orb_map' or not self.valid or time.monotonic()-self.valid_received>.25:
                return
            p,q=msg.pose.position,msg.pose.orientation
            t=transform([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
            stamp=self.stamp(msg)
            if self.poses and stamp<=self.poses[-1][0]:
                self.poses.clear()
            self.poses.append((stamp,time.monotonic(),t))
        except ValueError as e:
            self.poses.clear();self.last_reason=str(e)

    def snapshot(self):
        now=time.monotonic();ros_now=self.get_clock().now().nanoseconds*1e-9
        if not self.valid or now-self.valid_received>.25 or not self.poses or not self.joints:
            raise ValueError('valid pose/joint streams unavailable')
        for rows,limit,header_limit in ((self.poses,.20,1.0),(self.joints,.15,.15)):
            if now-rows[-1][1]>limit or not -.05<=ros_now-rows[-1][0]<=header_limit:
                raise ValueError('pose or joint messages are stale')
        # A brief pause while guiding the arm is not a settled calibration pose.
        # Require a longer quiet interval, then average only its final 0.5 s.
        settle = self.args.settle_seconds
        quiet = [row for row in self.joints if self.poses[-1][0]-settle <= row[0] <= self.poses[-1][0]]
        if len(quiet)<2 or quiet[-1][0]-quiet[0][0]<settle-.05:
            raise ValueError(f'hold still for {settle:.1f} s before capture')
        if np.max(np.ptp([row[2] for row in quiet],axis=0))>np.deg2rad(.5):
            raise ValueError(f'joints have not settled for {settle:.1f} s')
        window=[row for row in self.poses if self.poses[-1][0]-row[0]<=self.args.average_seconds]
        times=np.array([row[0] for row in self.joints]);values=np.array([row[2] for row in self.joints])
        matched=[]
        for stamp,_,t in window:
            nearest=np.min(np.abs(times-stamp))
            if nearest>.03 or stamp<times[0] or stamp>times[-1]:
                continue
            q=np.array([np.interp(stamp,times,values[:,i]) for i in range(self.model.nq)])
            matched.append((stamp,q,t))
        if len(matched)<10 or matched[-1][0]-matched[0][0]<.35:
            raise ValueError('need >=0.35 s/10 time-matched valid camera poses')
        qs=np.array([q for _,q,_ in matched])
        rotations=[t.rotation for _,_,t in matched]
        positions=np.array([t.translation for _,_,t in matched])
        rc=mean_rotation(rotations);pc=np.mean(positions,axis=0)
        qspan=float(np.max(np.ptp(qs,axis=0)))
        rotation_spread=max(angle_deg(r @ rc.T) for r in rotations)
        position_spread=float(np.max(np.linalg.norm(positions-pc,axis=1)))
        if qspan>np.deg2rad(.5) or rotation_spread>.8 or position_spread>.005:
            raise ValueError(f'not stationary: joint span={np.rad2deg(qspan):.2f} deg, camera rotation={rotation_spread:.2f} deg, position={position_spread*1000:.1f} mm')
        robot=[]
        for _,q,_ in matched:
            pin.framesForwardKinematics(self.model,self.data,q)
            robot.append(self.data.oMf[self.frame_id].copy())
        sample=dict(stamp=matched[-1][0],frames=len(matched),window_s=matched[-1][0]-matched[0][0],
                    q=np.mean(qs,axis=0).tolist(),robot_rotation=mean_rotation([t.rotation for t in robot]).tolist(),
                    robot_position=np.mean([t.translation for t in robot],axis=0).tolist(),
                    camera_rotation=rc.tolist(),camera_position=pc.tolist(),
                    joint_span_deg=float(np.rad2deg(qspan)),camera_rotation_spread_deg=rotation_spread,
                    camera_position_spread_m=position_spread)
        return sample

    def capture(self):
        sample=self.snapshot()
        if self.samples:
            if self.args.exclude_joint5:
                q5_values=[s['q'][4] for s in self.samples]+[sample['q'][4]]
                span=float(np.rad2deg(np.ptp(q5_values)))
                if span>self.args.joint5_tolerance_deg:
                    raise ValueError(f'joint5 range {span:.2f} deg exceeds fixed-joint5 tolerance; restore joint5 before capture')
            changes=[angle_deg(np.asarray(s['robot_rotation']).T @ np.asarray(sample['robot_rotation'])) for s in self.samples]
            if min(changes)<10.:
                raise ValueError(f'pose too similar to a previous capture ({min(changes):.1f} deg); change orientation 15–45 deg')
        tf=self.tf_buffer.lookup_transform(self.cfg['left_optical_frame'],'camera_link',rclpy.time.Time())
        t,q=tf.transform.translation,tf.transform.rotation
        tc=transform([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])
        if self.left_camera is not None and not np.allclose(tc.homogeneous,self.left_camera.homogeneous,atol=1e-7):
            raise ValueError('RealSense internal transform changed within session')
        self.left_camera=tc
        self.samples.append(sample)
        self.persist()
        self.get_logger().info(f'CAPTURED {len(self.samples)}/{self.args.target_poses}: {sample["frames"]} frames, '
                               f'rotation spread {sample["camera_rotation_spread_deg"]:.3f} deg; move to next orientation, keep BASE FIXED')
        return f'captured {len(self.samples)}/{self.args.target_poses}'

    def persist(self):
        save_json(self.output/'samples.json',dict(
            frame_definition=self.frame_definition,parent_frame=self.cfg['parent_frame'],
            urdf=str(Path(self.args.urdf).resolve()),urdf_sha256=hashlib.sha256(Path(self.args.urdf).read_bytes()).hexdigest(),
            controlled_frame='camera_link',world_frame='orb_map',
            settle_seconds=self.args.settle_seconds,
            average_seconds=self.args.average_seconds,
            exclude_joint5=self.args.exclude_joint5,
            joint5_tolerance_deg=self.args.joint5_tolerance_deg,
            p_parent_left=self.cfg['joint6_to_left_xyz'],
            T_left_camera=self.left_camera.homogeneous.tolist(),samples=self.samples))

    def solve(self):
        if self.left_camera is None:
            raise ValueError('capture samples first')
        result=solve_rotation(self.samples,self.left_camera,self.cfg['joint6_to_left_xyz'],
                              parent_frame=self.cfg['parent_frame'],min_poses=self.args.target_poses)
        result['exclude_joint5']=self.args.exclude_joint5
        result['joint5_range_deg']=float(np.rad2deg(np.ptp([s['q'][4] for s in self.samples])))
        if self.args.exclude_joint5 and result['joint5_range_deg']>self.args.joint5_tolerance_deg:
            result['accepted']=False
            result['rejection_reasons'].append('joint5 was not held fixed')
        save_json(self.output/'result.json',result)
        for p in result['pairs']:
            self.get_logger().info(f'pair {p["i"]}-{p["j"]}: A={p["robot_angle_deg"]:.1f}deg B={p["camera_angle_deg"]:.1f}deg usable={p["usable"]} residual={p.get("residual_deg",float("nan")):.3f}deg translation={p["translation_error_m"]*1000:.1f}mm')
        self.get_logger().info(json.dumps({k:v for k,v in result.items() if k!='pairs'}))
        if result['accepted']:
            apply_config(Path(self.args.config),result)
            self.finished=True
            return f'ACCEPTED and written to {self.args.config}; mean={result["mean_rotation_residual_deg"]:.3f} deg max={result["max_rotation_residual_deg"]:.3f} deg'
        raise ValueError('REJECTED: '+ '; '.join(result['rejection_reasons']))

    def action(self, function, response):
        try:
            response.message=function();response.success=True
        except Exception as error:
            response.message=str(error);response.success=False
            self.last_reason=str(error)
            self.get_logger().warning(str(error))
        return response

    def capture_service(self,request,response):
        return self.action(self.capture,response)

    def solve_service(self,request,response):
        return self.action(self.solve,response)

    def undo_service(self,request,response):
        if self.samples:
            self.samples.pop();self.persist();self.finished=False
        response.success=True;response.message=f'{len(self.samples)} samples remain'
        return response

    def auto_tick(self):
        if not self.args.auto or self.finished or time.monotonic()-self.last_auto<1.:
            return
        try:
            self.capture();self.last_auto=time.monotonic()
            if len(self.samples)>=self.args.target_poses:
                self.get_logger().info(self.solve())
        except Exception as error:
            self.last_reason=str(error)

    def status(self):
        self.get_logger().info(f'samples={len(self.samples)} valid={self.valid} '
                               f'auto={self.args.auto} finished={self.finished}: {self.last_reason}')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf',default=default_urdf())
    parser.add_argument('--config',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--auto',action='store_true',help='capture distinct stationary poses automatically')
    parser.add_argument('--resume',action='store_true',help='resume saved session ONLY with unchanged fixed base and SLAM map')
    parser.add_argument('--target-poses',type=int,default=10)
    parser.add_argument('--exclude-joint5',action='store_true',help='keep joint5 fixed; excite joints1..4')
    parser.add_argument('--joint5-tolerance-deg',type=float,default=1.)
    parser.add_argument('--settle-seconds',type=float,default=2.,
                        help='required joint quiet interval before the final 0.5 s average')
    parser.add_argument('--average-seconds',type=float,default=.5,
                        help='stationary averaging window; increase to 0.75 if SLAM falls below 20 Hz')
    args=parser.parse_args()
    if not 6<=args.target_poses<=12:
        parser.error('target-poses must be 6..12')
    if not np.isfinite(args.joint5_tolerance_deg) or args.joint5_tolerance_deg<=0:
        parser.error('joint5-tolerance-deg must be positive and finite')
    if not np.isfinite(args.settle_seconds) or not .5<=args.settle_seconds<=3.:
        parser.error('settle-seconds must be finite and in [0.5, 3.0]')
    if not np.isfinite(args.average_seconds) or not .35<=args.average_seconds<=min(1.,args.settle_seconds):
        parser.error('average-seconds must be finite, 0.35..1.0, and no longer than settle-seconds')
    rclpy.init();node=Calibration(args)
    interactive=sys.stdin.isatty()
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.05)
            if interactive and select.select([sys.stdin],[],[],0)[0]:
                command=sys.stdin.readline().strip().lower()
                if command=='q':break
                response=Trigger.Response()
                if command=='s':node.solve_service(None,response)
                elif command=='u':node.undo_service(None,response)
                else:node.capture_service(None,response)
                print(response.message,flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    main()
