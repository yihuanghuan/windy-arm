"""OFFLINE command preview only. No ROS, serial, sockets, or actuator imports.

Run with python -m task_space_smc.core.manual_guidance_preview --q <six angles>.
The printed MotorControl dictionary and float32 arrays are NOT transmitted.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct

import numpy as np
import pinocchio as pin

from .gravity_semantics import urdf_feedforward

NAMES = [f'joint{i}' for i in range(1, 7)]
SIGNS = np.array([1, 1, 1, -1, -1, -1])
RATED = np.array([9., 9., 9., 1.6, 1.6, 1.6])
PREVIEW_FF_LIMIT = 0.5 * RATED  # proposed first-test software envelope, not board PD limit
LOWER = np.array([-4., -3.1415, -1.5708, -4., -2., -4.])
UPPER = np.array([4., 2., 1.5708, 4., 2., 4.])
KP = [30., 30., 30., 5., 5., 5.]
KD = [1.5, 1.5, 1.5, 1., 1., 1.]


class UrdfGravity:
    def __init__(self, path):
        self.model = pin.buildModelFromUrdf(str(path))
        if self.model.nq != 6 or self.model.nv != 6 or list(self.model.names)[1:] != NAMES:
            raise ValueError('Preview requires verified joint1..joint6 fixed-base URDF')
        self.data = self.model.createData()

    def gravity(self, q):
        return pin.computeGeneralizedGravity(self.model, self.data, q).copy()


def preview(q, urdf=None):
    q = np.asarray(q, dtype=float)
    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise ValueError('q must have six finite values')
    path = Path(urdf) if urdf else Path(__file__).resolve().parents[2] / 'config/arm6.urdf'
    g = urdf_feedforward(UrdfGravity(path), q)
    margins = np.minimum(q-LOWER, UPPER-q)
    if np.any(margins <= .05):
        raise ValueError('Refuse preview within driver 0.05 rad guard: hidden gain/target override')
    if np.any(np.abs(g) > RATED):
        raise ValueError('URDF gravity exceeds configured rated feedforward clamp')
    if np.any(np.abs(g) > PREVIEW_FF_LIMIT):
        raise ValueError('Gravity exceeds proposed 50%-rated first-test feedforward envelope')
    msg = dict(header=dict(stamp=dict(sec=0, nanosec=0), frame_id='base_link'),
               position=q.tolist(), velocity=[0.]*6, current=g.tolist(),
               p=KP.copy(), d=KD.copy(), enable=1)
    arrays = {'0x10':SIGNS*q, '0x11':np.zeros(6), '0x12':SIGNS*np.clip(g,-RATED,RATED),
              '0x13':np.array(KP), '0x14':np.array(KD)}
    # Match C++ cast<double->float>. Fresh ProtocolV1 slot6 starts at zero.
    wire = {cmd:list(struct.unpack('<7f',struct.pack('<7f',*v,0.))) for cmd,v in arrays.items()}
    return dict(dry_run_only=True, hardware_ready=False,
        assumptions=['fixed base; gravity=[0,0,-9.81]', 'no additional tool/payload for arm6',
                     'zero model offsets', 'no historical ratio/alpha/friction/armature correction',
                     '0x12=Nm and Kp/Kd physical scaling are UNVERIFIED on new firmware',
                     'fresh ProtocolV1; unused slot6 initialized to zero'],
        urdf=str(path.resolve()), urdf_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        q=q.tolist(), gravity_nm=g.tolist(), proposed_kp=KP, proposed_kd=KD,
        proposed_feedforward=g.tolist(), rated_feedforward_limits=RATED.tolist(),
        proposed_feedforward_limits=PREVIEW_FF_LIMIT.tolist(),
        gravity_fraction_rated=(np.abs(g)/RATED).tolist(), joint_limit_margin_rad=margins.tolist(),
        near_joint_limit=(margins<.15).tolist(), finite=True,
        direction_check='signs match repository; physical control direction NOT tested',
        MotorControl=msg, theoretical_float32_protocol_arrays=wire,
        total_torque_limit_warning='DMMotor clamps only current feedforward; board PD total limit unknown')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    src=ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--q', nargs=6, type=float)
    src.add_argument('--joint-state-json', type=Path, help='Saved JointState dictionary: name and position')
    ap.add_argument('--urdf', type=Path)
    args=ap.parse_args()
    q=args.q
    if args.joint_state_json:
        data=json.loads(args.joint_state_json.read_text())
        if len(data['name'])!=6 or len(set(data['name']))!=6 or set(data['name'])!=set(NAMES) or len(data['position'])!=6:
            raise ValueError('Saved JointState must contain exactly six unique verified names/positions')
        q=[data['position'][data['name'].index(name)] for name in NAMES]
    print(json.dumps(preview(q,args.urdf),indent=2,allow_nan=False))


if __name__=='__main__':main()
