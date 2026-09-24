#!/usr/bin/env python3
"""Rotation-only hand-eye solve, with known-translation consistency diagnostics."""
import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares


def mean_rotation(rotations):
    u, _, vt = np.linalg.svd(np.sum(rotations, axis=0))
    return u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt


def angle_deg(rotation):
    return float(np.rad2deg(np.linalg.norm(pin.log3(rotation))))


def sample_transform(sample, prefix):
    r = np.asarray(sample[prefix+'_rotation'], dtype=float)
    p = np.asarray(sample[prefix+'_position'], dtype=float)
    if r.shape != (3,3) or p.shape != (3,) or not np.isfinite(np.r_[r.ravel(),p]).all():
        raise ValueError('invalid sample dimensions/non-finite values')
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or np.linalg.det(r) < .999:
        raise ValueError('sample is not a proper rotation')
    return r, p


def solve_rotation(samples, left_to_camera, left_xyz, min_poses=8,
                   min_angle_deg=10., max_mean_deg=2., max_pair_deg=5.,
                   max_translation_mean=.02, max_translation_pair=.05,
                   parent_frame='link6'):
    if len(samples) < min_poses:
        raise ValueError(f'need at least {min_poses} poses; have {len(samples)}')
    robot = [sample_transform(s, 'robot') for s in samples]
    camera = [sample_transform(s, 'camera') for s in samples]
    pairs, usable = [], []
    for i in range(len(samples)):
        for j in range(i+1, len(samples)):
            a = robot[i][0].T @ robot[j][0]
            b = camera[i][0].T @ camera[j][0]
            aa, ba = angle_deg(a), angle_deg(b)
            use = min(aa, ba) >= min_angle_deg and max(aa, ba) < 170.
            pairs.append(dict(i=i+1, j=j+1, robot_angle_deg=aa, camera_angle_deg=ba,
                              angle_mismatch_deg=abs(aa-ba), usable=use))
            if use:
                usable.append((a,b,len(pairs)-1))
    if len(usable) < 12:
        raise ValueError(f'only {len(usable)} usable relative pairs; collect varied 15–45 deg poses')
    av = np.array([pin.log3(a) for a,b,_ in usable])
    bv = np.array([pin.log3(b) for a,b,_ in usable])
    axes = bv / np.linalg.norm(bv, axis=1)[:,None]
    singular = np.linalg.svd(axes, compute_uv=False)
    excitation_ratio = float(singular[1]/singular[0])
    if excitation_ratio < .15:
        raise ValueError(f'nearly coaxial motions (second/first axis singular value={excitation_ratio:.4f}); move about a second axis')
    # log(A)=X log(B); Wahba supplies a global rotation initialization.
    u, _, vt = np.linalg.svd(av.T @ bv)
    initial = u @ np.diag([1.,1.,np.linalg.det(u@vt)]) @ vt
    def residual(delta):
        x = pin.exp3(delta) @ initial
        return np.concatenate([pin.log3(a @ x @ b.T @ x.T) for a,b,_ in usable])
    fit = least_squares(residual, np.zeros(3), loss='soft_l1', f_scale=np.deg2rad(1.),
                        xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=200)
    x = pin.exp3(fit.x) @ initial
    errors = np.rad2deg(np.linalg.norm(residual(fit.x).reshape(-1,3),axis=1))
    for error, (_,_,index) in zip(errors, usable):
        pairs[index]['residual_deg'] = float(error)
    # Calibration diagnostic only: a single constant axis alignment for comparing
    # relative translations. No world/base translation or online base estimator.
    world_base_rotations = [rc @ x.T @ rl.T for (rl,_),(rc,_) in zip(robot,camera)]
    rwb = mean_rotation(world_base_rotations)
    rlc_left = x @ left_to_camera.rotation.T
    plc = np.asarray(left_xyz) + rlc_left @ left_to_camera.translation
    predicted = [p + r @ plc for r,p in robot]
    translation_errors = []
    for pair in pairs:
        i,j = pair['i']-1,pair['j']-1
        predicted_delta = rwb @ (predicted[j]-predicted[i])
        measured_delta = camera[j][1]-camera[i][1]
        error = float(np.linalg.norm(predicted_delta-measured_delta))
        pair['translation_error_m'] = error
        pair['predicted_translation_norm_m'] = float(np.linalg.norm(predicted_delta))
        pair['measured_translation_norm_m'] = float(np.linalg.norm(measured_delta))
        translation_errors.append(error)
    failures = []
    if not fit.success:
        failures.append('nonlinear fit did not converge')
    if np.mean(errors)>max_mean_deg or np.max(errors)>max_pair_deg:
        failures.append('rotational residual exceeds threshold; check fixed base/tracking/model')
    if np.mean(translation_errors)>max_translation_mean or np.max(translation_errors)>max_translation_pair:
        failures.append('known-translation sanity check failed; check fixed base/translation/SLAM')
    result = dict(accepted=not failures, rejection_reasons=failures,
                sample_count=len(samples), usable_relative_pairs=len(usable),
                parent_frame=parent_frame,
                R_parent_camera=x.tolist(), q_parent_camera_xyzw=pin.Quaternion(x).coeffs().tolist(),
                joint6_to_left_optical_xyzw=pin.Quaternion(rlc_left).coeffs().tolist(),
                p_parent_camera=plc.tolist(), mean_rotation_residual_deg=float(np.mean(errors)),
                max_rotation_residual_deg=float(np.max(errors)),
                mean_translation_residual_m=float(np.mean(translation_errors)),
                max_translation_residual_m=float(np.max(translation_errors)),
                axes_singular_values=singular.tolist(), axes_second_first_ratio=excitation_ratio,
                constant_alignment_spread_deg=max(angle_deg(r @ rwb.T) for r in world_base_rotations),
                thresholds=dict(min_poses=min_poses,min_angle_deg=min_angle_deg,
                                max_mean_deg=max_mean_deg,max_pair_deg=max_pair_deg,
                                max_translation_mean=max_translation_mean,
                                max_translation_pair=max_translation_pair), pairs=pairs)
    result[f'R_{parent_frame}_camera'] = x.tolist()
    result[f'q_{parent_frame}_camera_xyzw'] = pin.Quaternion(x).coeffs().tolist()
    return result
