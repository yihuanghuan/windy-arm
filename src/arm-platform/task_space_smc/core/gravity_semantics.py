"""URDF inertias are the sole gravity source; encoder offsets require opt-in.

arm6/arm6_tool already contain fitted link inertias. Historical per-joint
first_moment_ratio is provenance, never another runtime torque multiplier.
"""
import numpy as np


def explicit_offsets(n, offsets=None):
    result = np.zeros(n) if offsets is None else np.asarray(offsets, dtype=float).reshape(n).copy()
    if not np.all(np.isfinite(result)):
        raise ValueError('model zero offsets must be finite')
    return result


def urdf_feedforward(model, q_encoder, offsets=None):
    q = np.asarray(q_encoder, dtype=float)
    if q.ndim != 1 or not np.all(np.isfinite(q)):
        raise ValueError('joint angles must be a finite vector')
    result = np.asarray(model.gravity(q + explicit_offsets(len(q), offsets)), dtype=float).copy()
    if result.shape != q.shape or not np.all(np.isfinite(result)):
        raise ValueError('invalid URDF gravity result')
    return result
