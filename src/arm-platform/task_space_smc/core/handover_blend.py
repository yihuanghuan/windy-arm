from __future__ import annotations

import numpy as np

__all__ = [
    "HandoverBlend",
    "smoothstep_decay",
    "trajectory_clock",
    "velocity_decay",
]


def trajectory_clock(t: float, blend_sec: float, freeze: bool) -> float:
    t = float(t)
    if not freeze:
        return t
    T = float(blend_sec)
    if not np.isfinite(T) or T <= 0.0:

        return t
    if t < T:
        return 0.0
    return t - T


def smoothstep_decay(tau: float, duration: float) -> tuple[float, float, float]:
    T = float(duration)
    if T <= 0.0:
        return 0.0, 0.0, 0.0
    u = float(tau) / T
    if u <= 0.0:
        return 1.0, 0.0, 0.0
    if u >= 1.0:
        return 0.0, 0.0, 0.0
    rise = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
    rise_d = (30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4) / T
    rise_dd = (60.0 * u - 180.0 * u**2 + 120.0 * u**3) / (T * T)
    return 1.0 - rise, -rise_d, -rise_dd


def velocity_decay(tau: float, duration: float) -> tuple[float, float, float]:
    T = float(duration)
    if T <= 0.0:
        return 0.0, 0.0, 0.0
    u = float(tau) / T
    if u <= 0.0:
        return 0.0, 1.0, 0.0
    if u >= 1.0:
        return 0.0, 0.0, 0.0
    val = (u - 6.0 * u**3 + 8.0 * u**4 - 3.0 * u**5) * T
    d = 1.0 - 18.0 * u**2 + 32.0 * u**3 - 15.0 * u**4
    dd = (-36.0 * u + 96.0 * u**2 - 60.0 * u**3) / T
    return val, d, dd


def _hat(w: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -w[2], w[1]],
                     [w[2], 0.0, -w[0]],
                     [-w[1], w[0], 0.0]])


def _log_so3(R: np.ndarray) -> np.ndarray:
    R = np.asarray(R, dtype=float).reshape(3, 3)
    cos = 0.5 * (np.trace(R) - 1.0)
    cos = float(np.clip(cos, -1.0, 1.0))
    theta = float(np.arccos(cos))
    if theta < 1e-8:

        return np.array([R[2, 1] - R[1, 2],
                         R[0, 2] - R[2, 0],
                         R[1, 0] - R[0, 1]]) * 0.5
    if np.pi - theta < 1e-6:

        A = 0.5 * (R + np.eye(3))
        axis = np.sqrt(np.clip(np.diag(A), 0.0, None))
        k = int(np.argmax(axis))
        if axis[k] < 1e-12:
            return np.zeros(3)
        axis = A[:, k] / axis[k]
        axis = axis / max(np.linalg.norm(axis), 1e-12)
        return theta * axis
    return (theta / (2.0 * np.sin(theta))) * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def _exp_so3(w: np.ndarray) -> np.ndarray:
    w = np.asarray(w, dtype=float).reshape(3)
    theta = float(np.linalg.norm(w))
    if theta < 1e-12:
        return np.eye(3)
    K = _hat(w / theta)
    return (np.eye(3) + np.sin(theta) * K
            + (1.0 - np.cos(theta)) * (K @ K))


def _minimal_rotvec(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    s = np.asarray(source, dtype=float).reshape(3)
    t = np.asarray(target, dtype=float).reshape(3)
    ns, nt = np.linalg.norm(s), np.linalg.norm(t)
    if ns < 1e-12 or nt < 1e-12:
        return np.zeros(3)
    s = s / ns
    t = t / nt
    cross = np.cross(s, t)
    dot = float(np.clip(np.dot(s, t), -1.0, 1.0))
    sin_theta = float(np.linalg.norm(cross))
    if sin_theta < 1e-12:
        if dot > 0.0:
            return np.zeros(3)

        axis = np.cross(s, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1e-9:
            axis = np.cross(s, np.array([0.0, 1.0, 0.0]))
        return np.pi * axis / np.linalg.norm(axis)
    return np.arctan2(sin_theta, dot) * cross / sin_theta


class HandoverBlend:

    def __init__(self, duration_sec: float = 0.0) -> None:
        duration_sec = float(duration_sec)
        if not np.isfinite(duration_sec) or duration_sec < 0.0:
            raise ValueError(
                f"duration_sec 必须是非负有限值，收到 {duration_sec}")
        self.duration_sec = duration_sec
        self.t0: float | None = None
        self.delta_position = np.zeros(3)
        self.delta_rotvec = np.zeros(3)
        self.delta_velocity = np.zeros(3)

    @property
    def enabled(self) -> bool:
        return self.duration_sec > 0.0

    @property
    def captured(self) -> bool:
        return self.t0 is not None

    def capture(self, t0, actual_position, actual_rotation,
                reference_position, reference_rotation,
                pen_axis_local=None,
                actual_linear_velocity=None,
                reference_linear_velocity=None) -> None:
        if self.captured:
            return
        self.t0 = float(t0)
        if not self.enabled:
            return
        p_act = np.asarray(actual_position, dtype=float).reshape(3)
        p_ref = np.asarray(reference_position, dtype=float).reshape(3)
        R_act = np.asarray(actual_rotation, dtype=float).reshape(3, 3)
        R_ref = np.asarray(reference_rotation, dtype=float).reshape(3, 3)
        self.delta_position = p_act - p_ref
        if (actual_linear_velocity is not None
                and reference_linear_velocity is not None):
            self.delta_velocity = (
                np.asarray(actual_linear_velocity, dtype=float).reshape(3)
                - np.asarray(reference_linear_velocity, dtype=float).reshape(3))
        if pen_axis_local is None:
            self.delta_rotvec = _log_so3(R_act @ R_ref.T)
        else:
            a = np.asarray(pen_axis_local, dtype=float).reshape(3)
            self.delta_rotvec = _minimal_rotvec(R_ref @ a, R_act @ a)

    def scale(self, t) -> float:
        if not self.enabled or not self.captured:
            return 0.0
        return smoothstep_decay(float(t) - self.t0, self.duration_sec)[0]

    def apply(self, t, position, rotation, linear_velocity, angular_velocity,
              linear_acceleration, angular_acceleration):
        p = np.asarray(position, dtype=float).reshape(3)
        R = np.asarray(rotation, dtype=float).reshape(3, 3)
        v = np.asarray(linear_velocity, dtype=float).reshape(3)
        w = np.asarray(angular_velocity, dtype=float).reshape(3)
        a = np.asarray(linear_acceleration, dtype=float).reshape(3)
        wd = np.asarray(angular_acceleration, dtype=float).reshape(3)

        if not self.enabled or not self.captured:
            return p, R, v, w, a, wd
        tau = float(t) - self.t0
        s, sd, sdd = smoothstep_decay(tau, self.duration_sec)
        bv, bvd, bvdd = velocity_decay(tau, self.duration_sec)
        if (s == 0.0 and sd == 0.0 and sdd == 0.0
                and bv == 0.0 and bvd == 0.0 and bvdd == 0.0):
            return p, R, v, w, a, wd


        p_out = p + s * self.delta_position + bv * self.delta_velocity
        v_out = v + sd * self.delta_position + bvd * self.delta_velocity
        a_out = a + sdd * self.delta_position + bvdd * self.delta_velocity

        R_off = _exp_so3(s * self.delta_rotvec)
        R_out = R_off @ R

        w_rot = R_off @ w
        w_out = sd * self.delta_rotvec + w_rot
        wd_out = (sdd * self.delta_rotvec
                  + sd * np.cross(self.delta_rotvec, w_rot)
                  + R_off @ wd)
        return p_out, R_out, v_out, w_out, a_out, wd_out

    def peak_extra_acceleration(self) -> float:
        if not self.enabled:
            return 0.0


        return (10.0 * np.sqrt(3.0) / 3.0
                * float(np.linalg.norm(self.delta_position))
                / (self.duration_sec ** 2)
                + 3.9403 * float(np.linalg.norm(self.delta_velocity))
                / self.duration_sec)
