"""Base-state providers for simulation injection and ROS feedback."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .quaternion_math import rotation_to_quat, skew


@dataclass
class BaseState:
    position_w: np.ndarray
    quaternion_wxyz: np.ndarray
    rotation_wb: np.ndarray
    velocity_lwa: np.ndarray
    acceleration_lwa: np.ndarray
    stamp: float


@dataclass
class SinusoidSpec:
    amplitude: np.ndarray
    frequency_hz: np.ndarray
    phase_rad: np.ndarray
    offset: np.ndarray

    @classmethod
    def from_lists(cls, amplitude, frequency_hz, phase_rad=None, offset=None) -> "SinusoidSpec":
        amp = np.asarray(amplitude, dtype=float).reshape(3)
        freq = np.asarray(frequency_hz, dtype=float).reshape(3)
        phase = np.zeros(3) if phase_rad is None else np.asarray(phase_rad, dtype=float).reshape(3)
        off = np.zeros(3) if offset is None else np.asarray(offset, dtype=float).reshape(3)
        return cls(amp, freq, phase, off)

    def value(self, t: float) -> np.ndarray:
        wt = 2.0 * math.pi * self.frequency_hz * t + self.phase_rad
        return self.offset + self.amplitude * np.sin(wt)

    def first(self, t: float) -> np.ndarray:
        w = 2.0 * math.pi * self.frequency_hz
        wt = w * t + self.phase_rad
        return self.amplitude * w * np.cos(wt)

    def second(self, t: float) -> np.ndarray:
        w = 2.0 * math.pi * self.frequency_hz
        wt = w * t + self.phase_rad
        return -self.amplitude * w * w * np.sin(wt)


def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rot_axis_second(axis: str, a: float, ad: float, add: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if axis == "x":
        R = _rot_x(a)
        G = skew([1.0, 0.0, 0.0])
    elif axis == "y":
        R = _rot_y(a)
        G = skew([0.0, 1.0, 0.0])
    elif axis == "z":
        R = _rot_z(a)
        G = skew([0.0, 0.0, 1.0])
    else:
        raise ValueError(axis)
    Rd = R @ G * ad
    Rdd = R @ (G @ G) * (ad * ad) + R @ G * add
    return R, Rd, Rdd


class SimulationInjectedBase:
    """Analytic six-axis sinusoidal base motion in world coordinates."""

    def __init__(self, translation: SinusoidSpec, rpy: SinusoidSpec):
        self.translation = translation
        self.rpy = rpy

    def sample(self, t: float) -> BaseState:
        p = self.translation.value(t)
        pd = self.translation.first(t)
        pdd = self.translation.second(t)
        rpy = self.rpy.value(t)
        rpy_d = self.rpy.first(t)
        rpy_dd = self.rpy.second(t)

        Rx, Rxd, Rxdd = _rot_axis_second("x", rpy[0], rpy_d[0], rpy_dd[0])
        Ry, Ryd, Rydd = _rot_axis_second("y", rpy[1], rpy_d[1], rpy_dd[1])
        Rz, Rzd, Rzdd = _rot_axis_second("z", rpy[2], rpy_d[2], rpy_dd[2])
        R = Rz @ Ry @ Rx
        Rd = Rzd @ Ry @ Rx + Rz @ Ryd @ Rx + Rz @ Ry @ Rxd
        Rdd = (
            Rzdd @ Ry @ Rx
            + Rz @ Rydd @ Rx
            + Rz @ Ry @ Rxdd
            + 2.0 * Rzd @ Ryd @ Rx
            + 2.0 * Rzd @ Ry @ Rxd
            + 2.0 * Rz @ Ryd @ Rxd
        )
        W = Rd @ R.T
        Wd = Rdd @ R.T + Rd @ Rd.T
        omega_w = np.array([W[2, 1], W[0, 2], W[1, 0]])
        alpha_w = np.array([Wd[2, 1], Wd[0, 2], Wd[1, 0]])
        return BaseState(
            position_w=p,
            quaternion_wxyz=rotation_to_quat(R),
            rotation_wb=R,
            velocity_lwa=np.concatenate([pd, omega_w]),
            acceleration_lwa=np.concatenate([pdd, alpha_w]),
            stamp=float(t),
        )


class RosFeedbackBaseState:
    """Minimal holder for an externally fused ROS base state.

    IMU specific force, mounting offsets, TF transforms and filtering must be
    resolved before calling update().  The controller consumes only the unified
    Pinocchio LOCAL_WORLD_ALIGNED free-flyer tangent convention.
    """

    def __init__(self, timeout_sec: float = 0.1):
        self.timeout_sec = float(timeout_sec)
        self._state: BaseState | None = None

    def update(self, state: BaseState) -> None:
        self._state = state

    def latest(self, now_sec: float) -> BaseState:
        if self._state is None:
            raise RuntimeError("base state has not been received")
        if now_sec - self._state.stamp > self.timeout_sec:
            raise RuntimeError("base state is stale")
        return self._state
