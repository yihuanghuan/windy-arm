"""Safety-oriented adapter from joint torques to motor commands."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class MotorCommand:
    joint_names: list[str]
    command: np.ndarray
    stamp: float
    control_mode: str
    enabled: bool
    reason: str = ""


@dataclass
class ActuatorLimits:
    torque: np.ndarray
    torque_rate: np.ndarray
    current: np.ndarray


class ActuatorAdapter:
    """Convert torque to protocol command with conservative safety gates.

    current_unit may be:
    - "ampere": command is electric current after tau/(Kt*N*eta).
    - "torque_nm": command is torque-like and is sent after torque limiting.
    - "unknown": hardware output is refused.
    """

    def __init__(
        self,
        joint_names: list[str],
        limits: ActuatorLimits,
        current_unit: str = "unknown",
        torque_constant_nm_per_a: np.ndarray | None = None,
        gear_ratio: np.ndarray | None = None,
        efficiency: np.ndarray | None = None,
        command_timeout_sec: float = 0.05,
        joint_state_timeout_sec: float = 0.1,
        base_state_timeout_sec: float = 0.1,
        hardware_enabled: bool = False,
    ):
        self.joint_names = list(joint_names)
        self.limits = limits
        self.current_unit = current_unit
        self.Kt = np.ones(7) if torque_constant_nm_per_a is None else np.asarray(torque_constant_nm_per_a, dtype=float).reshape(7)
        self.N = np.ones(7) if gear_ratio is None else np.asarray(gear_ratio, dtype=float).reshape(7)
        self.eta = np.ones(7) if efficiency is None else np.asarray(efficiency, dtype=float).reshape(7)
        self.command_timeout_sec = float(command_timeout_sec)
        self.joint_state_timeout_sec = float(joint_state_timeout_sec)
        self.base_state_timeout_sec = float(base_state_timeout_sec)
        self.hardware_enabled = bool(hardware_enabled)
        self.estop = False
        self._last_tau = np.zeros(7)
        self._last_stamp: float | None = None

    @staticmethod
    def zero(joint_names: list[str], stamp: float, reason: str) -> MotorCommand:
        return MotorCommand(list(joint_names), np.zeros(7), float(stamp), "zero", False, reason)

    def set_estop(self, enabled: bool = True) -> None:
        self.estop = bool(enabled)

    def torque_to_motor_command(
        self,
        tau_arm: np.ndarray,
        now_sec: float,
        joint_state_stamp: float | None,
        base_state_stamp: float | None,
        dt: float | None = None,
    ) -> MotorCommand:
        if self.estop:
            self._last_tau[:] = 0.0
            return self.zero(self.joint_names, now_sec, "estop")
        tau = np.asarray(tau_arm, dtype=float).reshape(7)
        if not np.all(np.isfinite(tau)):
            return self.zero(self.joint_names, now_sec, "command contains NaN/Inf")
        if joint_state_stamp is None or now_sec - joint_state_stamp > self.joint_state_timeout_sec:
            return self.zero(self.joint_names, now_sec, "joint state stale")
        if base_state_stamp is None or now_sec - base_state_stamp > self.base_state_timeout_sec:
            return self.zero(self.joint_names, now_sec, "base state stale")
        if dt is None:
            dt = 0.0 if self._last_stamp is None else max(0.0, float(now_sec - self._last_stamp))
        if self._last_stamp is not None and now_sec - self._last_stamp > self.command_timeout_sec:
            self._last_tau[:] = 0.0
        torque_limit = np.asarray(self.limits.torque, dtype=float).reshape(7)
        rate_limit = np.asarray(self.limits.torque_rate, dtype=float).reshape(7)
        tau = np.clip(tau, -torque_limit, torque_limit)
        if dt > 0.0:
            tau = np.clip(tau, self._last_tau - rate_limit * dt, self._last_tau + rate_limit * dt)
        self._last_tau = tau.copy()
        self._last_stamp = float(now_sec)

        if self.current_unit == "ampere":
            denom = self.Kt * self.N * self.eta
            if np.any(np.abs(denom) < 1e-9):
                return self.zero(self.joint_names, now_sec, "invalid torque-current calibration")
            cmd = tau / denom
            current_limit = np.asarray(self.limits.current, dtype=float).reshape(7)
            cmd = np.clip(cmd, -current_limit, current_limit)
            mode = "current_ampere"
        elif self.current_unit == "torque_nm":
            cmd = tau
            mode = "torque_nm_protocol"
        else:
            return self.zero(self.joint_names, now_sec, "current unit is unconfirmed")

        if not self.hardware_enabled:
            return MotorCommand(self.joint_names, cmd, float(now_sec), mode, False, "hardware output disabled")
        return MotorCommand(self.joint_names, cmd, float(now_sec), mode, True)
