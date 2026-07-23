#!/usr/bin/env python3
"""Shared moving-base trajectory configuration.

The IK demo, MPC demo, and visualizer import this file so base/world transforms
stay consistent. Edit this file when switching between fixed-base and
moving-base experiments.
"""

import math

import numpy as np
import pinocchio as pin


WORLD_FRAME = 'world'
BASE_FRAME = 'base_link'

# Moving-base experiment: small base motion around the world-frame origin.
BASE_CIRCLE_CENTER = np.array([0.0, 0.0, 0.0])
BASE_CIRCLE_RADIUS = 0.02 #水平运动半径
BASE_PERIOD_SEC = 1.0  # 基座平移和姿态周期，改小频率增大
BASE_HEIGHT = 0.01 # 固定高度（不运动）
BASE_ATTITUDE_AMPLITUDE_DEG = 0.1  # deg

BASE_ROLL_AMPLITUDE = math.radians(BASE_ATTITUDE_AMPLITUDE_DEG)
BASE_PITCH_AMPLITUDE = math.radians(BASE_ATTITUDE_AMPLITUDE_DEG)
BASE_YAW_AMPLITUDE = math.radians(BASE_ATTITUDE_AMPLITUDE_DEG)

EE_WORLD_CIRCLE_CENTER = np.array([0.3, 0.01, 0.1])
EE_CIRCLE_RADIUS = 0.08
EE_PERIOD_SEC = 4.0

TARGET_YAW = math.radians(90.0)
TARGET_PITCH = math.radians(90.0)
TARGET_ROLL = 0.0
TARGET_ROTATION = pin.rpy.rpyToMatrix(TARGET_ROLL, TARGET_PITCH, TARGET_YAW)


# 基座运动形状
def base_pose(t: float) -> np.ndarray:
    """Return base_link origin position in world frame."""
    w = 2.0 * math.pi / BASE_PERIOD_SEC
    angle = w * t
    return np.array([
        BASE_CIRCLE_CENTER[0] + BASE_CIRCLE_RADIUS * math.cos(angle),
        BASE_CIRCLE_CENTER[1] + BASE_CIRCLE_RADIUS * math.sin(angle),
        BASE_HEIGHT,
    ])


def base_rotation(t: float) -> np.ndarray:
    """Return base_link rotation matrix in world frame."""
    w = 2.0 * math.pi / BASE_PERIOD_SEC
    roll = BASE_ROLL_AMPLITUDE * math.sin(w * t)
    pitch = BASE_PITCH_AMPLITUDE * math.sin(w * t + 2.0 * math.pi / 3.0)
    yaw = BASE_YAW_AMPLITUDE * math.sin(w * t + 4.0 * math.pi / 3.0)
    return pin.rpy.rpyToMatrix(roll, pitch, yaw)


def ee_target_world(t: float) -> np.ndarray:
    """Return desired end-effector position in world frame."""
    w = 2.0 * math.pi / EE_PERIOD_SEC
    return EE_WORLD_CIRCLE_CENTER + EE_CIRCLE_RADIUS * np.array([
        math.cos(w * t),
        math.sin(w * t),
        0.0,
    ])


def target_rotation_world() -> np.ndarray:
    """Return desired end-effector rotation matrix in world frame."""
    return TARGET_ROTATION.copy()
