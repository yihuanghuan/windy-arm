#!/usr/bin/env python3
"""MPC IK demo: draw a horizontal circle in the base x-y plane."""

import rclpy

from move_arm_mpc_ik_demo import MoveArmMpcIkDemo


def main():
    rclpy.init()
    node = MoveArmMpcIkDemo(
        default_trajectory_mode="horizontal_circle",
        node_name="move_arm_horizontal_circle_mpc_ik_demo",
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
