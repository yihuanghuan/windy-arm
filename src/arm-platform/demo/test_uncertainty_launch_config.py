#!/usr/bin/env python3
"""Static contract tests for the one-command uncertainty experiment."""

import ast
from pathlib import Path
import unittest

import yaml


PACKAGE_DIR = Path(__file__).resolve().parent.parent
LAUNCH_PATH = PACKAGE_DIR / 'launch' / 'moving_base_uncertainty.launch.py'
PARAMETERS_PATH = PACKAGE_DIR / 'config' / 'moving_base_uncertainty.yaml'
RVIZ_PATH = PACKAGE_DIR / 'config' / 'moving_base_uncertainty.rviz'
CONTROLLER_PATH = (
    PACKAGE_DIR / 'demo' / 'move_base_circle_mpc_ik_demo.py')
VISUALIZER_PATH = PACKAGE_DIR / 'demo' / 'trajectory_visualizer_demo.py'


def _constant_first_arguments(path: Path, function_name: str):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    values = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        called = node.func
        name = (
            called.attr if isinstance(called, ast.Attribute)
            else called.id if isinstance(called, ast.Name)
            else None
        )
        if (
            name == function_name
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and isinstance(node.args[0].value, str)
        ):
            values.append(node.args[0].value)
    return values


class UncertaintyLaunchConfigTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.parameters = yaml.safe_load(
            PARAMETERS_PATH.read_text(encoding='utf-8'))
        cls.rviz = yaml.safe_load(RVIZ_PATH.read_text(encoding='utf-8'))

    def test_launch_exposes_only_the_three_scene_choices(self):
        arguments = _constant_first_arguments(
            LAUNCH_PATH, 'DeclareLaunchArgument')
        self.assertEqual(arguments, [
            'trajectory_shape',
            'base_disturbance_profile',
            'base_belief_mode',
        ])

    def test_yaml_covers_every_controller_and_visualizer_parameter(self):
        mappings = (
            (
                'move_base_circle_mpc_ik_demo',
                CONTROLLER_PATH,
            ),
            (
                'trajectory_visualizer_demo',
                VISUALIZER_PATH,
            ),
        )
        for node_name, source_path in mappings:
            with self.subTest(node=node_name):
                declared = set(_constant_first_arguments(
                    source_path, 'declare_parameter'))
                configured = set(
                    self.parameters[node_name]['ros__parameters'])
                self.assertEqual(configured, declared)

    def test_cross_node_truth_clock_and_plant_values_match(self):
        arm = self.parameters['student_arm_node']['ros__parameters']
        controller = self.parameters[
            'move_base_circle_mpc_ik_demo']['ros__parameters']
        visualizer = self.parameters[
            'trajectory_visualizer_demo']['ros__parameters']

        self.assertEqual(
            arm['velocity_feedforward_gain'],
            controller['plant_velocity_feedforward_gain'])
        self.assertEqual(
            arm['position_feedback_tau'],
            controller['plant_position_feedback_tau'])
        self.assertEqual(
            arm['feedback_velocity_limit'],
            controller['plant_feedback_velocity_limit'])
        self.assertEqual(
            arm['max_velocity'], controller['plant_max_velocity'])
        self.assertEqual(
            arm['max_acceleration'], controller['plant_max_acceleration'])

        for key in (
                'trajectory_shape',
                'trajectory_time_scale',
                'base_motion_time_scale',
                'base_disturbance_amplitude_m',
                'base_disturbance_seed',
                'base_disturbance_profile'):
            self.assertEqual(controller[key], visualizer[key], key)
        self.assertEqual(
            controller['path_progress_topic'],
            visualizer['path_progress_topic'])
        self.assertEqual(visualizer['path_clock_mode'], 'required')

    def test_rviz_uses_world_and_enables_disturbance_marker_array(self):
        manager = self.rviz['Visualization Manager']
        self.assertEqual(manager['Global Options']['Fixed Frame'], 'world')
        displays = {
            display.get('Name'): display
            for display in manager['Displays']
        }
        disturbance = displays['BaseDisturbance']
        self.assertTrue(disturbance['Enabled'])
        self.assertEqual(
            disturbance['Marker Topic']['Value'],
            '/moving_base/disturbance_markers')
        self.assertEqual(
            disturbance['Class'], 'rviz_default_plugins/MarkerArray')


if __name__ == '__main__':
    unittest.main()
