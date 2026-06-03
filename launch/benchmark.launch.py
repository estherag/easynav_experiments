# Copyright 2026 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Usage:
#   ros2 launch easynav_experiments benchmark.launch.py nav_mode:=nav2 goal:=goal_1
#   ros2 launch easynav_experiments benchmark.launch.py nav_mode:=easynav goal:=goal_2 run_id:=3

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _yaw_to_quat(yaw_rad):
    """Return (qz, qw) for a pure-yaw quaternion."""
    half = yaw_rad / 2.0
    return math.sin(half), math.cos(half)


def _setup(context, *args, **kwargs):
    """OpaqueFunction: resolves nav_mode-dependent paths and builds all runtime actions."""
    easynav_dir = get_package_share_directory('easynav_indoor_testcase')
    experiments_dir = get_package_share_directory('easynav_experiments')

    nav_mode   = context.launch_configurations['nav_mode']
    run_id     = context.launch_configurations['run_id']
    target_pid  = context.launch_configurations['target_pid']
    output_dir  = context.launch_configurations['output_dir']
    goals_file  = context.launch_configurations['goals_file']
    goal_key   = 'goal_%s' % context.launch_configurations['goal']

    # ── params_file: use override if given, else pick default by nav_mode ──
    params_file = context.launch_configurations.get('params_file', '')
    if not params_file:
        if nav_mode == 'easynav':
            params_file = os.path.join(
                easynav_dir, 'robots_params', 'simple.params.yaml')
        else:
            try:
                nav2_dir = get_package_share_directory('nav2_bringup')
                params_file = os.path.join(nav2_dir, 'params', 'nav2_params.yaml')
            except Exception:
                params_file = ''

    actions = []

    if nav_mode == 'easynav':
        actions.append(Node(
            package='easynav_system',
            executable='system_main',
            parameters=[params_file],
            output='screen',
        ))
    else:
        try:
            nav2_dir = get_package_share_directory('nav2_bringup')
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
                launch_arguments={'params_file': params_file}.items(),
            ))
        except Exception:
            raise RuntimeError('nav2_bringup not found but nav_mode:=nav2 was requested')

    actions.append(Node(
        package='easynav_experiments',
        executable='nav_metrics',
        parameters=[{
            'nav_mode': nav_mode,
            'run_id': int(run_id),
            'target_pid': int(target_pid),
            'output_dir': output_dir,
        }],
        output='screen',
    ))

    with open(goals_file, 'r') as f:
        data = yaml.safe_load(f)

    if goal_key not in data:
        raise RuntimeError(
            f'Goal "{goal_key}" not found in {goals_file}. '
            f'Available: {list(data.keys())}')

    g = data[goal_key]
    x, y, yaw = float(g['x']), float(g['y']), float(g['yaw'])
    qz, qw = _yaw_to_quat(yaw)

    pose_str = (
        f'{{header: {{frame_id: map}}, '
        f'pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, '
        f'orientation: {{x: 0.0, y: 0.0, z: {qz:.6f}, w: {qw:.6f}}}}}}}'
    )

    actions.append(TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--once',
                     '/goal_pose', 'geometry_msgs/msg/PoseStamped', pose_str],
                output='screen',
            )
        ],
    ))

    return actions


def generate_launch_description():

    easynav_dir = get_package_share_directory('easynav_indoor_testcase')
    experiments_dir = get_package_share_directory('easynav_experiments')

    declare_nav_mode = DeclareLaunchArgument(
        'nav_mode',
        default_value='easynav',
        description='"easynav" or "nav2"',
    )

    declare_goal = DeclareLaunchArgument(
        'goal',
        default_value='1',
        description='Goal number (integer). The launch looks up goal_<N> in benchmark_goals.yaml',
    )

    declare_run_id = DeclareLaunchArgument(
        'run_id',
        default_value='0',
        description='Run identifier — appended to the output JSON filename',
    )

    declare_target_pid = DeclareLaunchArgument(
        'target_pid',
        default_value='-1',
        description='PID of the navigation process to monitor (CPU/RAM)',
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description=(
            'Full path to the navigation params YAML. '
            'If empty, the default for the selected nav_mode is used: '
            'easynav → simple.params.yaml, nav2 → nav2_params.yaml'),
    )

    declare_goals_file = DeclareLaunchArgument(
        'goals_file',
        default_value=os.path.join(
            experiments_dir, 'config', 'benchmark_goals.yaml'),
        description='YAML file that contains the named goal poses',
    )

    declare_output_dir = DeclareLaunchArgument(
        'output_dir',
        default_value=os.path.join(experiments_dir, 'results'),
        description='Directory where nav_metrics_<run_id>.json files are saved',
    )

    ld = LaunchDescription()
    ld.add_action(declare_nav_mode)
    ld.add_action(declare_goal)
    ld.add_action(declare_run_id)
    ld.add_action(declare_target_pid)
    ld.add_action(declare_params_file)
    ld.add_action(declare_output_dir)
    ld.add_action(declare_goals_file)
    ld.add_action(OpaqueFunction(function=_setup))

    return ld
