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
# ros2 launch easynav_experiments benchmark.launch.py nav_mode:=nav2
# goal:=goal_1

import math
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, TimerAction)
from launch_ros.actions import Node
import yaml


def _yaw_to_quat(yaw_rad):
    """Return (qz, qw) for a pure-yaw quaternion."""
    half = yaw_rad / 2.0
    return math.sin(half), math.cos(half)


def _setup(context, *args, **kwargs):
    """OpaqueFunction: resolves nav_mode-dependent paths and builds all runtime actions."""
    nav_mode = context.launch_configurations['nav_mode']
    goal_num = context.launch_configurations['goal']
    target_pid = context.launch_configurations['target_pid']
    output_dir = context.launch_configurations['output_dir']
    goals_file = context.launch_configurations['goals_file']
    goal_key = 'goal_%s' % goal_num

    # ── Auto-detect PID if not provided ──
    if int(target_pid) <= 0:
        process_name = 'system_main' if nav_mode == 'easynav' else 'component_container_isolated'
        try:
            result = subprocess.check_output(
                ['pgrep', '-f', process_name], text=True).strip()
            target_pid = result.splitlines()[0]
        except subprocess.CalledProcessError:
            target_pid = '-1'

    actions = []

    actions.append(Node(
        package='easynav_experiments',
        executable='benchmark_evaluator',
        parameters=[{
            'nav_mode': nav_mode,
            'run_id': int(goal_num),
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

    shell_cmd = (
        f'ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '
        f'"{{header: {{stamp: {{sec: $(date +%s), nanosec: 0}}, frame_id: map}}, '
        f'pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, '
        f'orientation: {{x: 0.0, y: 0.0, z: {qz:.6f}, w: {qw:.6f}}}}}}}"'
    )

    actions.append(TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c', shell_cmd],
                output='screen',
            )
        ],
    ))

    return actions


def generate_launch_description():

    experiments_dir = get_package_share_directory('easynav_experiments')

    # __file__ is always the source file, regardless of install layout
    pkg_src_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    src_results_dir = os.path.join(pkg_src_dir, 'results')

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

    declare_target_pid = DeclareLaunchArgument(
        'target_pid',
        default_value='-1',
        description='PID of the navigation process to monitor (CPU/RAM)',
    )

    declare_goals_file = DeclareLaunchArgument(
        'goals_file',
        default_value=os.path.join(
            experiments_dir, 'config', 'benchmark_goals.yaml'),
        description='YAML file that contains the named goal poses',
    )

    declare_output_dir = DeclareLaunchArgument(
        'output_dir',
        default_value=src_results_dir,
        description='Directory where benchmark_evaluator_<goal>.json files are saved',
    )

    ld = LaunchDescription()
    ld.add_action(declare_nav_mode)
    ld.add_action(declare_goal)
    ld.add_action(declare_target_pid)
    ld.add_action(declare_output_dir)
    ld.add_action(declare_goals_file)
    ld.add_action(OpaqueFunction(function=_setup))

    return ld
