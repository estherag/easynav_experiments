# Copyright 2026 Intelligent Robotics Lab
#
# This file is part of the project Easy Navigation (EasyNav in short)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    experiments_dir = get_package_share_directory(
        'easynav_experiments')
    nav2_dir = get_package_share_directory(
        'nav2_bringup')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    run_id = LaunchConfiguration('run_id')
    package_src_dir = Path(__file__).resolve().parent.parent

    results_dir = str(package_src_dir / 'results/latency')

    output_file = PathJoinSubstitution([
        results_dir,
        PythonExpression([
            "'latency_nav2_' + str(",
            run_id,
            ") + '.csv'",
        ]),
    ])

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            experiments_dir,
            'maps',
            'home2.yaml'),
        description='Map used by Nav2 bringup',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            experiments_dir,
            'config',
            'kobuiki.yaml'),
        description='Nav2 parameter file',
    )

    declare_run_id_cmd = DeclareLaunchArgument(
        'run_id',
        default_value='1',
        description='Benchmark run identifier',
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_dir,
                'launch',
                'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_composition': 'True',
            # ADD INTRACOMMS
        }.items(),
    )

    scan_bridge_cmd = Node(
        package='easynav_experiments',
        executable='scan_mode_bridge',
        parameters=[
            {
                'latency_output_file': output_file,
                'dist_blocked': 0.40,
            },
        ],
        output='screen',
    )

    # remove spin behavior from yaml

    ld = LaunchDescription()

    ld.add_action(declare_map_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_run_id_cmd)

    ld.add_action(scan_bridge_cmd)
    ld.add_action(
        TimerAction(
            period=2.0,
            actions=[bringup_cmd],
        )
    )

    return ld
