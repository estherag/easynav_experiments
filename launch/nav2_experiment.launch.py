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
from launch.actions import DeclareLaunchArgument, Shutdown, TimerAction
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
    experiment_params = LaunchConfiguration('experiment_params_file')
    run_id = LaunchConfiguration('run_id')
    package_src_dir = Path(__file__).resolve().parent.parent

    results_dir = str(package_src_dir / 'results')

    output_file = PathJoinSubstitution([
        results_dir,
        PythonExpression([
            "'benchmark_nav2_' + str(",
            run_id,
            ") + '.csv'",
        ]),
    ])

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            experiments_dir,
            'maps',
            'my-home.yaml'),
        description='Map used by Nav2 bringup',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            experiments_dir,
            'config',
            'nav2.params.yaml'),
        description='Nav2 parameter file',
    )

    declare_experiment_params_cmd = DeclareLaunchArgument(
        'experiment_params_file',
        default_value=os.path.join(
            experiments_dir,
            'config',
            'experiment.yaml'),
        description='Benchmark parameter file',
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
            'use_intra_process_comms': 'True',
        }.items(),
    )

    evaluator_cmd = Node(
        package='easynav_experiments',
        executable='benchmark_evaluator',
        parameters=[
            experiment_params,
            {
                'navigation': 'nav2',
                'output_file': output_file,
                'target': 'component_conta',
            },
        ],
        output='screen',
        on_exit=Shutdown(),
    )

    ld = LaunchDescription()

    ld.add_action(declare_map_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_experiment_params_cmd)
    ld.add_action(declare_run_id_cmd)

    ld.add_action(bringup_cmd)
    ld.add_action(
        TimerAction(
            period=15.0,
            actions=[evaluator_cmd],
        )
    )

    return ld
