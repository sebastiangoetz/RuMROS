#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'depot.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                     'models'))

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    bridge_params = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'params',
        'general_bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )
    
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)

    drone_configs = [
       {'namespace': 'drone_1', 'x_pose': '2.0', 'y_pose': '0.0', 'z_pose': '0.2'},
    ]

    # Add each drone
    for config in drone_configs:
        namespace = config['namespace']
        x_pose = config['x_pose']
        y_pose = config['y_pose']
        z_pose = config['z_pose']

        spawn_quadcopter_x3_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_x3_quadcopter_launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'x_pose': x_pose,
                'y_pose': y_pose,
                'z_pose': z_pose,
            }.items()
        )

        ld.add_action(spawn_quadcopter_x3_cmd)

    spawn_rasprover_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_ugv_launch.py')
        ),
        launch_arguments={
            'namespace': "rover_1",
            'use_sim_time': use_sim_time,
            'x_pose': "-2.0",
            'y_pose': "0.0",
            'z_pose': "0.0",
        }.items()
    )
    ld.add_action(spawn_rasprover_cmd)

    robot_configs = [
        {'namespace': 'turtlebot_1', 'x_pose': '0.0', 'y_pose': '1.0', 'z_pose': '0.2'},
        {'namespace': 'turtlebot_2', 'x_pose': '0.0', 'y_pose': '-1.0', 'z_pose': '0.2'},
    ]

    # Add each robot
    for config in robot_configs:
        namespace = config['namespace']
        x_pose = config['x_pose']
        y_pose = config['y_pose']
        z_pose = config['z_pose']

        spawn_turtlebot_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'x_pose': x_pose,
                'y_pose': y_pose,
                'z_pose': z_pose,
            }.items()
        )

        ld.add_action(spawn_turtlebot_cmd)

    return ld
