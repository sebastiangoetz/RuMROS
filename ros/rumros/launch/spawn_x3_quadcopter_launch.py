# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import tempfile
import subprocess


def generate_launch_description():

    launch_file_dir = os.path.join(get_package_share_directory('rumros'), 'launch')

    # Get the urdf file
    model_folder = 'X3 UAV Config 5'

    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.0')
    namespace = LaunchConfiguration('namespace', default='drone_1')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the drone')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the drone')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='drone_1',
        description='Specify namespace of the drone')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    xacro_file_path = os.path.join(
        get_package_share_directory('rumros'),
        'models',
        model_folder,
        'model.sdf.xacro'
    )

    drone_description = Command([
        'xacro',
        ' "',
        xacro_file_path,
        '" ',
        'namespace:="', LaunchConfiguration('namespace'), '" ',
        'model_folder:="', model_folder, '" ',
    ])

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_x3_uav',
        namespace=namespace,
        arguments=[
            '-name', "X3",
            '-string', drone_description,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen',
        remappings=remappings
    )

    bridge_params_template = os.path.join(
        get_package_share_directory('rumros'),
        'params',
        'x3_quadcopter_bridge_template.yaml'
    )

    bridge_params_file = Command(['python3 ', launch_file_dir, '/ros_gz_bridge_helper.py ', bridge_params_template, ' ', namespace]),

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=[namespace, "_parameter_bridge"],
        arguments=[
            '--ros-args',
            '-p',
            bridge_params_file,
        ],
        output='screen',
    )

    # launch robot state publisher
    start_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            name='drone_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': drone_description,
            }],
            remappings=remappings
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_state_publisher)

    return ld