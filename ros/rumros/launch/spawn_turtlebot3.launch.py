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
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import rclpy.logging
import tempfile
import subprocess


def generate_launch_description():

    launch_file_dir = os.path.join(get_package_share_directory('rumros'), 'launch')

    # Get the urdf file
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL

    # Workaround taken from Paul Reidelshoefers solution for namespace bug of robot_state_publisher:
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    
    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    namespace = LaunchConfiguration('namespace', default='robot_1')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='robot_1',
        description='Specify namespace of the robot')
    
    declare_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    xacro_file_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf.xacro'
    )

    robot_description = Command([
        'xacro',
        ' ',
        xacro_file_path,
        ' ',
        'namespace:=', LaunchConfiguration('namespace')
    ])

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        output='screen',
        arguments=[
            '-name', [namespace, '_' + TURTLEBOT3_MODEL],
            '-string', robot_description,
            '-x', x_pose,
            '-y', y_pose,
            '-z', LaunchConfiguration('z_pose')
        ],
       remappings=remappings
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
        remappings=remappings
    )

    bridge_params_template = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'params',
        'turtlebot3_waffle_bridge_template.yaml'
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

    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name=[namespace, "_image_bridge"],
        arguments=[['/', namespace, '/camera/image_raw']],
        output='screen',
    )

    slam_config_file_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'params',
        'mapper_params_lifelong.yaml'
    )
    
    slam_remappings = remappings.copy()
    slam_remappings.extend([
        ('/scan', 'scan'),
        ('/map', 'map'),
        ('/odom', 'odom_internal'),
        ('/pose', 'pose'),
        ('pose', 'pose_internal'),
    ])
    
    start_gazebo_ros_slam_toolbox_cmd = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
        parameters=[
            slam_config_file_path,
            {
            'use_sim_time': True,
            'odom_frame': "odom",
            'map_frame': "map",
            'base_frame': "base_link",
            # 'scan_topic': [namespace, "/scan"],
        }],
        remappings=slam_remappings
    )

    start_pose_offsetter_cmd = Node(
        package='turtlebot3_gazebo',
        executable='pose_offsetter',
        name='pose_offsetter',
        namespace=namespace,
        parameters=[
            {'namespace': namespace},
            {'input_topic': 'pose_internal'},
            {'output_topic': 'pose'},
            {'message_type': 'pose'},
            {'x_offset': x_pose},
            {'y_offset': y_pose}
        ]
    )

    # start_map_saver_cmd = Node(
    #     package='turtlebot3_gazebo',
    #     executable='map_saver',
    #     name='map_saver',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': True,
    #     }]
    # )


    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_sim_time_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(start_robot_state_publisher_cmd)

    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_image_bridge_cmd)
    ld.add_action(start_gazebo_ros_slam_toolbox_cmd)
    ld.add_action(start_pose_offsetter_cmd)
    # ld.add_action(start_map_saver_cmd)

    return ld