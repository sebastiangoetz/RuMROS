import json
import os

from launch.actions import OpaqueFunction
from simple_launch import SimpleLauncher

sl = SimpleLauncher()

sl.declare_arg('swarm_config', description='Path to swarm configuration JSON file')

def find_key_by_id(data, id):
    """Helper to find a key in a dictionary that is associated with a
    value that is present in the list of values.

    Args:
        data (dict): A dictionary with lists as values.
        id (Any): The value to find the key for.

    Returns:
        Any: The key for the value or `None` if `id` was not found.
    """
    for k, v in data.items():
        if id in v:
            return k
    return None

def launch_setup():
    """
    ROS Launch file to launch nodes for all robots of the application.
    Execute the following command in the ROS2 workspace:
    ros2 launch swarm_controller swarm_launch.py swarm_config:=$(pwd)/config/swarm_config.json type_config:=$(pwd)/config/type_config.json
    """
    swarm_config_file = sl.arg('swarm_config')
    type_config_file = sl.arg('type_config')

    if not os.path.exists(swarm_config_file):
        raise FileNotFoundError(f"Expected config file at {swarm_config_file}, but it was not found.")

    with open(swarm_config_file, 'r') as f:
        swarm_config = json.load(f)

    with open(type_config_file, 'r') as f:
        type_config = json.load(f)

    for group_id, robot_ids in swarm_config.items():
        if not robot_ids: continue # IDs can be empty if group is purely used for nesting
        print(f'Swarm group {group_id}: IDs {min(robot_ids)} - {max(robot_ids)}')
        for robot_id in robot_ids:
            robot_type = str(find_key_by_id(type_config, robot_id)).lower()
            sl.node(
                package='swarm_controller',
                executable='node_main',
                name=f'sg{group_id}_bot{robot_id}',
                arguments=[str(group_id), str(robot_id), robot_type],
            )

    return sl.launch_description()

# /!\ no `def generate_launch_description():`
generate_launch_description = sl.launch_description(opaque_function = launch_setup)
