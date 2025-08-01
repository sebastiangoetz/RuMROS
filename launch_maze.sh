#!/bin/bash

# Set default ROS distribution to "jazzy"
ROS_DISTRO=${1:-jazzy}

echo "Starting ROS Application with maze world..."
colcon build --base-paths ros
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch rumros maze.launch.py