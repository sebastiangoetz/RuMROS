# copy of launch.sh
# just launch ROS 2 with a different world

#!/bin/bash

# Check if a ROS 2 distribution was provided as an argument
if [ -z "$1" ]; then
    echo "Usage: ./launch.sh <ros_distro>"
    echo "Example: ./launch.sh iron"
    exit 1
fi

ROS_DISTRO=$1

# Start ROS app in a new terminal
echo "Starting ROS Application..."
gnome-terminal -- bash -c "
    colcon build --base-paths ros
    source install/setup.bash
    export TURTLEBOT3_MODEL=waffle
    ros2 launch rumros new-world.launch.py
    exec bash"