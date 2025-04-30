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
    ros2 launch turtlebot3_gazebo lager.launch.py
    exec bash"

# Ensure MQTT broker is running
if ! systemctl is-active --quiet mosquitto; then
    echo "Starting MQTT broker..."
    sudo systemctl start mosquitto
fi

# Start MQTT client in a new terminal
echo "Starting ROS 2 MQTT Client..."
gnome-terminal -- bash -c "
    sh runtimemodel/mqtt_client/launch_mqtt_client.sh
    exec bash"

# Start runtime model app in a new terminal
echo "Starting Runtime Model Application..."
gnome-terminal -- bash -c "
    export LD_LIBRARY_PATH=/opt/ros/$ROS_DISTRO/lib
    ./runtimemodel/gradlew -p runtimemodel run
    exec bash"

# Start web app in a new terminal
echo "Starting Web Application..."
gnome-terminal -- bash -c "
    python3 webapp/mobileRTM.py
    exec bash"
