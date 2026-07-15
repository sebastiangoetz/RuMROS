#!/bin/bash
# This script launches the application with the Gazebo simulator

# Load launch functions
source launch_functions.sh

# Call cleanup on SIGINT (ctrl+c)
trap cleanup SIGINT

# Check if a ROS 2 distribution was provided as an argument
if [ -n "$1" ]; then
    RDISTRO=$1
elif [ -n "$ROS_DISTRO" ]; then
    RDISTRO=$ROS_DISTRO
else
    echo "Usage: ./launch.sh [<ros_distro>] [<python_path>]"
    echo "Example: ./launch.sh iron ~/venv/ros/bin/python"
    echo "If <ros_distro> is not provided, make sure the ROS_DISTRO environment variable is set to the name of the ROS2 distro to use."
    exit 1
fi

PYTHON=${2:-$(pwd)/webapp/venv/bin/python3}
export TRACETOOLS_RUNTIME_DISABLE=true

# Add ROS2 setup.bash
source /opt/ros/$RDISTRO/local_setup.bash

# Generate MQTT client configuration
echo "Generating MQTT client configuration..."
cd runtimemodel
./gradlew updateMqttConfig -Psimulator=gazebo
cd ..

# Build ROS app in a new terminal
echo "Building ROS Application..."
launch_terminal "
    colcon build --base-paths ros --packages-select turtlebot3_gazebo;
    source install/setup.bash;
    export TURTLEBOT3_MODEL=waffle;
    ros2 launch turtlebot3_gazebo lager.launch.py" "ROS Launch"

# Ensure MQTT broker is running
if ! systemctl is-active --quiet mosquitto; then
    echo "Starting MQTT broker..."
    sudo systemctl start mosquitto
fi

# Start MQTT client in a new terminal
echo "Starting ROS 2 MQTT Client..."
launch_terminal "
    sh runtimemodel/mqtt_client/launch_mqtt_client.sh" "MQTT Client"

# Start runtime model app in a new terminal
echo "Starting Runtime Model Application..."
launch_terminal "
    export LD_LIBRARY_PATH=/opt/ros/$RDISTRO/lib:/opt/ros/$RDISTRO/lib/x86_64-linux-gnu:\$LD_LIBRARY_PATH;
    ./runtimemodel/gradlew -p runtimemodel run -Psimulator=gazebo" "Runtime Model"

# Start web app in a new terminal
echo "Starting Web Application..."
launch_terminal "
    $PYTHON webapp/mobileRTM.py" "Web UI"

# Run main terminal until interrupted
echo "Script running. Press Ctrl+C to close all terminals."
while true; do sleep 1; done
