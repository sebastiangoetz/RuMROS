#!/bin/bash
# This script launches the application with the ARGoS3 simulator

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

# Add ROS2 setup.bash
source /opt/ros/$RDISTRO/local_setup.bash

# Faster RMW middleware (required for >5 robots)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export TRACETOOLS_RUNTIME_DISABLE=true

# Generate MQTT client config, ARGoS config and ROS Launch config
cd runtimemodel
./gradlew updateMqttConfig -Psimulator=argos
./gradlew generateArgosConfig -Psimulator=argos
./gradlew generateRosLaunchConfig -Psimulator=argos
cd ..

# Ensure MQTT broker is running
if ! systemctl is-active --quiet mosquitto; then
    echo "Starting MQTT broker..."
    sudo systemctl start mosquitto
fi

# Start MQTT client in a new terminal
echo "Starting ROS 2 MQTT Client..."
launch_terminal "sh runtimemodel/mqtt_client/launch_mqtt_client.sh" "MQTT Client"

# Start web app in a new terminal
echo "Starting Web Application..."
launch_terminal "$PYTHON webapp/mobileRTM.py" "Web UI"

# Build & source ROS workspace
echo "Building ROS Application..."
cd ros/rumros_swarm_ws
colcon build
source install/setup.bash

# Start Argos
launch_terminal "argos3 -c simulation/main.argos" "ARGoS 3"

# Start ROS nodes
launch_terminal "ros2 launch swarm_controller swarm_launch.py swarm_config:=$(pwd)/config/swarm_config.json type_config:=$(pwd)/config/type_config.json" "ROS Launch"

cd ../..
sleep 10

# Start runtime model app in a new terminal
echo "Starting Runtime Model Application..."
launch_terminal "export LD_LIBRARY_PATH=/opt/ros/$RDISTRO/lib:/opt/ros/$RDISTRO/lib/x86_64-linux-gnu:\$LD_LIBRARY_PATH;
                 ./runtimemodel/gradlew -p runtimemodel run -Psimulator=argos" "Runtime Model"

# Run main terminal until interrupted
echo "Script running. Press Ctrl+C to close all terminals."
while true; do sleep 1; done
