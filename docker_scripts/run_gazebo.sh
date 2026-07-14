#!/bin/bash
# This script launches RuMROS with Gazebo inside the docker container
source /app/rumros/docker_scripts/run_common.sh

# Run gradle pre-build tasks
echo "[RuMROS] Running gradle pre-build tasks..."
cd /app/rumros/runtimemodel

bash gradlew updateMqttConfig -Psimulator=gazebo

# Launch MQTT client
echo "[RuMROS] Launching MQTT client..."
cd /app/rumros
echo 'ros2 launch mqtt_client standalone.launch.xml params_file:="$(dirname "$0")/config.yaml"' > runtimemodel/mqtt_client/launch_mqtt_client.sh # Apply patch
bash runtimemodel/mqtt_client/launch_mqtt_client.sh &

# Launch runtime model
bash runtimemodel/gradlew -p runtimemodel run -Psimulator=gazebo &

# Build ROS workspace
echo "[RuMROS] Building ROS workspace..."
cd /app/rumros/
colcon build --base-paths ros --packages-select turtlebot3_gazebo --symlink-install --parallel-workers $(nproc)
source install/setup.bash

# Launch Gazebo (GUI, in foreground)
export TURTLEBOT3_MODEL=waffle;
ros2 launch turtlebot3_gazebo lager.launch.py
