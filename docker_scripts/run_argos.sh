#!/bin/bash
# This script launches RuMROS with ARGoS inside the docker container
source /app/rumros/docker_scripts/run_common.sh

# Run gradle pre-build tasks
echo "[RuMROS] Running gradle pre-build tasks..."
cd /app/rumros/runtimemodel

bash gradlew updateMqttConfig -Psimulator=argos
bash gradlew generateArgosConfig -Psimulator=argos
bash generateRosLaunchConfig -Psimulator=argos

# Launch MQTT client
echo "[RuMROS] Launching MQTT client..."
cd /app/rumros
echo 'ros2 launch mqtt_client standalone.launch.xml params_file:="$(dirname "$0")/config.yaml"' > runtimemodel/mqtt_client/launch_mqtt_client.sh # Apply patch
bash runtimemodel/mqtt_client/launch_mqtt_client.sh &

# Launch runtime model
echo "[RuMROS] Launching runtime model..."
bash runtimemodel/gradlew -p runtimemodel run -Psimulator=argos &

# Build ROS workspace
echo "[RuMROS] Building ROS workspace..."
WS="/app/rumros/ros/rumros_swarm_ws"
cd "$WS"

colcon build --symlink-install --parallel-workers $(nproc)
source install/setup.bash

# Launch ROS robot controllers
echo "[RuMROS] Launching ROS robot controllers..."
cd "$WS"
ros2 launch swarm_controller swarm_launch.py \
     swarm_config:="/app/rumros/ros/rumros_swarm_ws/config/swarm_config.json" \
     type_config:="/app/rumros/ros/rumros_swarm_ws/config/type_config.json" &

# Launch ARGoS (GUI, in foreground)
echo "[RuMROS] Launching ARGoS simulator..."
argos3 -c simulation/main.argos
