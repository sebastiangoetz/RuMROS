#!/bin/bash
# Common docker launch functions for both ARGoS and Gazebo
# Environment setup
export ROS_DISTRO=jazzy
export LD_LIBRARY_PATH=/opt/ros/$ROS_DISTRO/lib:/opt/ros/$ROS_DISTRO/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "[RuMROS] Container startup..."
source /opt/ros/$ROS_DISTRO/setup.bash

# Python venv setup
if [ ! -f "/app/venv/.rumros_deps_installed" ]; then
    echo "[RuMROS] Setting up Python environment..."

    python3 -m venv /app/venv
    source /app/venv/bin/activate

    pip install --upgrade pip
    pip install -r /app/rumros/webapp/requirements.txt

    touch /app/venv/.rumros_deps_installed
else
    echo "[RuMROS] Python deps already installed."
    source /app/venv/bin/activate
fi

# Launch MQTT broker
if ! pgrep mosquitto > /dev/null; then
    echo "[RuMROS] Starting MQTT broker..."
    mosquitto -d
else
    echo "[RuMROS] MQTT broker already running."
fi

# Launch web UI
echo "[RuMROS] Launching web interface..."
cd /app/rumros
source /app/venv/bin/activate
python3 webapp/mobileRTM.py &
deactivate