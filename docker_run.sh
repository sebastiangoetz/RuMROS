#!/bin/bash
# This script launches the RuMROS container

# Choose simulator: ARGoS or Gazebo
SIMULATOR="${1:-argos}"  # Default is ARGoS

# Map argument to launch script
if [ "$SIMULATOR" == "argos" ]; then
    LAUNCH_SCRIPT="/app/rumros/docker_scripts/run_argos.sh"
elif [ "$SIMULATOR" == "gazebo" ]; then
    LAUNCH_SCRIPT="/app/rumros/docker_scripts/run_gazebo.sh"
else
    echo "Usage: ./docker_run.sh [argos|gazebo]"
    exit 1
fi

xhost +local:docker # Allow X11 forwarding
docker run -it --rm \
    --net=host \
    --ipc=host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $XAUTHORITY:/root/.Xauthority:rw \
    -v $(pwd):/app/rumros \
    -v rumros_venv:/app/venv \
    -v rumros_gradle:/root/.gradle \
    -v rumros_argos_build:/app/rumros/ros/rumros_swarm_ws/build \
    -v rumros_argos_install:/app/rumros/ros/rumros_swarm_ws/install \
    -v rumros_argos_log:/app/rumros/ros/rumros_swarm_ws/log \
    -v rumros_gazebo_build:/app/rumros/build \
    -v rumros_gazebo_install:/app/rumros/install \
    -v rumros_gazebo_log:/app/rumros/log \
    rumros:jazzy \
    bash $LAUNCH_SCRIPT

