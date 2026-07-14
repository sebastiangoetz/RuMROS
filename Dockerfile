# Base file for ROS Jazzy
FROM osrf/ros:jazzy-desktop-full
EXPOSE 1883

# Set ROS distribution to use
ENV ROS_DISTRO='jazzy'
RUN echo 'export ROS_DISTRO=$ROS_DISTRO' >> ~/.bashrc && echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc

# Install Requirements
RUN apt-get update && apt-get install apt-utils -y
RUN apt-get install -y \
    nano \
    python3.12 \
    python3.12-venv \
    python3-pip \
    python3-sklearn \
    python3-matplotlib \
    python3-numpy \
    openjdk-21-jdk \
    ros-"$ROS_DISTRO"-desktop \
    ros-dev-tools \
    ros-"$ROS_DISTRO"-cartographer \
    ros-"$ROS_DISTRO"-cartographer-ros \
    ros-"$ROS_DISTRO"-mqtt-client \
    ros-"$ROS_DISTRO"-nav2-bringup \
    ros-"$ROS_DISTRO"-nav2-lifecycle-manager \
    ros-"$ROS_DISTRO"-navigation2 \
    ros-"$ROS_DISTRO"-ros-gz \
    ros-"$ROS_DISTRO"-slam-toolbox \
    ros-"$ROS_DISTRO"-tf-transformations \
    ros-"$ROS_DISTRO"-tf2 \
    ros-"$ROS_DISTRO"-turtlebot3 \
    ros-"$ROS_DISTRO"-turtlebot3-msgs \
    ros-"$ROS_DISTRO"-turtlebot3-simulations \
    ros-"$ROS_DISTRO"-crazyflie \
    ros-"$ROS_DISTRO"-crazyflie-interfaces \
    ros-"$ROS_DISTRO"-xacro \
    python3-colcon-common-extensions \
    mosquitto \
    freeglut3-dev \
    libgsl-dev

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

# Create Python venv
RUN python3 -m venv /app/venv

# Install ARGoS
COPY ./dependencies /app/dependencies
RUN apt-get install -y /app/dependencies/argos3_simulator-3.0.0-x86_64-beta59.deb

# Link libglut for ARGoS (ARGoS compiled for Ubuntu 22.04, where this path is different)
RUN cd /usr/lib/x86_64-linux-gnu \
    && ln -s libglut.so.3.12.0 libglut.so.3

# Install Turtlebot plugin for ARGoS
RUN cd /app \
    && git clone https://github.com/ilpincy/argos3-turtlebot3.git \
    && cd argos3-turtlebot3 \
    && mkdir build \
    && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release ../src -DCMAKE_INSTALL_PREFIX=/usr \
    && make \
    && make install \
    && ln -s /usr/lib/argos3 /usr/local/lib/argos3

# Install Crazyflie plugin for ARGoS
RUN cd /app \
    && git clone https://gitlab.com/uniluxembourg/snt/pcog/adars/crazyflie.git \
    && cd crazyflie \
    && mkdir build \
    && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release ../src -DCMAKE_INSTALL_PREFIX=/usr \
    && make \
    && make install
