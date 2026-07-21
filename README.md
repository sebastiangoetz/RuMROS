# RuMROS

RuMROS is a runtimemodel driven robotic application with an adaptive GUI. It supports control and simulation (Gazebo) of individual robots, as well as creating swarm applications and simulating swarm behavior via [ARGoS 3](https://github.com/ilpincy/argos3).

RuMROS consists of 3 main modules:

- ROS-backend node(s)
- A meta-model with a generated runtime backend
- A web frontend for the runtime model

The application is partially generated from the runtime meta-model. This includes the web frontend, which is fully generic, as well as runtime model backend code and most interface parameters. The Webapp is written in Flask and uses Socket.IO for the constant connection beteween backend and frontend. The runtime meta-model is specified in JastAdd, enabling automatic backend code generation as well as easy extension by the user.

RuMROS comes with support for two main ways of programming robots: Fully runtime-model-based control, as well as swarm programming. In the former case, Gazebo is used for simulation and the runtime model application controls the robots directly via ROS topics. This allows the user to fully define robot's behavior  in JastAdd. A generic ROS node is provided for each robot type (currently supported are turtlebots and drones, rovers are WIP) that simply steers the robots and sends sensor information to the runtime model.

When swarm programming is desired, RuMROS operates differently, due to how swarms operate. As there is no central instance controlling each robot's behavior in contrast to the aforementioned approach, each individual robot has to have a certain 'skillset'. RuMROS uses the same ROS node on every swarm member. It acts as a state machine, which is switched via a control topic that is unique to a swarm group. Different states implement different behavior, and the node comes with pre-defined swarm behavior blocks that can be called from the runtime model. This is the previously mentioned 'skillset', which each robot has. The runtime model only monitors robot states and switches between behaviors. This still allows for global control over swarm actions while keeping localized behavior and thus makes RuMROS a hybrid approach to swarm programming. 

## Demo & Interactive testing

A demo video showcasing functionality is available on [YouTube](https://youtu.be/qfai9XeWDPs). A pre-built docker container with interactive runtime environment can be launched on [AICOR-EASE](https://binder.intel4coro.de/v2/gh/akassuba/rumros-binder/HEAD?labpath=notebooks%2Finstructions.ipynb). Click the link or the button below to launch it.

[![Binder](https://binder.intel4coro.de/badge_logo.svg)](https://binder.intel4coro.de/v2/gh/akassuba/rumros-binder/HEAD?labpath=notebooks%2Finstructions.ipynb)

## Installation

RuMROS requires ROS 2 Jazzy, which is available natively on Ubuntu 24.04, so this is the recommended OS to use. To install RuMROS, first clone the git repository and initialize submodules:

```bash
git clone https://git-st.inf.tu-dresden.de/sgoetz/rumros.git
cd rumros
git submodule update --init --recursive
```

### Installation with Docker

RuMROS can be built inside a Docker container, which is designed to mount the cloned repository  at `/app/rumros`, such that developers can work with the code inside the repository while installing dependencies for ARGoS, ROS, etc. in a Docker image. RuMROS comes with a pre-configured docker image for ROS Jazzy, which can be built with:

```bash
bash docker_build.sh
```

As the container requires GUI support for ARGoS and Gazebo, X11 forwarding has to be supported on the system for development with Docker.

### Manual Installation

Then, install Python 3.12 and Java 21:

```bash
sudo apt install python3.12 python3.12-venv python3-pip python3-sklearn python3-matplotlib python3-numpy openjdk-21-jdk
```

Next, install ROS2 Jazzy by following the Installation Guide. Additional required packages can be installed by running:

```bash
sudo apt install ros-jazzy-desktop ros-dev-tools ros-jazzy-cartographer ros-jazzy-cartographer-ros ros-jazzy-mqtt-client ros-jazzy-nav2-bringup ros-jazzy-nav2-lifecycle-manager ros-jazzy-navigation2 ros-jazzy-ros-gz ros-jazzy-slam-toolbox ros-jazzy-tf-transformations ros-jazzy-tf2 ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-msgs ros-jazzy-turtlebot3-simulations ros-jazzy-crazyflie ros-jazzy-crazyflie-interfaces ros-jazzy-xacro python3-colcon-common-extensions mosquitto
```

Finally, for convenience and in order to avoid repeating these commands before every launch, append the following lines to ~/.bashrc with:

```bash
echo 'export ROS_DISTRO=jazzy' >> ~/.bashrc && echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc
```

The web frontend is written in Python and requires some dependencies. It is recommended to set up a virtual environment and install them there:

```bash
cd webapp
python -m venv venv
pip install Flask
pip install -r requirements.txt
```

This sets up the virtual environment in webapp/venv. Make sure to correctly set this path in the launch scripts `launch_with_gazebo.sh` and `launch_with_argos.sh`.

#### ARGoS Installation

Install ARGoS according to the instructions on the website (the latest binary is offered for Ubuntu 22.04, but still works for Ubuntu 24.04). The GLUT library also has to be installed:

```bash
sudo apt install freeglut3-dev libgsl-dev
```

On Ubuntu 24.04, argos3 will complain that libglut.so.3 cannot be found. It is installed but named differently and therefore has to be linked in the folder where ARGoS expects it:

```bash
cd /usr/lib/x86_64-linux-gnu
ln -s libglut.so.3.12.0 libglut.so.3
```

With updates to Ubuntu, the concrete GLUT version may differ. Adjust it according to any potential errors reported by ARGoS.

Finally, ARGoS plugins for supported robots need to be installed. Install the Turtlebot3 plugin for ARGoS according to the instructions from [GitHub](https://github.com/ilpincy/argos3-turtlebot3). In Ubuntu 24.04, the installation directory is `/usr/lib` instead of `/usr/local/lib`, so a symlink has to be created in order for ARGoS to find the libraries:

```bash
sudo ln -s /usr/lib/argos3 /usr/local/lib/argos3
```

For the Crazyflie 2.1, a plugin is available on University of Luxembourg's [GitLab](https://gitlab.com/uniluxembourg/snt/pcog/adars/crazyflie). Compile according to their instructions and make sure to use the `-DCMAKE_INSTALL_PREFIX=/usr` flag to install to `/usr` instead of `/usr/local`.

#### Optional: MQTT-CLient Launch Adjustment

On newer versions of the mqtt-client package, the path for the launch configuration may vary. If you get related errors on launch in the MQTT-client terminal, try locating the launch configuration file in `/opt/ros/$ROS_DISTRO/share/mqtt_client`. In older versions, it is located in this directory as `standalone.launch.ros2.xml`, whereas in more recent versions, it is located in `launch/standalone.launch.xml`. If you find the file to be in the launch folder, change the mqtt_client configuration as follows:

```bash
cd rumros
echo 'ros2 launch mqtt_client standalone.launch.xml params_file:="$(dirname "$0")/config.yaml"' > runtimemodel/mqtt_client/launch_mqtt_client.sh
```

## Usage

RuMROS can be launched either via Docker or the manual installation, for both methods, launch scripts are provided. Building the individual application parts is included in these launch scripts. Refer to them for detailed steps on how to do so, as well as how to manually launch the individual components.

After launch, the selected simulator will open. With ARGoS, the simulation has to be manually started by pressing the play button on the top left. To control the robots, use the web interface provided on [127.0.0.1:5000](http://127.0.0.1:5000). On the left side, it displays multiple tables with the nodes specified in the runtime meta-model and their current state. On the right side, Actions can be triggered on individual robots or swarm groups. Further tabs are provided for viewing tables and actions separately, as well as to view, start and stop the robot behavior model, which uses a state machine-like controller to steer robots, and can be programmed in JastAdd.

### Launching with Docker

When launching with Docker, the simulator can be chosen by providing a launch argument (default: ARGoS):

```bash
bash docker_run.sh [argos|gazebo]
```

The web interface is accessible from a regular browser outside the container at [127.0.0.1:5000](http://127.0.0.1:5000).

### Launching a manual installation

RuMROS can be run with either Gazebo or ARGoS. As this involves starting all of the application parts (multiple launch commands) it is recommended to use either one of the provided launch scripts in the project's root folder:

```bash
bash launch_with_gazebo.sh
bash launch_with_argos.sh
```

The web interface is accessible from a regular browser at [127.0.0.1:5000](http://127.0.0.1:5000).
