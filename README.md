# Runtime Model driven ROS (RuMROS)

RuMROS is a runtimemodel driven robotic application with an adaptive GUI. The application can be partially generated from a runtimemetamodel. The webapp frontend is completely generated. The backend is general and does not need to be generated. The Webapp is written in Flask and uses Socket.IO for the constant connection beteween backend and frontend. In the runtimemodel application only the runtimemodel is generated. The metamodel was created with EMF. The Pyecorgen (https://github.com/pyecore/pyecoregen) generator was used as the basis for the generator. It generates the runtimemodel from the .pyecore file and was extended in this work to generate the webapp frontend aswell. 

As an example, the Turtlebot robot can be controlled in the Gazebo robot simulation environment. The Communication between ROS and runtimemodel application is realized with ROS Topics. The Communication between the runtimemodel application and the webapp is realized with Python UDP Sockets.

![](/docs/images/demo.png)

The above screenshot show The simulation environment Gazebo Harmonic with the generated Dashboard as Webapp over it and the consoles of the runtime model at the right top and the webapp at the right bottom.

## Requirements:
- linux 22.04
- python 3.10
- ros2 (sudo apt install ros-iron-desktop, sudo apt install ros-dev-tools)
- gazebo harmonic (sudo apt install gz)
- gazebo ros (sudo apt install ros-iron-ros-gzharmonic)
- colcon (sudo apt install python3-colcon-common-extensions)
- flask (pip install Flask pip install -r requirements.txt )
- ros2 mqtt_client package
- java 11 or later
- mqtt broker (like moquitto, installation steps below)

## Launch Application with Gazebo Harmonic

### Start everything at once

1. if not done before, make launch script executable: ``chmod +x launch.sh``
2. run launch script: ``./launch.sh <distro>`` (e.g. ``/launch.sh iron``)

### Start in steps

1. go to ``ros`` folder
2. ``colcon build``
3. ``source install/setup.bash``
4. ``export TURTLEBOT3_MODEL=waffle``
5. ``ros2 launch turtlebot3_gazebo lager.launch.py``
6. verify mqtt broker is running: ``sudo systemctl status mosquitto`` and start if not running ``sudo systemctl start mosquitto``
7. start mqtt-client: ``sh runtimemodel/mqtt_client/launch_mqtt_client.sh``
8. set path to your ROS 2 distribution: ``export LD_LIBRARY_PATH=/opt/ros/<distro>/lib``
9. start runtime model: ``./runtimemodel/gradlew -p runtimemodel run``
10. start webapp: ``python3 webapp/mobileRTM.py``
11. open ``localhost:5000`` in any browser
12. to open RViz2 for namespace "robot_1": ``rviz2 --ros-args -r /tf:=/robot_1/tf -r /tf_static:=/robot_1/tf_static``



## Setup with Ubuntu 24, ROS2 Jazzy and Gazebo Harmonic
- install ubuntu 24, comes with python 3.12 by default
- ``sudo apt install ros-jazzy-desktop`` [docs](https://docs.ros.org/en/jazzy/Installation.html)
- ``sudo apt install ros-dev-tools``
- ``sudo apt install gz-harmonic`` [docs](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- ``sudo apt install ros-jazzy-ros-gz``
- ``sudo apt install python3-colcon-common-extensions``
- create a python venv: ``python3 -m venv ~/venv`` (or any other folder you prefer)
- ``~/venv/bin/pip install -r webapp/requirements.txt``
- use ``~/venv/bin/python`` to run webapp/mobileRTM.py

### Install Mosquitto as MQTT broker
- ``sudo apt install -y mosquitto mosquitto-clients``
- ``sudo systemctl enable mosquitto``
- ``sudo systemctl start mosquitto``
- config located at /etc/mosquitto/mosquitto.conf



## How to use the Application:
- choose a Goal Area and a Goal State for a Robot
- press "Start Goal Adaption" 
- Robot moves to the Goal coordinates
- To stop the robot (also during Goal adaption) choose State "waiting" and press "Start Goal Adaption" Button

## Current Issues
- Robot's odometry is bad, the target's are barely reached, because the robot does not know exactly where it is

## How to use the Generator
- run cli.py in pyecoregen
- type in source file (like robotModelStandard.ecore or robotModelExtension.ecore) and output path
- press enter to start
- Replace the files in runtimemodel/model and webapp/templates with the output files from the Generator
