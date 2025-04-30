# Runtime Model driven ROS (RuMROS)

RuMROS is a model-driven software development approach for robotic applications in the context of ROS2.
Developers model their application as a reference attribute grammar (i.e., a metamodel) and specify the behavior as attributes of the grammar.
We are working on an overview paper to describe the principles of RuMROS in detail.

The following figure describes the workflow for developers of robotic applications and the general architecture of RuMROS.
<img src="/docs/figures/generation.png" width="300"/>

Basic features of robotic applications like driving or gripping still have to be written in ROS. RuMROS allows to orchestrate these features into complex applications on a non-technical level. For this, developers metamodel their application and specify the application logic using terms from their metamodel instead of technical ROS2 concepts. RuMROS generates a webapp from the application specification that allows to control the application. The following figure shows an example of a generated application dashboard.

<img src="/docs/figures/screenshot_webapp.png" width="500">

The behavior is specified using attributes of a reference attribute grammar (RAG). The RAG is connected with ROS via RAGConnect and MQTT. This concept is depicted in the following figure.

<img src="/docs/figures/mape_k_loop.png" width="500">

## Requirements:
- linux 22.04
- python 3.10
- ros2 (sudo apt install ros-jazzy-desktop, sudo apt install ros-dev-tools)
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



## Credits
This project is lead by Sebastian Götz. The first version of RuMROS was developed by Adrian Scholze. The current version has been developed by Edgar Solf.
