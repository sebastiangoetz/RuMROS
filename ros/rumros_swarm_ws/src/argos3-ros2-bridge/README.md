# ARGoS3-ROS2-Bridge

This ROS package implements an ARGoS 3 controller, which acts as a bridge between the ARGoS 3 Simulator and ROS 2. It is based on the code of [CPS-Konstanz's argos3-ros2-bridge](https://github.com/CPS-Konstanz/argos3-ros2-bridge/tree/dfc12913aa2cf55012c6e221c11e397bfe611881). Support for the Turtlebot 3 with a lidar sensor was added, as well as user- and loop functions, which can be used to draw on the simulation screen.

## Compiling ARGoS3-ROS2-Bridge

### Requirements
- **ROS2 Jazzy** must be installed and sourced via `source /opt/ros/jazzy/local_setup.bash`
- **ARGoS3** must be installed

### Compiling the Code

You can build this package with colcon, though building `rumros_msgs` first is a prerequisite, as this package depends on its message definitions.

```bash
cd <ros_workspace>
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rumros_msgs argos3_ros2_bridge
```

The `argos3_ros2_bridge` should now be successfully compiled and ready for use. Colcon creates five shared libraries in the `install` folder, one for the robot base functionality, one for each robot implementation and one each for `loop_functions` and `qt_user_functions`. Except for the base library, they have to be included in the ARGoS configuration file by name in order to use them. Refer to the configuration created by the exemplary runtime model in `<ros_workspace>/simulation/main.argos` for more details.
