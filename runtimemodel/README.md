# Runtime Model driven ROS with RAGConnect

This is a Java application for a runtimemodel for a ROS2 application. On the basis of this runtimemodel a webapp can be launched, that dynamically displays the content of the runtimemodel in a form of tables and allows the user to perform actions.
The current runtimemodel is configured for the [RuMROS-project](https://git-st.inf.tu-dresden.de/sgoetz/rumros/-/blob/main/README.md?ref_type=heads), with which it can run out of the box without any configuration.

![](/docs/images/demo.png)

The above screenshot shows the generated Dashboard as a webapp.

## Requirements:
- **Linux 22.04**
- **ROS2** (tested with ROS2 Iron)
- **ROS2 mqtt_client package**
- **Java** 11 or later
- **Python** 3.0 or later

## Launch Application

1. Start ROS2 Application (like rumros)
2. Start MQTT-Client ``mqtt_client/launch_mqtt_client.sh``
3. Start ModelMain
4. Start Webapp ``python3 webapp/mobileRTM.py``
5. Open ``localhost:5000`` in any browser

## How to use the Application:
- choose an action from the right side
- press "Start" to run action
- view result on the tables on the left side

## Current Issues
- Picking up packages is not implemented yet
