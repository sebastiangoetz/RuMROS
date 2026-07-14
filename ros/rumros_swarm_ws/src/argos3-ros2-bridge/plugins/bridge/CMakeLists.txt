# Part of https://github.com/CPS-Konstanz/argos3-ros2-bridge/tree/dfc12913aa2cf55012c6e221c11e397bfe611881 (modified)
# Original Author: Sindiso Mkhatshwa (https://github.com/einstein07)

#find_package(argos3_ros2_bridge REQUIRED)
find_package(rumros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(crazyflie_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Python REQUIRED COMPONENTS Development)

# === Base helper library ===
add_library(robot_bridge_base SHARED
  base/ros_bridge_singleton.cpp
  base/robot_bridge_base.cpp
)

target_include_directories(robot_bridge_base
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/base>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}/plugins/bridge/base>
)

ament_target_dependencies(robot_bridge_base
  rclcpp
  geometry_msgs
  std_msgs
  rumros_msgs
)

# === Foot-Bot bridge ===

add_library(footbot_bridge SHARED
  footbot_bridge.cpp
)

target_link_libraries(footbot_bridge
  robot_bridge_base
  ${ARGOS_LIBRARY_DIRS}
  ${ARGOS_LDFLAGS}
  ${rclcpp_LIBRARIES}
  /usr/local/lib/argos3/libargos3plugin_simulator_footbot.so
  /usr/local/lib/argos3/libargos3plugin_simulator_genericrobot.so
  /usr/local/lib/argos3/libargos3core_simulator.so
)

ament_target_dependencies(footbot_bridge
  rclcpp
  geometry_msgs
  std_msgs
  ARGoS
  rumros_msgs
)

# === Turtlebot3 bridge ===

add_library(turtlebot_bridge SHARED
  turtlebot_bridge.cpp
)

target_link_libraries(turtlebot_bridge
  robot_bridge_base
  ${ARGOS_LIBRARY_DIRS}
  ${ARGOS_LDFLAGS}
  ${rclcpp_LIBRARIES}
  /usr/local/lib/argos3/libargos3plugin_simulator_turtlebot3.so
  /usr/local/lib/argos3/libargos3plugin_simulator_genericrobot.so
  /usr/local/lib/argos3/libargos3core_simulator.so
)

ament_target_dependencies(turtlebot_bridge
  rclcpp
  geometry_msgs
  std_msgs
  ARGoS
  rumros_msgs
)

# === Crazyflie bridge ===
set(CFFW_ROOT "${CMAKE_SOURCE_DIR}/cf_firmware")

add_library(cffirmware SHARED IMPORTED)

set_target_properties(cffirmware PROPERTIES
    IMPORTED_LOCATION "${CFFW_ROOT}/lib/_cffirmware.cpython-312-x86_64-linux-gnu.so"
    INTERFACE_INCLUDE_DIRECTORIES "${CFFW_ROOT}/include"
)

add_library(crazyflie_bridge SHARED
  base/ros_bridge_singleton.cpp
  crazyflie_bridge.cpp
)

target_link_libraries(crazyflie_bridge
  #robot_bridge_base # Not a 2-wheeled robot, so base not used for now (TODO: Separate base classes for 2D and 3D movement capable robots)
  ${ARGOS_LIBRARY_DIRS}
  ${ARGOS_LDFLAGS}
  ${rclcpp_LIBRARIES}
  /usr/local/lib/argos3/libargos3plugin_simulator_crazyflie.so
  /usr/local/lib/argos3/libargos3plugin_simulator_genericrobot.so
  /usr/local/lib/argos3/libargos3core_simulator.so
  cffirmware
  Python::Python
)

ament_target_dependencies(crazyflie_bridge
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  ARGoS
  rumros_msgs
  std_srvs
  crazyflie_interfaces
)

# === Common ===
install(
    TARGETS robot_bridge_base footbot_bridge turtlebot_bridge crazyflie_bridge
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib
)

# Add header files in case other libraries want to use them
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION include/${PROJECT_NAME}/plugins/bridge
  FILES_MATCHING PATTERN "*.h"
)

