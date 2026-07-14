add_library(ros_bridge_loop_functions SHARED
  ros_bridge_loop_functions.cpp
  ros_bridge_loop_functions.h
)

# Link against ROS 2 message package + rclcpp + ARGoS
ament_target_dependencies(ros_bridge_loop_functions
  rclcpp
  std_msgs
  geometry_msgs
  rumros_msgs
)

target_link_libraries(ros_bridge_loop_functions
  ${ARGOS_LIBRARY_DIRS}
  ${ARGOS_LDFLAGS}
  ${rclcpp_LIBRARIES}
  /usr/local/lib/argos3/libargos3plugin_simulator_genericrobot.so
  /usr/local/lib/argos3/libargos3core_simulator.so
  /usr/local/lib/argos3/libargos3plugin_simulator_dynamics2d.so
  /usr/local/lib/argos3/libargos3plugin_simulator_entities.so
  /usr/local/lib/argos3/libargos3plugin_simulator_media.so
)

install(
  TARGETS ros_bridge_loop_functions
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION include/${PROJECT_NAME}/plugins/loop_functions
  FILES_MATCHING PATTERN "*.h"
)
