# User functions build file

add_library(ros_bridge_user_functions SHARED
  ros_bridge_qtuser_functions.cpp
  ros_bridge_qtuser_functions.h
)

# Depend on ROS 2 + messages
ament_target_dependencies(ros_bridge_user_functions
  rclcpp
  std_msgs
  geometry_msgs
  rumros_msgs
)

# Link against loop_functions target + ARGoS libs
target_link_libraries(ros_bridge_user_functions
  ros_bridge_loop_functions # Depend on loop functions for parameter imports
  ${ARGOS_LIBRARY_DIRS}
  ${ARGOS_LDFLAGS}
  ${rclcpp_LIBRARIES}
  /usr/local/lib/argos3/libargos3core_simulator.so
  /usr/local/lib/argos3/libargos3plugin_simulator_entities.so
  /usr/local/lib/argos3/libargos3plugin_simulator_qtopengl.so
)

install(
  TARGETS ros_bridge_user_functions
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION include/${PROJECT_NAME}/plugins/user_functions
  FILES_MATCHING PATTERN "*.h"
)
