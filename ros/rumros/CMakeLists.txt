################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(rumros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ros_gz REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)

################################################################################
# Build
################################################################################
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "ros_gz"
  "slam_toolbox"
)

set(EXEC_NAME "turtlebot3_drive")
add_executable(${EXEC_NAME} src/turtlebot3_drive.cpp)
ament_target_dependencies(${EXEC_NAME} ${dependencies})

set(EXEC_NAME2 "map_saver")
add_executable(${EXEC_NAME2} src/map_saver.cpp)
ament_target_dependencies(${EXEC_NAME2} ${dependencies})

set(EXEC_NAME3 "pose_offsetter")
add_executable(${EXEC_NAME3} src/pose_offsetter.cpp)
ament_target_dependencies(${EXEC_NAME3} ${dependencies})

################################################################################
# Install
################################################################################
install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${EXEC_NAME2}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${EXEC_NAME3}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch models params rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(ros_gz)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(slam_toolbox)
ament_package()
