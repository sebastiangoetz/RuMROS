cmake_minimum_required(VERSION 3.5)
project(ros2_serialization_utils)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)

find_package(JNI REQUIRED)

add_library(ros2_serialization_utils SHARED src/ros2_serialization_utils.cpp)
ament_target_dependencies(ros2_serialization_utils rclcpp geometry_msgs nav_msgs)

install(TARGETS
  ros2_serialization_utils
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${JNI_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_package()

