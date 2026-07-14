# Part of https://github.com/CPS-Konstanz/argos3-ros2-bridge/tree/dfc12913aa2cf55012c6e221c11e397bfe611881 (modified)
# Original Author: Sindiso Mkhatshwa (https://github.com/einstein07)

cmake_minimum_required(VERSION 3.8)
project(argos3_ros2_bridge)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Disable unused parameter warnings
add_compile_options(-Wno-unused-parameter)

# For debugging
#set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rumros_msgs REQUIRED)

# Prefer GLVND (modern OpenGL handling, removes warning)
cmake_policy(SET CMP0072 NEW)

# Export include paths so other packages can find headers
set(EXPORT_INCLUDE_DIRS
  ${ARGOS_INCLUDE_DIRS}
  ${GSL_INCLUDE_DIR}
  ${LUA_INCLUDE_DIR}
)

# Filter out nonexisting dirs
foreach(dir ${EXPORT_INCLUDE_DIRS})
  if(EXISTS ${dir})
    list(APPEND VALID_EXPORT_INCLUDE_DIRS ${dir})
  endif()
endforeach()

ament_export_include_directories(${VALID_EXPORT_INCLUDE_DIRS})

# Export the runtime dependency on messages
ament_export_dependencies(rumros_msgs)


###########
## Build ##
###########

# Added for ARGoS
add_subdirectory(plugins)

ament_package()
