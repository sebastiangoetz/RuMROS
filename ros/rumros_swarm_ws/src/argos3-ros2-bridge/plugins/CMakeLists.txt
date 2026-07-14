# Part of https://github.com/CPS-Konstanz/argos3-ros2-bridge/tree/dfc12913aa2cf55012c6e221c11e397bfe611881 (modified)
# Original Author: Sindiso Mkhatshwa (https://github.com/einstein07)

### START - Adapted from argos3-examples/CMakeLists.txt

#cmake_minimum_required(VERSION 3.8)
#project(argos3_examples)

# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)

# Set the path where CMake will find additional scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Find the ARGoS package, make sure to save the ARGoS prefix
find_package(ARGoS REQUIRED)
include_directories(${ARGOS_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
# Set ARGoS link dir
link_directories(${ARGOS_LIBRARY_DIRS})
link_libraries(${ARGOS_LDFLAGS})

#pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(ARGOS_PREFIX ${ARGOS_PREFIX} CACHE INTERNAL "")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)

# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
#include(ARGoSCheckQTOpenGL)

# Find the GSL package
find_package(GSL REQUIRED)

# Find Lua
find_package(Lua REQUIRED)

message(STATUS "GSL_INCLUDE_DIR: ${GSL_INCLUDE_DIR}")
message(STATUS "LUA_INCLUDE_DIR: ${LUA_INCLUDE_DIR}")

# Descend into the controllers directory
#add_subdirectory(controllers)

# Descend into the loop_functions directory
#add_subdirectory(loop_functions)

# Descend into the embedding directory
#add_subdirectory(embedding)


### END


#include_directories(controllers)
add_subdirectory(loop_functions)
add_subdirectory(user_functions)
add_subdirectory(bridge)
