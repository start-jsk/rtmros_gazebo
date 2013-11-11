# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_gazebo_general)

find_package(catkin REQUIRED COMPONENTS hrpsys_ros_bridge hrpsys_gazebo_msgs)

catkin_package(CATKIN_DEPENDS hrpsys_ros_bridge hrpsys_gazebo_msgs)

## Build hrpsys for gazebo
# execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.hrpsys-base
#                 RESULT_VARIABLE _make_failed)
# if (_make_failed)
#   message(FATAL_ERROR "Build of hrpsys failed")
# endif(_make_failed)

## Gazebo plugins
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

include_directories( ${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories( ${GAZEBO_LIBRARY_DIRS} )

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/plugins)

add_library(IOBPlugin src/IOBPlugin.cpp)
add_library(AddForcePlugin src/AddForcePlugin.cpp)
add_library(SetVelPlugin src/SetVelPlugin.cpp)
add_library(GetVelPlugin src/GetVelPlugin.cpp)

