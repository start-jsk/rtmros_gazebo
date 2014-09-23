# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_gazebo_general)

find_package(catkin REQUIRED COMPONENTS hrpsys_ros_bridge hrpsys_gazebo_msgs)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)
pkg_check_modules(openhrp3 openhrp3.1 REQUIRED)
catkin_package(CATKIN_DEPENDS hrpsys_ros_bridge hrpsys_gazebo_msgs)

## Build only gazebo iob
find_package(PkgConfig)
pkg_check_modules(omniorb omniORB4 REQUIRED)
pkg_check_modules(omnidynamic omniDynamic4 REQUIRED)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)
pkg_check_modules(openhrp3 openhrp3.1 REQUIRED)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)
if(EXISTS ${hrpsys_SOURCE_DIR})
  set(ROBOTHARDWARE_SOURCE ${hrpsys_SOURCE_DIR}/src/rtc/RobotHardware)
else()
  set(ROBOTHARDWARE_SOURCE ${hrpsys_PREFIX}/share/hrpsys/src/rtc/RobotHardware)
endif()
include_directories(${catkin_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS} ${openhrp3_INCLUDE_DIRS} ${hrpsys_INCLUDE_DIRS})
link_directories(${CATKIN_DEVEL_PREFIX}/lib ${hrpsys_PREFIX}/lib ${openhrp3_LIBRARY_DIRS} /opt/ros/$ENV{ROS_DISTRO}/lib/)
add_subdirectory(iob)

add_custom_target(hrpsys_gazebo_general_iob ALL DEPENDS RobotHardware_gazebo)

## Gazebo plugins
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

include_directories( ${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS} ${openhrp3_INCLUDE_DIRS})
link_directories( ${GAZEBO_LIBRARY_DIRS} ${openhrp3_LIBRARY_DIRS})

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/plugins)

#if ($ENV{ROS_DISTRO} STREQUAL "groovy")
add_library(IOBPlugin src/IOBPlugin.cpp)
add_library(SetVelPlugin src/SetVelPlugin.cpp)
add_dependencies(hrpsys_gazebo_general_iob hrpsys_gazebo_msgs_gencpp)
add_dependencies(IOBPlugin hrpsys_gazebo_msgs_gencpp)
add_dependencies(SetVelPlugin hrpsys_gazebo_msgs_gencpp)
add_library(AddForcePlugin src/AddForcePlugin.cpp)
add_library(GetVelPlugin src/GetVelPlugin.cpp)
add_dependencies(AddForcePlugin hrpsys_gazebo_msgs_gencpp)
add_dependencies(GetVelPlugin hrpsys_gazebo_msgs_gencpp)
add_library(ThermoPlugin src/ThermoPlugin.cpp)
add_dependencies(ThermoPlugin hrpsys_gazebo_msgs_gencpp)
#endif()

