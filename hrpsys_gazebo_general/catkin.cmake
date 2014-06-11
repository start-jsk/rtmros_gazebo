# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_gazebo_general)

find_package(catkin REQUIRED COMPONENTS hrpsys_ros_bridge hrpsys_gazebo_msgs)

find_package(PkgConfig)
pkg_check_modules(omniorb omniORB4 REQUIRED)
pkg_check_modules(omnidynamic omniDynamic4 REQUIRED)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)
pkg_check_modules(openhrp3 openhrp3.1 REQUIRED)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)
catkin_package(CATKIN_DEPENDS hrpsys_ros_bridge hrpsys_gazebo_msgs)

## Build only iob
include_directories(${catkin_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS} ${openhrp3_INCLUDE_DIRS} ${hrpsys_INCLUDE_DIRS})
link_directories(/opt/ros/$ENV{ROS_DISTRO}/lib/ ${openhrp3_LIBRARY_DIRS})
set(ROBOTHARDWARE_SOURCE ${hrpsys_SOURCE_DIR}/src/rtc/RobotHardware)
add_subdirectory(iob)

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/lib/RobotHardware_gazebo.so
   COMMAND cmake -E make_directory ${PROJECT_SOURCE_DIR}/lib
   COMMAND cmake -E rename ${CATKIN_DEVEL_PREFIX}/lib/RobotHardware_gazebo.so ${PROJECT_SOURCE_DIR}/lib/RobotHardware_gazebo.so
   DEPENDS RobotHardware_gazebo)
add_custom_target(hrpsys_gazebo_general_iob ALL
  DEPENDS ${PROJECT_SOURCE_DIR}/lib/RobotHardware_gazebo.so)


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
#endif()

install(DIRECTORY lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
install(TARGETS IOBPlugin SetVelPlugin AddForcePlugin GetVelPlugin
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(PROGRAMS ${CATKIN_DEVEL_PREFIX}/lib/libhrpIo_gazebo.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(PROGRAMS ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/RobotHardwareComp_gazebo
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


