# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_gazebo_general)

find_package(catkin REQUIRED COMPONENTS hrpsys_ros_bridge hrpsys_gazebo_msgs)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)
pkg_check_modules(openhrp3 openhrp3.1 REQUIRED)
find_package(collada_urdf_jsk_patch)
catkin_package(CATKIN_DEPENDS hrpsys_ros_bridge hrpsys_gazebo_msgs CFG_EXTRAS compile_robot_model_for_gazebo.cmake)

# set CMAKE_BUILD_TYPE
if(NOT CMAKE_BUILD_TYPE)
  set(
    CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
## Build only gazebo iob
pkg_check_modules(omniorb omniORB4 REQUIRED)
pkg_check_modules(omnidynamic omniDynamic4 REQUIRED)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)
pkg_check_modules(openhrp3 openhrp3.1 REQUIRED)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)
### hotfix for https://github.com/fkanehiro/hrpsys-base/pull/803
list(GET hrpsys_INCLUDE_DIRS 0 hrpsys_first_incdir)
list(APPEND hrpsys_INCLUDE_DIRS ${hrpsys_first_incdir}/hrpsys)
list(APPEND hrpsys_INCLUDE_DIRS ${hrpsys_first_incdir}/hrpsys/idl)
if(EXISTS ${hrpsys_SOURCE_DIR})
  set(ROBOTHARDWARE_SOURCE ${hrpsys_SOURCE_DIR}/src/rtc/RobotHardware)
  set(HRPEC_SOURCE         ${hrpsys_SOURCE_DIR}/src/ec/hrpEC)
elseif(EXISTS ${hrpsys_SOURCE_PREFIX})
  set(ROBOTHARDWARE_SOURCE ${hrpsys_SOURCE_PREFIX}/src/rtc/RobotHardware)
  set(HRPEC_SOURCE         ${hrpsys_SOURCE_PREFIX}/src/ec/hrpEC)
else()
  set(ROBOTHARDWARE_SOURCE ${hrpsys_PREFIX}/share/hrpsys/src/rtc/RobotHardware)
  set(HRPEC_SOURCE         ${hrpsys_PREFIX}/share/hrpsys/src/ec/hrpEC)
endif()
include_directories(${catkin_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS} ${openhrp3_INCLUDE_DIRS} ${hrpsys_INCLUDE_DIRS})
link_directories(${CATKIN_DEVEL_PREFIX}/lib ${hrpsys_PREFIX}/lib ${openhrp3_LIBRARY_DIRS} /opt/ros/$ENV{ROS_DISTRO}/lib/)
add_subdirectory(iob)

add_custom_target(hrpsys_gazebo_general_iob ALL DEPENDS RobotHardware_gazebo)
add_dependencies(hrpsys_gazebo_general_iob hrpsys_gazebo_msgs_gencpp)

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

add_library(IOBPlugin src/IOBPlugin.cpp)
add_dependencies(IOBPlugin hrpsys_gazebo_msgs_gencpp)
install(TARGETS IOBPlugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
add_library(SetVelPlugin src/SetVelPlugin.cpp)
add_dependencies(SetVelPlugin hrpsys_gazebo_msgs_gencpp)
install(TARGETS SetVelPlugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
add_library(AddForcePlugin src/AddForcePlugin.cpp)
add_dependencies(AddForcePlugin hrpsys_gazebo_msgs_gencpp)
install(TARGETS AddForcePlugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
add_library(GetVelPlugin src/GetVelPlugin.cpp)
add_dependencies(GetVelPlugin hrpsys_gazebo_msgs_gencpp)
install(TARGETS GetVelPlugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
add_library(PubTfPlugin src/PubTfPlugin.cpp)
add_dependencies(PubTfPlugin hrpsys_gazebo_msgs_gencpp)
install(TARGETS PubTfPlugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
add_library(ThermoPlugin src/ThermoPlugin.cpp)
add_dependencies(ThermoPlugin hrpsys_gazebo_msgs_gencpp)
install(TARGETS ThermoPlugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
add_library(LIPPlugin src/LIPPlugin.cpp)
add_dependencies(LIPPlugin hrpsys_gazebo_msgs_gencpp)
install(TARGETS LIPPlugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

## Convert robot models
include(${PROJECT_SOURCE_DIR}/cmake/compile_robot_model_for_gazebo.cmake)
if(EXISTS ${hrpsys_ros_bridge_SOURCE_DIR})
  set(hrpsys_ros_bridge_PACKAGE_PATH ${hrpsys_ros_bridge_SOURCE_DIR})
elseif(EXISTS ${hrpsys_ros_bridge_SOURCE_PREFIX})
  set(hrpsys_ros_bridge_PACKAGE_PATH ${hrpsys_ros_bridge_SOURCE_PREFIX})
else()
  set(hrpsys_ros_bridge_PACKAGE_PATH ${hrpsys_ros_bridge_PREFIX}/share/hrpsys_ros_bridge)
endif()
generate_gazebo_urdf_file(${hrpsys_ros_bridge_PACKAGE_PATH}/models/SampleRobot.dae)
add_custom_target(all_robots_compile ALL DEPENDS ${compile_urdf_robots})

## install
install(DIRECTORY launch scripts worlds config robot_models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
install(PROGRAMS setup.sh DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
