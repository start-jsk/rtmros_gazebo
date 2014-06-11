# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_gazebo_general)

find_package(catkin REQUIRED COMPONENTS hrpsys_ros_bridge hrpsys_gazebo_msgs)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)
pkg_check_modules(openhrp3 openhrp3.1 REQUIRED)
catkin_package(CATKIN_DEPENDS hrpsys_ros_bridge hrpsys_gazebo_msgs)

## Build only iob
add_custom_command(OUTPUT ${CATKIN_DEVEL_PREFIX}/share/hrpsys_gazebo_general/lib
  COMMAND cmake -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/iob
  COMMAND cmake -E chdir ${CMAKE_CURRENT_BINARY_DIR}/iob cmake ${PROJECT_SOURCE_DIR}/iob -DCATKIN_INCLUDE_DIRS="${catkin_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS} ${openhrp3_INCLUDE_DIRS}"
  COMMAND cmake -E chdir ${CMAKE_CURRENT_BINARY_DIR}/iob make -j1
  COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/share/hrpsys_gazebo_general/lib
  COMMAND cmake -E copy  ${CMAKE_CURRENT_BINARY_DIR}/iob/libhrpIo.so ${CATKIN_DEVEL_PREFIX}/share/hrpsys_gazebo_general/lib
  DEPENDS ${PROJECT_SOURCE_DIR}/iob/iob.cpp
)
add_custom_target(hrpsys_gazebo_general_iob ALL
  DEPENDS ${CATKIN_DEVEL_PREFIX}/share/hrpsys_gazebo_general/lib)

## Build hrpsys for gazebo
#add_custom_command(OUTPUT ${CATKIN_DEVEL_PREFIX}/share/hrpsys_gazebo_general/lib
#  COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f ${PROJECT_SOURCE_DIR}/Makefile.hrpsys-base INSTALL_PREFIX_PATH=${CATKIN_DEVEL_PREFIX}/share/hrpsys_gazebo_general OPENRTM_DIR=${openrtm_aist_PREFIX}/lib/openrtm_aist HRPSYS_BASE_SOURCE=${hrpsys_SOURCE_DIR}/build/hrpsys-base-source CATKIN_INCLUDE_DIRS=${hrpsys_gazebo_msgs_INCLUDE_DIRS} CMAKE_PKG_CONFIG_PATH=${CATKIN_DEVEL_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH} installed
#  DEPENDS)
#add_custom_target(hrpsys_gazebo_general_iob ALL
#  DEPENDS ${CATKIN_DEVEL_PREFIX}/share/hrpsys_gazebo_general/lib)
#execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f ${PROJECT_SOURCE_DIR}/Makefile.hrpsys-base INSTALL_PREFIX_PATH=${CATKIN_DEVEL_PREFIX}/share/hrpsys_gazebo_general OPENRTM_DIR=${openrtm_aist_PREFIX}/lib/openrtm_aist HRPSYS_BASE_SOURCE=${hrpsys_SOURCE_DIR}/build/hrpsys-base-source CATKIN_INCLUDE_DIRS=${hrpsys_gazebo_msgs_INCLUDE_DIRS} installed
#  RESULT_VARIABLE _make_failed)
#if (_make_failed)
#  message(FATAL_ERROR "Build of hrpsys/iob failed")
#endif(_make_failed)

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

# move libhrpIo.so
# lib -> {source}/lib
if(NOT EXISTS ${PROJECT_SOURCE_DIR}/lib/)
  execute_process(
    COMMAND cmake -E make_directory ${PROJECT_SOURCE_DIR}/lib/
    RESULT_VARIABLE _make_failed)
  if (_make_failed)
    message(FATAL_ERROR "make_directory ${PROJECT_SOURCE_DIR}/lib/ failed: ${_make_failed}")
  endif(_make_failed)
endif()
if(EXISTS ${CATKIN_DEVEL_PREFIX}/share/hrpsys_gazebo_general/lib/libhrpIo.so)
  execute_process(
    COMMAND cmake -E rename ${CATKIN_DEVEL_PREFIX}/share/hrpsys_gazebo_general/lib/libhrpIo.so ${PROJECT_SOURCE_DIR}/lib/libhrpIo.so
    RESULT_VARIABLE _rename_failed)
  message("move libhrpIo.so ${PROJECT_SOURCE_DIR}/lib/libhrpIo.so")
  if (_rename_failed)
    message(FATAL_ERROR "Move libhrpIo.so failed: ${_rename_failed}")
  endif(_rename_failed)
endif()

install(DIRECTORY
  ${CATKIN_DEVEL_PREFIX}/share/hrpsys_gazebo_general/lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

