cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_gazebo_tutorials)

find_package(catkin REQUIRED COMPONENTS euscollada hrpsys_ros_bridge hrpsys_ros_bridge_tutorials)

set(PKG_CONFIG_PATH ${hrpsys_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH})
find_package(PkgConfig)
pkg_check_modules(openhrp3 openhrp3.1 REQUIRED)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)
pkg_check_modules(collada_urdf_jsk_patch collada_urdf_jsk_patch)

catkin_package(CATKIN_DEPENDS euscollada hrpsys_ros_bridge hrpsys_ros_bridge_tutorials)

if(NOT hrpsys_ros_bridge_tutorials_SOURCE_DIR)
  execute_process(
    COMMAND rospack find hrpsys_ros_bridge_tutorials
    OUTPUT_VARIABLE hrpsys_ros_bridge_tutorials_SOURCE_DIR)
  string(REGEX REPLACE "\n" "" hrpsys_ros_bridge_tutorials_SOURCE_DIR ${hrpsys_ros_bridge_tutorials_SOURCE_DIR})
endif()

## Convert robot models
if(EXISTS ${hrpsys_gazebo_general_SOURCE_DIR})
  set(hrpsys_gazebo_general_PACKAGE_PATH ${hrpsys_gazebo_general_SOURCE_DIR})
else()
  set(hrpsys_gazebo_general_PACKAGE_PATH ${hrpsys_gazebo_general_PREFIX}/share/hrpsys_gazebo_general)
endif()
include(${hrpsys_gazebo_general_PACKAGE_PATH}/cmake/compile_robot_model_for_gazebo.cmake)
generate_gazebo_urdf_file(${hrpsys_ros_bridge_tutorials_SOURCE_DIR}/models/SampleRobot.dae)
generate_gazebo_urdf_file(${hrpsys_ros_bridge_tutorials_SOURCE_DIR}/models/HRP3HAND_L.dae)
generate_gazebo_urdf_file(${hrpsys_ros_bridge_tutorials_SOURCE_DIR}/models/HRP3HAND_R.dae)
generate_gazebo_urdf_file(${hrpsys_ros_bridge_tutorials_SOURCE_DIR}/models/HRP2JSK.dae)
generate_gazebo_urdf_file(${hrpsys_ros_bridge_tutorials_SOURCE_DIR}/models/HRP2JSKNT.dae)
generate_gazebo_urdf_file(${hrpsys_ros_bridge_tutorials_SOURCE_DIR}/models/HRP2JSKNTS.dae)
generate_gazebo_urdf_file(${hrpsys_ros_bridge_tutorials_SOURCE_DIR}/models/STARO.dae)
generate_gazebo_urdf_file(${hrpsys_ros_bridge_tutorials_SOURCE_DIR}/models/HRP4C.dae)
add_custom_target(all_robots_compile_tutorials ALL DEPENDS ${compile_urdf_robots})

## install
install(FILES setup.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY euslisp worlds launch config environment_models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
