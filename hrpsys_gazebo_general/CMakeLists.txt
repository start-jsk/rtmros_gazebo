if(NOT USE_ROSBUILD)
  include(catkin.cmake)
  return()
endif()
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_find_ros_package(openrtm_aist)
rosbuild_find_ros_package(openhrp3)
rosbuild_find_ros_package(hrpsys)
rosbuild_find_ros_package(hrpsys_gazebo_msgs)
find_package(Eigen REQUIRED)
## Build only iob
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/share/hrpsys_gazebo_general/lib/RobotHardware.so
  COMMAND cmake -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/iob
  COMMAND cmake -E chdir ${CMAKE_CURRENT_BINARY_DIR}/iob cmake ${PROJECT_SOURCE_DIR}/iob -DCMAKE_INSTALL_PREFIX=${PROJECT_SOURCE_DIR} -DCATKIN_INCLUDE_DIRS="${Eigen_INCLUDE_DIRS};${openrtm_aist_PACKAGE_PATH}/include/openrtm-1.1;${openrtm_aist_PACKAGE_PATH}/include/openrtm-1.1/rtm/idl;${openrtm_aist_PACKAGE_PATH}/include/coil-1.1;${openhrp3_PACKAGE_PATH}/include/OpenHRP-3.1;${hrpsys_gazebo_msgs_PACKAGE_PATH}/msg_gen/cpp/include;${hrpsys_gazebo_msgs_PACKAGE_PATH}/srv_gen/cpp/include" -DRobotHardware_PACKAGE_PATH=${hrpsys_PACKAGE_PATH}/lib
  COMMAND cmake -E chdir ${CMAKE_CURRENT_BINARY_DIR}/iob make -j1
  COMMAND cmake -E chdir ${CMAKE_CURRENT_BINARY_DIR}/iob make -j1 install
  DEPENDS ${PROJECT_SOURCE_DIR}/iob/iob.cpp
)
add_custom_target(hrpsys_gazebo_general_iob ALL
  DEPENDS ${PROJECT_SOURCE_DIR}/share/hrpsys_gazebo_general/lib/RobotHardware.so)

## Gazebo plugins
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()
include_directories( ${GAZEBO_INCLUDE_DIRS} )
link_directories( ${GAZEBO_LIBRARY_DIRS} )

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/plugins)

rosbuild_add_library(IOBPlugin src/IOBPlugin.cpp)
rosbuild_add_library(AddForcePlugin src/AddForcePlugin.cpp)
rosbuild_add_library(SetVelPlugin src/SetVelPlugin.cpp)
rosbuild_add_library(GetVelPlugin src/GetVelPlugin.cpp)

