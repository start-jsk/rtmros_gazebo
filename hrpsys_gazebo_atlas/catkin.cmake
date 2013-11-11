# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_gazebo_atlas)

find_package(catkin REQUIRED COMPONENTS hrpsys_gazebo_general)

catkin_package(CATKIN_DEPENDS hrpsys_gazebo_general)

# Build hrpsys for gazebo
find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)
pkg_check_modules(openhrp3 openhrp3.1 REQUIRED)
set(OPENRTM_DIR ${openrtm_aist_PREFIX}/lib/openrtm_aist)
set(OPENHRP_DIR ${openhrp3_PREFIX}/share/openhrp3)
set(ENV{PKG_CONFIG_PATH} $ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig)
execute_process(COMMAND svn co http://hrpsys-base.googlecode.com/svn/trunk ${PROJECT_SOURCE_DIR}/build/hrpsys-base-source) ## deb does not have source modules
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.hrpsys-base OPENRTM_DIR=${OPENRTM_DIR} OPENHRP_DIR=${OPENHRP_DIR} SVN_DIR=${PROJECT_SOURCE_DIR} PKG_CONFIG_PATH_SETUP= INSTALL_DIR=${CATKIN_DEVEL_PREFIX}
                RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Build of hrpsys failed")
endif(_make_failed)

include_directories(${catkin_INCLUDE_DIRS})

## laser assember is not catkinized
include_directories(/opt/ros/groovy/stacks/laser_assembler/srv_gen/cpp/include/)
add_executable(atlas_laser_snapshotter src/atlas_laser_snapshotter.cpp)
target_link_libraries(atlas_laser_snapshotter ${catkin_LIBRARIES})

## pr2_controller_manager is not catkinized
# include_directories(/opt/ros/groovy/stacks/pr2_mechanism/pr2_controller_manager/include)
# include_directories(/opt/ros/groovy/stacks/pr2_mechanism/pr2_controller_interface/include)
# include_directories(/opt/ros/groovy/stacks/pr2_mechanism/pr2_mechanism_model/include)
# include_directories(/opt/ros/groovy/stacks/pr2_mechanism/pr2_hardware_interface/include)
# include_directories(/opt/ros/groovy/stacks/ros_control/hardware_interface/include)
# include_directories(/opt/ros/groovy/stacks/ros_control/controller_interface/include)
# include_directories(/opt/ros/groovy/stacks/ros_control/realtime_tools/include)
# add_executable(hand_controller src/hand_controller.cpp)
# target_link_libraries(hand_controller ${catkin_LIBRARIES})

if(0)
## test code
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/test/.gen_testcode
  COMMAND make -C ${PROJECT_SOURCE_DIR}/test/
  VERBATIM)
add_custom_target(gen_testcode DEPENDS ${PROJECT_SOURCE_DIR}/test/.gen_testcode)
add_dependencies(tests gen_testcode)
rosbuild_add_rostest(test/test-atlasmodel.launch)
endif()


