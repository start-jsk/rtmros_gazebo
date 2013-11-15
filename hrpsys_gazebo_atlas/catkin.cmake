# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_gazebo_atlas)

find_package(catkin REQUIRED COMPONENTS hrpsys_gazebo_general atlas_description)

catkin_package(CATKIN_DEPENDS hrpsys_gazebo_general atlas_description)


# atlas only works with gazebo ( we need osrf_msgs package )
if(NOT ("$ENV{ROS_DISTRO}" STREQUAL "groovy"))
  message(WARNING "[WARNING] hrpsys_gazebo_atlas does not support $ENV{ROS_DISTRO}")
  return()
endif()

if(EXISTS ${hrpsys_ros_bridge_SOURCE_DIR})
  set(hrpsys_ros_bridge_PACKAGE_PATH ${hrpsys_ros_bridge_SOURCE_DIR})
else()
  set(hrpsys_ros_bridge_PACKAGE_PATH ${hrpsys_ros_bridge_PREFIX}/share/hrpsys_ros_bridge)
endif()
include(${hrpsys_ros_bridge_PACKAGE_PATH}/cmake/compile_robot_model.cmake)

find_package(PkgConfig)
pkg_check_modules(collada_urdf_jsk_patch collada_urdf_jsk_patch)

if(EXISTS ${atlas_description_SOURCE_DIR})
  set(atlas_description_PACKAGE_PATH ${atlas_description_SOURCE_DIR})
else()
  set(atlas_description_PACKAGE_PATH ${atlas_description_PREFIX}/share/atlas_description)
endif()
if (collada_urdf_jsk_patch_FOUND)
if (EXISTS ${atlas_description_PACKAGE_PATH}/urdf/atlas.urdf)
  set(atlas_urdf "${PROJECT_SOURCE_DIR}/build/atlas.jsk.urdf")
  set(atlas_dae  "${PROJECT_SOURCE_DIR}/models/atlas.dae")
  add_custom_command(
    OUTPUT  ${atlas_urdf}
    COMMAND rosrun xacro xacro.py ${atlas_description_PACKAGE_PATH}/robots/atlas.urdf.xacro > ${atlas_urdf}
    COMMAND sed -i -e 's/link name=\"head\"/link name=\"atlas_head\"/' ${atlas_urdf}
    COMMAND sed -i -e 's/link=\"head\"/link=\"atlas_head\"/g' ${atlas_urdf}
    DEPENDS ${atlas_description_PACKAGE_PATH}/urdf/atlas.urdf
    )
  add_custom_command(
    OUTPUT ${atlas_dae}
    COMMAND ${collada_urdf_jsk_patch_PREFIX}/lib/collada_urdf_jsk_patch/urdf_to_collada ${atlas_urdf} ${atlas_dae}.bak
    COMMAND ${PROJECT_SOURCE_DIR}/scripts/add_sensor_to_collada.py ${atlas_dae}.bak > ${atlas_dae}
    DEPENDS ${atlas_urdf}
    )
  compile_collada_model(${atlas_dae}
    --euscollada-option "--without-technique-limit"
    --robothardware-conf-file-option "pdgains.file_name: ${PROJECT_SOURCE_DIR}/models/PDgains.sav"
    --conf-dt-option "0.003"
    --conf-file-option "abc_leg_offset: 0.0, 0.089, 0.0"
    --conf-file-option "abc_stride_parameter: 0.15,0.05,10"
    --conf-file-option "abc_end_effectors: :rarm,r_arm_mwx,back_ubx, :larm,l_arm_mwx,back_ubx, :rleg,r_leg_lax,pelvis, :lleg,l_leg_lax,pelvis,"
    --proj-file-root-option "0,0,1.0,0,0,1,0"
    )
else()
  message(FATAL_ERROR "${atlas_description_PACKAGE_PATH}/urdf/atlas.urdf is not found")
endif()

if (${collada_urdf_jsk_patch_FOUND} AND EXISTS ${atlas_description_PACKAGE_PATH}/robots/atlas_v3.urdf.xacro)
  set(atlas_v3_urdf "${PROJECT_SOURCE_DIR}/build/atlas_v3.jsk.urdf")
  set(atlas_v3_dae  "${PROJECT_SOURCE_DIR}/models/atlas_v3.dae")
  add_custom_command(
    OUTPUT  ${atlas_v3_urdf}
    COMMAND rosrun xacro xacro.py ${atlas_description_PACKAGE_PATH}/robots/atlas_v3.urdf.xacro > ${atlas_v3_urdf}
    COMMAND sed -i -e 's/link name=\"head\"/link name=\"atlas_head\"/' ${atlas_v3_urdf}
    COMMAND sed -i -e 's/link=\"head\"/link=\"atlas_head\"/g' ${atlas_v3_urdf}
    DEPENDS ${atlas_description_PACKAGE_PATH}/robots/atlas_v3.urdf.xacro
    )
  add_custom_command(
    OUTPUT ${atlas_v3_dae}
    COMMAND ${collada_urdf_jsk_patch_PREFIX}/lib/collada_urdf_jsk_patch/urdf_to_collada ${atlas_v3_urdf} ${atlas_v3_dae}.bak
    COMMAND ${PROJECT_SOURCE_DIR}/scripts/add_sensor_to_collada.py ${atlas_v3_dae}.bak > ${atlas_v3_dae}
    DEPENDS ${atlas_v3_urdf}
    )
  compile_collada_model(${atlas_v3_dae}
    --euscollada-option "--without-technique-limit"
    --robothardware-conf-file-option "pdgains.file_name: ${PROJECT_SOURCE_DIR}/models/PDgains.sav"
    --conf-file-option "abc_leg_offset: 0.0, 0.089, 0.0"
    --conf-file-option "abc_stride_parameter: 0.15,0.05,10"
    --conf-file-option "abc_end_effectors: :rarm,r_arm_wrx,back_bkx, :larm,l_arm_wrx,back_bkx, :rleg,r_leg_akx,pelvis, :lleg,l_leg_akx,pelvis,"
    --conf-file-option "collision_pair: pelvis:l_arm_wrx pelvis:l_arm_wry pelvis:r_arm_wrx pelvis:r_arm_wry l_arm_wrx:l_leg_akx l_arm_wrx:l_leg_aky l_arm_wry:l_leg_kny l_arm_wry:l_leg_kny r_arm_wrx:r_leg_akx r_arm_wrx:r_leg_aky r_arm_wry:r_leg_kny r_arm_wry:r_leg_kny r_arm_wrx:l_arm_wrx r_arm_wrx:l_arm_wry r_arm_wry:l_arm_wrx r_arm_wry:l_arm_wry r_leg_akx:l_leg_akx r_leg_akx:l_leg_aky r_leg_akx:l_leg_kny r_leg_aky:l_leg_akx r_leg_aky:l_leg_aky r_leg_aky:l_leg_kny r_leg_kny:l_leg_akx r_leg_kny:l_leg_aky r_leg_kny:l_leg_kny"
    #  --conf-file-option "collision_pair: back_bkx:l_arm_wrx back_bkx:l_arm_wry back_bkx:r_arm_wrx back_bkx:r_arm_wry back_bky:l_arm_wrx back_bky:l_arm_wry back_bky:r_arm_wrx back_bky:r_arm_wry back_bkz:l_arm_wrx back_bkz:l_arm_wry back_bkz:r_arm_wrx back_bkz:r_arm_wry l_arm_wrx:l_leg_akx l_arm_wrx:l_leg_aky l_arm_wry:l_leg_kny l_arm_wry:l_leg_kny r_arm_wrx:r_leg_akx r_arm_wrx:r_leg_aky r_arm_wry:r_leg_kny r_arm_wry:r_leg_kny r_arm_wrx:l_arm_wrx r_arm_wrx:l_arm_wry r_arm_wry:l_arm_wrx r_arm_wry:l_arm_wry r_leg_akx:l_leg_akx r_leg_akx:l_leg_aky r_leg_akx:l_leg_kny r_leg_aky:l_leg_akx r_leg_aky:l_leg_aky r_leg_aky:l_leg_kny r_leg_kny:l_leg_akx r_leg_kny:l_leg_aky r_leg_kny:l_leg_kny"
    --proj-file-root-option "0,0,1.0,0,0,1,0"
    )
else()
  message(FATAL_ERROR "${atlas_description_PACKAGE_PATH}/robots/atlas_v3.urdf.xacro")
endif()
endif (collada_urdf_jsk_patch_FOUND)

# Build hrpsys for gazebo
find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)
pkg_check_modules(openhrp3 openhrp3.1 REQUIRED)
set(OPENRTM_DIR ${openrtm_aist_PREFIX}/lib/openrtm_aist)
set(OPENHRP_DIR ${openhrp3_PREFIX}/share/openhrp3)
set(ENV{PKG_CONFIG_PATH} $ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig)
execute_process(COMMAND svn co http://hrpsys-base.googlecode.com/svn/trunk ${PROJECT_SOURCE_DIR}/build/hrpsys-base-source) ## deb does not have source modules
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.hrpsys-base OPENRTM_DIR=${OPENRTM_DIR} OPENHRP_DIR=${OPENHRP_DIR} SVN_DIR=${PROJECT_SOURCE_DIR}/build/hrpsys-base-source PKG_CONFIG_PATH_SETUP= INSTALL_DIR=${CATKIN_DEVEL_PREFIX}
                RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Build of hrpsys failed")
endif(_make_failed)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

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


