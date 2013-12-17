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
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_find_ros_package(hrpsys_ros_bridge)
include(${hrpsys_ros_bridge_PACKAGE_PATH}/cmake/compile_robot_model.cmake)

rosbuild_find_ros_package(atlas_description)
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
  COMMAND rosrun collada_urdf_jsk_patch urdf_to_collada ${atlas_urdf} ${atlas_dae}.bak
  COMMAND rosrun hrpsys_gazebo_atlas add_sensor_to_collada.py ${atlas_dae}.bak > ${atlas_dae}
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
endif()

rosbuild_find_ros_package(atlas_description)
if (EXISTS ${atlas_description_PACKAGE_PATH}/robots/atlas_v3.urdf.xacro)
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
  COMMAND rosrun collada_urdf_jsk_patch urdf_to_collada ${atlas_v3_urdf} ${atlas_v3_dae}.bak
  COMMAND rosrun hrpsys_gazebo_atlas add_sensor_to_collada.py ${atlas_v3_dae}.bak > ${atlas_v3_dae}
  DEPENDS ${atlas_v3_urdf}
)
compile_collada_model(${atlas_v3_dae}
  --euscollada-option "--without-technique-limit"
  --robothardware-conf-file-option "pdgains.file_name: ${PROJECT_SOURCE_DIR}/models/PDgains.sav"
  --conf-dt-option "0.003"
  --conf-file-option "abc_leg_offset: 0.0, 0.089, 0.0"
  --conf-file-option "abc_stride_parameter: 0.15,0.05,10"
  --conf-file-option "abc_end_effectors: :rarm,r_arm_wrx,back_bkx, :larm,l_arm_wrx,back_bkx, :rleg,r_leg_akx,pelvis, :lleg,l_leg_akx,pelvis,"
  --conf-file-option "end_effectors: :rarm,r_arm_wrx,back_bkx,0.0,-0.195946,0.04135,0.710565,-0.497543,-0.497543,1.90602, :larm,l_arm_wrx,back_bkx,0.0,0.195946,0.04135,-0.710565,-0.497543,0.497543,1.90602, :rleg,r_leg_akx,pelvis,0.04,0.0,-0.08,0.0,0.0,0.0,0.0, :lleg,l_leg_akx,pelvis,0.04,0.0,-0.08,0.0,0.0,0.0,0.0,"
  --conf-file-option "collision_pair: pelvis:l_arm_wrx pelvis:l_arm_wry pelvis:r_arm_wrx pelvis:r_arm_wry l_arm_wrx:l_leg_akx l_arm_wrx:l_leg_aky l_arm_wry:l_leg_kny l_arm_wry:l_leg_kny r_arm_wrx:r_leg_akx r_arm_wrx:r_leg_aky r_arm_wry:r_leg_kny r_arm_wry:r_leg_kny r_arm_wrx:l_arm_wrx r_arm_wrx:l_arm_wry r_arm_wry:l_arm_wrx r_arm_wry:l_arm_wry r_leg_akx:l_leg_akx r_leg_akx:l_leg_aky r_leg_akx:l_leg_kny r_leg_aky:l_leg_akx r_leg_aky:l_leg_aky r_leg_aky:l_leg_kny r_leg_kny:l_leg_akx r_leg_kny:l_leg_aky r_leg_kny:l_leg_kny"
#  --conf-file-option "collision_pair: back_bkx:l_arm_wrx back_bkx:l_arm_wry back_bkx:r_arm_wrx back_bkx:r_arm_wry back_bky:l_arm_wrx back_bky:l_arm_wry back_bky:r_arm_wrx back_bky:r_arm_wry back_bkz:l_arm_wrx back_bkz:l_arm_wry back_bkz:r_arm_wrx back_bkz:r_arm_wry l_arm_wrx:l_leg_akx l_arm_wrx:l_leg_aky l_arm_wry:l_leg_kny l_arm_wry:l_leg_kny r_arm_wrx:r_leg_akx r_arm_wrx:r_leg_aky r_arm_wry:r_leg_kny r_arm_wry:r_leg_kny r_arm_wrx:l_arm_wrx r_arm_wrx:l_arm_wry r_arm_wry:l_arm_wrx r_arm_wry:l_arm_wry r_leg_akx:l_leg_akx r_leg_akx:l_leg_aky r_leg_akx:l_leg_kny r_leg_aky:l_leg_akx r_leg_aky:l_leg_aky r_leg_aky:l_leg_kny r_leg_kny:l_leg_akx r_leg_kny:l_leg_aky r_leg_kny:l_leg_kny"
  --proj-file-root-option "0,0,1.0,0,0,1,0"
  )
endif()

## ALL
list(GET compile_robots 0 compile_robot)
list(LENGTH compile_robots compile_robots_num)
if (compile_robots_num GREATER 0)
list(REMOVE_AT compile_robots 0)
add_custom_target(openhrp_robots ALL)
add_dependencies(openhrp_robots ${compile_robot})
foreach(compile_target ${compile_robots})
  add_dependencies(${compile_robot} ${compile_target})
  list(GET compile_robots 0 compile_robot)
  list(REMOVE_AT compile_robots 0)
endforeach(compile_target ${compile_robots})
endif (compile_robots_num GREATER 0)
#
# set(atlas_dae  "${PROJECT_SOURCE_DIR}/models/atlas.dae")
# set(atlas_eus  "${PROJECT_SOURCE_DIR}/models/atlas.l")
# set(atlas_yaml "${PROJECT_SOURCE_DIR}/models/atlas.yaml")
# add_custom_command(
#   OUTPUT ${atlas_dae}
#   COMMAND rosrun collada_urdf_jsk_patch urdf_to_collada ${atlas_urdf} ${atlas_dae}
#   DEPENDS ${atlas_urdf}
# )
# add_custom_command(
#   OUTPUT ${atlas_eus}
#   COMMAND rosrun euscollada collada2eus_dev ${atlas_dae} ${atlas_yaml} ${atlas_eus}

#   DEPENDS ${atlas_dae}
# )

# add_custom_target(atlas_urdf ALL DEPENDS ${atlas_urdf})
# add_custom_target(atlas_dae  ALL DEPENDS ${atlas_dae})
# add_custom_target(atlas_eus  ALL DEPENDS ${atlas_eus})

# libiob
#rosbuild_find_ros_package(hrpsys)
#include_directories(${hrpsys_PACKAGE_PATH}/include/hrpsys/io)
#rosbuild_add_library(hrpIo src/iob.cpp)

# Build hrpsys for gazebo
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.hrpsys-base
                RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Build of hrpsys failed")
endif(_make_failed)

rosbuild_find_ros_package(laser_assembler)
if (EXISTS ${laser_assembler_PACKAGE_PATH}/srv_gen/cpp/include/laser_assembler/AssembleScans2.h)
  rosbuild_add_executable(atlas_laser_snapshotter src/atlas_laser_snapshotter.cpp)
else()
  message("*** WARNING *** : Use groovy version of laser_assmbler to use atlas_laser_snapshotter")
endif()

include_directories(${roseus_PACKAGE_PATH}/build/devel/include)
rosbuild_add_executable(hand_controller src/hand_controller.cpp)

## test code
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/test/.gen_testcode
  COMMAND make -C ${PROJECT_SOURCE_DIR}/test/
  VERBATIM)
add_custom_target(gen_testcode DEPENDS ${PROJECT_SOURCE_DIR}/test/.gen_testcode)
add_dependencies(tests gen_testcode)
rosbuild_add_rostest(test/test-atlasmodel.launch)

