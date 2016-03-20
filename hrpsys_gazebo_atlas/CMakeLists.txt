# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_gazebo_atlas)

find_package(catkin REQUIRED COMPONENTS
  hrpsys_gazebo_general
  jsk_footstep_msgs
  laser_assembler
  message_generation
  )
catkin_python_setup()

add_message_files(
  DIRECTORY msg
  FILES JointStateCompressed.msg TrianglePoints.msg
)

generate_messages(
  DEPENDENCIES geometry_msgs actionlib_msgs jsk_footstep_msgs
)

catkin_package(CATKIN_DEPENDS hrpsys_gazebo_general jsk_footstep_msgs laser_assembler message_runtime)


if(EXISTS ${hrpsys_ros_bridge_SOURCE_DIR})
  set(hrpsys_ros_bridge_PACKAGE_PATH ${hrpsys_ros_bridge_SOURCE_DIR})
elseif(EXISTS ${hrpsys_ros_bridge_SOURCE_PREFIX})
  set(hrpsys_ros_bridge_PACKAGE_PATH ${hrpsys_ros_bridge_SOURCE_PREFIX})
else()
  set(hrpsys_ros_bridge_PACKAGE_PATH ${hrpsys_ros_bridge_PREFIX}/share/hrpsys_ros_bridge)
endif()
include(${hrpsys_ros_bridge_PACKAGE_PATH}/cmake/compile_robot_model.cmake)

# check if atlas description is installed
find_package(PkgConfig)
pkg_check_modules(atlas_description atlas_description)
if(atlas_description_FOUND)
if(EXISTS ${atlas_description_SOURCE_DIR})
  set(atlas_description_PACKAGE_PATH ${atlas_description_SOURCE_DIR})
elseif(EXISTS ${atlas_description_SOURCE_PREFIX})
  set(atlas_description_PACKAGE_PATH ${atlas_description_SOURCE_PREFIX})
else(EXISTS ${atlas_description_SOURCE_DIR})
  set(atlas_description_PACKAGE_PATH ${atlas_description_PREFIX}/share/atlas_description)
endif(EXISTS ${atlas_description_SOURCE_DIR})

find_package(collada_urdf_jsk_patch QUIET)
if (collada_urdf_jsk_patch_FOUND)
if (EXISTS ${atlas_description_PACKAGE_PATH}/urdf/atlas.urdf)
  set(atlas_urdf "${CMAKE_CURRENT_BINARY_DIR}/atlas.jsk.urdf")
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
    --conf-file-option "end_effectors: rarm,r_arm_mwx,back_ubx,0.0,-0.195946,0.04135,0.710565,-0.497543,-0.497543,1.90602, larm,l_arm_mwx,back_ubx,0.0,0.195946,0.04135,-0.710565,-0.497543,0.497543,1.90602, rleg,r_leg_lax,pelvis,0.04,0.0,-0.08,0.0,0.0,0.0,0.0, lleg,l_leg_lax,pelvis,0.04,0.0,-0.08,0.0,0.0,0.0,0.0,"
    --proj-file-root-option "0,0,1.0,0,0,1,0"
    )
else()
  message(FATAL_ERROR "${atlas_description_PACKAGE_PATH}/urdf/atlas.urdf is not found")
endif (EXISTS ${atlas_description_PACKAGE_PATH}/urdf/atlas.urdf)

if (EXISTS ${atlas_description_PACKAGE_PATH}/robots/atlas_v3.urdf.xacro)
  set(atlas_v3_urdf "${CMAKE_CURRENT_BINARY_DIR}/atlas_v3.jsk.urdf")
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
    --conf-dt-option "0.003"
    --conf-file-option "abc_leg_offset: 0.0, 0.089, 0.0"
    --conf-file-option "abc_stride_parameter: 0.15,0.05,10"
    --conf-file-option "end_effectors: rarm,r_arm_wrx,back_bkx,0.0,-0.195946,0.04135,0.710565,-0.497543,-0.497543,1.90602, larm,l_arm_wrx,back_bkx,0.0,0.195946,0.04135,-0.710565,-0.497543,0.497543,1.90602, rleg,r_leg_akx,pelvis,0.04,0.0,-0.08,0.0,0.0,0.0,0.0, lleg,l_leg_akx,pelvis,0.04,0.0,-0.08,0.0,0.0,0.0,0.0,"
    --conf-file-option "collision_pair: pelvis:l_arm_wrx pelvis:l_arm_wry pelvis:r_arm_wrx pelvis:r_arm_wry l_arm_wrx:l_leg_akx l_arm_wrx:l_leg_aky l_arm_wry:l_leg_kny l_arm_wry:l_leg_kny r_arm_wrx:r_leg_akx r_arm_wrx:r_leg_aky r_arm_wry:r_leg_kny r_arm_wry:r_leg_kny r_arm_wrx:l_arm_wrx r_arm_wrx:l_arm_wry r_arm_wry:l_arm_wrx r_arm_wry:l_arm_wry r_leg_akx:l_leg_akx r_leg_akx:l_leg_aky r_leg_akx:l_leg_kny r_leg_aky:l_leg_akx r_leg_aky:l_leg_aky r_leg_aky:l_leg_kny r_leg_kny:l_leg_akx r_leg_kny:l_leg_aky r_leg_kny:l_leg_kny"
    #  --conf-file-option "collision_pair: back_bkx:l_arm_wrx back_bkx:l_arm_wry back_bkx:r_arm_wrx back_bkx:r_arm_wry back_bky:l_arm_wrx back_bky:l_arm_wry back_bky:r_arm_wrx back_bky:r_arm_wry back_bkz:l_arm_wrx back_bkz:l_arm_wry back_bkz:r_arm_wrx back_bkz:r_arm_wry l_arm_wrx:l_leg_akx l_arm_wrx:l_leg_aky l_arm_wry:l_leg_kny l_arm_wry:l_leg_kny r_arm_wrx:r_leg_akx r_arm_wrx:r_leg_aky r_arm_wry:r_leg_kny r_arm_wry:r_leg_kny r_arm_wrx:l_arm_wrx r_arm_wrx:l_arm_wry r_arm_wry:l_arm_wrx r_arm_wry:l_arm_wry r_leg_akx:l_leg_akx r_leg_akx:l_leg_aky r_leg_akx:l_leg_kny r_leg_aky:l_leg_akx r_leg_aky:l_leg_aky r_leg_aky:l_leg_kny r_leg_kny:l_leg_akx r_leg_kny:l_leg_aky r_leg_kny:l_leg_kny"
    --proj-file-root-option "0,0,1.0,0,0,1,0"
    )
else()
  message(FATAL_ERROR "${atlas_description_PACKAGE_PATH}/robots/atlas_v3.urdf.xacro")
endif (EXISTS ${atlas_description_PACKAGE_PATH}/robots/atlas_v3.urdf.xacro)
endif (collada_urdf_jsk_patch_FOUND)

## Build only atlas iob
pkg_check_modules(omniorb omniORB4 REQUIRED)
pkg_check_modules(omnidynamic omniDynamic4 REQUIRED)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)
pkg_check_modules(openhrp3 openhrp3.1 REQUIRED)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)
if(EXISTS ${hrpsys_SOURCE_DIR})
  set(ROBOTHARDWARE_SOURCE ${hrpsys_SOURCE_DIR}/src/rtc/RobotHardware)
elseif(EXISTS ${hrpsys_SOURCE_PREFIX})
  set(ROBOTHARDWARE_SOURCE ${hrpsys_SOURCE_PREFIX}/src/rtc/RobotHardware)
else()
  set(ROBOTHARDWARE_SOURCE ${hrpsys_PREFIX}/share/hrpsys/src/rtc/RobotHardware)
endif()
include_directories(${catkin_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS} ${openhrp3_INCLUDE_DIRS} ${hrpsys_INCLUDE_DIRS})
link_directories(${CATKIN_DEVEL_PREFIX}/lib ${hrpsys_PREFIX}/lib ${openhrp3_LIBRARY_DIRS} /opt/ros/$ENV{ROS_DISTRO}/lib/)
add_subdirectory(iob)
add_custom_target(hrpsys_gazebo_atlas_iob ALL DEPENDS RobotHardware_atlas)
else(atlas_description_FOUND)
  message(AUTHOR_WARNING "no atlas_description is installed")
endif(atlas_description_FOUND)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(atlas_laser_snapshotter src/atlas_laser_snapshotter.cpp)
target_link_libraries(atlas_laser_snapshotter ${catkin_LIBRARIES})

## pr2_controller_manager is not catkinized for groovy
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


