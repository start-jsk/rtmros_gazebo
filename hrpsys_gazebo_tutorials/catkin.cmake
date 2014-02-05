cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_gazebo_tutorials)

macro (generate_gazebo_urdf_file _robot_name)
  set(_out_dir "${PROJECT_SOURCE_DIR}/robot_models/${_robot_name}")
  set(_out_urdf_file "${_out_dir}/${_robot_name}.urdf")
  add_custom_command(OUTPUT ${_out_dir}/meshes
    COMMAND mkdir ${_out_dir}/meshes)
  add_custom_command(OUTPUT ${_out_dir}/hrpsys
    COMMAND mkdir ${_out_dir}/hrpsys)
  add_custom_command(OUTPUT ${_out_urdf_file}
    COMMAND ${_out_dir}/install_robot.sh
    DEPENDS ${_out_dir}/hrpsys ${_out_dir}/meshes)
  add_custom_target(${_robot_name}_compile DEPENDS ${_out_urdf_file})
  set(ROBOT ${_robot_name})
  string(TOLOWER ${_robot_name} _sname)
  #configure_file(${PROJECT_SOURCE_DIR}/scripts/default_robot_hrpsys_bringup.launch.in ${PROJECT_SOURCE_DIR}/build/${_sname}_hrpsys_bringup.launch)
  if(NOT EXISTS ${PROJECT_SOURCE_DIR}/launch/gazebo_${_sname}_no_controllers.launch)
    configure_file(${PROJECT_SOURCE_DIR}/scripts/default_gazebo_robot_no_controllers.launch.in ${PROJECT_SOURCE_DIR}/launch/gazebo_${_sname}_no_controllers.launch)
  endif()
  list(APPEND compile_robots ${_robot_name}_compile)
endmacro()

generate_gazebo_urdf_file(SampleRobot)
generate_gazebo_urdf_file(HRP3HAND_L)
generate_gazebo_urdf_file(HRP3HAND_R)
generate_gazebo_urdf_file(HRP2JSK)
generate_gazebo_urdf_file(HRP2JSKNT)
generate_gazebo_urdf_file(HRP2JSKNTS)
generate_gazebo_urdf_file(STARO)
generate_gazebo_urdf_file(HRP4C)


add_custom_target(all_robots_compile ALL DEPENDS ${compile_robots})
