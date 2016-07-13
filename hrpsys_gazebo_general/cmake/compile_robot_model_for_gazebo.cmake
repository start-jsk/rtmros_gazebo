##
## define macros
##
macro (generate_gazebo_urdf_file daefile)
  if(NOT EXISTS ${daefile})
    message(WARNING "generate gazebo_urdf_file: ${daefile} not found.")
    break()
  endif(NOT EXISTS ${daefile})
  # set variable
  if(hrpsys_gazebo_general_SOURCE_DIR)
    set(hrpsys_gazebo_general_PACKAGE_PATH ${hrpsys_gazebo_general_SOURCE_DIR})
  elseif(hrpsys_gazebo_general_SOURCE_PREFIX)
    set(hrpsys_gazebo_general_PACKAGE_PATH ${hrpsys_gazebo_general_SOURCE_PREFIX})
  else(hrpsys_gazebo_general_SOURCE_PREFIX)
    set(hrpsys_gazebo_general_PACKAGE_PATH ${hrpsys_gazebo_general_PREFIX}/share/hrpsys_gazebo_general)
  endif()
  get_filename_component(_robot_name ${daefile} NAME_WE)
  set(_workdir ${PROJECT_SOURCE_DIR}/robot_models)
  set(_out_dir "${_workdir}/${_robot_name}")
  set(_out_urdf_file "${_out_dir}/${_robot_name}.urdf")
  set(_out_urdf_gazebo_file "${_out_dir}/${_robot_name}_gazebo.urdf")
  add_custom_command(OUTPUT ${_out_dir}/meshes
    COMMAND mkdir ${_out_dir}/meshes)
  # convert dae to urdf
  message("generate_gazebo_urdf_file: converting ${daefile}.")
  ## ${compile_robots} is a global target used in compile_robot_model.cmake of hrpsys_ros_bridge.
  ## this dependency means that converting urdf after executing all of ${compile_robots}.
  add_custom_command(OUTPUT ${_out_urdf_file}
    COMMAND ${collada_urdf_jsk_patch_PREFIX}/lib/collada_urdf_jsk_patch/collada_to_urdf ${daefile} -G -A --mesh_output_dir ${_out_dir}/meshes --mesh_prefix "package://${PROJECT_NAME}/robot_models/${_robot_name}/meshes" --output_file=${_out_urdf_file}
    COMMAND ${_out_dir}/${_robot_name}_additional_urdf_setting.sh ${_out_urdf_file}
    DEPENDS ${_out_dir}/meshes ${compile_robots})
  add_custom_command(OUTPUT ${_out_urdf_gazebo_file}
    COMMAND sed -e "s@package://${PROJECT_NAME}/robot_models/@model://@g" ${_out_urdf_file} > ${_out_urdf_gazebo_file}
    DEPENDS ${_out_urdf_file})
  add_custom_target(${_robot_name}_${PROJECT_NAME}_compile DEPENDS ${_out_urdf_gazebo_file})
  # generate launch and world file
  set(ROBOT ${_robot_name})
  string(TOLOWER ${_robot_name} _robot_sname)
  if(NOT EXISTS ${PROJECT_SOURCE_DIR}/launch/gazebo_${_robot_sname}_no_controllers.launch)
    configure_file(${hrpsys_gazebo_general_PACKAGE_PATH}/scripts/default_gazebo_robot_no_controllers.launch.in ${PROJECT_SOURCE_DIR}/launch/gazebo_${_robot_sname}_no_controllers.launch)
    list(APPEND ${_robot_name}_${PROJECT_NAME}_compile ${PROJECT_SOURCE_DIR}/launch/gazebo_${_robot_sname}_no_controllers.launch)
  endif()
  if(NOT EXISTS ${PROJECT_SOURCE_DIR}/launch/${_robot_sname}_hrpsys_bringup.launch)
    configure_file(${hrpsys_gazebo_general_PACKAGE_PATH}/scripts/default_robot_hrpsys_bringup.launch.in ${PROJECT_SOURCE_DIR}/launch/${_robot_sname}_hrpsys_bringup.launch)
    list(APPEND ${_robot_name}_${PROJECT_NAME}_compile ${PROJECT_SOURCE_DIR}/launch/${_robot_sname}_hrpsys_bringup.launch)
  endif()
  if(NOT EXISTS ${_out_dir}/model.config)
    configure_file(${hrpsys_gazebo_general_PACKAGE_PATH}/scripts/model.config.in ${_out_dir}/model.config)
    list(APPEND ${_robot_name}_${PROJECT_NAME}_compile ${_out_dir}/model.config)
  endif()
  if(NOT EXISTS ${PROJECT_SOURCE_DIR}/config/${_robot_name}.yaml)
    configure_file(${hrpsys_gazebo_general_PACKAGE_PATH}/scripts/default_robot.yaml.in ${PROJECT_SOURCE_DIR}/config/${_robot_name}.yaml)
    list(APPEND ${_robot_name}_${PROJECT_NAME}_compile ${PROJECT_SOURCE_DIR}/config/${_robot_name}.yaml)
  endif()
  list(APPEND compile_urdf_robots ${_robot_name}_${PROJECT_NAME}_compile)
endmacro()
