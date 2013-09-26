#!/bin/bash

# usage
# ./install_robot.sh ROBOT_NAME (model directory)

ROBOT_NAME=$1
if [ $# -gt 1 ]; then
    INPUT_DIR=$2
else
    INPUT_DIR=$(rospack find hrpsys_ros_bridge_tutorials)/models
fi

OUTPUT_DIR=$(rospack find hrpsys_gazebo_tutorials)/robot_models/${ROBOT_NAME}
OUTPUT_FILE=${OUTPUT_DIR}/${ROBOT_NAME}.urdf
SED_SCRIPT_FILE=${OUTPUT_DIR}/${ROBOT_NAME}_optional_urdf_setting.sh

if [ ! -e ${OUTPUT_FILE} ]; then
##
    mkdir -p ${OUTPUT_DIR}/meshes
    rosrun collada_tools collada_to_urdf ${INPUT_DIR}/${ROBOT_NAME}.dae -G -A --mesh_output_dir ${OUTPUT_DIR}/meshes --mesh_prefix "package://${ROBOT_NAME}/meshes" --output_file=${OUTPUT_FILE}
## execute sed
    ${SED_SCRIPT_FILE} ${OUTPUT_FILE}
fi

if [ ! -e ${OUTPUT_DIR}/hrpsys ]; then
    mkdir -p ${OUTPUT_DIR}/hrpsys
fi
ln -sf ${INPUT_DIR}/${ROBOT_NAME}.conf ${OUTPUT_DIR}/hrpsys
ln -sf ${INPUT_DIR}/${ROBOT_NAME}.RobotHardware.conf ${OUTPUT_DIR}/hrpsys
ln -sf ${INPUT_DIR}/${ROBOT_NAME}.dae ${OUTPUT_DIR}/hrpsys
ln -sf ${INPUT_DIR}/${ROBOT_NAME}_nosim.xml ${OUTPUT_DIR}/hrpsys
