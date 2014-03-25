#!/bin/bash

function error {
    exit 1
}
trap error ERR


# usage
# ./install_robot_common.sh ROBOT_NAME (model directory) (output directory) (collada_to_urdf_binary) (additional_ros_package_path)

ROBOT_NAME=$1
if [ $# -gt 1 ]; then
    INPUT_DIR=$2
else
    INPUT_DIR=$(rospack find hrpsys_ros_bridge_tutorials)/models
fi

if [ $# -gt 2 ]; then
    OUTPUT_DIR=$3
else
    OUTPUT_DIR=$(rospack find hrpsys_gazebo_tutorials)/robot_models/${ROBOT_NAME}
fi

if [ $# -gt 3 ]; then
    COLLADA_TO_URDF=$4
else
    COLLADA_TO_URDF=`rospack find collada_tools`/bin/collada_to_urdf
fi

if [ $# -gt 4 ]; then
    ADDITIONAL_ROS_PACKAGE_PATH=$5
else
    ADDITIONAL_ROS_PACKAGE_PATH=
fi

OUTPUT_FILE=${OUTPUT_DIR}/${ROBOT_NAME}.urdf
SED_SCRIPT_FILE=${OUTPUT_DIR}/${ROBOT_NAME}_optional_urdf_setting.sh

if [ ! -e ${INPUT_DIR}/${ROBOT_NAME}.dae ]; then
    echo -e "\e[31m[WARNING] ${INPUT_DIR}/${ROBOT_NAME}.dae not found\e[m"
    exit 0
fi
if [ ! -e ${OUTPUT_FILE} ]; then
##
    mkdir -p ${OUTPUT_DIR}/meshes
    ${COLLADA_TO_URDF} ${INPUT_DIR}/${ROBOT_NAME}.dae -G -A --mesh_output_dir ${OUTPUT_DIR}/meshes --mesh_prefix "package://hrpsys_gazebo_tutorials/robot_models/${ROBOT_NAME}/meshes" --output_file=${OUTPUT_FILE}
    # if [ ${ROS_DISTRO} == "groovy" ]; then
    # 	rosrun collada_tools collada_to_urdf ${INPUT_DIR}/${ROBOT_NAME}.dae -G -A --mesh_output_dir ${OUTPUT_DIR}/meshes --mesh_prefix "package://${ROBOT_NAME}/meshes" --output_file=${OUTPUT_FILE}
    # elif [ ${ROS_DISTRO} == "hydro" ]; then
    # 	rosrun collada_urdf collada_to_urdf ${INPUT_DIR}/${ROBOT_NAME}.dae -G -A --mesh_output_dir ${OUTPUT_DIR}/meshes --mesh_prefix "package://${ROBOT_NAME}/meshes" --output_file=${OUTPUT_FILE}
    # fi
## execute sed
    ${SED_SCRIPT_FILE} ${OUTPUT_FILE} ${ADDITIONAL_ROS_PACKAGE_PATH}
fi

if [ ! -e ${OUTPUT_DIR}/hrpsys ]; then
    mkdir -p ${OUTPUT_DIR}/hrpsys
fi
ln -sf ${INPUT_DIR}/${ROBOT_NAME}.conf ${OUTPUT_DIR}/hrpsys
ln -sf ${INPUT_DIR}/${ROBOT_NAME}.RobotHardware.conf ${OUTPUT_DIR}/hrpsys
ln -sf ${INPUT_DIR}/${ROBOT_NAME}.dae ${OUTPUT_DIR}/hrpsys
ln -sf ${INPUT_DIR}/${ROBOT_NAME}_nosim.xml ${OUTPUT_DIR}/hrpsys
