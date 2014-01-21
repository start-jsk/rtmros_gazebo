#!/bin/bash

#. ~/.bashrc
#roscd hrpsys_gazebo_tutorials/robot_models/HRP2JSKNT_with_HRP3HAND/
rm -rf HRP2JSKNT_modified_for_hrp3hand.urdf HRP2JSKNT_with_HRP3HAND.urdf
cp ../HRP2JSKNT/HRP2JSKNT.urdf ./HRP2JSKNT_modified_for_hrp3hand.urdf

# remove LARM_LINK6
L_START=`grep -n "<link name=\"LARM_LINK6\"" -m 1 HRP2JSKNT_modified_for_hrp3hand.urdf -m 1 | cut -f1 -d:`
L_END=$(sed -n "${L_START},\$p" HRP2JSKNT_modified_for_hrp3hand.urdf | grep -n "<\/gazebo>" -m 1 | cut -f1 -d:)
L_END=`expr ${L_START} + ${L_END} - 1`
sed -i -e "${L_START},${L_END}d" HRP2JSKNT_modified_for_hrp3hand.urdf

# remove LARM_LINK7
L_START=`grep -n "<link name=\"LARM_LINK7\"" HRP2JSKNT_modified_for_hrp3hand.urdf -m 1 | cut -f1 -d:`
L_END=$(sed -n "${L_START},\$p" HRP2JSKNT_modified_for_hrp3hand.urdf | grep -n "<\/gazebo>" -m 1 | cut -f1 -d:)
L_END=`expr ${L_START} + ${L_END} - 1`
sed -i -e "${L_START},${L_END}d" HRP2JSKNT_modified_for_hrp3hand.urdf

# remove LARM_JOINT7
L_START=`grep -n "<joint name=\"LARM_JOINT7\"" HRP2JSKNT_modified_for_hrp3hand.urdf -m 1 | cut -f1 -d:`
L_END=$(sed -n "${L_START},\$p" HRP2JSKNT_modified_for_hrp3hand.urdf | grep -n "<\/gazebo>" -m 1 | cut -f1 -d:)
L_END=`expr ${L_START} + ${L_END} - 1`
sed -i -e "${L_START},${L_END}d" HRP2JSKNT_modified_for_hrp3hand.urdf

# remove RARM_LINK6
L_START=`grep -n "<link name=\"RARM_LINK6\"" HRP2JSKNT_modified_for_hrp3hand.urdf -m 1 | cut -f1 -d:`
L_END=$(sed -n "${L_START},\$p" HRP2JSKNT_modified_for_hrp3hand.urdf | grep -n "<\/gazebo>" -m 1 | cut -f1 -d:)
L_END=`expr ${L_START} + ${L_END} - 1`
sed -i -e "${L_START},${L_END}d" HRP2JSKNT_modified_for_hrp3hand.urdf

# remove RARM_LINK7
L_START=`grep -n "<link name=\"RARM_LINK7\"" HRP2JSKNT_modified_for_hrp3hand.urdf -m 1 | cut -f1 -d:`
L_END=$(sed -n "${L_START},\$p" HRP2JSKNT_modified_for_hrp3hand.urdf | grep -n "<\/gazebo>" -m 1 | cut -f1 -d:)
L_END=`expr ${L_START} + ${L_END} - 1`
sed -i -e "${L_START},${L_END}d" HRP2JSKNT_modified_for_hrp3hand.urdf

# remove RARM_JOINT7
L_START=`grep -n "<joint name=\"RARM_JOINT7\"" HRP2JSKNT_modified_for_hrp3hand.urdf -m 1 | cut -f1 -d:`
L_END=$(sed -n "${L_START},\$p" HRP2JSKNT_modified_for_hrp3hand.urdf | grep -n "<\/gazebo>" -m 1 | cut -f1 -d:)
L_END=`expr ${L_START} + ${L_END} - 1`
sed -i -e "${L_START},${L_END}d" HRP2JSKNT_modified_for_hrp3hand.urdf

# generate URDF with xacro
rosrun xacro xacro.py HRP2JSKNT_with_HRP3HAND.urdf.xacro > HRP2JSKNT_with_HRP3HAND.urdf
