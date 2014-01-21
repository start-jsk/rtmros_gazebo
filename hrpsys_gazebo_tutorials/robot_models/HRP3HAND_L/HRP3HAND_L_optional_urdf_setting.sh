#!/bin/bash

OUTPUT_FILE=$1

## add Plugin settings
##
sed -i -e 's@</robot>@  <gazebo>\n    <plugin filename="libIOBPlugin.so" name="hrpsys_gazebo_plugin" >\n      <robotname>HRP3HAND_L</robotname>\n      <controller>hrpsys_gazebo_configuration</controller>\n    </plugin>\n  </gazebo>\n</robot>@g' ${OUTPUT_FILE}
# continuous joint not working in GAZEBO
sed -i -e 's@continuous@revolute@g' ${OUTPUT_FILE}

# overwrite inertia because original value is too small
for l in `grep -n "inertia " ${OUTPUT_FILE} | cut -f1 -d:`
do
  sed -i "${l}c\      <inertia ixx=\"1\" ixy=\"0\" ixz=\"0\" iyy=\"1\" iyz=\"0\" izz=\"1\"/>" ${OUTPUT_FILE}
done


