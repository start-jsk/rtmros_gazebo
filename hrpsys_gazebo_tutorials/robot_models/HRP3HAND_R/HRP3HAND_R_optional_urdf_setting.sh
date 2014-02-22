#!/bin/bash

function error {
    exit 1
}
trap error ERR

OUTPUT_FILE=$1

# add Plugin settings
sed -i -e 's@</robot>@  <gazebo>\n    <plugin filename="libIOBPlugin.so" name="hrpsys_gazebo_plugin" >\n      <robotname>HRP3HAND_R</robotname>\n      <controller>hrpsys_gazebo_configuration</controller>\n    </plugin>\n  </gazebo>\n</robot>@g' ${OUTPUT_FILE}

# continuous joint not working in GAZEBO
sed -i -e 's@continuous@revolute@g' ${OUTPUT_FILE}

# overwrite inertia because original value is too small
for l in `grep -n "inertia " ${OUTPUT_FILE} | cut -f1 -d:`
do
  sed -i "${l}c\      <inertia ixx=\"1e-02\" ixy=\"0\" ixz=\"0\" iyy=\"1e-02\" iyz=\"0\" izz=\"1e-02\"/>" ${OUTPUT_FILE}
done

cp ${OUTPUT_FILE} `echo ${OUTPUT_FILE} | sed "s/.urdf/_with_plugin.urdf/g"`

# add Plugin settings for setting and getting force and velocity
sed -i -e 's@</robot>@  <gazebo>\n    <plugin filename="libAddForcePlugin.so" name="HRP3HAND_R_add_force_plugin" >\n      <objname>HRP3HAND_R</objname>\n      <linkname>RARM_LINK6</linkname>\n    </plugin>\n    <plugin filename="libGetVelPlugin.so" name="HRP3HAND_R_get_vel_plugin" >\n      <objname>HRP3HAND_R</objname>\n      <linkname>RARM_LINK6</linkname>\n    </plugin>\n    <plugin filename="libSetVelPlugin.so" name="HRP3HAND_R_set_vel_plugin" >\n      <objname>HRP3HAND_R</objname>\n      <linkname>RARM_LINK6</linkname>\n    </plugin>\n  </gazebo>\n</robot>@g' `echo ${OUTPUT_FILE} | sed "s/.urdf/_with_plugin.urdf/g"`
