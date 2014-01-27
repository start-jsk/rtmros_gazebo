#!/bin/bash

OUTPUT_FILE=$1

## add Plugin settings
##
sed -i -e 's@</robot>@  <gazebo>\n    <plugin filename="libIOBPlugin.so" name="hrpsys_gazebo_plugin" >\n      <robotname>HRP2JSKNT</robotname>\n      <controller>hrpsys_gazebo_configuration</controller>\n    </plugin>\n  </gazebo>\n</robot>@g' ${OUTPUT_FILE}
# continuous joint not working in GAZEBO
sed -i -e 's@continuous@revolute@g' ${OUTPUT_FILE}

# overwrite mass and inertia which have invalid settings.
sed -i -e 's@<mass value="0" />@<mass value="1" />@g' ${OUTPUT_FILE}
sed -i -e 's@<inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>@<inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>@g' ${OUTPUT_FILE}

## change foot parameters
sed -i -e '/<gazebo reference="LLEG_LINK5">/{N;N;N;N;s@  <gazebo reference="LLEG_LINK5">\n    <mu1>0.9</mu1>\n    <mu2>0.9</mu2>\n  </gazebo>@  <gazebo reference="LLEG_LINK5">\n    <kp>1000000.0</kp>\n    <kd>100.0</kd>\n    <mu1>1.5</mu1>\n    <mu2>1.5</mu2>\n    <fdir1>1 0 0</fdir1>\n    <maxVel>10.0</maxVel>\n    <minDepth>0.00</minDepth>\n  </gazebo>@;}' ${OUTPUT_FILE}
sed -i -e '/<gazebo reference="RLEG_LINK5">/{N;N;N;N;s@  <gazebo reference="RLEG_LINK5">\n    <mu1>0.9</mu1>\n    <mu2>0.9</mu2>\n  </gazebo>@  <gazebo reference="RLEG_LINK5">\n    <kp>1000000.0</kp>\n    <kd>100.0</kd>\n    <mu1>1.5</mu1>\n    <mu2>1.5</mu2>\n    <fdir1>1 0 0</fdir1>\n    <maxVel>10.0</maxVel>\n    <minDepth>0.00</minDepth>\n  </gazebo>@;}' ${OUTPUT_FILE}

## change foot geometry mesh -> box
sed -i -e '/<collision>/{N;N;N;s@<collision>\n      <origin xyz="0 0 0" rpy="0 -0 0"/>\n      <geometry>\n        <mesh filename="package://HRP2JSKNT/meshes/LLEG_LINK5_mesh.dae" scale="1 1 1" />@<collision>\n      <origin xyz="0.054 0.010 -0.070" rpy="0 -0 0"/>\n      <geometry>\n        <box size="0.2412 0.138 0.07"/>@;}' ${OUTPUT_FILE}
sed -i -e '/<collision>/{N;N;N;s@<collision>\n      <origin xyz="0 0 0" rpy="0 -0 0"/>\n      <geometry>\n        <mesh filename="package://HRP2JSKNT/meshes/RLEG_LINK5_mesh.dae" scale="1 1 1" />@<collision>\n      <origin xyz="0.054 -0.010 -0.070" rpy="0 -0 0"/>\n      <geometry>\n        <box size="0.2412 0.138 0.07"/>@;}' ${OUTPUT_FILE}

## delete toe link collision
sed -i -e 's@    <collision>\n      <origin xyz="0.115 0 -0.074" rpy="0 -0 0"/>\n      <geometry>\n        <mesh filename="package://HRP2JSKNT/meshes/LLEG_LINK6_mesh.dae" scale="1 1 1" />\n      </geometry>\n    </collision>@@g' ${OUTPUT_FILE}
sed -i -e 's@    <collision>\n      <origin xyz="0.115 0 -0.074" rpy="0 -0 0"/>\n      <geometry>\n        <mesh filename="package://HRP2JSKNT/meshes/RLEG_LINK6_mesh.dae" scale="1 1 1" />\n      </geometry>\n    </collision>@@g' ${OUTPUT_FILE}

# continuous joint not working in GAZEBO
sed -i -e 's@continuous@revolute@g' ${OUTPUT_FILE}

# overwrite velocity limit because collada2urdf doesn't reflect the velocity limit of collada model.
sed -i -e 's@velocity="0.5"@velocity="10.0"@g' ${OUTPUT_FILE}
sed -i -e 's@effort="100"@effort="1000"@g' ${OUTPUT_FILE}
