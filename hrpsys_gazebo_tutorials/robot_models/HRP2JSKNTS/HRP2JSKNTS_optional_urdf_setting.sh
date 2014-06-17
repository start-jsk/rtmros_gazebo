#!/bin/bash

function error {
    exit 1
}
trap error ERR

OUTPUT_FILE=$1
ADDITIONAL_ROS_PACKAGE_PATH=$2
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ADDITIONAL_ROS_PACKAGE_PATH
## add Plugin settings
##
sed -i -e 's@</robot>@  <gazebo>\n    <plugin filename="libIOBPlugin.so" name="hrpsys_gazebo_plugin" >\n      <robotname>HRP2JSKNTS</robotname>\n      <controller>hrpsys_gazebo_configuration</controller>\n    </plugin>\n  </gazebo>\n</robot>@g' ${OUTPUT_FILE}
# continuous joint not working in GAZEBO
sed -i -e 's@continuous@revolute@g' ${OUTPUT_FILE}

## add IMU sensor (TODO: sensors should be added when converting from collada file)
sed -i -e 's@</robot>@  <gazebo reference="CHEST_LINK1" >\n    <sensor name="waist_imu" type="imu">\n      <always_on>1</always_on>\n      <update_rate>1000.0</update_rate>\n      <imu>\n        <noise>\n          <type>gaussian</type>\n          <rate>\n            <mean>0.0</mean>\n            <stddev>2e-4</stddev>\n            <bias_mean>0.0000075</bias_mean>\n            <bias_stddev>0.0000008</bias_stddev>\n          </rate>\n          <accel>\n            <mean>0.0</mean>\n            <stddev>1.7e-2</stddev>\n            <bias_mean>0.1</bias_mean>\n            <bias_stddev>0.001</bias_stddev>\n          </accel>\n        </noise>\n      </imu>\n    </sensor>\n  </gazebo>\n</robot>@g' ${OUTPUT_FILE}

## add force sensors (TODO: sensors should be added when converting from collada file)
sed -i -e 's@</robot>@  <gazebo reference="LLEG_JOINT5">\n    <provideFeedback>1</provideFeedback>\n  </gazebo>\n  <gazebo reference="RLEG_JOINT5">\n    <provideFeedback>1</provideFeedback>\n  </gazebo>\n  <gazebo reference="LARM_JOINT6">\n    <provideFeedback>1</provideFeedback>\n  </gazebo>\n  <gazebo reference="RARM_JOINT6">\n    <provideFeedback>1</provideFeedback>\n  </gazebo>\n</robot>@g' ${OUTPUT_FILE}

# overwrite mass and inertia which have invalid settings.
sed -i -e 's@<mass value="0" />@<mass value="1e-03" />@g' ${OUTPUT_FILE}
sed -i -e 's@<inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>@<inertia ixx="1e-04" ixy="0" ixz="0" iyy="1e-04" iyz="0" izz="1e-04"/>@g' ${OUTPUT_FILE}

## change foot parameters
sed -i -e '/<gazebo reference="LLEG_LINK5">/{N;N;N;N;s@  <gazebo reference="LLEG_LINK5">\n    <mu1>0.9</mu1>\n    <mu2>0.9</mu2>\n  </gazebo>@  <gazebo reference="LLEG_LINK5">\n    <kp>1000000.0</kp>\n    <kd>100.0</kd>\n    <mu1>1.5</mu1>\n    <mu2>1.5</mu2>\n    <fdir1>1 0 0</fdir1>\n    <maxVel>10.0</maxVel>\n    <minDepth>0.00</minDepth>\n  </gazebo>@;}' ${OUTPUT_FILE}
sed -i -e '/<gazebo reference="RLEG_LINK5">/{N;N;N;N;s@  <gazebo reference="RLEG_LINK5">\n    <mu1>0.9</mu1>\n    <mu2>0.9</mu2>\n  </gazebo>@  <gazebo reference="RLEG_LINK5">\n    <kp>1000000.0</kp>\n    <kd>100.0</kd>\n    <mu1>1.5</mu1>\n    <mu2>1.5</mu2>\n    <fdir1>1 0 0</fdir1>\n    <maxVel>10.0</maxVel>\n    <minDepth>0.00</minDepth>\n  </gazebo>@;}' ${OUTPUT_FILE}

# ## change foot geometry mesh -> box
# sed -i -e '/<collision>/{N;N;N;s@<collision>\n      <origin xyz="0 0 0" rpy="0 -0 0"/>\n      <geometry>\n        <mesh filename="package://HRP2JSKNTS/meshes/LLEG_LINK5_mesh.dae" scale="1 1 1" />@<collision>\n      <origin xyz="0.054 0.010 -0.070" rpy="0 -0 0"/>\n      <geometry>\n        <box size="0.2412 0.138 0.07"/>@;}' ${OUTPUT_FILE}
# sed -i -e '/<collision>/{N;N;N;s@<collision>\n      <origin xyz="0 0 0" rpy="0 -0 0"/>\n      <geometry>\n        <mesh filename="package://HRP2JSKNTS/meshes/RLEG_LINK5_mesh.dae" scale="1 1 1" />@<collision>\n      <origin xyz="0.054 -0.010 -0.070" rpy="0 -0 0"/>\n      <geometry>\n        <box size="0.2412 0.138 0.07"/>@;}' ${OUTPUT_FILE}

# ## delete toe link collision
# sed -i -e '/<collision>/{N;N;N;N;N;s@<collision>\n      <origin xyz="0.115 0 -0.074" rpy="0 -0 0"/>\n      <geometry>\n        <mesh filename="package://HRP2JSKNTS/meshes/LLEG_LINK6_mesh.dae" scale="1 1 1" />\n      </geometry>\n    </collision>@@;}' ${OUTPUT_FILE}
# sed -i -e '/<collision>/{N;N;N;N;N;s@<collision>\n      <origin xyz="0.115 0 -0.074" rpy="0 -0 0"/>\n      <geometry>\n        <mesh filename="package://HRP2JSKNTS/meshes/RLEG_LINK6_mesh.dae" scale="1 1 1" />\n      </geometry>\n    </collision>@@;}' ${OUTPUT_FILE}

# remove LARM_LINK6
L_START=`grep -n "<link name=\"LARM_LINK6\"" -m 1 ${OUTPUT_FILE} -m 1 | cut -f1 -d:`
L_END=$(sed -n "${L_START},\$p" ${OUTPUT_FILE} | grep -n "<\/gazebo>" -m 1 | cut -f1 -d:) ##
L_END=`expr ${L_START} + ${L_END} - 1`
sed -i -e "${L_START},${L_END}d" ${OUTPUT_FILE}
# remove RARM_LINK6
L_START=`grep -n "<link name=\"RARM_LINK6\"" ${OUTPUT_FILE} -m 1 | cut -f1 -d:`

L_END=$(sed -n "${L_START},\$p" ${OUTPUT_FILE} | grep -n "<\/gazebo>" -m 1 | cut -f1 -d:) ##
L_END=`expr ${L_START} + ${L_END} - 1`
sed -i -e "${L_START},${L_END}d" ${OUTPUT_FILE}
# continuous joint not working in GAZEBO
sed -i -e 's@continuous@revolute@g' ${OUTPUT_FILE}
# overwrite velocity limit because collada2urdf doesn't reflect the velocity limit of collada model.
sed -i -e 's@velocity="0.5"@velocity="1000.0"@g' ${OUTPUT_FILE}
sed -i -e 's@effort="100"@effort="1000"@g' ${OUTPUT_FILE}


# generate URDF with xacro
cp ${OUTPUT_FILE} `echo ${OUTPUT_FILE} | sed "s/.urdf/_only_body.urdf/g"`
rosrun xacro xacro.py `echo ${OUTPUT_FILE} | sed "s/.urdf/.urdf.xacro/g"` > ${OUTPUT_FILE}
