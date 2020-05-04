#!/bin/bash

function error {
    exit 1
}
trap error ERR

OUTPUT_FILE=$1
## change foot parameters
sed -i -e '/<gazebo reference="LLEG_LINK6">/{N;N;N;N;s@  <gazebo reference="LLEG_LINK6">\n    <mu1>0.9</mu1>\n    <mu2>0.9</mu2>\n  </gazebo>@  <gazebo reference="LLEG_LINK6">\n    <kp>1000000.0</kp>\n    <kd>100.0</kd>\n    <mu1>1.5</mu1>\n    <mu2>1.5</mu2>\n    <fdir1>1 0 0</fdir1>\n    <maxVel>10.0</maxVel>\n    <minDepth>0.00</minDepth>\n  </gazebo>@;}' ${OUTPUT_FILE}
sed -i -e '/<gazebo reference="RLEG_LINK6">/{N;N;N;N;s@  <gazebo reference="RLEG_LINK6">\n    <mu1>0.9</mu1>\n    <mu2>0.9</mu2>\n  </gazebo>@  <gazebo reference="RLEG_LINK6">\n    <kp>1000000.0</kp>\n    <kd>100.0</kd>\n    <mu1>1.5</mu1>\n    <mu2>1.5</mu2>\n    <fdir1>1 0 0</fdir1>\n    <maxVel>10.0</maxVel>\n    <minDepth>0.00</minDepth>\n  </gazebo>@;}' ${OUTPUT_FILE}
# continuous joint not working in GAZEBO
sed -i -e 's@continuous@revolute@g' ${OUTPUT_FILE}
## change max effort
sed -i -e 's@effort="100"@effort="200"@g' ${OUTPUT_FILE}
## change max velocity
sed -i -e 's@velocity="0.5"@velocity="6.0"@g' ${OUTPUT_FILE}
## change friction
sed -i -e 's@friction="0"@friction="1"@g' ${OUTPUT_FILE}
## change damping
sed -i -e 's@damping="0.2"@damping="1"@g' ${OUTPUT_FILE}
