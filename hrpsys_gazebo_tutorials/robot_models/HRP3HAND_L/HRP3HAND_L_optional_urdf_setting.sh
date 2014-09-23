#!/bin/bash

function error {
    exit 1
}
trap error ERR

OUTPUT_FILE=$1

## add Plugin settings
##
sed -i -e 's@</robot>@  <gazebo>\n    <plugin filename="libIOBPlugin.so" name="hrpsys_gazebo_plugin" >\n      <robotname>HRP3HAND_L</robotname>\n      <controller>hrpsys_gazebo_configuration</controller>\n    </plugin>\n  </gazebo>\n</robot>@g' ${OUTPUT_FILE}
# continuous joint not working in GAZEBO
sed -i -e 's@continuous@revolute@g' ${OUTPUT_FILE}

# overwrite inertia because original value is too small
for l in `grep -n "inertia " ${OUTPUT_FILE} | cut -f1 -d:`
do
  sed -i "${l}c\      <inertia ixx=\"1e-02\" ixy=\"0\" ixz=\"0\" iyy=\"1e-02\" iyz=\"0\" izz=\"1e-02\"/>" ${OUTPUT_FILE}
done

sed -i -e '/<gazebo reference="L_INDEXMP_R_LINK">/{N;N;N;N;s@  <gazebo reference="L_INDEXMP_R_LINK">\n    <mu1>0.9</mu1>\n    <mu2>0.9</mu2>\n  </gazebo>@  <gazebo reference="L_INDEXMP_R_LINK">\n    <kp>140.0</kp>\n    <kd>2800.0</kd>\n    <mu1>1.5</mu1>\n    <mu2>1.5</mu2>\n    <fdir1>1 0 0</fdir1>\n    <maxVel>10.0</maxVel>\n    <minDepth>0.00</minDepth>\n  </gazebo>@;}' ${OUTPUT_FILE}
sed -i -e '/<gazebo reference="L_INDEXMP_P_LINK">/{N;N;N;N;s@  <gazebo reference="L_INDEXMP_P_LINK">\n    <mu1>0.9</mu1>\n    <mu2>0.9</mu2>\n  </gazebo>@  <gazebo reference="L_INDEXMP_P_LINK">\n    <kp>140.0</kp>\n    <kd>2800.0</kd>\n    <mu1>1.5</mu1>\n    <mu2>1.5</mu2>\n    <fdir1>1 0 0</fdir1>\n    <maxVel>10.0</maxVel>\n    <minDepth>0.00</minDepth>\n  </gazebo>@;}' ${OUTPUT_FILE}
sed -i -e '/<gazebo reference="L_INDEXMPIP_R_LINK">/{N;N;N;N;s@  <gazebo reference="L_INDEXMPIP_R_LINK">\n    <mu1>0.9</mu1>\n    <mu2>0.9</mu2>\n  </gazebo>@  <gazebo reference="L_INDEXMPIP_R_LINK">\n    <kp>140.0</kp>\n    <kd>2800.0</kd>\n    <mu1>1.5</mu1>\n    <mu2>1.5</mu2>\n    <fdir1>1 0 0</fdir1>\n    <maxVel>10.0</maxVel>\n    <minDepth>0.00</minDepth>\n  </gazebo>@;}' ${OUTPUT_FILE}
sed -i -e '/<gazebo reference="L_MIDDLEPIP_R_LINK">/{N;N;N;N;s@  <gazebo reference="L_MIDDLEPIP_R_LINK">\n    <mu1>0.9</mu1>\n    <mu2>0.9</mu2>\n  </gazebo>@  <gazebo reference="L_MIDDLEPIP_R_LINK">\n    <kp>140.0</kp>\n    <kd>2800.0</kd>\n    <mu1>1.5</mu1>\n    <mu2>1.5</mu2>\n    <fdir1>1 0 0</fdir1>\n    <maxVel>10.0</maxVel>\n    <minDepth>0.00</minDepth>\n  </gazebo>@;}' ${OUTPUT_FILE}
sed -i -e '/<gazebo reference="L_THUMBCM_Y_LINK">/{N;N;N;N;s@  <gazebo reference="L_THUMBCM_Y_LINK">\n    <mu1>0.9</mu1>\n    <mu2>0.9</mu2>\n  </gazebo>@  <gazebo reference="L_THUMBCM_Y_LINK">\n    <kp>140.0</kp>\n    <kd>2800.0</kd>\n    <mu1>1.5</mu1>\n    <mu2>1.5</mu2>\n    <fdir1>1 0 0</fdir1>\n    <maxVel>10.0</maxVel>\n    <minDepth>0.00</minDepth>\n  </gazebo>@;}' ${OUTPUT_FILE}
sed -i -e '/<gazebo reference="L_THUMBCM_P_LINK">/{N;N;N;N;s@  <gazebo reference="L_THUMBCM_P_LINK">\n    <mu1>0.9</mu1>\n    <mu2>0.9</mu2>\n  </gazebo>@  <gazebo reference="L_THUMBCM_P_LINK">\n    <kp>140.0</kp>\n    <kd>2800.0</kd>\n    <mu1>1.5</mu1>\n    <mu2>1.5</mu2>\n    <fdir1>1 0 0</fdir1>\n    <maxVel>10.0</maxVel>\n    <minDepth>0.00</minDepth>\n  </gazebo>@;}' ${OUTPUT_FILE}
sed -i -e '/<gazebo reference="L_INDEXMP_R">/{N;N;N;N;s@  <gazebo reference="L_INDEXMP_R">\n    <mu1>0.9</mu1>\n    <mu2>0.9</mu2>\n  </gazebo>@  <gazebo reference="L_INDEXMP_R">\n    <kp>140.0</kp>\n    <kd>2800.0</kd>\n    <mu1>1.5</mu1>\n    <mu2>1.5</mu2>\n    <fdir1>1 0 0</fdir1>\n    <maxVel>10.0</maxVel>\n    <minDepth>0.00</minDepth>\n  </gazebo>@;}' ${OUTPUT_FILE}
sed -i -e '/<gazebo reference="L_INDEXPIP_R_LINK">/{N;N;N;N;s@  <gazebo reference="L_INDEXPIP_R_LINK">\n    <mu1>0.9</mu1>\n    <mu2>0.9</mu2>\n  </gazebo>@  <gazebo reference="L_INDEXPIP_R_LINK">\n    <kp>140.0</kp>\n    <kd>2800.0</kd>\n    <mu1>1.5</mu1>\n    <mu2>1.5</mu2>\n    <fdir1>1 0 0</fdir1>\n    <maxVel>10.0</maxVel>\n    <minDepth>0.00</minDepth>\n  </gazebo>@;}' ${OUTPUT_FILE}

