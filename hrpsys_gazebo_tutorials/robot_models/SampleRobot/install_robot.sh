#!/bin/bash

if [ ! -e hrpsys ]; then
##
    mkdir meshes
    rosrun collada_tools collada_to_urdf $(rospack find hrpsys_ros_bridge_tutorials)/models/SampleRobot.dae -G -A --mesh_output_dir `pwd`/meshes --mesh_prefix 'package://SampleRobot/meshes'
## add Plugin settings
    sed -i -e 's@</robot>@  <gazebo>\n    <plugin filename="libIOBPlugin.so" name="hrpsys_gazebo_plugin" >\n      <robotname>SampleRobot</robotname>\n      <controller>hrpsys_gazebo_configuration</controller>\n    </plugin>\n  </gazebo>\n</robot>@g' SampleRobot.urdf
## add IMU sensor (TODO: sensors should be added when converting from collada file)
    sed -i -e 's@</robot>@  <gazebo reference="WAIST_LINK0" >\n    <sensor name="sample_imu_sensor" type="imu">\n      <always_on>1</always_on>\n      <update_rate>1000.0</update_rate>\n      <imu>\n        <noise>\n          <type>gaussian</type>\n          <rate>\n            <mean>0.0</mean>\n            <stddev>2e-4</stddev>\n            <bias_mean>0.0000075</bias_mean>\n            <bias_stddev>0.0000008</bias_stddev>\n          </rate>\n          <accel>\n            <mean>0.0</mean>\n            <stddev>1.7e-2</stddev>\n            <bias_mean>0.1</bias_mean>\n            <bias_stddev>0.001</bias_stddev>\n          </accel>\n        </noise>\n      </imu>\n    </sensor>\n  </gazebo>\n</robot>@g' SampleRobot.urdf
## add forse sensors (TODO: sensors should be added when converting from collada file)
    sed -i -e 's@</robot>@  <gazebo reference="LLEG_ANKLE_R">\n    <provideFeedback>1</provideFeedback>\n  </gazebo>\n  <gazebo reference="RLEG_ANKLE_R">\n    <provideFeedback>1</provideFeedback>\n  </gazebo>\n  <gazebo reference="LARM_WRIST_R">\n    <provideFeedback>1</provideFeedback>\n  </gazebo>\n  <gazebo reference="RARM_WRIST_R">\n    <provideFeedback>1</provideFeedback>\n  </gazebo>\n</robot>@g' SampleRobot.urdf
## change foot parameters
    sed -i -e '/<gazebo reference="LLEG_LINK6">/{N;N;N;N;s@  <gazebo reference="LLEG_LINK6">\n    <mu1>0.9</mu1>\n    <mu2>0.9</mu2>\n  </gazebo>@  <gazebo reference="LLEG_LINK6">\n    <kp>1000000.0</kp>\n    <kd>100.0</kd>\n    <mu1>1.5</mu1>\n    <mu2>1.5</mu2>\n    <fdir1>1 0 0</fdir1>\n    <maxVel>10.0</maxVel>\n    <minDepth>0.00</minDepth>\n  </gazebo>@;}' SampleRobot.urdf
    sed -i -e '/<gazebo reference="RLEG_LINK6">/{N;N;N;N;s@  <gazebo reference="RLEG_LINK6">\n    <mu1>0.9</mu1>\n    <mu2>0.9</mu2>\n  </gazebo>@  <gazebo reference="RLEG_LINK6">\n    <kp>1000000.0</kp>\n    <kd>100.0</kd>\n    <mu1>1.5</mu1>\n    <mu2>1.5</mu2>\n    <fdir1>1 0 0</fdir1>\n    <maxVel>10.0</maxVel>\n    <minDepth>0.00</minDepth>\n  </gazebo>@;}' SampleRobot.urdf
# continuous joint not working in GAZEBO
    sed -i -e 's@continuous@revolute@g' SampleRobot.urdf

    mkdir hrpsys
    ln -s  $(rospack find hrpsys_ros_bridge_tutorials)/models/SampleRobot.conf hrpsys
    ln -s  $(rospack find hrpsys_ros_bridge_tutorials)/models/SampleRobot.RobotHardware.conf hrpsys
    ln -s  $(rospack find hrpsys_ros_bridge_tutorials)/models/SampleRobot.dae hrpsys
    ln -s  $(rospack find hrpsys_ros_bridge_tutorials)/models/SampleRobot_nosim.xml hrpsys
fi
