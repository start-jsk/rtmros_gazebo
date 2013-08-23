#!/bin/bash

if [ ! -e hrpsys ]; then
##
    mkdir meshes
    rosrun collada_tools collada_to_urdf $(rospack find hrpsys_ros_bridge_tutorials)/models/SampleRobot.dae -G -A --mesh_output_dir `pwd`/meshes --mesh_prefix 'package://SampleRobot/meshes'
## add Plugin settings
    sed -i -e 's@</robot>@  <gazebo>\n    <plugin filename="libIOBPlugin.so" name="hrpsys_gazebo_plugin" >\n      <robotname>SampleRobot</robotname>\n      <controller>hrpsys_gazebo_configuration</controller>\n    </plugin>\n  </gazebo>\n</robot>@g' SampleRobot.urdf
## add IMU sensor (TODO: sensors should be added when converting from collada file)
    sed -i -e 's@</robot>@  <link name="gyrometer" />\n  <joint name="gyrometer_jnt" type="fixed">\n    <parent link="WAIST_LINK0"/>\n    <child  link="gyrometer"/>\n  </joint>\n  <gazebo reference="WAIST_LINK0" >\n    <sensor name="sample_imu_sensor" type="imu">\n      <always_on>1</always_on>\n      <update_rate>1000.0</update_rate>\n      <imu>\n        <noise>\n          <type>gaussian</type>\n          <rate>\n            <mean>0.0</mean>\n            <stddev>2e-4</stddev>\n            <bias_mean>0.0000075</bias_mean>\n            <bias_stddev>0.0000008</bias_stddev>\n          </rate>\n          <accel>\n            <mean>0.0</mean>\n            <stddev>1.7e-2</stddev>\n            <bias_mean>0.1</bias_mean>\n            <bias_stddev>0.001</bias_stddev>\n          </accel>\n        </noise>\n      </imu>\n    </sensor>\n  </gazebo>\n</robot>@g' SampleRobot.urdf
## add forse sensors (TODO: sensors should be added when converting from collada file)
    sed -i -e 's@</robot>@  <gazebo reference="LLEG_ANKLE_R">\n    <provideFeedback>1</provideFeedback>\n  </gazebo>\n  <gazebo reference="RLEG_ANKLE_R">\n    <provideFeedback>1</provideFeedback>\n  </gazebo>\n  <gazebo reference="LARM_WRIST_R">\n    <provideFeedback>1</provideFeedback>\n  </gazebo>\n  <gazebo reference="RARM_WRIST_R">\n    <provideFeedback>1</provideFeedback>\n  </gazebo>\n</robot>@g' SampleRobot.urdf
# continuous joint not working in GAZEBO
    sed -i -e 's@continuous@revolute@g' SampleRobot.urdf

    mkdir hrpsys
    ln -s  $(rospack find hrpsys_ros_bridge_tutorials)/models/SampleRobot.conf hrpsys
    ln -s  $(rospack find hrpsys_ros_bridge_tutorials)/models/SampleRobot.RobotHardware.conf hrpsys
    ln -s  $(rospack find hrpsys_ros_bridge_tutorials)/models/SampleRobot.dae hrpsys
    ln -s  $(rospack find hrpsys_ros_bridge_tutorials)/models/SampleRobot_nosim.xml hrpsys
fi
