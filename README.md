rtmros_gazebo  [![Build Status](https://travis-ci.org/start-jsk/rtmros_gazebo.png)](https://travis-ci.org/start-jsk/rtmros_gazebo)
-------------

[![Join the chat at https://gitter.im/start-jsk/rtmros_gazebo](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/start-jsk/rtmros_gazebo?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

[gazebo] simulation for rtmros robots

### Install

Please refer [rtmros_common] for installing these packages.

### Package Description

### [hrpsys_gazebo_general]

This package consists iob.cpp which is low-level interface of RobotHardware on hrpsys and IOBPlugin.cpp which is gazebo plugin to comunicate with iob.cpp.

#### Environment variables used in iob.cpp

- HRPSYS_GAZEBO_IOB_NAME
    - ROS node name of hrpsys(RobotHardware) node. (default: "hrpsys_gazebo_iob")
- HRPSYS_GAZEBO_CONFIGURATION
    - ROS namespace of configuration parameters. (default: "hrpsys_gazebo_configuration")
- HRPSYS_GAZEBO_ROBOTNAME
    - Robot Name
- HRPSYS_GAZEBO_IOB_SYNCHRONIZED
    - Synchronized mode between  hrpsys step and gazebo step. (default: false)
- HRPSYS_GAZEBO_IOB_SUBSTEPS
    - Number of substeps. Controlling command will be sent in every substeps. (default: 1)

### [hrpsys_gazebo_tutorials]

This package is a collection of examples for using hrpsys_gazebo system and utility scripts.

- You should prepare robot model file. Supported types of model file are collada(openrave) and VRML(openhrp3). URDF and OpenRAVE xml can be used by converting to collada.
    - *&lt;robot_name&gt;*.yaml for configurating gazebo setting and hrpsys setting
    - (automatically generated) *&lt;robot_name&gt;*.urdf under robot_models/*&lt;robot_name&gt;* directory
    - (automatically generated) hrpsys settings (you should have a collada or VRML robot model file)
    - *&lt;robot_name&gt;*_optional_urdf_setting.sh under robot_models/*&lt;robot_name&gt;* directory, this is for adding description used by gazebo (such as sensor settings, collision and friction setting)

#### Setting of *&lt;robot_name&gt;*.yaml files

This is yaml file for configuring gazebo setting.

    # top level name space()
    hrpsys_gazebo_configuration:
    # velocity feedback for joint control, use parameter gains/joint_name/p_v
      use_velocity_feedback: true
    # synchronized hrpsys and gazebo
      use_synchronized_command: false
    # name of robot (using for namespace)
      robotname: SampleRobot
    # joint_id (order) conversion from gazebo to hrpsys, joint_id_list[gazebo_id] := hrpsys_id
      joint_id_list: [0, ... , 28]
    # joints list used in gazebo, sizeof(joint_id_list) == sizeof(joints)
      joints:
        - RLEG_HIP_R
        - CHEST
    # joint gain settings
    # Torque feedback mode
    # effort := p * error + d * d/dt error + i * sigma (error) + vp * velocity_error
    #   error := reference_position - current_position
    #   velocity_error := reference_velocity - current_velocity
    # Velociy feedback mode
    # desired_velocity := p_v * error + reference_velocity
      gains:
        LLEG_HIP_R:      {p: 12000.0, d:  4.0, i: 0.0, vp:  6.0, i_clamp: 0.0, p_v: 250.0}
        RARM_WRIST_R:    {p:    20.0, d:  0.1, i: 0.0, vp:  0.0, i_clamp: 0.0, p_v: 100.0}
    # force sensor settings
    #   list of force sensorname
      force_torque_sensors:
        - lfsensor
        - rfsensor
    # configuration of force sensor
    #   key of force_torque_sensors_config should be a member of force_torque_sensors
      force_torque_sensors_config:
        lfsensor: {joint_name: 'JOINT_NAME0', frame_id: 'LINK_NAME0', translation: [0, 0, 0], rotation: [1, 0, 0, 0]}
        rfsensor: {joint_name: 'JOINT_NAME1', frame_id: 'LINK_NAME0', translation: [0, 0, 0], rotation: [1, 0, 0, 0]}
    # IMU sensor settings
    # configuration of IMU sensor
    #   key of imu_sensors_config should be a member of imu_sensors
      imu_sensors:
        - imu_sensor0
      imu_sensors_config:
        imu_sensor0: {ros_name: 'ros_imu_sensor', link_name: 'LINK_NAME0', frame_id: 'LINK_NAME0'}

#### (automatically generated files)

You can use robot_models/install_robot_common.sh for installing urdf model file. This scripts converts collada file in [hrpsys_ros_bridge_tutorials]/models directory to urdf file. 

    ./install_robot_common.sh ROBOT_NAME (model directory) (output directory) (collada_to_urdf_binary) (additional_ros_package_path)

[gazebo]:http://gazebosim.org
[rtmros_common]:https://github.com/start-jsk/rtmros_common
[hrpsys_gazebo_general]:https://github.com/start-jsk/rtmros_gazebo/tree/master/hrpsys_gazebo_general
[hrpsys_gazebo_tutorials]:https://github.com/start-jsk/rtmros_gazebo/tree/master/hrpsys_gazebo_tutorials
[hrpsys_ros_bridge_tutorials]:https://github.com/start-jsk/rtmros_tutorials/tree/master/hrpsys_ros_bridge_tutorials
