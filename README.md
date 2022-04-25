rtmros_gazebo  [![Build Status](https://app.travis-ci.com/start-jsk/rtmros_gazebo.svg?branch=master)](https://app.travis-ci.com/start-jsk/rtmros_gazebo)
-------------

[gazebo] simulation for rtmros robots

### Install

Please refer [rtmros_common] for installing these packages.

### Try Sample
Open Terminal and run gazebo

```
roslaunch hrpsys_gazebo_general gazebo_samplerobot_no_controllers.launch  # kinetic and above
roslaunch hrpsys_gazebo_general gazebo_samplerobot_no_controllers_indigo.launch  # indigo
```
Launch another terminal and start hrpsys-base
```
rtmlaunch hrpsys_gazebo_general samplerobot_hrpsys_bringup.launch  # kinetic and above
rtmlaunch hrpsys_gazebo_general samplerobot_hrpsys_bringup_indigo.launch  # indigo
```
Launch another terminal and send command to robot by roseus
```
roscd hrpsys_ros_bridge/euslisp/
roseus samplerobot-interface.l
(samplerobot-init)
(setq *robot* (instance samplerobot-robot :init))
(send *ri* :angle-vector (send *robot* :reset-pose) 5000)
(send *ri* :start-auto-balancer)
(send *ri* :start-st)
(send *ri* :go-pos 0 0 0)
```

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
    # pose of imu_sensor relative to parent link should be specified in URDF.
      imu_sensors:
        - imu_sensor0
      imu_sensors_config:
        imu_sensor0: {ros_name: 'ros_imu_sensor', link_name: 'LINK_NAME0', frame_id: 'LINK_NAME0'}

#### Custom Plugins

#### CranePlugin

This plugin provides fake "Crane" in gazebo environment.

To use this plugin, add the following lines to your URDF.
```
<gazebo>
  <plugin filename="libCranePlugin.so" name="crane_plugin">
    <linkname>CHEST_LINK1</linkname>  <!-- The name of the link where the crane is attached -->
    <liftheight>1.2</liftheight>  <!-- Crane hanging height [m] -->
    <lowerheight>0.5</lowerheight>  <!-- Crane will be disabled below this height [m] -->
    <liftvelocity>0.1</liftvelocity>  <!-- [m/s] -->
    <lowervelocity>0.03</lowervelocity>  <!-- [m/s] -->
    <pgain>2500</pgain>  <!-- P gain for Z-axis error -->
    <dgain>500</dgain>  <!-- D gain for Z-axis error -->
    <damp>10000</damp>  <!-- Damping factor for other axes and rotation -->
  </plugin>
</gazebo>
```

##### Subscribed topics

- `[objname]/CranePlugin/LowerCommand` (`std_msgs::Empty`)

Lower the crane to the ground.

- `[objname]/CranePlugin/LiftCommand` (`std_msgs::Empty`)

Lift the crane to the lift height.

- `[objname]/CranePlugin/PoseCommand` (`geometry_msgs::Pose`)

Place the robot to the pose.

[gazebo]:http://gazebosim.org
[rtmros_common]:https://github.com/start-jsk/rtmros_common
[hrpsys_gazebo_general]:https://github.com/start-jsk/rtmros_gazebo/tree/master/hrpsys_gazebo_general
