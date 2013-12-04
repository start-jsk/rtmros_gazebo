#!/bin/bash

rosservice call /RobotHardwareServiceROSBridge/servo '{name: "all", ss: 1}'
sleep 1
rosservice call /RobotHardwareServiceROSBridge/power '{name: "all", ss: 1}'
#./resetJointGroup.py RobotHardware0 $(rospack find hrpsys_gazebo_atlas)/models/atlas_v3.dae -ORBInitRef NameService=corbaloc:iiop:localhost:15005/NameService