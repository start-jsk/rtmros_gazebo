#!/bin/bash

rosservice call /RobotHardwareServiceROSBridge/power '{name: "all", ss: 0}'
sleep 1
./resetJointGroup.py RobotHardware0 $(rospack find hrpsys_gazebo_atlas)/models/atlas_v3.dae -ORBInitRef NameService=corbaloc:iiop:localhost:15005/NameService
sleep 1
rosservice call /StateHolderServiceROSBridge/goActual
sleep 1
rosservice call /RobotHardwareServiceROSBridge/servo '{name: "all", ss: 0}'
