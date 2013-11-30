#!/bin/bash

rosservice call /RobotHardwareServiceROSBridge/servo '{name: "all", ss: 1}'
sleep 1
rosservice call /RobotHardwareServiceROSBridge/power '{name: "all", ss: 1}'
