#!/bin/bash

rosservice call /RobotHardwareServiceROSBridge/power '{name: "all", ss: 0}'
sleep 1
rosservice call /StateHolderServiceROSBridge/goActual
sleep 2
rosservice call /RobotHardwareServiceROSBridge/servo '{name: "all", ss: 0}'
