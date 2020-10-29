#!/bin/bash

sleep 10 #wait for /clock
rtmlaunch hrpsys_gazebo_general samplerobot_hrpsys_bringup_indigo.launch 2>/dev/null #to avoid exceeding the maximum log length
