# gazebo
```
roslaunch drcsim_gazebo atlas_v3_sandia_hands.launch
```

# hrpsys
```
rtmlaunch hrpsys_gazebo_atlas atlas_hrpsys_bringup.launch
```

# set mode and start eus
```
rosrun hrpsys_gazebo_atlas atlas_pinned.sh # set pinned if you wanto to cheat
rosrun hrpsys_gazebo_atlas hrpsys_servo_on.sh # hrpsys servo on
roscd hrpsys_gazebo_atlas/euslisp
roseus atlas-interface.l
(atlas-init-ex)
(send *atlas* :reset-pose)
(send *ri* :angle-vector (send *atlas* :angle-vector) 1000)
```
