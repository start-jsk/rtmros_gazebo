#!/bin/bash

set -x

#
cd catkin_ws
mkdir src
catkin_workspace_init
wstool init
wstool merge https://rtm-ros-robotics.googlecode.com/svn-history/trunk/rtmros_gazebo/hrpsys_gazebo_atlas/dot.rosinstall
wstool update
cd ..

#
sudo apt-get update
rosdep udpate
rosdep install --from-paths src --ignore-src --rosdistro groovy -y -r
sudo apt-get install ros-groovy-hrpsys-ros-bridge ## this should installed as rosdep install command
catkin_make

##


