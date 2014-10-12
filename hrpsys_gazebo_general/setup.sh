#!/bin/sh

# DRCSIM_SETUP_SH=/usr/share/drcsim/setup.sh
DRCSIM_SETUP_SH=/usr/share/gazebo/setup.sh

if [ ! -e $DRCSIM_SETUP_SH ]; then
    echo -e "\e[31mdrcsim should be installed for using hrpsys_gazebo\e[m"
    echo -e "\e[31msee http://gazebosim.org/wiki/DRC/Install\e[m"
    return
fi

ROS_PACKAGE_PATH_ORG=$ROS_PACKAGE_PATH

unset GAZEBO_MODEL_PATH
unset GAZEBO_RESOURCE_PATH
unset GAZEBO_MASTER_URI
unset GAZEBO_PLUGIN_PATH
unset GAZEBO_MODEL_DATABASE_URI

source $DRCSIM_SETUP_SH

## append original package path because /opt/ros/groovy/setup.sh have been called in drcsim/setup.sh
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH_ORG:$ROS_PACKAGE_PATH
## sort package path
export ROS_PACKAGE_PATH=`echo $(echo $ROS_PACKAGE_PATH | sed -e "s/:/\n/g" | awk '!($0 in A) && A[$0] = 1' | grep -v "opt/ros"; echo $ROS_PACKAGE_PATH | sed -e "s/:/\n/g" | awk '!($0 in A) && A[$0] = 1' | grep "opt/ros") | sed -e "s/ /:/g"`

##
pkgdir=`rospack find hrpsys_gazebo_general`

if [ -e ${pkgdir} ]; then
    export GAZEBO_RESOURCE_PATH=${pkgdir}/worlds:$GAZEBO_RESOURCE_PATH
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${pkgdir}/robot_models:${pkgdir}/..
    export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${pkgdir}/plugins
fi

