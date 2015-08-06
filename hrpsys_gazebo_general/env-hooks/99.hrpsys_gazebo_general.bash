#!/bin/sh

GAZEBO_SETUP_SH=/usr/share/gazebo/setup.sh
DRCSIM_SETUP_SH=/usr/share/drcsim/setup.sh

if [ ! -e $GAZEBO_SETUP_SH ]; then
    echo -e "\e[31mgazebo should be installed for using hrpsys_gazebo\e[m"
    echo -e "\e[31msee http://gazebosim.org/wiki/DRC/Install\e[m"
    return
fi

source $GAZEBO_SETUP_SH
export DRCSIM_SKIP_ROS_GAZEBO_SETUP='true'
if [ -e $DRCSIM_SETUP_SH ]; then
    source $DRCSIM_SETUP_SH
fi

##
pkgdir=`rospack find hrpsys_gazebo_general`

if [ -e ${pkgdir} ]; then
    export GAZEBO_RESOURCE_PATH=${pkgdir}/worlds:$GAZEBO_RESOURCE_PATH
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${pkgdir}/robot_models:${pkgdir}/..
    export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$LD_LIBRARY_PATH
    export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${pkgdir}/plugins
fi
