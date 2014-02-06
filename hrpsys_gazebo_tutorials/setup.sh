#DRCSIM_SETUP_SH=/usr/share/drcsim/setup.sh
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
tpkgdir=`rospack find hrpsys_gazebo_tutorials`
drcdir=`rospack find drcsim_model_resources`
if [ ${ROS_DISTRO} == "groovy" ]; then
    hectdir=`rosstack find hector_models` # for groovy
elif [ ${ROS_DISTRO} == "hydro" ]; then
    hectdir=`rospack find hector_models` # for hydro
fi

if [ -e ${pkgdir} -a -e ${tpkgdir} ]; then
    for dname in `find ${tpkgdir}/robot_models -mindepth 1 -maxdepth 1 -type d -regex '.*[^\.svn]'`; do export ROS_PACKAGE_PATH=${dname}:$ROS_PACKAGE_PATH; done
#    export ROS_PACKAGE_PATH=${pkgdir}/ros:$ROS_PACKAGE_PATH
    export GAZEBO_RESOURCE_PATH=${tpkgdir}/worlds:$GAZEBO_RESOURCE_PATH
    export GAZEBO_MODEL_PATH=${tpkgdir}/robot_models:${tpkgdir}/environment_models:$GAZEBO_MODEL_PATH:${drcdir}/gazebo_models/environments:${hectdir}:${tpkgdir}/..
    export GAZEBO_PLUGIN_PATH=${pkgdir}/plugins:$GAZEBO_PLUGIN_PATH
fi
