cmake_minimum_required(VERSION 2.8.3)
project(eusgazebo)

find_package(catkin REQUIRED COMPONENTS message_generation gazebo_msgs rostest roseus)

catkin_package(CATKIN_DEPENDS message_runtime gazebo_msgs roseus)

## Install ##
install(DIRECTORY euslisp test scripts samples
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

## Testing ##
add_rostest(test/test-fall-arrow-object-simulation.test)
