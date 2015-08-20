# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_gazebo_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs)

add_message_files(
  DIRECTORY msg
  FILES JointCommand.msg  NamedImu.msg NamedWrench.msg RobotState.msg
)
add_service_files(
  DIRECTORY srv
  FILES SyncCommand.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs)






