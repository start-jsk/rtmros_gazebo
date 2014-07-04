cmake_minimum_required(VERSION 2.8.3)
project(eusgazebo)

find_package(catkin REQUIRED COMPONENTS message_generation)

catkin_package(CATKIN_DEPENDS message_runtime)

# find_package(catkin REQUIRED COMPONENTS message_generation gazebo_msgs rostest)

# catkin_package(CATKIN_DEPENDS message_runtime gazebo_msgs)
