#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import *
from nav_msgs.msg import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
import tf

def gazebo_robot_kinematics_mode_main():
    global robot_pose_pub
    rospy.wait_for_service('/gazebo/set_physics_properties')
    set_physics_properties_srv = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
    get_physics_properties_srv = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
    current_physics_properties = get_physics_properties_srv()
    set_physics_properties_srv(time_step=current_physics_properties.time_step,
                               max_update_rate=current_physics_properties.max_update_rate,
                               gravity=current_physics_properties.gravity,
                               ode_config=current_physics_properties.ode_config)
    rospy.Subscriber('/odom', Odometry, odom_cb)
    robot_pose_pub = rospy.Publisher('/ROBOT/SetVelPlugin/PoseCommand', Pose)

def odom_cb(msg):
    robot_pose_pub.publish(msg.pose.pose)

if __name__ == '__main__':
    rospy.init_node('gazebo_robot_kinematics_mode')
    gazebo_robot_kinematics_mode_main()
    rospy.spin()
