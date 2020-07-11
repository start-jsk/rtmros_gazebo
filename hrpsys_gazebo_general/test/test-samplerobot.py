#!/usr/bin/env python

PKG = 'hrpsys_gazebo_general'
NAME = 'test_samplerobot'

import argparse,unittest,rostest, time, sys, math, os
from copy import deepcopy
from numpy import *

import rospy,rospkg, tf
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from hrpsys_ros_bridge.srv import *

import actionlib

from trajectory_msgs.msg import JointTrajectoryPoint

class TestSampleRobot(unittest.TestCase):
    listener = None
    lfsensor = None
    rfsensor = None
    odom = None
    joint_states = None
    model_states = None
    feedback = None

    def lfsensor_cb(self, sensor):
        self.lfsensor = sensor

    def rfsensor_cb(self, sensor):
        self.rfsensor = sensor
    def odom_cb(self, odom):
        self.odom = odom
    def jointstate_cb(self, joint_states):
        self.joint_states = joint_states
    def modelstate_cb(self, model_states):
        self.model_states = model_states

    def setUp(self):
        rospy.init_node('TestSampleRobot')
        rospy.Subscriber('/lfsensor', WrenchStamped, self.lfsensor_cb)
        rospy.Subscriber('/rfsensor', WrenchStamped, self.rfsensor_cb)
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/joint_states', JointState, self.jointstate_cb)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.modelstate_cb)
        self.listener = tf.TransformListener()

    def test_odom(self): # need to check if map/ is published?
        # wait odom topic
        start_time = rospy.Time.now()
        r = rospy.Rate(1)
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < 120:
            if self.odom:
                break
            rospy.loginfo("waiting for /odom")
            r.sleep()
        if not self.odom:
            self.assertTrue(False, "no odom topic is available")
        else:
            self.assertTrue(True,"ok")

    def test_odom_tf(self): # echeck if tf/odom is published
        # wait odom topic
        try:
            rospy.loginfo("waiting for /WAIST_LINK0 to /odom")
            self.listener.waitForTransform('/WAIST_LINK0', '/odom', rospy.Time(), rospy.Duration(120))
        except tf.Exception:
            self.assertTrue(None, "could not found tf from /WAIST_LINK0 to /odom")
        (trans1,rot1) = self.listener.lookupTransform('/WAIST_LINK0', '/odom', rospy.Time(0))

        self.assertTrue(True,"ok")

    def test_force_sensor(self):
        while self.lfsensor == None or self.rfsensor == None:
            time.sleep(1)
            rospy.logwarn("wait for sensor...")
        rospy.logwarn("sensor = %r %r"%(self.lfsensor, self.rfsensor))
        self.assertAlmostEqual(self.lfsensor.wrench.force.z+self.rfsensor.wrench.force.z, 1300, delta=200)

    def impl_test_joint_angles(self, fullbody, goal):
        fullbody.wait_for_server()

        goal.trajectory.header.stamp = rospy.get_rostime()
        goal.trajectory.joint_names.append("RLEG_HIP_R")
        goal.trajectory.joint_names.append("RLEG_HIP_P")
        goal.trajectory.joint_names.append("RLEG_HIP_Y")
        goal.trajectory.joint_names.append("RLEG_KNEE")
        goal.trajectory.joint_names.append("RLEG_ANKLE_P")
        goal.trajectory.joint_names.append("RLEG_ANKLE_R")
        goal.trajectory.joint_names.append("RARM_SHOULDER_P")
        goal.trajectory.joint_names.append("RARM_SHOULDER_R")
        goal.trajectory.joint_names.append("RARM_SHOULDER_Y")
        goal.trajectory.joint_names.append("RARM_ELBOW")
        goal.trajectory.joint_names.append("RARM_WRIST_Y")
        goal.trajectory.joint_names.append("RARM_WRIST_P")
        goal.trajectory.joint_names.append("RARM_WRIST_R")
        goal.trajectory.joint_names.append("LLEG_HIP_R")
        goal.trajectory.joint_names.append("LLEG_HIP_P")
        goal.trajectory.joint_names.append("LLEG_HIP_Y")
        goal.trajectory.joint_names.append("LLEG_KNEE")
        goal.trajectory.joint_names.append("LLEG_ANKLE_P")
        goal.trajectory.joint_names.append("LLEG_ANKLE_R")
        goal.trajectory.joint_names.append("LARM_SHOULDER_P")
        goal.trajectory.joint_names.append("LARM_SHOULDER_R")
        goal.trajectory.joint_names.append("LARM_SHOULDER_Y")
        goal.trajectory.joint_names.append("LARM_ELBOW")
        goal.trajectory.joint_names.append("LARM_WRIST_Y")
        goal.trajectory.joint_names.append("LARM_WRIST_P")
        goal.trajectory.joint_names.append("LARM_WRIST_R")
        goal.trajectory.joint_names.append("WAIST_P")
        goal.trajectory.joint_names.append("WAIST_R")
        goal.trajectory.joint_names.append("CHEST")

        point = JointTrajectoryPoint()
        target_positions = [-0.004457,-21.6929,-0.01202,47.6723,-25.93,0.014025,17.8356,-9.13759,-6.61188,-36.456,0.0,0.0,0.0,-0.004457,-21.6929,-0.01202,47.6723,-25.93,0.014025,17.8356,9.13759,6.61188,-36.456,0.0,0.0,0.0,0.0,0.0,0.0] # reset-pose
        point.positions = [ x * math.pi / 180.0 for x in target_positions ]
        point.time_from_start = rospy.Duration(5.0)
        goal.trajectory.points.append(point)
        fullbody.send_goal(goal)
        fullbody.wait_for_result()

        ## wait for update joint_states
        while self.joint_states == None:
            time.sleep(1)
            rospy.logwarn("wait for joint_states..")
        rospy.sleep(1)
        current_positions = dict(zip(self.joint_states.name, self.joint_states.position))
        goal_angles = [ 180.0 / math.pi * current_positions[x] for x in goal.trajectory.joint_names]
        rospy.logwarn(goal_angles)
        rospy.logwarn("difference between two angles %r %r"%(array(target_positions)-array(goal_angles),linalg.norm(array(target_positions)-array(goal_angles))))
        self.assertAlmostEqual(linalg.norm(array(target_positions)-array(goal_angles)), 0, delta=0.1 * 180.0 / math.pi)

    # send joint angles
    def test_joint_angles(self):
        # for < kinetic
        if os.environ['ROS_DISTRO'] >= 'kinetic' :
            return True
        from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
        fullbody = actionlib.SimpleActionClient("/fullbody_controller/joint_trajectory_action", JointTrajectoryAction)
        self.impl_test_joint_angles(fullbody, JointTrajectoryGoal())

    def test_follow_joint_angles(self):
        # for >= kinetic
        if os.environ['ROS_DISTRO'] < 'kinetic' :
            return True
        from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
        fullbody = actionlib.SimpleActionClient("/fullbody_controller/follow_joint_trajectory_action", FollowJointTrajectoryAction)
        self.impl_test_joint_angles(fullbody, FollowJointTrajectoryGoal())

    # send walk motion
    # test_walk_motion should be executed after test_joint_angles and test_follow_joint_angles. The order is determined by sorting the test method names (https://docs.python.org/3/library/unittest.html).
    def test_walk_motion(self):
        while self.model_states == None:
            time.sleep(1)
            rospy.logwarn("wait for gazebo/model_states..")

        rospy.wait_for_service('/AutoBalancerServiceROSBridge/startAutoBalancer')
        startAutoBalancer = rospy.ServiceProxy('/AutoBalancerServiceROSBridge/startAutoBalancer', OpenHRP_AutoBalancerService_startAutoBalancer)
        startAutoBalancer(["rleg","lleg","rarm","larm"])
        rospy.sleep(3.0)
        current_poses = dict(zip(self.model_states.name, self.model_states.pose))
        self.assertAlmostEqual(current_poses["SampleRobot"].position.z, 0.668, delta=0.1)
        self.assertAlmostEqual(linalg.norm(array([current_poses["SampleRobot"].orientation.x,current_poses["SampleRobot"].orientation.y,current_poses["SampleRobot"].orientation.z,current_poses["SampleRobot"].orientation.w])-array([0.0,0.0,0.0,1.0])), 0, delta=0.1)

        rospy.wait_for_service('/StabilizerServiceROSBridge/startStabilizer')
        startStabilizer = rospy.ServiceProxy('/StabilizerServiceROSBridge/startStabilizer', OpenHRP_StabilizerService_startStabilizer)
        startStabilizer()
        rospy.sleep(3.0)
        current_poses = dict(zip(self.model_states.name, self.model_states.pose))
        self.assertAlmostEqual(current_poses["SampleRobot"].position.z, 0.668, delta=0.1)
        self.assertAlmostEqual(linalg.norm(array([current_poses["SampleRobot"].orientation.x,current_poses["SampleRobot"].orientation.y,current_poses["SampleRobot"].orientation.z,current_poses["SampleRobot"].orientation.w])-array([0.0,0.0,0.0,1.0])), 0, delta=0.1)

        rospy.wait_for_service('/AutoBalancerServiceROSBridge/goPos')
        goPos = rospy.ServiceProxy('/AutoBalancerServiceROSBridge/goPos', OpenHRP_AutoBalancerService_goPos)
        goPos(1.0,0,0)
        rospy.wait_for_service('/AutoBalancerServiceROSBridge/waitFootSteps')
        waitFootSteps = rospy.ServiceProxy('/AutoBalancerServiceROSBridge/waitFootSteps', OpenHRP_AutoBalancerService_waitFootSteps)
        waitFootSteps()
        rospy.sleep(1.0)
        current_poses = dict(zip(self.model_states.name, self.model_states.pose))
        self.assertAlmostEqual(current_poses["SampleRobot"].position.z, 0.668, delta=0.1)
        self.assertAlmostEqual(linalg.norm(array([current_poses["SampleRobot"].orientation.x,current_poses["SampleRobot"].orientation.y,current_poses["SampleRobot"].orientation.z,current_poses["SampleRobot"].orientation.w])-array([0.0,0.0,0.0,1.0])), 0, delta=0.1)
        self.assertAlmostEqual(current_poses["SampleRobot"].position.x, 1.0, delta=0.1)
        self.assertAlmostEqual(current_poses["SampleRobot"].position.y, 0.0, delta=0.1)

#unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestSampleRobot, sys.argv)
