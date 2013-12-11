#!/usr/bin/env python

import roslib
import rospy


roslib.load_manifest("std_msgs")

from std_msgs.msg import Empty
from sensor_msgs.msg import Joy



def joyCB(msg):
    if msg.buttons[0] == 1:
        headPub.publish(Empty())
    elif msg.buttons[1] == 1:
        lhandPub.publish(Empty())
    elif msg.buttons[2] == 1:
        rhandPub.publish(Empty())
    elif msg.buttons[3] == 1:
        lFishPub.publish(Empty())
    elif msg.buttons[4] == 1:
        rFishPub.publish(Empty())


rospy.init_node("sensor_padkontrol")

rospy.Subscriber("/joy_pad", Joy, joyCB)
headPub = rospy.Publisher("/head_snap/snapshot", Empty)
lhandPub = rospy.Publisher("/lhand_snap/snapshot", Empty)
rhandPub = rospy.Publisher("/rhand_snap/snapshot", Empty)
lFishPub = rospy.Publisher("/lfisheys_snap/snapshot", Empty)
rFishPub = rospy.Publisher("/rfisheys_snap/snapshot", Empty)

rospy.spin()

