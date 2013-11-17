#!/usr/bin/env python

import roslib
import rospy

roslib.load_manifest("hrpsys_gazebo_atlas")

from visualization_msgs.msg import Marker
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import PointStamped
from hrpsys_gazebo_atlas.msg import TrianglePoints
from std_srvs.srv import Empty, EmptyResponse
# global variables

g_offset = [-0.2, 0.0, 0.0]
g_rviz_marker_pub = None
g_image_marker_pub = None
g_triangle_point_state = None

def publishMode():
  global g_image_marker_pub
  marker = ImageMarker2()
  marker.header.stamp = rospy.Time(0.0)
  marker.type = ImageMarker2.TEXT
  marker.text = "mode: triangle_screenpoint.py"
  marker.position.x = 300
  marker.position.y = 50
  g_image_marker_pub.publish(marker)

class TrianglePointsState():
  def __init__(self):
    self.reset()
    self.triangle_publisher = rospy.Publisher("/trianglepoints", TrianglePoints)
    self.point_a_publisher = rospy.Publisher("/triangle_point_a", PointStamped)
    self.point_b_publisher = rospy.Publisher("/triangle_point_b", PointStamped)
    self.point_c_publisher = rospy.Publisher("/triangle_point_c", PointStamped)
  def reset(self):
    self.a = None
    self.b = None
    self.c = None
  def publishTriangles(self):
    msg = TrianglePoints()
    msg.header.stamp = rospy.Time()
    msg.a = self.a
    msg.b = self.b
    msg.c = self.c
    self.triangle_publisher.publish(msg)
  def publishHelperTopics(self):
    if self.a:
      self.point_a_publisher.publish(self.a)
    if self.b:
      self.point_b_publisher.publish(self.b)
    if self.c:
      self.point_c_publisher.publish(self.c)
    publishMode()

def pointCB(msg):
  global g_rviz_marker_pub, g_image_marker_pub, g_triangle_point_state
  if not g_triangle_point_state.a:
    rospy.loginfo("setting A")
    g_triangle_point_state.a = msg
    g_triangle_point_state.a.header.stamp = rospy.Time(0.0)
  elif not g_triangle_point_state.b:
    rospy.loginfo("setting B")
    g_triangle_point_state.b = msg
    g_triangle_point_state.b.header.stamp = rospy.Time(0.0)
  elif not g_triangle_point_state.c:
    rospy.loginfo("setting C")
    g_triangle_point_state.c = msg
    g_triangle_point_state.c.header.stamp = rospy.Time(0.0)
  g_triangle_point_state.publishHelperTopics()

def cancelCB(req):
  global g_triangle_point_state
  g_triangle_point_state.reset()
  return EmptyResponse()
  
def goCB(req):
  global g_triangle_point_state
  g_triangle_point_state.publishTriangles()
  return EmptyResponse()
  
def main():
  global g_rviz_marker_pub, g_image_marker_pub, g_triangle_point_state
  rospy.init_node("triangle_screenpoint")
  g_triangle_point_state = TrianglePointsState()
  g_rviz_marker_pub = rospy.Publisher("/triangle_marker", Marker)
  g_image_marker_pub = rospy.Publisher("/multisense_sl/camera/left/image_marker",
                                       ImageMarker2)
  
  rospy.Subscriber("/pointcloud_screenpoint_nodelet/output_point", PointStamped, 
                   pointCB)
  cancel = rospy.Service("~cancel", Empty, cancelCB)
  go = rospy.Service("~go", Empty, goCB)
  publishMode()
  rospy.spin()

if __name__ == "__main__":
   main()
