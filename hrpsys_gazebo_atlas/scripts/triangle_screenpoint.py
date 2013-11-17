#!/usr/bin/env python

import roslib
import rospy

roslib.load_manifest("hrpsys_gazebo_atlas")

from visualization_msgs.msg import Marker
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import PointStamped
from hrpsys_gazebo_atlas.msg import TrianglePoints

# global variables

g_offset = [-0.2, 0.0, 0.0]
g_rviz_marker_pub = None
g_image_marker_pub = None
g_triangle_point_state = None

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
  

def pointCB(msg):
  print msg
  global g_rviz_marker_pub, g_image_marker_pub, g_triangle_point_state
  if not g_triangle_point_state.a:
    g_triangle_point_state.a = msg
  elif not g_triangle_point_state.b:
    g_triangle_point_state.b = msg
  elif not g_triangle_point_state.c:
    g_triangle_point_state.c = msg
    # DONE!!
    g_triangle_point_state.publishTriangles()
  g_triangle_point_state.publishHelperTopics()
  
def main():
  global g_rviz_marker_pub, g_image_marker_pub, g_triangle_point_state
  rospy.init_node("triangle_screenpoint")
  g_triangle_point_state = TrianglePointsState()
  g_rviz_marker_pub = rospy.Publisher("/triangle_marker", Marker)
  g_image_marker_pub = rospy.Publisher("/multisense_sl/camera/left/image_marker",
                                       ImageMarker2)
  rospy.Subscriber("/pointcloud_screenpoint_nodelet/output_point", PointStamped, 
                   pointCB)
  rospy.spin()

if __name__ == "__main__":
   main()
