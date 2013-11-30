#!/usr/bin/env python

import math
import os
import subprocess

import roslib
import rospy

roslib.load_manifest("hrpsys_gazebo_atlas")

from visualization_msgs.msg import Marker
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import ColorRGBA
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
  marker.id = 1
  marker.type = ImageMarker2.TEXT
  marker.text = "mode: triangle_screenpoint.py"
  marker.position.x = 300
  marker.position.y = 50
  g_image_marker_pub.publish(marker)

class TrianglePointsState():
  def __init__(self):
    self.reset()
    self.triangle_publisher = rospy.Publisher("/trianglepoints", 
                                              TrianglePoints)
    self.point_a_publisher = rospy.Publisher("/triangle_point_a",
                                             PointStamped)
    self.point_b_publisher = rospy.Publisher("/triangle_point_b", 
                                             PointStamped)
    self.point_c_publisher = rospy.Publisher("/triangle_point_c", 
                                             PointStamped)
  def reset(self):
    self.a = None
    self.b = None
    self.c = None
  def makeImageMarker(self, p):
    now = rospy.Time(0.0)
    marker = ImageMarker2()
    marker.header.stamp = now
    marker.header.frame_id = p.header.frame_id
    marker.type = ImageMarker2.POLYGON3D
    marker.points3D.header.frame_id = p.header.frame_id
    marker.points3D.header.stamp = now
    R = 0.01
    N = 10
    for i in range(N) + [0]:
      point = Point()
      point.x = p.point.x + R * math.cos(2 * math.pi * i / N)
      point.y = p.point.y + R * math.sin(2 * math.pi * i / N)
      point.z = p.point.z
      marker.points3D.points.append(point)
    marker.outline_colors = [ColorRGBA(1.0, .0, .0, 1.0)]
    return marker
  def publishTriangles(self):
    msg = TrianglePoints()
    msg.header.stamp = rospy.Time()
    msg.a = self.a
    msg.b = self.b
    msg.c = self.c
    self.triangle_publisher.publish(msg)
  def publishHelperTopics(self):
    global g_image_marker_pub
    if self.a:
      self.point_a_publisher.publish(self.a)
      m = self.makeImageMarker(self.a)
      m.id = 2
      g_image_marker_pub.publish(m)
    if self.b:
      self.point_b_publisher.publish(self.b)
      m = self.makeImageMarker(self.b)
      m.id = 3
      m.outline_colors[0].r = 0.0
      m.outline_colors[0].g = 1.0
      g_image_marker_pub.publish(m)
    if self.c:
      self.point_c_publisher.publish(self.c)
      m = self.makeImageMarker(self.c)
      m.outline_colors[0].r = 0.0
      m.outline_colors[0].b = 1.0
      m.id = 4
      g_image_marker_pub.publish(m)
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
  rospy.loginfo("canceled")
  g_triangle_point_state.reset()
  return EmptyResponse()
  
def goCB(req):
  global g_triangle_point_state
  rospy.loginfo("go")
  g_triangle_point_state.publishTriangles()
  return EmptyResponse()
  
def main():
  global g_rviz_marker_pub, g_image_marker_pub, g_triangle_point_state
  rospy.init_node("triangle_screenpoint")
  g_triangle_point_state = TrianglePointsState()
  g_rviz_marker_pub = rospy.Publisher("/triangle_marker", Marker)
  g_image_marker_pub = rospy.Publisher("/multisense_sl/camera/left/image_marker",
                                       ImageMarker2)
  gui_process = subprocess.Popen([os.path.join(os.path.dirname(__file__), 
                                               "triangle_gui.py")])
  rospy.Subscriber("/pointcloud_screenpoint_nodelet/output_point", PointStamped, 
                   pointCB)
  cancel = rospy.Service("~cancel", Empty, cancelCB)
  go = rospy.Service("~go", Empty, goCB)
  publishMode()
  rospy.spin()

if __name__ == "__main__":
   main()
