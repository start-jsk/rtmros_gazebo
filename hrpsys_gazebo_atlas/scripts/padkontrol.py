#!/usr/bin/env python
#
# joystick input driver for Korg padKontrol input device
#
# Author: Ryohei Ueda
#

import roslib;
import rospy

import pygame
import pygame.midi

import sys

from sensor_msgs.msg import *

BUTTON_INDICES = [61, 69, 65, 63, 60, 59, 57, 55, 49, 51, 68, 56, 48, 52, 54, 58]


def main():
   pygame.midi.init()
   devices = pygame.midi.get_count()
   if devices < 1:
      print "No MIDI devices detected"
      exit(-1)
   print "Found %d MIDI devices" % devices

   if len(sys.argv) > 1:
      input_dev = int(sys.argv[1])
   else:
      print "no input device supplied. will try to use default device."
      input_dev = pygame.midi.get_default_input_id()
      if input_dev == -1:
         print "No default MIDI input device"
         exit(-1)
   print "Using input device %d" % input_dev

   controller = pygame.midi.Input(input_dev)
   print "Opened it"

   rospy.init_node('kontrol')
   pub = rospy.Publisher('joy', Joy, latch=True)

   m = Joy()
   m.axes = [ 0 ] * 0
   m.buttons = [ 0 ] * 16
   mode = None

   p = False

   while not rospy.is_shutdown():
      m.header.stamp = rospy.Time.now()
      # count the number of events that are coalesced together
      c = 0
      while controller.poll():
         c += 1
         data = controller.read(1)
         #print data
         # loop through events received
         for event in data:
            control = event[0]
            timestamp = event[1]
            # look for continuous controller commands

            # 153 -> pushed
            # 137 -> pulled
            if not (control[0] == 153 or control[0] == 137):
                continue        # not pushed
            else:
                button_index = control[1]
                for bi, i in zip(BUTTON_INDICES, range(len(BUTTON_INDICES))):
                    if button_index == bi:
                        if control[0] == 153:
                            m.buttons[i] = 1
                        else:
                            m.buttons[i] = 0
                        p = True
      if p:
         pub.publish(m)
         p = False

      rospy.sleep(0.01) # 100hz max
                  


if __name__ == '__main__':
   try:
      main()
   except rospy.ROSInterruptException: pass
