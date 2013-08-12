#! /usr/bin/env python

import sys
import os
import commands


def generate_room_world (name):

    # make models and world file
    eus_script_path = commands.getoutput('rospack find hrpsys_gazebo_tutorials') + '/euslisp/hrpsys-gazebo-utils.l'
    os.system('rosrun roseus roseus %s "(progn (generate-room-models \\"%s\\") (exit))"' % (eus_script_path, name))


if __name__ == '__main__':
    if len(sys.argv) > 1:
        generate_room_world(sys.argv[1])
    else:
        print "usage: %s room-name" % sys.argv[0]
