#! /usr/bin/env python

import sys
import os
import commands


def make_model_static (name):

    urdf_dir_path = commands.getoutput('rospack find hrpsys_gazebo_tutorials') + '/environment_models/' + name
    urdf_path = urdf_dir_path + '/' + 'model.urdf'
    if len(commands.getoutput("grep \"<static>true</static>\" %s" % urdf_path)) == 0:
        os.system('sed -i -e \"s@</robot>@  <gazebo>\\n    <static>true</static>\\n  </gazebo>\\n</robot>@g\" %s' % urdf_path)


if __name__ == '__main__':
    if len(sys.argv) > 1:
        eus2urdf_for_gazebo_pyscript(sys.argv[1])
