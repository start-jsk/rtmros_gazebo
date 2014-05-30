#! /usr/bin/env python
# prismatic joint is not simulated in gazebo 1.5

import os
import commands


def change_prismatic_joint_fixed ():
    print "Change prismatic joint to fixed joint included in static model"
    package_dir_path = commands.getoutput('rospack find hrpsys_gazebo_tutorials')
    model_dir_path = package_dir_path + '/environment_models/'

    os.system("ls " + model_dir_path + '*static/model.urdf | xargs sed -i -e "s/prismatic/fixed/g"')
    print "Done"

if __name__ == '__main__':
    change_prismatic_joint_fixed()


