#! /usr/bin/env python

import os
import commands


def remove_all_generated_files ():

    package_dir_path = commands.getoutput('rospack find hrpsys_gazebo_tutorials')
    world_dir_path = package_dir_path + '/worlds/'
    launch_dir_path = package_dir_path + '/launch/'

    os.system("rm -rf $(svn st %s | grep ? | cut -f2- -d\" \")" % world_dir_path)
    os.system("rm -rf $(svn st %s | grep ? | cut -f2- -d\" \")" % launch_dir_path)

    os.system("rosrun hrpsys_gazebo_tutorials remove_all_added_models.py")

if __name__ == '__main__':
    remove_all_generated_files()


