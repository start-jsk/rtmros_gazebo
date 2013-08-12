#! /usr/bin/env python

import os
import commands


def remove_all_added_models ():

    manifest_dir_path = commands.getoutput('rospack find hrpsys_gazebo_tutorials') + '/environment_models/'
    manifest_path = manifest_dir_path + 'manifest.xml'

    os.system("rm -rf $(svn st %s | grep ? | cut -f2- -d\" \")" % manifest_dir_path)
    os.system("svn revert %s" % manifest_path)

if __name__ == '__main__':
    remove_all_added_models()


