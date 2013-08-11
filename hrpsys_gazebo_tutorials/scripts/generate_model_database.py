#! /usr/bin/env python

import commands

def generate_model_database ():

    manifest_dir_path = commands.getoutput('rospack find hrpsys_gazebo_tutorials') + '/environment_models/'
    manifest_path = manifest_dir_path + 'manifest.xml'
    f = open(manifest_path, 'w')

    f.write("<!-- automatically generated -->\n<?xml version=\"1.0\" ?>\n<database>\n  <name>hrpsys_gazebo</name>\n  <license>Creative Commons Attribution 3.0 Unported</license>\n\n  <models>\n")

    model_name_list = commands.getoutput("ls -F -1 %s | grep / | sed \"s@/@@g\"" % manifest_dir_path).splitlines()
    for model_name in model_name_list:
        f.write("    <uri>file://%s</uri>\n" % model_name)

    f.write("  </models>\n</database>\n")

    f.close()



if __name__ == '__main__':
    generate_model_database()
