#!/usr/bin/env python
import roslib; roslib.load_manifest('hrpsys_gazebo_atlas')
from hrpsys_gazebo_atlas.atlas_client import *

if __name__ == '__main__':
    shcf=ATLASHrpsysConfigurator()
    shcf.init(sys.argv[1], sys.argv[2])

    shcf.resetJointGroup()
