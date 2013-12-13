#!/bin/bash 

rostopic pub /atlas/atlas_sim_interface_command atlas_msgs/AtlasSimInterfaceCommand "{header: {stamp: now}, behavior: 0, k_effort: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}" -1
