#!/bin/bash 

rostopic pub /atlas/atlas_sim_interface_command atlas_msgs/AtlasSimInterfaceCommand "{header: {stamp: now}, behavior: 1, k_effort: [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]}" -1
