#!/bin/bash 

rostopic pub atlas/mode std_msgs/String "harnessed" -1
rostopic pub /atlas/atlas_sim_interface_command atlas_msgs/AtlasSimInterfaceCommand "{header: {stamp: now}, behavior: 2, k_effort: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}" -1
rostopic pub /atlas/atlas_sim_interface_command atlas_msgs/AtlasSimInterfaceCommand "{header: {stamp: now}, behavior: 3, k_effort: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}" -1
rostopic pub atlas/mode std_msgs/String "nominal" -1
rostopic pub /atlas/atlas_sim_interface_command atlas_msgs/AtlasSimInterfaceCommand "{header: {stamp: now}, behavior: 0, k_effort: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}" -1
