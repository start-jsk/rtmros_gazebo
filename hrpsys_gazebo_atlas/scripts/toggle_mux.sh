#!/bin/bash

# /lhand_pointcloud_mux or /rhand_pointcloud_mux
# /pointcloud_roi/lhand_pointcloud_mux
hand=$1
echo $hand
target_mux=$1_pointcloud_mux
echo $target_mux
selected_topic=`rostopic echo /pointcloud_roi/${target_mux}/selected -n 1 | grep data | awk '{print $2}'`

if [ `echo $selected_topic | grep -c dummy` == 1 ]; then
    next_topic=/sandia_hands/${hand}/points2
else
    next_topic=/sandia_hands/${hand}/points2_dummy
fi

echo $selected_topic '->' $next_topic

rosrun topic_tools mux_select /pointcloud_roi/${hand}_pointcloud_mux $next_topic
