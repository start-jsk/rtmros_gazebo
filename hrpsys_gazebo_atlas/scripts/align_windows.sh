#!/bin/bash

function getWID(){
    name=$1
    hint="$2"
    THE_PID=`ps aux | grep $name | grep "$hint" | grep -v grep | awk '{print $2}'`
    THE_WINDOW_ID=`wmctrl -lp | grep $THE_PID | awk '{print $1}'`
    echo $THE_WINDOW_ID
}

RVIZ_WID=`getWID rviz atlas_joint_marker.rviz`
wmctrl -ir $RVIZ_WID -e 1,1920,0,1920,1080

IMAGE_VIEW2_HEAD_RESIZED_WID=`getWID image_view2 head_resized`
wmctrl -ir $IMAGE_VIEW2_HEAD_RESIZED_WID -e 1,0,0,320,240

IMAGE_VIEW2_LHAND_RESIZED_WID=`getWID image_view2 lhand_resized`
echo lhand $IMAGE_VIEW2_LHAND_RESIZED_WID
wmctrl -ir $IMAGE_VIEW2_LHAND_RESIZED_WID -e 1,320,0,320,240

IMAGE_VIEW2_RHAND_RESIZED_WID=`getWID image_view2 rhand_resized`
wmctrl -ir $IMAGE_VIEW2_RHAND_RESIZED_WID -e 1,640,0,320,240

IMAGE_VIEW2_HEAD_SNAP_WID=`getWID image_view2 head_snap`
wmctrl -ir $IMAGE_VIEW2_HEAD_SNAP_WID -e 1,0,290,320,240

IMAGE_VIEW2_LHAND_SNAP_WID=`getWID image_view2 lhand_snap`
wmctrl -ir $IMAGE_VIEW2_LHAND_SNAP_WID -e 1,320,290,320,240

IMAGE_VIEW2_RHAND_SNAP_WID=`getWID image_view2 rhand_snap`
wmctrl -ir $IMAGE_VIEW2_RHAND_SNAP_WID -e 1,640,290,320,240

IMAGE_VIEW2_LFISH_SNAP_WID=`getWID image_view2 lfisheye_snap`
wmctrl -ir $IMAGE_VIEW2_LFISH_SNAP_WID -e 1,960,290,320,240

IMAGE_VIEW2_RFISH_SNAP_WID=`getWID image_view2 rfisheye_snap`
wmctrl -ir $IMAGE_VIEW2_RFISH_SNAP_WID -e 1,1280,290,320,240

IMAGE_GUI_WID=`getWID image_gui hrpsys`
wmctrl -ir $IMAGE_GUI_WID -e 1,0,560,320,400

IMAGE_ROI_WID=`getWID lib/image_view roi`
wmctrl -ir $IMAGE_ROI_WID -e 1,320,560,320,240

wmctrl -r 'I1:bashI' -e 1,960,0,320,240

OCS_LAUNCH_WID=`getWID xterm ocs.launch`
wmctrl -ir $OCS_LAUNCH_WID -e 1,1280,0,320,240

FOOTSTEP_WID=`getWID xterm ps3_footstep`
wmctrl -ir $FOOTSTEP_WID -e 1,1600,0,320,520


# EUS_IK_WID=`wmctrl -lp | grep "IK server" | awk '{pring $1}'`
wmctrl -r 'IK server' -e 1,640,560,400,400
wmctrl -r 'Interactive IK' -e 1,1040,560,400,400
ROSPING_WID=`getWID rosping rosping_gui`
wmctrl -ir $ROSPING_WID -e 1,1440,560,500,520
