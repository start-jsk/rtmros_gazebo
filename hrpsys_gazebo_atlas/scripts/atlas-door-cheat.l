#!/usr/bin/env roseus

(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")

(defmethod atlas-interface
  (:init ()
	 (ros::advertise "atlas/control_mode" std_msgs::String 1)
	 (ros::advertise "atlas/atlas_sim_interface_command"
			 atlas_msgs::AtlasSimInterfaceCommand 1)
	 (ros::subscribe "atlas/atlas_sim_interface_state"
			 atlas_msgs::AtlasSimInterfaceState #'send self :sim-interface-state-cb)
	 (ros::advertise "atlas/mode" std_msgs::String 1)
	 (ros::advertise "atlas/set_pose" geometry_msgs::Pose 1)
	 ))

(setq *ri* (instance atlas-interface :init))

(send *ri* :teleport-to (make-coords :pos #f(-500 -700 1000) :rpy (list (/ (* pi 2) 3) 0 0)) nil nil t)

(exit)
