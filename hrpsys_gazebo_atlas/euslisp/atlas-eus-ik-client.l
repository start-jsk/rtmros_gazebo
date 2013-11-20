#!/usr/bin/env roseus

(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::load-ros-manifest "arm_navigation_msgs")
(ros::load-ros-manifest "moveit_msgs")

(ros::roseus "atlas_eus_ik_client")
(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::load-ros-manifest "moveit_msgs")

(do-until-key
 (let ((req (instance moveit_msgs::GetPositionIKRequest :init))
       (D 0.2))
   (if (< (random 2.0) 1)
       (if (< (random 2.0) 1)
	   (send req :ik_request :group_name "larm")
	 (send req :ik_request :group_name "larm_torso"))
     (if (< (random 2.0) 1)
	 (send req :ik_request :group_name "rarm")
       (send req :ik_request :group_name "rarm_torso")))
   (send req :ik_request :group_name "rarm_torso")
   (send req :ik_request :group_name "larm_torso")
   (let ((c (make-coords)))
     (if (or (string= (send req :ik_request :group_name) "larm")
	     (string= (send req :ik_request :group_name) "larm_torso"))
	 (send c :rotate (deg2rad 90) :x)
       (progn
	 (send c :rotate (deg2rad 180) :z)
	 (send c :rotate (deg2rad 90) :x)))
     ;;(send c :rotate (deg2rad 90) :x))
     (let ((q (matrix2quaternion (send c :rot))))
       (send req :ik_request :pose_stamped (instance geometry_msgs::PoseStamped :init
						     :header (instance std_msgs::Header :init
								       :stamp (ros::time 0.0)
								       :frame_id "/pelvis")
						     :pose (instance geometry_msgs::Pose :init
								     :position (instance geometry_msgs::Point :init
											 :x (+ 0.8 (- (random D) (/ D 2.0)))
											 :y (- (random (* D 2)) (/ (* 2 D) 2.0))
											 :z (+ 0.3 (- (random D) (/ D 2.0))))
								     :orientation
								     (instance geometry_msgs::Quaternion :init
									       :x (elt q 1)
									       :y (elt q 2)
									       :z (elt q 3)
									       :w (elt q 0))
								     )))
       (let ((res (ros::service-call "/atlas_eus_ik" req)))
	 (if (= (send res :error_code :val) moveit_msgs::MoveItErrorCodes::*SUCCESS*)
	     (print (send res :solution :joint_state :position))
	   (print "failed!")
	   )))
     )))
;(exit)

