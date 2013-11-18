#!/usr/bin/env roseus

(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::load-ros-manifest "moveit_msgs")
(require :rtm-ros-robotics-interface "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")

(load "package://hrpsys_gazebo_atlas/euslisp/atlas-model.l")


(ros::roseus "atlas_eus_ik")

(defun ik-service-cb (req)
  (let ((res (send req :response))
        (arm (if (or (string= (send req :ik_request :group_name) "larm")
                     (string= (send req :ik_request :group_name) "larm_torso")
                     (string= (send req :ik_request :group_name) "lhand")
                     (string= (send req :ik_request :group_name) "lhand_torso"))
                 :larm
               :rarm)))
    (send *atlas* :reset-pose)
    (send *atlas* :newcoords (make-coords)) ;force to move pelvis origin
    (let ((goal (send req :ik_request :pose_stamped)))
      (let ((goal-in-world (send *tf* :transform-pose "/pelvis" goal)))
        (let ((args (if (or (string= (send req :ik_request :group_name) "larm_torso")
                            (string= (send req :ik_request :group_name) "lhand_torso")
                            (string= (send req :ik_request :group_name) "rarm_torso")
                            (string= (send req :ik_request :group_name) "rhand_torso"))
                        (list :link-list (send *atlas* :link-list (send (send *atlas* arm :end-coords) :parent))))))
        ;;(let ((ik-ret (send *atlas* arm :inverse-kinematics goal-in-world :debug-view :no-messages :rotation-axis t))) ;default it uses torso
        ;;(let ((ik-ret (send *atlas* arm :inverse-kinematics goal-in-world :debug-view :no-messages :use-torso t :rotation-axis nil))) ;default it uses torso
        (let ((ik-ret (send* *atlas* arm :inverse-kinematics goal-in-world :debug-view :no-messages :use-torso t
                            :rotation-axis nil args))) ;default it uses torso
          (if ik-ret
              (progn
                (send res :error_code :val moveit_msgs::MoveItErrorCodes::*SUCCESS*)
                (send res :solution :joint_state :name (send-all (send *atlas* :joint-list) :name))
                (send res :solution :joint_state :position
                      (map float-vector #'deg2rad (send *atlas* :angle-vector)))
                )
            (progn
              (send res :error_code :val moveit_msgs::MoveItErrorCodes::*FAILURE*)))
          res))))))


;; initialization
(setq *atlas* (atlas-with-hand))
(objects (list *atlas*))

(setq *tf* (instance ros::transform-listener :init))


(ros::advertise-service "/atlas_eus_ik" moveit_msgs::GetPositionIK #'ik-service-cb)
(while (ros::ok)
  (x::window-main-one)
  (ros::spin-once))
