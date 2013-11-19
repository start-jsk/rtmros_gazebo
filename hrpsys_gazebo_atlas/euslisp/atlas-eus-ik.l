#!/usr/bin/env roseus

(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::load-ros-manifest "moveit_msgs")
(require :rtm-ros-robotics-interface "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")

(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")


(ros::roseus "atlas_eus_ik")

(defun setup-end-coords ()
  ;; right
  (let ((end-coords (make-coords)) 
        (arm :rarm)
        ;;(palm-link :right_palm_lk)
        (palm-link :right_palm_left_camera_optical_frame_lk)
        (axis (float-vector 0 0 1))
        (rad 3.8))
        ;;(rad 1.57))
    ;;(rad 0))
    (send end-coords :rotate rad axis)
    (send end-coords :translate (float-vector 12.5 -225 4.5) :world)
    (let ((new-end-coords
           (send (send *atlas* :r_hand_lk) :copy-worldcoords)))
           ;;(send (send (send *atlas* :hand arm) palm-link) :copy-worldcoords)))
      (send new-end-coords :transform end-coords :local)
      (send new-end-coords :draw-on :flush t)
      (let ((trans (send (send *atlas* arm :end-coords :copy-worldcoords)
                         :transformation new-end-coords)))
        (send (send *atlas* arm :end-coords) :transform trans))
      ))
  ;; left
  (let ((end-coords (make-coords)) 
        (arm :larm)
        ;;(palm-link :right_palm_lk)
        (palm-link :left_palm_left_camera_optical_frame_lk)
        (axis (float-vector 0 0 1))
        (rad -1.57))
        ;;(rad 0))
    (send end-coords :rotate rad axis)
    (send end-coords :translate (float-vector 12.5 225 4.5) :world)
    (let ((new-end-coords
           (send (send (send *atlas* :hand arm) palm-link) :copy-worldcoords)))
      (send new-end-coords :transform end-coords :local)
      (let ((trans (send (send *atlas* arm :end-coords) :transformation
                   new-end-coords)))
        (send (send *atlas* arm :end-coords) :transform trans))
      ))
  )

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
                            :rotation-axis t args))) ;default it uses torso
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
(setup-end-coords)
(objects (list *atlas*))

(setq *tf* (instance ros::transform-listener :init))


(ros::advertise-service "/atlas_eus_ik" moveit_msgs::GetPositionIK #'ik-service-cb)
(while (ros::ok)
  (x::window-main-one)
  (ros::spin-once))


#|
;; client sample
(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::load-ros-manifest "moveit_msgs")

(ros::roseus "foo")
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
(exit)

|#

