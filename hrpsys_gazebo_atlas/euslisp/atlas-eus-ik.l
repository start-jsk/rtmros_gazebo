#!/usr/bin/env roseus

(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::load-ros-manifest "sandia_hand_msgs")
(ros::load-ros-manifest "humanoid_nav_msgs")
(ros::load-ros-manifest "atlas_msgs")
(ros::load-ros-manifest "arm_navigation_msgs")
(ros::load-ros-manifest "moveit_msgs")

(require :rtm-ros-robotics-interface "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")

(require "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
(require "package://hrpsys_gazebo_atlas/euslisp/atlas-fullbody-ik.l")

(ros::roseus "atlas_eus_ik")

(defun setup-end-coords ()
  (unless (boundp '*setup-end-coords-for-hku-flag*)
    ;; right
    (let ((end-coords (make-coords)) 
	  (arm :rarm)
	  ;;(palm-link :right_palm_lk)
	  (palm-link :right_palm_left_camera_optical_frame_lk)
	  (axis (float-vector 0 0 1))
	  (rad 2.23))
      ;;(rad 1.57))
      ;;(rad 0))
      (send end-coords :rotate rad axis)
      (send end-coords :translate (float-vector 30 -250 45) :world)
      (let ((new-end-coords
	     (send (send *atlas* :r_hand_lk) :copy-worldcoords)))
	;;(send (send (send *atlas* :hand arm) palm-link) :copy-worldcoords)))
	(send new-end-coords :transform end-coords :local)
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
	  (rad 0.91))
      (send end-coords :rotate rad axis)
      (send end-coords :translate (float-vector 30 250 45) :world)
      (let ((new-end-coords
	     (send (send *atlas* :l_hand_lk) :copy-worldcoords)))
	;;(send (send (send *atlas* :hand arm) palm-link) :copy-worldcoords)))
	(send new-end-coords :transform end-coords :local)
	(let ((trans (send (send *atlas* arm :end-coords) :transformation
			   new-end-coords)))
	  (send (send *atlas* arm :end-coords) :transform trans))
	))
    )
  (setq *setup-end-coords-for-hku-flag* t)
  )

(defun fullbody-ik-with-collision (arm goal-in-world)
  (fullbody-ik-main
   :target-limb (list arm :lleg :rleg)
   :target-coords (list goal-in-world nil nil)
   :collision? t
   ;:centroid-thre 100
   :thre '(1 1 1)
   :stop 100
   :rotation-axis '(t t t)))

(defun torso-ik-with-collision (arm goal-in-world)
  (fullbody-ik-main
   :target-limb (list arm :lleg :rleg)
   :target-coords (list goal-in-world nil nil)
   :additional-weight-list
   (mapcar
    #'(lambda (k gain)
	(list (send *atlas* :torso k :child-link)
	      gain))
    (list :waist-r :waist-p :waist-y)
    (list 0.1 0.1 0.1))
   ;;(list 1e-3 1e-3 1e-3))
   :root-link-virtual-joint-weight
   (float-vector 0 0 0 0 0 0)
   :min-vec (float-vector 0 0 0 0 0 0)
   :max-vec (float-vector 0 0 0 0 0 0)
   :collision? t
   ;:centroid-thre 200
   :thre '(1 1 1)
   :stop 100
   :rotation-axis '(t t t)))

(defun normal-ik-with-collision (arm goal-in-world)
  (let* ((w
	  (send-all (send *atlas* :joint-list)
		    :max-joint-velocity))
	 (vel-limit
	  (send-all (send *robot* :joint-list)
		    :max-joint-velocity 0.2))
	 (ret
	  (fullbody-ik-main
	   :target-limb (list arm :lleg :rleg)
	   :target-coords (list goal-in-world nil nil)
	   :additional-weight-list
	   (mapcar
	    #'(lambda (k gain)
		(list (send *atlas* :torso k :child-link)
		      gain))
	    (list :waist-r :waist-p :waist-y)
	    (list 0.0 0.0 0.0))
	   ;;(list 1e-3 1e-3 1e-3))
	   :root-link-virtual-joint-weight
	   (float-vector 0 0 0 0 0 0)
	   :min-vec (float-vector 0 0 0 0 0 0)
	   :max-vec (float-vector 0 0 0 0 0 0)
	   :collision? t
	   :centroid-thre nil
	   :stop 150
	   :thre '(1 1 1)
	   :rthre (list (deg2rad 1) (deg2rad 1) (deg2rad 1))
	   :rotation-axis '(t t t))))
    (mapcar
     #'(lambda (j g) (send j :max-joint-velocity g))
     (send *atlas* :joint-list) w)
    ret))

(defun ik-service-cb (req)
  (let ((res (send req :response))
        (arm (if (or (string= (subseq (send req :ik_request :group_name) 0 4) "larm")
                     (string= (subseq (send req :ik_request :group_name) 0 4) "lhand"))
                 :larm
               :rarm))
        (torsop (or (string= (send req :ik_request :group_name) "larm_torso")
                    (string= (send req :ik_request :group_name) "rarm_torso")))
        (fullbodyp (or (string= (send req :ik_request :group_name) "larm_fullbody")
                       (string= (send req :ik_request :group_name) "rarm_fullbody"))))
    ;; taking joint state into account
    (if (and (send req :ik_request :robot_state)
               (send req :ik_request :robot_state :joint_state)
               (send req :ik_request :robot_state :joint_state :name)
               (send req :ik_request :robot_state :joint_state :position)
               (= (length (send req :ik_request :robot_state :joint_state :name))
                  (length (send req :ik_request :robot_state :joint_state :position))))
        (progn
          (dotimes (i (length (send req :ik_request :robot_state :joint_state :name)))
            (let ((joint-name (elt (send req :ik_request :robot_state :joint_state :name) i)))
              (let ((j (find joint-name (send *atlas* :joint-list) :test #'string= :key #'(lambda (x) (send x :name)))))
                (if j
                    (send j :joint-angle (rad2deg (elt (send req :ik_request :robot_state :joint_state :position) i)))
                  (warn "we cannot find ~A~%" joint-name)
                  )))))
      (send *atlas* :reset-manip-pose)
      )
    (send *viewer* :draw-objects)
    (send *atlas* :newcoords (make-coords)) ;force to move pelvis origin
    (let ((goal (send req :ik_request :pose_stamped)))
      ;; check that goal is malformed or not
      (let ((q (float-vector (send req :ik_request :pose_stamped :pose :orientation :w)
                             (send req :ik_request :pose_stamped :pose :orientation :x)
                             (send req :ik_request :pose_stamped :pose :orientation :y)
                             (send req :ik_request :pose_stamped :pose :orientation :z))))
        (if (not (eps= (norm q) 1.0))
            (return nil)))
      (let ((goal-in-world (send *tf* :transform-pose "/pelvis" goal)))
        (let ((ik-ret (cond
                       (fullbodyp
                        (fullbody-ik-with-collision arm goal-in-world)
			)
                       (torsop
                        (torso-ik-with-collision arm goal-in-world))
                       (t
                        (normal-ik-with-collision arm goal-in-world)))))
          (if ik-ret
              (progn
                (send res :error_code :val moveit_msgs::MoveItErrorCodes::*SUCCESS*)
                (send res :solution :joint_state :name (send-all (send *atlas* :joint-list) :name))
                (send res :solution :joint_state :position
                      (map float-vector #'deg2rad (send *atlas* :angle-vector)))
                )
            (progn
              (send res :error_code :val moveit_msgs::MoveItErrorCodes::*FAILURE*)))
          res)))))

(unless (boundp '*do-not-start-ik-server*)
  ;; initialization
  (setq *atlas* (atlas-with-hand))
  (send *atlas* :newcoords (make-coords))
  (send *atlas* :reset-manip-pose)
  (setq *robot* *atlas*)
  (send-all (send *atlas* :joint-list) :max-joint-velocity 0.3)
  (setup-end-coords)
  (make-foot-convex)
  (make-centroid-object)
  (objects (list *atlas* *foot-convex* *centroid-object*))
  (send *irtviewer* :change-background #f(1.0 0.9 0.9))
  (send *irtviewer* :title "IK server")
  (send *irtviewer* :draw-objects)

  (setq *tf* (instance ros::transform-listener :init))

  (init-fullbody-ik-main)
  (make-foot-convex)

  (ros::advertise-service "/atlas_eus_ik" moveit_msgs::GetPositionIK #'ik-service-cb)
  (ros::rate 10)
  (do-until-key
   (if (not (ros::ok)) (return-from nil nil))
   (x::window-main-one)
   (ros::sleep)
   (ros::spin-once)))

