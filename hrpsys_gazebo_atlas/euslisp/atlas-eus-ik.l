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
(require "models/arrow-object.l")

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

(defun fullbody-ik-with-collision (arm goal-in-world &rest args)
  (apply
   #'fullbody-ik-main
   (append
    (list
     :target-limb (list arm :lleg :rleg)
     :target-coords (list goal-in-world nil nil)
     :collision? t
					;:centroid-thre 100
     :root-link-virtual-joint-weight
     (float-vector 0.1 0.1 0.1 0.04 0.04 0.04)
     :thre '(1 1 1)
     :stop 100
     :rotation-axis '(t t t))
    args)))

(defun torso-ik-with-collision (arm goal-in-world &rest args)
  (apply
   #'fullbody-ik-main
   (append
    (list
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
     :rotation-axis '(t t t))
    args)))

(defun normal-ik-with-collision (arm goal-in-world &rest args)
  (let* ((w
	  (send-all (send *atlas* :joint-list)
		    :max-joint-velocity))
	 (vel-limit
	  (send-all (send *robot* :joint-list)
		    :max-joint-velocity 0.2))
	 (ret
	  (apply
	   #'fullbody-ik-main
	   (append
	    (list
	     :target-limb (list arm :lleg :rleg)
	     :target-coords (list goal-in-world nil nil)
	     :additional-weight-list
	     (append
	      (mapcar
	       #'(lambda (k gain)
		   (list (send *atlas* :torso k :child-link)
			 gain))
	       (list :waist-r :waist-p :waist-y)
	       (make-list 3 :initial-element 0))
	      (mapcar
	       #'(lambda (k gain)
		   (list (send *atlas* arm k :child-link)
			 gain))
	       '(:shoulder-r :shoulder-y :elbow-p)
	       '(1e-4 1e-1 1e+10)))
	     ;; :additional-nspace-list
	     ;; (mapcar
	     ;;  #'(lambda (k)
	     ;; 	(list (send *atlas* arm k :child-link)
	     ;; 	      #'(lambda nil (- (random 1.0) 2.0))))
	     ;;  '(:elbow-p))
	     ;;(list 1e-3 1e-3 1e-3))
	     :root-link-virtual-joint-weight
	     (coerce (make-list 6 :initial-element 0) float-vector)
	     :min-vec (float-vector 0 0 0 0 0 0)
	     :max-vec (float-vector 0 0 0 0 0 0)
	     :collision? t
	     :centroid-thre nil
	     :stop 100
	     :thre '(1 1 1)
	     :rthre (list (deg2rad 1) (deg2rad 1) (deg2rad 1))
	     :rotation-axis '(t t t))
	    args))))
    (mapcar
     #'(lambda (j g) (send j :max-joint-velocity g))
     (send *atlas* :joint-list) w)
    ret))

(defun ik-service-cb (req)
  ;(setq hoge req)
  (let ((res (send req :response))
        (arm (if (or (string= (subseq (send req :ik_request :group_name) 0 4) "larm")
                     (string= (subseq (send req :ik_request :group_name) 0 4) "lhand"))
                 :larm
               :rarm))
        (torsop (or (string= (send req :ik_request :group_name) "larm_torso")
                    (string= (send req :ik_request :group_name) "rarm_torso")))
        (fullbodyp (or (string= (send req :ik_request :group_name) "larm_fullbody")
                       (string= (send req :ik_request :group_name) "rarm_fullbody")))
	(opt (apply
	      #'append
	      (mapcar
	       #'(lambda (dd) (list (car dd) (cdr dd)))
	       (send (send req :ik_request) :plist)))))
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
      (send *robot* :reset-manip-pose)
      (send *robot* :fix-leg-to-coords (make-coords) :both)
      (make-foot-convex)
      )
    ;;
    (send *viewer* :draw-objects)
    (send *robot* :fix-leg-to-coords (make-coords) :both)
    ;(send *atlas* :newcoords (make-coords)) ;force to move pelvis origin
    (let ((goal (send req :ik_request :pose_stamped))
      ;; check that goal is malformed or not
	  (q (float-vector (send req :ik_request :pose_stamped :pose :orientation :w)
			   (send req :ik_request :pose_stamped :pose :orientation :x)
			   (send req :ik_request :pose_stamped :pose :orientation :y)
			   (send req :ik_request :pose_stamped :pose :orientation :z))))
      (send res :error_code :val moveit_msgs::MoveItErrorCodes::*FAILURE*)
      (cond
       ((not (eps= (norm q) 1.0))
	(format t "invalid quotanion detected ~A~%" q)
	res)
       ((let* ((goal-in-pelvis (send *tf* :transform-pose "/pelvis" goal))
	       (goal-in-world
		(make-coords
		 :pos (v+ (send *robot* :worldpos)
			  (send goal-in-pelvis :worldpos))
		 :rot (m* (send *robot* :worldrot)
			  (send goal-in-pelvis :worldrot))))
	       (draw-arrow
		(send *arrow* :newcoords goal-in-world))
	       (ik-ret (cond
			(fullbodyp
			 (apply
			  #'fullbody-ik-with-collision
			  (append
			   (list arm goal-in-world)
			   opt))
			 )
			(torsop
			 (apply
			  #'torso-ik-with-collision
			  (append
			   (list arm goal-in-world)
			   opt)))
			(t
			 (apply
			  #'normal-ik-with-collision
			  (append
			   (list arm goal-in-world)
			   opt))))))
	  ik-ret)
	(send res :error_code :val moveit_msgs::MoveItErrorCodes::*SUCCESS*)
	(send res :solution :joint_state :name (send-all (send *atlas* :joint-list) :name))
	(send res :solution :joint_state :position
	      (map float-vector #'deg2rad (send *atlas* :angle-vector)))
	res)
       (t
	(format t "ik unresolved~%" q)
	res)))))

(defun start-ik-server
  nil
  (unless (boundp '*do-not-start-ik-server*)
    ;; initialization
    (setq *arrow* (arrow))
    (setq *atlas* (atlas-with-hand))
					;(send *atlas* :newcoords (make-coords))
    (send *atlas* :reset-manip-pose)
    (send *atlas* :fix-leg-to-coords (make-coords) :both)
    (setq *robot* *atlas*)
    (send-all (send *atlas* :joint-list) :max-joint-velocity 0.3)
    (setup-end-coords)
    (make-foot-convex)
    (make-centroid-object)
    (cond
     ((not (and (boundp '*irtviewer*) *irtviewer*))
      (make-irtviewer :width 640 :height 640)
      (send *irtviewer* :change-background #f(0.9 1.0 0.9))
      (send *irtviewer* :title "IK server")))
    (send *irtviewer*
	  :objects
	  (list *atlas* *foot-convex*
		*arrow* *centroid-object*))
    (send *irtviewer* :look-all)
    (send *viewer* :draw-objects)
    ;;
    (setq *tf* (instance ros::transform-listener :init))
    ;;
    (init-fullbody-ik-main)
    ;;
    (ros::advertise-service "/atlas_eus_ik" moveit_msgs::GetPositionIK #'ik-service-cb)
    (ros::rate 10)
    (do-until-key
     (if (not (ros::ok)) (return-from nil nil))
     (unless (or (null x::*display*) (= x::*display* 0))
       (x::window-main-one))
     (ros::sleep)
     (ros::spin-once))))

(start-ik-server)

;; roseus "(setq *setup-end-coords-for-hku-flag* nil)" atlas-eus-ik.l 
