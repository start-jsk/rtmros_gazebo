#!/usr/bin/env roseus

(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::load-ros-manifest "sandia_hand_msgs")
(ros::load-ros-manifest "humanoid_nav_msgs")
(ros::load-ros-manifest "atlas_msgs")
(ros::load-ros-manifest "arm_navigation_msgs")
(ros::load-ros-manifest "moveit_msgs")

(require :rtm-ros-robotics-interface
	 "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")

(require "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
(require "package://hrpsys_gazebo_atlas/euslisp/atlas-fullbody-ik.l")
(require "models/arrow-object.l")
(require "package://hrpsys_gazebo_atlas/euslisp/atlas-end-coords.l")

(ros::roseus "atlas_eus_ik")

(defun fullbody-ik-with-collision (arm goal-in-world &rest args)
  (apply
   #'fullbody-ik-main
   (append
    args
    (list
     :target-limb (list arm :lleg :rleg)
     :target-coords (list goal-in-world nil nil)
     :collision? t
     ;;:centroid-thre 100
     :root-link-virtual-joint-weight
     (float-vector 0.02 0.02 0.1 0.01 0.04 0.04)
     :thre '(5 5 5)
     :cog-gain 0.7
     ;;:target-centroid-pos nil
     ;;:stop 100
     :rotation-axis '(t t t)))))

(defun torso-ik-with-collision (arm goal-in-world
				    &rest args
				    &key
				    (waist-gain '(0.1 0.1 0.1))
				    (root-link-virtual-joint-weight
				     (float-vector 0 0 0 0 0 0))
				    (max
				     (map float-vector
					  #'*
					  root-link-virtual-joint-weight
					  #F(1000 1000 1000 200 200 200)))
				    (min
				     (scale
				      -1
				      (map float-vector
					   #'*
					   root-link-virtual-joint-weight
					   #F(1000 1000 1000 200 200 200))))
				    &allow-other-keys)
  (apply
   #'fullbody-ik-main
   (append
    args
    (list
     :target-limb (list arm :lleg :rleg)
     :target-coords (list goal-in-world nil nil)
     :additional-weight-list
     (mapcar
      #'(lambda (k gain)
	  (list (send *atlas* :torso k :child-link)
		gain))
      (list :waist-r :waist-p :waist-y)
      waist-gain)
     ;;(list 1e-3 1e-3 1e-3))
     :root-link-virtual-joint-weight
     root-link-virtual-joint-weight
     :min-vec min
     :max-vec max
     :collision? t
     :dif-pos-ratio 1.0
     :dif-rot-ratio 0.00500
     :manipulability-gain 0 ;0.00500
     ;;:centroid-thre 200
     ;;:target-centroid-pos nil
     :thre '(5 5 5)
     ;;:stop 100
     :rotation-axis '(t t t)))))

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
	    args
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
	       (make-list 3 :initial-element 0)))
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
	     :centroid-thre 1e+10000
	     ;:stop 100
	     :dif-pos-ratio 1.0
	     :dif-rot-ratio 0.00500
	     :manipulability-gain 0.00500
	     :thre '(5 5 5)
	     :rthre (list (deg2rad 1) (deg2rad 1) (deg2rad 1))
	     :rotation-axis '(t t t))))))
    (mapcar
     #'(lambda (j g) (send j :max-joint-velocity g))
     (send *atlas* :joint-list) w)
    ret))

(defun ik-service-cb (req)
  (let* ((res (send req :response))
         (arm (if (or (substringp "larm" (send req :ik_request :group_name))
                      (substringp "lhand" (send req :ik_request :group_name)))
                  :larm
                :rarm))
         (torsop (substringp "torso" (send req :ik_request :group_name)))
	 (torso2p (substringp "torso2" (send req :ik_request :group_name)))
	 (torso3p (substringp "torso3" (send req :ik_request :group_name)))
         (fullbodyp (substringp "fullbody" (send req :ik_request :group_name)))
	 (end-coords-id
	  (if (substringp "eus" (send req :ik_request :group_name))
	      (switch-end-coords :id 2)
	    (switch-end-coords :id 1)))
         (opt-str (send (send (send req :ik_request) :constraints) :name))
         (opt
          (cond
           ((and (stringp opt-str)
                 (plusp (length opt-str))
                 (listp (setq opt-str (read-from-string opt-str))))
            opt-str)
           (t (list :update-convex t)))))
    ;; taking joint state into account
    (cond
     ((and (send req :ik_request :robot_state)
           (send req :ik_request :robot_state :joint_state)
           (send req :ik_request :robot_state :joint_state :name)
           (send req :ik_request :robot_state :joint_state :position)
           (= (length (send req :ik_request :robot_state :joint_state :name))
              (length (send req :ik_request :robot_state :joint_state :position))))
      (dotimes (i (length (send req :ik_request :robot_state :joint_state :name)))
        (let ((joint-name (elt (send req :ik_request :robot_state :joint_state :name) i)))
          (let ((j (find joint-name (send *atlas* :joint-list)
                         :test #'string= :key #'(lambda (x) (send x :name)))))
            (if j
                (send j :joint-angle
                      (rad2deg
                       (elt
                        (send req :ik_request :robot_state :joint_state :position)
                        i)))
              (warn "we cannot find ~A~%" joint-name)
              )))))
     (t (send *robot* :reset-manip-pose)))
    (format t "[ik-server] options: ~A~%" opt)
    (format t "            target: limb=~A, torso=~A(~A), fullbody=~A, end-coords=~A~%"
	    arm torsop torso2p fullbodyp end-coords-id)
    (send *robot* :fix-leg-to-coords (make-coords) :both)
    (cond
     ((cadr (member :update-convex opt))
      (make-foot-convex :force? t)
      (send *irtviewer* :objects
	    (list *robot* *centroid-object* *foot-convex* *arrow*))))
    ;;(make-foot-convex)
    ;;(send *irtviewer* :objects
    ;;(list *robot* *foot-convex* *centroid-object* *arrow*))
    (send *viewer* :draw-objects)
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
                (send (send *robot* :copy-worldcoords)
                      :transform goal-in-pelvis))
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
			(torso2p
			 (apply
                          #'torso-ik-with-collision
                          (append
                           (list arm goal-in-world)
			   (list :root-link-virtual-joint-weight
				 (float-vector 0.02 0.02 0.1 0.0 0.0 0.04))
                           opt)))
			(torso3p
			 (apply
                          #'torso-ik-with-collision
                          (append
                           (list arm goal-in-world)
			   (list :waist-gain '(0 0 1))
                           opt)))
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
          ;;(print (list 'original goal))
          ;; (print (list 'original
          ;;              (send goal :header :frame_id)
          ;;              (send goal :pose :position :x)
          ;;              (send goal :pose :position :y)
          ;;              (send goal :pose :position :z)
          ;;              (send goal :pose :orientation :x)
          ;;              (send goal :pose :orientation :y)
          ;;              (send goal :pose :orientation :z)
          ;;              (send goal :pose :orientation :w)))
          ;; (print (list 'goal-in-pelvis goal-in-pelvis))
          ;; (print (list 'goal-in-world goal-in-world))
          (vectorp ik-ret))
	;;(format t "[ik-server] SUCCESS!!~%")
        (send res :error_code :val moveit_msgs::MoveItErrorCodes::*SUCCESS*)
	(send *viewer* :draw-objects)
        (send res :solution :joint_state :name
	      (send-all (send *atlas* :joint-list) :name))
        (send res :solution :joint_state :position
              (map float-vector #'deg2rad (send *atlas* :angle-vector)))
        res)
       (t res)))))

(defun start-ik-server
  nil
  ;; initialization
  (defvar *arrow* (arrow))
  (defvar *atlas* (atlas-with-hand))
					;(send *atlas* :newcoords (make-coords))
  (send *atlas* :reset-manip-pose)
  (send *atlas* :fix-leg-to-coords (make-coords) :both)
  (defvar *robot* *atlas*)
  (send-all (send *atlas* :joint-list) :max-joint-velocity 0.3)
  ;;
  (send *robot* :torso :waist-r :max-angle 10)
  (send *robot* :torso :waist-r :min-angle -10)
  ;;
  (send *robot* :torso :waist-p :max-angle 10)
  (send *robot* :torso :waist-p :min-angle -10)
  ;;
  (cond
   ((not (and (boundp '*irtviewer*) *irtviewer*))
    (make-irtviewer :width 640 :height 640)
    (send *irtviewer* :change-background #f(0.9 1.0 0.9))
    (send *irtviewer* :title "IK server")))
  (send *irtviewer*
	:objects
	(list *atlas* *arrow*))
  (make-foot-convex)
  ;;(make-centroid-object)
  (send *irtviewer*
	:objects
	(list *atlas* *foot-convex* *arrow*))
  (send *irtviewer* :look-all)
  (send *viewer* :draw-objects)
  ;;
  (setq *tf* (instance ros::transform-listener :init))
  ;;
  (init-fullbody-ik-main)
  (send *irtviewer*
	:objects
	(list *atlas* *foot-convex* *arrow* *centroid-object*))
  (switch-end-coords :id 1)
  ;;
  (ros::advertise-service "/atlas_eus_ik" moveit_msgs::GetPositionIK #'ik-service-cb)
  (ros::rate 10)
  (loop-ik-server)
  )

(defun loop-ik-server
  nil
  (do-until-key
   (if (not (ros::ok)) (return-from nil nil))
   (unless (or (null x::*display*) (= x::*display* 0))
     (x::window-main-one))
   (ros::sleep)
   (ros::spin-once)))

(unless (boundp '*do-not-start-ik-server*)
  (start-ik-server))

;;
;; roseus "(setq *setup-end-coords-for-hku-flag* nil)" atlas-eus-ik.l
;;
