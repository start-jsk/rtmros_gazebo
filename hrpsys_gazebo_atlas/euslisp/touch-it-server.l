#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "move_base_msgs")
;; (ros::load-ros-manifest "actionlib_tutorials")

;;(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(ros::roseus "touch_it_action_node")


(defmethod ros::simple-action-server
  (:set-rejected
   (msg &optional (text ""))
   (ros::ros-info ";; Action Rejected ~A" (send goal-id :id))
   (setq status actionlib_msgs::GoalStatus::*rejected*)
   (send self :publish-result msg text))
  (:set-lost
   (msg &optional (text ""))
   (ros::ros-info ";; Action Lost ~A" (send goal-id :id))
   (setq status actionlib_msgs::GoalStatus::*lost*)
   (send self :publish-result msg text)))

(defun touch-it-action-cb (server goal)
  (let* ((pose-stamped (send goal :goal :target_pose))
	 (pose (send pose-stamped :pose))
	 (pos (send pose :position))
	 (ori (send pose :orientation))
	 (seq (send goal :goal :target_pose :header :seq))
	 (vec (scale 1000.0 (float-vector (send pos :x) (send pos :y) (send pos :z))))
	 (goal-coords (make-coords :pos vec))
	 (tc (send *tf* :transform-pose "/pelvis" pose-stamped))
	 (ik-type (case seq
		    (0 :larm)
		    (1 :larm-torso)
		    (2 :larm-fullbody)
		    (3 :rarm)
		    (4 :rarm-torso)
		    (5 :rarm-fullbody)))
	 arm
	 msg)

    (x::window-main-one)

    ;; recieve arguments
    (warning-message 4 "[touch_it] callback function was called !~%")
    (warning-message 1 "[touch_it] goal pose is ~a !~%" pose)
    (warning-message 1 "[touch_it] pos = (~a, ~a, ~a)~%"
		     (send pos :x) (send pos :y) (send pos :z))
    (warning-message 1 "[touch_it] ori = (~a, ~a, ~a, ~a)~%"
		     (send ori :w) (send ori :x) (send ori :y) (send ori :z))
    (warning-message 1 "[touch_it] sequence is ~a !~%" seq)

    (cond ((substringp "larm" (string ik-type)) (setq arm :larm))
	  ((substringp "rarm" (string ik-type)) (setq arm :rarm)))

    ;; get initial position
    (if (and (boundp '*ri*) *ri*) (real2model))
      ;;(send *atlas* :reset-pose))
      ;;(send *atlas* :reset-manip-pose))
    (send *irtviewer* :draw-objects)
    (send* tc :rpy (car (rpy-angle (send (send *atlas* arm :end-coords) :worldrot))))

    ;; control
    (warning-message 1 "[touch_it] tc is ~a !~%" tc)
    (touch-it-control tc :arm arm :ik-type ik-type)

    ;; set result
    (setq msg (send server :result))

    ;; send result
    (warning-message 1 "[touch_it] result is ~a !~%" msg)
    ;; Touched: Succeeded
    ;; IKfound: Rejected
    ;; Cancelled: Aboarted
    ;; NotReached: Lost
    (send server :set-succeeded msg)
    ;; (send server :set-aborted msg)
    ;; (send server :set-rejected)
    ;; (send server :set-lost)
    ))

(defun touch-it-control
  (tc &key (arm :rarm) (ik-type arm))

  ;; calculate target
  (let ((div 30)
	(i 0))
    (do-until-key
     ;; check count
     (when (> i div)
       (return))
     ;; check the force sensor value
     (let* (f)
       (setq f 
	     (if (and (boundp '*ri*) *ri*)
		 (send *ri* :state :absolute-force-sensor arm)
	       0))
       (warning-message 2 "[touch_it] ~a force = ~a # touchitforce~%" arm f)
       (when nil
	 (return)))

     ;; set target coords
     (let* ((r (/ (+ i 1.0) div))
	    (c (midcoords r
			  (send (send *atlas* arm :end-coords) :worldcoords)
			  tc)))

       ;; solve ik
       (warning-message 1 "[touch_it] calculating pose <~a = ~a/~a> !~%" r i div)
       (cond ((substringp "fullbody" (string ik-type))
	      (fullbody-ik-with-collision arm c))
	     ((substringp "torso" (string ik-type))
	      (torso-ik-with-collision arm c))
	     (t
	      (send *atlas* arm :inverse-kinematics c :stop 100
		    :thre 2
		    :rthre (deg2rad 5)at
		    :rotation-axis :z
		    :revert-if-fail t))
	     )

       ;; send angle
       (if (and (boundp '*ri*) *ri*)
	   (progn
	     (model2real :time 2000 :wait t))
;;	     (send *ri* :angle-vector (send *atlas* :angle-vector) 2000)
;;	     (unix:sleep 1))
	 (progn
	   (unix:usleep (* 20 1000))))
       (send *irtviewer* :draw-objects)
       )
     (incf i)
     ))
  )


(unless (boundp '*do-not-start-touch-it*)
  (setq *s* (instance ros::simple-action-server :init
		      "/touchit/act/exec" move_base_msgs::MoveBaseAction
		      :execute-cb 'touch-it-action-cb))

  (ros::rate 500)

  (setq *do-not-start-ik-server* t)
  (load "package://hrpsys_gazebo_atlas/euslisp/atlas-eus-ik.l")
  (setq *tf* (instance ros::transform-listener :init))
  (atlas-init-ex :real nil)
  (send *atlas* :reset-manip-pose)
  (setup-end-coords)
  (send *irtviewer* :draw-objects)
  (send *atlas* :newcoords (make-coords))

  (send *irtviewer* :change-background #f(0.2 0.2 0.8))
  (send *irtviewer* :title "TouchIt!")
  (send *irtviewer* :draw-objects)


  (warning-message 1 "~%")
  (warning-message 1 "[touch_it] touch_it_action_node started !~%")

  (do-until-key
   (x::window-main-one)
   (send *s* :worker)
   (ros::spin-once)
   (ros::sleep))
  )
