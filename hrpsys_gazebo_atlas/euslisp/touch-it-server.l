#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "move_base_msgs")
;; (ros::load-ros-manifest "actionlib_tutorials")

;;(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(ros::roseus "touch_it_action_node")


(defun touch-it-action-cb (server goal)
  (let* ((pos (send goal :goal :target_pose :pose :position))
	 (seq (send goal :goal :target_pose :header :seq))
	 (vec (float-vector (send pos :x) (send pos :y) (send pos :z)))
	 msg)

    (x::window-main-one)

    ;; recieve arguments
    (warning-message 4 "[touch_it] callback function was called !~%")
    (warning-message 1 "[touch_it] goal position is ~a !~%" pos)
    (warning-message 1 "[touch_it] ~a = (~a, ~a, ~a)~%"
		     pos (send pos :x) (send pos :y) (send pos :z))
    (warning-message 1 "[touch_it] sequence is ~a !~%" seq)

    ;; calculate motion
    (print vec)
    (print *atlas*)
    (send *atlas* :larm :move-end-pos vec)
    (send *irtviewer* :draw-objects)

    ;; set result
    (setq msg (send server :result))
    ;;;; debug
    ;; (print (send server :slots))
    ;; (print (send (send server :result) :status))
    ;; (print (send (send (send server :result) :status) :methods))
    ;; (print (send (send (send server :result) :status) :status))
    ;; (send (send (send server :result) :status) :status 9)
    ;; (print (send (send (send server :result) :status) :status))

    ;; send result
    (warning-message 1 "[touch_it] result is ~a !~%" msg)
    (send server :set-succeeded msg)
    ;;(send server :set-aborted msg)
    ))

(setq *s* (instance ros::simple-action-server :init
		    "/touchit/act/exec" move_base_msgs::MoveBaseAction
		    :execute-cb 'touch-it-action-cb))

(ros::rate 500)

(setq *do-not-start-ik-server* t)
(load "package://hrpsys_gazebo_atlas/euslisp/atlas-eus-ik.l")
(atlas-init-ex :real nil)
(send *atlas* :reset-manip-pose)
(send *irtviewer* :draw-objects)

(warning-message 1 "~%")
(warning-message 1 "[touch_it] touch_it_action_node started !~%")

(do-until-key
 (x::window-main-one)
 (send *s* :worker)
 (ros::spin-once)
 (ros::sleep))

