#!/usr/bin/env roseus


(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "move_base_msgs")
;; (ros::load-ros-manifest "actionlib_tutorials")

;;(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(ros::roseus "touch_it_action_node")

(load "package://hrpsys_gazebo_atlas/euslisp/touch-it-util.l")


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
   tc
   (ik-type (case seq
        (0 :larm)
        (1 :larm-torso)
        (2 :larm-fullbody)
        (3 :rarm)
        (4 :rarm-torso)
        (5 :rarm-fullbody)))
   touch-flag
   arm
   msg)

    (x::window-main-one)

    (warning-message 4 "[touch_it] callback function was called !~%")

    ;; recieve arguments
    (warning-message 4 "[touch_it] original pose timestamp is ~a!~%"
         (send goal :goal :target_pose :header :stamp))
    (send goal :goal :target_pose :header :stamp (ros::time 0)) ;; This is a bug.
    (warning-message 4 "[touch_it] overwrite pose timestamp is ~a!~%"
         (send goal :goal :target_pose :header :stamp))
    (setq tc (send *tf* :transform-pose "/pelvis" pose-stamped))

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
    (setq touch-flag (touch-it-control tc :arm arm :ik-type ik-type))

    ;; set result
    (setq msg (send server :result))

    ;; send result
    (warning-message 1 "[touch_it] result is ~a !~%" msg)
    ;; Touched: Succeeded
    ;; IKfound: Rejected
    ;; Cancelled: Aboarted
    ;; NotReached: Lost
    (send server :set-succeeded msg)
    ;; (cond (touch-flag (send server :set-succeeded msg))
    ;;      (t (send server :set-lost msg)))
    ;; (send server :set-aborted msg)
    ;; (send server :set-rejected msg)
    ;; (send server :set-lost msg)
    ))


(unless (boundp '*do-not-start-touch-it*)
  (setq *s* (instance ros::simple-action-server :init
          "/touchit/act/exec" move_base_msgs::MoveBaseAction
          :execute-cb 'touch-it-action-cb))

  (ros::rate 500)

  (setq *tf* (instance ros::transform-listener :init))
  ;;(atlas-init-ex :real nil)
  ;;(atlas-init-ex :set-reset-pose nil)
  ;;(send *atlas* :reset-manip-pose)

  (init-touch-it-without-server)
  ;; (init-touch-it-without-server :real nil)

  (send *irtviewer* :draw-objects)
  (send *atlas* :newcoords (make-coords))

  (send *irtviewer* :change-background #f(0.2 0.2 0.8))
  (send *irtviewer* :title "TouchIt!")
  (send *irtviewer* :draw-objects)


  (warning-message 1 "~%")
  (warning-message 1 "[touch_it] touch_it_action_node started !~%")

  (while t
    (x::window-main-one)
    (send *s* :worker)
    (ros::spin-once)
    (ros::sleep))
  )
