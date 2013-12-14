#!/usr/bin/env roseus


(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "move_base_msgs")
;; (ros::load-ros-manifest "actionlib_tutorials")

;;(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(ros::roseus "touch_it_action_node")

(load "package://hrpsys_gazebo_atlas/euslisp/touch-it-util.l")


;; global variables to store IK status
(setq *touch-it-goal-new-p* t)
(setq *touch-it-force-reference* nil)
(setq *touch-it-iteration-count* 0)
(setq *touch-it-iteration-div* 0)

(defun touch-it-preempt-cb (server goal)
  ;; does not support distinghushing between goal, cancel everything always
  (setq *touch-it-goal-new-p* t)
  (when (and (boundp '*ri*) *ri*)
    (real2model)
    (model2real :time 500))
  (send server :set-preempted)
  )

(defun touch-it-action-cb (server goal)
  (let* ((pose-stamped (send goal :goal :target_pose))
         (pose (send pose-stamped :pose))
         (pos (send pose :position))
         (ori (send pose :orientation))
         (seq (send goal :goal :target_pose :header :seq))
         (vec (scale 1000.0 (float-vector (send pos :x) (send pos :y) (send pos :z))))
         (goal-coords (make-coords :pos vec))
         tc
         (ik-type
	  (case (mod seq 10)
	    (0 :larm)
	    (1 :larm-torso)
	    (2 :larm-fullbody)
	    (3 :rarm)
	    (4 :rarm-torso)
	    (5 :rarm-fullbody)))
         touch-flag
	 (axis
	  (case (round (/ seq 10))
	    (0 :z)
	    (1 :x)
	    (2 :y)
	    (3 nil)
	    (t t)))
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

    ;; get initial position only if the goal is new
    (if *touch-it-goal-new-p*
        (if (and (boundp '*ri*) *ri*) (real2model)))
    ;;(send *atlas* :reset-pose))
    ;;(send *atlas* :reset-manip-pose))
    (send *irtviewer* :draw-objects)
    (send* tc :rpy (car (rpy-angle (send (send *atlas* arm :end-coords) :worldrot))))

    ;; control
    (warning-message 1 "[touch_it] tc is ~a !~%" tc)

    ;; initialize global variables if needed
    (when *touch-it-goal-new-p*
      (setq *touch-it-goal-new-p* nil)
      (if (and (boundp '*ri*) *ri*)
	  (setq *touch-it-force-reference*
		(if (send *ri* :state :force-vector arm)
		    (send *ri* :state :absolute-force-vector arm)
		  #f(0 0 0))))
      (setq *touch-it-iteration-count* 0)
      (let* ((pos-crnt (send *atlas* arm :end-coords :worldpos))
	     (pos-dest (send tc :worldpos))
	     (tmp-dist (norm (v- pos-crnt pos-dest))))
	(setq *touch-it-iteration-div* (+ (/ tmp-dist 50) 1)))
      )

    (warning-message 1 "[touch_it] arm=~A ik-type=~A axis=~A !~%" arm ik-type axis)
    (setq touch-flag (touch-it-control-proc tc
					    :arm arm
					    ;; :ik-type ik-type
					    :f-orig *touch-it-force-reference*
					    :i *touch-it-iteration-count*
					    :div *touch-it-iteration-div*
					    ;; :axis axis
					    :thre *touchit-thre*))
    (incf *touch-it-iteration-count*)
    ;; set result
    (setq msg (send server :result))

    ;; send result
    (warning-message 1 "[touch_it] result is ~a !~%" msg)
    ;; Touched: Succeeded
    ;; Cancelled: Preemped
    ;; NotReached: Aboarted
    (cond ((eq touch-flag :continue)
           (warning-message 1 "[touch_it] continues to solve ik~%")
           ;; not sending goal yet, we will solve IK on the next iteratoin (:worker method calls :execution-cb)
           )
          ((eq touch-flag :arrived-at-goal)
           (warning-message 1 "[touch_it] IK finished~%")
           (send server :set-aborted msg)
           (setq *touch-it-goal-new-p* t)
           )
          ((eq touch-flag :touched)
           (warning-message 1 "[touch_it] touched!~%")
	   (send server :set-succeeded msg)
	   (when (and (boundp '*ri*) *ri*)
	     (real2model)
	     (model2real :time 500))
	   (setq *touch-it-goal-new-p* t)
           ))
    (send server :publish-feedback
	  (instance move_base_msgs::MoveBaseActionFeedback :init :feedback
		    (instance move_base_msgs::MoveBaseFeedback :init :base_position
			      (ros::coords->tf-pose-stamped (make-coords :pos (float-vector *touchit-val* *touchit-thre* *touchit-ratio*)
									 :rpy (list (if *touchit-ik-res* 1 0) 0 0)) 0))))
    ))

(defun touchit-thre-cb (msg)
  (let* ((val (send msg :data)))
    (when (and (>= val 0) (<= val 100))
      (setq *touchit-thre* val)
      )))


(unless (boundp '*do-not-start-touch-it*)
  
  (cond ((unix:getenv "TOUCH_IT_ACTION_TOPIC_NAME")
	 (setq *touch-it-action-topic-name* (unix:getenv "TOUCH_IT_ACTION_TOPIC_NAME")))
	(t
	 (setq *touch-it-action-topic-name* "/touchit/act/exec")))

  (setq *s* (instance ros::simple-action-server :init
                      *touch-it-action-topic-name* move_base_msgs::MoveBaseAction
                      :execute-cb 'touch-it-action-cb
                      :preempt-cb 'touch-it-preempt-cb))

  (ros::rate 1.0)

  (setq *tf* (instance ros::transform-listener :init))


  (ros::ros-warn "TOUCH_IT_ACTION_TOPIC_NAME=~a" (unix:getenv "TOUCH_IT_ACTION_TOPIC_NAME"))
  (unix::usleep (* 10000 1000))
  (init-touch-it-without-server :real t)
  (ros::ros-warn "touchit ri=~a" *ri*)
  (send *atlas* :newcoords (make-coords))

  (send *irtviewer* :title (format nil "~a : ~a" "TouchIt!" (unix:getenv "TOUCH_IT_ACTION_TOPIC_NAME")))
  (cond ((substringp "ocs" (unix:getenv "TOUCH_IT_ACTION_TOPIC_NAME"))
	 (send *irtviewer* :change-background #f(0.2 0.2 0.8)))
	(t
	 (send *irtviewer* :change-background #f(0.5 0.5 0.9))))

  (send *irtviewer* :draw-objects)

  (ros::ros-warn "~%")
  (ros::ros-warn "[touch_it] touch_it_action_node started !~%")

  (setq *touchit-val* 0)
  (setq *touchit-thre* 20)
  (setq *touchit-ratio* 0)
  (setq *touchit-ik-res* nil)

  (ros::subscribe "/touchit/thre" std_msgs::Float64 #'touchit-thre-cb)

  (do-until-key
   (x::window-main-one)
   (send *s* :worker)
   (ros::spin-once)
   (ros::sleep))
  )

