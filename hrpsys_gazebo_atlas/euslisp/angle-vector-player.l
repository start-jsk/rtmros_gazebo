#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "std_srvs")
(ros::load-ros-manifest "sensor_msgs")

(ros::roseus "angle-vector-player")

(require "atlas-model.l")
(defvar *robot* (atlas-with-hand))

(defun log
  (str
   &key
   (pre-fix
    "[angle-vector-player]")
   (space-pre-fix
    "                     "))
  (format t "~A ~A~%" (or pre-fix space-pre-fix) str))

(defun prev (req)
  (log "prev")
  (cond
   ((zerop (length *angle-vector-list*))
    (log "no-candidates!!" :pre-fix nil))
   ((<= *angle-vector-list-index* 0)
    (setq *angle-vector-list-index*
	  (- (length *angle-vector-list*) 1))
    (send *robot* :angle-vector
	  (nth *angle-vector-list-index*
	       *angle-vector-list*)))
   (t
    (setq *angle-vector-list*
	  (- *angle-vector-list* 1))
    (send *robot* :angle-vector
	  (nth *angle-vector-list-index*
	       *angle-vector-list*))))
  (publish-joint-states)
  (send req :response)
  )

(defun refresh (req)
  (log "refresh")
  (publish-joint-states)
  (send req :response)
  )

(defun next (req)
  (log "next")
  (cond
   ((zerop (length *angle-vector-list*))
    (log "no-candidates!!" :pre-fix nil))
   ((>= *angle-vector-list-index*
	(- (length *angle-vector-list*) 1))
    (setq *angle-vector-list-index* 0)
    (send *robot* :angle-vector
	  (nth *angle-vector-list-index*
	       *angle-vector-list*)))
   (t
    (setq *angle-vector-list*
	  (+ *angle-vector-list* 1))
    (send *robot* :angle-vector
	  (nth *angle-vector-list-index*
	       *angle-vector-list*))))
  (publish-joint-states)
  (send req :response)
  )

(defun publish-joint-states
  nil
  (let ((joint-angles nil)
	(joint-names nil)
	(joint-list (send *robot* :joint-list))
	(joint-state-msg
	 (instance sensor_msgs::JointState :init
		   :header (instance std_msgs::header :init
				     :stamp (ros::time-now)))))
    (dotimes (x (length joint-list))
      (push (deg2rad (send (elt joint-list x) :joint-angle)) joint-angles)
      (push (send (elt joint-list x) :name) joint-names)
      )
    (send joint-state-msg :position joint-angles)
    (send joint-state-msg :name joint-names)
    ;;
    (ros::publish (format nil "~A/atlas/reset_joint_states" *im-nodename*)
		  joint-state-msg))
  )

(defvar *im-nodename* "jsk_model_marker_interface")
(defvar *angle-vector-list-index* 0)
(defvar *angle-vector-list*)

(ros::advertise
 (format nil "~A/atlas/reset_joint_states" *im-nodename*)
 sensor_msgs::JointState)

(ros::advertise-service
 "/prev" std_srvs::Empty #'prev)

(ros::advertise-service
 "/refresh" std_srvs::Empty #'refresh)

(ros::advertise-service
 "/next" std_srvs::Empty #'next)

(ros::rate 500)

(log "start!!")
(do-until-key
 (x::window-main-one)
 (ros::spin-once)
 (ros::sleep))

