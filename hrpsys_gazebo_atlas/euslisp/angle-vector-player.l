#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "std_srvs")
(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "visualization_msgs")

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

(defun nth-angle-vector
  (id)
  (cond
   ((or (< id 0)
	(>= id (length *angle-vector-list*)))
    (log "index out of range ~A~%" id) nil)
   (t
    (let ((target (nth id *angle-vector-list*)))
      (cond
       ((vectorp target) target)
       ((functionp target) (funcall target :id id))
       (t (log "unknow type ~A~%" target) nil))))))

(defun prev (req)
  (log "prev")
  (cond
   ((zerop (length *angle-vector-list*))
    (log "no-candidates!!" :pre-fix nil))
   ((<= *angle-vector-list-index* 0)
    (publish-text "no candidate")
    ;;(setq *angle-vector-list-index*
    ;;(- (length *angle-vector-list*) 1))
    (send *robot* :angle-vector
	  (nth *angle-vector-list-index*
	       *angle-vector-list*)))
   (t
    (setq *angle-vector-list-index*
	  (- *angle-vector-list-index* 1))
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
    (publish-text "no candidate")
    ;;(setq *angle-vector-list-index* 0)
    (send *robot* :angle-vector
	  (nth-angle-vector *angle-vector-list-index*)))
   (t
    (setq *angle-vector-list-index*
	  (+ *angle-vector-list-index* 1))
    (send *robot* :angle-vector
	  (nth-angle-vector
	   *angle-vector-list-index*))))
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

(defun publish-text
  (text)
  (let ((marker (instance visualization_msgs::marker :init)))
    (send marker :header :frame_id "/im_target_coords_arrow")
    (send marker :id 1)
    (send marker :lifetime (ros::time 5))
    (send marker :pose :position :z 1)
    (send marker :pose :orientation :w 1)
    (send marker :color :r 1)
    (send marker :color :g 1)
    (send marker :color :a 1)
    (send marker :scale :z 0.3)
    (send marker :type visualization_msgs::marker::*TEXT_VIEW_FACING*)
    (send marker :text text)
    (ros::publish (format nil "~A/marker" *server-nodename*) marker)
    )
  )

(defvar *server-nodename*
  "/jsk_interactive_marker_manipulation")
(defvar *im-nodename* "jsk_model_marker_interface")
(defvar *angle-vector-list-index* 0)
(defvar *angle-vector-list*)

(ros::advertise
 (format nil "~A/marker" *server-nodename*)
 visualization_msgs::Marker)

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

