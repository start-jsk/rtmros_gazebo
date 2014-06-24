#!/usr/bin/env roseus

(ros::load-ros-manifest "arm_navigation_msgs")
(ros::load-ros-manifest "jsk_interactive")
(ros::load-ros-manifest "std_msgs")
(ros::roseus "atlas_motion_sequence_player")

(require "atlas-interface.l")

(defvar *sequence-data*)
(require "atlas-motion-sequence-data.l")

(atlas-init-ex :view nil :set-reset-pose nil)
(defvar *robot* *atlas*)
(real2model)

(defvar *now-status* :wait)
(defvar *callback-cnt* 0)

(defun log
  (str &rest args)
  (let ((message
	 (apply
	  #'format
	  (append (list nil str) args))))
    (pub message)
    (format t "[motion-sequence-player] ~A~%" message)))

(defun pub
  (mes)
  (ros::publish
   "/eus_message2ocs"
   (instance std_msgs::string :init :data mes)))

(defun pre-call
  (tag)
  (setq *callback-cnt* 0)
  (let ((data
	 (find-if
	  #'(lambda (l) (eq tag (cdr (assoc :tag l))))
	  *sequence-data*)))
    (cond
     ((null data)
      (log "unknow tag ~A" tag))
     (t
      (log "correct pre-call")
      ;;(model2real :sleep-time 10000)
      (setq *now-status* tag)))))

(defun last-call
  (tag)
  (let* ((data
	  (find-if
	   #'(lambda (l) (eq tag (cdr (assoc :tag l))))
	   *sequence-data*))
	 (av
	  (cdr (assoc :angle-vector data)))
	 (time
	  (cdr (assoc :sleep-time data))))
    (cond
     ((> *callback-cnt* 10)
      (log "time exceeded"))
     ((null data)
      (log "unknow tag ~A" tag))
     ((or (null av)
	  (null time)
	  (not (eq (length av) (length time))))
      (log "invalid format ~A" tag))
     ((not (eq *now-status* tag))
      (log "invalid status ~A != ~A"
	   tag *now-status*))
     (t
      (log "correct last-call")
      (send *robot* :angle-vector
	    (cadr (assoc :angle-vector data)))
      (model2real :sleep-time 10000)
      (send *ri*
	    :angle-vector-sequence
	    av time)
      (send *ri* :wait-interpolation)))
    (setq *now-status* :wait)))

(defun sequence-subscriber
  (msg)
  ;; valid-form
  ;; (:atlas-motion-sequence-player :status)
  (let* ((command (send msg :data))
	 (command-s
	  (if (and (stringp command)
		   (plusp (length command)))
	      (read-from-string command))))
    (cond
     ((or (atom command-s)
	  (not (eq (car command-s)
		   :atlas-motion-sequence-player)))
      (log "invalid form ~A" command))
     ((not (eq (cadr command-s) *now-status*))
      (pre-call (cadr command-s)))
     (t (last-call (cadr command-s))))))

(ros::rate 1)
(defun start-loop
  ()
  (do-until-key
   (format t "[motion-sequence-player] ~A/~A"
	   *callback-cnt* 30)
   (bench
    (ros::spin-once)
    (unless (or (null x::*display*) (= x::*display* 0))
      (x::window-main-one))
    (ros::sleep)
    (setq *callback-cnt*
	  (min (+ *callback-cnt* 1) 30))
    (unless (ros::ok) (return))))
  )

(ros::subscribe "/eus_message2fc"
		std_msgs::string
		#'sequence-subscriber 1)
(ros::advertise "/eus_message2ocs"
		std_msgs::string 1)

(start-loop)
