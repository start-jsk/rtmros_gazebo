#!/usr/bin/env roseus

(ros::load-ros-manifest "arm_navigation_msgs")
(ros::load-ros-manifest "jsk_interactive")
(ros::roseus "atlas-test")
(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")

(atlas-init-ex :view nil :set-reset-pose nil)
;;(atlas-init-ex :set-reset-pose nil)
;; (set-user)
(real2model)
;;(model2real :time 5000 :wait t)

(when (and (boundp '*irtviewer*) *irtviewer*)
  (send *irtviewer* :change-background #f(0.3 0.3 0.7))
  (send *irtviewer* :title "Interactive Marker Joints")
  (send *irtviewer* :draw-objects))

(defun joint-state-callback
  (msg)
  (let ((joint-names (send msg :name))
       (joint-angles (send msg :position))
       joint-name joint-angle
       )
    (dotimes (x (length joint-names))
      (setq joint-name (elt joint-names x))
      (setq joint-angle (rad2deg (elt joint-angles x)))
      (send *atlas* (intern (string-upcase joint-name) *keyword-package*)
	    :joint-angle joint-angle)
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      )

    (model2real :wait t :time 5000)
    )
  )

(setq server-nodename "jsk_model_marker_interface")

(ros::subscribe (format nil "~A/atlas/joint_states_ri_move" server-nodename)
		sensor_msgs::JointState #'joint-state-callback)

(ros::rate 30)

(warn "type (start-loop) to start~%")

(defun start-loop
  ()
  (while t
   (ros::spin-once)
   (unless (or (null x::*display*) (= x::*display* 0))
     (x::window-main-one))
   (ros::sleep)

   (unless (ros::ok) (return)))
  )

(start-loop)