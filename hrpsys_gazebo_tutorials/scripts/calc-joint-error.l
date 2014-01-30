#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "hrpsys_gazebo_msgs")
(ros::load-ros-manifest "sensor_msgs")

(ros::roseus "joint_error_publisher")


;; method call
(defclass calc-joint-error-class
  :super propertied-object
  :slots (joint-name-list
	  joint-command-vec joint-state-vec
	  joint-error-vec))
(defmethod calc-joint-error-class
  (:init
   ()
   (ros::subscribe "/HRP2JSKNT/joint_command"
		   hrpsys_gazebo_msgs::JointCommand #'send self :joint-command-cb)
   (ros::subscribe "/joint_states"
		   sensor_msgs::JointState #'send self :joint-state-cb)
   (ros::advertise "/joint_error" std_msgs::Float64MultiArray 1))
  (:joint-command-cb
   (msg)
   ;;(print (send msg :position))
   (setq joint-command-vec (send msg :position))
   (when (and joint-command-vec joint-state-vec)
     (setq joint-error-vec
	   (concatenate float-vector (mapcar #'rad2deg
				(concatenate cons (v- joint-state-vec joint-command-vec)))))
     (send self :publish-error))
   )
  (:joint-state-cb
   (msg)
   ;;(print (send msg :name))
   ;;(print (send msg :position))
   (unless joint-name-list
     (setq joint-name-list (send msg :name)))
   (setq joint-state-vec (send msg :position))
   (when (and joint-command-vec joint-state-vec)
     (setq joint-error-vec
	   (concatenate float-vector (mapcar #'rad2deg
				(concatenate cons (v- joint-state-vec joint-command-vec)))))
     (send self :publish-error))
   )
  (:publish-error
   ()
   (setq msg (instance std_msgs::Float64MultiArray :init))
   (when joint-error-vec
     (send msg :data joint-error-vec))
   (ros::publish "/joint_error" msg)
   )
  )
(setq m (instance calc-joint-error-class :init))

(do-until-key
 (ros::spin-once)
)

