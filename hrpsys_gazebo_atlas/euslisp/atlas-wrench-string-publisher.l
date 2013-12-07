#!/usr/bin/env roseus

(ros::load-ros-manifest "visualization_msgs")
(ros::load-ros-manifest "sensor_msgs")

(defun publish-string-result (text &optional (frame) (id))
  ;;(pprint text)
  (let ((marker (instance visualization_msgs::marker :init)))
    (send marker :header :frame_id frame)
    (send marker :id id)
    (send marker :lifetime (ros::time 0))
    ;;
    (send marker :pose :position :x -0.45)
    (if (substringp "lh" frame)
        (send marker :pose :position :y 0.2)
      (send marker :pose :position :y -0.2))
    (send marker :pose :position :z 0.0)
    ;;
    (send marker :pose :orientation :w 1)
    (cond
     ((substringp "lh" frame)
      (send marker :color :r 0.7)
      (send marker :color :g 0.3)
      (send marker :color :b 0)
      )
     (t
      (send marker :color :r 0.7)
      (send marker :color :g 0.0)
      (send marker :color :b 0.3)
      ))
    (send marker :color :a 1)
    (send marker :scale :z 0.07)
    (send marker :type visualization_msgs::marker::*TEXT_VIEW_FACING*)
    (send marker :text text)
    (ros::publish "/force_sensor_string" marker)
    ))

(defun callback (msg)
  ;;(pprint msg)
  (publish-string-result
   (format nil "F: ~4,1F ~4,1F ~4,1F (~4,1F)~%T: ~4,1F ~4,1F ~4,1F (~4,1F)"
	   (send msg :wrench :force :x)
	   (send msg :wrench :force :y)
	   (send msg :wrench :force :z)
	   (norm (float-vector (send msg :wrench :force :x)
			       (send msg :wrench :force :y)
			       (send msg :wrench :force :z)))
	   (send msg :wrench :torque :x)
	   (send msg :wrench :torque :y)
	   (send msg :wrench :torque :z)
	   (norm (float-vector (send msg :wrench :torque :x)
			       (send msg :wrench :torque :y)
			       (send msg :wrench :torque :z))))
   (send msg :header :frame_id)
   (if (substringp "lh" (send msg :header :frame_id))
       1001 1002)
   ))

(ros::roseus "wrench_string_publisher")

(ros::subscribe "/off_lhsensor_update" geometry_msgs::WrenchStamped
                #'callback)
(ros::subscribe "/off_rhsensor_update" geometry_msgs::WrenchStamped
                #'callback)

(ros::advertise "/force_sensor_string" visualization_msgs::marker)

(ros::rate 10)
(do-until-key
 (ros::spin-once)
 (ros::sleep))
