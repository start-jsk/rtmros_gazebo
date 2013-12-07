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
    (cond
     ((substringp "lh" frame)
      (send marker :pose :position :x -0.35)
      (send marker :pose :position :y 0.2)
      (send marker :pose :position :z 0.0))
     ((substringp "rh" frame)
      (send marker :pose :position :x -0.45)
      (send marker :pose :position :y -0.2)
      (send marker :pose :position :z 0.0))
     ((substringp "lf" frame)
      (send marker :pose :position :x 0.0)
      (send marker :pose :position :y 0.5)
      (send marker :pose :position :z 0.2))
     ((substringp "rf" frame)
      (send marker :pose :position :x 0.0)
      (send marker :pose :position :y -0.5)
      (send marker :pose :position :z 0.2))
     )
    ;;
    (send marker :pose :orientation :w 1)
    (cond
     ((substringp "lh" frame)
      (send marker :color :r 1.0)
      (send marker :color :g 0.55)
      (send marker :color :b 0.25))
     ((substringp "rh" frame)
      (send marker :color :r 1.0)
      (send marker :color :g 0.25)
      (send marker :color :b 0.55))
     ((substringp "lf" frame)
      (send marker :color :r 0.55)
      (send marker :color :g 0.25)
      (send marker :color :b 1.0))
     ((substringp "rf" frame)
      (send marker :color :r 0.25)
      (send marker :color :g 0.55)
      (send marker :color :b 1.0))
     )
    (send marker :color :a 1)
    (send marker :scale :z 0.07)
    (send marker :type visualization_msgs::marker::*TEXT_VIEW_FACING*)
    (send marker :text text)
    (ros::publish "/force_sensor_string" marker)
    ))

(defun callback (msg)
  ;;(pprint msg)
  (let ((fr (send msg :header :frame_id)))
    (publish-string-result
     (format nil "~A~%F: ~4,1F ~4,1F ~4,1F (~4,1F)~%T: ~4,1F ~4,1F ~4,1F (~4,1F)"
	     (cond
	      ((substringp "lh" fr) "L hand")
	      ((substringp "rh" fr) "R hand")
	      ((substringp "lf" fr) "L foot")
	      ((substringp "rf" fr) "R foot"))
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
     fr
     (cond
      ((substringp "lh" fr) 1001)
      ((substringp "rh" fr) 1002)
      ((substringp "lf" fr) 1003)
      ((substringp "rf" fr) 1004))
     )))

(ros::roseus "wrench_string_publisher")

(ros::subscribe "/off_lhsensor_update" geometry_msgs::WrenchStamped
                #'callback)
(ros::subscribe "/off_rhsensor_update" geometry_msgs::WrenchStamped
                #'callback)
(ros::subscribe "/off_lfsensor_update" geometry_msgs::WrenchStamped
                #'callback)
(ros::subscribe "/off_rfsensor_update" geometry_msgs::WrenchStamped
                #'callback)

(ros::advertise "/force_sensor_string" visualization_msgs::marker)

(ros::rate 10)
(do-until-key
 (ros::spin-once)
 (ros::sleep))
