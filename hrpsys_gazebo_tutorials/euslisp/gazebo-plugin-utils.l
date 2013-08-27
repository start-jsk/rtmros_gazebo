#!/usr/bin/env roseus

(ros::load-ros-manifest "geometry_msgs")


(defun init-gazebo-plugin ()
  (ros::roseus "GazeboPluginUtilsEus")
  (init-add-force)
  (init-set-vel)
  (init-get-vel)
)


(defun init-add-force ()
  (ros::roseus "AddForceEus")
  (ros::advertise "AddForcePlugin/ForceCommand" geometry_msgs::wrench 1)
  (ros::rate 100))

(defun add-force (_f &optional (_t (float-vector 0 0 0)))
  (let* ((msg (instance geometry_msgs::wrench :init)))
    (send msg :force :x (elt _f 0))
    (send msg :force :y (elt _f 1))
    (send msg :force :z (elt _f 2))
    (send msg :torque :x (elt _t 0))
    (send msg :torque :y (elt _t 1))
    (send msg :torque :z (elt _t 2))
    (ros::publish "AddForcePlugin/ForceCommand" msg))
  )
(defun reset-force ()
  (add-force (float-vector 0 0 0)))

(defun init-get-vel ()
  (ros::roseus "GetVelEus")

  (setq *abs-vel* nil)
  (setq *abs-accel* nil)
  (setq *rel-vel* nil)
  (setq *rel-accel* nil)
  (setq *pose* nil)
  (ros::subscribe "/GetVelPlugin/AbsVel" geometry_msgs::TwistStamped #'(lambda (msg) (setq *abs-vel* msg)))
  (ros::subscribe "/GetVelPlugin/AbsAccel" geometry_msgs::TwistStamped #'(lambda (msg) (setq *abs-accel* msg)))
  (ros::subscribe "/GetVelPlugin/RelVel" geometry_msgs::TwistStamped #'(lambda (msg) (setq *rel-vel* msg)))
  (ros::subscribe "/GetVelPlugin/RelAccel" geometry_msgs::TwistStamped #'(lambda (msg) (setq *rel-accel* msg)))
  (ros::subscribe "/GetVelPlugin/Pose" geometry_msgs::PoseStamped #'(lambda (msg) (setq *pose* msg)))
  )

(defun get-abs-vel ()
  (ros::spin-once)
  *abs-vel*)
(defun get-abs-accel ()
  (ros::spin-once)
  *abs-accel*)
(defun get-rel-vel ()
  (ros::spin-once)
  *rel-vel*)
(defun get-rel-accel ()
  (ros::spin-once)
  *rel-accel*)

(defun get-time-vel (&optional (vel (get-rel-vel)))
  (send (send (send vel :header) :stamp) :to-sec))
(defun get-linear-vel (&optional (vel (get-rel-vel)))
  (rosvector32eusvector (send (send vel :twist) :linear)))
(defun get-angular-vel (&optional (vel (get-rel-vel)))
  (rosvector32eusvector (send (send vel :twist) :angular)))
(defun get-pose ()
  (ros::spin-once)
  (let* ((pos (scale 1000 (rosvector32eusvector (send (send *pose* :pose) :position))))
	 (quat (rosvector42eusvector (send (send *pose* :pose) :orientation)))
	 (rot (quaternion2matrix quat)))
    (make-coords :pos pos :rot rot)
    ))


(defun init-set-vel ()
  (ros::roseus "SetVelEus")
  (ros::advertise "/SetVelPlugin/PoseCommand" geometry_msgs::pose 1)
  (ros::rate 100))

(defun set-pose (&optional (_c (make-coords)))
  (let* ((msg (instance geometry_msgs::pose :init))
	 (pos (scale 0.001 (send _c :worldpos)))
	 (rot (send _c :worldrot))
	 (quat (matrix2quaternion rot)))
    (send msg :position :x (elt pos 0))
    (send msg :position :y (elt pos 1))
    (send msg :position :z (elt pos 2))
    (send msg :orientation :w (elt quat 0))
    (send msg :orientation :x (elt quat 1))
    (send msg :orientation :y (elt quat 2))
    (send msg :orientation :z (elt quat 3))
    (ros::publish "/SetVelPlugin/PoseCommand" msg))
  )



(defun reset-world ()
  (reset-force)
  (set-pose))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; samples

(defun tilt-box-by-pd-control (&optional (dest-pitch (/ pi 6)))
  (do-until-key
   (let* ((dest-pitch-angle dest-pitch)
	  (crnt-pitch-angle (elt (car (rpy-angle (send (get-pose) :rot))) 1))
	  (diff-pitch-angle (- dest-pitch-angle crnt-pitch-angle))
	  (crnt-dpitch-angle (elt (get-angular-vel (get-abs-vel)) 1))
	  (p-gain 150)
	  (d-gain 75)
	  (pitch-moment (+ (* p-gain diff-pitch-angle) (- (* d-gain crnt-dpitch-angle))))
	  (f (float-vector 0 0 0))
	  (m (float-vector 0 pitch-moment 0)))
     (add-force f m)
     (warn "angle: ~a  moment: ~a~%" (rad2deg crnt-pitch-angle) pitch-moment)
     (unix::usleep (* 100 1000)))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; utils

(defun rosvector32eusvector (vec)
  (let* ((x (send vec :x)) (y (send vec :y)) (z (send vec :z)))
    (float-vector x y z)))
(defun rosvector42eusvector (vec)
  (let* ((x (send vec :x)) (y (send vec :y)) (z (send vec :z)) (w (send vec :w)))
    (float-vector w x y z)))
