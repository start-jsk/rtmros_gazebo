#!/usr/bin/env roseus

(ros::load-ros-manifest "geometry_msgs")


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; class definition

(defun init-gazebo-plugin
  (&optional (_objname "") (_objmodel nil))
  (let* ((gp (instance gazebo-plugin-obj-operate :init _objname)))
    (when _objmodel (send _objmodel :put :gp gp))
    gp))

(defclass gazebo-plugin-obj-operate
  :super propertied-object
  :slots (objname
	  abs-vel abs-accel rel-vel rel-accel pose)
  )

(defmethod gazebo-plugin-obj-operate
  ;; initialize
  (:init
   (&optional (_objname ""))
   (setq objname _objname)
   (ros::roseus "GazeboPluginUtilsEus")
   (ros::rate 100)		   
   (send self :init-add-force)
   (send self :init-set-vel)
   (send self :init-get-vel)
   (send self :init-set-vel)
   (send self :update))
  (:update ()
	   (ros::spin-once))
  ;; add-froce plugin ;;;;;;
  (:init-add-force
   ()
   (ros::roseus "AddForceEus")
   (ros::advertise (format nil "~a~a" objname "/AddForcePlugin/ForceCommand") geometry_msgs::wrench 1))
  (:add-force
   (_f &optional (_t (float-vector 0 0 0)))
   (let* ((msg (instance geometry_msgs::wrench :init)))
     (send msg :force :x (elt _f 0))
     (send msg :force :y (elt _f 1))
     (send msg :force :z (elt _f 2))
     (send msg :torque :x (elt _t 0))
     (send msg :torque :y (elt _t 1))
     (send msg :torque :z (elt _t 2))
     (ros::publish (format nil "~a~a" objname "/AddForcePlugin/ForceCommand") msg)))
  (:reset-force
   ()
   (send self :add-force (float-vector 0 0 0)))
  ;; get-vel plugin ;;;;;;
  (:init-get-vel
   ()
   (ros::roseus "GetVelEus")
   (ros::subscribe (format nil "~a~a" objname "/GetVelPlugin/AbsVel") geometry_msgs::TwistStamped #'send self :get-abs-vel-cb)
   (ros::subscribe (format nil "~a~a" objname "/GetVelPlugin/AbsAccel") geometry_msgs::TwistStamped #'send self :get-abs-vel-cb)
   (ros::subscribe (format nil "~a~a" objname "/GetVelPlugin/RelVel") geometry_msgs::TwistStamped #'send self :get-rel-vel-cb)
   (ros::subscribe (format nil "~a~a" objname "/GetVelPlugin/RelAccel") geometry_msgs::TwistStamped #'send self :get-rel-accel-cb)
   (ros::subscribe (format nil "~a~a" objname "/GetVelPlugin/Pose") geometry_msgs::PoseStamped #'send self :get-vel-cb))
  (:get-abs-vel-cb (msg) (setq abs-vel msg))
  (:get-abs-accel-cb (msg) (setq abs-accel msg))
  (:get-rel-vel-cb (msg) (setq rel-vel msg))
  (:get-rel-accel-cb (msg) (setq rel-accel msg))
  (:get-vel-cb (msg) (setq pose msg))
  (:get-abs-vel
   ()
   (send self :update)
   abs-vel)
  (:get-abs-accel
   ()
   (send self :update)
   abs-accel)
  (:get-rel-vel
   ()
   (send self :update)
   rel-vel)
  (:get-rel-accel
   ()
   (send self :update)
   rel-accel)
  (:get-time-vel
   (&optional (vel (send self :get-rel-vel)))
   (send (send (send vel :header) :stamp) :to-sec))
  (:get-linear-vel
   (&optional (vel (send self :get-rel-vel)))
   (rosvector32eusvector (send (send vel :twist) :linear)))
  (:get-angular-vel
   (&optional (vel (send self :get-rel-vel)))
   (rosvector32eusvector (send (send vel :twist) :angular)))
  (:get-pose
   ()
   (send self :update)
   (let* ((pos (scale 1000 (rosvector32eusvector (send (send pose :pose) :position))))
	  (quat (rosvector42eusvector (send (send pose :pose) :orientation)))
	  (rot (quaternion2matrix quat)))
     (make-coords :pos pos :rot rot)))
  ;; set-vel plugin ;;;;;;
  (:init-set-vel
   ()
   (ros::roseus "SetVelEus")
   (ros::advertise (format nil "~a~a" objname "/SetVelPlugin/PoseCommand") geometry_msgs::pose 1)
   (ros::advertise (format nil "~a~a" objname "/SetVelPlugin/VelCommand") geometry_msgs::twist 1)
   (ros::rate 100))
  (:set-pose
   (&optional (_c (make-coords)))
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
     (ros::publish (format nil "~a~a" objname "/SetVelPlugin/PoseCommand") msg)))
  (:set-vel
   (_v &optional (_w (float-vector 0 0 0)))
   (let* ((msg (instance geometry_msgs::twist :init)))
     (send msg :linear :x (elt _v 0))
     (send msg :linear :y (elt _v 1))
     (send msg :linear :z (elt _v 2))
     (send msg :angular :x (elt _w 0))
     (send msg :angular :y (elt _w 1))
     (send msg :angular :z (elt _w 2))
     (ros::publish (format nil "~a~a" objname "/SetVelPlugin/VelCommand") msg)))
  (:reset-world
   ()
   (send self :reset-force)
   (send self :set-pose))
  )
;;(warn "~%(setq *gp* (init-gazebo-plugin \"simple_box\"))")
;;(warn "~%(send *gp* :add-force #f(50 0 0))")
;;(warn "~%(send *gp* :set-vel #f(0.5 0 0) #f(0 0 0))")


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; pose

(defun obj-gzb2eus
  (&optional (obj *obj*) &key (loop? nil))
  (unless (send obj :get :gp)
    (warning-message 1 "^a has no gazebo plugin instance.~%" (send obj :name))
    (return-from gzb2eus))
  (do-until-key
   (let* ((pose (send (send obj :get :gp) :get-pose)))
     (send obj :newcoords pose))
   (send *pickview* :draw-objects)
   (unless loop? (return))
   (unix::usleep (* 10 1000)))
  )

(defun obj-eus2gzb
  (&optional (obj *obj*))
  (unless (send obj :get :gp)
    (warning-message 1 "^a has no gazebo plugin instance.~%" (send obj :name))
    (return-from eus2gzb))
  (let* ((pose (send obj :worldcoords)))
    (send (send obj :get :gp) :set-pose pose))
  )


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; samples

(defun tilt-box-by-pd-control (&optional (objname "") (dest-pitch (/ pi 4)))
  (setq *gp* (init-gazebo-plugin objname))
  (unix::usleep (* 1000 1000))
  (do-until-key
   (let* ((dest-pitch-angle dest-pitch)
	  (crnt-pitch-angle (elt (car (rpy-angle (send (send *gp* :get-pose) :rot))) 1))
	  (diff-pitch-angle (- dest-pitch-angle crnt-pitch-angle))
	  (crnt-dpitch-angle (elt (send *gp* :get-angular-vel (send *gp* :get-abs-vel)) 1))
	  (p-gain 4.0) ;; (p-gain 150)
	  (d-gain 4.0) ;; (d-gain 75)
	  (pitch-moment (+ (* p-gain diff-pitch-angle) (- (* d-gain crnt-dpitch-angle))))
	  (f (float-vector 0 0 0))
	  (m (float-vector 0 pitch-moment 0)))
     (send *gp* :add-force f m)
     (warn "angle: ~a  moment: ~a~%" (rad2deg crnt-pitch-angle) pitch-moment)
     (unix::usleep (* 100 1000))))
  )


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; utils

(defun rosvector32eusvector (vec)
  (let* ((x (send vec :x)) (y (send vec :y)) (z (send vec :z)))
    (float-vector x y z)))
(defun rosvector42eusvector (vec)
  (let* ((x (send vec :x)) (y (send vec :y)) (z (send vec :z)) (w (send vec :w)))
    (float-vector w x y z)))
