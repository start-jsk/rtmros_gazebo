#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "move_base_msgs")

(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::load-ros-manifest "arm_navigation_msgs")
(ros::load-ros-manifest "moveit_msgs")
(ros::load-ros-manifest "jsk_interactive")
(ros::load-ros-manifest "jsk_interactive_marker")

(ros::roseus "atlas-ik-controller")

(require "atlas-model.l")
(require "atlas-fullbody-ik.l")
(require "package://hrpsys_gazebo_atlas/euslisp/atlas-joint-state-compresser-settings.l")
(require "models/arrow-object.l")
(require "package://roseus/euslisp/actionlib.l")

;;;;;;;; setup ;;;;;;;;;;;

(defun call-ik-server
  (&key
   (target-limb *move-limb*)
   (group_name
    (nth
     (cond
      (*use-torso* 1)
      (*use-fullbody* 2)
      (t 0))
     (case target-limb
       (:larm '("larm" "larm_torso" "larm_fullbody"))
       (:rarm '("rarm" "rarm_torso" "rarm_fullbody")))))
   (req (instance moveit_msgs::GetPositionIKRequest :init))
   (move-coords (make-coords))
    ;; (make-coords
    ;;  :pos (v- (send *target-coords* :worldpos)
    ;; 	      (send *origin-coords* :worldpos))
    ;;  :rot (m*
    ;; 	   (transpose
    ;; 	    (send *origin-coords* :worldrot))
    ;; 	   (send *target-coords* :worldrot))))
   (initial-coords
    (send *robot* target-limb :end-coords :copy-worldcoords))
   (target-coords
    (make-coords
     :pos (v+ (send initial-coords :worldpos)
	      (send move-coords :worldpos))
     :rot (m* (send initial-coords :worldrot)
	      (send move-coords :worldrot)
	      )))
   (target-coords-from-pelvis
    (make-coords
     :pos (v- (send target-coords :worldpos)
	      (send *robot* :worldpos))
     :rot (m* (transpose
	       (send *robot* :worldrot))
	      (send target-coords :worldrot))))
   ;;
   (prev-state
    (list (cons :angle-vector
		(copy-seq (send *robot* :angle-vector)))
	  (cons :worldcoords
		(copy-object (send *robot* :worldcoords)))))
   (log-stream t)
   &allow-other-keys
   )
  (let* ((c target-coords-from-pelvis)
	 (p (scale 1e-3 (send c :worldpos)))
	 (q (matrix2quaternion (send c :worldrot)))
	 res)
    (send req :ik_request :group_name group_name)
    ;;
    (send *prev-js*
	  :header
	  (instance std_msgs::Header :init
		    :stamp (ros::time 0.0)
		    :frame_id "/pelvis"))
    (send *prev-js*
	  :position
	  (map float-vector
	       #'deg2rad
	       (send *robot* :angle-vector)))
    (send req :ik_request :robot_state :joint_state *prev-js*)
    ;;
    (send req :ik_request
	  :pose_stamped
	  (instance geometry_msgs::PoseStamped :init
		    :header
		    (instance std_msgs::Header :init
			      :stamp (ros::time 0.0)
			      :frame_id "/pelvis")
		    :pose
		    (instance geometry_msgs::Pose :init
			      :position
			      (instance geometry_msgs::Point
					:init
					:x (elt p 0)
					:y (elt p 1)
					:z (elt p 2))
			      :orientation
			      (instance geometry_msgs::Quaternion
					:init
					:x (elt q 1)
					:y (elt q 2)
					:z (elt q 3)
					:w (elt q 0)))
		    ))
    (setq res (ros::service-call "/atlas_eus_ik" req))
    (if (= (send res :error_code :val)
	   moveit_msgs::MoveItErrorCodes::*SUCCESS*)
	(progn
	  (format log-stream "successed!~%")
	  (setq *prev-js* (send res :solution :joint_state))
	  (send *robot* :angle-vector
		(map float-vector
		     #'rad2deg
		     (send res :solution :joint_state :position)))
	  (send *robot* :fix-leg-to-coords
		(make-coords) :both)
	  (send *viewer* :draw-objects)
	  ;;
	  (if (or
	       (null *success-state-collection*)
	       (> (norm
		   (v- (send *robot* :angle-vector)
		       (cdr (assoc :angle-vector prev-state))))
		  1))
	      (setq *success-state-collection*
		    (subseq
		     (cons
		      (list (cons :angle-vector
				  (copy-object
				   (send *robot* :angle-vector)))
			    (cons :worldcoords
				  (copy-object
				   (send *robot* :worldcoords))))
		      *success-state-collection*)
		     0 500)))
	  (send *robot* :angle-vector)
	  )
      (format log-stream "failed!~%")
      )))

;; initial pose
(defun init-pose
  nil
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords) :both)
  (if (and (boundp '*viewer*) *viewer*)
      (send *viewer* :draw-objects))
  )

;;;;;;;; setup ;;;;;;;;;;;

;;;;;;;; controller ;;;;;;;;;

(defun str-subscriber
  (msg)
  (let ((command (send msg :data))
	(move))
    (if (zerop (length command))
	(setq command "hoge"))
    (setq move
	  (scale
	   5
	   (case (aref command 0)
	     (#\u #F(0 0 1))
	     (#\d #F(0 0 -1))
	     (#\r #F(0 -1 0 ))
	     (#\l #F(0 1 0))
	     (#\f #F(1 0 0))
	     (#\b #F(-1 0 0))
	     (#\v (read-from-string
		   (concatenate
		    string
		    "#f"
		    (subseq command 1))))
	     (t #F(0 0 0)))))
    (call-ik-server
     :move-coords
     (make-coords :pos move))))

(defun spacenav-subscriber
  (msg
   &optional
   (move-filter '(1 1 1))
   (rpy-filter '(1 1 1)))
  (cond
   ((and (> (aref (send msg :buttons) 0) 0.1)
	 (> (aref (send msg :buttons) 1) 0.1))
    (format t "mouse-mode~%")
    (format t "standby..?~%")
    (setq *mouse-mode* (not *mouse-mode*))
    (print 3) (unix:sleep 1)
    (print 2) (unix:sleep 1)
    (print 1) (unix:sleep 1)
    (format t "start!!~%")
    )
   ((not *mouse-mode*)
    (return-from spacenav-subscriber nil))
   ((> (aref (send msg :buttons) 1) 0.1)
    (print 'waist-fix)
    (waist-fix :waist-key :waist-r :cnt 3)
    (waist-fix :waist-key :waist-p :cnt 3)
    (waist-fix :waist-key :waist-y :cnt 3)
    (print 'done)
    )
   ((null *success-state-collection*) nil)
   ((> (aref (send msg :buttons) 0) 0.1)
    (print 'pop-back-robot-state)
    (let ((state (pop *success-state-collection*)))
      (send *robot* :angle-vector
	    (cdr (assoc :angle-vector state)))
      (send *robot* :newcoords
	    (cdr (assoc :worldcoords state)))
      (send *viewer* :draw-objects))
    ))
  (let ((move
	 (scale
	  30
	  (coerce
	   (subseq (send msg :axes) 0 3)
	   float-vector)))
	(rpy
	 (reverse
	  (scale
	   0.12
	   (coerce
	    (subseq (send msg :axes) 3 6)
	    float-vector)))))
    (cond
     ((< (norm
	  (concatenate
	   float-vector
	   (scale 1e-3 move)
	   rpy)) 0.01)
      (format t "skip ~A ~A~%" move rpy))
     (t
      (setq move
	    (map float-vector
		 #'*
		 move move-filter))
      (setq rpy
	    (map float-vector
		 #'*
		 rpy rpy-filter))
      (call-ik-server
       :move-coords
       (make-coords :pos move :rpy rpy))))))

(defun joint-state-subscriber
  (msg)
  (send *real-robot*
	:angle-vector
	(map float-vector
	     #'rad2deg
	     (send msg :position)))
  (send *real-robot*
	:fix-leg-to-coords (make-coords) :both)
  )

(defun ros::tf-point->tf-pose (point)
  (let ((pose (instance geometry_msgs::PoseStamped :init
                        :header (instance std_msgs::header :init
                                          :frame_id (send point :header :frame_id)
                                          :stamp (send point :header :stamp)))))
    (send pose :pose :position :x (send point :point :x))
    (send pose :pose :position :y (send point :point :y))
    (send pose :pose :position :z (send point :point :z))
    (send pose :pose :orientation :w 1.0)
    pose))

(defun waist-fix
  (&key
   (waist-key :waist-r)
   (waist-joint (send *robot* :torso waist-key))
   (waist-angle (send waist-joint :joint-angle))
   (angle-target 0)
   (prev-waist-angle waist-angle)
   (move-step 1)
   (cnt 10)
   (ret #F()))
  (while (and (vectorp ret)
	      (> (abs (- waist-angle angle-target))
		 move-step)
	      (plusp (decf cnt)))
    (x::window-main-one)
    (setq prev-waist-angle waist-angle)
    (setq waist-angle
	  (* (if (> waist-angle 0) 1 -1)
	     (- (abs waist-angle) move-step)))
    (send waist-joint :joint-angle waist-angle)
    (setq ret (call-ik-server :rotate? :x)))
  (if (not (vectorp ret))
      (send waist-joint :joint-angle prev-waist-angle)))

(defun hand-even-fix
  (&key
   (rpy-id 2)
   (rpy-target 0)
   (cnt 10)
   (rpy-step (deg2rad 3))
   )
  (let* (now-rpy
	 now-target
	 ret)
    (do-until-key
     (x::window-main-one)
     (setq now-rpy
	   (car (send *gun-coords* :rpy-angle)))
     (setq now-target (nth rpy-id now-rpy))
     (if (< (abs (- now-target rpy-target)) rpy-step)
	 (return-from nil nil))
     (if (< (decf cnt) 0) (return-from nil nil))
     (setq index -1)
     (setq
      ret
      (call-ik-server
       :move-coords
       (make-coords
	:rpy
	(mapcar
	 #'(lambda (rpy)
	     (cond
	      ((not (eq (incf index) rpy-id)) 0)
	      ((< (- rpy-target now-target) 0)
	       (* -1 rpy-step))
	      (t rpy-step)))
	 now-rpy))))
     (if (not ret) (return-from nil nil)))))

(defun base-fix
  (&key
   (base-id 0)
   (base-rpy
    (car (send *robot* :rpy-angle)))
   (prev-base-rpy base-rpy)
   (move-step
    (let ((index -1))
      (mapcar
       #'(lambda (hoge)
	   (if (eq (incf index) base-id)
	       (deg2rad 1) 0))
       (make-list 3))))
   (cnt 10)
   (ret #F())
   buf)
  (while (and (vectorp ret)
	      (> (abs (nth base-id base-rpy))
		 (nth base-id move-step))
	      (plusp (decf cnt)))
    (setq prev-base-rpy base-rpy)
    (setq base-rpy
	  (mapcar
	   #'(lambda (rpy step)
	       (* (if (> rpy 0) 1 -1)
		  (- (abs rpy) step)))
	   base-rpy move-step))
    (send *robot* :newcoords
	  (make-coords
	   :pos (copy-object (send *robot* :worldpos))
	   :rpy (copy-object base-rpy)))
    (send *viewer* :draw-objects)
    (read-line)
    (setq ret (call-ik-server
	       :rotate? :x :debug-view :no-message)))
  (if (not (vectorp ret))
      (send *robot* :newcoords
	    (make-coords
	     :pos (copy-object (send *robot* :worldpos))
	     :rpy (copy-object prev-base-rpy)))))

;;Interactive Marker Callback
(defun move-callback
  ( msg )
  (let ((menu (send msg :menu))
	(type (send msg :type)))
    (cond
     ((eq menu jsk_interactive_marker::MarkerMenu::*SET_ORIGIN*)
      (set-origin)
      )
     ;;reset target coords
     ((eq menu jsk_interactive_marker::MarkerMenu::*RESET_COORDS*)
      (send *target-coords* :move-to *old-coords* :world)
      (set-marker-pose-robot (send *target-coords* :worldcoords))
      )
     ;;set origin to hand
     ((eq menu jsk_interactive_marker::MarkerMenu::*SET_ORIGIN_RHAND*)
      (print "set origin to rhand")
      (set-origin-to-hand :rarm)
      )
     ((eq menu jsk_interactive_marker::MarkerMenu::*SET_ORIGIN_LHAND*)
      (print "set origin to lhand")
      (set-origin-to-hand :larm)
      )
     ;;change ik-rotation-axis
     ((eq menu jsk_interactive_marker::MarkerMenu::*IK_ROTATION_AXIS_T*)
      (print "set ik-rotation-axis t")
      (setq *ik-rotation-axis* t)
      )
     ((eq menu jsk_interactive_marker::MarkerMenu::*IK_ROTATION_AXIS_NIL*)
      (print "set ik-rotation-axis nil")
      (setq *ik-rotation-axis* nil)
      )
     ;;change use-torso
     ((eq menu jsk_interactive_marker::MarkerMenu::*USE_TORSO_T*)
      (print "use torso t")
      (setq *use-torso* t)
      (setq *use-fullbody* nil)
      )
     ((eq menu jsk_interactive_marker::MarkerMenu::*USE_TORSO_NIL*)
      (print "use torso nil")
      (setq *use-torso* nil)
      (setq *use-fullbody* nil)
      )
     ((eq menu jsk_interactive_marker::MarkerMenu::*USE_FULLBODY*)
      (print "use torso nil")
      (setq *use-fullbody* t)
      )
     ;;changeControllArm
     ((eq menu jsk_interactive_marker::MarkerMenu::*SET_MOVE_RARM*)
      (print "move rarm")
      (setq *move-limb* :rarm)
      )
     ((eq menu jsk_interactive_marker::MarkerMenu::*SET_MOVE_LARM*)
      (print "move larm")
      (setq *move-limb* :larm)
      )
     ((eq menu jsk_interactive_marker::MarkerMenu::*SET_MOVE_ARMS*)
      (print "move arms")
      (setq *move-limb* :arms)
      )
     ((eq menu jsk_interactive_marker::MarkerMenu::*PUBLISH_MARKER*)
      (publish-target-arrow-obj)
      (publish-origin-arrow-obj)
      )
     )
    (when (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects))
    )
  (publish-arrow-tf)
  )

;;called when marker is moved
(defun pose-callback
  (msg)
  (let* ((pose (send msg :pose))
	 (cds (ros::tf-pose-stamped->coords pose))
	 )
    (send *target-coords* :move-to
	  (send *robot* :worldcoords) :world)
    (send *target-coords* :transform cds)
    (when (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects))
    )
  (publish-arrow-tf)
  )

(defun set-origin
  ()
  (send *robot*
	:angle-vector
	(copy-object
	 (send *real-robot* :angle-vector)))
  (setq *old-coords* (send *target-coords* :copy-worldcoords))
  (send *origin-coords* :move-to *target-coords* :world)
  )

(defun set-origin-to-hand
  (&optional (arm :rarm))
  ;;(send *robot* :angle-vector
  ;;(send ri :state :potentio-vector))
  (send *target-coords* :move-to
	(send *robot* arm :end-coords :worldcoords) :world)
  (set-origin)
  (set-marker-pose-robot (send *target-coords* :worldcoords))
  )


;;fixed flamae is foot?
(defun get-base-coords
  ()
  (let (coords torso->map)
    (setq coords (send (send *robot* :r_foot_lk :worldcoords) :copy-worldcoords))
    (while t
      (setq torso->map
	    (send *tfl* :lookup-transform
		  "r_foot" "map" (ros::time 0)))
      (cond
       (torso->map
	(send coords :transform torso->map)
	(return)
	)
       (t
	(ros::ros-info "r_foot -> map cannnot transform yet")
	(ros::sleep)
	)
       )
      )
    coords
    )
  )

(defun set-marker-pose
  (coords &key (marker-name) (frame "/map") (markers) (server *server-nodename*))
  (let ((req (instance jsk_interactive_marker::MarkerSetPoseRequest :init))
	(msg (ros::coords->tf-pose-stamped coords frame)))
    (send req :marker_name marker-name)
    (send req :pose msg)
    (send req :markers markers)
    (ros::service-call (format nil "~A/set_pose" server)
		       req)
    ))

(defun set-marker-pose-robot
  (coords &rest args)
  (if args
      (apply #'set-marker-pose
	     (append
	      (list
	       (send (get-base-coords)
		     :transformation coords :local)
	       args)))
    (set-marker-pose
     (send (get-base-coords) :transformation coords :local))
    ))

(defun publish-arrow-tf
  ()
  (send *tfb* :send-transform
	(send (get-base-coords) :transformation
	      *target-coords* :local)
	"/map" "/im_target_coords_arrow")
  ;;
  (send *tfb* :send-transform
	(send (get-base-coords) :transformation
	      *origin-coords* :local)
	"/map" "/im_origin_coords_arrow")
  )

(defun publish-target-arrow-obj
  ()
  (let ((org  (send *target-coords* :copy-worldcoords)))
    (send *target-coords* :reset-coords)
    (send *target-coords* :worldcoords)
    (send-all (send *target-coords* :bodies) :worldcoords)
    (ros::publish (format nil "~A/marker" *server-nodename*)
		  (object->marker-msg *target-coords*
				      (instance std_msgs::header :init
						:frame_id "/im_target_coords_arrow")))
    (send *target-coords* :transform org)
    (send *target-coords* :worldcoords)
    )
  )

(defun publish-origin-arrow-obj
  ()
  (let ((org  (send *origin-coords* :copy-worldcoords)))
    (send *origin-coords* :reset-coords)
    (send *origin-coords* :worldcoords)
    (send-all (send *origin-coords* :bodies) :worldcoords)
    (ros::publish (format nil "~A/origin_marker" *server-nodename*)
		  (object->marker-msg *origin-coords*
				      (instance std_msgs::header :init
						:frame_id "/im_origin_coords_arrow")))
    (send *origin-coords* :transform org)
    (send *origin-coords* :worldcoords)
    )
  )

(defvar *target-coords* (instance arrow-object :init))
(send *target-coords* :translate #f(500 0 0)) ;; initial pose
(defvar *origin-coords* (instance arrow-object :init)) ;;arrow for origin of object
(send *origin-coords* :translate #f(500 0 0))
(defvar *old-coords* (send *target-coords* :copy-worldcoords))
(defvar *use-fullbody* t)
(defvar *use-torso* nil)

;;initialize joint-index-list
;;it is used to convert joint-name -> index
(joint-index-init)

(ros::roseus "ik_server")
(setq *tfb* (instance ros::transform-broadcaster :init))
(setq *tfl* (instance ros::transform-listener :init))
(ros::load-ros-manifest "hrpsys_gazebo_atlas")
;;
(ros::roseus-add-msgs "hrpsys_gazebo_atlas")
(ros::roseus-add-msgs "sensor_msgs")
;;
(ros::subscribe "/ik_server_command"
		std_msgs::String
		#'str-subscriber
		1)
(ros::subscribe "/spacenav/joy"
		sensor_msgs::Joy
		#'spacenav-subscriber
		1)

(defvar *server-nodename*
  "/jsk_interactive_marker_manipulation")
(ros::advertise (format nil "~A/marker" *server-nodename*)
		visualization_msgs::Marker)
(ros::advertise
 (format nil "~A/origin_marker" *server-nodename*)
 visualization_msgs::Marker)
(ros::subscribe
 (format nil "~A/pose" *server-nodename*)
 jsk_interactive_marker::MarkerPose #'pose-callback)
(ros::subscribe
 (format nil "~A/marker_menu" *server-nodename*)
 jsk_interactive_marker::MarkerMenu #'move-callback)

(defvar *im-nodename* "atlas_ik_visualizer")
;;Update Interactive Marker
(ros::advertise
 (format nil "~A/atlas/reset_joint_states" *im-nodename*)
 sensor_msgs::JointState)

(ros::subscribe
 "/atlas/joint_states_decompressed"
 sensor_msgs::JointState
 #'joint-state-subscriber)

(defun demo-mouse-controller
  nil
  (ros::rate 10)
  (do-until-key
   (if (not (ros::ok)) (return-from nil nil))
   (x::window-main-one)
   (ros::spin-once)
   (ros::sleep)))

(defun demo-pos-controller
  (&key
   (now
    (send *robot* *move-limb*
	  :end-coords :copy-worldcoords))
   (goal (copy-object now))
   (now-from-pelvis
    (make-coords
     :pos
     (v- (send now :worldpos) (send *robot* :worldpos))
     :rot
     (m* (transpose (send *robot* :worldrot))
   	 (send now :worldrot))))
   (goal-from-pelvis
    (make-coords
     :pos
     (v- (send goal :worldpos) (send *robot* :worldpos))
     :rot
     (m* (transpose (send *robot* :worldrot))
   	 (send goal :worldrot))))
   ;; (goal-from-pelvis
   ;;  (make-coords
   ;;   :pos
   ;;   (v- (send now :worldpos) (send *robot* :worldpos))
   ;;   :rot
   ;;   (m* (transpose (send *robot* :worldrot))
   ;; 	 (send now :worldrot))))
   ;; (goal
   ;;  (make-coords
   ;;   :pos
   ;;   (v+ (send goal-from-pelvis :worldpos)
   ;; 	 (send *robot* :worldpos))
   ;;   :rot
   ;;   (m* (send *robot* :worldrot)
   ;; 	 (send goal-from-pelvis :worldrot))))
   (real? nil)
   (real-feedback? nil)
   (cnt 1000)
   (non-stop nil)
   (return? nil)
   )
  (send *arrow* :newcoords (Copy-Object goal))
  (if return?
      (setq return?
	    (length *success-state-collection*)))
  (let (ret move rot buf tmp)
    (do-until-key
     (if (< (decf cnt) 0) (return-from nil nil))
     (ros::spin-once)
     (while *mouse-mode*
       (x::window-main-one)
       (ros::sleep)
       (ros::spin-once))
     ;;
     (waist-fix :waist-key :waist-r :cnt 3)
					;(waist-fix :waist-key :waist-p :cnt 3)
					;(waist-fix :waist-key :waist-y :cnt 3)
					;(hand-even-fix :cnt 1)
					;(dotimes (i 2)
					;(call-ik-server :rotate? :x :log-stream nil))
     (setq
      now
      (send *robot* *move-limb*
	    :end-coords :copy-worldcoords))
     (setq
      now-from-pelvis
      (make-coords
       :pos
       (v- (send now :worldpos) (send *robot* :worldpos))
       :rot
       (m* (transpose (send *robot* :worldrot))
	   (send now :worldrot))))
     (setq
      goal-from-pelvis
      (make-coords
       :pos
       (v- (send goal :worldpos) (send *robot* :worldpos))
       :rot
       (m* (transpose (send *robot* :worldrot))
	   (send goal :worldrot))))
     (setq
      move
      (transform
       (send now-from-pelvis :worldrot)
       (send now-from-pelvis :difference-position
	     goal-from-pelvis)))
     (setq
      rot
      (transform
       (send now-from-pelvis :worldrot)
       (send now-from-pelvis :difference-rotation
	     goal-from-pelvis)))
     (format t "[pos-controller] :dif-pos ~A, :dif-rot ~A~%"
	     (norm move) (norm rot))
     (if (and (not non-stop)
	      (< (norm move) 10)
	      (< (norm rot) (deg2rad 5)))
	 (return-from nil t))
     (setq move
	   (scale
	    (min 50 (norm move))
	    (normalize-vector move)))
     (setq rot (scale (min 0.15 (norm rot))
		      (normalize-vector rot)))
     (setq tmp (call-ik-server
		:real? real?
		:move-coords
		(make-coords
		 :pos move
		 :rpy (reverse rot))
		:rotate? :x
		:log-stream nil
		))
     (cond
      ((vectorp tmp))
      (t ;(evenp (random 2))
       (setq tmp (call-ik-server
		  :real? real?
		  :real-feedback? real-feedback?
					;:rotate? :x
		  :move-coords
		  (make-coords
		   :pos (random-vector 50.0)
		   :rpy (random-vector 0.2))
		  :log-stream nil
		  ))))
     (x::window-main-one)))
  (if return?
      (subseq *success-state-collection*
	      0
	      (- (length *success-state-collection*)
		 return?))
    t)
  )

(defun demo-pos-controller2
  (&key
   (now
    (send *robot* *move-limb*
	  :end-coords :copy-worldcoords))
   (goal (copy-object now))
   (now-from-pelvis
    (make-coords
     :pos
     (v- (send now :worldpos) (send *robot* :worldpos))
     :rot
     (m* (transpose (send *robot* :worldrot))
   	 (send now :worldrot))))
   (goal-from-pelvis
    (make-coords
     :pos
     (v- (send goal :worldpos) (send *robot* :worldpos))
     :rot
     (m* (transpose (send *robot* :worldrot))
   	 (send goal :worldrot))))
   ;; (goal-from-pelvis
   ;;  (make-coords
   ;;   :pos
   ;;   (v- (send now :worldpos) (send *robot* :worldpos))
   ;;   :rot
   ;;   (m* (transpose (send *robot* :worldrot))
   ;; 	 (send now :worldrot))))
   ;; (goal
   ;;  (make-coords
   ;;   :pos
   ;;   (v+ (send goal-from-pelvis :worldpos)
   ;; 	 (send *robot* :worldpos))
   ;;   :rot
   ;;   (m* (send *robot* :worldrot)
   ;; 	 (send goal-from-pelvis :worldrot))))
   (real? nil)
   (real-feedback? nil)
   (cnt 1000)
   (non-stop t)
   (return? nil)
   )
  (send *arrow* :newcoords (Copy-Object goal))
  (if return?
      (setq return?
	    (length *success-state-collection*)))
  (let (ret move rot buf tmp str)
    (do-until-key
     (if (and (not non-stop)
	      (< (decf cnt) 0)) (return-from nil nil))
     (ros::spin-once)
     (while *mouse-mode*
       (x::window-main-one)
       (ros::sleep)
       (ros::spin-once))
     ;;
     (waist-fix :waist-key :waist-r :cnt 3)
					;(waist-fix :waist-key :waist-p :cnt 3)
					;(waist-fix :waist-key :waist-y :cnt 3)
					;(hand-even-fix :cnt 1)
					;(dotimes (i 2)
					;(call-ik-server :rotate? :x :log-stream nil))
     (setq
      now
      (send *robot* *move-limb*
	    :end-coords :copy-worldcoords))
     (setq
      str
      (send *real-robot* *move-limb*
	    :end-coords :copy-worldcoords))
     (setq
      goal
      (make-coords
       :pos
       (v+
	(send str :worldpos)
	(v- (send *target-coords* :worldpos)
	    (send *origin-coords* :worldpos)))
       :rot
       (m*
	(transpose (send str :worldrot))
	(m*
	 (transpose (send *target-coords* :worldrot))
	 (send *origin-coords* :worldrot)))))
     (setq
      now-from-pelvis
      (make-coords
       :pos
       (v- (send now :worldpos) (send *robot* :worldpos))
       :rot
       (m* (transpose (send *robot* :worldrot))
	   (send now :worldrot))))
     (setq
      goal-from-pelvis
      (make-coords
       :pos
       (v- (send goal :worldpos) (send *robot* :worldpos))
       :rot
       (m* (transpose (send *robot* :worldrot))
	   (send goal :worldrot))))
     (setq
      move
      (transform
       (send now-from-pelvis :worldrot)
       (send now-from-pelvis :difference-position
	     goal-from-pelvis)))
     (setq
      rot
      (transform
       (send now-from-pelvis :worldrot)
       (send now-from-pelvis :difference-rotation
	     goal-from-pelvis)))
     (format t "[pos-controller] :dif-pos ~A, :dif-rot ~A~%"
	     (norm move) (norm rot))
     (if (and (not non-stop)
	      (< (norm move) 10)
	      (< (norm rot) (deg2rad 5)))
	 (return-from nil t))
     (setq move
	   (scale
	    (min 50 (norm move))
	    (normalize-vector move)))
     (setq rot (scale (min 0.15 (norm rot))
		      (normalize-vector rot)))
     (setq tmp (call-ik-server
		:real? real?
		:move-coords
		(make-coords
		 :pos move
		 :rpy (reverse rot))
		:rotate? :x
		:log-stream nil
		))
     (cond
      ((vectorp tmp))
      (t ;(evenp (random 2))
       (setq tmp (call-ik-server
		  :real? real?
		  :real-feedback? real-feedback?
					;:rotate? :x
		  :move-coords
		  (make-coords
		   :pos (random-vector 50.0)
		   :rpy (random-vector 0.2))
		  :log-stream nil
		  ))))
     (x::window-main-one)))
  (if return?
      (subseq *success-state-collection*
	      0
	      (- (length *success-state-collection*)
		 return?))
    t)
  )


;; export ROS_MASTER_URI=http://10.10.72.112:11311
;; export ROS_IP=10.10.72.109 && export ROS_HOSTNAME=$ROS_IP
;; load "ssc-20131118.l"
;; load "ssc-20131119.l"
;; load "ssc-20131119-3.l"
(defun demo-collection-play
  (&key
   (real? nil)
   (data (reverse *success-state-collection*))
   (inital-sleep (if real? 10000 100))
   (cnt -1)
   d)
  ;; (cond
  ;;  (real?
  ;;   (init-grasp-pose)
  ;;   (model2real :sleep-time inital-sleep :wait t)))
  ;;
  (do-until-key
   (print (incf cnt))
   (if (null data) (return-from nil t))
   (setq d (car data))
   (setq data (cdr data))
   (send *robot* :angle-vector (cdr (assoc :angle-vector d)))
   (send *robot* :newcoords (cdr (assoc :worldcoords d)))
   (send *viewer* :draw-objects)
   (x::window-main-one)
   (cond
    (real?
     (model2real :sleep-time inital-sleep :wait t))
    (t (unix:usleep (round (* inital-sleep 1e+3)))))
   (setq inital-sleep 100))
  )

(atlas-with-hand)
(setq *robot* *atlas*)
(setq *real-robot* (copy-object *robot*))
(setq *arrow* (arrow))
(init-pose)
					;(objects (list *robot* *arrow*))
(objects
 (list *robot* *origin-coords* *target-coords*))
;;
(send *irtviewer* :change-background #f(0.9 0.9 1.0))
(send *irtviewer* :title "Interactive IK")
(send *irtviewer* :look-all)

(send-all (send *robot* :joint-list) :max-joint-velocity 0.3)

(defvar *mouse-mode* nil)
(defvar *success-state-collection* nil)
(defvar *move-limb* :larm)

(defvar *prev-js*
  (instance sensor_msgs::jointstate
	    :init
	    :header
	    (instance std_msgs::Header :init
		      :stamp (ros::time 0.0)
		      :frame_id "/pelvis")
	    :position
	    (map float-vector
		 #'deg2rad
		 (send *robot* :angle-vector))
	    :name
	    (send-all (send *robot* :joint-list) :name)))
					;
(send *robot* :rleg :knee-p :min-angle 10)
(send *robot* :lleg :knee-p :min-angle 10)

;(demo-mouse-controller)
(send *robot* :angle-vector
      (copy-object (send *real-robot* :angle-vector)))
(send *real-robot*
      :fix-leg-to-coords (make-coords) :both)
(demo-pos-controller2)

#|

(ros::subscribe
 (format nil "~A/pose" server-nodename)
 jsk_interactive_marker::MarkerPose #'send self :pose-callback)

(ros::subscribe
 (format nil "~A/marker_menu" server-nodename)
 jsk_interactive_marker::MarkerMenu #'send self :move-callback)
