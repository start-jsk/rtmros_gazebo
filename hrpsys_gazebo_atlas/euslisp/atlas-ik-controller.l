#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "move_base_msgs")

(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::load-ros-manifest "arm_navigation_msgs")
(ros::load-ros-manifest "moveit_msgs")

(ros::roseus "atlas-ik-controller")

(require "atlas-model.l")
(require "atlas-fullbody-ik.l")
(require "package://hrpsys_gazebo_atlas/euslisp/atlas-joint-state-compresser-settings.l")

;;;;;;;; setup ;;;;;;;;;;;

(defun call-ik-server
  (&key
   (target-limb :larm)
   (group_name
    (caddr '("larm" "larm_torso" "larm_fullbody"
	     "rarm" "rarm_torso" "rarm_fullbody")))
   (req (instance moveit_msgs::GetPositionIKRequest :init))
   (move-coords (make-coords))
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
   &allow-other-keys
   )
  (print target-coords-from-pelvis)
  (let* ((c target-coords-from-pelvis)
	 (p (scale 1e-3 (send c :worldpos)))
	 (q (matrix2quaternion (send c :worldrot)))
	 res)
    (send req :ik_request :group_name group_name)
    (print (send req :ik_request :group_name))
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
		    :header (instance std_msgs::Header :init
				      :stamp (ros::time 0.0)
				      :frame_id "/pelvis")
		    :pose
		    (instance geometry_msgs::Pose :init
			      :position
			      (instance geometry_msgs::Point :init
					:x (elt p 0)
					:y (elt p 1)
					:z (elt p 2))
			      :orientation
			      (instance geometry_msgs::Quaternion :init
					:x (elt q 1)
					:y (elt q 2)
					:z (elt q 3)
					:w (elt q 0)))
		    ))
    (setq res (ros::service-call "/atlas_eus_ik" req))
    (if (= (send res :error_code :val)
	   moveit_msgs::MoveItErrorCodes::*SUCCESS*)
	(progn
	  (print "successed!")
	  (setq *prev-js* (send res :solution :joint_state))
	  (send *robot* :angle-vector
		(map float-vector
		     #'rad2deg
		     (send res :solution :joint_state :position)))
	  (send *viewer* :draw-objects)
	  (send *robot* :angle-vector)
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
	  )
      (print "failed!")
      )))

;; initial pose
(defun init-pose
  (&optional
   (av (float-vector 0.0 0.0 0.0 30.0 90.0 0.0 -80.0 15.0 0.0 -30.0 90.0 0.0 80.0 15.0 0.0 30.0 0.0 -20.0 0.0 -10.0 0.0 30.0 0.0 -20.0 0.0 -10.0 0.0 0.0)))
  (send *robot* :angle-vector av)
  (send *robot* :newcoords (make-coords))
  (send *robot* :rleg
	:inverse-kinematics
	(make-coords :pos #F(0 -150 -700))
	:link-list
	(send *robot* :link-list
	      (send *robot* :rleg :end-coords :parent))
	:debug-view nil)
  (send *robot* :lleg
	:inverse-kinematics
	(make-coords :pos #F(0 +150 -700))
	:link-list
	(send *robot* :link-list
	      (send *robot* :lleg :end-coords :parent))
	:debug-view nil)
  (if (and (boundp '*viewer*) *viewer*)
      (send *viewer* :draw-objects))
  (update-objects)
  )

;;;;;;;; setup ;;;;;;;;;;;

(defun update-interactive-marker
  ()
  (let ((joint-angles nil)
	(joint-names nil)
	(joint-list (send *atlas* :joint-list))
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
		  joint-state-msg)
    )
  )

(defun publish-js-compressed
  ()
  (let ((js-compressed (instance hrpsys_gazebo_atlas::JointStateCompressed :init))
	joint-indexes joint-angles
	(joint-list (send *atlas* :joint-list))
	)
    (send js-compressed :header (instance std_msgs::header :init :stamp (ros::time-now)))
    ;;
    (dolist (j joint-list)
      (push (get-joint-index (send j :name)) joint-indexes)
      (push (send j :joint-angle) joint-angles)
      )
    (send js-compressed :index (coerce joint-indexes string))
    (send js-compressed :position (coerce joint-angles float-vector))
    ;;
    (ros::publish *fc-eus-nodename* js-compressed)
    (ros::ros-info "joint states compressed was published~%")
    )
  )

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
    (call-ik-server :move-coords
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
   ((null *success-state-collection*) nil)
   ((> (aref (send msg :buttons) 0) 0.1)
    (print 'pop-back-robot-state)
    (let ((state (pop *success-state-collection*)))
      (send *robot* :angle-vector
	    (cdr (assoc :angle-vector state)))
      (send *robot* :newcoords
	    (cdr (assoc :worldcoords state)))
      (fix-gun-coords)
      (send *viewer* :draw-objects))
    )
   ((> (aref (send msg :buttons) 1) 0.1)
    (print 'waist-fix)
    (waist-fix :waist-key :waist-r :cnt 3)
    (waist-fix :waist-key :waist-p :cnt 3)
    (waist-fix :waist-key :waist-y :cnt 3)
    (print 'done)
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
	   0.15
	   (coerce
	    (subseq (send msg :axes) 3 6)
	    float-vector)))))
    (setq move
	  (map float-vector
	       #'*
	       move move-filter))
    (setq rpy
	  (map float-vector
	       #'*
	       rpy rpy-filter))
    (call-ik-server :move-coords
		    (make-coords :pos move :rpy rpy))))

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

(defun trianglepoints-subscriber
  (msg)
  (let ((apose (ros::tf-point->tf-pose (send msg :a)))
        (bpose (ros::tf-point->tf-pose (send msg :b)))
        (cpose (ros::tf-point->tf-pose (send msg :c))))
    (let ((acoords (send *tf* :transform-pose "/pelvis" apose))
          (bcoords (send *tf* :transform-pose "/pelvis" bpose))
          (ccoords (send *tf* :transform-pose "/pelvis" cpose)))
      (let ((a (send acoords :worldpos))
            (b (send bcoords :worldpos))
            (c (send ccoords :worldpos)))
	(setq *triangle*
	      (make-prism (list a b c)
			  (triangle-n (list a b c))))
	(send *triangle* :set-color #F(0 1 0))
	(gl::transparent *triangle* 0.3)
	;;
	(my-objects)
	(mapcar
	 #'(lambda (p) (demo-pos-controller :pos p))
	 (triangle-pos-list))
	))))

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

(ros::roseus "ik_server")
(setq *tf* (instance ros::transform-listener :init))
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
(ros::subscribe "/trianglepoints"
		hrpsys_gazebo_atlas::trianglepoints
		#'trianglepoints-subscriber
		1)


(setq *im-nodename* "atlas_ik_visualizer")
(setq *fc-eus-nodename* "atlas_ik_fc_eus")
;;Update Interactive Marker
(ros::advertise (format nil "~A/atlas/reset_joint_states" *im-nodename*)
		sensor_msgs::JointState)

(ros::advertise *fc-eus-nodename* hrpsys_gazebo_atlas::JointStateCompressed 1)

;;initialize joint-index-list
;;it is used to convert joint-name -> index
(joint-index-init)


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
   (pos (car (triangle-pos-list)))
   (n (triangle-n))
   (real? nil)
   (real-feedback? nil)
   (cnt 1000)
   (return? nil)
   )
  (if return?
      (setq return?
	    (length *success-state-collection*)))
  (let (ret move rpy buf tmp)
    (do-until-key
     (if (< (decf cnt) 0) (return-from nil nil))
     (ros::spin-once)
     (while *mouse-mode*
       (x::window-main-one)
       (ros::sleep)
       (ros::spin-once))
     ;;
     (cond
      ((and real-feedback?
	    (boundp '*ri*) *ri*)
       (leg-coords-fix-from-real
	:init-grasp? nil)))
     ;;
     (waist-fix :waist-key :waist-r :cnt 3)
     ;(waist-fix :waist-key :waist-p :cnt 3)
     ;(waist-fix :waist-key :waist-y :cnt 3)
     ;(hand-even-fix :cnt 1)
     (dotimes (i 2) (call-ik-server :rotate? :x))
     (setq move
	   (v- pos (send *gun-coords* :worldpos)))
     (setq rpy
	   (coerce
	    (rpy-from-2vec
	     (transform (send *gun-coords* :worldrot)
			#F(1 0 0))
	     n)
	    float-vector))
     (setq rpy
	   (v- rpy
	       (scale (v. rpy
			  (reverse (triangle-n)))
		      (reverse (triangle-n)))))
     (if (and (< (norm move) 50)
	      (< (norm rpy) (deg2rad 180)))
	 (return-from nil t))
     (setq move
	   (scale
	    (min 30 (norm move))
	    (normalize-vector move)))
     (setq rpy (scale (min 0.15 (norm rpy))
		      (normalize-vector rpy)))
     (setq tmp (call-ik-server
		:real? real?
		:move-coords
		(make-coords
		 :move move :rpy rpy)
		:rotate? :x))
     (cond
      ((vectorp tmp))
      (t ;(evenp (random 2))
       (setq tmp (call-ik-server
		  :real? real?
		  :real-feedback? real-feedback?
		  ;:rotate? :x
		  :move-coords
		  (make-coords
		   :move
		   (progn
		     (setq buf (random-vector 50.0))
		     (v- buf
			 (scale (v. buf (triangle-n))
				(triangle-n))))
		   :rpy
		   (progn
		     (setq buf (random-vector 0.2))
		     (v-
		      buf
		      (scale (v. buf (reverse (triangle-n)))
			     (reverse (triangle-n))))))
		  )))
      ;; (t
      ;;  (setq ret (fullbody-ik-main
      ;; 		  :move
      ;; 		  (float-vector
      ;; 		   0
      ;; 		   (- (random 2.0) 1.0)
      ;; 		   (- (random 2.0) 1.0))
      ;; 		  :rotate? :x)))
      )
     (x::window-main-one)))
  (if return?
      (subseq *success-state-collection*
	      0
	      (- (length *success-state-collection*)
		 return?))
    t)
  )

(defun triangle-pos-list
  nil
  (union
   (send
    (find-if
     #'(lambda (f)
	 (find :bottom (send f :id)))
     (send *triangle* :faces))
    :vertices)
   nil))

(defun triangle-n
  (&optional
   (vlist
    (union
     (send
      (find-if
       #'(lambda (f)
	   (find :bottom (send f :id)))
       (send *triangle* :faces))
      :vertices)
     nil)))
  (let* ((c (scale (/ 1.0 (length vlist))
		   (reduce #'v+ vlist)))
	 (ret (normalize-vector
	       (v* (v- (nth 0 vlist) (nth 1 vlist))
		   (v- (nth 2 vlist) (nth 1 vlist))))))
    (scale
     (if (> (v. ret (v- c (send *robot* :worldpos))) 0)
	 1 -1)
     ret)))

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
  ;;
  (cond
   ((and real? (not (and (boundp '*ri*) *ri*)))
    (require "atlas-interface.l")
    (atlas-init-ex)
    (my-objects)
    (send *ri* :start-auto-balancer)
    (set-user)
    ))
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
   (fix-gun-coords)
   (send *viewer* :draw-objects)
   (x::window-main-one)
   (cond
    (real?
     (model2real :sleep-time inital-sleep :wait t))
    (t (unix:usleep (round (* inital-sleep 1e+3)))))
   (setq inital-sleep 100))
  )

(defun triangle-trial
  (&optional
   (coords
    (copy-object (send *triangle* :worldcoords))))
  (send *triangle* :newcoords coords)
  (init-pose)
  (bench2
   (mapcar
    #'(lambda (p) (demo-pos-controller :pos p))
    (reverse (triangle-pos-list)))))

(atlas-with-hand)
(setq *robot* *atlas*)

(cond
 ((not (and (boundp '*gun-coords*) *gun-coords*))
  (setq *gun-coords*
	(let* ((root (make-sphere 50))
	       (lcoords
		(make-cascoords
		 :init :link-list
		 :name :lcoords
		 :coords
		 (make-coords
		  :pos #F(0 10 -100)
		  ;:pos #F(0 100 10)
		  :rpy '(0 0 0))
		  ;:rpy '(0 0 1.57))
		 :parent root))
	       (rcoords
		(make-cascoords
		 :init :link-list
		 :name :rcoords
		 :coords
		 (make-coords
		  :pos #F(-100 0 0)
		  :rpy '(0 1.57 0))
		  ;:pos #F(0 -100 10)
		  ;:rpy '(0 0 -1.57))
		 :parent root)))
	  ;(send root :translate #F(700 -300 200))
	  (send *robot*
		:newcoords
		(make-coords
		 :rot
		 (m*
		  (send rcoords :worldrot)
		  (transpose
		   (send *robot* :rarm
			 :end-coords :worldrot))
		  )))
	  (send *robot* :translate
		(v- (send rcoords :worldpos)
		    (send *robot* :rarm :end-coords :worldpos))
		:world)
	  ;; (send *robot*
	  ;; 	:fullbody-inverse-kinematics
	  ;; 	(list (send rcoords :worldcoords))
	  ;; 	:link-list
	  ;; 	(list
	  ;; 	 (send *robot* :link-list
	  ;; 	       (send *robot* :rarm :end-coords :parent)))
	  ;; 	:move-target
	  ;; 	(list (send *robot* :rarm :end-coords))
	  ;; 	:target-centroid-pos nil
	  ;; 	:debug-view :no-message)
	  (send root :set-color #F(1 0 0))
	  root))
  (send *robot* :set-end-coords
  	:rhand-coords
  	(send *gun-coords* :worldcoords))
  (send (send *robot* :rarm :end-coords)
  	:assoc
  	(cadr (send *gun-coords* :descendants)))
  (send *sandia_hand_left* :grasp-pose)
  (send *sandia_hand_right* :grasp-pose)
  ))

(send-all (send *robot* :joint-list) :max-joint-velocity 0.3)
;(send *robot* :lleg :crotch-r :min-angle -5)
;(send *robot* :lleg :crotch-r :joint-angle 15)
;(send *robot* :rleg :crotch-r :max-angle 5)
;(send *robot* :rleg :crotch-r :joint-angle -15)

(defvar *mouse-mode* nil)

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

;; (defvar *triangle*
;;   (make-prism (list #F(0 0 0) #F(600 0 0) #F(0 600 0)) 5))
;; (send *triangle* :set-color #F(0 1 0))
;; (send *triangle* :rotate (deg2rad 90) :y)
;; (send *triangle* :translate #F(600 -200 600) :world)
;; (gl::transparent *triangle* 0.3)

(cond
 ((not (and (boundp '*triangle*) *triangle*))
  (setq *triangle*
	(make-prism (list #F(0 0 0) #F(300 0 0) #F(0 600 0)) 5))
  (send *triangle* :set-color #F(0 1 0))
  (send *triangle* :rotate (deg2rad -90) :y)
  (send *triangle* :translate #F(600 -500 0) :world)
;(send *triangle* :translate #F(100 0 0) :world)
  (gl::transparent *triangle* 0.3)
;  (load "triangle.l")
;  (send *triangle* :translate #F(-100 0 200) :world)
  ))

(if (not (and (boundp '*centroid-object*) *centroid-object*))
    (init-fullbody-ik-main))

;(init-grasp-pose)
(init-pose)
(my-objects)
;(demo-mouse-controller)

(send *robot* :rleg :knee-p :min-angle 10)
(send *robot* :lleg :knee-p :min-angle 10)



