;#-:jsk  (jsk)
;#-:rbrain-basic (rbrain)

(require "atlas-model.l")
;(require "models/arrow-object.l")


;;;;;;; overwrite ;;;;;;;;

(defmethod cascaded-link
  (:cog-convergence-check
   (centroid-thre target-centroid-pos &optional centroid-offset-func)
   (let ((cdiff
	  (send self :difference-cog-position target-centroid-pos centroid-offset-func)))
     (cond
      ((numberp centroid-thre) (> centroid-thre (norm cdiff)))
      ((functionp centroid-thre) (funcall cdiff))
      ((vectorp centroid-thre) (v< (map float-vector #'abs cdiff) centroid-thre)))))
  )

(defmethod euscollada-robot
  (:fullbody-inverse-kinematics-with-error
   (target-coords
    &rest
    args
    &key
    (revert-if-fail t)
    link-list move-target
    rotation-axis translation-axis thre rthre
    target-centroid-pos centroid-thre centroid-offset-func
    &allow-other-keys)
   (let* ((initial-av (copy-object (send self :angle-vector)))
	  (inital-coords (copy-object (send self :worldcoords)))
	  (target-coords
	   (mapcar #'(lambda (x)
		       (if (functionp x) (funcall x) x))
		   target-coords))
	  dif-pos dif-rot success cdiff
	  )
     (if (atom (car link-list)) (setq link-list (list link-list)))
     (if (atom move-target) (setq move-target (list move-target)))
     (if (atom target-coords) (setq target-coords (list target-coords)))
     (if (atom rotation-axis) (setq rotation-axis (list rotation-axis)))
     (if (atom translation-axis) (setq translation-axis (list translation-axis)))
     (if (atom thre) (setq thre (list thre)))
     (if (atom rthre) (setq rthre (list rthre)))
     (send* self :fullbody-inverse-kinematics target-coords
	    :revert-if-fail nil args)
     (setq dif-pos
	   (mapcar #'(lambda (mv tc trans-axis)
		       (send mv :difference-position tc
			     :translation-axis trans-axis))
		   move-target target-coords translation-axis))
     (setq dif-rot
	   (mapcar #'(lambda (mv tc rot-axis)
		       (send mv :difference-rotation tc
			     :rotation-axis rot-axis))
		   move-target target-coords rotation-axis))
     (setq cdiff
	   (send self :difference-cog-position
		 target-centroid-pos centroid-offset-func))
     (setq success
	   (send self :ik-convergence-check
		 t dif-pos dif-rot
		 rotation-axis translation-axis thre rthre
		 centroid-thre target-centroid-pos centroid-offset-func))
     (cond
      ((and (not success) revert-if-fail)
       (send self :angle-vector initial-av)
       (send self :newcoords inital-coords)
       (list
	(cons :dif-pos dif-pos)
	(cons :dif-rot dif-rot)
	(cons :thre thre)
	(cons :rthre rthre)
	(cons :cog-dif cdiff)
	(cons :cthre centroid-thre))
       )
      (t (send self :angle-vector))))))

;;;;;;;; setup ;;;;;;;;;;;

(atlas-with-hand)
(setq *robot* *atlas*)

;; initial pose
(defun init-pose
  nil
  (send *robot* :reset-pose)
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
  )

(defun mirror-coords
  (coords
   &key
   (plane #F(0 1 0))
   (rot-filter
    (map float-vector
	 #'(lambda (n) (cond ((zerop n) -1) (t 1)))
	 plane))
   (pos-filter (scale -1 rot-filter))
   )
  (make-coords
   :pos
   (map float-vector #'*
	pos-filter (send coords :worldpos))
   :rpy
   (map cons #'* rot-filter
	(car (rpy-angle (send coords :worldrot))))))

;; hand end-coords
(defvar *right-hand-offset*
  (make-coords
   :rot
   (copy-object
    #2f((0.321394 -0.116978 -0.939693)
	(-0.883022 0.321394 -0.34202)
	(0.34202 0.939693 9.445569e-16)))))
(defvar *left-hand-offset*
  (mirror-coords *right-hand-offset*))

(defun hand-end-coords
  (&key
   (hand-key :rarm)
   (hand-offset
    (case hand-key
	  (:rarm *right-hand-offset*)
	  (:larm *left-hand-offset*))))
  (make-coords
   :pos
   (v+ (send hand-offset :worldpos)
       (send (send *robot* hand-key :end-coords :worldcoords)
	     :worldpos))
   :rot
   (m*
    (send (send *robot* hand-key :end-coords :worldcoords)
	  :worldrot)
    (send hand-offset :worldrot))))

(defun fix-centroid-object
  (&optional
   (centroid (send *robot* :centroid))
   (leg-h (scale 0.5
		 (v+
		  (send *robot* :rleg :end-coords :worldpos)
		  (send *robot* :lleg :end-coords :worldpos))
		 )))
  (send *centroid-object*
	:newcoords
	(make-coords
	 :pos (v+
	       #F(0 0 -200)
	       (concatenate float-vector
			    (subseq centroid 0 2)
			    (list (+ 10 (aref leg-h 2)))))))
  )

(defun fix-gun-coords
  nil
  (fix-centroid-object)
  (send *gun-coords*
 	:newcoords
	(copy-object
	 (send *robot* :rarm :end-coords :worldcoords)))
  )

(defmethod robot-model
  (:set-end-coords
   (&key
    (rhand-coords (hand-end-coords :hand-key :rarm))
    (lhand-coords (hand-end-coords :hand-key :larm)))
   (let* ((now-rhand (send self :rarm :end-coords))
	  (now-lhand (send self :larm :end-coords))
	  (rroot (send now-rhand :parent))
	  (lroot (send now-lhand :parent)))
     (send rroot :dissoc now-rhand)
     (send lroot :dissoc now-lhand)
     (setq
      rarm-end-coords
      (make-cascoords :init :link-list
		      :parent rroot
		      :coords rhand-coords
		      :name (send now-rhand :name)))
     (setq
      larm-end-coords
      (make-cascoords :init :link-list
		      :parent lroot
		      :coords lhand-coords
		      :name (send now-lhand :name)))
     (setq *right-hand-offset* (make-coords))
     (setq *left-hand-offset* (make-coords))
     (list rarm-end-coords larm-end-coords))))

(send *robot* :set-end-coords)

(setq *gun-coords*
      (let* ((root (make-sphere 50))
	     (lcoords
	      (make-cascoords
	       :init :link-list
	       :name :lcoords
	       :coords
	       (make-coords
		:pos #F(0 80 -100)
		:rpy '(0 0 0))
	       :parent root))
	     (rcoords
	      (make-cascoords
	       :init :link-list
	       :name :rcoords
	       :coords
	       (make-coords
		:pos #F(0 -100 80)
		:rpy '(0 0 -1.57))
	       :parent root)))
	(send root :set-color #F(1 0 0))
	root))
(init-pose)
(send *gun-coords* :translate #F(500 0 200))
(send *robot*
      :rarm
      :inverse-kinematics
      (send (car (send *gun-coords* :descendants))
	    :worldcoords)
      :link-list
      (send *robot* :link-list
	    (send *robot* :rarm :end-coords :parent))
      :debug-view :no-message)
(send *robot* :set-end-coords
      :rhand-coords
      (send *gun-coords* :worldcoords))
(send (send *robot* :rarm :end-coords)
      :assoc
      (cadr (send *gun-coords* :descendants)))
(send *sandia_hand_left* :grasp-pose)
(send *sandia_hand_right* :grasp-pose)
(init-pose)

(defvar *centroid-object* (make-cylinder 20 400))
(send *centroid-object* :set-color #F(1 1 0))

(defvar *triangle*
  (make-prism (list #F(0 0 0) #F(600 0 0) #F(0 600 0)) 5))
(send *triangle* :set-color #F(0 1 0))
(send *triangle* :rotate (deg2rad 90) :y)
(send *triangle* :translate #F(600 -200 600) :world)
(gl::transparent *triangle* 0.3)

(defvar *triangle-normal* #F(1 0 0))

;(if (not (and (boundp '*viewer*) *viewer*))
;    (pickview :no-menu t))
(objects
 (list *gun-coords* *robot*
       *centroid-object* *triangle*))

;;;;;;;; setup ;;;;;;;;;;;

;;;;;;;; controller ;;;;;;;;;

;(require "package://eus_nlopt/euslisp/nlopt-ik-overwrite.l")
;(require "package://eus_nlopt/euslisp/nlopt-ik.l")

(defun init-grasp-pose
  nil
  (send *robot* :angle-vector
	(float-vector 30.2942 14.5437 19.8382 83.7327 59.5451 23.0702 -72.3031 -87.9407 172.319 -99.3999 93.7892 50.9956 60.4444 -24.5719 94.5786 113.802 -7.9603 -80.5369 7.93411 -33.2496 -0.00524 114.234 5.71181 -80.8654 -5.738 -33.3534 -0.002034 0.0))
  (send *robot* :newcoords
	(make-coords
	 :pos (float-vector -219.358 -10.1211 -111.377)
	 :rot (copy-object
	       #2f((1.0 -5.742127e-05 -0.000283)
		   (5.729671e-05 1.0 -0.00044)
		   (0.000283 0.00044 1.0)))))
  (fix-gun-coords)
  )

(send-all (send *robot* :joint-list) :max-joint-velocity 1.0)
(init-grasp-pose)
(defvar *fix-leg-coords*
  (mapcar
   #'(lambda (k)
       (send *robot* k :end-coords
	     :copy-worldcoords))
   '(:rleg :lleg)))
(send *viewer* :draw-objects)

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
    (ik-main :move move)))

(defun spacenav-subscriber
  (msg)
  (cond
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
    (ik-main :move move :rpy rpy)))

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
  (setq *triangle-normal*
	(normalize-vector
	 (v* (v- a b)
	     (v- c b))))
  (setq *triangle*
	(make-prism
	 (list a b c)
	 *triangle-normal*))
  (send *triangle* :set-color #F(0 1 0))
  (gl::transparent *triangle* 0.3)
  ;;
  (objects
   (list *gun-coords* *robot*
	 *centroid-object* *triangle*))
  (mapcar
   #'(lambda (p) (demo-pos-controller :pos p))
   (triangle-pos-list))
  ))))

(defvar *success-state-collection* nil)
(defun ik-main
  (&key
   (move #F(0 0 0))
   (rpy '(0 0 0))
   (rotate? t)
   (translate? t)
   ;; meta
   (coords-org
    (copy-object (send *gun-coords* :worldcoords)))
   (prev-state
    (list
     (cons :angle-vector
	   (copy-object (send *robot* :angle-vector)))
     (cons :worldcoords
	   (copy-object (send *robot* :worldcoords)))))
   (move-target
    (mapcar
     #'(lambda (k)
	 (send *robot* k :end-coords))
     '(:rarm :larm :rleg :lleg)))
   (link-list
    (mapcar
     #'(lambda (k)
	 (send *robot* :link-list
	       (send *robot* k :end-coords :parent)
	       (if (find k nil);'(:rarm :larm))
		   (send *robot* k :root-link))))
     '(:rarm :larm :rleg :lleg)))
   (target-centroid-pos
    (scale
     0.5
     (apply #'v+
	    (mapcar
	     #'(lambda (k)
		 (send *robot* k :end-coords
		       :worldpos))
	     '(:rleg :lleg)))))
   (additional-weight-list
    (mapcar
     #'(lambda (k gain)
	 (list (send *robot* :torso k :child-link)
	       gain))
     (list :waist-r :waist-p :waist-y)
     (list 1e-3 1e-3 1e-3)))
   (centroid-thre
    (let ((margin 1.0)
	  (foot-convex
	   (apply
	    #'vmax
	    (mapcar
	     #'(lambda (f)
		 (map float-vector #'abs (apply #'v- (send f :vertices))))
	     (flatten
	      (send-all
	       (send (send *robot* :rleg :end-coords :parent)
		     :faces)
	       :edges)))))
	  (foot-dist
	   (map float-vector
		#'abs
		(apply #'v-
		       (mapcar
			#'(lambda (k)
			    (send *robot* k :end-coords
				  :worldpos))
			'(:rleg :lleg))))))
      (float-vector
       (* margin (/ (aref foot-convex 0) 2.0))
       (* margin
	  (+ (/ (aref foot-convex 1) 2.0)
	     (/ (aref foot-dist 1) 2.0))))))
   (debug-view nil)
   )
  (let (target-coords ret)
    (send *gun-coords* :newcoords
	  (make-coords
	   :pos
	   (v+ (send *gun-coords* :worldpos)
	       move)
	   :rpy
	   (map cons #'+
;		(car (send *gun-coords* :rpy-angle))
		(list
		 0 0
		 (caar (send *gun-coords* :rpy-angle)))
		rpy)))
    (setq target-coords
	  (append
	   (list (send *gun-coords* :worldcoords)
		 (car (send *robot* :rarm :end-coords
			    :descendants)))
	   *fix-leg-coords*))
    (setq
     ret
     (send *robot*
	   :fullbody-inverse-kinematics-with-error
	   target-coords
	   :move-target move-target
	   :link-list link-list
	   :rotation-axis (list rotate? t t t)
	   :translation-axis (list translate? t t t)
	   :target-centroid-pos target-centroid-pos
	   :cog-gain 0.1
	   :additional-weight-list additional-weight-list
	   :stop 10
	   :thre (make-list 4 :initial-element 5)
	   :rthre (make-list 4 :initial-element (deg2rad 3))
	   :centroid-thre centroid-thre
	   ;:root-link-virtual-joint-weight
	   ;#F(0.1 0.1 1 0.1 0.1 0.1)
	   ;:min #F(-1000 -1000 -1000 -200 -200 -200)
	   ;:max #F(1000 1000 1000 200 200 200)
	   :dump-command nil
	   :debug-view debug-view
	   :warnp t))
    ;;
    (fix-centroid-object)
    (cond
     ((vectorp ret)
      ;(fix-gun-coords)
      (send *robot* :head :look-at
	    (send *gun-coords* :worldpos))
      (send *viewer* :draw-objects)
      (if (or
	   (null *success-state-collection*)
	   (> (norm
	       (v- ret
		   (cdr (assoc :angle-vector prev-state))))
	      1))
	  (setq *success-state-collection*
		(subseq
		 (cons
		  (list (cons :angle-vector (copy-seq ret))
			(cons :worldcoords
			      (copy-object
			       (send *robot* :worldcoords))))
		  *success-state-collection*)
		 0 40)))
      )
     (t
      (format t "[ik-main] fail!!~%")
      (print ret)
      (send *gun-coords* :newcoords coords-org)))
    ret
    ))

(defun waist-fix
  (&key
   (waist-key :waist-r)
   (waist-joint (send *robot* :torso waist-key))
   (waist-angle (send waist-joint :joint-angle))
   (move-step 1)
   (cnt 10)
   (ret :start))
  (while (and ret
	      (> (abs waist-angle) move-step)
	      (plusp (decf cnt)))
    (setq waist-angle
	  (* (if (> waist-angle 0) 1 -1)
	     (- (abs waist-angle) move-step)))
    (send waist-joint :joint-angle waist-angle)
    (setq ret (ik-main :rotate? :x)))
  (send waist-joint :joint-angle waist-angle))

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
   (n *triangle-normal*))
  (let (ret
	move
	rpy
	(cnt 30))
    (do-until-key
     (if (< (decf cnt) 0) (return-from nil nil))
     (waist-fix :waist-key :waist-r :cnt 20)
     (waist-fix :waist-key :waist-p :cnt 20)
     (waist-fix :waist-key :waist-y :cnt 20)
     (setq move
	   (v- pos (send *gun-coords* :worldpos)))
     (if (< (norm move) 50) (return-from nil t))
     (setq move
	   (scale
	    30
	    (normalize-vector move)))
     (setq ret (ik-main :move move :rotate? :x))
     (cond
      ((vectorp ret))
      ((evenp (random 2))
       (setq ret (ik-main
		  ;:debug-view :no-message
		  :move
		  (float-vector
		   0
		   (- (random 2.0) 1.0)
		   (- (random 2.0) 1.0))
		  :rpy
		  (list 0 0 (- (random 0.2) 0.1)))))
      ;; (t
      ;;  (setq ret (ik-main
      ;; 		  :move
      ;; 		  (float-vector
      ;; 		   0
      ;; 		   (- (random 2.0) 1.0)
      ;; 		   (- (random 2.0) 1.0))
      ;; 		  :rotate? :x)))
      )
     (x::window-main-one))))

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

;(demo-mouse-controller)

#|

(defun demo-pos-controller
ppppp  (&key
   (pos (car (triangle-pos-list))))
  (let (ret
	move)
    (do-until-key
     (waist-fix :waist-key :waist-r :cnt 20)
     (waist-fix :waist-key :waist-p :cnt 20)
     (waist-fix :waist-key :waist-y :cnt 20)
     (setq move
	   (v- pos (send *gun-coords* :worldpos)))
     (if (< (norm move) 50) (return-from nil t))
     (setq move
	   (scale
	    30
	    (normalize-vector move)))
     (setq ret (ik-main :move move :rotate? :x))
     (if (listp ret) (return-from nil nil))
     (x::window-main-one))))


|#


(ros::rate 10)
(do-until-key
 (ros::spin-once)
 (ros::sleep)
 )
