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
      ((functionp centroid-thre) (funcall centroid-thre cdiff))
      ((vectorp centroid-thre)
       (v< (map float-vector #'abs cdiff) centroid-thre)))))
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

(defun rpy-from-2vec
  (target org)
  (let ((v
	 (scale
	  (acos (v. target org))
	  (normalize-vector (v* target org)))))
    (reverse
     (mapcar
      #'(lambda (n) (v. n v))
      (list #F(1 0 0) #F(0 1 0) #F(0 0 1))))))

(defun gen-foot-convex
  nil
  (let* ((margin 1.0)
	 (foot-center
	  (mapcar
	   #'(lambda (k)
	       (send
		(send *robot* k :end-coords :parent)
		:centroid))
	   '(:rleg :lleg)))
	 (foot-param
	  (mapcar
	   #'(lambda (k)
	       (apply
		#'vmax
		(mapcar
		 #'(lambda (f)
		     (map float-vector #'abs
			  (transform
			   (transpose
			    (send *robot* k :end-coords :worldrot))
			   (map float-vector
				#'*
				#F(1 1 0)
				(apply #'v- (send f :vertices))))))
		 (flatten
		  (send-all
		   (send (send *robot* k :end-coords :parent) :faces)
		   :edges)))))
	   '(:rleg :lleg)))
	 (foot-corner
	  (mapcar
	   #'(lambda (k center filter param)
	       (v+ center
		   (transform
		    (send *robot* k :end-coords :worldrot)
		    (scale 0.5
			   (map float-vector
				#'*
				filter param)))))
	   '(:rleg :rleg :lleg :lleg)
	   (list (car foot-center)(car foot-center)
		 (cadr foot-center) (cadr foot-center))
	   (list #F(2 -1 0) #F(-1 -1 0) #F(-1 1 0) #F(2 1 0))
	   (list (car foot-param) (car foot-param)
		 (cadr foot-param) (cadr foot-param))
	   ))
	 )
    foot-corner))

(defun check-centroid-with-convex
  (diff
   &optional
   (convex (convex-pos-list))
   (center
    (scale
     0.5
     (apply
      #'v+
      (mapcar
       #'(lambda (k)
	   (send *robot* k :end-coords :worldpos))
       '(:rleg :lleg))))))
  (let* ((diff (concatenate float-vector diff (list 0)))
	 (target (v+ diff center))
	 (t-vec (mapcar
		 #'(lambda (v) (v- target v))
		 convex))
	 (rotate
	  (mapcar
	   #'(lambda (v1 v2) (v* v1 v2))
	   t-vec (append (cdr t-vec) (list (car t-vec)))))
	 )
    (eq
     1
     (length
      (union
       nil
       (mapcar
	#'(lambda (v) (minusp (aref v 2)))
	rotate))))))

(defmethod robot-model
  (:set-end-coords
   (&key
    (rhand-coords (send self :rarm :end-coords :copy-worldcoords))
    (lhand-coords (send self :larm :end-coords :copy-worldcoords)))
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
		  :rpy '(0 0 0))
		 :parent root))
	       (rcoords
		(make-cascoords
		 :init :link-list
		 :name :rcoords
		 :coords
		 (make-coords
		  :pos #F(0 -100 10)
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
  (init-pose)))

(defvar *centroid-object* (make-cylinder 20 400))
(send *centroid-object* :set-color #F(1 1 0))

(defvar *mouse-mode* nil)

;; (defvar *triangle*
;;   (make-prism (list #F(0 0 0) #F(600 0 0) #F(0 600 0)) 5))
;; (send *triangle* :set-color #F(0 1 0))
;; (send *triangle* :rotate (deg2rad 90) :y)
;; (send *triangle* :translate #F(600 -200 600) :world)
;; (gl::transparent *triangle* 0.3)

(cond
 ((not (and (boundp '*triangle*) *triangle*))
  (setq *triangle*
    (make-prism (list #F(0 0 0) #F(600 0 0) #F(0 600 0)) 5))
  (send *triangle* :set-color #F(0 1 0))
  (send *triangle* :rotate (deg2rad 90) :y)
  (send *triangle* :translate #F(600 -200 650) :world)
;(send *triangle* :translate #F(100 0 0) :world)
  (gl::transparent *triangle* 0.3)
;  (load "triangle.l")
;  (send *triangle* :translate #F(-100 0 200) :world)
  ))

;;;;;;;; setup ;;;;;;;;;;;

;;;;;;;; controller ;;;;;;;;;

;(require "package://eus_nlopt/euslisp/nlopt-ik-overwrite.l")
;(require "package://eus_nlopt/euslisp/nlopt-ik.l")

(defun init-grasp-pose
  nil
  (send *robot* :angle-vector
	(float-vector 0.666105 0.453636 0.169232 89.9685 105.481 -2.2947 -81.9084 -11.3984 140.62 -71.4656 122.04 50.1474 82.4132 -21.7127 105.881 117.167 -10.602 -54.342 15.0 -54.8502 23.8069 116.511 11.6134 -53.0905 -16.4626 -55.8006 -32.5296 65.4853))
;	(float-vector 26.7629 4.57067 19.5455 80.5325 94.6333 -5.135 -59.8389 -54.5193 130.226 -80.9536 108.013 66.5278 72.6201 -31.1291 103.478 109.713 -19.3623 -70.8337 19.583 -41.9359 -0.451053 112.13 14.8006 -71.7101 -15.0 -43.3939 -2.2247 51.8388))
;	(float-vector 30.2914 14.5512 19.832 82.6847 57.502 16.9294 -75.707 -86.2673 162.803 -101.47 94.6749 50.8246 60.5665 -23.4032 94.9548 114.013 -7.16839 -80.8545 7.72476 -33.9763 0.646874 113.767 6.43725 -80.8649 -5.88093 -33.7164 0.452182 41.0989))
;	(float-vector 30.2942 14.5437 19.8382 83.7327 59.5451 23.0702 -72.3031 -87.9407 172.319 -99.3999 93.7892 50.9956 60.4444 -24.5719 94.5786 113.802 -7.9603 -80.5369 7.93411 -33.2496 -0.00524 114.234 5.71181 -80.8654 -5.738 -33.3534 -0.002034 0.0))
  (send *robot* :newcoords
	(make-coords
	 :pos (float-vector 1.773 -48.58 -124.121)
	 :rpy '(0.079 -0.154 -0.009)))
;	(make-coords
;	 :pos (float-vector -130.206 -122.709 -96.739)
;	 :rpy '(0.026 0.05 -0.003)))
;	(make-coords
;	 :pos (float-vector -213.994 -3.235 -108.462)
;	 :rpy '(-0.009 0.014 -0.01)))
;	(make-coords
;	 :pos (float-vector -219.358 -10.1211 -111.377)
;	 :rot (copy-object
;	       #2f((1.0 -5.742127e-05 -0.000283)
;		   (5.729671e-05 1.0 -0.00044)
;		   (0.000283 0.00044 1.0)))))
  (setq *fix-leg-coords*
	(mapcar
	 #'(lambda (k)
	     (send *robot* k :end-coords :copy-worldcoords))
	 '(:rleg :lleg)))
  (fix-gun-coords)
  )

(defun init-grasp-pose-single-hand
  nil
  (let ((buf
	 (send-all
	  (progn
	    (send *robot* :reset-manip-pose)
	    (send-all (send *robot* :larm :links) :joint))
	  :joint-angle)))
    (init-grasp-pose)
    (mapcar
     #'(lambda (j av) (send j :joint-angle av))
     (send-all (send *robot* :larm :links) :joint)
     buf))
  )

(send-all (send *robot* :joint-list) :max-joint-velocity 1.0)
;(send *robot* :lleg :crotch-r :min-angle -5)
;(send *robot* :lleg :crotch-r :joint-angle 15)
;(send *robot* :rleg :crotch-r :max-angle 5)
;(send *robot* :rleg :crotch-r :joint-angle -15)

(init-grasp-pose)

(cond
 ((not (and (boundp '*foot-convex*) *foot-convex*))
  (defvar *foot-convex*
    (make-prism (gen-foot-convex) 5))
  (send *foot-convex* :set-color #F(0 0 1))
  (gl::transparent *foot-convex* 0.3)))

(defvar *fix-leg-coords*
  (mapcar
   #'(lambda (k)
       (send *robot* k :end-coords
	     :copy-worldcoords))
   '(:rleg :lleg)))

(objects
 (list *gun-coords* *robot* *foot-convex*
       *centroid-object* *triangle*))

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
  (msg
   &optional
   (move-filter '(0 1 1))
   (rpy-filter '(0 0 1)))
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
	(setq *triangle*
	      (make-prism (list a b c)
			  (triangle-n (list a b c))))
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

(defun leg-coords-fix-from-real
  (&key
   (init-grasp? t)
   (solve-ik? t))
  (let* ((initial-av
	  (copy-object (send *robot* :angle-vector)))
	 (initial-coords
	  (copy-object (send *robot* :worldcoords))))
    (cond
     ((not (and (boundp '*ri*) *ri*))
      (require "atlas-interface.l")
      (atlas-init-ex)
      (objects
       (list *robot* *centroid-object*
	     *gun-coords* *triangle*))
      (send *ri* :start-auto-balancer)
      (set-user)
      ))
    (real2model)
    (send *robot* :fix-leg-to-coords (car *fix-leg-coords*) :rleg)
    (setq *fix-leg-coords*
	  (mapcar
	   #'(lambda (k)
	       (send *robot* k :end-coords :copy-worldcoords))
	   '(:rleg :lleg)))
    ;(send *robot* :angle-vector initial-av)
    ;(send *robot* :newcoords initial-coords)
    (if init-grasp? (init-grasp-pose))
    (if solve-ik? (ik-main :rotate? :x))
    (fix-gun-coords)
    (send *viewer* :draw-objects)
    ))

(send (send *robot* :utorso_lk) :make-pqpmodel :fat 30)
(defvar *success-state-collection* nil)
(defun ik-main
  (&key
   (move #F(0 0 0))
   (rpy '(0 0 0))
   (real? nil)
   (real-feedback? nil)
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
   (target-limb '(:rarm :larm :rleg :lleg))
   (left-support? t)
   (move-target
    (mapcar
     #'(lambda (k)
	 (send *robot* k :end-coords))
     target-limb))
   (link-list
    (mapcar
     #'(lambda (k)
	 (send *robot* :link-list
	       (send *robot* k :end-coords :parent)
	       (if (find k nil);'(:rarm :larm))
		   (send *robot* k :root-link))))
     target-limb))
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
     (list 1e-3 1e-3 1e-2)))
   ;; (centroid-thre
   ;;  (let ((margin 1.0)
   ;; 	  (foot-convex
   ;; 	   (apply
   ;; 	    #'vmax
   ;; 	    (mapcar
   ;; 	     #'(lambda (f)
   ;; 		 (map float-vector #'abs (apply #'v- (send f :vertices))))
   ;; 	     (flatten
   ;; 	      (send-all
   ;; 	       (send (send *robot* :rleg :end-coords :parent)
   ;; 		     :faces)
   ;; 	       :edges)))))
   ;; 	  (foot-dist
   ;; 	   (map float-vector
   ;; 		#'abs
   ;; 		(apply #'v-
   ;; 		       (mapcar
   ;; 			#'(lambda (k)
   ;; 			    (send *robot* k :end-coords
   ;; 				  :worldpos))
   ;; 			'(:rleg :lleg))))))
   ;;    (float-vector
   ;;     (* margin (/ (aref foot-convex 0) 2.0))
   ;;     (* margin
   ;; 	  (+ (/ (aref foot-convex 1) 2.0)
   ;; 	     (/ (aref foot-dist 1) 2.0))))))
   ;;
   (rotation-axis
    (mapcar
     #'(lambda (k)
	 (case k
	       (:rarm rotate?)
	       (:larm left-support?)
	       (t t)))
     target-limb))
   (translation-axis
    (mapcar
     #'(lambda (k)
	 (case k
	       (:rarm translate?)
	       (:larm left-support?)
	       (t t)))
     target-limb))
   ;;
   (initial-av (copy-seq (send *robot* :angle-vector)))
   (initial-coords (copy-object (send *robot* :worldcoords)))
   (debug-view :no-message)
   (stop 50)
   collision?
   (collision-pair
    (mapcar
     #'(lambda (l) (list l (send *robot* :utorso_lk)))
     (remove-if
      #'(lambda (l)
	  (or (eq (send l :parent)
		  (send *robot* :utorso_lk))
	      (eq (send (send l :parent) :parent)
		  (send *robot* :utorso_lk))))
      (apply
       #'append
       (mapcar
	#'(lambda (k) (send *robot* k :links))
	(remove-if
	 #'(lambda (k) (find k '(:rleg :lleg)))
	 target-limb))))))
   )
  (let (target-coords ret)
    (cond
     ((and real-feedback?
	   (boundp '*ri*) *ri*)
      (leg-coords-fix-from-real
       :init-grasp? nil
       :solve-ik? nil)))
    (send *gun-coords* :newcoords
	  (make-coords
	   :pos
	   (v+ (send *gun-coords* :worldpos)
	       move)
	   :rpy
	   (map cons #'+
		(car (send *gun-coords* :rpy-angle))
;		(list
;		 0 0
;		 (caar (send *gun-coords* :rpy-angle)))
		rpy)))
    (setq target-coords
	  (mapcar
	   #'(lambda (k)
	       (case k
		     (:rarm (send *gun-coords* :worldcoords))
		     (:larm (car (send *robot* :rarm :end-coords
				       :descendants)))
		     (:rleg (car *fix-leg-coords*))
		     (:lleg (cadr *fix-leg-coords*))))
	   target-limb))
    ;;
    (setq
     ret
     (send *robot*
	   :fullbody-inverse-kinematics-with-error
	   target-coords
	   :move-target move-target
	   :link-list link-list
	   :rotation-axis rotation-axis
	   :translation-axis translation-axis
	   :target-centroid-pos target-centroid-pos
	   :cog-gain 0.1
	   :additional-weight-list additional-weight-list
	   :stop stop
	   :thre (make-list 4 :initial-element 5)
	   :rthre (make-list 4 :initial-element (deg2rad 3))
	   :centroid-thre #'check-centroid-with-convex
	   ;;
	   :collision-avoidance-link-pair collision-pair
	   :root-link-virtual-joint-weight
	   #F(0.1 1.5 1 0.1 0.1 0.5)
	   ;:min #F(-1000 -1000 -1000 -200 -200 -200)
	   ;:max #F(1000 1000 1000 200 200 200)
	   :avoid-collision-distance 10
	   :avoid-collision-null-gain 0.3
	   :avoid-collision-joint-gain 0.3
	   ;;
	   :dump-command nil
	   :debug-view debug-view
	   :warnp nil))
    ;;
    (setq collision?
	  (send *robot* :self-collision-check :pairs
		(mapcar
		 #'(lambda (ll) (cons (car ll) (cadr ll)))
		 collision-pair)))
    (format t "[collision check] ~A~%" collision?)
    ;;
    (cond
     ((and (not collision?) (vectorp ret))
      (fix-centroid-object)
      (send *irtviewer* :draw-objects)
      (cond
       ((and real? (boundp '*ri*) *ri*)
	(let ((interpolate-time 1000))
	  (cond
	   ((> (norm (v- (send *ri* :state :potentio-vector)
			 (send *robot* :angle-vector))) 100)
	    (setq interpolate-time 4000)))
	  (model2real :time interpolate-time :wait t)
	  (unix::usleep (round (* 1000 1000)))
	  ;(read-line)
	  )))
      (send *gun-coords*
	    :newcoords
	    (make-coords
	     :pos
	     (copy-object
	      (send *gun-coords* :worldpos))
	     :rot
	     (copy-object
	      (send *robot* :rarm :end-coords :worldrot))))
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
		 0 500)))
      )
     (t
      (format t "[ik-main] fail!!~%")
      (send *robot* :angle-vector initial-av)
      (send *robot* :newcoords initial-coords)
      (send *gun-coords* :newcoords coords-org)
      (send *viewer* :draw-objects)
      (print ret)
      (setq ret nil)))
    ret
    ))

(defun waist-fix
  (&key
   (waist-key :waist-r)
   (waist-joint (send *robot* :torso waist-key))
   (waist-angle (send waist-joint :joint-angle))
   (prev-waist-angle waist-angle)
   (move-step 1)
   (cnt 10)
   (ret #F()))
  (while (and (vectorp ret)
	      (> (abs waist-angle) move-step)
	      (plusp (decf cnt)))
    (setq prev-waist-angle waist-angle)
    (setq waist-angle
	  (* (if (> waist-angle 0) 1 -1)
	     (- (abs waist-angle) move-step)))
    (send waist-joint :joint-angle waist-angle)
    (setq ret (ik-main :rotate? :x)))
  (if (not (vectorp ret))
      (send waist-joint :joint-angle prev-waist-angle)))

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
    (setq ret (ik-main :rotate? :x :debug-view :no-message)))
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
   )
  (let (ret
	move
	rpy
	buf
	(cnt 1000))
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
     (waist-fix :waist-key :waist-p :cnt 3)
     (waist-fix :waist-key :waist-y :cnt 3)
     (dotimes (i 2) (ik-main :rotate? :x))
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
     (setq ret (ik-main
		:real? real?
		:move move :rpy rpy
		:rotate? :x))
     (cond
      ((vectorp ret))
      (t ;(evenp (random 2))
       (setq ret (ik-main
		  :real? real?
		  :real-feedback? real-feedback?
		  :rotate? :x
		  :move
		  (progn
		    (setq buf (random-vector 50.0))
		    (v- buf
			(scale (v. buf (triangle-n))
			       (triangle-n))))
		  ;; :rpy
		  ;; (progn
		  ;;   (setq buf (random-vector 0.2))
		  ;;   (v-
		  ;;    buf
		  ;;    (scale (v. buf (reverse (triangle-n)))
		  ;; 	    (reverse (triangle-n)))))
		  )))
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

(defun convex-pos-list
  nil
  (union
   (send
    (find-if
     #'(lambda (f)
	 (find :bottom (send f :id)))
     (send *foot-convex* :faces))
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
    (objects
     (list *robot* *centroid-object*
	   *gun-coords* *triangle*))
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


;(demo-mouse-controller)

;;



#|

(init-grasp-pose)
(send (car *fix-leg-coords*) :translate #F(0 100 0) :world)
(ik-main)
(send (car *fix-leg-coords*) :translate #F(0 100 0) :world)

(mapcar
 #'(lambda (c deg)
     (send c :rotate (deg2rad deg) :z))
 *fix-leg-coords* '(-30 30))

(atlas-set-servo-gain-by-torque-limit 10 :torso)

(defun mm
  nil
  (require "atlas-interface.l")
  (cond
   ((not (and (boundp '*ri*) *ri*))
    (atlas-init-ex)
    (objects
     (list *robot* *centroid-object*
	   *gun-coords* *triangle*))))
  (stand-up)
  (real2model)
  (model2real :wait t)
  (send *ri* :start-auto-balancer)
  (set-user))

(ros::rate 10)
(do-until-key
 (ros::spin-once)
 (ros::sleep)
 )

(do-until-key
 (mapcar
  #'(lambda (p) (demo-pos-controller :pos p))
  (triangle-pos-list))
 )

(mm)
(init-grasp-pose)
(model2real :wait t)
 (mapcar
  #'(lambda (p) (demo-pos-controller :pos p :real? t :real-feedback? t))
  (triangle-pos-list))

;; procedure

rossetatlas
roseus atlas-dual-arm-ik.l

(require "atlas-interface.l")
(atlas-init-ex)
(objects
 (list *robot* *centroid-object*
       *gun-coords* *triangle*))

;; servo

(init-grasp-pose)
(model2real :time 10000)

(load "ssc-20131119.l")
(demo-collection-play)

|#
