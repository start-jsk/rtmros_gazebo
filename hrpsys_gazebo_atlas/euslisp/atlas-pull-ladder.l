(require "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
(require "package://hrpsys_gazebo_atlas/euslisp/atlas-fullbody-ik.l")
(require "package://hrpsys_gazebo_atlas/models/drc_ladder.l")

(setq *ladder* (drc-ladder))
(atlas-init-ex :real nil)
(setq *robot* *atlas*)

(send *ladder* :rotate (deg2rad -90) :z)
(send *ladder* :translate #F(1180 0 0) :world)

(defmethod robot-model
  (:calc-static-balance-point
   (forces
    &optional (force-points
               (mapcar #'(lambda (tmp-arm)
                           (send (send self tmp-arm :end-coords) :worldpos))
                       '(:rarm :larm)))
    &key (ref-cog-height (aref (apply #'midpoint 0.5 (send self :legs :end-coords :worldpos)) 2)))
   (let* ((sbp (float-vector 0 0 ref-cog-height))
          (cog (send self :centroid))
          (mg (* 1e-6 (send self :weight nil) (elt *g-vec* 2))))
     (dotimes (idx 2)
       (let ((denom mg)
             (nume (* mg (elt cog idx))))
         (mapcar #'(lambda (f fp)
                     (setq nume
                           (+ nume
                              (- (* (- (elt fp 2) ref-cog-height) (elt f idx))
                                 (* (elt fp idx) (elt f 2))))
                           denom (- denom (elt f 2))))
                 forces force-points)
         (setf (elt sbp idx) (/ nume denom))
         ))
     sbp)))

(setq *end-coords-collection* (make-hash-table))
(defun my-end-coords
  (key)
  (let (buf)
    (cond
     ((send *end-coords-collection* :get key)
      (send *end-coords-collection* :get key))
     ((find key '(:rarm :larm))
      (send *end-coords-collection* :enter key
	    (make-cascoords
	     :init :link-list
	     :name
	     (read-from-string
	      (format nil "~A-my-end-coords" key))
	     :parent (send *robot* key :end-coords :parent)
	     :coords
	     (make-coords
	      :pos
	      (copy-object
	       (send
		(send *robot* key :end-coords :parent)
		:worldpos))
	      ;; :rot
	      ;; (copy-object
	      ;;  (send
	      ;; 	(send *robot* key :end-coords :parent)
	      ;; 	:worldrot))
	      ))))
     (t (send *end-coords-collection* :enter key
	      (send *robot* key :end-coords))))))

(defun make-foot-convex
  (&key
   (target-limbs '(:rleg :lleg)))
  (setq *foot-convex*
	(make-prism
	 (quickhull
	  (mapcar
	   #'(lambda (v)
	       (concatenate float-vector
			    (subseq v 0 2)
			    (list
			     (+
			      -10
			      (apply
			       #'min
			       (mapcar
				#'(lambda (k)
				    (aref
				     (send *robot* k :end-coords :worldpos)
				     2))
				(remove-if
				 #'(lambda (k) (not (find k '(:rleg :lleg))))
				 target-limbs)))))))
	   (append
	    (mapcar
	     #'(lambda (hk)
		 (copy-object (send (my-end-coords hk) :worldpos)))
	     (remove-if
	      #'(lambda (k) (not (find k '(:rarm :larm))))
	      target-limbs))
	    (flatten
	     (mapcar
	      #'(lambda (k)
		  (send-all
		   (send
		    (send (my-end-coords k) :parent)
		    :faces)
		   :vertices))
	      (remove-if
	       #'(lambda (k) (not (find k '(:rleg :lleg))))
	       target-limbs)
	      )))))
	 5))
  (send *foot-convex* :set-color #F(0 0 1))
  (gl::transparent *foot-convex* 0.3))

(defvar *right-force* #f(0 0 0))
(defvar *left-force* #F(0 0 0))
(defun fix-centroid-object
  (&optional (centroid;; (send *robot* :centroid)) 
	      (send *robot* :calc-static-balance-point 
		    (list *right-force* *left-force*)))
	     (leg-h
	      (scale
	       0.5
	       (v+
		(send *robot* :rleg :end-coords :worldpos)
		(send *robot* :lleg :end-coords :worldpos)))))
  (if
      (and (boundp '*centroid-object*) *centroid-object*)
      (send
       *centroid-object*
       :newcoords
       (make-coords
	:pos
	(v+
	 #f(0.0 0.0 -200.0)
	 (concatenate
	  float-vector
	  (subseq centroid 0 2)
	  (list (+ 10 (aref leg-h 2)))))))))

(defun update-objects
  (&key
   (target-limbs '(:rleg :lleg)))
  (if (not (and (boundp '*centroid-object*) *centroid-object*))
      (make-centroid-object))
  (fix-centroid-object)
  (make-foot-convex :target-limbs target-limbs)
  (cond
   ((not (and (boundp '*viewer*) *viewer*))
    (objects
     (list *robot* *ladder* *centroid-object* *foot-convex*))
    (send *irtviewer* :change-background #f(1 1 1)))
   (t
    (send *irtviewer* :change-background #f(1 1 1))
    (send *irtviewer* :objects
	  (list *robot* *ladder* *centroid-object* *foot-convex*))))
  (send *viewer* :draw-objects))

(defvar *angle-vector-collection*)
(defun m2r
  nil
  (push (copy-object (send *robot* :angle-vector)) *angle-vector-collection*)
  (model2real :sleep-time 5000))

(defun r2m
  (&key
   (support-leg '(:rleg :lleg))); :lleg)))
  (real2model)
  (ros::spin-once) (ros::sleep)
  (ros::spin-once) (ros::sleep)
  (ros::spin-once) (ros::sleep)
  ;;(send *robot* :newcoords (send *odom* :copy-worldcoords))
  ;;(send *robot* :rotate (deg2rad 180) :z)
  ;; (mapcar
  ;;  #'(lambda (k)
  ;;      (send *robot* k :inverse-kinematics
  ;; 	     (make-coords)
  ;; 	     :translation-axis t
  ;; 	     :debug-view :no-message))
  ;;  support-leg)
  (send *robot* :fix-leg-to-coords (make-coords) (car support-leg))
  (update-objects))

(defun init-pose
  nil
  (send *robot* :angle-vector
	(copy-object #f(-0.012063 24.2414 -0.014197 58.5405 71.9975 42.8905 14.5838 -60.0031 92.9974 -58.5362 72.0057 -42.8788 -14.5638 -60.0247 92.9999 56.9033 0.004696 -61.7731 -0.008555 -10.3528 -0.000607 56.9067 0.004696 -61.7755 -0.008555 -10.3538 -0.000607 0.0)))
  (send *robot* :fix-leg-to-coords (make-coords))
  (update-objects))

(defun lift-up-lleg
  (&key
   (balance-leg :rleg)
   (rarm #F(0 0 0))
   (larm #F(0 0 0))
   (rleg #F(0 0 0))
   (lleg #F(0 0 0))
   (torso nil)
   (centroid-offset #F(0 0 0))
   (centroid-offset-func
    '(lambda ()
       (x::window-main-one)
       (send *robot*
	     :calc-static-balance-point
	     (list *right-force* *left-force*)))))
  (send *robot*
	:fullbody-inverse-kinematics
	(append
	 (list
	  (send (send *robot* :rarm :end-coords :copy-worldcoords)
		:translate rarm)
	  (send (send *robot* :larm :end-coords :copy-worldcoords)
		:translate larm)
	  (send (send *robot* :rleg :end-coords :copy-worldcoords)
		:translate rleg)
	  (send (send *robot* :lleg :end-coords :copy-worldcoords)
		:translate lleg))
	 (if torso
	     (list (send (send *robot* :torso :end-coords :copy-worldcoords)
			 :translate torso)))
	 )
	:rotation-axis (append '(:y :y t t)
			       (if torso (list nil)))
	:centroid-offset-func centroid-offset-func
	:target-centroid-pos
	(v+
	 centroid-offset
	 (case balance-leg
	       (:both
		(scale 0.5
		       (v+ (send *robot* :rleg :end-coords :worldpos)
			   (send *robot* :lleg :end-coords :worldpos))))
	       (t (copy-object (send *robot* balance-leg :end-coords :worldpos)))))
	:move-target
	(mapcar
	 #'(lambda (k) (send *robot* k :end-coords))
	 (append '(:rarm :larm :rleg :lleg)
		 (if torso (list :torso))))
	:centroid-thre 40
	:rthre
	(append
	 (list (deg2rad 10) (deg2rad 10) (deg2rad 5) (deg2rad 5))
	 (if torso (list (deg2rad 5))))
	:thre
	(append
	 (list 30 30 5 5)
	 (if torso (list 30)))
	:link-list
	(mapcar
	 #'(lambda (k)
	     (send *robot* :link-list (send *robot* k :end-coords :parent)))
	 (append '(:rarm :larm :rleg :lleg)
		 (if torso (list :torso))))
	:debug-view :no-message)
  (update-objects :target-limbs
		  (case balance-leg
			(:both '(:rleg :lleg))
			(t (list balance-leg))))
  (send *robot* :angle-vector)
  )

(defun leg-orbit
  (&key
   (time-list (list 0 0.7 1.0))
   (move-key :lleg)
   (fix-key (case move-key (:rleg :lleg) (:lleg :rleg)))
   (start-coords
    (copy-object (send *robot* move-key :end-coords :worldcoords)))
   (end-coords
    (let* ((move (send *robot* move-key :end-coords :worldcoords))
	   (fix (send *robot* fix-key :end-coords :worldcoords)))
      (make-coords
       :pos
       (v+ (map float-vector #'* #F(0 1 0) (send move :worldpos))
	   (map float-vector #'* #F(1 0 1) (send fix :worldpos)))
       :rot (copy-object (send fix :worldrot)))))
   (margin 100)
   (p-list
    (list
     (send start-coords :worldpos)
     (let ((s (send start-coords :worldpos))
	   (e (send end-coords :worldpos)))
       (concatenate
	float-vector
	(subseq (v+ (scale 0.3 s) (scale 0.7 e)) 0 2)
	(list (+ (aref (vmax s e) 2) margin))))
     (v+ (float-vector 0 0 30)
	 (send end-coords :worldpos))
     ))
   (dp-list
    (list #F(0 0 0) nil #F(0 0 -1)))
   (ddp-list
    (list #F(0 0 0) nil #F(0 0 0)))
   (time-step 0.05)
   (spline-coeff-func 'solve-spline)
   (debug? t)
   (n nil))
  (require "euslib/demo/s-noda/util/spline.l")
  (let*
      ((coeff
	(apply
	 spline-coeff-func
	 (append
	  (list :time-list time-list)
	  (list
	   :p
	   (mapcar
	    (function
	     (lambda (v)
	       (cond
		((vectorp v) (list (cons :vector v))))))
	    p-list))
	  (list
	   :dp
	   (mapcar
	    (function
	     (lambda (v)
	       (cond
		((vectorp v) (list (cons :vector v))))))
	    dp-list))
	  (list
	   :ddp
	   (mapcar
	    (function
	     (lambda (v)
	       (cond
		((vectorp v) (list (cons :vector v))))))
	    ddp-list))
	  (if n (list :n n))
	  (list
	   :d
	   (length
	    (find-if
	     #'vectorp
	     (append p-list dp-list ddp-list)))))))
       (timer (car time-list))
       (end-time (car (last time-list)))
       ret
       sphere
       obj)
    (while
	(<= timer end-time)
      (push (calc-spline coeff timer) ret)
      (setq timer (+ timer time-step)))
    (cond
     (debug?
      (dolist (pos ret)
	(setq sphere (make-sphere 10))
	(send sphere :set-color #F(0 1 0))
	(send sphere :newcoords (make-coords :pos pos))
	(push sphere obj))
      (send *irtviewer* :objects
	    (append
	     (list *ladder* *robot*)
	     ;(send *irtviewer* :objects)
	     (flatten obj)))
      (send *viewer* :draw-objects)
      ))
    (reverse ret)))

(defun simple-robot-state
  (&optional tag)
  (list (cons :tag tag)
	(cons :angle-vector
	      (copy-object (send *robot* :angle-vector)))
	(cons :worldcoords
	      (copy-object (send *robot* :worldcoords)))))

(defun track-orbit
  (&key
   (move-key :lleg)
   (fix-key (case move-key (:rleg :lleg) (:lleg :rleg)))
   (target-limb '(:rleg :lleg))
   (orbit (leg-orbit :move-key move-key))
   (centroid-offset #F(0 0 0))
   ret pos buf
   )
  (do-until-key
   (setq pos (car orbit))
   (setq orbit (cdr orbit))
   (if (null orbit) (return-from nil (reverse ret)))
   (setq
    buf
    (lift-up-lleg
     :centroid-offset centroid-offset
     :balance-leg fix-key
     move-key (v- pos (send *robot* move-key :end-coords :worldpos))))
   (if (not (vectorp buf)) (return-from nil nil))
   (push (simple-robot-state :track-orbit) ret))
  ret)

(defun simple-rsd-play
  (rsd
   &key
   (time-list
    (list (cons :init 500)
	  (cons :grasp 500)
	  (cons :balance 500)
	  (cons :lean 500)
	  (cons :track-orbit 50)
	  (cons :land 50)
	  (cons :default 100))))
  (mapcar
   #'(lambda (rsd)
       (send *robot* :angle-vector
	     (cdr (assoc :angle-vector rsd)))
       (send *robot* :newcoords
	     (cdr (assoc :worldcoords rsd)))
       (send *viewer* :draw-objects)
       (x::window-main-one)
       (unix:usleep
	(round (* 1000
		  (or (cdr (assoc
			    (cdr (assoc :tag rsd))
			    time-list))
		      (cdr (assoc
			    :default
			    time-list)))))))
   (reverse rsd)))

(send *robot* :rleg :crotch-r :min-angle -11)
(send *robot* :rleg :crotch-r :max-angle 5)
(send *robot* :lleg :crotch-r :min-angle -5)
(send *robot* :lleg :crotch-r :max-angle 11)

(init-pose)
(send *irtviewer* :look-all)

(setq *right-force* #F(10 0 0))
(setq *left-force* #F(10 0 0))

(lift-up-lleg :balance-leg :lleg)
(lift-up-lleg :balance-leg :lleg :rleg #F(0 0 100))
(lift-up-lleg :balance-leg :lleg :rleg #F(0 0 100))
(lift-up-lleg :balance-leg :lleg :rleg #F(0 0 100))
(lift-up-lleg :balance-leg :lleg :rleg #F(100 0 0))
(lift-up-lleg :balance-leg :lleg :rleg #F(100 0 0))
(lift-up-lleg :balance-leg :lleg :rleg #F(0 0 -20))

(setq *right-force* #F(100 0 0))
(setq *left-force* #F(100 0 0))

(lift-up-lleg :balance-leg :rleg :centroid-offset #F(50 0 0))
(lift-up-lleg :balance-leg :rleg :lleg #F(0 0 100) :centroid-offset #F(50 0 0))
(lift-up-lleg :balance-leg :rleg :lleg #F(0 0 100) :centroid-offset #F(50 0 0))
(lift-up-lleg :balance-leg :rleg :lleg #F(0 0 100) :centroid-offset #F(50 0 0))
(lift-up-lleg :balance-leg :rleg :lleg #F(100 0 0) :centroid-offset #F(50 0 0))
(lift-up-lleg :balance-leg :rleg :lleg #F(100 0 0) :centroid-offset #F(50 0 0))
(lift-up-lleg :balance-leg :rleg :lleg #F(0 0 -20) :centroid-offset #F(50 0 0))

(track-orbit :centroid-offset #F(50 0 0))
(lift-up-lleg :balance-leg :rleg :lleg #F(0 0 -30) :centroid-offset #F(50 0 0))

;; (atlas-init-ex :set-reset-pose nil)

#|

(send *robot*
      :fullbody-inverse-kinematics
      (list
       (send (send *robot* :rarm :end-coords :copy-worldcoords)
	     :translate #f(-50 0 0))
       (send (send *robot* :larm :end-coords :copy-worldcoords)
	     :translate #f(-50 0 0))
       (send (send *robot* :rleg :end-coords :copy-worldcoords)
	     :translate #f(0 0 0))
       (send (send *robot* :lleg :end-coords :copy-worldcoords)
	     :translate #f(0 0 0))
       )
      :centroid-thre 30
      :target-centroid-pos
      (scale
       0.5
       (v+
	(send *robot* :rleg :end-coords :worldpos)
	(send *robot* :lleg :end-coords :worldpos)))
      :move-target
      (mapcar
       #'(lambda (k) (send *robot* k :end-coords))
       '(:rarm :larm :rleg :lleg))
      :link-list
      (mapcar
       #'(lambda (k)
	   (send *robot* :link-list (send *robot* k :end-coords :parent)))
       '(:rarm :larm :rleg :lleg))
      :stop 100
      :debug-view :no-message))

