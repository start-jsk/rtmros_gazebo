(require "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
(require "package://hrpsys_gazebo_atlas/models/drc_ladder.l")
(require "euslib/demo/s-noda/util/spline.l")

(setq *ladder* (drc-ladder))
(atlas-init-ex :real nil)
(setq *robot* *atlas*)

(send *ladder* :rotate (deg2rad -90) :z)
(send *ladder* :translate #F(1180 0 0) :world)

(defun init-pose
  (&rest args)
  (send *robot* :reset-manip-pose)
  (send *robot* :newcoords (make-coords))
  (send *robot* :larm :shoulder-r :joint-angle 90)
  (send *robot* :rarm :shoulder-r :joint-angle -90)
  ;;
  (send *robot* :rarm :elbow-p :joint-angle 0)
  (send *robot* :larm :elbow-p :joint-angle 0)
  ;;
  (send *robot* :rleg :crotch-y :joint-angle -45)
  (send *robot* :lleg :crotch-y :joint-angle 45)
  ;;
  (send *robot* :torso :waist-y :joint-angle -90)
  ;;
  (send *robot* :lleg :inverse-kinematics
  	(send
  	 (send *robot* :lleg :end-coords :copy-worldcoords)
  	 :translate #F(-150 0 0)
  	 :world)
  	:move-target (send *robot* :lleg :end-coords))
  ;; (send *robot* :lleg :inverse-kinematics
  ;; 	(send
  ;; 	 (send *robot* :lleg :end-coords :copy-worldcoords)
  ;; 	 :translate #F(0 50 0)
  ;; 	 :world)
  ;; 	:move-target (send *robot* :lleg :end-coords))
  ;;
  (send *robot* :head :look-at #F(0 0 1000))
  ;;
  (send *sandia_hand_left* :grasp-pose)
  (send *sandia_hand_right* :grasp-pose)
  (send *robot* :newcoords
	(make-coords
	 :rpy (list (deg2rad (+ 135 0)) 0 0)
	 :pos
	 (float-vector
	  -50 -50
	  (+
	   0
	   (aref
	    (v- (send *robot* :worldpos)
		(send *robot* :rleg :end-coords :worldpos))
	    2)))))
  (send *viewer* :draw-objects)
  *robot*
  )

(init-pose)
(objects (list *ladder* *robot*))

(setq *foot-print*
      (let ((now (float-vector 0 0 0))
	    (step (float-vector (/ 305 (tan (deg2rad 60))) 0 305))
	    (cnt 9))
	(mapcar
	 #'(lambda (hoge)
	     (setq now (v+ now step))
	     (decf cnt)
	     (mapcar
	      #'(lambda (k)
		  (make-coords
		   :pos
		   (v+
		    (if (zerop cnt)
			#F(150 0 0) #F(0 0 0))
		    (v+ now
			(map float-vector
			     #'*
			     #F(0 1.0 0)
			     (send *robot* k :end-coords :worldpos))))
		   :rot
		   (copy-object (send *robot* k :end-coords :worldrot))))
	      '(:rleg :lleg)))
	 (make-list cnt))))
(send-all (flatten *foot-print*) :draw-on :flush t)

(defun static-leg-move
  (&key
   (move-key :rleg)
   (fix-key (case move-key
		  (:rleg :lleg)
		  (:lleg :rleg)))
   (target-limb '(:rleg :lleg))
   (move #F(0 0 0))
   )
  (case move-key
  	(:rleg
  	 (send *robot* :rarm :elbow-p :joint-angle 0)
  	 (send *robot* :larm :elbow-p :joint-angle 0);123)
  	 )
  	(:lleg
  	 (send *robot* :rarm :elbow-p :joint-angle 0);-123)
  	 (send *robot* :larm :elbow-p :joint-angle 0)
  	 ))
  (let* ((mt (mapcar
	      #'(lambda (k) (send *robot* k :end-coords))
	      target-limb))
	 (ll (mapcar
	      #'(lambda (m) (send *robot* :link-list (send m :parent)))
	      mt))
	 buf)
    (send *robot* :fullbody-inverse-kinematics
	  (mapcar
	   #'(lambda (mt k)
	       (setq buf (send mt :copy-worldcoords))
	       (cond
		((eq k move-key)
		 (send buf :translate move :world))
		(t t))
	       buf)
	   mt target-limb)
	  :move-target mt
	  :link-list ll
	  :debug-view :no-message
	  :min #F(-1000 -1000 -1000 -200 -200 -200)
	  :max #F(1000 1000 1000 200 200 200)
	  ;:translation-axis (list nil nil t t)
	  ;:rotation-axis (list nil nil t t)
	  :thre (make-list (length target-limb)
			   :initial-element 3)
	  :rthre (make-list (length target-limb)
			    :initial-element (deg2rad 3))
	  :centroid-offset-func
	  #'(lambda nil
	      (x::window-main-one)
	      (send *robot* :centroid))
	  :target-centroid-pos
	  (v+ (case move-key
		    (:rleg #F(50 0 0))
		    (:lleg #F(0 0 0))
		    (:both #F(0 0 0)))
	      (if (eq move-key :both)
		  (scale 0.5
			 (v+
			  (send *robot* :rleg :end-coords :worldpos)
			  (send *robot* :lleg :end-coords :worldpos)))
		(send *robot* fix-key :end-coords :worldpos)))
	  ;; :obstacles (list (send *ladder* :link_lk))
	  ;; :collision-avoidance-link-pair
	  ;; (apply
	  ;;  #'append
	  ;;  (mapcar
	  ;;   #'(lambda (k)
	  ;; 	(mapcar
	  ;; 	 #'(lambda (l)
	  ;; 	     (list l (send *ladder* :link_lk)))
	  ;; 	 (send *robot* k :links)))
	  ;;   '(:rarm :larm)))
	  :root-link-virtual-joint-weight
	  #f(0.1 0.1 0.1 0.1 0.5 0.0)
	  :stop 500
	  )
    ))

(defun nearest-land-coords
  (&key
   (move-key :rleg)
   (now-c (send *robot* move-key :end-coords :copy-worldcoords))
   )
  (sort
   (mapcar
    (case move-key
	  (:rleg #'car)
	  (:lleg #'cadr))
    *foot-print*)
   #'(lambda (a b)
       (< (norm2
	   (concatenate
	    float-vector
	    (scale 1e-3 (send a :difference-position now-c))
	    (send a :difference-rotation now-c)))
	  (norm2
	   (concatenate
	    float-vector
	    (scale 1e-3 (send b :difference-position now-c))
	    (send b :difference-rotation now-c)))))))

(defun on-ladder
  (&key
   (move-key :rleg)
   (now-c (send *robot* move-key :end-coords :copy-worldcoords))
   (target-c
    (nearest-land-coords
     :move-key move-key
     :now-c now-c)))
  (static-leg-move
   :move-key move-key
   :move (v+
	  (case move-key
		(:rleg #F(30 0 0))
		(:lleg #F(-10 0 0)))
	  (v- (send (car target-c) :worldpos)
	      (send now-c :worldpos))))
  )

(send *robot* :rleg :knee-p :min-angle 10)
(send *robot* :lleg :knee-p :min-angle 10)
(defun demo-static-climb-ladder
  (&key
   (cnt 9))
  (init-pose)
  (do-until-key
   (if (minusp (decf cnt)) (return-from nil t))
   (if
       (not
	(and
	 (static-leg-move :move-key :both)
	 (static-leg-move :move-key :rleg)
	 (static-leg-move :move-key :rleg :move #F(0 0 100))
	 (static-leg-move :move-key :rleg :move #F(0 0 100))
	 (static-leg-move :move-key :rleg :move #F(0 0 100))
	 (static-leg-move :move-key :rleg :move #F(0 0 50))
	 (or
	  t
	  (not (eq cnt 0))
	  (static-leg-move :move-key :rleg :move #F(0 0 30)))
	 (static-leg-move :move-key :rleg :move #F(100 0 0))
	 (static-leg-move :move-key :rleg :move #F(100 0 0))
	 (or
	  (not (eq cnt 0))
	  (static-leg-move :move-key :rleg :move #F(50 0 0)))
	 (on-ladder :move-key :rleg)
	 (static-leg-move :move-key :both)
	 (static-leg-move :move-key :lleg)
	 (static-leg-move :move-key :lleg :move #F(0 0 100))
	 (static-leg-move :move-key :lleg :move #F(0 0 100))
	 (static-leg-move :move-key :lleg :move #F(0 0 100))
	 (static-leg-move :move-key :lleg :move #F(0 0 50))
	 (or
	  t
	  (not (eq cnt 0))
	  (static-leg-move :move-key :lleg :move #F(0 0 30)))
	 (static-leg-move :move-key :lleg :move #F(100 0 0))
	 (static-leg-move :move-key :lleg :move #F(100 0 0))
	 (or
	  (not (eq cnt 0))
	  (static-leg-move :move-key :lleg :move #F(50 0 0)))
	 (on-ladder :move-key :lleg))
	)
       (return-from nil nil)))
  )

(defun leg-orbit
  (&key
   (time-list (list 0 0.7 1.0))
   (target-limb :rleg)
   (start-coords
    (copy-object (send *robot* target-limb :end-coords :worldcoords)))
   (end-coords
    (find-if
     #'(lambda (c)
	 (and
	  (>
	   (norm (v- (send c :worldpos)
		     (send start-coords :worldpos)))
	   100)
	  (plusp
	   (aref
	    (v- (send c :worldpos)
		(send start-coords :worldpos))
	    2))))
     (nearest-land-coords :move-key target-limb)))
   (margin 100)
   (p-list
    (list
     (send start-coords :worldpos)
     (let ((s (send start-coords :worldpos))
	   (e (send end-coords :worldpos)))
       (concatenate
	float-vector
	(subseq (scale 0.5 (v+ s e)) 0 2)
	(list (+ (aref (vmax s e) 2) margin))))
     (v+ (float-vector 0 0 30)
	 (send end-coords :worldpos))
     ))
   (dp-list
    (list #F(0 0 0) nil #F(0 0 0)))
   (ddp-list
    (list #F(0 0 0) nil #F(0 0 0)))
   (time-step 0.05)
   (spline-coeff-func 'solve-spline)
   (debug? t)
   (n nil))
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

(defun track-orbit
  (&key
   (target-limb :rleg)
   (orbit (leg-orbit :target-limb target-limb))
   ret pos buf
   )
  (do-until-key
   (setq pos (car orbit))
   (setq orbit (cdr orbit))
   (if (null orbit) (return-from nil (reverse ret)))
   (setq
    buf
    (static-leg-move
     :move-key target-limb
     :move (v- pos (send *robot* target-limb :end-coords :worldpos))))
   (if (not (vectorp buf)) (return-from nil nil))
   (push
    (list (cons :angle-vector (copy-object (send *robot* :angle-vector)))
	  (cons :worldcoords (copy-object (send *robot* :worldcoords))))
    ret))
  (reverse ret))

(defun demo-static-climb-ladder2
  (&key
   (cnt 9))
  (init-pose)
  (do-until-key
   (if (minusp (decf cnt)) (return-from nil t))
   (static-leg-move :move-key :both)
   (static-leg-move :move-key :rleg)
   (track-orbit :target-limb :rleg)
   (on-ladder :move-key :rleg)
   (static-leg-move :move-key :both)
   (static-leg-move :move-key :lleg)
   (track-orbit :target-limb :lleg)
   (on-ladder :move-key :lleg)
   )
  )

#|

