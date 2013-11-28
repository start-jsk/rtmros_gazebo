(require "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
(require "package://hrpsys_gazebo_atlas/models/drc_ladder.l")

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
(cond
 ((boundp '*irtviewer*)
  (send *irtviewer* :change-background #f(1 1 1)))
 ((boundp '*pickview*)
  (send *viewer* :viewsurface :bg-color #F(1 1 1))))
(send *viewer* :draw-objects)

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

(setq *hand-print*
      (let ((now #F(-690.0 430.0 0.0))
	    (step (float-vector (/ 305 (tan (deg2rad 60))) 0 305))
	    (rot (make-coords))
	    (cnt 30))
	(send rot :rotate (deg2rad 90) :z)
	(send rot :rotate (deg2rad 30) :x)
	(send rot :rotate (deg2rad 90) :z)
	(mapcar
	 #'(lambda (hoge)
	     (setq now (v+ now step))
	     (make-coords
	      :pos now
	      :rot (copy-object (send rot :worldrot))))
	 (make-list cnt))))
(send-all (flatten *hand-print*) :draw-on :flush t)

(defun static-leg-move
  (&key
   (move-key :rleg)
   (fix-key (case move-key
		  (:rleg :lleg)
		  (:lleg :rleg)
		  (t :both)))
   (target-limb '(:rleg :lleg))
   (move #F(0 0 0))
   ;;
   (mt (mapcar
	#'(lambda (k) (send *robot* k :end-coords))
	target-limb))
   (ll (mapcar
	#'(lambda (m) (send *robot* :link-list (send m :parent)))
	mt))
   (target-coords nil)
   )
  ;; (case move-key
  ;; 	(:rleg
  ;; 	 (send *robot* :rarm :elbow-p :joint-angle 0)
  ;; 	 (send *robot* :larm :elbow-p :joint-angle 0);123)
  ;; 	 )
  ;; 	(:lleg
  ;; 	 (send *robot* :rarm :elbow-p :joint-angle 0);-123)
  ;; 	 (send *robot* :larm :elbow-p :joint-angle 0)
  ;; 	 ))
  (let* (buf)
    (send *robot* :fullbody-inverse-kinematics
	  (mapcar
	   #'(lambda (mt k)
	       (setq buf (send mt :copy-worldcoords))
	       (cond
		((eq k move-key)
		 (if target-coords
		     (send buf :newcoords target-coords)
		   (send buf :translate move :world)))
		(t t))
	       buf)
	   mt target-limb)
	  :move-target mt
	  :link-list ll
	  :debug-view :no-message
	  :min #F(-1000 -1000 -1000 -200 -200 -200)
	  :max #F(1000 1000 1000 200 200 200)
	  :translation-axis
	  (make-list (length target-limb) :initial-element t)
	  :rotation-axis
	  (make-list (length target-limb) :initial-element t)
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
		    (:lleg #F(25 0 0))
		    (t #F(0 0 0)))
	      (if (eq fix-key :both)
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
   (candidates
    (if (find move-key '(:rleg :lleg))
	*foot-print* *hand-print*))
   )
  (sort
   (mapcar
    (case move-key
	  (:rleg #'car)
	  (:lleg #'cadr)
	  (t #'identity))
    candidates)
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
   (candidates
    (if (find move-key '(:rleg :lleg)) *foot-print* *hand-print*))
   (target-limb '(:rleg :lleg))
   (find-func #'identity)
   (target-c
    (find-if
     find-func
     (nearest-land-coords
      :candidates candidates
      :move-key move-key
      :now-c now-c))))
  (static-leg-move
   :target-limb target-limb
   :move-key move-key
   :move (v+
	  (case move-key
		(:rleg #F(30 0 0))
		(:lleg #F(-10 0 0))
		(t #F(0 0 0)))
	  (v- (send target-c :worldpos)
	      (send now-c :worldpos)))
   :target-coords
   (if (find move-key '(:rarm :larm)) target-c))
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

;; spline orbit

(defun leg-orbit
  (&key
   (time-list (list 0 0.7 1.0))
   (move-key :rleg)
   (start-coords
    (copy-object (send *robot* move-key :end-coords :worldcoords)))
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
     (nearest-land-coords :move-key move-key)))
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
   (move-key :rleg)
   (target-limb '(:rleg :lleg))
   (orbit (leg-orbit :move-key move-key))
   ret pos buf
   )
  (do-until-key
   (setq pos (car orbit))
   (setq orbit (cdr orbit))
   (if (null orbit) (return-from nil (reverse ret)))
   (setq
    buf
    (static-leg-move
     :move-key move-key
     :target-limb target-limb
     :move (v- pos (send *robot* move-key :end-coords :worldpos))))
   (if (not (vectorp buf)) (return-from nil nil))
   (push (simple-robot-state :track-orbit) ret))
  ret)

(defun demo-spline-climb-ladder
  (&key
   (cnt 9)
   ret)
  (init-pose)
  (push (simple-robot-state :init) ret)
  (do-until-key
   (if (minusp (decf cnt)) (return-from nil t))
   (static-leg-move :move-key :both)
   (push (simple-robot-state :balance) ret)
   (static-leg-move :move-key :rleg)
   (push (simple-robot-state :lean) ret)
   (setq
    ret
    (append
     (track-orbit :move-key :rleg)
     ret))
   (on-ladder :move-key :rleg)
   (push (simple-robot-state :land) ret)
   (static-leg-move :move-key :both)
   (push (simple-robot-state :balance) ret)
   (static-leg-move :move-key :lleg)
   (push (simple-robot-state :lean) ret)
   (setq
    ret
    (append
     (track-orbit :move-key :lleg)
     ret))
   (on-ladder :move-key :lleg)
   (push (simple-robot-state :lean) ret)
   )
  ret
  )

;; with hand

(defun demo-hand-climb-ladder
  (&key
   (cnt 3)
   ret)
  ;;
  (init-pose)
  (mapcar
   #'(lambda (j av) (send j :joint-angle av))
   (send *robot* :rarm :joint-list)
   '(-122.721 100.575 -5.69718 89.8952 19.1718 87.8511))
  (mapcar
   #'(lambda (j av) (send j :joint-angle av))
   (send *robot* :larm :joint-list)
   '(122.718 100.538 5.72497 -89.8946 19.1693 87.8309))
  (push (simple-robot-state :init) ret)
  ;;
  (on-ladder :move-key :rarm
	     :target-limb '(:rarm :rleg :lleg))
  (push (simple-robot-state :grasp) ret)
  ;;
  (do-until-key
   (if (minusp (decf cnt)) (return-from nil t))
   (static-leg-move :move-key :both
		    :target-limb '(:rarm :rleg :lleg))
   (push (simple-robot-state :balance) ret)
   ;;
   (static-leg-move :move-key :rleg
		    :target-limb '(:rarm :rleg :lleg))
   (push (simple-robot-state :lean) ret)
   ;;
   (setq
    ret
    (append
     (track-orbit :move-key :rleg
		  :target-limb '(:rarm :rleg :lleg))
     ret))
   ;;
   (on-ladder :move-key :rleg
	      :target-limb '(:rarm :rleg :lleg))
   (push (simple-robot-state :land) ret)
   ;;
   (static-leg-move :move-key :both
		    :target-limb '(:rarm :rleg :lleg))
   (push (simple-robot-state :balance) ret)
   ;;
   (on-ladder :move-key :rarm
	      :find-func
	      #'(lambda (v)
		  (let ((diff (v- (send v :worldpos)
				  (send *robot* :rarm :end-coords :worldpos))))
		    (and (> (norm diff) 100)
			 (> (aref diff 2) 2))))
	      :target-limb '(:rarm :rleg :lleg))
   (push (simple-robot-state :grasp) ret)
   ;;
   (static-leg-move :move-key :lleg
		    :target-limb '(:rarm :rleg :lleg))
   (push (simple-robot-state :lean) ret)
   ;;
   (setq
    ret
    (append
     (track-orbit :move-key :lleg
		  :target-limb '(:rarm :rleg :lleg))
     ret))
   ;;
   (on-ladder :move-key :lleg
	      :target-limb '(:rarm :rleg :lleg))
   (push (simple-robot-state :land) ret)
   )
  ret
  )

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



#|

(dump-loadable-structure "climb-with-hand-20131128.l" *rsd*)
(load "climb-with-hand-20131128.l")
(do-until-key (simple-rsd-play *rsd*))
