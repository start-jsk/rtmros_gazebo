(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")

(load "package://hrpsys_gazebo_atlas/models/drc_ladder.l")
(setq *ladder* (drc-ladder))
(atlas-init-ex :real nil)
(setq *robot* *atlas*)

(send *ladder* :rotate (deg2rad -90) :z)
(send *ladder* :translate #F(1180 0 0) :world)

(defun init-pose
  (&rest args)
  (send *robot* :reset-manip-pose)
  (send *robot* :larm :shoulder-r :joint-angle 90)
  (send *robot* :rarm :shoulder-r :joint-angle -90)
  ;;
  (send *robot* :rarm :elbow-p :joint-angle 0)
  (send *robot* :larm :elbow-p :joint-angle 0)
  ;;
  (send *sandia_hand_left* :grasp-pose)
  (send *sandia_hand_right* :grasp-pose)
  (send *robot* :newcoords
	(make-coords
	 :rpy '(3.14 0 0)
	 :pos
	 (float-vector
	  0 0
	  (+
	   0
	   (aref
	    (v- (send *robot* :worldpos)
		(send *robot* :rleg :end-coords :worldpos))
	    2))))))

(init-pose)
(objects (list *ladder* *robot*))

(setq *foot-print*
      (let ((now (float-vector 0 0 0))
	    (step (float-vector (/ 305 (tan (deg2rad 60))) 0 305))
	    (cnt 9))
	(mapcar
	 #'(lambda (hoge)
	     (setq now (v+ now step))
	     (mapcar
	      #'(lambda (k)
		  (make-coords
		   :pos
		   (v+ now
		       (map float-vector
			    #'*
			    #F(0 1.2 0)
			    (send *robot* k :end-coords :worldpos)))
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
	  (v+ #F(50 0 0)
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

(defun on-ladder
  (&key
   (move-key :rleg)
   (now-c (send *robot* move-key :end-coords :copy-worldcoords))
   target-c)
  (setq
   target-c
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
  (static-leg-move
   :move-key move-key
   :move (v+
	  #F(0 0 0)
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



#|

