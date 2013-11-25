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
  (send *sandia_hand_left* :grasp-pose)
  (send *sandia_hand_right* :grasp-pose)
  (send *robot* :newcoords
	(make-coords
	 :rpy '(3.14 0 0)
	 :pos
	 (float-vector
	  0 0
	  (aref (v- (send *robot* :worldpos)
		    (send *robot* :rleg :end-coords :worldpos)) 2)))))

(init-pose)
(objects (list *ladder* *robot*))

(setq *foot-print*
      (let ((now (float-vector 0 0 0))
	    (step (float-vector (/ 300 (tan (deg2rad 60))) 0 300))
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
			    #F(0 1 0)
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
   (move #F(0 0 0)))
  (let* ((mt (mapcar
	      #'(lambda (k) (send *robot* k :end-coords))
	      '(:rleg :lleg)))
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
	   mt '(:rleg :lleg))
	  :move-target mt
	  :link-list ll
	  :debug-view :no-message
	  :target-centroid-pos
	  (v+ #F(30 0 0) (send *robot* fix-key :end-coords :worldpos))
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
   :move (v- (send (car target-c) :worldpos)
	     (send now-c :worldpos)))
  )


#|

(mapcar
 #'(lambda (k)
     (send *robot* k :inverse-kinematics
	   (make-coords :pos #F(-10000 0 0))
	   :move-target (send *robot* k :end-coords)
	   :translation-axis :x
	   :rotation-axis nil
	   :debug-view :no-message
	   :revert-if-fail nil)
     )
 '(:rarm :larm))