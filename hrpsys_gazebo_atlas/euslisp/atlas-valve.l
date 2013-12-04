(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
(load "package://hrpsys_gazebo_atlas/models/drc-valve.l")

(load "package://hrpsys_gazebo_atlas/euslisp/atlas-eus-ik.l")

(setq *do-not-start-ik-server* t)
(switch-end-coords :id 1)

(defun init-valve-setting
  (&key (valve-r 200))

  (setq *valve* (instance valve :init valve-r))

  (atlas-init-ex :real nil)

  (send *atlas* :reset-manip-pose)
  (send *atlas* :fix-leg-to-coords (make-coords))

  (send *valve* :locate #f(1000 0 1000) :world)
  (send *valve* :orient 90 :y :world)
  (send *valve* :orient (deg2rad -90) :y :world)

  (send (send (send *valve* :valve-handle0) :rotate (deg2rad -90) :x) :rotate (deg2rad -90) :z)
  (send (send (send *valve* :valve-handle1) :rotate (deg2rad -90) :x) :rotate (deg2rad -90) :z)

  (send *irtviewer* :draw-objects)


  (atlas-init-ex :real nil)
  (setq *robot* *atlas*)
  (init-fullbody-ik-main)

  (init-valve-pose)
  (make-foot-convex)

  (objects (list *atlas* *valve* *foot-convex* *centroid-object*))
  )


(defun init-valve-pose
  nil
  (send *robot* :reset-manip-pose)
  (fix-leg)
  ;; (send *robot* :rleg
  ;; 	:inverse-kinematics
  ;; 	(make-coords :pos #F(0 -200 0) :rpy (list (deg2rad -20) 0 0))
  ;; 	:link-list
  ;; 	(send *robot* :link-list
  ;; 	      (send *robot* :rleg :end-coords :parent))
  ;; 	:debug-view nil)
  ;; (send *robot* :lleg
  ;; 	:inverse-kinematics
  ;; 	(make-coords :pos #F(0 +200 0) :rpy (list (deg2rad +20) 0 0))
  ;; 	:link-list
  ;; 	(send *robot* :link-list
  ;; 	      (send *robot* :lleg :end-coords :parent))
  ;; 	:debug-view nil)
  (if (and (boundp '*viewer*) *viewer*)
      (send *viewer* :draw-objects))
  )



(defun try-valve-once
  ()

  (init-valve-pose)

  (grasp :real nil)
  (let* ((valve-rotate-angle 120.0)
	 (div-num 20)
	 (arm :rarm)
	 (hname :valve-handle0))

    (send (send *valve* :valve-joint) :joint-angle (+ (/ valve-rotate-angle 2)))
    (dotimes (i div-num)

      (apply
       #'fullbody-ik-main
       (list
	:target-limb (list arm :lleg :rleg)
	:target-coords (list (send *valve* hname) nil nil)
	:additional-weight-list
	(mapcar
	 #'(lambda (k gain)
	     (list (send *atlas* :torso k :child-link)
		   gain))
	 (list :waist-r :waist-p :waist-y)
	 '(0 0 1))
	;;(list 1e-3 1e-3 1e-3))
	:root-link-virtual-joint-weight
	(float-vector 0 0 0 0 0 0)
	:min-vec (min
		  (scale
		   -1
		   (map float-vector
			#'*
			(float-vector 0 0 0 0 0 0)
			#F(1000 1000 1000 200 200 200))))
	:max-vec (map float-vector
		      #'*
		      (float-vector 0 0 0 0 0 0)
		      #F(1000 1000 1000 200 200 200))
	:collision? t
	;;:centroid-thre 200
	;;:target-centroid-pos nil
	:thre '(1 1 1)
	;;:stop 100
	:rotation-axis '(nil t t)))


      (send (send *valve* :valve-joint)
	    :joint-angle (- (/ valve-rotate-angle div-num)) :relative t)
      (x::window-main-one))))

(defun try-valve
  ()

  (dolist (vr (list 230 460))
    (init-valve-setting :valve-r vr)
    (try-valve-once)))
