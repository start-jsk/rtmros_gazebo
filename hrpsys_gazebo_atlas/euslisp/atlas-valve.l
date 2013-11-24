(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
(load "package://hrpsys_gazebo_atlas/models/drc-valve.l")

(load "package://hrpsys_gazebo_atlas/euslisp/atlas-fullbody-ik.l")


(defun init-valve-setting
  (&key (valve-r 200))

  (setq *valve* (instance valve :init valve-r))
  ;; (send (send *valve* :valve-handle0) :rotate (deg2rad 90) :y :local)
  ;; (send (send *valve* :valve-handle1) :rotate (deg2rad 90) :y :local)

  ;; (setq *valve* (make-valve))
  (atlas-init-ex :real nil)

  (send *atlas* :reset-manip-pose)
  (send *atlas* :fix-leg-to-coords (make-coords))

  (send *valve* :locate #f(1000 0 1000) :world)
  (send *valve* :orient 90 :y :world)
  (send *valve* :orient (deg2rad -90) :y :world)

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
  (send *robot* :rleg
	:inverse-kinematics
	(make-coords :pos #F(0 -200 0) :rpy (list (deg2rad -20) 0 0))
	:link-list
	(send *robot* :link-list
	      (send *robot* :rleg :end-coords :parent))
	:debug-view nil)
  (send *robot* :lleg
	:inverse-kinematics
	(make-coords :pos #F(0 +200 0) :rpy (list (deg2rad +20) 0 0))
	:link-list
	(send *robot* :link-list
	      (send *robot* :lleg :end-coords :parent))
	:debug-view nil)
  (if (and (boundp '*viewer*) *viewer*)
      (send *viewer* :draw-objects))
  )



(defun try-valve-once
  ()

  (init-valve-pose)

  (grasp :real nil)
  (let* ((valve-rotate-angle 150.0)
	 (div-num 20))
    (send (send *valve* :valve-joint) :joint-angle (- (/ valve-rotate-angle 2)))
    (dotimes (i div-num)

      (fullbody-ik-main
       :target-limb
       (list :rarm :larm :rleg :lleg)
       :target-coords
       (append (mapcar #'(lambda (hname) (send *valve* hname)) (list :valve-handle0 :valve-handle1)) (list nil nil))
       :rotation-axis (list :y :y t t))
      (send (send *valve* :valve-joint)
	    :joint-angle (/ valve-rotate-angle div-num) :relative t)
      (x::window-main-one))))

(defun try-valve
  ()

  (dolist (vr (list 150 200 250 300 350 400))
    (init-valve-setting :valve-r vr)
    (try-valve-once)))

