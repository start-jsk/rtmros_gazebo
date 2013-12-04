(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
(load "package://hrpsys_gazebo_atlas/euslisp/atlas-eus-ik.l")


(setq *do-not-start-ik-server* t)
(load "package://euslisp/jskeus/eus/models/room73b2-door-right-object.l")


(defun init-door-setting
  (&optional (robot-x-pos -600))

  (setq *door* (room73b2-door-right))
  (send *door* :door :max-angle 100)
  (send *door* :door :min-angle -100)
  (send (elt (send *door* :links) 2) :assoc
	(send (send *door* :handle-front-knob) :translate #f(200 0 0)))
  (send (elt (send *door* :links) 2) :assoc (send *door* :handle-rear-knob))
  (objects (list *door*))

  (atlas-init-ex :real nil)
  (setq *robot* *atlas*)
  (init-fullbody-ik-main)

  (init-door-pose)
  (send *atlas* :fix-leg-to-coords (make-coords :pos (float-vector robot-x-pos 0 0)))
  (make-foot-convex)

  (objects (list *atlas* *door* *foot-convex* *centroid-object*))
  )

(defun init-door-pose
  ()
  (send *atlas* :reset-manip-pose))

(defun try-door
  (&optional (door-rotate-angle -30.0))

  (init-door-pose)

  (grasp :real nil)
  (let* ((div-num 20)
	 (arm :larm)
	 (hname :handle-front-knob))

    (send *door* :door :joint-angle 0)
    (dotimes (i div-num)

      (apply
       #'fullbody-ik-main
       (list
	:target-limb (list arm :lleg :rleg)
	:target-coords (list (send *door* hname) nil nil)
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


      (send (send *door* :door)
	    :joint-angle (- (/ door-rotate-angle div-num)) :relative t)
      (x::window-main-one))))

(defun try-push-door
  ()
  (init-door-setting)
  (try-door))

(defun try-pull-door
  ()
  (init-door-setting -700)
  (try-door 30))

