(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")


(defun make-valve
  (&optional
   (ring-r-inch 8)
   (ring-r (* ring-r-inch 25.4))
   (div 10)
   (valve-r (* ring-r 0.2)))
  (let* (v-list)
    (dotimes (i div)
      (push (float-vector (+ (* valve-r (cos (* i (/ 2pi div)))) ring-r) 0  
			  (* valve-r (sin (* i (/ 2pi div))))) v-list))
    (make-torus v-list)))


(setq *valve* (make-valve))
(atlas-init-ex :real nil)
(objects (list *atlas* *valve*))

(send *atlas* :reset-manip-pose)
(send *atlas* :fix-leg-to-coords (make-coords))

(send *valve* :locate #f(600 0 1200) :world)
(send *valve* :orient 90 :y :world)
(send *valve* :orient (deg2rad 90) :y :world)

(send *irtviewer* :draw-objects)