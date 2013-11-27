(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")

(defun init-touch-it-without-server
  ()
  (setq *do-not-start-ik-server* t)
  (load "package://hrpsys_gazebo_atlas/euslisp/atlas-eus-ik.l")
  (setq *do-not-start-touch-it* t)
  (load "package://hrpsys_gazebo_atlas/euslisp/touch-it-server.l")
  (setup-end-coords))


(defun move-down-with-touch-it
  ()
  (send *atlas* :reset-manip-pose)
  (send *atlas* :rarm :move-end-rot 90 :x :local)
  (send *irtviewer* :draw-objects)
  (model2real :time 3000 :wait t)
  (my-wait)
  (touch-it-control (send (send (send *atlas* :rarm :end-coords) :copy-worldcoords) :translate #f(0 0 300))))


(defun move-lateral-with-touch-it
  ()
  (send *atlas* :reset-manip-pose)
  (send *atlas* :rarm :move-end-pos #f(0 0 -200))
  (send *irtviewer* :draw-objects)
  (model2real :time 3000 :wait t)
  (my-wait)
  (touch-it-control (send (send (send *atlas* :rarm :end-coords) :copy-worldcoords) :translate #f(0 0 300)))) 

