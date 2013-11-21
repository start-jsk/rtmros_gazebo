(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")

(load "package://hrpsys_gazebo_atlas/models/drc_ladder.l")
(setq *ladder* (drc-ladder))
(atlas-init-ex :real nil)
(objects (list *atlas* *ladder*))

(send *atlas* :reset-manip-pose)
(send *atlas* :fix-leg-to-coords (make-coords :pos #f(0 -1500 0) :rpy (list pi/2 0 0)))

(send *irtviewer* :draw-objects)
