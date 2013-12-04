(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
(load "package://hrpsys_gazebo_atlas/euslisp/atlas-eus-ik.l")

(load "package://euslisp/jskeus/eus/models/room73b2-door-right-object.l")


(defun init-door-model
  ()
  (setq *door* (room73b2-door-right))
  (send *door* :door :max-angle -100)
  (send *door* :door :min-angle -100)
  (objects (list *door*)))
