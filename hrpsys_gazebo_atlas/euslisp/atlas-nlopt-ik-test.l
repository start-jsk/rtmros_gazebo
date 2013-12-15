(require "package://hrpsys_gazebo_atlas/euslisp/atlas-model.l")
(require "package://eus_nlopt/euslisp/nlopt-ik.l")

(defvar *robot* (atlas-with-hand))
(objects *robot*)
(setq *pickview* *irtviewer*)


(send *robot* :move-to (make-coords
                        :pos #F(-22.421 -4.996e-15 794.585)
                        :rpy '(2.068e-25 0.003 -1.222e-06)) :world)
(send *robot* :angle-vector #f(-0.00036 3.11053 0.000566 122.718 100.538 5.72497 -89.8946 19.1693 87.8309 -122.721 100.575 -5.69718 89.8952 19.1718 87.8511 74.3935 0.0 -37.4868 7.000000e-05 -37.0747 0.0 74.3934 0.0 -37.4867 7.000000e-05 -37.0747 0.0 0.0))
(send *pickview* :look-all)

(ik-test
 :max-time 20
 :debug-view :success-draw
 :target-limb :larm
 :xtol 1e-16 :ftol 1e-16
 :alg 3
 :link-list
 (send *robot* :link-list
       (send *robot* :larm :end-coords :parent)
       (send *robot* :larm :root-link))
 :target-coords
 (make-coords
  :pos #f(581.991 136.263 1067.804)
  :rpy  '(2.068e-25 0.003 0))
 )

