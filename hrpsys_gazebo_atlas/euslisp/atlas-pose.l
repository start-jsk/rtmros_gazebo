
(defun atlas-calibration-pose (&optional n)
  (send *atlas* :reset-pose)
  (case n
    (1 (send *atlas* :larm :angle-vector #f(30.0 90.0 0.0 -80.0 15.0 0.0)) ;; reset-pose
       )
    (2 (send *atlas* :larm :angle-vector #f(110.0 90.0 30.0 -80.0 15.0 0.0)) ;; reset-manip-pose
       )
    (3 (send *atlas* :larm :angle-vector #f(83.794 105.755 20.0516 -45.8593 -42.4294 12.345))
       )
    )
  )
