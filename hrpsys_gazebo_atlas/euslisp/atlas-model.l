(if (boundp '*atlas-name*)
    (cond
     ((equal *atlas-name* :atlas-v0)
      (require :atlas "package://hrpsys_gazebo_atlas/models/atlas.l"))
     ((equal *atlas-name* :atlas-v3)
      (require :atlas "package://hrpsys_gazebo_atlas/models/atlas_v3.l")))
  (progn
    (ros::ros-warn "atlas-v3 is used as default")
    (require :atlas "package://hrpsys_gazebo_atlas/models/atlas_v3.l"))
  )
(require :sandia_hand_left  "package://hrpsys_gazebo_atlas/models/sandia_hand_left.l")
(require :sandia_hand_right "package://hrpsys_gazebo_atlas/models/sandia_hand_right.l")

(defun atlas-with-hand ()
  (atlas)
  ;; left hand
  (sandia_hand_left)
  (send *sandia_hand_left* :move-to (send (send *atlas* :links :l_hand :copy-worldcoords) :rotate pi/2 :y :local))
  ;;(send *sandia_hand_left* :move-to (send *atlas* :links :l_hand :copy-worldcoords))
  (send *atlas* :links :l_hand :assoc *sandia_hand_left*)
  (setf (get *atlas* :lhand-model) *sandia_hand_left*)
  ;; right hand
  (sandia_hand_right)
  (send *sandia_hand_right* :move-to (send (send *atlas* :links :r_hand :copy-worldcoords) :rotate pi/2 :y :local))
  ;;(send *sandia_hand_right* :move-to (send *atlas* :links :r_hand :copy-worldcoords))
  (send *atlas* :links :r_hand :assoc *sandia_hand_right*)
  (setf (get *atlas* :rhand-model) *sandia_hand_right*)
  ;; small limit
  (send *atlas* :lleg :crotch-r :min-angle -5)
  (send *atlas* :rleg :crotch-r :max-angle 5)
  ;; update bodies
  (let ((atbd (*atlas* . bodies)))
    (setq atbd (append atbd (send *sandia_hand_left* :bodies)))
    (setq atbd (append atbd (send *sandia_hand_right* :bodies)))
    (setq (*atlas* . bodies) atbd)
    )
  *atlas*)

(defmethod atlas-robot
  (:lhand
   ()
   (send self :get :lhand-model))
  (:rhand
   ()
   (send self :get :rhand-model)))

;; hand pose
(dolist (ahand '(sandia_hand_left-robot sandia_hand_right-robot))
  (let ((hand-pose-def-macros
         `(defmethod ,ahand
            (:open-pose
             ()
             (send self :angle-vector #f(0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 90.0 0.0 0.0))
             )
            (:open-pose2
             ()
             (send self :angle-vector #f(0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 -90.0 0.0))
             )
            (:hook-pose
             ()
             (send self :angle-vector #f(0.0 20.0 20.0 0.0 20.0 20.0 0.0 20.0 20.0 0.0 0.0 30.0))
             )
            (:grasp-pose
             ()
             (send self :angle-vector #f(0.0 90.0 90.0 0.0 90.0 90.0 0.0 90.0 90.0 0.0 50.0 50.0))
             )
            )
         ))
    ;;(pprint (macroexpand hand-pose-def-macros))
    (eval hand-pose-def-macros)
    ))
