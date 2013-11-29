(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")


(defun init-touch-it-without-server
  (&key (real t))
  (cond ((and real (not (and (boundp '*ri*) *ri*)))
   (atlas-init-ex :set-reset-pose nil :real t))
  ((not (and (boundp '*atlas*) *atlas*))
   (atlas-init-ex :set-reset-pose nil :real nil)))
  (setq *do-not-start-ik-server* t)
  (load "package://hrpsys_gazebo_atlas/euslisp/atlas-eus-ik.l")
  ;; (setq *do-not-start-touch-it* t)
  ;;(load "package://hrpsys_gazebo_atlas/euslisp/touch-it-server.l")
  (setup-end-coords)
  (when (and (boundp '*ri*) *ri*)
    (send *ri* :set-interpolation-mode ":linear")))


(defun move-down-with-touch-it
  ()
  (send *atlas* :reset-manip-pose)
  (send *atlas* :rarm :move-end-rot 90 :x :local)
  (send *atlas* :rarm :move-end-pos #f(0 0 150) :world :rotation-axis :z :revert-if-fail t)
  (send *irtviewer* :draw-objects)
  (model2real :time 3000 :wait t)
  (my-wait)
  (touch-it-control (send (send (send *atlas* :rarm :end-coords) :copy-worldcoords) :translate #f(0 0 -300) :world)))


(defun move-lateral-with-touch-it
  ()
  (send *atlas* :reset-manip-pose)
  (send *atlas* :rarm :move-end-pos #f(0 0 -200))
  (send *irtviewer* :draw-objects)
  (model2real :time 3000 :wait t)
  (my-wait)
  (touch-it-control (send (send (send *atlas* :rarm :end-coords) :copy-worldcoords) :translate #f(0 0 300)))) 


(defun touch-it-control
  (tc &key (arm :rarm) (ik-type arm))

  ;; calculate target
  (let ((div 30)
  (i 0)
  (touch-flag t)
  (f-orig (send *ri* :state :absolute-force-vector arm)))
    (block :touch-it-loop
      (do-until-key
       ;; check count
       (when (> i div)
   (setq touch-flag nil)
   (return-from :touch-it-loop nil))
       ;; check the force sensor value
       (let* (f
        f-conv
        dir
        val
        (thre 20))
   (if (and (boundp '*ri*) *ri*)
       (setq f (v- (send *ri* :state :absolute-force-vector arm) f-orig))
     (setq f (float-vector 0 0 0)))
   (setq f-conv (float-vector (elt f 0) (- (elt f 1)) (- (elt f 2))))
   (setq dir (normalize-vector (v- (send tc :worldpos) (send (send *atlas* arm :end-coords) :worldpos))))
   (setq val (abs (v. f-conv dir)))
   (warning-message 2 "[touch_it] ~a force = ~a # touchitforce~%" arm f)
   (warning-message 2 "[touch_it] ~a dir = ~a  val = ~a  thre = ~a # touchitforce~%" arm dir val thre)
   (when (> val thre)
     (return-from :touch-it-loop nil)))

       ;; set target coords
       (let* ((r (/ (+ i 1.0) div))
        (c (midcoords r
          (send (send *atlas* arm :end-coords) :worldcoords)
          tc)))

   ;; solve ik
   (warning-message 1 "[touch_it] calculating pose <~a = ~a/~a> !~%" r i div)
   (cond ((substringp "fullbody" (string ik-type))
    (fullbody-ik-with-collision arm c))
         ((substringp "torso" (string ik-type))
    (torso-ik-with-collision arm c))
         (t
    (send *atlas* arm :inverse-kinematics c :stop 100
          :thre 2
          :rthre (deg2rad 5)
          :rotation-axis :z
          :revert-if-fail t))
         )

   ;; send angle
   (if (and (boundp '*ri*) *ri*)
       (progn
         (model2real :time 2000 :wait t))
     ;;(send *ri* :angle-vector (send *atlas* :angle-vector) 2000)
     ;;(unix:usleep (* 1500 1000)))
     (progn
       (unix:usleep (* 20 1000))))
   (send *irtviewer* :draw-objects)
   )
       (incf i))
      (read-line)
      )

    (when (and (boundp '*ri*) *ri*)
      (real2model)
      (model2real :time 500))
    (send *irtviewer* :draw-objects)
    touch-flag
    ))

