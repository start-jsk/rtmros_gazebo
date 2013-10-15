(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")


(defun atlas-walk-demo
  ()

  ;; init
  (my-init)
  (send *ri* :start-st)

  (send *atlas* :walk-pose)
  (model2real :wait t)

  ;; straight
  (send *ri* :go-pos 1 0 0)

  ;; turn
  (send *ri* :go-pos 0 0 90)

  (send *ri* :go-pos 0 -0.5 0)

  ;; curve
  (send *ri* :go-pos 1 -1 -30)

  (send *atlas* :walk-manip-pose)
  (model2real :wait t)

  ;; manip walk (keep upper body during walking)
  (send *ri* :go-pos 1 -1 -30)
  (send *ri* :go-pos 0 0 -30)

  ;; manip walk (move upper body during walking)
  (send *ri* :start-auto-balancer)
  (send *ri* :go-velocity 1 0 0)
  (unix::usleep (* 2000 1000))
  (send *atlas* :walk-pose)
  (model2real :wait t :time 2000)
  (send *atlas* :walk-manip-pose)
  (model2real :wait t :time 2000)
  (send *atlas* :walk-pose)
  (model2real :wait t :time 2000)
  (send *atlas* :walk-manip-pose)
  (model2real :wait t :time 2000)
  (unix::usleep (* 2000 1000))
  (send *ri* :go-stop)

  ;; change waist height
  (send *atlas* :reset-manip-pose)
  (send *atlas* :legs :move-end-pos #f(0 0 20))
  (model2real :wait t)
  (send *ri* :start-auto-balancer)
  (send *ri* :go-velocity 1 0 0)
  (unix::usleep (* 2000 1000))
  (model2real :wait t :time 2000)
  (send *atlas* :legs :move-end-pos #f(0 0 150))
  (model2real :wait t :time 2000)
  (unix::usleep (* 4000 1000))
  (send *atlas* :legs :move-end-pos #f(0 0 -150))
  (model2real :wait t :time 2000)
  (unix::usleep (* 2000 1000))
  (send *ri* :go-stop)
  )


(defun atlas-st-demo
  ()

  ;; init
  (my-init)
  (send *ri* :start-auto-balancer)
  (send *ri* :start-st)
  (send *atlas* :angle-vector #f(0.0 0.0 0.0 58.0499 129.6 57.8137 -33.1998 -40.5001 -45.0058 -74.2499 91.7999 -32.5154 74.5998 -29.2501 0.0 29.5427 0.057929 -16.8707 -0.057929 -12.672 -0.013939 29.5415 0.05793 -16.8671 -0.05793 -12.6744 -0.013939 0.0))
  (model2real :wait t)

  ;; add obj
  (load "package://euslib/demo/murooka/simulation_planner/wholebody-holding.l")
  (load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
  (add-box "simple_box" (make-coords :pos #f(1000 0 1000)))
  (setq *obj* *simple_box*)
  (objects (list *atlas* *obj*))
  (unix:usleep (* 1000 1000))
  (obj-gzb2eus)
  (fix-leg)
  (send *obj* :newcoords (apply #'midcoords 0.5 (send *atlas* :arms :end-coords)))
  (send *obj* :translate #f(0 0 300) :world)
  (send *obj* :translate #f(-50 0 0) :world)
  (send *obj* :rotate (deg2rad -20) :y :world)
  (send *pickview* :draw-objects)
  (obj-eus2gzb)

  ;; walk with object
  (send *atlas* :legs :move-end-pos #f(0 0 100))
  (model2real :wait t :time 4000)
  (send *ri* :go-velocity 0.1 0 0)
  (unix:usleep (* 15 1000 1000))
  (send *ri* :autobalancerservice_goStop) ;; :go-stop but keep auto-balancer
  (send *atlas* :legs :move-end-pos #f(0 0 -100))
  (model2real :wait t :time 4000)
  )


(defun atlas-impedance-demo
  ()

  ;; init
  (my-init)
  (send *ri* :start-auto-balancer)
  (send *ri* :start-st)
  (send *ri* :start-impedance :arms)
  (send *ri* :set-impedance-controller-param :arms :moment-gain (list 0 0 0))
  (send *atlas* :reset-manip-pose)
  (model2real :wait t)

  ;; set robot pose
  (send *atlas* :larm :inverse-kinematics (make-coords :pos (send (send *atlas* :larm :end-coords :copy-worldcoords) :worldpos) :rpy (list -pi/2 0 0)))
  (send *atlas* :rarm :inverse-kinematics (make-coords :pos (send (send *atlas* :rarm :end-coords :copy-worldcoords) :worldpos) :rpy (list pi/2 0 0)))
  (model2real :wait t)
  (send *atlas* :larm :move-end-pos #f(0 175 0) :world)
  (send *atlas* :rarm :move-end-pos #f(0 -175 0) :world)
  (model2real :wait t)
  (send *atlas* :legs :move-end-pos #f(0 0 150) :world)
  (send *atlas* :arms :move-end-pos #f(0 0 -50) :world)
  (model2real :wait t)

  ;; add obj
  (load "package://euslib/demo/murooka/simulation_planner/wholebody-holding.l")
  (load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
  (add-box "simple_box2" (make-coords :pos #f(1000 0 0)))
  (setq *obj* *simple_box2*)
  (objects (list *atlas* *obj*))
  (unix:usleep (* 1000 1000))
  (obj-gzb2eus)
  (fix-leg)
  (send *obj* :newcoords (make-coords :pos #f(825 0 0)))
  (obj-eus2gzb)
  (send *pickview* :draw-objects)

  ;; grasp obj
  (load "package://hrpsys_gazebo_atlas/euslisp/atlas-manip-obj.l")
  (reach-hand)
  (set-ref-force :y -30 :arm :rarm)
  (set-ref-force :y 30 :arm :larm)

  ;; push walk
  (set-ref-force :x -50 :y -50 :arm :rarm)
  (set-ref-force :x -50 :y 50 :arm :larm)
  (send *ri* :set-auto-balancer-param
	:default-zmp-offsets (list (float-vector 200 0 0) (float-vector 200 0 0)))
  (send *ri* :go-velocity 0.1 0 0)
  (unix:usleep (* 15 1000 1000))
  (send *ri* :autobalancerservice_goStop) ;; :go-stop but keep auto-balancer
  (send *ri* :set-auto-balancer-param
	:default-zmp-offsets (list (float-vector 30 0 0) (float-vector 30 0 0)))
  (unix:usleep (* 1 1000 1000))

  ;; release
  (set-ref-force :arm :arms)
  (send *atlas* :larm :move-end-pos #f(-60 50 0) :world)
  (send *atlas* :rarm :move-end-pos #f(60 50 0) :world)
  (model2real :wait t)
  )




(defun atlas-qual-door-demo
  ()

  ;; init
  (my-init)
  (send *ri* :start-st)
  (send *ri* :start-auto-balancer)

  ;; set pose
  (grasp :real t)
  (send *atlas* :angle-vector #f(0.0 0.0 0.0 0.0 28.8 -24.981 -79.9998 -67.5002 -54.007 0.0 28.8 24.981 79.9998 -67.5002 -54.007 69.8105 0.066304 -38.0241 -0.066304 -31.7866 -0.010269 69.8101 0.066304 -38.0215 -0.066304 -31.7888 -0.010269 0.0))
  (model2real :wait t :time 4000)

  ;; start-walk
  (send *ri* :go-velocity 0.2 0 0)

  (warn "input following command to stop walking~% (go-stop-with-abc)~%")
  )

;; go-pos,go-vel,go-stop keeping auto balancer
(defun go-pos-with-abc
  (xx yy th &key (wait t))
  (send *ri* :go-pos-no-wait xx yy th)
  (when wait (send *ri* :wait-foot-steps)))
(defun go-vel-with-abc
  (xx yy th)
  (send *ri* :go-velocity xx yy th))
(defun go-stop-with-abc
  ()
  (send *ri* :autobalancerservice_goStop))

