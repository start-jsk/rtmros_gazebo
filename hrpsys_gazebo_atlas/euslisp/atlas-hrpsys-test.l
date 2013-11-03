(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")


;; validation function ;;;;;;;;;;;;;;;;;;;;;;;
(defun init-hrpsys-test
  ()

  (atlas-init-ex)
  (fix-leg)
  (warning-message 1 "set servo-on by dashboard~%")
  (my-wait)
  ;; (send *ri* :start-auto-balancer)
  (set-user)

  (warning-message 3 "followings are test function~%")
  (warning-message 3 "  (test-upper-angle)~%")
  (warning-message 3 "  (test-lower-angle)~%")
  )


(defun test-upper-angle
  (&key (real nil))

  (warning-message 1 "test-upper-angle~%")

  (warning-message 1 "[test-uppper-angle] reset-pose~%")
  (send *atlas* :reset-pose)
  (send *irtviewer* :draw-objects)
  (when real (model2real-safe) (draw-real-robot))
  (my-wait)
  
  (warning-message 1 "[test-uppper-angle] move shoulder-y~%")
  (send *atlas* :reset-pose)
  (send *atlas* :rarm :shoulder-y :joint-angle -20 :relative t)
  (send *irtviewer* :draw-objects)
  (when real (model2real-safe) (draw-real-robot))
  (my-wait)

  (warning-message 1 "[test-uppper-angle] move shoulder-r~%")
  (send *atlas* :reset-pose)
  (send *atlas* :rarm :shoulder-y :joint-angle -20 :relative t)
  (send *atlas* :rarm :shoulder-r :joint-angle -10 :relative t)
  (send *irtviewer* :draw-objects)
  (when real (model2real-safe) (draw-real-robot))
  (my-wait)

  (warning-message 1 "[test-uppper-angle] move elbow-p~%")
  (send *atlas* :reset-pose)
  (send *atlas* :rarm :shoulder-y :joint-angle -20 :relative t)
  (send *atlas* :rarm :shoulder-r :joint-angle -10 :relative t)
  (send *atlas* :rarm :elbow-p :joint-angle -20 :relative t)
  (send *irtviewer* :draw-objects)
  (when real (model2real-safe) (draw-real-robot))
  (my-wait)
  
  (warning-message 1 "[test-uppper-angle] move elbow-p~%")
  (send *atlas* :reset-manip-pose)
  (send *irtviewer* :draw-objects)
  (when real (model2real-safe) (draw-real-robot))
  (my-wait)

  (warning-message 1 "[test-all-angle] reset-pose~%")
  (send *atlas* :reset-pose)
  (send *irtviewer* :draw-objects)
  (when real (model2real-safe) (draw-real-robot))
  )


(defun test-lower-angle
  (&key (real nil))

  (warning-message 1 "test-lower-angle~%")

  (warning-message 1 "[test-lower-angle] reset-pose~%")
  (send *atlas* :reset-pose)
  (send *irtviewer* :draw-objects)
  (when real (model2real-safe) (draw-real-robot))
  (my-wait)
  
  (warning-message 1 "[test-lower-angle] walk-pose~%")
  (send *atlas* :walk-pose)
  (send *irtviewer* :draw-objects)
  (when real (model2real-safe) (draw-real-robot))
  (my-wait)

  (warning-message 1 "[test-lower-angle] move left leg above~%")
  (send *atlas* :walk-pose)
  (send *atlas* :lleg :move-end-pos #f(0 0 100))
  (send *irtviewer* :draw-objects)
  (when real (model2real-safe) (draw-real-robot))
  (my-wait)
  
  (warning-message 1 "[test-lower-angle] move left leg outer~%")
  (send *atlas* :walk-pose)
  (send *atlas* :lleg :move-end-pos #f(0 100 100))
  (send *irtviewer* :draw-objects)
  (when real (model2real-safe) (draw-real-robot))
  (my-wait)

  (warning-message 1 "[test-lower-angle] move left leg yaw direction~%")
  (send *atlas* :walk-pose)
  (send *atlas* :lleg :move-end-pos #f(0 100 100))
  (send *atlas* :lleg :move-end-rot 30 :z)
  (send *irtviewer* :draw-objects)
  (when real (model2real-safe) (draw-real-robot))
  (my-wait)

  (warning-message 1 "[test-lower-angle] move both leg above~%")
  (send *atlas* :walk-pose)
  (send *atlas* :legs :move-end-pos #f(0 0 100))
  (send *irtviewer* :draw-objects)
  (when real (model2real-safe) (draw-real-robot))
  (my-wait)

  (warning-message 1 "[test-lower-angle] reset-pose~%")
  (send *atlas* :reset-pose)
  (send *irtviewer* :draw-objects)
  (when real (model2real-safe) (draw-real-robot))
  )


(defun test-force-sensor
  ()

  (warning-message 1 "test-force-sensor~%")

  (my-wait)
  (warning-message 1 "print arm force sensor~%")
  (do-until-key
   (unix::usleep (* 1000 1000))
   (warning-message 2 "left: ~a  right: ~a ~%"
		    (send *ri* :state :absolute-force-vector :larm)
		    (send *ri* :state :absolute-force-vector :rarm)))

  (my-wait)
  (warning-message 1 "print leg force sensor~%")
  (do-until-key
   (unix::usleep (* 1000 1000))
   (warning-message 2 "left: ~a  right: ~a~%"
		    (send *ri* :state :absolute-force-vector :lleg)
		    (send *ri* :state :absolute-force-vector :rleg)))
  )


(defun test-imu-sensor
  ()

  (warning-message 1 "test-imu-sensor~%")

  (my-wait)
  (warning-message 1 "apply hrpsys imu value to the eus model~%")
  (move-robot-with-imu :loop-p t)
  (fix-leg))


(defun test-auto-balancer-balance
  (&key (real nil))

  (warning-message 1 "test-auto-balancer~%")

  (warning-message 1 "check that the robot is in the air~%")
  (y-or-n-p)

  (warning-message 1 "send reset-manip-pose with bending knees~%")
  (my-wait)
  (send *atlas* :reset-manip-pose)
  (send *atlas* :legs :move-end-pos #f(0 0 100))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t :time 3000))

  (warning-message 1 "call (send *ri* :start-auto-balancer)~%")
  (my-wait)
  (when real (send *ri* :start-auto-balancer))

  (warning-message 1 "move the robot on the ground~%")
  (y-or-n-p)

  (warning-message 1 "move waist in the loop~%")
  (y-or-n-p)
  (let ((cnt 0))
    (do-until-key
     (incf cnt)
     (send *atlas* :reset-manip-pose)
     (send *atlas* :legs :move-end-pos #f(0 0 100))
     (send *atlas* :torso :waist-y :joint-angle (+ (* 25 (sin (/ cnt 2.0)))))
     (send *atlas* :torso :waist-p :joint-angle (+ (abs (* 25 (sin (/ cnt 2.0))))))
     (send *irtviewer* :draw-objects)
     (if real
	 (model2real :wait t :time 3000)
       (unix::usleep (* 100 1000)))))
  (send *atlas* :reset-manip-pose)
  (send *atlas* :legs :move-end-pos #f(0 0 100))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t :time 5000))

  (warning-message 1 "move legs in the loop~%")
  (y-or-n-p)
  (let ((cnt 0))
    (do-until-key
     (incf cnt)
     (send *atlas* :reset-manip-pose)
     (send *atlas* :legs :move-end-pos #f(0 0 100))
     (send *atlas* :legs :move-end-pos (float-vector 0 0 (+ 100 (* 25 (sin (/ cnt 2.0))))))
     (send *irtviewer* :draw-objects)
     (if real
	 (model2real :wait t :time 5000)
       (unix::usleep (* 100 1000)))))
  (send *atlas* :reset-manip-pose)
  (send *atlas* :legs :move-end-pos #f(0 0 100))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t :time 5000))

  (warning-message 1 "move left arm in the loop~%")
  (y-or-n-p)
  (let ((cnt 0))
    (do-until-key
     (incf cnt)
     (send *atlas* :reset-manip-pose)
     (send *atlas* :legs :move-end-pos #f(0 0 100))
     (send *atlas* :larm :inverse-kinematics
	   (send (send *atlas* :larm :end-coords :copy-worldcoords) :translate (float-vector (abs (* 400 (sin (/ cnt 6.0)))) 0 0) :world) :rotation-axis nil :revert-if-fail nil)
     (send *irtviewer* :draw-objects)
     (if real
	 (model2real :wait t :time 4000)
       (unix::usleep (* 100 1000)))))
  (send *atlas* :reset-manip-pose)
  (send *atlas* :legs :move-end-pos #f(0 0 100))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t :time 5000))

  (warning-message 1 "move both arms in the loop~%")
  (y-or-n-p)
  (let ((cnt 0))
    (do-until-key
     (incf cnt)
     (send *atlas* :reset-manip-pose)
     (send *atlas* :legs :move-end-pos #f(0 0 100))
     (send *atlas* :larm :inverse-kinematics
	   (send (send *atlas* :larm :end-coords :copy-worldcoords) :translate (float-vector (abs (* 400 (sin (/ cnt 6.0)))) 0 0) :world) :rotation-axis nil :revert-if-fail nil)
     (send *atlas* :rarm :inverse-kinematics
	   (send (send *atlas* :rarm :end-coords :copy-worldcoords) :translate (float-vector (abs (* 400 (sin (/ cnt 6.0)))) 0 0) :world) :rotation-axis nil :revert-if-fail nil)
     (send *irtviewer* :draw-objects)
     (if real
	 (model2real :wait t :time 4000)
       (unix::usleep (* 100 1000)))))
  (send *atlas* :reset-manip-pose)
  (send *atlas* :legs :move-end-pos #f(0 0 100))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t :time 5000))
  )


(defun test-auto-balancer-walk
  ()

  (warning-message 1 "test-auto-balancer-walk~%")

  (warning-message 1 "check that the robot is in the air~%")
  (y-or-n-p)

  (warning-message 1 "call (send *ri* :start-auto-balancer)~%")
  (my-wait)
  (when real (send *ri* :start-auto-balancer))

  (warning-message 1 "send reset-manip-pose with bending knee~%")
  (my-wait)
  (send *atlas* :reset-manip-pose)
  (send *atlas* :legs :move-end-pos #f(0 0 100))
  (model2real :time 5000)

  (warning-message 1 "call (send *ri* :go-pos 0 0 0)~%")
  (y-or-n-p)
  (send *ri* :go-pos 0 0 0)

  (warning-message 1 "call (send *ri* :go-pos 1 0 0)~%")
  (y-or-n-p)
  (send *ri* :go-pos 1 0 0)

  (warning-message 1 "check that the robot on the ground~%")
  (y-or-n-p)

  (warning-message 1 "call (send *ri* :go-pos 0 0 0)~%")
  (y-or-n-p)
  (send *ri* :go-pos 0 0 0)

  (warning-message 1 "call (send *ri* :go-velociy 0 0 0)~%")
  (y-or-n-p)
  (send *ri* :go-velocity 0 0 0)
  (my-wait)
  (send *ri* :go-stop)
  )


(defun test-auto-balancer-old
  (&key (real nil))

  (warning-message 1 "test-auto-balancer~%")

  (warning-message 1 "check that the robot is in the air~%")
  (y-or-n-p)

  (my-wait)
  (warning-message 1 "call (send *ri* :start-auto-balancer)~%")
  (when real (send *ri* :start-auto-balancer))
  
  (my-wait)
  (warning-message 1 "send reset-manip-pose with bending knees~%")
  (send *atlas* :reset-manip-pose)
  (send *atlas* :legs :move-end-pos #f(0 0 100))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t :time 3000))

  (my-wait)
  (warning-message 1 "call (send *ri* :stop-auto-balancer)~%")
  (when real (send *ri* :stop-auto-balancer))

  (my-wait)
  (warning-message 1 "send reset-pose~%")
  (send *atlas* :reset-pose)
  (send *atlas* :legs :move-end-pos #f(0 0 100))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t :time 3000))

  (my-wait)
  (warning-message 1 "call (send *ri* :start-auto-balancer)~%")
  (when real (send *ri* :start-auto-balancer))
  
  (my-wait)
  (warning-message 1 "send reset-pose with bending knees~%")
  (send *atlas* :reset-pose)
  (send *atlas* :legs :move-end-pos #f(0 0 100))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t :time 3000))

  (warning-message 1 "move the robot on the ground~%")
  (y-or-n-p)
  )


(defun test-impedance
  ()

  (warning-message 1 "test-impedance~%")

  (warning-message 1 "call (send *ri* :stop-auto-balancer)~%")
  (my-wait)
  (send *ri* :stop-auto-balancer)

  (warning-message 1 "call (send *ri* :start-impedance :larm)~%")
  (my-wait)
  (send *ri* :start-impedance :larm)

  (warning-message 1 "set ref-force #f(-10 0 0)~%")
  (my-wait)
  (send *ri* :set-impedance-controller-param :larm :ref-force #f(-10 0 0))

  (my-wait)
  (warning-message 1 "call (send *ri* :stop-impedance :larm)~%")
  (send *ri* :stop-impedance :larm)
  )


(defun test-st
  ()

  (warning-message 1 "test-st~%")

  (warning-message 1 "call (send *ri* :start-auto-balancer)~%")
  (my-wait)
  (when real (send *ri* :start-auto-balancer))

  (warning-message 1 "send reset-manip-pose with bending knee~%")
  (my-wait)
  (send *atlas* :reset-manip-pose)
  (send *atlas* :legs :move-end-pos #f(0 0 100))
  (model2real :time 5000)

  (warning-message 1 "check that the robot on the ground~%")
  (y-or-n-p)

  (warning-message 1 "start st~%")
  (my-wait)
  (send *ri* :set-st-param
	:k-tpcc-p #f(0 0)
	:k-tpcc-x #f(5 5)
	:k-brot-p #f(5 5)
	:k-brot-tc #f(0.1 0.1))
  (send *ri* :start-st)

  (warning-message 1 "chagne :k-tpcc-p~%")
  (warn "(send *ri* :set-st-param
	:k-tpcc-p #f(0 0)
	:k-tpcc-x #f(5 5)
	:k-brot-p #f(5 5)
	:k-brot-tc #f(0.1 0.1))~%")

  )


(defun test-reach-ground
  (&key (real nil))

  (warning-message 1 "test-reach-ground~%")

  (warning-message 1 "check that the robot is in the air~%")
  (y-or-n-p)

  (send *robot* :reset-pose)
  (send *robot* :newcoords (make-coords))
  (send *irtviewer* :draw-objects)
  (when real (model2real-safe :time 10000))

  (dolist (h (list -300 -500 -700))
    (send *robot* :fullbody-inverse-kinematics
	  (list
	   (make-coords :pos (float-vector 700 -100 h))
	   (copy-object (send *robot* :rleg :end-coords :worldcoords))
	   (copy-object (send *robot* :lleg :end-coords :worldcoords)))
	  :rotation-axis (list nil t t)
	  :move-target
	  (mapcar
	   #'(lambda (k) (send *robot* k :end-coords))
	   '(:rarm :rleg :lleg))
	  :min #F(-1000 -1000 -1000 -200 -200 -200)
	  :max #F(1000 1000 1000 200 200 200)
	  :root-link-virtual-joint-weight
	  #F(0.01 0.01 1 0.01 0.01 0.01)
	  :revert-if-fail nil
	  :link-list
	  (mapcar
	   #'(lambda (k)
	       (send *robot* :link-list
		     (send (send *robot* k :end-coords) :parent)))
	   '(:rarm :rleg :lleg))
	  :debug-view :no-message
	  )
    (send *irtviewer* :draw-objects)
    (when real (model2real-safe :time 10000)))
  )

(defun test-reach-ground
  (&key (real nil))

  (warning-message 1 "test-reach-ground~%")

  (warning-message 1 "check that the robot is in the air~%")
  (y-or-n-p)

  (send *robot* :reset-pose)
  (send *robot* :newcoords (make-coords))
  (send *irtviewer* :draw-objects)
  (when real (model2real-safe :time 10000))

  (dolist (h (list -300 -500 -700))
    (send *robot* :fullbody-inverse-kinematics
	  (list
	   (make-coords :pos (float-vector 700 -100 h))
	   (copy-object (send *robot* :rleg :end-coords :worldcoords))
	   (copy-object (send *robot* :lleg :end-coords :worldcoords)))
	  :rotation-axis (list nil t t)
	  :move-target
	  (mapcar
	   #'(lambda (k) (send *robot* k :end-coords))
	   '(:rarm :rleg :lleg))
	  :min #F(-1000 -1000 -1000 -200 -200 -200)
	  :max #F(1000 1000 1000 200 200 200)
	  :root-link-virtual-joint-weight
	  #F(0.01 0.01 1 0.01 0.01 0.01)
	  :revert-if-fail nil
	  :link-list
	  (mapcar
	   #'(lambda (k)
	       (send *robot* :link-list
		     (send (send *robot* k :end-coords) :parent)))
	   '(:rarm :rleg :lleg))
	  :debug-view :no-message
	  )
    (send *irtviewer* :draw-objects)
    (when real (model2real-safe :time 10000)))
  )


