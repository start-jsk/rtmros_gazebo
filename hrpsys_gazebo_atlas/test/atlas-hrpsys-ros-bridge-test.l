#!/usr/bin/env roseus

;; (load "package://hrpsys_ros_bridge_tutorials/test/hrpsys-ros-bridge-test-suits.l")

;; test for :angle-vector
(defun tmp-test-seq-angle-vector
  (rob &optional (func))
  (mapcar #'(lambda (av)
              (let ((newav (send rob :angle-vector av)))
                (send *ri* :angle-vector (send rob :angle-vector) 1000)
                (send *ri* :wait-interpolation)
                (if func (setq newav (funcall func)))
                (eps= (distance (send *ri* :state :potentio-vector) newav) 0.0)))
          (list (instantiate float-vector (length (send rob :angle-vector)))
                (send rob :reset-pose)))
  )

(deftest test-seq-angle-vector
  (assert
   (every #'identity
          (tmp-test-seq-angle-vector
           *atlas*
           #'(lambda ()
               (let ((av (send *atlas* :angle-vector)))
                 ;;(mapcar #'(lambda (front back) (setf (elt av back) (elt av front))) '(20 26) '(34 35))
                 (send *atlas* :angle-vector av)))
           ))))

;; (deftest test-force-offset-imp
;;   (assert (every #'identity (tmp-test-force-offset-imp *atlas* (make-default-ForceCalibPoses *atlas*) :M-p 4 :D-p 40))))

;; (deftest test-impedance-spring
;;   (assert (every #'(lambda (x) (< (abs x) 1.0)) (tmp-test-impedance-spring *atlas* (float-vector 350 -550 900)))))

;; test for simple walking of AutoBalancer.rtc by checking whether the robot tumbled or not
(defun tmp-test-walk-simple
  (rob &key (pose-func
             #'(lambda ()
                 (send rob :reset-pose)
                 (send rob :fix-leg-to-coords (make-coords)))))
  (funcall pose-func)
  (send *ri* :angle-vector (send rob :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *ri* :start-auto-balancer)
  (send *ri* :go-pos 0.3 0.1 5)
  (send *ri* :go-velocity 0 0 0)
  (send *ri* :go-stop)
  (send *ri* :stop-auto-balancer)
  (let ((ypr (car (send (send *ri* :state :imucoords) :rpy-angle))))
    (and (< (rad2deg (abs (elt ypr 1))) 10)
         (< (rad2deg (abs (elt ypr 2))) 10))
    ))

(deftest test-walk-simple
  (assert (tmp-test-walk-simple *atlas* :pose-func #'(lambda () (send *atlas* :walk-pose)))))

(warn ";; (run-hrpsys-ros-bridge-test \"atlas\" :robot-model-pkg-name \"hrpsys_gazebo_atlas\") ;; run-all-tests~%")
(warn ";; (hrpsys-ros-bridge-test-debug-mode \"atlas\" :robot-model-pkg-name \"hrpsys_gazebo_atlas\") ;; setup *ri* and robot without error exit~%")
