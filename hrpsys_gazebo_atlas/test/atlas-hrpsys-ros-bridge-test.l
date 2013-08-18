#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/test/hrpsys-ros-bridge-test-suits.l")

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

(deftest test-walk-simple
  (assert (tmp-test-walk-simple *atlas* :pose-func #'(lambda () (send *atlas* :walk-pose)))))

(warn ";; (run-hrpsys-ros-bridge-test \"atlas\" :robot-model-pkg-name \"hrpsys_gazebo_atlas\") ;; run-all-tests~%")
(warn ";; (hrpsys-ros-bridge-test-debug-mode \"atlas\" :robot-model-pkg-name \"hrpsys_gazebo_atlas\") ;; setup *ri* and robot without error exit~%")