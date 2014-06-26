;;(load "package://hrpsys_ros_bridge_tutorials/test/hrpsys-ros-bridge-test-suits.l")
(load "package://hrpsys_ros_bridge/euslisp/calib-force-sensor-params.l")

(defun calibrate-imp-offset-atlas
  (&key (calib-poses (make-default-ForceCalibPoses *atlas*))
        (D-p 25) (M-p 2.5)
        (K-r 1.0) (D-r 1) (M-r 0.1))
  (let ((thre 0.5)
        (rthre (deg2rad 0.5))
        (limbs '(:rarm :larm)))
    (mapcar #'(lambda (x)
                (send *ri* :stop-impedance x))
            limbs)
    (unix:system (format nil "rm -f /tmp/atlas-force-moment-offset.l"))
    (forceCalib-for-limbs *atlas*
                          :base-time 2000
                          :fname (format nil "/tmp/atlas-force-moment-offset.l")
                          :poses calib-poses)))


(defun load-imp-offset-atlas
  (&key (filename
         (format nil "~a/euslisp/atlas-force-moment-offset.l" (ros::rospack-find "hrpsys_gazebo_atlas"))))
  (send *ri* :load-forcemoment-offset-param filename))
