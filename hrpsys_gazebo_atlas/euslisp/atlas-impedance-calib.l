;;(load "package://hrpsys_ros_bridge_tutorials/test/hrpsys-ros-bridge-test-suits.l")
(load "package://hrpsys_ros_bridge/euslisp/calib-force-sensor-params.l")

(defun calibrate-imp-offset-atlas
  (&key (calib-poses (make-default-ForceCalibPoses *atlas*)))
  (let ((limbs '(:rarm :larm)))
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
