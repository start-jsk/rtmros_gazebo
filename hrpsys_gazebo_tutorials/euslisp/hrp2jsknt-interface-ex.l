(load "package://jsk_hrpsys_ros_bridge/euslisp/hrp2jsknt-interface.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-utils.l")

(load "package://hrpsys_gazebo_tutorials/euslisp/hand-command-publisher.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/hrp2jsknt-with-limb-trajectory.l")

(defun init
  (&key (real t))
  (cond
   (real
    (hrp2jsknt-init)
    (send *ri* :init-hand))
   (t
    (if (not (boundp '*hrp2jsknt*))
	(setq *hrp2jsknt* (instance hrp2jsknt-robot :init)))))
  (setq *hrp2* *hrp2jsknt*)
  (setq *robot* *hrp2jsknt*)
  (objects (list *hrp2*))
  (send *irtviewer* :change-background #f(1 1 1)))

(defun reset-pose
  (&key (wait-time 2000) (wait? t))
  (send-special-pose :reset-pose :wait-time wait-time :wait? wait?))

(defun reset-manip-pose
  (&key (wait-time 2000) (wait? t))
  (send-special-pose :reset-manip-pose :wait-time wait-time :wait? wait?))

(defun send-special-pose
  (pose &key (wait-time 2000) (wait? t))
  (send *ri* :angle-vector (send *robot* pose) wait-time)
  (when wait? (send *ri* :wait-interpolation)))

