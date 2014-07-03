(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-utils.l")

(load "package://hrpsys_gazebo_tutorials/euslisp/hand-command-publisher.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/hrp2jsknts-with-limb-trajectory.l")

(defun hrp2jsknts-init-ex
  (&key (real t))
  (cond
   (real
    (hrp2jsknts-init)
    (send *ri* :init-hand))
   (t
    (if (not (boundp '*hrp2jsknts*))
	(setq *hrp2jsknts* (instance hrp2jsknts-robot :init)))))
  (setq *hrp2* *hrp2jsknts*)
  (setq *robot* *hrp2jsknts*)
  (objects (list *hrp2*))
  (send *irtviewer* :change-background #f(1 1 1)))

(defun init
  (&key (real t))
  (hrp2jsknts-init-ex :real real))

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

(defun model2real
  ()
  (send *ri* :angle-vector (send *robot* :angle-vector)))