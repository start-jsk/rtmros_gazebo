(defmethod hrp2jsknt-interface
  (:init
   ()
   (prog1 (send-super :init :robot hrp2jsknt-robot)
     ;; add controllers
     (dolist (l (list
		 (cons :larm-controller "larm_controller/follow_joint_trajectory_action")
		 (cons :rarm-controller "rarm_controller/follow_joint_trajectory_action")
		 (cons :lleg-controller "lleg_controller/follow_joint_trajectory_action")
		 (cons :rleg-controller "rleg_controller/follow_joint_trajectory_action")
		 (cons :head-controller "head_controller/follow_joint_trajectory_action")
		 (cons :torso-controller "torso_controller/follow_joint_trajectory_action")))
       (let ((type (car l))
	     (name (cdr l))
	     action)
	 (setq action (find-if #'(lambda (ac) (string= name (send ac :name)))
			       controller-actions))
	 (setf (gethash type controller-table) (list action))
	 ))))
  ;; controller setting
  (:larm-controller
   ()
   (list
    (list
     (cons :controller-action "larm_controller/follow_joint_trajectory_action")
     (cons :controller-state "larm_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "LARM_JOINT0" "LARM_JOINT1" "LARM_JOINT2" "LARM_JOINT3" "LARM_JOINT4" "LARM_JOINT5" "LARM_JOINT6")))))
  (:rarm-controller
   ()
   (list
    (list
     (cons :controller-action "rarm_controller/follow_joint_trajectory_action")
     (cons :controller-state "rarm_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "RARM_JOINT0" "RARM_JOINT1" "RARM_JOINT2" "RARM_JOINT3" "RARM_JOINT4" "RARM_JOINT5" "RARM_JOINT6")))))
  (:lleg-controller
   ()
   (list
    (list
     (cons :controller-action "lleg_controller/follow_joint_trajectory_action")
     (cons :controller-state "lleg_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "LLEG_JOINT0" "LLEG_JOINT1" "LLEG_JOINT2" "LLEG_JOINT3" "LLEG_JOINT4" "LLEG_JOINT5" "LLEG_JOINT6")))))
  (:rleg-controller
   ()
   (list
    (list
     (cons :controller-action "rleg_controller/follow_joint_trajectory_action")
     (cons :controller-state "rleg_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "RLEG_JOINT0" "RLEG_JOINT1" "RLEG_JOINT2" "RLEG_JOINT3" "RLEG_JOINT4" "RLEG_JOINT5" "RLEG_JOINT6")))))
  (:head-controller
   ()
   (list
    (list
     (cons :controller-action "head_controller/follow_joint_trajectory_action")
     (cons :controller-state "head_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "HEAD_JOINT0" "HEAD_JOINT1")))))
  (:torso-controller
   ()
   (list
    (list
     (cons :controller-action "torso_controller/follow_joint_trajectory_action")
     (cons :controller-state "torso_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "CHEST_JOINT0" "CHEST_JOINT1")))))
  (:default-controller
   ()
   (append
    (send self :larm-controller)
    (send self :rarm-controller)
    (send self :lleg-controller)
    (send self :rleg-controller)
    (send self :head-controller)
    (send self :torso-controller)
    )))
