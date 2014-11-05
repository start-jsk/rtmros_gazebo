(ros::load-ros-manifest "sandia_hand_msgs")
(ros::load-ros-manifest "osrf_msgs")
(ros::load-ros-manifest "sensor_msgs")

(defmethod atlas-interface
    (:init-hand
     ()
     (setq hcp (instance sandia-hand-command-publisher-class :init)))
    (:hand-angle-vector
     (av &optional l/r)
     (cond ((or (equal l/r :lhand) (equal l/r :larm))
	    (send hcp :send-command (list av nil)))
	   ((or (equal l/r :rhand) (equal l/r :rarm))
	    (send hcp :send-command (list nil av)))
	   (t
	    (send hcp :send-command av))
	   ))
    (:hand-angle-state-vector
     (&optional l/r)
     (cond ((or (equal l/r :lhand) (equal l/r :larm))
	    (elt (send hcp :get-state) 0))
	   ((or (equal l/r :rhand) (equal l/r :rarm))
	    (elt (send hcp :get-state) 1))
	   (t
	    (send hcp :get-state))
	   ))
    (:hand-effort-vector
     (ef &optional l/r)
     (cond ((or (equal l/r :lhand) (equal l/r :larm))
	    (send hcp :send-effort-command (list ef nil)))
	   ((or (equal l/r :rhand) (equal l/r :rarm))
	    (send hcp :send-effort-command (list nil ef)))
	   (t
	    (send hcp :send-effort-command (list ef ef)))
	   ))
    (:start-grasp
     (&optional l/r (type "cylindrical"))
     (let* ((msg (instance sandia_hand_msgs::SimpleGrasp :init)))
       (send msg :name type)
       (send msg :closed_amount 1)
       (cond ((or (equal l/r :larm) (equal l/r :lhand))
              (ros::publish "/sandia_hands/l_hand/simple_grasp" msg))
             ((or (equal l/r :rarm) (equal l/r :rhand))
              (ros::publish "/sandia_hands/r_hand/simple_grasp" msg))
             (t
              (ros::publish "/sandia_hands/l_hand/simple_grasp" msg)
              (ros::publish "/sandia_hands/r_hand/simple_grasp" msg)))
       ))
    (:stop-grasp
     (&optional l/r)
     (let* ((msg (instance sandia_hand_msgs::SimpleGrasp :init)))
       (send msg :name "cylindrical")
       (send msg :closed_amount 0)
       (cond ((or (equal l/r :larm) (equal l/r :lhand))
              (ros::publish "/sandia_hands/l_hand/simple_grasp" msg))
             ((or (equal l/r :rarm) (equal l/r :rhand))
              (ros::publish "/sandia_hands/r_hand/simple_grasp" msg))
             (t
              (ros::publish "/sandia_hands/l_hand/simple_grasp" msg)
              (ros::publish "/sandia_hands/r_hand/simple_grasp" msg)))))
    )
;;(warn "~%")
;;(warn "(send *ri* (list (float-vector 10 20 30 40 50 60) (float-vector 50 40 30 20 10 0)))")
;;(warn "(send *ri* :get-state)~%")
;;(warn "(send *ri* :hand-angle-vector #f(10 20 30 40 50 60) :larm)")
;;(warn "(send *ri* :get-state :larm)~%")


(defclass sandia-hand-command-publisher-class
  :super propertied-object
  :slots (left-hand-av right-hand-av))
(defmethod sandia-hand-command-publisher-class
  (:init
   ()
   (ros::advertise "/sandia_hands/l_hand/joint_commands" osrf_msgs::JointCommands 1)
   (ros::advertise "/sandia_hands/r_hand/joint_commands" osrf_msgs::JointCommands 1)
   (ros::advertise "/sandia_hands/l_hand/simple_grasp" sandia_hand_msgs::SimpleGrasp 1)
   (ros::advertise "/sandia_hands/r_hand/simple_grasp" sandia_hand_msgs::SimpleGrasp 1)
   (ros::subscribe "/sandia_hands/l_hand/joint_states" sensor_msgs::JointState #'send self :left-hand-cb)
   (ros::subscribe "/sandia_hands/r_hand/joint_states" sensor_msgs::JointState #'send self :right-hand-cb)
   ;; (ros::rate 100)
   )
  (:send-command
   (av-list)
   (let* ((left-av (elt av-list 0))
	  (right-av (elt av-list 1)))
     (setq msg (instance osrf_msgs::JointCommands :init))
     (when left-av
       (send msg :position (concatenate float-vector (mapcar #'deg2rad (concatenate cons left-av))))
       (ros::publish "/sandia_hands/l_hand/joint_commands" msg))
     (when right-av
       (send msg :position (concatenate float-vector (mapcar #'deg2rad (concatenate cons right-av))))
       (ros::publish "/sandia_hands/r_hand/joint_commands" msg)))
   )
  (:send-effort-command
   (ef-list)
   (let* ((left-ef (elt ef-list 0))
	  (right-ef (elt ef-list 1)))
     (setq msg (instance osrf_msgs::JointCommands :init))
     (when left-ef
       (send msg :effort left-ef)
       (ros::publish "/sandia_hands/l_hand/joint_commands" msg))
     (when right-ef
       (send msg :effort right-ef)
       (ros::publish "/sandia_hands/r_hand/joint_commands" msg)))
   )
  (:get-state
   ()
   (ros::spin-once)
   (list left-hand-av right-hand-av))
  (:left-hand-cb
   (msg)
   (setq left-hand-av
	 (concatenate float-vector (mapcar #'rad2deg (concatenate cons (send msg :position)))))
   )
  (:right-hand-cb
   (msg)
   (setq right-hand-av
	 (concatenate float-vector (mapcar #'rad2deg (concatenate cons (send msg :position)))))
   ))
;;(warn "~%")
;;(warn "(setq *hc* (instance sandia-hand-command-publisher-class :init))")
;;(warn "(send *hc* :send-command (list (float-vector 0 0 0 0 0 0) (float-vector 30 30 30 30 30 30)))~%")
;;(warn "(send *hc* :send-command (list nil (float-vector 30 30 30 30 30 30)))~%")





(defun send-hand-pose
  (hpose &optional l/r)
  (cond ((or (equal l/r :lhand) (equal l/r :larm))
	 (send *ri* :hand-angle-vector (send (send *atlas* :lhand) hpose) :larm))
	((or (equal l/r :rhand) (equal l/r :rarm))
	 (send *ri* :hand-angle-vector (send (send *atlas* :rhand) hpose) :rarm))
	(t
	 (send *ri* :hand-angle-vector (send (send *atlas* :lhand) hpose) :larm)
	 (send *ri* :hand-angle-vector (send (send *atlas* :rhand) hpose) :rarm))))

(defun init-hand-pose
  (&optional l/r)
  (send-hand-pose :init-pose l/r))

(defun open-hand-pose
  (&optional l/r)
  (send-hand-pose :open-pose l/r))

(defun hook-hand-pose
  (&optional l/r)
  (send-hand-pose :hook-pose l/r))

(defun grasp-hand-pose
  (&optional l/r)
  (send-hand-pose :grasp-pose l/r))


(defun start-grasp
  (&optional l/r)
  (send *ri* :start-grasp l/r))

(defun stop-grasp
  (&optional l/r)
  (send *ri* :stop-grasp l/r))
