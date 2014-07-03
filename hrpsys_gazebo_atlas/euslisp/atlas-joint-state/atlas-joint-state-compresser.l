(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::roseus "atlas-joint-state-compresser")
(load "package://hrpsys_gazebo_atlas/euslisp/atlas-joint-state-compresser-settings.l")
(setq js-topic-name "/atlas/joint_states")
(setq js-compressed-topic-name "/atlas/joint_states_compressed")

(setq display-info nil)

(defun get-joint-index (joint-name)
  (cdr (assoc joint-name joint-index-list :key #'car :test #'string=))
)

(defun compress-callback (msg)
  (let (
	(js-compressed (instance sensor_msgs::JointState :init))
	joint-indexes joint-index joint-angles joint-name
	)
    (send js-compressed :header (send msg :header))
    (dotimes (i (length (send msg :name)))
      (setq joint-index (get-joint-index (elt (send msg :name) i)))
      (when joint-index
	(setf (elt js-compressed-list joint-index) (elt (send msg :position) i))
	)
      )
    (send js-compressed :position (coerce js-compressed-list float-vector))
    (ros::publish js-compressed-topic-name js-compressed)
    (if display-info
	(ros::ros-info "joint_states_compressed was published~%")
      )
    )
  )

(defun main
  ()
  (joint-index-init)
  (setq nan (sqrt -1))
  (setq js-compressed-list (make-list (length atlas-joint-name-list) :initial-element nan))

  (ros::subscribe js-topic-name
		  sensor_msgs::JointState #'compress-callback)
  ;;(ros::advertise js-compressed-topic-name hrpsys_gazebo_atlas::JointStateCompressed 1)
  (ros::advertise js-compressed-topic-name sensor_msgs::JointState 1)

  (ros::rate 10)
  (do-until-key
   (ros::spin-once)
   (ros::sleep)
   (unless (ros::ok) (return)))
  )

(main)
