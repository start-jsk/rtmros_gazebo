(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::roseus "atlas-joint-state-decompresser")
(load "package://hrpsys_gazebo_atlas/euslisp/atlas-joint-state-compresser-settings.l")

(setq js-decompressed-topic-name "/atlas/joint_states_decompressed")
(setq js-compressed-topic-name "/atlas/joint_states_compressed_buffered")


(defun get-joint-name (joint-index)
  (if (> (length joint-name-list) joint-index)
      (elt joint-name-list joint-index)
      nil)
)

(defun decompress-callback (msg)
  (let ((js-decompressed (instance sensor_msgs::JointState :init))
	joint-names joint-name joint-angles
	)
    (setq hoge msg)
    (send js-decompressed :header (send msg :header))
    (dotimes (i (length (send msg :index)))
      (setq joint-name (get-joint-name (elt (send msg :index) i)))
      (when joint-name
	(push joint-name joint-names)
	(push (elt (send msg :position) i) joint-angles)
	)
      )
    (send js-decompressed :name joint-names)
    (send js-decompressed :position (coerce joint-angles float-vector))
    (ros::publish js-decompressed-topic-name js-decompressed)
    (format t "published ~A~%" js-decompressed-topic-name)
    )
  )

(defun main
  ()
  (joint-index-init)
  (ros::advertise js-decompressed-topic-name
		  sensor_msgs::JointState 1)
  (ros::subscribe js-compressed-topic-name 
		  hrpsys_gazebo_atlas::JointStateCompressed #'decompress-callback)

  (ros::rate 10)
  (do-until-key
   (print "loop")
   (ros::spin-once)
   (ros::sleep)
   (unless (ros::ok) (return)))
  )

(main)



