(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::roseus "get-pointcloud")

;;(setq topic-name "/openni/depth_registered/points_throttle")
;;(setq topic-name "/bw_pcl_nodelet/voxelgrid/output")
(setq divide-num 10)
(setq topic-name-list nil)
(dotimes (i divide-num)
  (push (format nil "/pcl_divider_nodelet/voxelgrid~A/output_update" (+ i 1))
	topic-name-list ))
	;;(setq topic-name "/pcl_divider_nodelet/voxelgrid/output_update")

(defun get-callback (msg)
  (print "get-pointcloud")
  (setq hoge msg)
  (setq *eus-pc* (make-eus-pointcloud-from-ros-msg msg :remove-nan t))
  (unless *eus-pc*
    (setq *eus-pc* (instance pointcloud :init)))
  (objects (list *eus-pc*))
)

(defun main
  ()
  (dolist (topic-name topic-name-list)
    ;;(ros::subscribe topic-name
    ;;sensor_msgs::PointCloud2 #'get-callback)
    (ros::subscribe topic-name
		sensor_msgs::PointCloud2 #'get-callback)
    )
  (ros::rate 1)
  (do-until-key
   (print "loop")
   (ros::spin-once)
   (x::window-main-one)
   (ros::sleep)
   (unless (ros::ok) (return)))
  )

(main)


