(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::roseus "get-pointcloud")

;;(setq topic-name "/openni/depth_registered/points_throttle")
;;(setq topic-name "/bw_pcl_nodelet/voxelgrid/output")
(setq topic-name "/pcl_divider_nodelet/voxelgrid1/output")
;;(setq topic-name "/bw_pcl_nodelet/voxelgrid/output")

(defun get-callback (msg)
  (print "get-pointcloud")
  (setq *eus-pc* (make-eus-pointcloud-from-ros-msg msg :remove-nan t))
  (format t "~A points" (car (array-dimensions (send *eus-pc* :points))))
  (objects (list *eus-pc*))
)

(defun main
  ()
  (ros::subscribe topic-name
		  sensor_msgs::PointCloud2 #'get-callback)
  (ros::rate 1)
  (do-until-key
   (print "loop")
   (ros::spin-once)
   (x::window-main-one)
   (ros::sleep)
   (unless (ros::ok) (return)))
  )

(main)


