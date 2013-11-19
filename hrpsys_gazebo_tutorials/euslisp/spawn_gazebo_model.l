(ros::roseus "spawn_robot_model")

(load "package://hrpsys_gazebo_tutorials/euslisp/gazebo-msgs-util.l")

(when (ros::has-param "~robot_name")
  (setq *robot_name* (ros::get-param "~robot_name")))

(when (ros::has-param "~model_file")
  (setq *model_file* (ros::get-param "~model_file")))

(setq *translate* (float-vector 0 0 0))
(when (ros::has-param "~translate_x")
  (setf (elt *translate* 0) (* 1000 (ros::get-param "~translate_x"))))
(when (ros::has-param "~translate_y")
  (setf (elt *translate* 1) (* 1000 (ros::get-param "~translate_y"))))
(when (ros::has-param "~translate_z")
  (setf (elt *translate* 2) (* 1000 (ros::get-param "~translate_z"))))

(gazebo-spawn-model *robot_name* (read-xml-from-file *model_file*)
                    :initial-pose (make-coords :pos *translate*)
                    :wait t)

;;(gazebo-unpose-physics)

(exit 0)
