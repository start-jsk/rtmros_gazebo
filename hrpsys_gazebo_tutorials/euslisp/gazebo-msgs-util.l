(ros::load-ros-manifest "gazebo_msgs")

#|
topic
"/gazebo/set_link_state" gazebo_msgs::LinkState
"/gazebo/set_model_state" gazebo_msgs::ModelState
|#
;;
;; get states topic
;;
(defun gazebo-get-link-states (&optional (timeout 1000))
  (one-shot-subscribe "/gazebo/link_states" gazebo_msgs::LinkStates :timeout timeout))
(defun gazebo-get-model-states (&optional (timeout 1000))
  (one-shot-subscribe "/gazebo/model_states" gazebo_msgs::ModelStates :timeout timeout))

#|
service
"/gazebo/apply_body_wrench" gazebo_msgs::ApplyBodyWrench
"/gazebo/apply_joint_effort" gazebo_msgs::ApplyJointEffort

"/gazebo/clear_body_wrenches" gazebo_msgs::BodyRequest
"/gazebo/clear_joint_forces" gazebo_msgs::JointRequest

"/gazebo/set_joint_properties" gazebo_msgs::SetJointProperties
"/gazebo/set_link_properties" gazebo_msgs::SetLinkProperties
"/gazebo/set_physics_properties" gazebo_msgs::SetPhysicsProperties
|#
;;
;; get properties service
;;
(defun gazebo-get-joint-properties (joint_name)
  (let ((srv (instance gazebo_msgs::GetJointPropertiesRequest :init
                       :joint_name joint_name)))
    (ros::service-call "/gazebo/get_joint_properties" srv)))
(defun gazebo-get-link-properties (link_name)
  (let ((srv (instance gazebo_msgs::GetLinkPropertiesRequest :init
                       :link_name link_name)))
    (ros::service-call "/gazebo/get_link_properties" srv)))
(defun gazebo-get-model-properties (model_name)
  (let ((srv (instance gazebo_msgs::GetModelPropertiesRequest :init
                       :model_name model_name)))
    (ros::service-call "/gazebo/get_model_properties" srv)))
(defun gazebo-get-physics-properties ()
  (let ((srv (instance gazebo_msgs::GetPhysicsPropertiesRequest)))
    (ros::service-call "/gazebo/get_physics_properties" srv)))
(defun gazebo-get-world-properties ()
  (let ((srv (instance gazebo_msgs::GetWorldPropertiesRequest)))
    (ros::service-call "/gazebo/get_world_properties" srv)))

(defun gazebo-get-link-state (link_name &optional (ref_frame ""))
  (let ((srv (instance gazebo_msgs::GetLinkStateRequest
                       :init :link_name link_name
                       :reference_frame ref_frame)))
    (ros::service-call "/gazebo/get_link_state" srv)))
(defun gazebo-get-model-state (model_name &optional (rel_name ""))
  (let ((srv (instance gazebo_msgs::GetModelStateRequest
                       :init :model_name model_name
                       :relative_entity_name rel_name)))
    (ros::service-call "/gazebo/get_model_state" srv)))

;;
;; get states service
;;
(defun gazebo-set-link-state (link_name link_coords &key (twist (instance geometry_msgs::Twist :init))
                                        (reference-frame ""))
  (let ((srv (instance gazebo_msgs::SetLinkStateRequest :init
                       :link_state
                       (instance gazebo_msgs::LinkState :init
                                 :link_name link_name
                                 :pose (ros::coords->tf-pose link_coords)
                                 :twist twist
                                 :reference_frame reference-frame))))
    (ros::service-call "/gazebo/set_link_state" srv)))
(defun gazebo-set-model-state (model_name model_coords &key (twist (instance geometry_msgs::Twist :init))
                                        (reference-frame ""))
  (let ((srv (instance gazebo_msgs::SetModelStateRequest :init
                       :model_state
                       (instance gazebo_msgs::ModelState :init
                                 :model_name model_name
                                 :pose (ros::coords->tf-pose model_coords)
                                 :twist twist
                                 :reference_frame reference-frame))))
    (ros::service-call "/gazebo/set_model_state" srv)))

;;
;; spawn and delete model
;;
(defun gazebo-spawn-model (model_name
                           model_xml
                           &key (name_space "/") (initial-pose (make-coords))
                           (reference-frame "") (type :urdf) (wait nil))

  (let ((srv (instance gazebo_msgs::SpawnModelRequest :init
                       :model_name model_name
                       :model_xml model_xml
                       :robot_namespace name_space
                       :initial_pose (ros::coords->tf-pose initial-pose)
                       :reference_frame reference-frame))
        res)
    (case type
      (:urdf
       (if wait (ros::wait-for-service "/gazebo/spawn_urdf_model"))
       (setq res (ros::service-call "/gazebo/spawn_urdf_model" srv)))
      (:sdf
       (if wait (ros::wait-for-service "/gazebo/spawn_sdf_model"))
       (setq res (ros::service-call "/gazebo/spawn_sdf_model" srv)))
      (:gazebo
       (if wait (ros::wait-for-service "/gazebo/spawn_gazebo_model"))
       (setq res (ros::service-call "/gazebo/spawn_gazebo_model" srv)))
      (t (ros::ros-error "unknown type ~A" type)))
    res
    ))
(defun gazebo-delete-model (model_name)
  (let ((srv (instance gazebo_msgs::DeleteModelRequest :init
                       :model_name model_name)))
    (ros::service-call "/gazebo/delete_model" srv)))

;; set configuration
(defun gazebo-set-model-configuration (model_name &key (param_name "") (wait nil)
                                                  (joint-list) (name-list) (positions))
  (if wait (ros::wait-for-service "/gazebo/set_model_configuration"))
  (let ((srv (instance gazebo_msgs::SetModelConfigurationRequest :init
                       :model_name model_name
                       :urdf_param_name param_name ;; not using now in gazebo_ros_pkgs
                       :joint_names
                       (if joint-list
                           (mapcar #'(lambda (j) (format nil "~A::~A" model_name (send j :name))) joint-list)
                         name-list)
                       :joint_positions
                       (if joint-list
                           (map float-vector #'(lambda (j) (send j :ros-joint-angle)) joint-list)
                         positions)
                       )))
    (ros::service-call "/gazebo/set_model_configuration" srv)))

;;
;; simulation util
;;
(defun gazebo-pause-physics (&optional (wait nil))
  (if wait (ros::wait-for-service "/gazebo/pause_physics"))
  (let ((srv (instance std_srvs::EmptyRequest :init)))
    (ros::service-call "/gazebo/pause_physics" srv)))
(defun gazebo-unpause-physics (&optional (wait nil))
  (if wait (ros::wait-for-service "/gazebo/unpause_physics"))
  (let ((srv (instance std_srvs::EmptyRequest :init)))
    (ros::service-call "/gazebo/unpause_physics" srv)))
(defun gazebo-reset-simulation () ;; controller not be reseted
  (let ((srv (instance std_srvs::EmptyRequest :init)))
    (ros::service-call "/gazebo/reset_simulation" srv)))
(defun gazebo-reset-world ()
  (let ((srv (instance std_srvs::EmptyRequest :init)))
    (ros::service-call "/gazebo/reset_world" srv)))

(defun hrpsys-servo-on ()
  (send *ri* :RobotHardwareService_power :name "all" :ss 0)
  (unix::usleep (* 1000 1000))
  (send *ri* :RobotHardwareService_goActual)
  (unix::usleep (* 2 1000 1000))
  (send *ri* :RobotHardwareService_servo :name "all" :ss 0)
  )

(defun hrpsys-servo-off ()
  (send *ri* :RobotHardwareService_servo :name "all" :ss 1)
  (unix::usleep (* 1000 1000))
  (send *ri* :RobotHardwareService_power :name "all" :ss 1)
  )

;;
;; euslisp utility
;;
(defun read-xml-from-file (path)
  (let ((rpath (ros::resolve-ros-path path))
        (newline (coerce (list 10) string))
        str)
    (when rpath
      (with-open-file
       (f rpath :direction :input)
       (let (ln)
         (while (setq ln (read-line f nil nil))
           (push ln str))))
      (reduce #'(lambda (x y) (concatenate string y newline x)) str)
      )
    ))
#|
(ros::roseus "gazebomsgtest")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")
(samplerobot)
(send *samplerobot* :reset-pose)

(gazebo-pause-physics)

(gazebo-spawn-model "SampleRobot" (read-xml-from-file "package://SampleRobot/SampleRobot.urdf")
                    :initial-pose (make-coords :pos #f(0 0 800)))
(gazebo-get-model-properties "SampleRobot")

(gazebo-set-model-configuration "SampleRobot" :joint-list (send *samplerobot* :joint-list))
(gazebo-set-model-state "SampleRobot" (make-coords :pos #f(0 0 760)))

(gazebo-unpause-physics)
|#