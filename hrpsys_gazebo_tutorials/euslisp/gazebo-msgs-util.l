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

;; set configuration
(defun gazebo-set-model-configuration (model_name &key (param_name)
                                                  (joint-list) (name-list) (positions))
  (let ((srv (instance gazebo_msgs::SetModelConfiguration :init
                       :model_name model_name
                       :urdf_param_name param_name
                       :joint_names
                       (if joint-list
                           (mapcar #'(lambda (j) (send j :name)) joint-list)
                         name-list)
                       :joint_positions
                       (if joint-list
                           (map float-vector #'(lambda (j) (send j :joint-angle)) joint-list)
                         positions)
                       )))
    (ros::service-call "/gazebo/set_model_configuration" srv)))

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

#|
"/gazebo/spawn_gazebo_model" gazebo_msgs::SpawnModel
"/gazebo/spawn_sdf_model" gazebo_msgs::SpawnModel
"/gazebo/spawn_urdf_model" gazebo_msgs::SpawnModel

"/gazebo/delete_model" gazebo_msgs::DeleteModel
|#
;;
;; spawn and delete model
;;


;;
;; simulation util
;;
(defun gazebo-pause-physics ()
  (let ((srv (instance std_srvs::EmptyRequest :init)))
    (ros::service-call "/gazebo/pause_physics" srv)))
(defun gazebo-unpause-physics ()
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
