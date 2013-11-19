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

;;
;; wrench service
;;
(defun gazebo-apply-body-wrench (body_name wrench &key (reference_frame "")
                                           (start_time) (duration)
                                           (reference_point))
  (let ((srv (instance gazebo_msgs::ApplyBodyWrenchRequest :init
                       :body_name body_name
                       :reference_frame reference_frame
                       :reference_point reference_point
                       :wrench wrench
                       :start_time start_time
                       :ducation duration
                       )))
    (ros::service-call "/gazebo/apply_body_wrench" srv)))
(defun clear_body_wrenches (body_name)
  (let ((srv (instance gazebo_msgs::BodyRequestRequest :init
                       :body_name body_name)))
    (ros::service-call "/gazebo/clear_body_wrench" srv)))

;;
;; joint service
;;
(defun gazebo-apply-joint-effort (joint_name effort &key (start_time) (duration))
  (let ((srv (instance gazebo_msgs::ApplyJointEffortRequest :init
                       :joint_name joint_name
                       :effort effort
                       :start_time start_time
                       :ducation duration
                       )))
    (ros::service-call "/gazebo/apply_joint_effort" srv)))
(defun clear_body_wrenches (joint_name)
  (let ((srv (instance gazebo_msgs::JointRequestRequest :init
                       :joint_name joint_name)))
    (ros::service-call "/gazebo/clear_joint_forces" srv)))

;;
;; set properties service
;;
(defun gazebo-set-joint-properties (joint_name)
  (let ((srv (instance gazebo_msgs::SetJointPropertiesRequest :init
                       :joint_name joint_name
                       ;; :ode_joint_config (instance gazebo_msgs::ODEJointProperties :init)
                       )))
    (ros::service-call "/gazebo/set_joint_properties" srv)))
(defun gazebo-set-link-properties (link_name)
  (let ((srv (instance gazebo_msgs::SetLinkPropertiesRequest :init
                       :link_name link_name
                       ;; :com (ros::coords->tf-pose cds)
                       ;; :gravity_mode
                       ;; :mass
                       ;; :ixx
                       ;; :ixy
                       ;; :ixz
                       ;; :iyy
                       ;; :iyz
                       ;; :izz
                       )))
    (ros::service-call "/gazebo/set_link_properties" srv)))
(defun gazebo-set-physics-properties ()
  (let ((srv (instance gazebo_msgs::SetPhysicsPropertiesRequest :init
                       ;; :time_step
                       ;; :max_update_rate
                       ;; :gravity
                       ;; :ode_config (instance gazebo_msgs::ODEPhysics :init)
                       )))
    (ros::service-call "/gazebo/set_physics_properties" srv)))

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

;;
;; get state service
;;
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
;; set state service
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
  (let ((rpath (if (substringp "://" path)
                   (ros::resolve-ros-path path) path))
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


;; hrpsys_gazebo_general
(defun gazebo-reset-joint-reference (robotname &optional (wait nil))
  (let ((sname (format nil "~A/reset_joint_reference" robotname))
        (srv (instance std_srvs::EmptyRequest :init)))
    (if wait (ros::wait-for-service sname))
    (ros::service-call sname srv)))

#|
(ros::roseus "gazebomsgtest")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")
(samplerobot)
(send *samplerobot* :reset-pose)

(gazebo-pause-physics)

(gazebo-spawn-model "SampleRobot" (read-xml-from-file "package://SampleRobot/SampleRobot.urdf")
                    :initial-pose (make-coords :pos #f(0 0 1000)))
(gazebo-get-model-properties "SampleRobot")

(gazebo-set-model-configuration "SampleRobot" :joint-list (send *samplerobot* :joint-list))
(gazebo-reset-joint-reference)
(gazebo-set-model-state "SampleRobot" (make-coords :pos #f(0 0 760)))

(gazebo-unpause-physics)
|#