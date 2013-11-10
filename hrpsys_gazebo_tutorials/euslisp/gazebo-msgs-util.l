(load-ros-manifest "gazebo_msgs")

#|
topic
"/gazebo/set_link_state" gazebo_msgs::LinkState
"/gazebo/set_model_state" gazebo_msgs::ModelState
|#
(defun get-link-states ()
  (one-shot-subscribe "/gazebo/link_states" gazebo_msgs::LinkStates :timeout 1.0))
(defun get-model-states ()
  (one-shot-subscribe "/gazebo/model_states" gazebo_msgs::ModelStates :timeout 1.0))

#|
service
"/gazebo/apply_body_wrench" gazebo_msgs::ApplyBodyWrench
"/gazebo/apply_joint_effort" gazebo_msgs::ApplyJointEffort

"/gazebo/clear_body_wrenches" gazebo_msgs::BodyRequest
"/gazebo/clear_joint_forces" gazebo_msgs::JointRequest

"/gazebo/get_link_state" gazebo_msgs::GetLinkState
"/gazebo/get_model_state" gazebo_msgs::GetModelState

"/gazebo/set_link_state" gazebo_msgs::SetLinkState
"/gazebo/set_model_state" gazebo_msgs::SetModelState

"/gazebo/get_joint_properties" gazebo_msgs::GetJointProperties
"/gazebo/get_link_properties" gazebo_msgs::GetLinkProperties

"/gazebo/set_joint_properties" gazebo_msgs::SetJointProperties
"/gazebo/set_link_properties" gazebo_msgs::SetLinkProperties

"/gazebo/get_physics_properties" gazebo_msgs::GetPhysicsProperties
"/gazebo/set_physics_properties" gazebo_msgs::SetPhysicsProperties

"/gazebo/get_world_properties" gazebo_msgs::GetWorldProperties

"/gazebo/get_model_properties" gazebo_msgs::GetModelProperties

"/gazebo/set_model_configuration" gazebo_msgs::SetModelConfiguration
|#

#|
"/gazebo/spawn_gazebo_model" gazebo_msgs::SpawnModel
"/gazebo/spawn_sdf_model" gazebo_msgs::SpawnModel
"/gazebo/spawn_urdf_model" gazebo_msgs::SpawnModel

"/gazebo/delete_model" gazebo_msgs::DeleteModel
|#
(defun gazebo-pause-physics ()
  (let ((srv (instance std_srvs::EmptyRequest :init)))
    (ros::service-call "/gazebo/pause_physics" srv)))
(defun gazebo-pause-physics ()
  (let ((srv (instance std_srvs::EmptyRequest :init)))
    (ros::service-call "/gazebo/unpause_physics" srv)))
(defun gazebo-reset-simulation ()
  (let ((srv (instance std_srvs::EmptyRequest :init)))
    (ros::service-call "/gazebo/reset_simulation" srv)))
(defun gazebo-reset-world ()
  (let ((srv (instance std_srvs::EmptyRequest :init)))
    (ros::service-call "/gazebo/reset_world" srv)))

#|
    def on_servo_on(self, evt):
      servo = rospy.ServiceProxy("/RobotHardwareServiceROSBridge/servo", OpenHRP_RobotHardwareService_servo )
      power = rospy.ServiceProxy("/RobotHardwareServiceROSBridge/power", OpenHRP_RobotHardwareService_power )
      actual = rospy.ServiceProxy("/StateHolderServiceROSBridge/goActual", OpenHRP_StateHolderService_goActual )
      try:
        power(OpenHRP_RobotHardwareService_powerRequest("all",0))
        time.sleep(1)
        actual(OpenHRP_StateHolderService_goActualRequest())
        time.sleep(2)
        servo(OpenHRP_RobotHardwareService_servoRequest("all",0))
      except rospy.ServiceException, e:
        wx.MessageBox("Failed to put the hrpsys in servo on mode: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)

    def on_servo_off(self, evt):
      servo = rospy.ServiceProxy("/RobotHardwareServiceROSBridge/servo", OpenHRP_RobotHardwareService_servo )
      power = rospy.ServiceProxy("/RobotHardwareServiceROSBridge/power", OpenHRP_RobotHardwareService_power )
      try:
        servo(OpenHRP_RobotHardwareService_servoRequest("all",1));
        time.sleep(1)
        power(OpenHRP_RobotHardwareService_powerRequest("all",1))
      except rospy.ServiceException, e:
        wx.MessageBox("Failed to put the hrpsys in servo off mode: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)
|#
