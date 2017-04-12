#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/String.h>


namespace gazebo
{
  class SetVel : public ModelPlugin
  {

  public:
    // Initialize
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;

      // read option args in sdf tags
      this->obj_name = "";
      if (_sdf->HasElement("objname")) {
        this->obj_name = _sdf->Get<std::string>("objname");
      }
      this->link_name = "root";
      if (_sdf->HasElement("linkname")) {
        this->link_name = _sdf->Get<std::string>("linkname");
      }
      this->apply_in_gazebo_loop = true;
      if (_sdf->HasElement("apply_in_gazebo_loop")) {
        if(_sdf->Get<std::string>("apply_in_gazebo_loop") == "false") {
          this->apply_in_gazebo_loop = false;
        }
      }

      // find root link
      this->link = this->model->GetLink(this->link_name);
      if(!this->link) {
        gzerr << "Root link are not found. (link_name is "<< this->link_name << ")" << std::endl;
        return;
      }

      // initialize flag
      this->set_pose_flag = false;
      this->set_vel_flag = false;

      // Make sure the ROS node for Gazebo has already been initialized
      if (!ros::isInitialized()) {
        gzerr << "A ROS node for Gazebo has not been initialized, unable to load plugin. "
              << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)";
        return;
      }
      // ros node
      this->rosNode = new ros::NodeHandle("");
      // ros callback queue for processing subscription
      this->deferredLoadThread = boost::thread(boost::bind(&SetVel::DeferredLoad, this));
    }

    void DeferredLoad() {
      // ros topic subscribtions
      ros::SubscribeOptions VelCommandSo =
        ros::SubscribeOptions::create<geometry_msgs::Twist>("/" + this->obj_name + "/SetVelPlugin/VelCommand", 100,
                                                    boost::bind(&SetVel::SetVelCommand, this, _1),
                                                    ros::VoidPtr(), &this->rosQueue);
      ros::SubscribeOptions PoseCommandSo =
        ros::SubscribeOptions::create<geometry_msgs::Pose>("/" + this->obj_name + "/SetVelPlugin/PoseCommand", 100,
                                              boost::bind(&SetVel::SetPoseCommand, this, _1),
                                              ros::VoidPtr(), &this->rosQueue);
      // Enable TCP_NODELAY because TCP causes bursty communication with high jitter,
      VelCommandSo.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true);
      this->subVelCommand = this->rosNode->subscribe(VelCommandSo);
      PoseCommandSo.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true);
      this->subPoseCommand = this->rosNode->subscribe(PoseCommandSo);

      // ros callback queue for processing subscription
      this->callbackQueeuThread = boost::thread(boost::bind(&SetVel::RosQueueThread, this));

      // Listen to the update event.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SetVel::OnUpdate, this, _1));

      gzmsg << "SetVelPlugin was loaded !" << std::endl;
    }

    void SetVelCommand(const geometry_msgs::Twist::ConstPtr &_msg)
    {
      this->linear_vel.x = _msg->linear.x;
      this->linear_vel.y = _msg->linear.y;
      this->linear_vel.z = _msg->linear.z;
      this->angular_vel.x = _msg->angular.x;
      this->angular_vel.y = _msg->angular.y;
      this->angular_vel.z = _msg->angular.z;
      this->set_vel_flag = true;
      gzmsg << "subscribed SetVelCommand. ( linear: " << this->linear_vel << "  angular: " << this->angular_vel << " )" << std::endl;
      if (!this->apply_in_gazebo_loop) {
        this->model->SetLinearVel(this->linear_vel);
        this->model->SetAngularVel(this->angular_vel);
      }
    }

    void SetPoseCommand(const geometry_msgs::Pose::ConstPtr &_msg)
    {
      this->model->SetLinearVel(math::Vector3(0, 0, 0));
      this->model->SetAngularVel(math::Vector3(0, 0, 0));
      this->pose.Set(math::Vector3(_msg->position.x, _msg->position.y, _msg->position.z),
                     math::Quaternion(_msg->orientation.w, _msg->orientation.x, _msg->orientation.y, _msg->orientation.z));
      this->set_pose_flag = true;
      gzdbg << "subscribed SetPoseCommand. ( position: " << this->pose.pos << "  orientation: " << this->pose.rot << " )" << std::endl;
      if (!this->apply_in_gazebo_loop) {
        // this->model->SetLinkWorldPose(this->pose, this->link);
        this->model->SetWorldPose(this->pose);
      }
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      if (this->apply_in_gazebo_loop) {
        if (this->set_pose_flag) {
          this->model->SetWorldPose(this->pose);
        }
        if (this->set_vel_flag) {
          this->model->SetLinearVel(this->linear_vel);
          this->model->SetAngularVel(this->angular_vel);
        }
      }
    }

    // Ros loop thread function
    void RosQueueThread() {
      static const double timeout = 0.01;

      while (this->rosNode->ok()) {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

  private:
    physics::ModelPtr model;
    std::string obj_name;
    std::string link_name;
    math::Vector3 linear_vel;
    math::Vector3 angular_vel;
    math::Pose pose;
    physics::LinkPtr link;
    event::ConnectionPtr updateConnection;
    bool set_pose_flag;
    bool set_vel_flag;
    bool apply_in_gazebo_loop;

    ros::NodeHandle* rosNode;
    ros::CallbackQueue rosQueue;
    ros::Subscriber subVelCommand;
    ros::Subscriber subPoseCommand;
    boost::thread callbackQueeuThread;
    boost::thread deferredLoadThread;
  };

  GZ_REGISTER_MODEL_PLUGIN(SetVel)
}
