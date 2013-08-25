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
  class AddForce : public ModelPlugin
  {

  public:
    // Initialize
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;

      // read option args in sdf tags
      this->link_name = "root";
      if (_sdf->HasElement("linkname")) {
	this->link_name = _sdf->Get<std::string>("linkname");
      }
      this->force = math::Vector3(0, 0, 0);
      if (_sdf->HasElement("force")) {
	this->force = _sdf->Get<math::Vector3>("force");
      }
      this->torque = math::Vector3(0, 0, 0);
      if (_sdf->HasElement("torque")) {
	this->torque = _sdf->Get<math::Vector3>("torque");
      }
      this->position = math::Vector3(0, 0, 0);
      if(_sdf->HasElement("position")) {
	this->position = _sdf->Get<math::Vector3>("position");
      }

      // find root link
      this->link = this->model->GetLink(this->link_name);
      if(!this->link) {
	gzerr << "Root link are not found. (link_name is "<< this->link_name << ")" << std::endl;
	return;
      }
      
      // Make sure the ROS node for Gazebo has already been initialized                                                                                    
      if (!ros::isInitialized()) {
	gzerr << "A ROS node for Gazebo has not been initialized, unable to load plugin. "
	      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)";
	return;
      }
      // ros node
      this->rosNode = new ros::NodeHandle("");
      // ros callback queue for processing subscription
      this->deferredLoadThread = boost::thread(boost::bind(&AddForce::DeferredLoad, this));

      gzmsg << "AddForcePlugin was loaded ! ( force: " << this->force << "  torque: " << this->torque << "  position: " << this->position << ")" << std::endl;
    }

    void DeferredLoad() {
      // ros topic subscribtions
      ros::SubscribeOptions ForceCommandSo =
	ros::SubscribeOptions::create<geometry_msgs::Wrench>("/AddForcePlugin/ForceCommand", 100,
						    boost::bind(&AddForce::SetForceCommand, this, _1),
						    ros::VoidPtr(), &this->rosQueue);
      ros::SubscribeOptions ForcePositionSo =
	ros::SubscribeOptions::create<geometry_msgs::Vector3>("/AddForcePlugin/ForcePosition", 100,
					      boost::bind(&AddForce::SetForcePosition, this, _1),
					      ros::VoidPtr(), &this->rosQueue);
      // Enable TCP_NODELAY because TCP causes bursty communication with high jitter,
      ForceCommandSo.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true);
      this->subForceCommand = this->rosNode->subscribe(ForceCommandSo);
      ForcePositionSo.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true);
      this->subForcePosition = this->rosNode->subscribe(ForcePositionSo);

      // ros callback queue for processing subscription
      this->callbackQueeuThread = boost::thread(boost::bind(&AddForce::RosQueueThread, this));

      // Listen to the update event.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&AddForce::OnUpdate, this, _1));
    }

    void SetForceCommand(const geometry_msgs::Wrench::ConstPtr &_msg)
    {
      this->force.x = _msg->force.x;
      this->force.y = _msg->force.y;
      this->force.z = _msg->force.z;
      this->torque.x = _msg->torque.x;
      this->torque.y = _msg->torque.y;
      this->torque.z = _msg->torque.z;
      gzmsg << "subscribed AddForceCommand. ( force: " << this->force << "  torque: " << this->torque << " )" << std::endl;
    }

    void SetForcePosition(const geometry_msgs::Vector3::ConstPtr &_msg)
    {
      this->position.x = _msg->x;
      this->position.y = _msg->y;
      this->position.z = _msg->z;
      gzmsg << "subscribed AddForcePosition. ( position: " << this->position << " )" << std::endl;
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      this->link->AddRelativeForce(this->force);
      // this->link->AddForceAtRelativePosition(this->force, this->position);
      this->link->AddTorque(this->torque);
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
    std::string link_name;
    math::Vector3 force;
    math::Vector3 torque;
    math::Vector3 position;
    physics::LinkPtr link;
    event::ConnectionPtr updateConnection;

    ros::NodeHandle* rosNode;
    ros::CallbackQueue rosQueue;
    ros::Subscriber subForceCommand;
    ros::Subscriber subForcePosition;
    boost::thread callbackQueeuThread;
    boost::thread deferredLoadThread;
  };

  GZ_REGISTER_MODEL_PLUGIN(AddForce)
}
