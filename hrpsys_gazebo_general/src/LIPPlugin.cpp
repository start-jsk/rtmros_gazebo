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
#include "hrpsys_gazebo_msgs/LIPState.h"
#include "hrpsys_gazebo_msgs/LIPSetState.h"
#include "hrpsys_gazebo_msgs/LIPSwitchFoot.h"

#include "PubQueue.h"

namespace gazebo
{
  class LIPPlugin : public ModelPlugin
  {

  public:
    // Initialize
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model_ = _parent;

      // Make sure the ROS node for Gazebo has already been initialized
      if (!ros::isInitialized()) {
        gzerr << "A ROS node for Gazebo has not been initialized, unable to load plugin. "
              << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)";
        return;
      }
      // ros node
      this->ros_node_ = new ros::NodeHandle("");
      // ros callback queue for processing subscription
      this->deferred_load_thread_ = boost::thread(boost::bind(&LIPPlugin::DeferredLoad, this));
    }

    void DeferredLoad() {
      // initialize variables
      std::string base_link_name = "LINK0_link";
      std::string mass_link_name = "LINK4_link";
      std::string root_x_joint_name = "JOINT0";
      std::string root_y_joint_name = "JOINT1";
      std::string linear_joint_name = "JOINT2";
      // base_link
      this->base_link_ = this->model_->GetLink(base_link_name);
      if(!this->base_link_) {
        gzerr << "Base link is not found. (link_name is "<< base_link_name << ")" << std::endl;
        return;
      }
      // mass_link
      this->mass_link_ = this->model_->GetLink(mass_link_name);
      if(!this->mass_link_) {
        gzerr << "Mass link is not found. (link_name is "<< mass_link_name << ")" << std::endl;
        return;
      }
      // root_x_joint
      this->root_x_joint_ = this->model_->GetJoint(root_x_joint_name);
      if(!this->root_x_joint_) {
        gzerr << "Root X joint is not found. (joint_name is "<< root_x_joint_name << ")" << std::endl;
        return;
      }
      // root_y_joint
      this->root_y_joint_ = this->model_->GetJoint(root_y_joint_name);
      if(!this->root_y_joint_) {
        gzerr << "Root Y joint is not found. (joint_name is "<< root_y_joint_name << ")" << std::endl;
        return;
      }
      // linear_joint
      this->linear_joint_ = this->model_->GetJoint(linear_joint_name);
      if(!this->linear_joint_) {
        gzerr << "Linear joint is not found. (joint_name is "<< linear_joint_name << ")" << std::endl;
        return;
      }

      // Publish multi queue
      this->pmq_.startServiceThread();
      this->state_pub_queue_ = this->pmq_.addPub<hrpsys_gazebo_msgs::LIPState>();
      this->state_pub_ = this->ros_node_->advertise<hrpsys_gazebo_msgs::LIPState>("LIP/state", 100, true);

      // Initialize service server
      ros::AdvertiseServiceOptions switch_foot_srv_option =
        ros::AdvertiseServiceOptions::create<hrpsys_gazebo_msgs::LIPSwitchFoot> ("LIP/switch_foot", boost::bind(&LIPPlugin::SwitchFootCallback, this, _1, _2),
                                                                                        ros::VoidPtr(), &this->service_queue_);
      this->switch_foot_srv_ = this->ros_node_->advertiseService(switch_foot_srv_option);
      ros::AdvertiseServiceOptions set_state_srv_option =
        ros::AdvertiseServiceOptions::create<hrpsys_gazebo_msgs::LIPSetState> ("LIP/set_state", boost::bind(&LIPPlugin::SetStateCallback, this, _1, _2),
                                                                                        ros::VoidPtr(), &this->service_queue_);
      this->set_state_srv_ = this->ros_node_->advertiseService(set_state_srv_option);

      // Listen to the update event
      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&LIPPlugin::OnUpdate, this, _1));
      this->service_callback_thread_ = boost::thread(boost::bind(&LIPPlugin::ServiceCallbackThread, this));

      gzmsg << "LIPPlugin was loaded !" << std::endl;
    }

    void ServiceCallbackThread() {
      while (this->ros_node_->ok()) {
        this->service_queue_.callAvailable();
      }
    }

    bool SwitchFootCallback(hrpsys_gazebo_msgs::LIPSwitchFootRequest &req, hrpsys_gazebo_msgs::LIPSwitchFootResponse &res)
    {
      math::Pose base_pose(req.command.foot_position.x, req.command.foot_position.y, req.command.foot_position.z, 0, 0, 0);
      math::Pose mass_pose;
      math::Vector3 mass_velocity(req.command.mass_velocity.x, req.command.mass_velocity.y, req.command.mass_velocity.z);

      mass_pose = this->mass_link_->GetWorldPose();
      if (req.command.keep_mass_velocity) {
        mass_velocity = this->mass_link_->GetWorldLinearVel();
      }

      ROS_INFO_STREAM_THROTTLE(1.0, "[LIP command]  switch foot command" << std::endl
                               << "  foot_pos: " << base_pose.pos << std::endl
                               << "  mass_vel: (keep: " << bool(req.command.keep_mass_velocity) << ") " << mass_velocity);

      // reset force and velocity
      this->linear_joint_->SetForce(0, 0);
      this->root_x_joint_->SetVelocity(0, 0);
      this->root_y_joint_->SetVelocity(0, 0);
      this->linear_joint_->SetVelocity(0, 0);
      // set position and velocity
      this->model_->SetWorldPose(base_pose);
      this->mass_link_->SetWorldPose(mass_pose);
      this->mass_link_->SetLinearVel(mass_velocity);

      return true;
    }

    bool SetStateCallback(hrpsys_gazebo_msgs::LIPSetStateRequest &req, hrpsys_gazebo_msgs::LIPSetStateResponse &res)
    {
      math::Pose base_pose(req.command.foot_position.x, req.command.foot_position.y, req.command.foot_position.z, 0, 0, 0);
      math::Pose mass_pose(req.command.mass_position.x, req.command.mass_position.y, req.command.mass_position.z, 0, 0, 0);
      math::Vector3 mass_velocity(req.command.mass_velocity.x, req.command.mass_velocity.y, req.command.mass_velocity.z);

      ROS_INFO_STREAM_THROTTLE(1.0, "[LIP command]  set state command" << std::endl
                               << "  foot_pos: " << base_pose.pos << std::endl
                               << "  mass_pos: " << mass_pose.pos << std::endl
                               << "  mass_vel: " << mass_velocity);

      // reset force and velocity
      this->linear_joint_->SetForce(0, 0);
      this->root_x_joint_->SetVelocity(0, 0);
      this->root_y_joint_->SetVelocity(0, 0);
      this->linear_joint_->SetVelocity(0, 0);
      // set position and velocity
      this->model_->SetWorldPose(base_pose);
      this->mass_link_->SetWorldPose(mass_pose);
      this->mass_link_->SetLinearVel(mass_velocity);

      return true;
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      addForce();
      publishState();
    }

    void addForce()
    {
      math::Pose base_pose = this->base_link_->GetWorldPose();;
      math::Pose mass_pose = this->mass_link_->GetWorldPose();;
      double mass_z = mass_pose.pos.z;
      double mass_length = (base_pose.pos - mass_pose.pos).GetLength();
      double mass = this->mass_link_->GetInertial()->GetMass();
      double gravity = 9.81;
      double force = mass * gravity * mass_length / mass_z;
      double force_limit = this->linear_joint_->GetEffortLimit(0);
      force = math::clamp(force, -force_limit, force_limit);

      ROS_INFO_STREAM_THROTTLE(1.0, "[LIP control]  " << "mg: " << mass * gravity << "  z: " << mass_z << "  l: " << mass_length << "  f: " << force);

      this->linear_joint_->SetForce(0, force);
    }

    void publishState()
    {
      hrpsys_gazebo_msgs::LIPState state;
      math::Pose base_pose = this->base_link_->GetWorldPose();;
      math::Pose mass_pose = this->mass_link_->GetWorldPose();;
      math::Vector3 mass_velocity = this->mass_link_->GetWorldLinearVel();

      std_msgs::Header tmp_header;
      tmp_header.stamp = ros::Time::now();
      state.header = tmp_header;

      state.foot_position.x = base_pose.pos.x;
      state.foot_position.y = base_pose.pos.y;
      state.foot_position.z = base_pose.pos.z;

      state.mass_position.x = mass_pose.pos.x;
      state.mass_position.y = mass_pose.pos.y;
      state.mass_position.z = mass_pose.pos.z;

      state.mass_velocity.x = mass_velocity.x;
      state.mass_velocity.y = mass_velocity.y;
      state.mass_velocity.z = mass_velocity.z;

      this->state_pub_queue_->push(state, this->state_pub_);
    }

  private:
    physics::ModelPtr model_;
    event::ConnectionPtr update_connection_;

    ros::NodeHandle* ros_node_;
    PubMultiQueue pmq_;

    boost::thread deferred_load_thread_;
    boost::thread service_callback_thread_;

    ros::Publisher state_pub_;
    PubQueue<hrpsys_gazebo_msgs::LIPState>::Ptr state_pub_queue_;

    ros::ServiceServer switch_foot_srv_;
    ros::ServiceServer set_state_srv_;

    physics::LinkPtr base_link_;
    physics::LinkPtr mass_link_;
    physics::JointPtr root_x_joint_;
    physics::JointPtr root_y_joint_;
    physics::JointPtr linear_joint_;

    ros::CallbackQueue service_queue_;
  };

  GZ_REGISTER_MODEL_PLUGIN(LIPPlugin)
}
