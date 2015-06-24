#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Time.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

#include "PubQueue.h"


namespace gazebo
{
  class PubTf : public ModelPlugin
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

      // find root link
      this->link = this->model->GetLink(this->link_name);
      if(!this->link) {
        gzerr << "Root link are not found. (link_name is "<< this->link_name << ")" << std::endl;
        return;
      }
      world = this->model->GetWorld();

      // Make sure the ROS node for Gazebo has already been initialized
      if (!ros::isInitialized()) {
        gzerr << "A ROS node for Gazebo has not been initialized, unable to load plugin. "
              << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)";
        return;
      }
      // ros node
      this->rosNode = new ros::NodeHandle("");
      // ros callback queue for processing subscription
      this->deferredLoadThread = boost::thread(boost::bind(&PubTf::DeferredLoad, this));
    }

    void DeferredLoad() {
      // publish multi queue
      this->pmq.startServiceThread();

      // Listen to the update event.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&PubTf::OnUpdate, this, _1));

      gzmsg << "PubTfPlugin was loaded !" << std::endl;
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      common::Time curTime = this->world->GetSimTime();

      // publish topics
      this->PublishTf(curTime);
    }

    // Publish function
    void PublishTf(const common::Time &_curTime)
    {
      math::Pose pose;
      tf::Transform transform;

      // set pose
      pose = this->link->GetWorldPose();
      transform.setOrigin(tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z));
      tf::Quaternion q(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
      transform.setRotation(q);
      // publish pose
      this->br.sendTransform(tf::StampedTransform(transform, ros::Time(_curTime.sec, _curTime.nsec), "gazebo_world", this->link_name));
    }

  private:
    physics::ModelPtr model;
    physics::WorldPtr world;
    std::string obj_name;
    std::string link_name;
    physics::LinkPtr link;
    event::ConnectionPtr updateConnection;

    ros::NodeHandle* rosNode;
    PubMultiQueue pmq;
    boost::thread deferredLoadThread;

    tf::TransformBroadcaster br;
  };

  GZ_REGISTER_MODEL_PLUGIN(PubTf)
}
