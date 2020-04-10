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

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/String.h>

#include "PubQueue.h"

#if GAZEBO_MAJOR_VERSION >= 7
#else
namespace ignition::math
{
  using equal = gazebo::math::equal;
  using clamp = gazebo::math::clamp;
  using Pose3d = gazebo::math::Pose;
  using Vector3d = gazebo::math::Vector3;
  using Quaterniond = gazebo::math::Quaterion;
}
#endif


namespace gazebo
{
  class GetVel : public ModelPlugin
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
      this->deferredLoadThread = boost::thread(boost::bind(&GetVel::DeferredLoad, this));
    }

    void DeferredLoad() {
      // publish multi queue
      this->pmq.startServiceThread();

      // ros topic publications
      this->pubRelVelQueue = this->pmq.addPub<geometry_msgs::TwistStamped>();
      this->pubRelVel = this->rosNode->advertise<geometry_msgs::TwistStamped>("/" + this->obj_name + "/GetVelPlugin/RelVel", 100, true);
      this->pubAbsVelQueue = this->pmq.addPub<geometry_msgs::TwistStamped>();
      this->pubAbsVel = this->rosNode->advertise<geometry_msgs::TwistStamped>("/" + this->obj_name + "/GetVelPlugin/AbsVel", 100, true);
      this->pubRelAccelQueue = this->pmq.addPub<geometry_msgs::TwistStamped>();
      this->pubRelAccel = this->rosNode->advertise<geometry_msgs::TwistStamped>("/" + this->obj_name + "/GetVelPlugin/RelAccel", 100, true);
      this->pubAbsAccelQueue = this->pmq.addPub<geometry_msgs::TwistStamped>();
      this->pubAbsAccel = this->rosNode->advertise<geometry_msgs::TwistStamped>("/" + this->obj_name + "/GetVelPlugin/AbsAccel", 100, true);
      this->pubPoseQueue = this->pmq.addPub<geometry_msgs::PoseStamped>();
      this->pubPose = this->rosNode->advertise<geometry_msgs::PoseStamped>("/" + this->obj_name + "/GetVelPlugin/Pose", 100, true);

      // Listen to the update event.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GetVel::OnUpdate, this, _1));

      gzmsg << "GetVelPlugin was loaded !" << std::endl;
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
#if GAZEBO_MAJOR_VERSION >= 7
      common::Time curTime = this->world->SimTime();
#else
      common::Time curTime = this->world->GetSimTime();
#endif

      // publish topics
      this->PublishVel(curTime);
    }

    // Publish function
    void PublishVel(const common::Time &_curTime)
    {
      geometry_msgs::TwistStamped _twist;
      geometry_msgs::PoseStamped _pose;
      ignition::math::Vector3d linear_vel;
      ignition::math::Vector3d angular_vel;
      ignition::math::Vector3d linear_accel;
      ignition::math::Vector3d angular_accel;
      ignition::math::Pose3d pose;
      _twist.header.stamp = ros::Time(_curTime.sec, _curTime.nsec);
      _pose.header.stamp = ros::Time(_curTime.sec, _curTime.nsec);

      // set rel vel
#if GAZEBO_MAJOR_VERSION >= 9
      linear_vel = this->model->RelativeLinearVel();
#else
      linear_vel = this->model->GetRelativeLinearVel();
#endif
#if GAZEBO_MAJOR_VERSION >= 7
      _twist.twist.linear.x = linear_vel.X();
      _twist.twist.linear.y = linear_vel.Y();
      _twist.twist.linear.z = linear_vel.Z();
#else
      _twist.twist.linear.x = linear_vel.x;
      _twist.twist.linear.y = linear_vel.y;
      _twist.twist.linear.z = linear_vel.z;
#endif
#if GAZEBO_MAJOR_VERSION >= 9
      angular_vel = this->model->RelativeAngularVel();
#else
      angular_vel = this->model->GetRelativeAngularVel();
#endif
#if GAZEBO_MAJOR_VERSION >= 7
      _twist.twist.angular.x = angular_vel.X();
      _twist.twist.angular.y = angular_vel.Y();
      _twist.twist.angular.z = angular_vel.Z();
#else
      _twist.twist.angular.x = angular_vel.x;
      _twist.twist.angular.y = angular_vel.y;
      _twist.twist.angular.z = angular_vel.z;
#endif
      // publish rel vel
      this->pubRelVelQueue->push(_twist, this->pubRelVel);

      // set abs vel
#if GAZEBO_MAJOR_VERSION >= 9
      linear_vel = this->model->WorldLinearVel();
#else
      linear_vel = this->model->GetWorldLinearVel();
#endif
#if GAZEBO_MAJOR_VERSION >= 7
      _twist.twist.linear.x = linear_vel.X();
      _twist.twist.linear.y = linear_vel.Y();
      _twist.twist.linear.z = linear_vel.Z();
#else
      _twist.twist.linear.x = linear_vel.x;
      _twist.twist.linear.y = linear_vel.y;
      _twist.twist.linear.z = linear_vel.z;
#endif
#if GAZEBO_MAJOR_VERSION >= 9
      angular_vel = this->model->WorldAngularVel();
#else
      angular_vel = this->model->GetWorldAngularVel();
#endif
#if GAZEBO_MAJOR_VERSION >= 7
      _twist.twist.angular.x = angular_vel.X();
      _twist.twist.angular.y = angular_vel.Y();
      _twist.twist.angular.z = angular_vel.Z();
#else
      _twist.twist.angular.x = angular_vel.x;
      _twist.twist.angular.y = angular_vel.y;
      _twist.twist.angular.z = angular_vel.z;
#endif
      // publish abs vel
      this->pubAbsVelQueue->push(_twist, this->pubAbsVel);

      // set rel accel
#if GAZEBO_MAJOR_VERSION >= 9
      linear_accel = this->model->RelativeLinearAccel();
#else
      linear_accel = this->model->GetRelativeLinearAccel();
#endif
#if GAZEBO_MAJOR_VERSION >= 7
      _twist.twist.linear.x = linear_accel.X();
      _twist.twist.linear.y = linear_accel.Y();
      _twist.twist.linear.z = linear_accel.Z();
#else
      _twist.twist.linear.x = linear_accel.x;
      _twist.twist.linear.y = linear_accel.y;
      _twist.twist.linear.z = linear_accel.z;
#endif
#if GAZEBO_MAJOR_VERSION >= 9
      angular_accel = this->model->RelativeAngularAccel();
#else
      angular_accel = this->model->GetRelativeAngularAccel();
#endif
#if GAZEBO_MAJOR_VERSION >= 7
      _twist.twist.angular.x = angular_accel.X();
      _twist.twist.angular.y = angular_accel.Y();
      _twist.twist.angular.z = angular_accel.Z();
#else
      _twist.twist.angular.x = angular_accel.x;
      _twist.twist.angular.y = angular_accel.y;
      _twist.twist.angular.z = angular_accel.z;
#endif
      // publish rel accel
      this->pubRelAccelQueue->push(_twist, this->pubRelAccel);

      // set abs accel
#if GAZEBO_MAJOR_VERSION >= 9
      linear_accel = this->model->WorldLinearAccel();
#else
      linear_accel = this->model->GetWorldLinearAccel();
#endif
#if GAZEBO_MAJOR_VERSION >= 7
      _twist.twist.linear.x = linear_accel.X();
      _twist.twist.linear.y = linear_accel.Y();
      _twist.twist.linear.z = linear_accel.Z();
#else
      _twist.twist.linear.x = linear_accel.x;
      _twist.twist.linear.y = linear_accel.y;
      _twist.twist.linear.z = linear_accel.z;
#endif
#if GAZEBO_MAJOR_VERSION >= 9
      angular_accel = this->model->WorldAngularAccel();
#else
      angular_accel = this->model->GetWorldAngularAccel();
#endif
#if GAZEBO_MAJOR_VERSION >= 7
      _twist.twist.angular.x = angular_accel.X();
      _twist.twist.angular.y = angular_accel.Y();
      _twist.twist.angular.z = angular_accel.Z();
#else
      _twist.twist.angular.x = angular_accel.x;
      _twist.twist.angular.y = angular_accel.y;
      _twist.twist.angular.z = angular_accel.z;
#endif
      // publish abs accel
      this->pubAbsAccelQueue->push(_twist, this->pubAbsAccel);

      // set pose
#if GAZEBO_MAJOR_VERSION >= 9
      pose = this->link->WorldCoGPose();
#else
      pose = this->link->GetWorldCoGPose();
#endif
      // pose = this->link->GetWorldPose();
#if GAZEBO_MAJOR_VERSION >= 7
      _pose.pose.position.x = pose.Pos().X();
      _pose.pose.position.y = pose.Pos().Y();
      _pose.pose.position.z = pose.Pos().Z();
      _pose.pose.orientation.x = pose.Rot().X();
      _pose.pose.orientation.y = pose.Rot().Y();
      _pose.pose.orientation.z = pose.Rot().Z();
      _pose.pose.orientation.w = pose.Rot().W();
#else
      _pose.pose.position.x = pose.pos.x;
      _pose.pose.position.y = pose.pos.y;
      _pose.pose.position.z = pose.pos.z;
      _pose.pose.orientation.x = pose.rot.x;
      _pose.pose.orientation.y = pose.rot.y;
      _pose.pose.orientation.z = pose.rot.z;
      _pose.pose.orientation.w = pose.rot.w;
#endif
      // publish pose
      this->pubPoseQueue->push(_pose, this->pubPose);
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
    boost::mutex mutex;
    boost::thread deferredLoadThread;

    ros::Publisher pubRelVel;
    PubQueue<geometry_msgs::TwistStamped>::Ptr pubRelVelQueue;
    ros::Publisher pubAbsVel;
    PubQueue<geometry_msgs::TwistStamped>::Ptr pubAbsVelQueue;
    ros::Publisher pubRelAccel;
    PubQueue<geometry_msgs::TwistStamped>::Ptr pubRelAccelQueue;
    ros::Publisher pubAbsAccel;
    PubQueue<geometry_msgs::TwistStamped>::Ptr pubAbsAccelQueue;
    ros::Publisher pubPose;
    PubQueue<geometry_msgs::PoseStamped>::Ptr pubPoseQueue;
  };

  GZ_REGISTER_MODEL_PLUGIN(GetVel)
}
