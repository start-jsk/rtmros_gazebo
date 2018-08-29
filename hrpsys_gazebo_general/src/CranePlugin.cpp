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
#include <std_msgs/Empty.h>


namespace gazebo
{
  class Crane : public ModelPlugin
  {

  public:
    // Initialize
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->world = this->model->GetWorld();

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
        gzerr << "[CranePlugin] Root link are not found. (link_name is "<< this->link_name << ")" << std::endl;
        return;
      }

      // find pull up height
      this->lift_height = 2.0;
      if (_sdf->HasElement("liftheight")) {
	this->lift_height = _sdf->Get<double>("liftheight");
      }

      //find lift velocity
      this->lift_velocity = 1.0;
      if (_sdf->HasElement("liftvelocity")) {
        this->lift_velocity = _sdf->Get<double>("liftvelocity");
      }
      
      //find lower velocity
      this->lower_velocity = 1.0;
      if (_sdf->HasElement("lowervelocity")) {
        this->lower_velocity = _sdf->Get<double>("lowervelocity");
      }

      //find lower height
      this->lower_height = 0.0;
      if (_sdf->HasElement("lowerheight")) {
        this->lower_height = _sdf->Get<double>("lowerheight");
      }

      //find pgain
      this->pgain = 10000;
      if (_sdf->HasElement("pgain")) {
        this->pgain = _sdf->Get<double>("pgain");
      }
      //find dgain
      this->dgain = 10;
      if (_sdf->HasElement("dgain")) {
        this->pgain = _sdf->Get<double>("dgain");
      }

      //find damp
      this->damp = 1;
      if (_sdf->HasElement("damp")) {
        this->damp = _sdf->Get<double>("damp");
      }
      
      // initialize
      {
	std_msgs::Empty::ConstPtr msg;
	LiftCommand(msg);
      }
      
      // Make sure the ROS node for Gazebo has already been initialized
      if (!ros::isInitialized()) {
        gzerr << "[CranePlugin] A ROS node for Gazebo has not been initialized, unable to load plugin. "
              << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)";
        return;
      }
      // ros node
      this->rosNode = new ros::NodeHandle("");
      // ros callback queue for processing subscription
      this->deferredLoadThread = boost::thread(boost::bind(&Crane::DeferredLoad, this));
    }

    void DeferredLoad() {
      // ros topic subscribtions
      ros::SubscribeOptions LiftCommandSo =
        ros::SubscribeOptions::create<std_msgs::Empty>("/" + this->obj_name + "/CranePlugin/LiftCommand", 100,
                                                    boost::bind(&Crane::LiftCommand, this, _1),
                                                    ros::VoidPtr(), &this->rosQueue);
      ros::SubscribeOptions LowerCommandSo =
        ros::SubscribeOptions::create<std_msgs::Empty>("/" + this->obj_name + "/CranePlugin/LowerCommand", 100,
                                              boost::bind(&Crane::LowerCommand, this, _1),
                                              ros::VoidPtr(), &this->rosQueue);
      // Enable TCP_NODELAY because TCP causes bursty communication with high jitter,
      LiftCommandSo.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true);
      this->subLiftCommand = this->rosNode->subscribe(LiftCommandSo);
      LowerCommandSo.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true);
      this->subLowerCommand = this->rosNode->subscribe(LowerCommandSo);

      // ros callback queue for processing subscription
      this->callbackQueeuThread = boost::thread(boost::bind(&Crane::RosQueueThread, this));

      // Listen to the update event.
      this->updateConnection = event::Events::ConnectWorldUpdateEnd(boost::bind(&Crane::OnUpdate, this));

      ROS_INFO("CranePlugin was loaded !");
    }

    void LiftCommand(const std_msgs::Empty::ConstPtr &_msg)
    {
      this->pull_up_flag = true;
      this->pull_down_flag = false;
      this->target_height = this->link->GetWorldPose().pos.z;
      this->lift_fin = false;
      // initialize update time
      this->lastUpdateTime = this->world->GetSimTime();
      this->count = 0;
      gzmsg << "[CranePlugin]subscribed LiftCommand." << std::endl;      
    }

    void LowerCommand(const std_msgs::Empty::ConstPtr &_msg)
    {
      if(this->pull_up_flag){
	this->pull_up_flag = false;
	this->pull_down_flag = true;
	// initialize update time
	this->lastUpdateTime = this->world->GetSimTime();
      }
      gzdbg << "[CranePlugin]subscribed LowerCommand." << std::endl;
    }

    // Called by the world update start event
    void OnUpdate()
    {
      if (this->pull_up_flag||this->pull_down_flag) {
	common::Time curTime = this->world->GetSimTime();
	if (this->pull_up_flag){
	  if(this->target_height<this->lift_height){
	    this->target_height+=lift_velocity * (curTime - this->lastUpdateTime).Double();
	  }else{
	    this->target_height=this->lift_height;
	    count++;
	    if(count%100==0){
	      math::Vector3 lin=this->model->GetWorldLinearVel();
	      math::Vector3 ang=this->model->GetWorldAngularVel();
	      lin *= 0.7;
	      ang *= 0.7;
	      this->model->SetWorldTwist(lin,ang);
	    }
	    // if(!this->lift_fin){
	    //   this->lift_fin=true;
	    //   math::Quaternion tmp=this->link->GetWorldPose().rot;
	    //   math::Vector3 vec=this->link->GetWorldPose().pos;
	    //   this->model->SetLinkWorldPose(math::Pose(vec,math::Quaternion(0.0,0.0,tmp.GetYaw())),this->link);
	    //   this->model->SetWorldTwist(math::Vector3(),math::Vector3());
	    // }
	  }
	}
	if (this->pull_down_flag){
	  if(this->target_height<this->lower_height){
	    this->pull_down_flag=false;
	    return;
	  }
	  this->target_height-=lower_velocity * (curTime - this->lastUpdateTime).Double();
	}
	double error  = this->target_height - this->link->GetWorldPose().pos.z;
	double derror = 0.0;
	if (!math::equal((curTime - this->lastUpdateTime).Double(),0.0)){
	  derror=(error-pre_error)/(curTime - this->lastUpdateTime).Double();
	}
	double F_z = pgain * error + dgain * derror;
	if (F_z>0.0){
	  math::Vector3 lin=this->model->GetWorldLinearVel();
	  math::Vector3 ang=this->model->GetWorldAngularVel();
	  lin *= -this->damp;
	  ang *= -this->damp;
	  lin.z=F_z; 
	  this->link->AddForce(lin);
	  this->link->AddTorque(ang);
	}
	this->lastUpdateTime = curTime;
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
    physics::WorldPtr world;
    physics::ModelPtr model;
    std::string obj_name;
    std::string link_name;
    double lift_height;
    double lower_height;
    double target_height;
    double lift_velocity;
    double lower_velocity;
    double pgain;
    double dgain;
    double pre_error;
    bool lift_fin;
    physics::LinkPtr link;
    event::ConnectionPtr updateConnection;
    bool pull_up_flag;
    bool pull_down_flag;
    bool damp;
    common::Time lastUpdateTime;
    int count;

    ros::NodeHandle* rosNode;
    ros::CallbackQueue rosQueue;
    ros::Subscriber subLiftCommand;
    ros::Subscriber subLowerCommand;
    boost::thread callbackQueeuThread;
    boost::thread deferredLoadThread;
  };

  GZ_REGISTER_MODEL_PLUGIN(Crane)
}
