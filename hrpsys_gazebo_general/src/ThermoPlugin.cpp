#include <iostream>
#include <limits>
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

#include <std_msgs/Float32MultiArray.h>

#include "PubQueue.h"

namespace gazebo {
class ThermoPlugin: public ModelPlugin {

public:
  class MotorHeatParam
  {
  public:
    double temperature;
    double surface_temperature;
    double currentCoeffs; // electric_resistance / A_vs_Nm / A_vs_Nm 
    double R1; //thermal resistance between Core and Surface
    double R2; //thermal resistance between Surface ans Air
    double core_C;
    double surface_C;
  };
  
  // Initialize
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    // Store the pointer to the model
    this->model = _parent;
    this->world = this->model->GetWorld();

    this->robot_name = this->parsesdfParam(_sdf, "robotname", std::string("robot"));
    this->controller_name = this->parsesdfParam(_sdf, "controllername", std::string("thermo"));
    
    this->configuration_name = this->robot_name + "/" + this->controller_name + "_gazebo_configuration";

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()) {
      gzerr
	<< "A ROS node for Gazebo has not been initialized, unable to load plugin. "
	<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)";
      return;
    }

    // ros node
    this->rosNode = new ros::NodeHandle("");
    
    XmlRpc::XmlRpcValue param_val;
    this->rosNode->getParam(this->configuration_name, param_val);
    if (param_val.getType() ==  XmlRpc::XmlRpcValue::TypeStruct) {
      // joint name
      XmlRpc::XmlRpcValue joint_lst = param_val["joints"];
      if (joint_lst.getType() == XmlRpc::XmlRpcValue::TypeArray) {
	for(int s = 0; s < joint_lst.size(); s++) {
	  std::string n = joint_lst[s];
	  ROS_INFO("add joint: %s", n.c_str());
	  this->jointNames.push_back(n);
	}
      } else {
	ROS_WARN("Controlled Joints: no setting exists");
      }
    } else{
      ROS_WARN("param_val.getType() !=  XmlRpc::XmlRpcValue::TypeStruct");
    }
    
    // get pointers to joints from gazebo
    this->joints.resize(this->jointNames.size());
    ROS_INFO("joints size = %ld", this->joints.size());
    for (unsigned int i = 0; i < this->joints.size(); ++i) {
      this->joints[i] = this->model->GetJoint(this->jointNames[i]);
      if (!this->joints[i])  {
	ROS_ERROR("%s robot expected joint[%s] not present, plugin not loaded",
		  this->robot_name.c_str(), this->jointNames[i].c_str());
	return;
      }
    }

    {
      this->atomosphere_temperature = this->parsesdfParam(_sdf, "atomosphere_temperature", 25.0);
      std::string pname = this->configuration_name+"/atomosphere_temperature";
      if (this->rosNode->hasParam(pname)) {
	double ret;
	this->rosNode->getParam(pname, ret);
	ROS_INFO("atomosphere_temperature %lf", ret);
	this->atomosphere_temperature = ret;
      }
    }
    
    //motor heat params
    motorheatparams.resize(this->jointNames.size());
    for (unsigned int i = 0; i < this->joints.size(); ++i) {
      std::string joint_ns=this->configuration_name;
      joint_ns += ("/motor_heat_params/" + this->joints[i]->GetName() + "/");
      
      double currentCoeffs_val = 0.0;
      std::string currentCoeffs_str = joint_ns+"currentCoeffs";
      if (!this->rosNode->getParam(currentCoeffs_str, currentCoeffs_val)) {
	ROS_WARN("IOBPlugin: couldn't find a currentCoeffs param for %s", joint_ns.c_str());
      }
      this->motorheatparams[i].currentCoeffs  =  currentCoeffs_val;
      
      double R1_val = std::numeric_limits<double>::max();
      std::string R1_str = joint_ns+"R1";
      if (!this->rosNode->getParam(R1_str, R1_val)) {
	ROS_WARN("IOBPlugin: couldn't find a R1 param for %s", joint_ns.c_str());
      }
      this->motorheatparams[i].R1  =  R1_val;

      double R2_val = std::numeric_limits<double>::max();
      std::string R2_str = joint_ns+"R2";
      if (!this->rosNode->getParam(R2_str, R2_val)) {
	ROS_WARN("IOBPlugin: couldn't find a R2 param for %s", joint_ns.c_str());
      }
      this->motorheatparams[i].R2  =  R2_val;

      double core_C_val = std::numeric_limits<double>::max();
      std::string core_C_str = joint_ns+"core_C";
      if (!this->rosNode->getParam(core_C_str, core_C_val)) {
	ROS_WARN("IOBPlugin: couldn't find a core_C param for %s", joint_ns.c_str());
      }
      this->motorheatparams[i].core_C  =  core_C_val;

      double surface_C_val = std::numeric_limits<double>::max();
      std::string surface_C_str = joint_ns+"surface_C";
      if (!this->rosNode->getParam(surface_C_str, surface_C_val)) {
	ROS_WARN("IOBPlugin: couldn't find a surface_C param for %s", joint_ns.c_str());
      }
      this->motorheatparams[i].surface_C  =  surface_C_val;
      
      this->motorheatparams[i].temperature = atomosphere_temperature;
      this->motorheatparams[i].surface_temperature = atomosphere_temperature;
    }

    {
      this->thermal_publish_step = this->parsesdfParam(_sdf, "thermal_publish_step", 10);
      std::string pname = this->configuration_name+"/thermal_publish_step";
      if (this->rosNode->hasParam(pname)) {
	int ret;
	this->rosNode->getParam(pname, ret);
	ROS_INFO("thermal_publish_step %d", ret);
	this->thermal_publish_step = ret;
      }
      this->thermal_publish_cnt = this->thermal_publish_step ;
    }
    
    this->lastUpdateTime = this->world->GetSimTime() ;
		
    // ros callback queue for processing subscription
    this->deferredLoadThread = boost::thread(
					     boost::bind(&ThermoPlugin::DeferredLoad, this));
  }

  void DeferredLoad() {
    // publish multi queue
    this->pmq.startServiceThread();

    // ros topic publications
    this->pubCoilThermoQueue = this->pmq.addPub<std_msgs::Float32MultiArray>();
    this->pubCoilThermo = this->rosNode->advertise<std_msgs::Float32MultiArray>(
								      "/" + this->robot_name + "/" + this->controller_name + "/coil", 100, true);
    this->pubCaseThermoQueue = this->pmq.addPub<std_msgs::Float32MultiArray>();
    this->pubCaseThermo = this->rosNode->advertise<std_msgs::Float32MultiArray>(
								      "/" + this->robot_name + "/" + this->controller_name + "/case", 100, true);

    // Listen to the update event.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
								    boost::bind(&ThermoPlugin::OnUpdate, this, _1));

    ROS_INFO("ThermoPlugin was loaded !");
  }

  // Called by the world update start event
  void OnUpdate(const common::UpdateInfo & /*_info*/) {
    this->curTime = this->world->GetSimTime();
    for (int i=0;i<this->joints.size();i++){
      this->EstimateThermo(this->motorheatparams[i],this->joints[i]->GetForce(0));
    }
    this->lastUpdateTime = this->curTime ;
    if ( --this->thermal_publish_cnt > 0 ) return ;

    this->thermal_publish_cnt = this->thermal_publish_step;
    this->PublishThermo();
  }

  void EstimateThermo(MotorHeatParam &param,double tau){
    double Qin, Qmid, Qout;
    double dt = (this->curTime - this->lastUpdateTime).Double();
    Qin = param.currentCoeffs * std::pow(tau, 2);
    Qmid = (param.temperature - param.surface_temperature) / param.R1;
    Qout = (param.surface_temperature - this->atomosphere_temperature) / param.R2;
    param.temperature += (Qin - Qmid) / param.core_C * dt;
    param.surface_temperature += (Qmid - Qout) / param.surface_C * dt;
  }
  
  // Link has only 1 joint, and the joint has only 1 axis
  void PublishThermo() {
    std_msgs::Float32MultiArray the1, the2;

    the1.data.resize(this->joints.size());
    for(int i;i<this->joints.size();i++){
	the1.data[i] = this->motorheatparams[i].temperature;
    }
    this->pubCaseThermoQueue->push(the1, this->pubCoilThermo);

    the2.data.resize(this->joints.size());
    for(int i;i<this->joints.size();i++){
	the2.data[i] = this->motorheatparams[i].surface_temperature;
    }
    this->pubCaseThermoQueue->push(the2, this->pubCaseThermo);
  }

private:
  physics::ModelPtr model;
  physics::WorldPtr world;
  ros::NodeHandle* rosNode;
  std::string robot_name;
  std::vector<std::string> jointNames;
  physics::Joint_V joints;
  std::string controller_name;
  std::string configuration_name;
  std::vector<MotorHeatParam> motorheatparams;
  double atomosphere_temperature;
  int thermal_publish_step, thermal_publish_cnt;
  common::Time lastUpdateTime;
  common::Time curTime;
  event::ConnectionPtr updateConnection;
  
  PubMultiQueue pmq;
  boost::thread deferredLoadThread;

  ros::Publisher pubCoilThermo;
  PubQueue<std_msgs::Float32MultiArray>::Ptr pubCoilThermoQueue;
  ros::Publisher pubCaseThermo;
  PubQueue<std_msgs::Float32MultiArray>::Ptr pubCaseThermoQueue;

  template<typename T>
  T parsesdfParam(sdf::ElementPtr _sdf, std::string name, T defo){
    T ret ;
    if (_sdf->HasElement(name)) {
      ret = _sdf->Get<T>(name);
    } else {
      ret = defo;
    }
    std::cout << " [thermo plugin] " << name << " = " << ret << std::endl;
    return ret ;
  }
};

  GZ_REGISTER_MODEL_PLUGIN(ThermoPlugin)
}
