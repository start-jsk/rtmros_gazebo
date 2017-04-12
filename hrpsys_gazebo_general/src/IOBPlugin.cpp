#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <memory>

#include <gazebo/transport/Node.hh>
#include <gazebo/common/Assert.hh>

#include "IOBPlugin.h"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(IOBPlugin);

IOBPlugin::IOBPlugin() : publish_joint_state(false),
                         publish_joint_state_step(0),
                         publish_joint_state_counter(0),
                         use_synchronized_command(false),
                         use_velocity_feedback(false),
                         use_joint_effort(false),
                         use_loose_synchronized(true),
                         iob_period(0.005),
                         force_sensor_average_window_size(6),
                         force_sensor_average_cnt(0),
                         effort_average_window_size(6),
                         effort_average_cnt(0),
                         publish_step(2),
                         publish_count(0)
{
}

IOBPlugin::~IOBPlugin() {
}

void IOBPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

  this->controller_name = "hrpsys_gazebo_configuration";
  if (_sdf->HasElement("controller")) {
    this->controller_name = _sdf->Get<std::string>("controller");
  }

  this->robot_name = _parent->GetScopedName();
  if (_sdf->HasElement("robotname")) {
    this->robot_name = _sdf->Get<std::string>("robotname");
    ROS_WARN("USE ROBOT NAME from URDF: %s, scoped name(%s)",
             this->robot_name.c_str(),
             _parent->GetScopedName().c_str());
  }
  this->controller_name = this->robot_name + "/" + this->controller_name;

  if (_sdf->HasElement("force_sensor_average_window_size")) {
    this->force_sensor_average_window_size = _sdf->Get<float>("force_sensor_average_window_size");
  }

  this->use_synchronized_command = false;
  if (_sdf->HasElement("synchronized_command")) {
    this->use_synchronized_command = _sdf->Get<bool>("synchronized_command");
    std::cerr << ";; use synchronized command" << std::endl;
  }

  this->use_velocity_feedback = false;
  if (_sdf->HasElement("velocity_feedback")) {
    this->use_velocity_feedback = _sdf->Get<bool>("velocity_feedback");
    std::cerr << ";; use velocity feedback" << std::endl;
  }

  // initialize ros
  if (!ros::isInitialized()) {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  // ros node
  this->rosNode = new ros::NodeHandle("");

  this->model = _parent;
  this->world = this->model->GetWorld();

  // save sdf
  this->sdf = _sdf;

  // initialize update time
  this->lastControllerUpdateTime = this->world->GetSimTime();

  // creating joints from ros param
  if (this->rosNode->hasParam(this->controller_name)) {
    { // read synchronized_command from rosparam
      std::string pname = this->controller_name + "/use_synchronized_command";
      if (this->rosNode->hasParam(pname)) {
        bool ret;
        this->rosNode->getParam(pname, ret);
        if (!this->use_synchronized_command) {
          this->use_synchronized_command = ret;
        } else {
          ROS_WARN("override use_synchronized_command at %d by %d",
                   this->use_synchronized_command, ret);
        }
      }
    }
    { // read synchronized_command from rosparam
      std::string pname = this->controller_name + "/use_loose_synchronized";
      if (this->rosNode->hasParam(pname)) {
        bool ret = false;
        this->rosNode->getParam(pname, ret);
        this->use_loose_synchronized = ret;
      }
      ROS_INFO("loose synchronized %d", this->use_loose_synchronized);
    }
    { // read velocity_feedback from rosparam
      std::string pname = this->controller_name + "/use_velocity_feedback";
      if (this->rosNode->hasParam(pname)) {
        bool ret;
        this->rosNode->getParam(pname, ret);
        ROS_WARN("override use_veolcity_feedback at %d by %d",
                 this->use_velocity_feedback, ret);
        this->use_velocity_feedback = ret;
      }
    }
    { // read use_joint_effort from rosparam
      std::string pname = this->controller_name + "/use_joint_effort";
      if (this->rosNode->hasParam(pname)) {
        bool ret;
        this->rosNode->getParam(pname, ret);
        ROS_INFO("use_joint_effort %d", ret);
        this->use_joint_effort = ret;
      }
    }
    {
      std::string pname = this->controller_name + "/iob_rate";
      if (this->rosNode->hasParam(pname)) {
        double rate;
        this->rosNode->getParam(pname, rate);
        ROS_INFO("iob rate %f", rate);
        this->iob_period = 1.0 / rate;
      }
    }
    {
      std::string pname = this->controller_name + "/force_sensor_average_window_size";
      if (this->rosNode->hasParam(pname)) {
        int asize;
        this->rosNode->getParam(pname, asize);
        force_sensor_average_window_size = asize;
        ROS_INFO("force_sensor_average_window_size %d", asize);
      }
    }
    { // read publish_joint_state from rosparam
      std::string pname = this->controller_name + "/publish_joint_state";
      if (this->rosNode->hasParam(pname)) {
        std::string topic;
        if (this->rosNode->hasParam(pname + "/topic")) {
          this->rosNode->getParam(pname + "/topic", topic);
        } else {
          topic = this->robot_name + "/joint_state";
        }
        //
        this->publish_joint_state_counter = 0;
        this->publish_joint_state_step = 1;
        if (this->rosNode->hasParam(pname + "/step")) {
          int stp;
          this->rosNode->getParam(pname + "/step", stp);
          this->publish_joint_state_step = stp;
        }
        //
        this->pubJointStateQueue = this->pmq.addPub<sensor_msgs::JointState>();
        this->pubJointState
          = this->rosNode->advertise<sensor_msgs::JointState>(topic, 100, true);
        ROS_INFO("publish joint state");
        this->publish_joint_state = true;
      }
    }
    XmlRpc::XmlRpcValue param_val;
    this->rosNode->getParam(this->controller_name, param_val);
    if (param_val.getType() ==  XmlRpc::XmlRpcValue::TypeStruct) {
      std::string rname = param_val["robotname"];
      XmlRpc::XmlRpcValue joint_lst = param_val["joints"];
      XmlRpc::XmlRpcValue fsensors = param_val["force_torque_sensors"];
      XmlRpc::XmlRpcValue fsensors_config = param_val["force_torque_sensors_config"];
      XmlRpc::XmlRpcValue imusensors = param_val["imu_sensors"];
      XmlRpc::XmlRpcValue imusensors_config = param_val["imu_sensors_config"];

      if (rname != this->robot_name) {
        ROS_WARN("mismatch robotnames: %s (ros parameter) != %s (gazebo element)",
                 rname.c_str(), this->robot_name.c_str());
      } else {
        ROS_INFO("robotname: %s", rname.c_str());
      }
      // joint name
      if (joint_lst.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for(int s = 0; s < joint_lst.size(); s++) {
          std::string n = joint_lst[s];
          ROS_INFO("add joint: %s", n.c_str());
          this->jointNames.push_back(n);
        }
      } else {
        ROS_WARN("Controlled Joints: no setting exists");
      }
      // Force sensor setting
      if (fsensors.getType() == XmlRpc::XmlRpcValue::TypeArray &&
          fsensors_config.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        for(int s = 0; s < fsensors.size(); s++) {
          this->forceSensorNames.push_back(fsensors[s]);
        }
        for(XmlRpc::XmlRpcValue::iterator f = fsensors_config.begin(); f != fsensors_config.end(); f++) {
          std::string sensor_name = f->first;
          if (f->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            std::string jn = f->second["joint_name"];
            std::string fi = f->second["frame_id"];
            ROS_INFO("force: %s, %s %s", sensor_name.c_str(), jn.c_str(), fi.c_str());

            struct force_sensor_info fsi;
            fsi.joint = this->model->GetJoint(jn);
            if(!fsi.joint) {
              gzerr << "force torque joint (" << jn << ") not found\n";
            } else {
              fsi.frame_id = fi;
              XmlRpc::XmlRpcValue trs = f->second["translation"];
              XmlRpc::XmlRpcValue rot = f->second["rotation"];
              fsi.pose.reset();
              if ((trs.getType() == XmlRpc::XmlRpcValue::TypeArray) ||
                  (rot.getType() == XmlRpc::XmlRpcValue::TypeArray)) {
                math::Vector3 vtr;
                math::Quaternion qt;
                if (trs.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                  vtr.x = xmlrpc_value_as_double(trs[0]);
                  vtr.y = xmlrpc_value_as_double(trs[1]);
                  vtr.z = xmlrpc_value_as_double(trs[2]);
                }
                if (rot.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                  qt.w = xmlrpc_value_as_double(rot[0]);
                  qt.x = xmlrpc_value_as_double(rot[1]);
                  qt.y = xmlrpc_value_as_double(rot[2]);
                  qt.z = xmlrpc_value_as_double(rot[3]);
                }
                fsi.pose = PosePtr(new math::Pose (vtr, qt));
                this->forceSensors[sensor_name] = fsi;
              }
            }
          } else {
            ROS_ERROR("Force-Torque sensor: %s has invalid configuration", sensor_name.c_str());
          }
          // setup force sensor publishers
          std::shared_ptr<std::vector<std::shared_ptr<geometry_msgs::WrenchStamped> > > forceValQueue(new std::vector<std::shared_ptr<geometry_msgs::WrenchStamped> >);
          // forceValQueue->resize(this->force_sensor_average_window_size);
          for ( int i=0; i<this->force_sensor_average_window_size; i++ ){
            std::shared_ptr<geometry_msgs::WrenchStamped> fbuf(new geometry_msgs::WrenchStamped);
            forceValQueue->push_back(fbuf);
          }
          this->forceValQueueMap[sensor_name] = forceValQueue;
        }
      } else {
        ROS_WARN("Force-Torque sensor: no setting exists");
      }
      // IMU sensor setting
      if (imusensors.getType() == XmlRpc::XmlRpcValue::TypeArray &&
          imusensors_config.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        for(int s = 0; s < imusensors.size(); s++) {
          this->imuSensorNames.push_back(imusensors[s]);
        }
        for(XmlRpc::XmlRpcValue::iterator im = imusensors_config.begin(); im != imusensors_config.end(); im++) {
          std::string sensor_name = im->first;
          if (im->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            std::string sn = im->second["ros_name"];
            std::string ln = im->second["link_name"];
            std::string fi = im->second["frame_id"];
            ROS_INFO("imu: %s, %s, %s, %s", sensor_name.c_str(), sn.c_str(),
                     ln.c_str(), fi.c_str());

            struct imu_sensor_info msi;
            msi.sensor_name = sn;
            msi.frame_id = fi;
            msi.link = this->model->GetLink(ln);

            if (!msi.link)  {
              gzerr << ln << " not found\n";
            } else {
              // Get imu sensors
              msi.sensor = std::dynamic_pointer_cast<sensors::ImuSensor>
                (sensors::SensorManager::Instance()->GetSensor
                 (this->world->GetName() + "::" + msi.link->GetScopedName() + "::" + msi.sensor_name));

              if (!msi.sensor)  {
                gzerr << sensor_name << "("                             \
                      << (this->world->GetName() + "::" + msi.link->GetScopedName() + "::" + msi.sensor_name) \
                      <<" not found\n" << "\n";
              }
              imuSensors[sensor_name] = msi;
            }
          } else {
            ROS_ERROR("IMU sensor: %s has invalid configuration", sensor_name.c_str());
          }
        }
      } else {
        ROS_WARN("IMU sensor: no setting exists");
      }
    } else {
      ROS_WARN_STREAM("param: " << this->controller_name << ", configuration is not an array.");
    }
  } else {
    ROS_ERROR_STREAM("controller: " << this->controller_name << " has no parameter.");
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
    if (this->use_joint_effort) {
      this->joints[i]->SetProvideFeedback(true);
    }
  }

  // get effort limits from gazebo
  this->effortLimit.resize(this->jointNames.size());
  for (unsigned i = 0; i < this->effortLimit.size(); ++i) {
    this->effortLimit[i] = this->joints[i]->GetEffortLimit(0);
    ROS_DEBUG("effort_limit: %s %f", this->joints[i]->GetName().c_str(), this->joints[i]->GetEffortLimit(0));
  }

  {
    // initialize PID states: error terms
    this->errorTerms.resize(this->joints.size());
    for (unsigned i = 0; i < this->errorTerms.size(); ++i) {
      this->errorTerms[i].q_p = 0;
      this->errorTerms[i].d_q_p_dt = 0;
      this->errorTerms[i].k_i_q_i = 0;
      this->errorTerms[i].qd_p = 0;
    }
  }

  {
    // TODO: hardcoded for now
#define JOINT_DAMPING_LOWER_BOUND 0.1
    for (unsigned int i = 0; i < this->jointNames.size(); ++i)
    {
      // set max allowable damping coefficient to
      //  max effort allowed / max velocity allowed
      double maxEffort = this->joints[i]->GetEffortLimit(0);
      double maxVelocity = this->joints[i]->GetVelocityLimit(0);
      if (math::equal(maxVelocity, 0.0))
      {
        ROS_ERROR("Set Joint Damping Upper Limit: Joint[%s] "
                  "effort limit [%f] velocity limit[%f]: "
                  "velocity limit is unbounded, artificially setting "
                  "damping coefficient max limit to 1.0.  This should not"
                  "have happened for Atlas robot, please check your model.",
                 this->jointNames[i].c_str(), maxEffort, maxVelocity);
        maxVelocity = maxEffort;
      }

      this->jointDampingMax.push_back(maxEffort / maxVelocity * 50);

      this->jointDampingMin.push_back(JOINT_DAMPING_LOWER_BOUND);

      this->jointDampingModel.push_back(this->joints[i]->GetDamping(0));

      this->lastJointCFMDamping.push_back(this->joints[i]->GetDamping(0));

      ROS_INFO("Bounds for joint[%s] is [%f, %f], model default is [%f]",
               this->jointNames[i].c_str(),
               this->jointDampingMin[i], this->jointDampingMax[i],
               this->jointDampingModel[i]);
    }
  }

  {
    // We are not sending names due to the fact that there is an enum
    // joint indices in ...
    this->robotState.position.resize(this->joints.size());
    this->robotState.velocity.resize(this->joints.size());
    this->robotState.effort.resize(this->joints.size());
    // effort average
    effortValQueue.resize(0);
    for(int i = 0; i < this->effort_average_window_size; i++) {
      std::shared_ptr<std::vector<double> > vbuf(new std::vector<double> (this->joints.size()));
      effortValQueue.push_back(vbuf);
    }
    // for reference
    this->robotState.ref_position.resize(this->joints.size());
    this->robotState.ref_velocity.resize(this->joints.size());
    if (!this->use_velocity_feedback) {
      // for effort feedback
      this->robotState.kp_position.resize(this->joints.size());
      this->robotState.ki_position.resize(this->joints.size());
      this->robotState.kd_position.resize(this->joints.size());
      this->robotState.kp_velocity.resize(this->joints.size());
      this->robotState.i_effort_min.resize(this->joints.size());
      this->robotState.i_effort_max.resize(this->joints.size());
    } else {
      // for velocity feedback
      this->robotState.kpv_position.resize(this->joints.size());
      this->robotState.kpv_velocity.resize(this->joints.size());
    }
  }

  {
    this->jointCommand.position.resize(this->joints.size());
    this->jointCommand.velocity.resize(this->joints.size());
    this->jointCommand.effort.resize(this->joints.size());
    if (!this->use_velocity_feedback) {
      // for effort feedback
      this->jointCommand.kp_position.resize(this->joints.size());
      this->jointCommand.ki_position.resize(this->joints.size());
      this->jointCommand.kd_position.resize(this->joints.size());
      this->jointCommand.kp_velocity.resize(this->joints.size());
      this->jointCommand.i_effort_min.resize(this->joints.size());
      this->jointCommand.i_effort_max.resize(this->joints.size());
    } else {
      // for velocity feedback
      this->jointCommand.kpv_position.resize(this->joints.size());
      this->jointCommand.kpv_velocity.resize(this->joints.size());
    }

    this->ZeroJointCommand();
  }

  // ros callback queue for processing subscription
  this->deferredLoadThread = boost::thread(boost::bind(&IOBPlugin::DeferredLoad, this));
}

void IOBPlugin::ZeroJointCommand() {
  for (unsigned i = 0; i < this->jointNames.size(); ++i) {
    this->jointCommand.position[i] = 0;
    this->jointCommand.velocity[i] = 0;
    this->jointCommand.effort[i] = 0;
    if (!this->use_velocity_feedback) {
      // store these directly on altasState, more efficient for pub later
      this->robotState.kp_position[i] = 0;
      this->robotState.ki_position[i] = 0;
      this->robotState.kd_position[i] = 0;
      this->robotState.kp_velocity[i] = 0;
      this->robotState.i_effort_min[i] = 0;
      this->robotState.i_effort_max[i] = 0;
    } else {
      this->robotState.kpv_position[i] = 0;
      this->robotState.kpv_velocity[i] = 0;
    }
  }
  this->jointCommand.desired_controller_period_ms = 0;
}

void IOBPlugin::LoadPIDGainsFromParameter() {
  // pull down controller parameters
  std::string namestr(this->controller_name);

  for (unsigned int i = 0; i < this->joints.size(); ++i) {
    std::string joint_ns(namestr);
    joint_ns += ("/gains/" + this->joints[i]->GetName() + "/");

    if (this->use_velocity_feedback) {
      double p_v_val, vp_v_val;
      std::string p_v_str = std::string(joint_ns)+"p_v";
      std::string vp_v_str = std::string(joint_ns)+"vp_v";
      if (!this->rosNode->getParam(p_v_str, p_v_val)) {
        ROS_WARN("IOBPlugin: couldn't find a P_V param for %s", joint_ns.c_str());
      }
      if (!this->rosNode->getParam(vp_v_str, vp_v_val)) {
        ROS_WARN("IOBPlugin: couldn't find a VP_V param for %s", joint_ns.c_str());
      }

      this->robotState.kpv_position[i] = p_v_val;
      this->robotState.kpv_velocity[i] = vp_v_val;
    } else {
      // this is so ugly
      double p_val = 0, i_val = 0, d_val = 0, i_clamp_val = 0, vp_val = 0;
      std::string p_str = std::string(joint_ns)+"p";
      std::string i_str = std::string(joint_ns)+"i";
      std::string d_str = std::string(joint_ns)+"d";
      std::string i_clamp_str = std::string(joint_ns)+"i_clamp";
      std::string vp_str = std::string(joint_ns)+"vp";

      if (!this->rosNode->getParam(p_str, p_val)) {
        ROS_WARN("IOBPlugin: couldn't find a P param for %s", joint_ns.c_str());
      }
      if (!this->rosNode->getParam(i_str, i_val)) {
        ROS_WARN("IOBPlugin: couldn't find a I param for %s", joint_ns.c_str());
      }
      if (!this->rosNode->getParam(d_str, d_val)) {
        ROS_WARN("IOBPlugin: couldn't find a D param for %s", joint_ns.c_str());
      }
      if (!this->rosNode->getParam(i_clamp_str, i_clamp_val)) {
        ROS_WARN("IOBPlugin: couldn't find a I_CLAMP param for %s", joint_ns.c_str());
      }
      if (!this->rosNode->getParam(vp_str, vp_val)) {
        ROS_WARN("IOBPlugin: couldn't find a VP param for %s", joint_ns.c_str());
      }

      // store these directly on altasState, more efficient for pub later
      this->robotState.kp_position[i]  =  p_val;
      this->robotState.ki_position[i]  =  i_val;
      this->robotState.kd_position[i]  =  d_val;
      this->robotState.i_effort_min[i] = -i_clamp_val;
      this->robotState.i_effort_max[i] =  i_clamp_val;
      this->robotState.kp_velocity[i]  =  vp_val;
    }
  }
}

void IOBPlugin::DeferredLoad() {
  // publish multi queue
  this->pmq.startServiceThread();

  // pull down controller parameters
  this->LoadPIDGainsFromParameter();

  // ROS Controller API
  this->pubRobotStateQueue = this->pmq.addPub<RobotState>();
  this->pubRobotState = this->rosNode->advertise<RobotState>(this->robot_name + "/robot_state", 100, true);

  // ros topic subscribtions
  ros::SubscribeOptions IOBCommandSo =
    ros::SubscribeOptions::create<JointCommand>(this->robot_name + "/joint_command", 100,
                                                boost::bind(&IOBPlugin::SetJointCommand, this, _1),
                                                ros::VoidPtr(), &this->rosQueue);
  // Enable TCP_NODELAY because TCP causes bursty communication with high jitter,
  IOBCommandSo.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true);
  this->subIOBCommand = this->rosNode->subscribe(IOBCommandSo);

  { // reset_joint_reference service
    ros::AdvertiseServiceOptions IOBRefServO =
      ros::AdvertiseServiceOptions::create<std_srvs::Empty>
      (this->robot_name + "/reset_joint_reference", boost::bind(&IOBPlugin::serviceRefCallback, this, _1, _2),
       ros::VoidPtr(), &this->rosQueue);
    this->jointrefServ = this->rosNode->advertiseService(IOBRefServO);
  }

  if (this->use_synchronized_command) {
    // ros service
    ROS_INFO("use synchronized command");

    ros::AdvertiseServiceOptions IOBServO =
      ros::AdvertiseServiceOptions::create<hrpsys_gazebo_msgs::SyncCommand>
      (this->robot_name + "/iob_command", boost::bind(&IOBPlugin::serviceCallback, this, _1, _2),
       ros::VoidPtr(), &this->srvQueue);
    this->controlServ = this->rosNode->advertiseService(IOBServO);
#if 0
    ros::AdvertiseServiceOptions IOBTickServO =
      ros::AdvertiseServiceOptions::create<std_srvs::Empty>
      (this->robot_name + "/tick_synchronized_command",
       boost::bind(&IOBPlugin::serviceRefCallback, this, _1, _2), ros::VoidPtr(), &this->srvQueue);
    this->tickServ = this->rosNode->advertiseService(IOBTickServO);
#endif
    // ros callback queue for processing subscription
    this->callbackQueeuThread_srv =
      boost::thread(boost::bind(&IOBPlugin::SrvQueueThread, this));
  }

  this->callbackQueeuThread_msg =
    boost::thread(boost::bind(&IOBPlugin::RosQueueThread, this));

  this->updateConnection =
    event::Events::ConnectWorldUpdateBegin(boost::bind(&IOBPlugin::UpdateStates, this));
}
void IOBPlugin::SetJointCommand(const JointCommand::ConstPtr &_msg) {
  this->SetJointCommand_impl(*_msg);
}
void IOBPlugin::SetJointCommand_impl(const JointCommand &_msg) {
  // Update Joint Command
  boost::mutex::scoped_lock lock(this->mutex);

  this->jointCommand.header.stamp = _msg.header.stamp;

  // for jointCommand, only position, velocity and efforts are used.
  if (_msg.position.size() == this->jointCommand.position.size())
    std::copy(_msg.position.begin(), _msg.position.end(), this->jointCommand.position.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
      " elements position[%ld] than expected[%ld]",
      _msg.position.size(), this->jointCommand.position.size());

  if (_msg.velocity.size() == this->jointCommand.velocity.size())
    std::copy(_msg.velocity.begin(), _msg.velocity.end(), this->jointCommand.velocity.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
      " elements velocity[%ld] than expected[%ld]",
      _msg.velocity.size(), this->jointCommand.velocity.size());

  if (_msg.effort.size() == this->jointCommand.effort.size())
    std::copy(_msg.effort.begin(), _msg.effort.end(), this->jointCommand.effort.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
      " elements effort[%ld] than expected[%ld]",
      _msg.effort.size(), this->jointCommand.effort.size());

  if (!this->use_velocity_feedback) {
    // the rest are stored in robotState for publication
    if (_msg.kp_position.size() == this->robotState.kp_position.size())
      std::copy(_msg.kp_position.begin(), _msg.kp_position.end(), this->robotState.kp_position.begin());
    else
      ROS_DEBUG("JointCommand message contains different number of"
                " elements kp_position[%ld] than expected[%ld]",
                _msg.kp_position.size(), this->robotState.kp_position.size());

    if (_msg.ki_position.size() == this->robotState.ki_position.size())
      std::copy(_msg.ki_position.begin(), _msg.ki_position.end(), this->robotState.ki_position.begin());
    else
      ROS_DEBUG("JointCommand message contains different number of"
                " elements ki_position[%ld] than expected[%ld]",
                _msg.ki_position.size(), this->robotState.ki_position.size());

    if (_msg.kd_position.size() == this->robotState.kd_position.size())
      std::copy(_msg.kd_position.begin(), _msg.kd_position.end(), this->robotState.kd_position.begin());
    else
      ROS_DEBUG("JointCommand message contains different number of"
                " elements kd_position[%ld] than expected[%ld]",
                _msg.kd_position.size(), this->robotState.kd_position.size());

    if (_msg.kp_velocity.size() == this->robotState.kp_velocity.size())
      std::copy(_msg.kp_velocity.begin(), _msg.kp_velocity.end(), this->robotState.kp_velocity.begin());
    else
      ROS_DEBUG("JointCommand message contains different number of"
                " elements kp_velocity[%ld] than expected[%ld]",
                _msg.kp_velocity.size(), this->robotState.kp_velocity.size());

    if (_msg.i_effort_min.size() == this->robotState.i_effort_min.size())
      std::copy(_msg.i_effort_min.begin(), _msg.i_effort_min.end(), this->robotState.i_effort_min.begin());
    else
      ROS_DEBUG("JointCommand message contains different number of"
                " elements i_effort_min[%ld] than expected[%ld]",
                _msg.i_effort_min.size(), this->robotState.i_effort_min.size());

    if (_msg.i_effort_max.size() == this->robotState.i_effort_max.size())
      std::copy(_msg.i_effort_max.begin(), _msg.i_effort_max.end(), this->robotState.i_effort_max.begin());
    else
      ROS_DEBUG("JointCommand message contains different number of"
                " elements i_effort_max[%ld] than expected[%ld]",
                _msg.i_effort_max.size(), this->robotState.i_effort_max.size());
  } else {
    // use velocity feedback
    if (_msg.kpv_position.size() == this->robotState.kpv_position.size())
      std::copy(_msg.kpv_position.begin(), _msg.kpv_position.end(), this->robotState.kpv_position.begin());
    else
      ROS_DEBUG("JointCommand message contains different number of"
                " elements kpv_position[%ld] than expected[%ld]",
                _msg.kpv_position.size(), this->robotState.kpv_position.size());

    if (_msg.kpv_velocity.size() == this->robotState.kpv_velocity.size())
      std::copy(_msg.kpv_velocity.begin(), _msg.kpv_velocity.end(), this->robotState.kpv_velocity.begin());
    else
      ROS_DEBUG("JointCommand message contains different number of"
                " elements kpv_velocity[%ld] than expected[%ld]",
                _msg.kpv_velocity.size(), this->robotState.kpv_velocity.size());
  }
  this->jointCommand.desired_controller_period_ms =
    _msg.desired_controller_period_ms;
}
void IOBPlugin::PublishJointState() {
  this->publish_joint_state_counter++;
  if(this->publish_joint_state_step > this->publish_joint_state_counter) {
    return;
  }
  if(this->jointNames.size() != this->joints.size()) {
    ROS_ERROR("joint length miss match %ld != %ld", this->jointNames.size(), this->joints.size());
    return;
  }
  if(this->jointNames.size() == 0) {
    ROS_ERROR("joint length is zero");
    return;
  }
  if(!this->pubJointStateQueue || !this->pubJointState) {
    ROS_ERROR("no publisher %d %d", !this->pubJointStateQueue, !this->pubJointState);
    return;
  }
  // publish joint_state
  sensor_msgs::JointState jstate;
  jstate.header.stamp = this->robotState.header.stamp;
  jstate.name.resize(this->jointNames.size());
  jstate.position.resize(this->joints.size());
  for (unsigned int i = 0; i < this->joints.size(); ++i) {
    jstate.name[i] = this->jointNames[i];
    jstate.position[i] = this->joints[i]->GetAngle(0).Radian();
  }
  this->pubJointStateQueue->push(jstate, this->pubJointState);
  this->publish_joint_state_counter = 0;
}
void IOBPlugin::UpdateStates() {
  //ROS_DEBUG("update");
  common::Time curTime = this->world->GetSimTime();
  if (curTime > this->lastControllerUpdateTime) {
    // gather robot state data
    this->GetRobotStates(curTime);

    this->publish_count++;
    if(this->publish_count % this->publish_step == 0) {
      // publish robot states
      this->pubRobotStateQueue->push(this->robotState, this->pubRobotState);
      if (this->publish_joint_state) this->PublishJointState();
    }

    if (this->use_synchronized_command) {
      {
        boost::unique_lock<boost::mutex> lock(this->uniq_mutex);
        //ROS_DEBUG("return notify");
        return_service_cond_.notify_all();

        //ROS_DEBUG("service wait");
        wait_service_cond_.wait(lock);
      }
    } else if (this->use_loose_synchronized) { // loose synchronization
      ros::Time rnow;
      rnow.fromSec(curTime.Double());
      int counter = 0;
      //if ((rnow - this->jointCommand.header.stamp).toSec() > 0.002) {
      //  ROS_WARN("%f update fail %f", rnow.toSec(), this->jointCommand.header.stamp.toSec());
      //}
      while ((rnow - this->jointCommand.header.stamp).toSec() > this->iob_period) {
        ros::WallDuration(0, 200000).sleep(); // 0.2 ms
        if(counter++ > 100) {
          ROS_WARN_THROTTLE(5, "timeout: loose synchronization / you should check hrpsys is working");
          break;
        }
      }
      //if(counter > 0) {
      //  ROS_WARN("%f recover %f", rnow.toSec(), this->jointCommand.header.stamp.toSec());
      //}
    }
    {
      boost::mutex::scoped_lock lock(this->mutex);
      if (this->use_velocity_feedback) {
        // velocity control
        this->UpdatePID_Velocity_Control((curTime - this->lastControllerUpdateTime).Double());
      } else {
        // effort control
        this->UpdatePIDControl((curTime - this->lastControllerUpdateTime).Double());
      }
    }
    this->lastControllerUpdateTime = curTime;
  }
}

bool IOBPlugin::serviceCallback(hrpsys_gazebo_msgs::SyncCommandRequest &req,
                                hrpsys_gazebo_msgs::SyncCommandResponse &res)
{
  this->SetJointCommand_impl(req.joint_command);
  {
    boost::unique_lock<boost::mutex> lock(this->uniq_mutex);
    //ROS_DEBUG("service notify");
    wait_service_cond_.notify_all();

    //ROS_DEBUG("return wait");
    return_service_cond_.wait(lock);
    res.robot_state = this->robotState;
  }
  return true;
}

bool IOBPlugin::serviceRefCallback(std_srvs::EmptyRequest &req,
                                   std_srvs::EmptyResponse &res)
{
  // set reference to actual
  JointCommand jc;
  jc.position.resize(this->joints.size());
  jc.velocity.resize(this->joints.size());
  jc.effort.resize(this->joints.size());

  for (unsigned int i = 0; i < this->joints.size(); ++i) {
    jc.position[i] = this->joints[i]->GetAngle(0).Radian();
    jc.velocity[i] = 0;
    jc.effort[i] = 0;
  }
  for (unsigned i = 0; i < this->errorTerms.size(); ++i) {
    this->errorTerms[i].q_p = 0;
    this->errorTerms[i].d_q_p_dt = 0;
    this->errorTerms[i].k_i_q_i = 0;
    this->errorTerms[i].qd_p = 0;
  }
  this->SetJointCommand_impl(jc);

  return true;
}

void IOBPlugin::GetRobotStates(const common::Time &_curTime){

  // populate robotState from robot
  this->robotState.header.stamp = ros::Time(_curTime.sec, _curTime.nsec);

  std::shared_ptr<std::vector<double > > vbuf = effortValQueue.at(this->effort_average_cnt);
  // joint states
  for (unsigned int i = 0; i < this->joints.size(); ++i) {
    this->robotState.position[i] = this->joints[i]->GetAngle(0).Radian();
    this->robotState.velocity[i] = this->joints[i]->GetVelocity(0);
    if (this->use_joint_effort) {
      physics::JointPtr j = this->joints[i];
      physics::JointWrench w = j->GetForceTorque(0u);
      {
        math::Vector3 a = j->GetLocalAxis(0u);
        vbuf->at(i) = a.Dot(w.body1Torque);
        this->robotState.effort[i] = 0.0;
        //this->robotState.effort[i] = a.Dot(w.body1Torque);
      }
#if 0 // DEBUG
      {
        math::Vector3 a = j->GetGlobalAxis(0u);
        math::Vector3 m = this->joints[i]->GetChild()->GetWorldPose().rot * w.body2Torque;
        float ret = this->robotState.effort[i] = a.Dot(m);
        ROS_INFO("f[%d]->%f,%f", i, this->robotState.effort[i], ret);
      }
#endif
    } else {
      if (this->use_velocity_feedback) {
        this->robotState.effort[i] = this->joints[i]->GetForce(0);
      }
    }
  }
  if (this->use_joint_effort) {
    for (int j = 0; j < effortValQueue.size(); j++) {
      std::shared_ptr<std::vector<double > > vbuf = effortValQueue.at(j);
      for (int i = 0; i < this->joints.size(); i++) {
        this->robotState.effort[i] += vbuf->at(i);
      }
    }
    if (effortValQueue.size() > 0 ){
      for (int i = 0; i < this->joints.size(); i++) {
        this->robotState.effort[i] *= 1.0/effortValQueue.size();
      }
    } else {
      ROS_WARN("invalid force val queue size 0");
    }
  }
  this->effort_average_cnt = (this->effort_average_cnt+1) % this->effort_average_window_size;

  // enqueue force sensor values
  this->robotState.sensors.resize(this->forceSensorNames.size());
  for (unsigned int i = 0; i < this->forceSensorNames.size(); i++) {
    forceSensorMap::iterator it = this->forceSensors.find(this->forceSensorNames[i]);
    std::shared_ptr<std::vector<std::shared_ptr<geometry_msgs::WrenchStamped> > > forceValQueue = this->forceValQueueMap.find(this->forceSensorNames[i])->second;
    std::shared_ptr<geometry_msgs::WrenchStamped> forceVal = forceValQueue->at(this->force_sensor_average_cnt);
    if(it != this->forceSensors.end()) {
      physics::JointPtr jt = it->second.joint;
      if (!!jt) {
        physics::JointWrench wrench = jt->GetForceTorque(0u);
        this->robotState.sensors[i].name = this->forceSensorNames[i];
        this->robotState.sensors[i].frame_id = it->second.frame_id;
        if (!!it->second.pose) {
          // convert force
          math::Vector3 force_trans = it->second.pose->rot * wrench.body2Force;
          math::Vector3 torque_trans = it->second.pose->rot * wrench.body2Torque;
          // rotate force
          forceVal->wrench.force.x = force_trans.x;
          forceVal->wrench.force.y = force_trans.y;
          forceVal->wrench.force.z = force_trans.z;
          // rotate torque + additional torque
          torque_trans += it->second.pose->pos.Cross(force_trans);
          forceVal->wrench.torque.x = torque_trans.x;
          forceVal->wrench.torque.y = torque_trans.y;
          forceVal->wrench.torque.z = torque_trans.z;
        } else {
          forceVal->wrench.force.x = wrench.body2Force.x;
          forceVal->wrench.force.y = wrench.body2Force.y;
          forceVal->wrench.force.z = wrench.body2Force.z;
          forceVal->wrench.torque.x = wrench.body2Torque.x;
          forceVal->wrench.torque.y = wrench.body2Torque.y;
          forceVal->wrench.torque.z = wrench.body2Torque.z;
        }
      } else {
        ROS_WARN("[ForceSensorPlugin] joint not found for %s", this->forceSensorNames[i].c_str());
      }
    }
    this->robotState.sensors[i].force.x = 0;
    this->robotState.sensors[i].force.y = 0;
    this->robotState.sensors[i].force.z = 0;
    this->robotState.sensors[i].torque.x = 0;
    this->robotState.sensors[i].torque.y = 0;
    this->robotState.sensors[i].torque.z = 0;
    for ( int j=0; j<forceValQueue->size() ; j++ ){
      std::shared_ptr<geometry_msgs::WrenchStamped> forceValBuf = forceValQueue->at(j);
      this->robotState.sensors[i].force.x += forceValBuf->wrench.force.x;
      this->robotState.sensors[i].force.y += forceValBuf->wrench.force.y;
      this->robotState.sensors[i].force.z += forceValBuf->wrench.force.z;
      this->robotState.sensors[i].torque.x += forceValBuf->wrench.torque.x;
      this->robotState.sensors[i].torque.y += forceValBuf->wrench.torque.y;
      this->robotState.sensors[i].torque.z += forceValBuf->wrench.torque.z;
    }
    if ( forceValQueue->size() > 0 ){
      this->robotState.sensors[i].force.x *= 1.0/forceValQueue->size();
      this->robotState.sensors[i].force.y *= 1.0/forceValQueue->size();
      this->robotState.sensors[i].force.z *= 1.0/forceValQueue->size();
      this->robotState.sensors[i].torque.x *= 1.0/forceValQueue->size();
      this->robotState.sensors[i].torque.y *= 1.0/forceValQueue->size();
      this->robotState.sensors[i].torque.z *= 1.0/forceValQueue->size();
    } else {
      ROS_WARN("invalid force val queue size 0");
    }
  }
  this->force_sensor_average_cnt = (this->force_sensor_average_cnt+1) % this->force_sensor_average_window_size;

  // imu sensors
  this->robotState.Imus.resize(this->imuSensorNames.size());
  for (unsigned int i = 0; i < this->imuSensorNames.size(); i++) {
    imuSensorMap::iterator it = this->imuSensors.find(this->imuSensorNames[i]);
    ImuSensorPtr sp = it->second.sensor;
    if(!!sp) {
      this->robotState.Imus[i].name = this->imuSensorNames[i];
      this->robotState.Imus[i].frame_id = it->second.frame_id;
      math::Vector3 wLocal = sp->AngularVelocity();
      math::Vector3 accel = sp->LinearAcceleration();
      math::Quaternion imuRot = sp->Orientation();
      this->robotState.Imus[i].angular_velocity.x = wLocal.x;
      this->robotState.Imus[i].angular_velocity.y = wLocal.y;
      this->robotState.Imus[i].angular_velocity.z = wLocal.z;
      this->robotState.Imus[i].linear_acceleration.x = accel.x;
      this->robotState.Imus[i].linear_acceleration.y = accel.y;
      this->robotState.Imus[i].linear_acceleration.z = accel.z;
      this->robotState.Imus[i].orientation.x = imuRot.x;
      this->robotState.Imus[i].orientation.y = imuRot.y;
      this->robotState.Imus[i].orientation.z = imuRot.z;
      this->robotState.Imus[i].orientation.w = imuRot.w;
    }
  }
  {
    boost::mutex::scoped_lock lock(this->mutex);
    for (unsigned int i = 0; i < this->joints.size(); ++i) {
      this->robotState.ref_position[i] = this->jointCommand.position[i];
      this->robotState.ref_velocity[i] = this->jointCommand.velocity[i];
    }
  }
}

void IOBPlugin::UpdatePID_Velocity_Control(double _dt) {

  /// update pid with feedforward force
  for (unsigned int i = 0; i < this->joints.size(); ++i) {

    // truncate joint position within range of motion
    double positionTarget = math::clamp(this->jointCommand.position[i],
                                        static_cast<double>(this->joints[i]->GetLowStop(0).Radian()),
                                        static_cast<double>(this->joints[i]->GetHighStop(0).Radian()));

    double q_p = positionTarget - this->robotState.position[i];

    if (!math::equal(_dt, 0.0))
      this->errorTerms[i].d_q_p_dt = (q_p - this->errorTerms[i].q_p) / _dt;

    this->errorTerms[i].q_p = q_p;

    this->errorTerms[i].qd_p =
      this->jointCommand.velocity[i] - this->robotState.velocity[i];

    double max_vel = this->joints[i]->GetVelocityLimit(0);
    double j_velocity =
      this->jointCommand.velocity[i] +
      static_cast<double>(this->robotState.kpv_position[i]) * this->errorTerms[i].q_p +
      static_cast<double>(this->robotState.kpv_velocity[i]) * this->errorTerms[i].qd_p;

    // update max force
    this->joints[i]->SetParam("max_force", 0, this->joints[i]->GetEffortLimit(0));
    // clamp max velocity
    j_velocity = math::clamp(j_velocity, -max_vel, max_vel);
#if 0
    ROS_INFO("%d %f / %f %f %f %f",
             i, this->joints[i]->GetMaxForce(0),
             j_velocity, positionTarget, robotState.position[i],
             this->robotState.kpv_position[i]);
#endif
    // apply velocity to joint
    this->joints[i]->SetVelocity(0, j_velocity);
  }
}

void IOBPlugin::UpdatePIDControl(double _dt) {

  /// update pid with feedforward force
  for (unsigned int i = 0; i < this->joints.size(); ++i) {
    // truncate joint position within range of motion
    double positionTarget = math::clamp(this->jointCommand.position[i],
                                        static_cast<double>(this->joints[i]->GetLowStop(0).Radian()),
                                        static_cast<double>(this->joints[i]->GetHighStop(0).Radian()));

    double q_p = positionTarget - this->robotState.position[i];

    if (!math::equal(_dt, 0.0))
      this->errorTerms[i].d_q_p_dt = (q_p - this->errorTerms[i].q_p) / _dt;

    //
    // Take advantage of cfm damping by passing kp_velocity through
    // to intrinsic joint damping coefficient.  Simulating
    // infinite bandwidth kp_velocity.
    //
    // kp_velocity is truncated within (jointDampingModel, jointDampingMax).
    //
    // To take advantage of utilizing full range of cfm damping dynamically
    // for controlling the robot, set model damping (jointDmapingModel)
    // to jointDampingMin first.
    double jointDampingCoef = math::clamp(
      static_cast<double>(this->robotState.kp_velocity[i]),
      this->jointDampingModel[i], this->jointDampingMax[i]);

    // skip set joint damping if value is not changing
    if (!math::equal(this->lastJointCFMDamping[i], jointDampingCoef))
    {
      this->joints[i]->SetDamping(0, jointDampingCoef);
      this->lastJointCFMDamping[i] = jointDampingCoef;
    }

    // approximate effort generated by a non-zero joint velocity state
    // this is the approximate force of the infinite bandwidth
    // kp_velocity term, we'll use this to bound additional forces later.
    // Force generated by cfm damping from damping coefficient smaller than
    // jointDampingModel is generated for free.
    double kpVelocityDampingEffort = 0;
    double kpVelocityDampingCoef =
      jointDampingCoef - this->jointDampingModel[i];
    if (kpVelocityDampingCoef > 0.0)
      kpVelocityDampingEffort =
        kpVelocityDampingCoef * this->robotState.velocity[i];
    //

    this->errorTerms[i].q_p = q_p;

    this->errorTerms[i].qd_p =
      this->jointCommand.velocity[i] - this->robotState.velocity[i];

    this->errorTerms[i].k_i_q_i = math::clamp(this->errorTerms[i].k_i_q_i +
                                              _dt *
                                              static_cast<double>(this->robotState.ki_position[i])
                                              * this->errorTerms[i].q_p,
                                              static_cast<double>(this->robotState.i_effort_min[i]),
                                              static_cast<double>(this->robotState.i_effort_max[i]));

    // use gain params to compute force cmd
    double forceUnclamped =
      static_cast<double>(this->robotState.kp_position[i]) * this->errorTerms[i].q_p +
                                                             this->errorTerms[i].k_i_q_i +
      static_cast<double>(this->robotState.kd_position[i]) * this->errorTerms[i].d_q_p_dt +
      static_cast<double>(this->robotState.kp_velocity[i]) * this->errorTerms[i].qd_p +
                                                             this->jointCommand.effort[i];

    // keep unclamped force for integral tie-back calculation
    double forceClamped = math::clamp(forceUnclamped, -this->effortLimit[i], this->effortLimit[i]);

    // clamp force after integral tie-back
    // shift by kpVelocityDampingEffort to prevent controller from
    // exerting too much force from use of kp_velocity --> cfm damping
    // pass through.
    forceClamped = math::clamp(forceUnclamped,
                               -this->effortLimit[i] + kpVelocityDampingEffort,
                               this->effortLimit[i] + kpVelocityDampingEffort);

    // integral tie-back during control saturation if using integral gain
    if (!math::equal(forceClamped, forceUnclamped) &&
        !math::equal(static_cast<double>(this->robotState.ki_position[i]), 0.0)) {
      // lock integral term to provide continuous control as system moves
      // out of staturation
      this->errorTerms[i].k_i_q_i = math::clamp(this->errorTerms[i].k_i_q_i + (forceClamped - forceUnclamped),
                                                static_cast<double>(this->robotState.i_effort_min[i]),
                                                static_cast<double>(this->robotState.i_effort_max[i]));
    }
    // clamp force after integral tie-back
    forceClamped = math::clamp(forceUnclamped, -this->effortLimit[i], this->effortLimit[i]);

    // apply force to joint
    this->joints[i]->SetForce(0, forceClamped);

    // fill in jointState efforts
    this->robotState.effort[i] = forceClamped;
  }
}

void IOBPlugin::RosQueueThread() {
  static const double timeout = 0.01;

  while (this->rosNode->ok()) {
    ros::spinOnce();
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
void IOBPlugin::SrvQueueThread() {
  while (this->rosNode->ok()) {
    this->srvQueue.callAvailable();
  }
}
}
