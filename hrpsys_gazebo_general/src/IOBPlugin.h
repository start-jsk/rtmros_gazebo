#include <string>
#include <vector>
#include <memory>

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

#include <gazebo/math/gzmath.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/Sensor.hh>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include "hrpsys_gazebo_msgs/JointCommand.h"
#include "hrpsys_gazebo_msgs/RobotState.h"

#include "hrpsys_gazebo_msgs/SyncCommand.h"

#include "PubQueue.h"

namespace gazebo
{
  typedef std::shared_ptr< sensors::ImuSensor > ImuSensorPtr;
  typedef hrpsys_gazebo_msgs::JointCommand JointCommand;
  typedef hrpsys_gazebo_msgs::RobotState RobotState;
  typedef std::shared_ptr< math::Pose > PosePtr;

  class IOBPlugin : public ModelPlugin
  {
  public:
    IOBPlugin();
    virtual ~IOBPlugin();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  private:
    void UpdateStates();
    void RosQueueThread();
    void SrvQueueThread();
    void DeferredLoad();
    void GetRobotStates(const common::Time &_curTime);
    void SetJointCommand(const JointCommand::ConstPtr &_msg);
    void SetJointCommand_impl(const JointCommand &_msg);
    void LoadPIDGainsFromParameter();
    void ZeroJointCommand();
    void UpdatePIDControl(double _dt);
    void UpdatePID_Velocity_Control(double _dt);
    void PublishJointState();

    void GetIMUState(const common::Time &_curTime);
    void GetForceTorqueSensorState(const common::Time &_curTime);

    bool serviceCallback(hrpsys_gazebo_msgs::SyncCommandRequest &req,
                         hrpsys_gazebo_msgs::SyncCommandResponse &res);
    bool serviceRefCallback(std_srvs::EmptyRequest &req,
                            std_srvs::EmptyResponse &res);

    struct force_sensor_info {
      physics::JointPtr joint;
      std::string frame_id;
      PosePtr pose;
    };

    struct imu_sensor_info {
      physics::LinkPtr link;
      ImuSensorPtr sensor;
      std::string sensor_name;
      std::string frame_id;
    };

    ros::NodeHandle* rosNode;
    ros::CallbackQueue rosQueue;
    ros::CallbackQueue srvQueue;

    physics::WorldPtr world;
    physics::ModelPtr model;
    sdf::ElementPtr sdf;

    event::ConnectionPtr updateConnection;

    boost::thread callbackQueeuThread_msg;
    boost::thread callbackQueeuThread_srv;
    boost::thread deferredLoadThread;

    common::Time lastControllerUpdateTime;

    RobotState robotState;
    ros::Publisher pubRobotState;
    PubQueue<RobotState>::Ptr pubRobotStateQueue;

    bool publish_joint_state;
    int  publish_joint_state_step;
    int  publish_joint_state_counter;
    ros::Publisher pubJointState;
    PubQueue<sensor_msgs::JointState>::Ptr pubJointStateQueue;

    ros::ServiceServer jointrefServ;
    ros::ServiceServer controlServ;

    JointCommand jointCommand;
    ros::Subscriber subIOBCommand;

    std::vector<std::string> jointNames;
    physics::Joint_V joints;

    std::vector<double> lastJointCFMDamping;
    std::vector<double> jointDampingModel;
    std::vector<double> jointDampingMax;
    std::vector<double> jointDampingMin;

    typedef std::map< std::string, struct force_sensor_info > forceSensorMap;
    typedef std::map< std::string, struct imu_sensor_info > imuSensorMap;
    std::vector<std::string> forceSensorNames;
    std::vector<std::string> imuSensorNames;
    forceSensorMap forceSensors;
    imuSensorMap imuSensors;

    std::vector<double> effortLimit;

    class ErrorTerms {
      /// error term contributions to final control output
      double q_p;
      double d_q_p_dt;
      double k_i_q_i;  // integral term weighted by k_i
      double qd_p;
      friend class IOBPlugin;
    };
    std::vector<ErrorTerms> errorTerms;

    //
    PubMultiQueue pmq;
    boost::mutex mutex;
    //
    boost::mutex uniq_mutex;
    boost::condition_variable wait_service_cond_;
    boost::condition_variable return_service_cond_;
    //
    std::string robot_name;
    std::string controller_name;
    bool use_synchronized_command;
    bool use_loose_synchronized;
    bool use_velocity_feedback;
    bool use_joint_effort;
    double iob_period;

    static inline int xmlrpc_value_as_int(XmlRpc::XmlRpcValue &v) {
      if((v.getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
         (v.getType() == XmlRpc::XmlRpcValue::TypeInt)) {
        if(v.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
          double d = v;
          return (int)d;
        } else {
          int i = v;
          return i;
        }
      }
      // not number
      return 0;
    }
    static inline double xmlrpc_value_as_double(XmlRpc::XmlRpcValue &v) {
      if((v.getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
         (v.getType() == XmlRpc::XmlRpcValue::TypeInt)) {
        if(v.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
          double d = v;
          return d;
        } else {
          int i = v;
          return (double)i;
        }
      }
      // not number
      return 0.0;
    }
    // force sensor averaging
    int force_sensor_average_window_size;
    int force_sensor_average_cnt;
    std::map<std::string, std::shared_ptr<std::vector<std::shared_ptr<geometry_msgs::WrenchStamped> > > > forceValQueueMap;
    // effort averaging
    int effort_average_cnt;
    int effort_average_window_size;
    std::vector< std::shared_ptr<std::vector<double> > > effortValQueue;
    // stepping data publish cycle
    int publish_count;
    int publish_step;
  };
}
