#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <vector>

#ifndef IOB2
#include "io/iob.h"
#else
#include "iob2.h"
#include <functional>
#endif

#include <ros/ros.h>
#include <boost/algorithm/string.hpp>

#include <hrpsys_gazebo_msgs/JointCommand.h>
#include <hrpsys_gazebo_msgs/RobotState.h>
#include <hrpsys_gazebo_msgs/SyncCommand.h>

#include <hrpUtil/Eigen3d.h>

#ifndef IOB2
typedef hrpsys_gazebo_msgs::JointCommand JointCommand;
typedef hrpsys_gazebo_msgs::RobotState RobotState;

static ros::NodeHandle* rosnode;

static ros::ServiceClient serv_command;
static ros::Publisher pub_joint_command;
static ros::Subscriber sub_robot_state;
static bool iob_synchronized;
static bool start_robothw = false;
static bool use_velocity_feedback = false;
static bool use_servo_on = false;

static JointCommand jointcommand;
static JointCommand initial_jointcommand;

static RobotState js;
static bool init_sub_flag = false;

static std::vector<double> command;
static std::vector<double> prev_command;
static std::vector<std::vector<double> > forces;
static std::vector<std::vector<double> > gyros;
static std::vector<std::vector<double> > accelerometers;
static std::vector<std::vector<double> > attitude_sensors;
static std::vector<std::vector<double> > force_offset;
static std::vector<std::vector<double> > gyro_offset;
static std::vector<std::vector<double> > accel_offset;
static std::vector<int> power;
static std::vector<int> servo;
static bool isLocked = false;
static unsigned long long frame = 0;
static timespec g_ts;
static long g_period_ns=1000000;
static long overwrite_g_period_ns = -1;
static ros::Time rg_ts;
static int num_of_substeps = 1;
#endif

#define CHECK_JOINT_ID(id) if ((id) < 0 || (id) >= number_of_joints()) return E_ID
#define CHECK_FORCE_SENSOR_ID(id) if ((id) < 0 || (id) >= number_of_force_sensors()) return E_ID
#define CHECK_ACCELEROMETER_ID(id) if ((id) < 0 || (id) >= number_of_accelerometers()) return E_ID
#define CHECK_GYRO_SENSOR_ID(id) if ((id) < 0 || (id) >= number_of_gyro_sensors()) return E_ID
#define CHECK_ATTITUDE_SENSOR_ID(id) if ((id) < 0 || (id) >= number_of_attitude_sensors()) return E_ID

#define JOINT_ID_REAL2MODEL(id) joint_real2model_vec[id]
#define JOINT_ID_MODEL2REAL(id) joint_id_model2real(id)
#define NUM_OF_REAL_JOINT (joint_real2model_vec.size())

#ifndef IOB2
static std::map<int, int> joint_model2real_map;
static std::vector<int>   joint_real2model_vec;
#endif

#ifndef IOB2
static inline int joint_id_model2real(int id)
#else
inline int iob::joint_id_model2real(int id)
#endif
{
  std::map< int, int>::iterator it = joint_model2real_map.find (id);

  if (it == joint_model2real_map.end()) {
    return -1;
  } else {
    return it->second;
  }
}

#ifndef IOB2
static inline void tick_service_command()
#else
inline void iob::tick_service_command()
#endif
{
  // tick gazebo ...
  hrpsys_gazebo_msgs::SyncCommandRequest req;
  // req.joint_command = jointcommand; // do not need, because just tick gazebo time
  hrpsys_gazebo_msgs::SyncCommandResponse res;
  // std::cerr << "[iob] tick srv c" << std::endl;
  serv_command.call(req, res);
  // std::cerr << "[iob] tick srv r" << std::endl;
  js = res.robot_state;
}

#ifdef __APPLE__
typedef int clockid_t;
#define CLOCK_MONOTONIC 0
#include <mach/mach_time.h>
int clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    if (clk_id != CLOCK_MONOTONIC) return -1;

    uint64_t clk;
    clk = mach_absolute_time();

    static mach_timebase_info_data_t info = {0,0};
    if (info.denom == 0) mach_timebase_info(&info);

    uint64_t elapsednano = clk * (info.numer / info.denom);

    tp->tv_sec = elapsednano * 1e-9;
    tp->tv_nsec = elapsednano - (tp->tv_sec * 1e9);
    return 0;
}

#define TIMER_ABSTIME 0
int clock_nanosleep(clockid_t clk_id, int flags, struct timespec *tp,
    struct timespec *remain)
{
    if (clk_id != CLOCK_MONOTONIC || flags != TIMER_ABSTIME) return -1;

    static mach_timebase_info_data_t info = {0,0};
    if (info.denom == 0) mach_timebase_info(&info);

    uint64_t clk = (tp->tv_sec*1e9 + tp->tv_nsec)/(info.numer/info.denom);

    mach_wait_until(clk);
    return 0;
}
#endif

#ifndef IOB2
int number_of_joints()
#else
int iob::number_of_joints()
#endif
{
    return (int)command.size();
}

#ifndef IOB2
int number_of_force_sensors()
#else
int iob::number_of_force_sensors()
#endif
{
    return (int)forces.size();
}

#ifndef IOB2
int number_of_gyro_sensors()
#else
int iob::number_of_gyro_sensors()
#endif
{
    return (int)gyros.size();
}

#ifndef IOB2
int number_of_accelerometers()
#else
int iob::number_of_accelerometers()
#endif
{
    return (int)accelerometers.size();
}

#ifndef IOB2
int number_of_attitude_sensors()
#else
int iob::number_of_attitude_sensors()
#endif
{
    return (int)attitude_sensors.size();
}

#ifndef IOB2
int set_number_of_joints(int num)
#else
int iob::set_number_of_joints(int num)
#endif
{
    std::cerr << ";; IOB / set num of joint = " << num << std::endl;
    command.resize(num);
    prev_command.resize(num);
    power.resize(num);
    servo.resize(num);
    for (int i=0; i<num; i++){
        command[i] = power[i] = servo[i] = prev_command[i] = 0;
    }
    return TRUE;
}

#ifndef IOB2
int set_number_of_force_sensors(int num)
#else
int iob::set_number_of_force_sensors(int num)
#endif
{
    std::cerr << ";; set_number_of_force_sensors = " << num << std::endl;
    forces.resize(num);
    force_offset.resize(num);
    for (unsigned int i=0; i<forces.size();i++){
        forces[i].resize(6);
        force_offset[i].resize(6);
        for (int j=0; j<6; j++){
            forces[i][j] = 0;
            force_offset[i][j] = 0;
        }
    }
    return TRUE;
}

#ifndef IOB2
int set_number_of_gyro_sensors(int num)
#else
int iob::set_number_of_gyro_sensors(int num)
#endif
{
    std::cerr << ";; set_number_of_gyro_sensors = " << num << std::endl;
    gyros.resize(num);
    gyro_offset.resize(num);
    for (unsigned int i=0; i<gyros.size();i++){
        gyros[i].resize(3);
        gyro_offset[i].resize(3);
        for (int j=0; j<3; j++){
            gyros[i][j] = 0.0;
            gyro_offset[i][j] = 0.0;
        }
    }
    return TRUE;
}

#ifndef IOB2
int set_number_of_accelerometers(int num)
#else
int iob::set_number_of_accelerometers(int num)
#endif
{
  std::cerr << ";; set_number_of_number_of_accelerometers = " << num << std::endl;
    accelerometers.resize(num);
    accel_offset.resize(num);
    for (unsigned int i=0; i<accelerometers.size();i++){
        accelerometers[i].resize(3);
        accel_offset[i].resize(3);
        for (int j=0; j<3; j++){
            accelerometers[i][j] = j == 2 ? 9.81 : 0.0; // z direction should be virtical???
            accel_offset[i][j] = 0;
        }
    }
    return TRUE;
}

#ifndef IOB2
int set_number_of_attitude_sensors(int num)
{
    attitude_sensors.resize(num);
    for (unsigned int i=0; i<attitude_sensors.size();i++){
        attitude_sensors[i].resize(3);
        for (int j=0; j<3; j++){
            attitude_sensors[i][j] = 0.0;
        }
    }
    return TRUE;
}
#endif

#ifndef IOB2
int read_power_state(int id, int *s)
#else
int iob::read_power_state(int id, int *s)
#endif
{
    CHECK_JOINT_ID(id);
    *s = power[id];
    return TRUE;
}

#ifndef IOB2
int write_power_command(int id, int com)
#else
int iob::write_power_command(int id, int com)
#endif
{
    CHECK_JOINT_ID(id);
    power[id] = com;
    return TRUE;
}

#ifndef IOB2
int read_power_command(int id, int *com)
#else
int iob::read_power_command(int id, int *com)
#endif
{
    CHECK_JOINT_ID(id);
    *com = power[id];
    return TRUE;
}

#ifndef IOB2
int read_servo_state(int id, int *s)
#else
int iob::read_servo_state(int id, int *s)
#endif
{
    CHECK_JOINT_ID(id);
    *s = servo[id];
    return TRUE;
}

#ifndef IOB2
int read_servo_alarm(int id, int *a)
#else
int iob::read_servo_alarm(int id, int *a)
#endif
{
    CHECK_JOINT_ID(id);
    *a = 0;
    return TRUE;
}

#ifndef IOB2
int read_control_mode(int id, joint_control_mode *s)
#else
int iob::read_control_mode(int id, joint_control_mode *s)
#endif
{
    CHECK_JOINT_ID(id);
    *s = JCM_POSITION;
    return TRUE;
}

#ifndef IOB2
int write_control_mode(int id, joint_control_mode s)
#else
int iob::write_control_mode(int id, joint_control_mode s)
#endif
{
    CHECK_JOINT_ID(id);
    return TRUE;
}

#ifndef IOB2
int read_actual_angle(int id, double *angle)
#else
int iob::read_actual_angle(int id, double *angle)
#endif
{
  CHECK_JOINT_ID(id);

  if(init_sub_flag){
    if (JOINT_ID_MODEL2REAL(id) < 0) {
      *angle = command[id];
    }else{
      *angle = js.position[JOINT_ID_MODEL2REAL(id)];
    }
  }else{
    *angle = command[id];
  }
  return TRUE;
}

#ifndef IOB2
int read_actual_angles(double *angles)
#else
int iob::read_actual_angles(double *angles)
#endif
{
    for (int i=0; i<number_of_joints(); i++){
        read_actual_angle(i, &angles[i]);
    }
    return TRUE;
}

#ifndef IOB2
int read_actual_torques(double *torques)
#else
int iob::read_actual_torques(double *torques)
#endif
{
  if(init_sub_flag) {
    for(int i=0; i<number_of_joints(); i++){
      if(JOINT_ID_MODEL2REAL(i) < 0) {
        *(torques+i) = -1;
      }else{
        *(torques+i) = js.effort[JOINT_ID_MODEL2REAL(i)];
      }
    }
  }

  return TRUE;
}

#ifndef IOB2
int read_command_torque(int id, double *torque)
#else
int iob::read_command_torque(int id, double *torque)
#endif
{
    return FALSE;
}

#ifndef IOB2
int write_command_torque(int id, double torque)
#else
  int iob::write_command_torque(int id, double torque)
#endif
{
    return FALSE;
}

#ifndef IOB2
int read_command_torques(double *torques)
#else
int iob::read_command_torques(double *torques)
#endif
{
    return FALSE;
}

#ifndef IOB2
int write_command_torques(const double *torques)
#else
int iob::write_command_torques(const double *torques)
#endif
{
    return FALSE;
}

#ifndef IOB2
int read_command_angle(int id, double *angle)
#else
int iob::read_command_angle(int id, double *angle)
#endif
{
    CHECK_JOINT_ID(id);
    *angle = command[id];
    return TRUE;
}

#ifndef IOB2
int write_command_angle(int id, double angle)
#else
int iob::write_command_angle(int id, double angle)
#endif
{
    CHECK_JOINT_ID(id);
    command[id] = angle;
    return TRUE;
}

#ifndef IOB2
int read_command_angles(double *angles)
#else
int iob::read_command_angles(double *angles)
#endif
{
    for (int i=0; i<number_of_joints(); i++){
        angles[i] = command[i];
    }
    return TRUE;
}

#ifndef IOB2
int write_command_angles(const double *angles)
#else
int iob::write_command_angles(const double *angles)
#endif
{
    //std::cerr << "[iob] write command[" << frame << "]" << std::endl;
    for (int i=0; i<number_of_joints(); i++){
      prev_command[i] = command[i];
      command[i] = angles[i];
    }

    JointCommand send_com(jointcommand);

    send_com.header.stamp = ros::Time::now();

    for (int i=0; i<NUM_OF_REAL_JOINT; i++) {
      if (JOINT_ID_REAL2MODEL(i) < number_of_joints()){
	if (use_servo_on){
	  if (servo[JOINT_ID_REAL2MODEL(i)] > 0) send_com.servo[i] = true;
	  else send_com.servo[i] = false;
	  
	  if (power[JOINT_ID_REAL2MODEL(i)] > 0) send_com.power[i] = true;
	  else send_com.power[i] = false;
	}
	send_com.position[i] = command[JOINT_ID_REAL2MODEL(i)];
	send_com.velocity[i] = (command[JOINT_ID_REAL2MODEL(i)] - prev_command[JOINT_ID_REAL2MODEL(i)]) / (overwrite_g_period_ns * 1e-9);
      }
    }

    if (iob_synchronized) {
      hrpsys_gazebo_msgs::SyncCommandRequest req;
      req.joint_command = send_com;
      hrpsys_gazebo_msgs::SyncCommandResponse res;
      // std::cerr << "[iob] srv c" << std::endl;
      serv_command.call(req, res);
      // std::cerr << "[iob] srv r" << std::endl;
      js = res.robot_state;
      init_sub_flag = true;
    } else {
      pub_joint_command.publish(send_com);
      ros::spinOnce();
    }
    if (!start_robothw) {
      frame = 0;
      start_robothw = true;
    }
    
    return TRUE;
}

#ifndef IOB2
int read_pgain(int id, double *gain)
#else
int iob::read_pgain(int id, double *gain)
#endif
{
  if(JOINT_ID_MODEL2REAL(id) < 0) {
    std::cerr << ";;; read pgain: " << id << " failed." << std::endl;
  }else{
    int iid = JOINT_ID_MODEL2REAL(id);
    if (!use_velocity_feedback) {
      *(gain) = jointcommand.kp_position[iid] / initial_jointcommand.kp_position[iid];
      //std::cerr << ";;; read gain: " << id << " = " << *gain << std::endl;
    } else {
      if (initial_jointcommand.kpv_position[iid] <= 0) {
        *(gain) = 1.0;
      } else {
        *(gain) = jointcommand.kpv_position[iid] / initial_jointcommand.kpv_position[iid];
      }
    }
  }
  return TRUE;
}

#ifndef IOB2
int write_pgain(int id, double gain)
#else
int iob::write_pgain(int id, double gain)
#endif
{

  if(JOINT_ID_MODEL2REAL(id) < 0) {
    std::cerr << ";;; write pgain: " << id << " failed." << std::endl;
  }else{
    int iid = JOINT_ID_MODEL2REAL(id);
    if(!use_velocity_feedback) {
      jointcommand.kp_position[iid] = gain * initial_jointcommand.kp_position[iid];
      //std::cerr << ";;; write pgain: " << id << " = " << gain << std::endl;
    } else {
      jointcommand.kpv_position[iid] = gain * initial_jointcommand.kpv_position[iid];
    }
  }
  return TRUE;
}

#ifndef IOB2
int read_dgain(int id, double *gain)
#else
int iob::read_dgain(int id, double *gain)
#endif
{
  if(JOINT_ID_MODEL2REAL(id) < 0) {
    //
  }else{
    int iid = JOINT_ID_MODEL2REAL(id);
    if (!use_velocity_feedback) {
      *(gain) = jointcommand.kd_position[iid] / initial_jointcommand.kd_position[iid];
      //std::cerr << ";;; read dgain: " << id << " = " << *gain << std::endl;
    } else {
      if(initial_jointcommand.kpv_velocity[iid] <= 0) {
        *(gain) = 1.0;
      } else {
        *(gain) = jointcommand.kpv_velocity[iid] / initial_jointcommand.kpv_velocity[iid];
      }
    }
  }
  return TRUE;
}

#ifndef IOB2
int write_dgain(int id, double gain)
#else
int iob::write_dgain(int id, double gain)
#endif
{
  if(JOINT_ID_MODEL2REAL(id) < 0) {
    //
  }else{
    int iid = JOINT_ID_MODEL2REAL(id);
    if (!use_velocity_feedback) {
      jointcommand.kd_position[iid] =
        gain * initial_jointcommand.kd_position[iid];
      //std::cerr << ";;; write dgain: " << id << " = " << gain << std::endl;
    } else {
      jointcommand.kpv_velocity[iid] =
        gain * initial_jointcommand.kpv_velocity[iid];
    }
  }
  return TRUE;
}

#ifndef IOB2
int read_force_sensor(int id, double *forces)
#else
int iob::read_force_sensor(int id, double *forces)
#endif
{
  CHECK_FORCE_SENSOR_ID(id);

  if (id >= js.sensors.size()) {
    return E_ID;
  }
  if (init_sub_flag) {
    forces[0] = js.sensors[id].force.x + force_offset[id][0];
    forces[1] = js.sensors[id].force.y + force_offset[id][1];
    forces[2] = js.sensors[id].force.z + force_offset[id][2];
    forces[3] = js.sensors[id].torque.x + force_offset[id][3];
    forces[4] = js.sensors[id].torque.y + force_offset[id][4];
    forces[5] = js.sensors[id].torque.z + force_offset[id][5];
  } else {
    forces[0] = forces[1] = forces[2] = forces[3] = forces[4] = forces[5] = 0.0;
  }
  return TRUE;
}

#ifndef IOB2
int read_gyro_sensor(int id, double *rates)
#else
int iob::read_gyro_sensor(int id, double *rates)
#endif
{
  CHECK_GYRO_SENSOR_ID(id);
  if(init_sub_flag){
    if (id >= js.Imus.size()) {
      return E_ID;
    }
#if 0
    Eigen::Quaternion<double> q(js.Imus[id].orientation.w,
                                js.Imus[id].orientation.x,
                                js.Imus[id].orientation.y,
                                js.Imus[id].orientation.z);
    hrp::Vector3 rpy = hrp::rpyFromRot(q.toRotationMatrix());
    rates[0] = rpy[0];
    rates[1] = rpy[1];
    rates[2] = rpy[2];
#endif
    rates[0] = js.Imus[id].angular_velocity.x + gyro_offset[id][0];
    rates[1] = js.Imus[id].angular_velocity.y + gyro_offset[id][1];
    rates[2] = js.Imus[id].angular_velocity.z + gyro_offset[id][2];
  } else {
    // tempolary values when sensor is not ready.
    rates[0] = rates[1] = rates[2] = 0.0;
  }
  //fprintf(stderr, "rates[%ld]: %f %f %f\n", frame, rates[0], rates[1], rates[2]);
  return TRUE;
}

#ifndef IOB2
int read_accelerometer(int id, double *accels)
#else
int iob::read_accelerometer(int id, double *accels)
#endif
{
  CHECK_ACCELEROMETER_ID(id);

  if(init_sub_flag){
    if (id >= js.Imus.size()) {
      return E_ID;
    }
    accels[0] = js.Imus[id].linear_acceleration.x + accel_offset[id][0];
    accels[1] = js.Imus[id].linear_acceleration.y + accel_offset[id][1];
    accels[2] = js.Imus[id].linear_acceleration.z + accel_offset[id][2];
  } else {
    // tempolary values when sensor is not ready.
    //accels[0] = accels[1] = accels[2] = 0.0;
    accels[0] = accels[1] = 0.0;
    accels[2] = -9.8;
  }
  return TRUE;
}

#ifndef IOB2
int read_touch_sensors(unsigned short *onoff)
{
    return FALSE;
}
#endif

#ifndef IOB2
int read_attitude_sensor(int id, double *att)
#else
int iob::read_attitude_sensor(int id, double *att)
#endif
{
  return FALSE;
}

#ifndef IOB2
int read_current(int id, double *mcurrent)
{
    return FALSE;
}
#endif

#ifndef IOB2
int read_current_limit(int id, double *v)
{
    return FALSE;
}
#endif

#ifndef IOB2
int read_currents(double *currents)
{
    return FALSE;
}
#endif

#ifndef IOB2
int read_gauges(double *gauges)
{
    return FALSE;
}
#endif

#ifndef IOB2
int read_actual_velocity(int id, double *vel)
#else
int iob::read_actual_velocity(int id, double *vel)
#endif
{
    // TODO: impliment here
    return FALSE;
}

#ifndef IOB2
int read_command_velocity(int id, double *vel)
#else
int iob::read_command_velocity(int id, double *vel)
#endif
{
    // TODO: impliment here
    return FALSE;
}

#ifndef IOB2
int write_command_velocity(int id, double vel)
#else
int iob::write_command_velocity(int id, double vel)
#endif
{
    // TODO: impliment here
    return FALSE;
}

#ifndef IOB2
int read_actual_velocities(double *vels)
#else
int iob::read_actual_velocities(double *vels)
#endif
{
    // TODO: impliment here
    return FALSE;
}

#ifndef IOB2
int read_command_velocities(double *vels)
#else
int iob::read_command_velocities(double *vels)
#endif
{
    // TODO: impliment here
    return FALSE;
}

#ifndef IOB2
int write_command_velocities(const double *vels)
#else
int iob::write_command_velocities(const double *vels)
#endif
{
    // TODO: impliment here
    return FALSE;
}

#ifndef IOB2
int read_temperature(int id, double *v)
#else
int iob::read_temperature(int id, double *v)
#endif
{
    return FALSE;
}

#ifndef IOB2
int write_servo(int id, int com)
#else
int iob::write_servo(int id, int com)
#endif
{
  std::cerr << "servo: id: " << id;
  std::cerr << "(" << servo.size() << ")";
  std::cerr << " - " << com << std::endl;
    servo[id] = com;
    return TRUE;
}

#ifndef IOB2
int write_dio(unsigned short buf)
{
    return FALSE;
}
#endif

// callback
#ifndef IOB2
static void setJointStates(const RobotState::ConstPtr &_js)
#else
void iob::setJointStates(const RobotState::ConstPtr &_js)
#endif
{
  ROS_DEBUG("[iob] subscribe RobotState");
  js = *_js;
  init_sub_flag = true;
  //if (iob_synchronized) {
  //ROS_WARN("iob subscribed wrong topic ...");
  //}
}

#ifndef IOB2
int open_iob(void)
#else
  int iob::open_iob(std::string iob_name)
#endif
{
    static bool isInitialized = false;
    if ( isInitialized ) return TRUE;
    isInitialized = true;

    std::cerr << "[iob] Open IOB / start " << std::endl;

    std::string node_name;
    {
#ifndef IOB2
      char *ret = getenv("HRPSYS_GAZEBO_IOB_NAME");
#else
      char *ret = getenv((iob_name+"_HRPSYS_GAZEBO_IOB_NAME").c_str());
#endif
      if (ret != NULL) {
        node_name.assign(ret);
      } else {
        node_name = "hrpsys_gazebo_iob";
      }
      std::cerr << "[iob] set node name : " << node_name << std::endl;
    }

#ifdef IOB2
    if (!ros::isInitialized()){
#endif
    std::map<std::string, std::string> arg;
    ros::init(arg, "hrpsys_gazebo_iob", ros::init_options::NoSigintHandler);
#ifdef IOB2
    }
#endif
    rosnode = new ros::NodeHandle();
    ros::WallDuration(0.5).sleep(); // wait for initializing ros

    std::string controller_name;
    { // setting configuration name
#ifndef IOB2
      char *ret = getenv("HRPSYS_GAZEBO_CONFIGURATION");
#else
      char *ret = getenv((iob_name+"_HRPSYS_GAZEBO_CONFIGURATION").c_str());
#endif
      if (ret != NULL) {
        controller_name.assign(ret);
      } else {
        controller_name = "hrpsys_gazebo_configuration";
      }
      ROS_INFO_STREAM( "[iob] set controller_name : " << controller_name);
    }
    std::string robotname;
    { // setting configuration name
#ifndef IOB2
      char *ret = getenv("HRPSYS_GAZEBO_ROBOTNAME");
#else
      char *ret = getenv((iob_name+"_HRPSYS_GAZEBO_ROBOTNAME").c_str());
#endif
      if (ret != NULL) {
        robotname.assign(ret);
        // controller_name -> robotname/controller_name
        controller_name = robotname + "/" + controller_name;
      } else {
        std::string rname_str = std::string(controller_name) + "/robotname";
        rosnode->getParam(rname_str, robotname);
      }
      if (robotname.empty()) {
        ROS_ERROR("[iob] did not find robot_name");
        robotname = "default";
      }
      ROS_INFO_STREAM( "[iob] set robot_name : " << robotname);
    }

    { // setting synchronized
#ifndef IOB2
      char *ret = getenv("HRPSYS_GAZEBO_IOB_SYNCHRONIZED");
#else
      char *ret = getenv((iob_name+"_HRPSYS_GAZEBO_IOB_SYNCHRONIZED").c_str());
#endif
      if (ret != NULL) {
        std::string ret_str(ret);
        if (ret_str.size() > 0) {
          iob_synchronized = true;
        }
      } else {
        iob_synchronized = false;
      }
      if(rosnode->hasParam(controller_name + "/use_synchronized_command")) {
        rosnode->getParam(controller_name + "/use_synchronized_command", iob_synchronized);
      }
      if(iob_synchronized) ROS_INFO("[iob] use synchronized command");
    }
    { // setting substeps
#ifndef IOB2
      char *ret = getenv("HRPSYS_GAZEBO_IOB_SUBSTEPS");
#else
      char *ret = getenv((iob_name+"_HRPSYS_GAZEBO_IOB_SUBSTEPS").c_str());
#endif
      if (ret != NULL) {
        int num = atoi(ret);
        if (num > 0) {
          num_of_substeps = num;
          ROS_INFO("[iob] use substeps %d", num);
        }
      }
      if(rosnode->hasParam(controller_name + "/iob_substeps")) {
        rosnode->getParam(controller_name + "/iob_substeps", num_of_substeps);
        ROS_INFO("[iob] use substeps %d", num_of_substeps);
      }
    }
    { // settting rate
      if(rosnode->hasParam(controller_name + "/iob_rate")) {
        double rate = 0;
        rosnode->getParam(controller_name + "/iob_rate", rate);
        overwrite_g_period_ns = (long) ((1000 * 1000 * 1000) / rate);
        fprintf(stderr, "iob::period %d\n", int(overwrite_g_period_ns));
        ROS_INFO("[iob] period_ns %d", int(overwrite_g_period_ns));
      }
    }

    joint_real2model_vec.resize(0);

    if (rosnode->hasParam(controller_name + "/joint_id_list")) {
      XmlRpc::XmlRpcValue param_val;
      rosnode->getParam(controller_name + "/joint_id_list", param_val);
      if (param_val.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for(int i = 0; i < param_val.size(); i++) {
          int num = param_val[i];
          joint_real2model_vec.push_back(num);
        }
      } else {
        ROS_WARN("[iob] %s/joint_id_list is not list of integer", controller_name.c_str());
      }
    } else {
      ROS_DEBUG("[iob] %s/joint_id_list is nothing", controller_name.c_str());
    }
    if (rosnode->hasParam(controller_name + "/use_velocity_feedback")) {
        bool ret;
        rosnode->getParam(controller_name + "/use_velocity_feedback", ret);
        use_velocity_feedback = ret;
        if(ret) {
          ROS_INFO("[iob] use_velocity_feedback");
        }
    }
    if (rosnode->hasParam(controller_name + "/use_servo_on")) {
        bool ret;
        rosnode->getParam(controller_name + "/use_servo_on", ret);
        use_servo_on = ret;
        if(ret) {
          ROS_INFO("[iob] use_servo_on");
        }
    }
    XmlRpc::XmlRpcValue param_val;
    std::vector<std::string> joint_lst;
    rosnode->getParam(controller_name + "/joints", param_val);
    if (param_val.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for(unsigned int s = 0; s < param_val.size(); s++) {
          std::string nstr = param_val[s];
          ROS_DEBUG("add joint: %s", nstr.c_str());
          joint_lst.push_back(nstr);
        }
    } else {
      ROS_ERROR("[iob] %s/joints is not list of joint name", controller_name.c_str());
    }

    if (joint_real2model_vec.size() == 0) {
      for(unsigned int i = 0; i < joint_lst.size(); i++) {
        joint_real2model_vec.push_back(i);
      }
    } else if (joint_real2model_vec.size() != joint_lst.size()) {
      ROS_ERROR("[iob] size differece on joint_id_list and joints (%ld,  %ld)",
                joint_real2model_vec.size(), joint_lst.size());
    }

    for(unsigned int i = 0; i < joint_real2model_vec.size(); i++) {
      joint_model2real_map[joint_real2model_vec[i]] = i;
    }

    unsigned int n = NUM_OF_REAL_JOINT;

    initial_jointcommand.servo.resize(n);
    initial_jointcommand.power.resize(n);
    
    initial_jointcommand.position.resize(n);
    initial_jointcommand.velocity.resize(n);
    //initial_jointcommand.effort.resize(n);
    initial_jointcommand.effort.resize(0);

    if(!use_velocity_feedback) {
      initial_jointcommand.kp_position.resize(n);
      initial_jointcommand.ki_position.resize(n);
      initial_jointcommand.kd_position.resize(n);
      initial_jointcommand.kp_velocity.resize(n);
      initial_jointcommand.i_effort_min.resize(n);
      initial_jointcommand.i_effort_max.resize(n);
    } else {
      initial_jointcommand.kpv_position.resize(n);
      initial_jointcommand.kpv_velocity.resize(n);
    }

    for (unsigned int i = 0; i < NUM_OF_REAL_JOINT; ++i) {
      if (use_servo_on){
	initial_jointcommand.servo[i] = false;
	initial_jointcommand.power[i] = false;
      }else{
	initial_jointcommand.servo[i] = true;
	initial_jointcommand.power[i] = true;
      }
    }
    
    for (unsigned int i = 0; i < joint_lst.size(); ++i) {
      std::string joint_ns(controller_name);
      joint_ns += ("/gains/" + joint_lst[i] + "/");

      if (!use_velocity_feedback) {
        double p_val = 0, i_val = 0, d_val = 0, i_clamp_val = 0, vp_val = 0;
        std::string p_str = std::string(joint_ns)+"p";
        std::string i_str = std::string(joint_ns)+"i";
        std::string d_str = std::string(joint_ns)+"d";
        std::string i_clamp_str = std::string(joint_ns)+"i_clamp";
        std::string vp_str = std::string(joint_ns)+"vp";
        if (!rosnode->getParam(p_str, p_val)) {
          ROS_WARN("[iob] couldn't find a P param for %s", joint_ns.c_str());
        }
        if (!rosnode->getParam(i_str, i_val)) {
          ROS_WARN("[iob] couldn't find a I param for %s", joint_ns.c_str());
        }
        if (!rosnode->getParam(d_str, d_val)) {
          ROS_WARN("[iob] couldn't find a D param for %s", joint_ns.c_str());
        }
        if (!rosnode->getParam(i_clamp_str, i_clamp_val)) {
          ROS_WARN("[iob] couldn't find a I_CLAMP param for %s", joint_ns.c_str());
        }
        if (!rosnode->getParam(vp_str, vp_val)) {
          ROS_WARN("[iob] couldn't find a VP param for %s", joint_ns.c_str());
        }
        // store these directly on altasState, more efficient for pub later
        initial_jointcommand.kp_position[i] = p_val;
        initial_jointcommand.ki_position[i] = i_val;
        initial_jointcommand.kd_position[i] = d_val;
        initial_jointcommand.i_effort_min[i] = -i_clamp_val;
        initial_jointcommand.i_effort_max[i] = i_clamp_val;
        initial_jointcommand.velocity[i]     = 0;
        //initial_jointcommand.effort[i]       = 0;
        initial_jointcommand.kp_velocity[i]  = vp_val;
      } else {
        // velocity feedback
        double p_v_val = 0, vp_v_val = 0;
        std::string p_v_str  = std::string(joint_ns) + "p_v";
        std::string vp_v_str = std::string(joint_ns) + "vp_v";
        if (!rosnode->getParam(p_v_str, p_v_val)) {
          ROS_WARN("[iob] couldn't find a P_V param for %s", joint_ns.c_str());
        }
        if (!rosnode->getParam(vp_v_str, vp_v_val)) {
          ROS_WARN("[iob] couldn't find a VP_V param for %s", joint_ns.c_str());
        }
        initial_jointcommand.kpv_position[i] = p_v_val;
        initial_jointcommand.kpv_velocity[i] = vp_v_val;
      }
    }

    initial_jointcommand.desired_controller_period_ms =
      static_cast<unsigned int>(overwrite_g_period_ns * 1e-6);

    if (iob_synchronized) {
      serv_command =
        ros::service::createClient<hrpsys_gazebo_msgs::SyncCommand> (robotname + "/iob_command", true); // persistent = true,
      ROS_INFO_STREAM("[iob] waiting service " << robotname << "/iob_command");
      serv_command.waitForExistence();
      ROS_INFO_STREAM("[iob] found service " <<  robotname << "/iob_command");
    } else {
      pub_joint_command = rosnode->advertise <JointCommand> (robotname + "/joint_command", 1, true);

      // ros topic subscribtions
      ros::SubscribeOptions jointStatesSo =
#ifndef IOB2
        ros::SubscribeOptions::create<RobotState>(robotname + "/robot_state", 1, setJointStates,
						  ros::VoidPtr(), rosnode->getCallbackQueue());
#else
      ros::SubscribeOptions::create<RobotState>(robotname + "/robot_state", 1, std::bind(&iob::setJointStates,this,std::placeholders::_1),
	                                          ros::VoidPtr(), rosnode->getCallbackQueue());
#endif
      // Because TCP causes bursty communication with high jitter,
      // declare a preference on UDP connections for receiving
      // joint states, which we want to get at a high rate.
      // Note that we'll still accept TCP connections for this topic
      // (e.g., from rospy nodes, which don't support UDP);
      // we just prefer UDP.
      // temporary use TCP / Using UDP occured some problem when message size more than 1500.
      //jointStatesSo.transport_hints =
      //ros::TransportHints().maxDatagramSize(3000).unreliable().reliable().tcpNoDelay(true);
      jointStatesSo.transport_hints =
        ros::TransportHints().reliable().tcpNoDelay(true);
      sub_robot_state = rosnode->subscribe(jointStatesSo);
    }

    for (int i=0; i < number_of_joints(); i++){
        command[i] = 0.0;
        power[i] = OFF;
        servo[i] = OFF;
    }
    clock_gettime(CLOCK_MONOTONIC, &g_ts);
    rg_ts = ros::Time::now();

    jointcommand = initial_jointcommand;
    std::cerr << "[iob]  " << number_of_joints() << " / " << initial_jointcommand.position.size() << " / " << NUM_OF_REAL_JOINT << std::endl;

    if (iob_synchronized) {
      hrpsys_gazebo_msgs::SyncCommandRequest req;
      req.joint_command = jointcommand;
      hrpsys_gazebo_msgs::SyncCommandResponse res;
      std::cerr << "[iob] first service call" << std::endl;
      serv_command.call(req, res);
      std::cerr << "[iob] first service returned" << std::endl;
      js = res.robot_state;
      init_sub_flag = true;
    } else {
      std::cerr << "[iob] block until subscribing first robot_state";
      ros::Time start_tm = ros::Time::now();
      ros::Rate rr(100);
      ros::spinOnce();
      while (!init_sub_flag) {
        if ((ros::Time::now() - start_tm).toSec() > 5.0) {
          std::cerr << "[iob] timeout for waiting robot_state";
          break;
        }
        std::cerr << ".";
        rr.sleep();
        ros::spinOnce();
      }
    }

    std::cerr << std::endl << "[iob] Open IOB / finish " << std::endl;

    return TRUE;
}

#ifndef IOB2
int close_iob(void)
#else
int iob::close_iob(void)
#endif
{
    std::cerr << "[iob] IOB is closed" << std::endl;
    return TRUE;
}

#ifndef IOB2
int reset_body(void)
#else
int iob::reset_body(void)
#endif
{
    for (int i=0; i<number_of_joints(); i++){
        power[i] = servo[i] = OFF;
    }
    return TRUE;
}

#ifndef IOB2
int joint_calibration(int id, double angle)
{
    return FALSE;
}
#endif

#ifndef IOB2
int read_gyro_sensor_offset(int id, double *offset)
#else
int iob::read_gyro_sensor_offset(int id, double *offset)
#endif
{
    for (int i=0; i<3; i++){
        offset[i] = gyro_offset[id][i];
    }
    return TRUE;
}

#ifndef IOB2
int write_gyro_sensor_offset(int id, double *offset)
#else
int iob::write_gyro_sensor_offset(int id, double *offset)
#endif
{
    for (int i=0; i<3; i++){
        gyro_offset[id][i] = offset[i];
    }
    return TRUE;
}

#ifndef IOB2
int read_accelerometer_offset(int id, double *offset)
#else
int iob::read_accelerometer_offset(int id, double *offset)
#endif
{
    for (int i=0; i<3; i++){
        offset[i] = accel_offset[id][i];
    }
    return TRUE;
}

#ifndef IOB2
int write_accelerometer_offset(int id, double *offset)
#else
int iob::write_accelerometer_offset(int id, double *offset)
#endif
{
    for (int i=0; i<3; i++){
        accel_offset[id][i] = offset[i];
    }
    return TRUE;
}

#ifndef IOB2
int read_force_offset(int id, double *offsets)
#else
int iob::read_force_offset(int id, double *offsets)
#endif
{
    for (int i=0; i<6; i++){
        offsets[i] = force_offset[id][i];
    }
    return TRUE;
}

#ifndef IOB2
int write_force_offset(int id, double *offsets)
#else
int iob::write_force_offset(int id, double *offsets)
#endif
{
    for (int i=0; i<6; i++){
        force_offset[id][i] = offsets[i];
    }
    return TRUE;
}

#ifndef IOB2
int write_attitude_sensor_offset(int id, double *offset)
#else
int iob::write_attitude_sensor_offset(int id, double *offset)
#endif
{
    return FALSE;
}

#ifndef IOB2
int read_calib_state(int id, int *s)
#else
int iob::read_calib_state(int id, int *s)
#endif
{
    CHECK_JOINT_ID(id);
    int v = id/2;
    *s = v%2==0 ? ON : OFF;
    return TRUE;
}

#ifndef IOB2
int lock_iob()
#else
int iob::lock_iob()
#endif
{
    if (isLocked) return FALSE;

    //isLocked = true;
    return TRUE;
}

#ifndef IOB2
int unlock_iob()
#else
int iob::unlock_iob()
#endif
{
    isLocked = false;
    return TRUE;
}

#ifndef IOB2
int read_lock_owner(pid_t *pid)
#else
int iob::read_lock_owner(pid_t *pid)
#endif
{
  return FALSE;
}
#ifndef IOB2
int read_limit_angle(int id, double *angle)
{
  return FALSE;
}
#endif
#ifndef IOB2
int read_angle_offset(int id, double *angle)
#else
int iob::read_angle_offset(int id, double *angle)
#endif
{
  return FALSE;
}
#ifndef IOB2
int write_angle_offset(int id, double angle)
#else
int iob::write_angle_offset(int id, double angle)  
#endif
{
  return FALSE;
}
#ifndef IOB2
int read_ulimit_angle(int id, double *angle)
{
  return FALSE;
}
#endif
#ifndef IOB2
int read_llimit_angle(int id, double *angle)
{
  return FALSE;
}
#endif
#ifndef IOB2
int read_encoder_pulse(int id, double *ec)
{
  return FALSE;
}
#endif
#ifndef IOB2
int read_gear_ratio(int id, double *gr)
{
  return FALSE;
}
#endif
#ifndef IOB2
int read_torque_const(int id, double *tc)
{
  return FALSE;
}
#endif
#ifndef IOB2
int read_torque_limit(int id, double *limit)
{
  return FALSE;
}
#endif

#ifndef IOB2
unsigned long long read_iob_frame()
#else
unsigned long long iob::read_iob_frame()
#endif
{
    ++frame;
    if (iob_synchronized && start_robothw) {
      if(frame % num_of_substeps != 0) {
        tick_service_command();
      } // else break to executing RobotHardware
    }
    return frame;
}

#ifndef IOB2
int number_of_substeps()
#else
int iob::number_of_substeps()
#endif
{
  if (iob_synchronized) {
    return num_of_substeps;
  } else {
    return 1;
  }
}

#ifndef IOB2
int read_power(double *voltage, double *current)
#else
int iob::read_power(double *voltage, double *current)
#endif
{
    *voltage = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*1+48;
    *current = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*0.5+1;
    return TRUE;
}

#ifndef IOB2
int read_driver_temperature(int id, unsigned char *v)
#else
int iob::read_driver_temperature(int id, unsigned char *v)
#endif
{
    *v = id * 2;
    return TRUE;
}

void timespec_add_ns(timespec *ts, long ns)
{
    ts->tv_nsec += ns;
    while (ts->tv_nsec > 1e9){
        ts->tv_sec += 1;
        ts->tv_nsec -= 1e9;
    }
}

double timespec_compare(timespec *ts1, timespec *ts2)
{
    double dts = ts1->tv_sec - ts2->tv_sec;
    double dtn = ts1->tv_nsec - ts2->tv_nsec;
    return dts*1e9+dtn;
}

#ifndef IOB2
int wait_for_iob_signal()
#else
int iob::wait_for_iob_signal()
#endif
{
  if (iob_synchronized) {
    //std::cerr << "wait>" << std::endl;
    if (start_robothw) {
      // no wait
    }
    //std::cerr << "wait<" << std::endl;
    return 0;
  } else {
    //
    ros::Time rnow;
    ros::Duration tm = ros::Duration(0, overwrite_g_period_ns);
    ros::WallDuration wtm = ros::WallDuration(0, 100000); // 0.1 ms
    while ((rnow = ros::Time::now()) < rg_ts) {
      wtm.sleep();
    }

    if ((rg_ts - rnow).toSec() < 0) {
      if((rg_ts + tm).toSec() - rnow.toSec() < 0) {
        fprintf(stderr, "iob::critical overrun (%f[ms]), w:%f -> %f\n",
                (rnow - rg_ts).toSec()*1000, rnow.toSec(), rg_ts.toSec());
      }
      do {
        rg_ts += tm;
      } while ((rg_ts - rnow).toSec() <= 0);
    } else {
      rg_ts += tm;
    }
    // fprintf(stderr, "iob:: %f\n", rg_ts.toSec()); // debug

    return 0;
  }

  return 0;
}

#ifndef IOB2
size_t length_of_extra_servo_state(int id)
#else
size_t iob::length_of_extra_servo_state(int id)
#endif
{
    return 0;
}

#ifndef IOB2
int read_extra_servo_state(int id, int *state)
#else
int iob::read_extra_servo_state(int id, int *state)
#endif
{
    return TRUE;
}

#ifndef IOB2
int set_signal_period(long period_ns)
#else
int iob::set_signal_period(long period_ns)
#endif
{
    g_period_ns = period_ns;
    if(overwrite_g_period_ns < 0) {
      overwrite_g_period_ns = g_period_ns;
    }
    return TRUE;
}

#ifndef IOB2
long get_signal_period()
#else
long iob::get_signal_period()
#endif
{
    return g_period_ns;
}

#ifndef IOB2
int initializeJointAngle(const char *name, const char *option)
#else
int iob::initializeJointAngle(const char *name, const char *option)
#endif
{
    sleep(3);
    return TRUE;
}

#ifndef IOB2
int read_digital_input(char *dinput)
#else
int iob::read_digital_input(char *dinput)
#endif
{
  return 0;
}

#ifndef IOB2
int write_digital_output(const char *doutput)
#else
int iob::write_digital_output(const char *doutput)
#endif
{
  return 0;
}

#ifndef IOB2
int length_digital_input(void)
#else
int iob::length_digital_input(void)
#endif
{
  return 0;
}

#ifndef IOB2
int write_digital_output_with_mask(const char *doutput, const char *mask)
#else
int iob::write_digital_output_with_mask(const char *doutput, const char *mask)
#endif
{
  return FALSE;
}

#ifndef IOB2
int length_digital_output(void)
#else
int iob::length_digital_output(void)
#endif
{
  return 0;
}

#ifndef IOB2
int read_digital_output(char *doutput)
#else
int iob::read_digital_output(char *doutput)
#endif
{
  return 0;
}

