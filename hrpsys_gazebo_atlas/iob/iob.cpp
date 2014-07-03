#include <unistd.h>
#include <iostream>
#include <string>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "io/iob.h"

#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <osrf_msgs/JointCommands.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <atlas_msgs/AtlasState.h>
#include <atlas_msgs/AtlasCommand.h>
#include <hrpUtil/Eigen3d.h>

static ros::NodeHandle* rosnode;
static ros::Publisher pub_joint_commands_;
static ros::Subscriber sub_atlas_state;
static atlas_msgs::AtlasCommand jointcommands;
static atlas_msgs::AtlasCommand initial_jointcommands;
//static osrf_msgs::JointCommands jointcommands;
static atlas_msgs::AtlasState js;
static int init_sub_flag = FALSE;

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
static int frame = 0;
static timespec g_ts;
static long g_period_ns=3000000;
static ros::Time rg_ts;
static ros::Time last_callback_time;

#define CHECK_JOINT_ID(id) if ((id) < 0 || (id) >= number_of_joints()) return E_ID
#define CHECK_FORCE_SENSOR_ID(id) if ((id) < 0 || (id) >= number_of_force_sensors()) return E_ID
#define CHECK_ACCELEROMETER_ID(id) if ((id) < 0 || (id) >= number_of_accelerometers()) return E_ID
#define CHECK_GYRO_SENSOR_ID(id) if ((id) < 0 || (id) >= number_of_gyro_sensors()) return E_ID
#define CHECK_ATTITUDE_SENSOR_ID(id) if ((id) < 0 || (id) >= number_of_attitude_sensors()) return E_ID

#if 0
// atlas_v0
Collada Robot Model (MODEL)
// This order comes from order of <attachment_full joint joint="***"> tag in hrpsys_gazebo_atlas/models/atlas.dae
 [0] back_lbz
 [1] back_mby
 [2] back_ubx
 [3] l_arm_usy
 [4] l_arm_shx
 [5] l_arm_ely
 [6] l_arm_elx
 [7] l_arm_uwy
 [8] l_arm_mwx
 [9] neck_ay
[10] center_bottom_led_frame_joint
[11] center_top_led_frame_joint
[12] head_imu_joint
[13] hokuyo_joint
[14] head_hokuyo_joint
[15] left_camera_frame_joint
[16] left_camera_optical_frame_joint
[17] left_led_frame_joint
[18] right_camera_frame_joint
[19] right_camera_optical_frame_joint
[20] right_led_frame_joint
[21] r_arm_usy
[22] r_arm_shx
[23] r_arm_ely
[24] r_arm_elx
[25] r_arm_uwy
[26] r_arm_mwx
[27] imu_joint
[28] l_leg_uhz
[29] l_leg_mhx
[30] l_leg_lhy
[31] l_leg_kny
[32] l_leg_uay
[33] l_leg_lax
[34] r_leg_uhz
[35] r_leg_mhx
[36] r_leg_lhy
[37] r_leg_kny
[38] r_leg_uay
[39] r_leg_lax
gazebo robot model (REAL)
//This order comes from ros topic atlas/joint_states/name
'back_lbz'
'back_mby'
'back_ubx'
'neck_ay'
'l_leg_uhz'
'l_leg_mhx'
'l_leg_lhy'
'l_leg_kny'
'l_leg_uay'
'l_leg_lax'
'r_leg_uhz'
'r_leg_mhx'
'r_leg_lhy'
'r_leg_kny'
'r_leg_uay'
'r_leg_lax'
'l_arm_usy'
'l_arm_shx'
'l_arm_ely'
'l_arm_elx'
'l_arm_uwy'
'l_arm_mwx'
'r_arm_usy'
'r_arm_shx'
'r_arm_ely'
'r_arm_elx'
'r_arm_uwy'
'r_arm_mwx'

// atals_v3
Collada Robot Model (MODEL)
// This order comes from order of <attachment_full joint joint="***"> tag in hrpsys_gazebo_atlas/models/atlas_v3.dae
 [0] back_bkz
 [1] back_bky
 [2] back_bkx
 [3] l_arm_shy
 [4] l_arm_shx
 [5] l_arm_ely
 [6] l_arm_elx
 [7] l_arm_wry
 [8] l_arm_wrx
 [9] l_situational_awareness_camera_joint
[10] l_situational_awareness_camera_optical_frame_joint
[11] neck_ry
[12] center_bottom_led_frame_joint
[13] center_top_led_frame_joint
[14] head_imu_joint
[15] hokuyo_joint
[16] head_hokuyo_joint
[17] left_camera_frame_joint
[18] left_camera_optical_frame_joint
[19] left_led_frame_joint
[20] right_camera_frame_joint
[21] right_camera_optical_frame_joint
[22] right_led_frame_joint
[23] r_arm_shy
[24] r_arm_shx
[25] r_arm_ely
[26] r_arm_elx
[27] r_arm_wry
[28] r_arm_wrx
[29] r_situational_awareness_camera_joint
[30] r_situational_awareness_camera_optical_frame_joint
[31] rear_situational_awareness_camera_joint
[32] rear_situational_awareness_camera_optical_frame_joint
[33] imu_joint
[34] l_leg_hpz
[35] l_leg_hpx
[36] l_leg_hpy
[37] l_leg_kny
[38] l_leg_aky
[39] l_leg_akx
[40] r_leg_hpz
[41] r_leg_hpx
[42] r_leg_hpy
[43] r_leg_kny
[44] r_leg_aky
[45] r_leg_akx

gazebo robot model (REAL)
//This order comes from ros topic atlas/joint_states/name
'back_bkz'
'back_bky'
'back_bkx'
'neck_ry'
'l_leg_hpz'
'l_leg_hpx'
'l_leg_hpy'
'l_leg_kny'
'l_leg_aky'
'l_leg_akx'
'r_leg_hpz'
'r_leg_hpx'
'r_leg_hpy'
'r_leg_kny'
'r_leg_aky'
'r_leg_akx'
'l_arm_shy'
'l_arm_shx'
'l_arm_ely'
'l_arm_elx'
'l_arm_wry'
'l_arm_wrx'
'r_arm_shy'
'r_arm_shx'
'r_arm_ely'
'r_arm_elx'
'r_arm_wry' 
'r_arm_wrx'
#endif

#define JOINT_ID_REAL2MODEL(id) get_joint_id_real2model(id)
#define JOINT_ID_MODEL2REAL(id) get_joint_id_model2real(id)
#define NUM_OF_REAL_JOINT get_num_of_real_joint()
/* for atlas_v0 (old model without backpack) */
static int joint_id_real2model_atlas_v0[] = {0, 1, 2, 9, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 3, 4, 5, 6, 7, 8, 21, 22, 23, 24, 25, 26};
/* for atlas_v3 */
static int joint_id_real2model_atlas_v3[] = {0, 1, 2, 11, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 3, 4, 5, 6, 7, 8, 23, 24, 25, 26, 27, 28};
/* robot idx */
enum AtlasName {ATLAS_V0, ATLAS_V3, UNKNOWN};
static enum AtlasName atlas_name = UNKNOWN;

static void update_atlas_name()
{
  std::string iob_atlas_name_value;
  rosnode->getParam("iob_atlas_name", iob_atlas_name_value);
  if (iob_atlas_name_value == "atlas_v0") {
    ROS_WARN_STREAM("atlas_name is ATLAS_V0");
    atlas_name = ATLAS_V0;
  } else if (iob_atlas_name_value == "atlas_v3") {
    atlas_name = ATLAS_V3;
    ROS_WARN_STREAM("atlas_name is ATLAS_V3");
  } else {
    ROS_ERROR_STREAM("atlas_name is INVALID");
  }
}

static int get_joint_id_real2model(int id)
{
  if (atlas_name == UNKNOWN) {
    update_atlas_name();
  }

  if (atlas_name == ATLAS_V0) {
    return joint_id_real2model_atlas_v0[id];
  } else if(atlas_name == ATLAS_V3) {
    return joint_id_real2model_atlas_v3[id];
  } else {
    return -1;
  }
}
static int get_num_of_real_joint()
{
  if (atlas_name == UNKNOWN) {
    update_atlas_name();
  }

  if (atlas_name == ATLAS_V0) {
    return sizeof(joint_id_real2model_atlas_v0)/sizeof(joint_id_real2model_atlas_v0[0]);
  } else if(atlas_name == ATLAS_V3) {
    return sizeof(joint_id_real2model_atlas_v3)/sizeof(joint_id_real2model_atlas_v3[0]);
  } else {
    return -1;
  }
}

static int get_joint_id_model2real(int id)
{
  for (int i = 0; i < NUM_OF_REAL_JOINT; i++){
    if (get_joint_id_real2model(i) == id){
      return i;
    }
  }
  return -1;
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


int number_of_joints()
{
    return (int)command.size();
}

int number_of_force_sensors()
{
    return (int)forces.size();
}

int number_of_gyro_sensors()
{
    return (int)gyros.size();
}

int number_of_accelerometers()
{
    return (int)accelerometers.size();
}

int number_of_attitude_sensors()
{
    return (int)attitude_sensors.size();
}

int set_number_of_joints(int num)
{
    command.resize(num);
    prev_command.resize(num);
    power.resize(num);
    servo.resize(num);
    for (int i=0; i<num; i++){
        command[i] = power[i] = servo[i] = prev_command[i] = 0;
    }
    return TRUE;
}

int set_number_of_force_sensors(int num)
{
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

int set_number_of_gyro_sensors(int num)
{
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

int set_number_of_accelerometers(int num)
{
    accelerometers.resize(num);
    accel_offset.resize(num);
    for (unsigned int i=0; i<accelerometers.size();i++){
        accelerometers[i].resize(3);
        accel_offset[i].resize(3);
        for (int j=0; j<3; j++){
            accelerometers[i][j] = j == 2 ? 9.81 : 0.0;
            accel_offset[i][j] = 0;
        }
    }
    return TRUE;
}

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

int read_power_state(int id, int *s)
{
    CHECK_JOINT_ID(id);
    *s = power[id];
    return TRUE;
}

int write_power_command(int id, int com)
{
    CHECK_JOINT_ID(id);
    power[id] = com;
    return TRUE;
}

int read_power_command(int id, int *com)
{
    CHECK_JOINT_ID(id);
    *com = power[id];
    return TRUE;
}

int read_servo_state(int id, int *s)
{
    CHECK_JOINT_ID(id);
    *s = servo[id];
    return TRUE;
}

int read_servo_alarm(int id, int *a)
{
    CHECK_JOINT_ID(id);
    *a = 0;
    return TRUE;
}

int read_control_mode(int id, joint_control_mode *s)
{
    CHECK_JOINT_ID(id);
    *s = JCM_POSITION;
    return TRUE;
}

int write_control_mode(int id, joint_control_mode s)
{
    CHECK_JOINT_ID(id);
    return TRUE;
}

int read_actual_angle(int id, double *angle)
{
  CHECK_JOINT_ID(id);
  //*angle = command[id]+0.01;
  //*angle = command[id];
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

int read_actual_angles(double *angles)
{
    for (int i=0; i<number_of_joints(); i++){
        read_actual_angle(i, &angles[i]);
    }
    return TRUE;
}

int read_actual_torques(double *torques)
{
  // return FALSE;

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

int read_command_torque(int id, double *torque)
{
    return FALSE;
}

int write_command_torque(int id, double torque)
{
    return FALSE;
}

int read_command_torques(double *torques)
{
    return FALSE;
}

int write_command_torques(const double *torques)
{
    return FALSE;
}

int read_command_angle(int id, double *angle)
{
    CHECK_JOINT_ID(id);
    *angle = command[id];
    return TRUE;
}

int write_command_angle(int id, double angle)
{
    CHECK_JOINT_ID(id);
    command[id] = angle;
    return TRUE;
}

int read_command_angles(double *angles)
{
    for (int i=0; i<number_of_joints(); i++){
        angles[i] = command[i];
    }
    return TRUE;
}

int write_command_angles(const double *angles)
{
    for (int i=0; i<number_of_joints(); i++){
        prev_command[i] = command[i];
        command[i] = angles[i];
    }

    atlas_msgs::AtlasCommand send_com(jointcommands);
    bool servo_on = true;

    send_com.header.stamp = ros::Time::now();

    for (int i=0; i<NUM_OF_REAL_JOINT; i++) {
      if (servo[JOINT_ID_REAL2MODEL(i)] > 0) {
        send_com.position[i] = command[JOINT_ID_REAL2MODEL(i)];
        send_com.velocity[i] = (command[JOINT_ID_REAL2MODEL(i)] - prev_command[JOINT_ID_REAL2MODEL(i)]) / (g_period_ns * 1e-9);
        if(atlas_name == ATLAS_V3) {
          if(i == 8 || i == 9 || i == 14 || i == 15) {
            send_com.kp_velocity[i]  = 0;
          } else {
            send_com.kp_velocity[i]  = 50;
          }
        } else if(atlas_name == ATLAS_V0) {
          send_com.kp_velocity[i]  = 200;
        }
      } else {
        servo_on = false;
        send_com.position[i] = 0;
        send_com.velocity[i] = 0;
        send_com.effort[i] = 0;
        send_com.kp_position[i] = 0;
        send_com.ki_position[i] = 0;
        send_com.kd_position[i] = 0;
        send_com.kp_velocity[i]  = 0;
        send_com.kp_velocity[i]  = 0;
      }
    }

    if (servo_on) {
      // ROS_INFO_STREAM("pub: " << send_com);
      pub_joint_commands_.publish(send_com);
    }

    // check if the joint_states are published
    if ( ros::Time::now()  - last_callback_time > ros::Duration(10.0) ) {
      ROS_ERROR_STREAM("/atlas/atlas_state  has not been published last 10.0 seconds, check by `rostopic info /atlas/atlas_state`");
      last_callback_time = ros::Time::now();
    }

    ros::spinOnce();

    return TRUE;
}

int read_pgain(int id, double *gain)
{
  if(JOINT_ID_MODEL2REAL(id) < 0) {
    //
    std::cerr << ";;; read gain: " << id << " failed." << std::endl;
  }else{
    int iid = JOINT_ID_MODEL2REAL(id);
    *(gain) = jointcommands.kp_position[iid] / initial_jointcommands.kp_position[iid];
    //std::cerr << ";;; read gain: " << id << " = " << *gain << std::endl;
  }
  return TRUE;
}

int write_pgain(int id, double gain)
{
  //std::cerr << ";;; write pgain: " << id << " = " << gain << std::endl;
  if(JOINT_ID_MODEL2REAL(id) < 0) {
    //
  }else{
    int iid = JOINT_ID_MODEL2REAL(id);
    jointcommands.kp_position[iid] =
      gain * initial_jointcommands.kp_position[iid];
  }
  return TRUE;
}

int read_dgain(int id, double *gain)
{
  //std::cerr << ";;; read dgain: " << id << " = " << *gain << std::endl;
  if(JOINT_ID_MODEL2REAL(id) < 0) {
    //
  }else{
    int iid = JOINT_ID_MODEL2REAL(id);
    *(gain) = jointcommands.kd_position[iid] / initial_jointcommands.kd_position[iid];
  }
  return TRUE;
}

int write_dgain(int id, double gain)
{
  //std::cerr << ";;; write dgain: " << id << " = " << gain << std::endl;
  if(JOINT_ID_MODEL2REAL(id) < 0) {
    //
  }else{
    int iid = JOINT_ID_MODEL2REAL(id);
    jointcommands.kd_position[iid] =
      gain * initial_jointcommands.kd_position[iid];
  }
  return TRUE;
}

int read_force_sensor(int id, double *forces)
{
  CHECK_FORCE_SENSOR_ID(id);
  // for (int i=0; i<6; i++){
  //     forces[i] = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*2
  //         + 2 + force_offset[id][i]; // 2 = initial offset
  // }
  std::vector<geometry_msgs::Wrench*> fsensors;
  fsensors.push_back(&(js.l_hand));
  fsensors.push_back(&(js.r_hand));
  fsensors.push_back(&(js.l_foot));
  fsensors.push_back(&(js.r_foot));
  if(init_sub_flag){
    forces[0] = fsensors[id]->force.x + force_offset[id][0];
    forces[1] = fsensors[id]->force.y + force_offset[id][1];
    forces[2] = fsensors[id]->force.z + force_offset[id][2];
    forces[3] = fsensors[id]->torque.x + force_offset[id][3];
    forces[4] = fsensors[id]->torque.y + force_offset[id][4];
    forces[5] = fsensors[id]->torque.z + force_offset[id][5];
  }

  return TRUE;
}

int read_gyro_sensor(int id, double *rates)
{
  CHECK_GYRO_SENSOR_ID(id);
  // for (int i=0; i<3; i++){
  //     rates[i] = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*0.01
  //         + 0.01 + gyro_offset[id][i]; // 0.01 = initial offset
  // }
  if(init_sub_flag){
    
    // use atlas orientation
    Eigen::Quaternion<double> q(js.orientation.w,
                                js.orientation.x,
                                js.orientation.y,
                                js.orientation.z);
    hrp::Vector3 rpy = hrp::rpyFromRot(q.toRotationMatrix());
    rates[0] = rpy[0];
    rates[1] = rpy[1];
    rates[2] = rpy[2];

    // rates[0] = js.angular_velocity.x
    //   + 0.01 + gyro_offset[id][0]; // 0.01 = initial offset
    // rates[1] = js.angular_velocity.y
    //   + 0.01 + gyro_offset[id][1]; // 0.01 = initial offset
    // rates[2] = js.angular_velocity.z
    //   + 0.01 + gyro_offset[id][2]; // 0.01 = initial offset
  } else {
    // tempolary values when sensor is not ready.
    rates[0] = rates[1] = rates[2] = 0.0;
  }

  return TRUE;
}

int read_accelerometer(int id, double *accels)
{
  CHECK_ACCELEROMETER_ID(id);
  // for (int i=0; i<3; i++){
  //     double randv = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*0.01;
  //     accels[i] = (i == 2 ? (9.8+randv) : randv)
  //         + 0.01 + accel_offset[id][i]; // 0.01 = initial offset
  // }
  if(init_sub_flag){
    accels[0] = js.linear_acceleration.x
      + 0.01 + accel_offset[id][0]; // 0.01 = initial offset
    accels[1] = js.linear_acceleration.y
      + 0.01 + accel_offset[id][1]; // 0.01 = initial offset
    accels[2] = js.linear_acceleration.z
      + 0.01 + accel_offset[id][2]; // 0.01 = initial offset
  } else {
    // tempolary values when sensor is not ready.
    accels[0] = accels[1] = 0.0;
    accels[2] = 9.8;
  }

  return TRUE;
}

int read_touch_sensors(unsigned short *onoff)
{
    return FALSE;
}

int read_attitude_sensor(int id, double *att)
{
  return FALSE;
}

int read_current(int id, double *mcurrent)
{
    return FALSE;
}

int read_current_limit(int id, double *v)
{
    return FALSE;
}

int read_currents(double *currents)
{
    return FALSE;
}

int read_gauges(double *gauges)
{
    return FALSE;
}

int read_actual_velocity(int id, double *vel)
{
    return FALSE;
}

int read_command_velocity(int id, double *vel)
{
    return FALSE;
}

int write_command_velocity(int id, double vel)
{
    return FALSE;
}

int read_actual_velocities(double *vels)
{
    return FALSE;
}

int read_command_velocities(double *vels)
{
    return FALSE;
}

int write_command_velocities(const double *vels)
{
    return FALSE;
}

int read_temperature(int id, double *v)
{
    return FALSE;
}

int write_servo(int id, int com)
{
    servo[id] = com;
    return TRUE;
}

int write_dio(unsigned short buf)
{
    return FALSE;
}

// callback
static void setJointStates(const atlas_msgs::AtlasState::ConstPtr &_js) {
  ROS_DEBUG(";; subscribe JointState");
  last_callback_time = ros::Time::now();
  js = *_js;
  init_sub_flag = TRUE;
}

int open_iob(void)
{
    static bool isInitialized = false;
    if ( isInitialized ) return TRUE;

    std::map<std::string, std::string> arg;
    ros::init(arg, "hrpsys_gazebo_atlas", ros::init_options::NoSigintHandler);
    rosnode = new ros::NodeHandle();

    // http://gazebosim.org/wiki/Tutorials/drcsim/2.2/sending_joint_controller_commands_over_ros
    // Hardcoded List of Joint Names
    // List of joint names in the Atlas robot. Note the order must not change for this function to work correctly.
    if (atlas_name == UNKNOWN) {
      update_atlas_name();
    }

    std::vector<std::string> names;
    if (atlas_name == ATLAS_V0) {
    /* for atlas_v0 (old model without backpack) */
    names.push_back("atlas::back_lbz"); // 0
    names.push_back("atlas::back_mby"); // 1
    names.push_back("atlas::back_ubx"); // 2
    names.push_back("atlas::neck_ay"); // 9
    names.push_back("atlas::l_leg_uhz"); // 28
    names.push_back("atlas::l_leg_mhx"); // 29
    names.push_back("atlas::l_leg_lhy"); // 30
    names.push_back("atlas::l_leg_kny"); // 31
    names.push_back("atlas::l_leg_uay"); // 32
    names.push_back("atlas::l_leg_lax"); // 33
    names.push_back("atlas::r_leg_uhz"); // 34
    names.push_back("atlas::r_leg_mhx"); // 35
    names.push_back("atlas::r_leg_lhy"); // 36
    names.push_back("atlas::r_leg_kny"); // 37
    names.push_back("atlas::r_leg_uay"); // 38
    names.push_back("atlas::r_leg_lax"); // 39
    names.push_back("atlas::l_arm_usy"); // 3
    names.push_back("atlas::l_arm_shx"); // 4
    names.push_back("atlas::l_arm_ely"); // 5
    names.push_back("atlas::l_arm_elx"); // 6
    names.push_back("atlas::l_arm_uwy"); // 7
    names.push_back("atlas::l_arm_mwx"); // 8
    names.push_back("atlas::r_arm_usy"); // 21
    names.push_back("atlas::r_arm_shx"); // 22
    names.push_back("atlas::r_arm_ely"); // 23
    names.push_back("atlas::r_arm_elx"); // 24
    names.push_back("atlas::r_arm_uwy"); // 25
    names.push_back("atlas::r_arm_mwx"); // 26
    } else if (atlas_name == ATLAS_V3) {
      /* for atlas_v3 */
      names.push_back("atlas::back_bkz"); // 0
      names.push_back("atlas::back_bky"); // 1
      names.push_back("atlas::back_bkx"); // 2
      names.push_back("atlas::neck_ry"); // 9
      names.push_back("atlas::l_leg_hpz"); // 28
      names.push_back("atlas::l_leg_hpx"); // 29
      names.push_back("atlas::l_leg_hpy"); // 30
      names.push_back("atlas::l_leg_kny"); // 31
      names.push_back("atlas::l_leg_aky"); // 32
      names.push_back("atlas::l_leg_akx"); // 33
      names.push_back("atlas::r_leg_hpz"); // 34
      names.push_back("atlas::r_leg_hpx"); // 35
      names.push_back("atlas::r_leg_hpy"); // 36
      names.push_back("atlas::r_leg_kny"); // 37
      names.push_back("atlas::r_leg_aky"); // 38
      names.push_back("atlas::r_leg_akx"); // 39
      names.push_back("atlas::l_arm_shy"); // 3
      names.push_back("atlas::l_arm_shx"); // 4
      names.push_back("atlas::l_arm_ely"); // 5
      names.push_back("atlas::l_arm_elx"); // 6
      names.push_back("atlas::l_arm_wry"); // 7
      names.push_back("atlas::l_arm_wrx"); // 8
      names.push_back("atlas::r_arm_shy"); // 21
      names.push_back("atlas::r_arm_shx"); // 22
      names.push_back("atlas::r_arm_ely"); // 23
      names.push_back("atlas::r_arm_elx"); // 24
      names.push_back("atlas::r_arm_wry"); // 25
      names.push_back("atlas::r_arm_wrx"); // 26
    } else {
      ROS_ERROR_STREAM("atlas_name is INVALID");
    }
    unsigned int n = names.size();

    jointcommands.position.resize(n);
    jointcommands.velocity.resize(n);
    jointcommands.effort.resize(n);
    jointcommands.kp_position.resize(n);
    jointcommands.ki_position.resize(n);
    jointcommands.kd_position.resize(n);
    jointcommands.kp_velocity.resize(n);
    jointcommands.i_effort_min.resize(n);
    jointcommands.i_effort_max.resize(n);

    for (unsigned int i = 0; i < n; i++) {
        std::vector<std::string> pieces;
        boost::split(pieces, names[i], boost::is_any_of(":"));
        double kp;
        rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/p", kp);
        jointcommands.kp_position[i] = kp;

        double ki;
        rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i", ki);
        jointcommands.ki_position[i] = ki;

        double kd;
        rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/d", kd);
        jointcommands.kd_position[i] = kd;

        double emin;
        rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", emin);
        jointcommands.i_effort_min[i] = -emin;

        double emax;
        rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", emax);
        jointcommands.i_effort_max[i] = emax;

        jointcommands.velocity[i]     = 0;
        jointcommands.effort[i]       = 0;
        // jointcommands.kp_velocity[i]  = 50;
        if(atlas_name == ATLAS_V3) {
          if(i == 8 || i == 9 || i == 14 || i == 15) {
            jointcommands.kp_velocity[i]  = 0;
          } else {
            jointcommands.kp_velocity[i]  = 50;
          }
        } else if(atlas_name == ATLAS_V0) {
          jointcommands.kp_velocity[i]  = 200;
        }
    }
    jointcommands.desired_controller_period_ms = static_cast<unsigned int>(g_period_ns * 1e-6);
    //
    char *ret = getenv("HRPSYS_GAZEBO_ATLAS_COMMAND_TOPIC_NAME");
    std::string command_topic_name;
    if (ret != NULL) {
        command_topic_name.assign(ret);
    } else {
        command_topic_name = "/atlas/atlas_command";
    }
    ROS_INFO_STREAM( "[iob] HRPSYS_GAZEBO_ATLAS_COMMAND_TOPIC_NAME : " << command_topic_name);
    // pub_joint_commands_ = rosnode->advertise<osrf_msgs::JointCommands>("/atlas/joint_commands", 1, true);
    // pub_joint_commands_ = rosnode->advertise <atlas_msgs::AtlasCommand> ("/atlas/atlas_command", 1, true);
    pub_joint_commands_ = rosnode->advertise <atlas_msgs::AtlasCommand> (command_topic_name, 1, true);

    // subscribe
    // ros topic subscribtions
    last_callback_time = ros::Time::now();
    ros::SubscribeOptions jointStatesSo =
      ros::SubscribeOptions::create<atlas_msgs::AtlasState>("/atlas/atlas_state", 1, setJointStates,
                                                            ros::VoidPtr(), rosnode->getCallbackQueue());
    // Because TCP causes bursty communication with high jitter,
    // declare a preference on UDP connections for receiving
    // joint states, which we want to get at a high rate.
    // Note that we'll still accept TCP connections for this topic
    // (e.g., from rospy nodes, which don't support UDP);
    // we just prefer UDP.
    jointStatesSo.transport_hints =
      ros::TransportHints().unreliable().reliable().tcpNoDelay(true);

    sub_atlas_state = rosnode->subscribe(jointStatesSo);


    std::cout << "JointState IOB is opened" << std::endl;
    for (int i=0; i<number_of_joints(); i++){
        command[i] = 0.0;
        power[i] = OFF;
        servo[i] = OFF;
    }
    clock_gettime(CLOCK_MONOTONIC, &g_ts);
    rg_ts = ros::Time::now();

    initial_jointcommands = jointcommands;
    isInitialized = true;
    return TRUE;
}

int close_iob(void)
{
    std::cout << "JointState IOB is closed" << std::endl;
    return TRUE;
}

int reset_body(void)
{
    for (int i=0; i<number_of_joints(); i++){
        power[i] = servo[i] = OFF;
    }
    return TRUE;
}

int joint_calibration(int id, double angle)
{
    return FALSE;
}

int read_gyro_sensor_offset(int id, double *offset)
{
    for (int i=0; i<3; i++){
        offset[i] = gyro_offset[id][i];
    }
    return TRUE;
}

int write_gyro_sensor_offset(int id, double *offset)
{
    for (int i=0; i<3; i++){
        gyro_offset[id][i] = offset[i];
    }
    return TRUE;
}

int read_accelerometer_offset(int id, double *offset)
{
    for (int i=0; i<3; i++){
        offset[i] = accel_offset[id][i];
    }
    return TRUE;
}

int write_accelerometer_offset(int id, double *offset)
{
    for (int i=0; i<3; i++){
        accel_offset[id][i] = offset[i];
    }
    return TRUE;
}

int read_force_offset(int id, double *offsets)
{
    for (int i=0; i<6; i++){
        offsets[i] = force_offset[id][i];
    }
    return TRUE;
}

int write_force_offset(int id, double *offsets)
{
    for (int i=0; i<6; i++){
        force_offset[id][i] = offsets[i];
    }
    return TRUE;
}

int write_attitude_sensor_offset(int id, double *offset)
{
    return FALSE;
}

int read_calib_state(int id, int *s)
{
    CHECK_JOINT_ID(id);
    int v = id/2;
    *s = v%2==0 ? ON : OFF;
    return TRUE;
}

int lock_iob()
{
    if (isLocked) return FALSE;

    //isLocked = true;
    return TRUE;
}
int unlock_iob()
{
    isLocked = false;
    return TRUE;
}

int read_lock_owner(pid_t *pid)
{
  return FALSE;
}

int read_limit_angle(int id, double *angle)
{
  return FALSE;
}

int read_angle_offset(int id, double *angle)
{
  return FALSE;
}

int write_angle_offset(int id, double angle)
{
  return FALSE;
}

int read_ulimit_angle(int id, double *angle)
{
  return FALSE;
}
int read_llimit_angle(int id, double *angle)
{
  return FALSE;
}
int read_encoder_pulse(int id, double *ec)
{
  return FALSE;
}
int read_gear_ratio(int id, double *gr)
{
  return FALSE;
}
int read_torque_const(int id, double *tc)
{
  return FALSE;
}
int read_torque_limit(int id, double *limit)
{
  return FALSE;
}

unsigned long long read_iob_frame()
{
    ++frame;
    //if (frame == 5) frame = 0;
    return frame;
}

int number_of_substeps()
{
    return 1;
}

int read_power(double *voltage, double *current)
{
    *voltage = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*1+48;
    *current = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*0.5+1;
    return TRUE;
}

int read_driver_temperature(int id, unsigned char *v)
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

int wait_for_iob_signal()
{
#if 0
    // use system clock
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &g_ts, 0);
    timespec_add_ns(&g_ts, g_period_ns);
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double dt = timespec_compare(&g_ts, &now);

    if (dt <= 0){
        fprintf(stderr, "iob::overrun (%d[ms])\n", -dt*1e6);
        do {
            timespec_add_ns(&g_ts, g_period_ns);
        }while(timespec_compare(&g_ts, &now)<=0);
    }
#endif
#if 1
    // use ROS Time
    //ros::Time::sleepUntil(rg_ts);
    // while (ros::Time::now() < rg_ts);

    // {
    //   ros::Duration tm = ros::Duration(0, g_period_ns);
    //   rg_ts += tm;
    // }
    // ros::Time rnow = ros::Time::now();
    // ros::Duration rdt = rg_ts - rnow;

    //ros::Time::sleepUntil(rg_ts);
    ros::Time rnow;
    ros::Duration tm = ros::Duration(0, g_period_ns);
    ros::WallDuration wtm = ros::WallDuration(0, 100000); // 0.1 ms
    while ((rnow = ros::Time::now()) < rg_ts) {
      wtm.sleep();
    }

    rg_ts += tm;
    if ((rg_ts - rnow).toSec() <= 0) {
      fprintf(stderr, "iob::overrun (%f[ms]), w:%f -> %f\n",
              (rnow - rg_ts).toSec()*1000, rnow.toSec(), rg_ts.toSec());
      do {
        rg_ts += tm;
      } while ((rg_ts - rnow).toSec() <= 0);
    }
#endif
    return 0;
}

size_t length_of_extra_servo_state(int id)
{
    return 0;
}

int read_extra_servo_state(int id, int *state)
{
    return TRUE;
}

int set_signal_period(long period_ns)
{
    g_period_ns = period_ns;
    return TRUE;
}

long get_signal_period()
{
    return g_period_ns;
}

int initializeJointAngle(const char *name, const char *option)
{
    sleep(3);
    return TRUE;
}

int read_digital_input(char *dinput)
{
  return 0;
}

int write_digital_output(const char *doutput)
{
  return 0;
}

int length_digital_input(void)
{
  return 0;
}

int write_digital_output_with_mask(const char *doutput, const char *mask)
{
  return FALSE;
}

int length_digital_output(void)
{
  return 0;
}

int read_digital_output(char *doutput)
{
  return 0;
}

