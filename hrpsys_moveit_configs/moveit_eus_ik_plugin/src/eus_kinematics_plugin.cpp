/*!
  @brief  MoveIt Plugin for Euslisp Inverse Kinematics
  @author Masaki Murooka
  @date   2014/02/02
*/

// #include <moveit/pr2_arm_kinematics/pr2_arm_kinematics_plugin.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include <ros/ros.h>
#include <algorithm>
#include <numeric>
#include <moveit/kinematics_base/kinematics_base.h>
#include <urdf/model.h>


#define ARRAY_LENGTH(array) (int(sizeof(array) / sizeof(array[0])))

using namespace KDL;
using namespace tf;
using namespace std;
using namespace ros;

namespace eus_kinematics {

  class EusKinematicsPlugin : public kinematics::KinematicsBase
  {
    std::vector<std::string> joint_names_;
    std::vector<double> joint_min_vector_;
    std::vector<double> joint_max_vector_;
    std::vector<bool> joint_has_limits_vector_;
    std::vector<std::string> link_names_;
    size_t num_joints_;
    bool active_; // Internal variable that indicates whether solvers are configured and ready

    const std::vector<std::string>& getJointNames() const { return joint_names_; }
    const std::vector<std::string>& getLinkNames() const { return link_names_; }

  public:

    /** @class
     *  @brief Interface for an Eus kinematics plugin
     */
    EusKinematicsPlugin():active_(false){}

    /**
     * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @param solution the solution vector
     * @param error_code an error code that encodes the reason for failure or success
     * @return True if a valid solution was found, false otherwise
     */

    // Returns the first IK solution that is within joint limits, this is called by get_ik() service
    bool getPositionIK(const geometry_msgs::Pose &ik_pose,
		       const std::vector<double> &ik_seed_state,
		       std::vector<double> &solution,
		       moveit_msgs::MoveItErrorCodes &error_code,
		       const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    /**
     * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
     * This particular method is intended for "searching" for a solutions by stepping through the redundancy
     * (or other numerical routines).
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @return True if a valid solution was found, false otherwise
     */
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
			  const std::vector<double> &ik_seed_state,
			  double timeout,
			  std::vector<double> &solution,
			  moveit_msgs::MoveItErrorCodes &error_code,
			  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    /**
     * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
     * This particular method is intended for "searching" for a solutions by stepping through the redundancy
     * (or other numerical routines).
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @param the distance that the redundancy can be from the current position
     * @return True if a valid solution was found, false otherwise
     */
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
			  const std::vector<double> &ik_seed_state,
			  double timeout,
			  const std::vector<double> &consistency_limits,
			  std::vector<double> &solution,
			  moveit_msgs::MoveItErrorCodes &error_code,
			  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    /**
     * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
     * This particular method is intended for "searching" for a solutions by stepping through the redundancy
     * (or other numerical routines).
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @return True if a valid solution was found, false otherwise
     */
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
			  const std::vector<double> &ik_seed_state,
			  double timeout,
			  std::vector<double> &solution,
			  const IKCallbackFn &solution_callback,
			  moveit_msgs::MoveItErrorCodes &error_code,
			  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    /**
     * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
     * This particular method is intended for "searching" for a solutions by stepping through the redundancy
     * (or other numerical routines).  The consistency_limit specifies that only certain redundancy positions
     * around those specified in the seed state are admissible and need to be searched.
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @param consistency_limit the distance that the redundancy can be from the current position
     * @return True if a valid solution was found, false otherwise
     */
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
			  const std::vector<double> &ik_seed_state,
			  double timeout,
			  const std::vector<double> &consistency_limits,
			  std::vector<double> &solution,
			  const IKCallbackFn &solution_callback,
			  moveit_msgs::MoveItErrorCodes &error_code,
			  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    /**
     * @brief Given a set of joint angles and a set of links, compute their pose
     *
     * This FK routine is only used if 'use_plugin_fk' is set in the 'arm_kinematics_constraint_aware' node,
     * otherwise ROS TF is used to calculate the forward kinematics
     *
     * @param link_names A set of links for which FK needs to be computed
     * @param joint_angles The state for which FK is being computed
     * @param poses The resultant set of poses (in the frame returned by getBaseFrame())
     * @return True if a valid solution was found, false otherwise
     */
    bool getPositionFK(const std::vector<std::string> &link_names,
		       const std::vector<double> &joint_angles,
		       std::vector<geometry_msgs::Pose> &poses) const;

  private:

    bool initialize(const std::string &robot_description,
		    const std::string& group_name,
		    const std::string& base_name,
		    const std::string& tip_name,
		    double search_discretization);

    bool ComputeIkEus(double *trans, double *vals, double *sol) const;
    void ComputeFkEus(double *angles, double *eetrans, double *eerot) const;

  }; // end class

  bool EusKinematicsPlugin::initialize(const std::string &robot_description,
				       const std::string& group_name,
				       const std::string& base_name,
				       const std::string& tip_name,
				       double search_discretization)
  {
    setValues(robot_description, group_name, base_name, tip_name, search_discretization);

    ros::NodeHandle node_handle("~/"+group_name);

    std::string robot;
    node_handle.param("robot",robot,std::string());

    urdf::Model robot_model;
    std::string xml_string;

    std::string urdf_xml,full_urdf_xml;
    node_handle.param("urdf_xml",urdf_xml,robot_description);
    node_handle.searchParam(urdf_xml,full_urdf_xml);

    ROS_DEBUG_NAMED("eusik","Reading xml file from parameter server");
    if (!node_handle.getParam(full_urdf_xml, xml_string))
      {
	ROS_FATAL_NAMED("eusik","Could not load the xml from parameter server: %s", urdf_xml.c_str());
	return false;
      }

    node_handle.param(full_urdf_xml,xml_string,std::string());
    robot_model.initString(xml_string);

    ROS_DEBUG_STREAM_NAMED("eusik","Reading joints and links from URDF");

    boost::shared_ptr<urdf::Link> link = boost::const_pointer_cast<urdf::Link>(robot_model.getLink(tip_frame_));
    while(link->name != base_frame_)
      {
	ROS_DEBUG_NAMED("eusik","Link %s",link->name.c_str());
	link_names_.push_back(link->name);
	boost::shared_ptr<urdf::Joint> joint = link->parent_joint;
	if(joint)
	  {
	    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
	      {
		ROS_DEBUG_STREAM_NAMED("eusik","Adding joint " << joint->name );

		joint_names_.push_back(joint->name);
		float lower, upper;
		int hasLimits;
		if ( joint->type != urdf::Joint::CONTINUOUS )
		  {
		    if(joint->safety)
		      {
			lower = joint->safety->soft_lower_limit;
			upper = joint->safety->soft_upper_limit;
		      } else {
		      lower = joint->limits->lower;
		      upper = joint->limits->upper;
		    }
		    hasLimits = 1;
		  }
		else
		  {
		    lower = -M_PI;
		    upper = M_PI;
		    hasLimits = 0;
		  }
		if(hasLimits)
		  {
		    joint_has_limits_vector_.push_back(true);
		    joint_min_vector_.push_back(lower);
		    joint_max_vector_.push_back(upper);
		  }
		else
		  {
		    joint_has_limits_vector_.push_back(false);
		    joint_min_vector_.push_back(-M_PI);
		    joint_max_vector_.push_back(M_PI);
		  }
	      }
	  } else
	  {
	    ROS_WARN_NAMED("eusik","no joint corresponding to %s",link->name.c_str());
	  }
	link = link->getParent();
      }

    num_joints_ = joint_names_.size();

    if(joint_names_.size() != num_joints_)
      {
	ROS_FATAL_STREAM_NAMED("eusik","Joint numbers mismatch: URDF has " << joint_names_.size() << " and EusIK has " << num_joints_);
	return false;
      }

    std::reverse(link_names_.begin(),link_names_.end());
    std::reverse(joint_names_.begin(),joint_names_.end());
    std::reverse(joint_min_vector_.begin(),joint_min_vector_.end());
    std::reverse(joint_max_vector_.begin(),joint_max_vector_.end());
    std::reverse(joint_has_limits_vector_.begin(), joint_has_limits_vector_.end());

    for(size_t i=0; i <num_joints_; ++i)
      ROS_INFO_STREAM_NAMED("eusik",joint_names_[i] << " " << joint_min_vector_[i] << " " << joint_max_vector_[i] << " " << joint_has_limits_vector_[i]);

    active_ = true;
    return true;
  }

  bool EusKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
					  const std::vector<double> &ik_seed_state,
					  std::vector<double> &solution,
					  moveit_msgs::MoveItErrorCodes &error_code,
					  const kinematics::KinematicsQueryOptions &options) const
  {
    ROS_DEBUG_STREAM_NAMED("eusik","getPositionIK");

    if(!active_)
      {
	ROS_ERROR_STREAM_NAMED("eusik","Kinematics not active");
	error_code.val = error_code.NO_IK_SOLUTION;
	return false;
      }

    double *sol = new double[num_joints_];

    KDL::Frame frame;
    tf::poseMsgToKDL(ik_pose,frame);

    double trans[3];
    double vals[9];
    KDL::Rotation mult;

    trans[0] = frame.p[0];
    trans[1] = frame.p[1];
    trans[2] = frame.p[2];

    mult = frame.M;

    vals[0] = mult(0,0);
    vals[1] = mult(0,1);
    vals[2] = mult(0,2);
    vals[3] = mult(1,0);
    vals[4] = mult(1,1);
    vals[5] = mult(1,2);
    vals[6] = mult(2,0);
    vals[7] = mult(2,1);
    vals[8] = mult(2,2);
    
    const bool solved_flag = ComputeIkEus(trans, vals, sol);

    ROS_INFO_STREAM_NAMED("eusik","ComputeIKEus() was called.");
    std::string tmp_string("    frame.p:");
    for(int i = 0; i < ARRAY_LENGTH(trans); i++){
      char ts[10];
      sprintf(ts, " %5.3f", trans[i]);
      tmp_string += string(ts);
    }
    tmp_string += string("   frame.M:");
    for(int i = 0; i < ARRAY_LENGTH(vals); i++){
      char ts[10];
      sprintf(ts, " %5.3f", vals[i]);
      tmp_string += string(ts);
    }
    ROS_INFO_STREAM_NAMED("eusik",tmp_string.c_str());
    tmp_string = string("");
    for(int i = 0; i < int(num_joints_); i++){
      char ts[10];
      sprintf(ts, " %5.3f", sol[i]);
      tmp_string += string(ts);
    }
    ROS_INFO_STREAM_NAMED("eusik","    Solution: " << tmp_string);



    if(!solved_flag)
      {
	ROS_DEBUG_STREAM_NAMED("eusik","No IK solution");
	error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
	delete [] sol;
	return false;
      }
	
    // All elements of solution obey limits
    for(int i = 0; i < int(num_joints_); i++)
      {
	solution.push_back(sol[i]);
      }
    //ROS_DEBUG_NAMED("eusik","Solution: " << *sol);

    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    delete [] sol;
    return true;
  }

  bool EusKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
					     const std::vector<double> &ik_seed_state,
					     double timeout,
					     std::vector<double> &solution,
					     moveit_msgs::MoveItErrorCodes &error_code,
					     const kinematics::KinematicsQueryOptions &options) const
  {
    static IKCallbackFn solution_callback = 0;
    static std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code);
  }

  bool EusKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
					     const std::vector<double> &ik_seed_state,
					     double timeout,
					     const std::vector<double> &consistency_limits,
					     std::vector<double> &solution,
					     moveit_msgs::MoveItErrorCodes &error_code,
					     const kinematics::KinematicsQueryOptions &options) const
  {
    static IKCallbackFn solution_callback = 0;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code);
  }

  bool EusKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
					     const std::vector<double> &ik_seed_state,
					     double timeout,
					     std::vector<double> &solution,
					     const IKCallbackFn &solution_callback,
					     moveit_msgs::MoveItErrorCodes &error_code,
					     const kinematics::KinematicsQueryOptions &options) const
  {
    static std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code);
  }

  bool EusKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
					     const std::vector<double> &ik_seed_state,
					     double timeout,
					     const std::vector<double> &consistency_limits,
					     std::vector<double> &solution,
					     const IKCallbackFn &solution_callback,
					     moveit_msgs::MoveItErrorCodes &error_code,
					     const kinematics::KinematicsQueryOptions &options) const
  {
    return EusKinematicsPlugin::getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
  }

  bool EusKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
					     const std::vector<double> &joint_angles,
					     std::vector<geometry_msgs::Pose> &poses) const
  {
    KDL::Frame p_out;
    if(link_names.size() == 0) {
      ROS_WARN_STREAM_NAMED("eusik","Link names with nothing");
      return false;
    }

    if(link_names.size()!=1 || link_names[0]!=tip_frame_){
      ROS_ERROR_NAMED("eusik","Can compute FK for %s only",tip_frame_.c_str());
      return false;
    }

    bool valid = true;

    double eerot[9],eetrans[3];
    double angles[joint_angles.size()];
    for (unsigned char i=0; i < joint_angles.size(); i++)
      angles[i] = joint_angles[i];

    ComputeFkEus(angles, eetrans, eerot);
    ROS_DEBUG_STREAM_NAMED("eusik","ComputeFKEus() was called. angle: " << angle << " trans: " << eetrans << " rot: " << eerot);

    for(int i=0; i<3;++i)
      p_out.p.data[i] = eetrans[i];

    for(int i=0; i<9;++i)
      p_out.M.data[i] = eerot[i];

    poses.resize(1);
    tf::poseKDLToMsg(p_out,poses[0]);

    return valid;
  }

  bool EusKinematicsPlugin::ComputeIkEus(double *trans, double *vals, double *sol) const
  {
    for(int i = 0; i < int(num_joints_); i++) {
      sol[i] = 1;
    }
    return true;
  }

  void EusKinematicsPlugin::ComputeFkEus(double *angles, double *eetrans, double *eerot) const
  {
    return;
  }

} // namespace

//register EusKinematicsPlugin as a KinematicsBase implementation
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(eus_kinematics::EusKinematicsPlugin, kinematics::KinematicsBase);
