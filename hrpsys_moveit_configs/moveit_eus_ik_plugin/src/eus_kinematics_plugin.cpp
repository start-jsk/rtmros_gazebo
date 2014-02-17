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
#include <moveit_msgs/GetPositionIK.h>
#include <boost/shared_ptr.hpp>


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

    boost::shared_ptr<ros::ServiceClient> ik_service_client;

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

    int ComputeIkEus(const geometry_msgs::Pose ik_pose, std::vector<double> &result_angle) const;
    void ComputeFkEus(void) const;

  }; // end class

  bool EusKinematicsPlugin::initialize(const std::string &robot_description,
				       const std::string& group_name,
				       const std::string& base_name,
				       const std::string& tip_name,
				       double search_discretization)
  {
    setValues(robot_description, group_name, base_name, tip_name, search_discretization);

    ros::NodeHandle node_handle(group_name);

    ik_service_client = boost::make_shared<ros::ServiceClient>(node_handle.serviceClient<moveit_msgs::GetPositionIK>("eus_ik_request"));
    ROS_INFO_STREAM("service client started. service name: " << ik_service_client->getService());
      
    std::string robot;
    node_handle.param("robot",robot,std::string());

    urdf::Model robot_model;
    std::string xml_string;

    std::string urdf_xml,full_urdf_xml;
    node_handle.param("urdf_xml",urdf_xml,robot_description);
    node_handle.searchParam(urdf_xml,full_urdf_xml);

    ROS_DEBUG_STREAM("Reading xml file from parameter server");
    if (!node_handle.getParam(full_urdf_xml, xml_string))
      {
	ROS_FATAL_STREAM("Could not load the xml from parameter server " << urdf_xml.c_str());
	return false;
      }

    node_handle.param(full_urdf_xml,xml_string,std::string());
    robot_model.initString(xml_string);

    ROS_DEBUG_STREAM("Reading joints and links from URDF");

    boost::shared_ptr<urdf::Link> link = boost::const_pointer_cast<urdf::Link>(robot_model.getLink(tip_frame_));
    while(link->name != base_frame_)
      {
	ROS_DEBUG_STREAM("Link " << link->name.c_str());
	link_names_.push_back(link->name);
	boost::shared_ptr<urdf::Joint> joint = link->parent_joint;
	if(joint)
	  {
	    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
	      {
		ROS_DEBUG_STREAM("Adding joint " << joint->name );

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
	    ROS_WARN_STREAM("no joint corresponding to " << link->name.c_str());
	  }
	link = link->getParent();
      }

    num_joints_ = joint_names_.size();

    if(joint_names_.size() != num_joints_)
      {
	ROS_FATAL_STREAM("Joint numbers mismatch: URDF has " << joint_names_.size() << " and EusIK has " << num_joints_);
	return false;
      }

    std::reverse(link_names_.begin(),link_names_.end());
    std::reverse(joint_names_.begin(),joint_names_.end());
    std::reverse(joint_min_vector_.begin(),joint_min_vector_.end());
    std::reverse(joint_max_vector_.begin(),joint_max_vector_.end());
    std::reverse(joint_has_limits_vector_.begin(), joint_has_limits_vector_.end());

    ROS_INFO_STREAM("eus_kinematics_plugin was called for plannning group " << getGroupName());
    for(size_t i=0; i <num_joints_; ++i)
      ROS_INFO_STREAM(joint_names_[i] << " " << joint_min_vector_[i] << " " << joint_max_vector_[i] << " " << joint_has_limits_vector_[i]);

    active_ = true;
    return true;
  }

  bool EusKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
					  const std::vector<double> &ik_seed_state,
					  std::vector<double> &solution,
					  moveit_msgs::MoveItErrorCodes &error_code,
					  const kinematics::KinematicsQueryOptions &options) const
  {
    ROS_DEBUG_STREAM("getPositionIK");

    if(!active_)
      {
	ROS_ERROR_STREAM("Kinematics not active");
	error_code.val = error_code.NO_IK_SOLUTION;
	return false;
      }

    error_code.val = ComputeIkEus(ik_pose, solution);

    ROS_INFO_STREAM("ComputeIKEus() was called.");

    if(error_code.val != 1)
      {
	ROS_DEBUG_STREAM("No IK solution");
	return false;
      }
	
    // All elements of solution obey limits
    // for(int i = 0; i < int(num_joints_); i++)
    //   {
    // 	solution.push_back(sol[i]);
    //   }
    //ROS_DEBUG_STREAM("Solution: " << *sol);

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
      ROS_WARN_STREAM("Link names with nothing");
      return false;
    }

    if(link_names.size()!=1 || link_names[0]!=tip_frame_){
      ROS_ERROR_STREAM("Can compute FK for " << tip_frame_.c_str() << " only");
      return false;
    }

    bool valid = true;

    double eerot[9],eetrans[3];

    ComputeFkEus();
    ROS_DEBUG_STREAM("ComputeFKEus() was called. angle: " << angle << " trans: " << eetrans << " rot: " << eerot);

    for(int i=0; i<3;++i)
      p_out.p.data[i] = eetrans[i];

    for(int i=0; i<9;++i)
      p_out.M.data[i] = eerot[i];

    poses.resize(1);
    tf::poseKDLToMsg(p_out,poses[0]);

    return valid;
  }

  int EusKinematicsPlugin::ComputeIkEus(const geometry_msgs::Pose ik_pose, std::vector<double> &result_angle) const
  {
    moveit_msgs::GetPositionIK ik_srv;
    geometry_msgs::PoseStamped ik_pose_st;

    ik_pose_st.pose = ik_pose;

    ik_srv.request.ik_request.group_name = getGroupName();
    ik_srv.request.ik_request.robot_state.joint_state.name = joint_names_;
    ik_srv.request.ik_request.pose_stamped = ik_pose_st;

    ROS_INFO_STREAM("service was called. service name: " << ik_service_client->getService() << "ik_request:");
    ROS_INFO_STREAM(ik_srv.request.ik_request);
    if (ik_service_client->call(ik_srv))
      {
	result_angle = ik_srv.response.solution.joint_state.position;
	ROS_INFO_STREAM("service succeeded. solution:");
	ROS_INFO_STREAM(ik_srv.response.solution);
      }
    else
      {
	ROS_ERROR_STREAM("service failed. service name: " << ik_service_client->getService());
      }

    return ik_srv.response.error_code.val;
  }

  void EusKinematicsPlugin::ComputeFkEus(void) const
  {
    return;
  }

} // namespace

//register EusKinematicsPlugin as a KinematicsBase implementation
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(eus_kinematics::EusKinematicsPlugin, kinematics::KinematicsBase);
