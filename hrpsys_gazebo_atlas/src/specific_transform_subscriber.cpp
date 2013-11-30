#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <hrpsys_gazebo_atlas/specific_transform_subscriber.h>

void transformCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
  static tf::TransformBroadcaster tfb_;
  tf::StampedTransform stf;
  tf::transformStampedMsgToTF(*msg, stf);
  tfb_.sendTransform(stf);
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "specific_transform_subscriber");

  ros::NodeHandle n;
  ros::NodeHandle pnh_("~");
  //tf::TransformListener tfl_;

  ros::Subscriber sub_ =  pnh_.subscribe<geometry_msgs::TransformStamped> ("/specific_transform", 1, transformCallback);
  /*
  std::string parent_frame;
  pnh_.param("parent_frame", parent_frame, std::string ("") );

  std::string child_frame;
  pnh_.param("child_frame", child_frame, std::string ("") );
  */
  //  pnh_.param("loop_hz", loop_hz, 1.0 );

  /*  ROS_INFO_STREAM("parent_frame:" << parent_frame);
  ROS_INFO_STREAM("child_frame:" << child_frame);
  ROS_INFO_STREAM("loop_hz:" << loop_hz);*/


  ros::spin();
}
