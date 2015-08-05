/*
 * LaserScanIntensityFilter.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <vector>
#include <algorithm>
#include <iterator>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace hrpsys_gazebo_general
{

class LaserScanIntensityFilter
{
public:
  ros::NodeHandle nh_;

  ros::Publisher  pub_;
  ros::Subscriber sub_;

  int subscriberCount_;
  sensor_msgs::LaserScanPtr pub_msg_;

  LaserScanIntensityFilter() {
    subscriberCount_ = 0;
    pub_msg_ = boost::shared_ptr<sensor_msgs::LaserScan>(new sensor_msgs::LaserScan());
    ros::SubscriberStatusCallback connectCallback = boost::bind(&LaserScanIntensityFilter::connectCallback, this, _1);
    ros::SubscriberStatusCallback disconnectCallback = boost::bind(&LaserScanIntensityFilter::disconnectCallback, this, _1);
    pub_ = nh_.advertise<sensor_msgs::LaserScan>("output", 1, connectCallback, disconnectCallback);
  };

  ~LaserScanIntensityFilter() {};

  void messageCallback(const sensor_msgs::LaserScanConstPtr &msg) {
    pub_msg_->header = msg->header;
    pub_msg_->angle_min = msg->angle_min;
    pub_msg_->angle_max = msg->angle_max;
    pub_msg_->angle_increment = msg->angle_increment;
    pub_msg_->time_increment = msg->time_increment;
    pub_msg_->scan_time = msg->scan_time;
    pub_msg_->range_min = msg->range_min;
    pub_msg_->range_max = msg->range_max;

    pub_msg_->ranges.clear();
    pub_msg_->ranges.reserve(msg->ranges.size());
    std::copy(msg->ranges.begin(), msg->ranges.end(), std::back_inserter(pub_msg_->ranges));

    pub_.publish(*pub_msg_);
  }

  void subscribe() {
    ROS_INFO("subscribe laserscan topic");
    sub_ = nh_.subscribe("input", 1, &LaserScanIntensityFilter::messageCallback, this);
  }

  void unsubscribe() {
    ROS_INFO("unsubscribe laserscan topic");
    sub_.shutdown();
  }

  void connectCallback(const ros::SingleSubscriberPublisher&) {
    if (subscriberCount_++ == 0) {
      subscribe();
    }
  }

  void disconnectCallback(const ros::SingleSubscriberPublisher&) {
    subscriberCount_--;
    if (subscriberCount_ == 0) {
      unsubscribe();
    }
  }

};
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_scan_intensity_filter");
  hrpsys_gazebo_general::LaserScanIntensityFilter f;
  ros::spin();
  return 0;
}