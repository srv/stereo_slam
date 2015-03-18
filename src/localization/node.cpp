#include <ros/ros.h>
#include "localization/base.h"

int main(int argc, char **argv)
{
  // Override SIGINT handler
  ros::init(argc, argv, "stereo_slam");

  // Stereo slam class
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  slam::SlamBase slam_node(nh,nh_private);

  // ROS spin
  ros::MultiThreadedSpinner spinner(0);
  spinner.spin();

  return 0;
}