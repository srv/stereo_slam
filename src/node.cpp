/**
 * @file
 * @brief ROS node for stereo_slam code
 */


#include <ros/ros.h>
#include "base.h"

int main(int argc, char **argv)
{
  ros::init(argc,argv,"stereo_slam");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  stereo_slam::StereoSlamBase stereo_slam(nh,nh_private);

  // Use 2 async threads, one for every callback: messages and timer
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}