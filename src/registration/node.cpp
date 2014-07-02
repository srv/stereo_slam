/**
 * @file
 * @brief ROS node for pointcloud registration
 */

#include <ros/ros.h>
#include "registration/base.h"

int main(int argc, char **argv)
{
  ros::init(argc,argv,"registration");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  registration::RegistrationBase registration(nh,nh_private);

  ros::spin();
  return 0;
}