/**
 * @file
 * @brief ROS node for pointcloud registration
 */

#include <ros/ros.h>
#include "reconstruction/registration_base.h"

int main(int argc, char **argv)
{
  ros::init(argc,argv,"registration");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  reconstruction::RegistrationBase registration(nh,nh_private);

  ros::spin();
  return 0;
}