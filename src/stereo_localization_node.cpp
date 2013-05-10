/**
 * @file
 * @brief ROS node for stereo_localization code
 */


#include <ros/ros.h>
#include "stereo_localization_base.h"

int main(int argc, char **argv)
{
  ros::init(argc,argv,"stereo_localization");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  stereo_localization::StereoLocalizationBase stereo_localization(nh,nh_private);

  ros::spin();

  return 0;
}