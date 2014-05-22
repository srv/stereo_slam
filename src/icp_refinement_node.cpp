/**
 * @file
 * @brief ROS node for icp_refinement code
 */


#include <ros/ros.h>
#include "icp_refinement_base.h"

int main(int argc, char **argv)
{
  ros::init(argc,argv,"icp_refinement");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  stereo_slam::IcpRefinementBase icp_refinement(nh,nh_private);

  ros::spin();
  return 0;
}