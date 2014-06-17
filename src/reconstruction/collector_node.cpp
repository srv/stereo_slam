/**
 * @file
 * @brief ROS node for pointcloud collector
 */

#include <ros/ros.h>
#include "reconstruction/collector_base.h"

int main(int argc, char **argv)
{
  ros::init(argc,argv,"collector");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  reconstruction::CollectorBase collector(nh,nh_private);

  ros::spin();
  return 0;
}