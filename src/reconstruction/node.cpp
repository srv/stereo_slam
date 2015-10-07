#include <ros/ros.h>

#include "reconstruction/vrip.h"


/** \brief Main entry point
  */
int main(int argc, char **argv)
{
  // Override SIGINT handler
  ros::init(argc, argv, "stereo_slam_reconstruction");

  // Merge pointclouds
  reconstruction::Vrip vrip;
  vrip.merge();

  ros::spin();
  return 0;
}
