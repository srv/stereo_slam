#include <ros/ros.h>
#include "reconstruction/base.h"

using namespace reconstruction;

// Main entry point
int main(int argc, char **argv)
{
  ros::init(argc,argv,"reconstruction");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Init node
  ReconstructionBase reconstruction(nh,nh_private);

  // Subscription is handled at start and stop service callbacks.
  //ros::spin();

  return 0;
}