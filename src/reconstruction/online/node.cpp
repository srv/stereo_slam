#include <ros/ros.h>
#include "reconstruction/online/base.h"

using namespace reconstruction;

// Main entry point
int main(int argc, char **argv)
{
  ros::init(argc,argv,"reconstruction");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Init node
  ReconstructionBase reconstruction(nh,nh_private);

  // Call callbacks from multiple threads
  ros::AsyncSpinner spinner(2); // Use 2 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}