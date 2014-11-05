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

  // Accumulate all the clouds
  reconstruction.build3D();

  return 0;
}