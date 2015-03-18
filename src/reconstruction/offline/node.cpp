#include <iostream>
#include <ros/ros.h>
#include "reconstruction/offline/base.h"

using namespace reconstruction;

// Main entry point
int main(int argc, char **argv)
{
  // Parse arguments
  if (argc < 2) {
    // Inform the user of how to use the program
    std::cout << "Usage is: rosrun stereo_slam reconstruction <working directory>\n";
    std::cin.get();
    exit(0);
  }
  string work_dir = argv[1];

  ros::init(argc,argv,"reconstruction");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Init node
  ReconstructionBase reconstruction(nh,nh_private);

  // Init the node
  reconstruction.setParameters(work_dir);

  // Accumulate all the clouds
  reconstruction.build3D();

  return 0;
}