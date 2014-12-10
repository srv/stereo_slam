#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include "reconstruction/online/base.h"
#include "reconstruction/online/receiver.h"

using namespace reconstruction;

int main(int argc, char **argv)
{
  // Override SIGINT handler
  ros::init(argc, argv, "reconstruction");

  // Stereo slam class
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ReconstructionBase reconstruction(nh,nh_private);

  // FIXME: Advertising services here because not working on inside the class
  reconstruction::Receiver receiver = reconstruction.getReceiver();
  ros::ServiceServer recieve_graph_srv = nh_private.advertiseService("recieve_graph",
                                                &reconstruction::Receiver::recieveGraph,
                                                &receiver);
  ros::ServiceServer recieve_cloud_srv = nh_private.advertiseService("recieve_cloud",
                                                &reconstruction::Receiver::recieveCloud,
                                                &receiver);

  // Start the reconstruction
  reconstruction.start();

  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  spinner.spin();

  // Finalize
  reconstruction.stop();

  // Exit
  return (0);
}