#include <ros/ros.h>

#include <boost/thread.hpp>

#include "localization/frame_publisher.h"
#include "localization/tracking.h"
#include "localization/graph.h"

/** \brief Read the node parameters
  */
void readParameters(slam::Tracking::Params &tracking_params)
{
  ros::NodeHandle nhp("~");
  string odom_topic, camera_topic;
  nhp.param("odom_topic",   tracking_params.odom_topic,   string(""));
  nhp.param("camera_topic", tracking_params.camera_topic, string(""));
}

/** \brief Main entry point
  */
int main(int argc, char **argv)
{
  // Override SIGINT handler
  ros::init(argc, argv, "stereo_slam");
  ros::start();

  // Frame publisher
  slam::FramePublisher f_pub;

  // Tracker
  slam::Tracking tracker(&f_pub);

  // Read parameters
  slam::Tracking::Params tracking_params;
  readParameters(tracking_params);

  // Set the parameters for every object
  tracker.setParams(tracking_params);

  // Launch threads
  boost::thread trackingThread(&slam::Tracking::run, &tracker);

  // ROS spin
  ros::Rate r(10);
  while (ros::ok())
  {
    r.sleep();
  }

  ros::shutdown();

  return 0;
}