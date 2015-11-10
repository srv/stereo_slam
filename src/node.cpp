#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

#include "constants.h"
#include "publisher.h"
#include "tracking.h"
#include "graph.h"
#include "loop_closing.h"
#include "calibration.h"

namespace fs  = boost::filesystem;

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

  // For debugging purposes
  slam::Publisher publisher;

  // Threads
  slam::LoopClosing loop_closing;
  slam::Graph graph(&loop_closing);
  slam::Calibration calibration(&graph);
  slam::Tracking tracker(&publisher, &graph, &calibration);

  // Read parameters
  slam::Tracking::Params tracking_params;
  readParameters(tracking_params);

  // Set the parameters for every object
  tracker.setParams(tracking_params);
  loop_closing.setGraph(&graph);

  // Launch threads
  boost::thread trackingThread(&slam::Tracking::run, &tracker);
  boost::thread graphThread(&slam::Graph::run, &graph);
  boost::thread loopClosingThread(&slam::LoopClosing::run, &loop_closing);

  // ROS spin
  ros::Rate r(10);
  while (ros::ok())
  {
    r.sleep();
  }

  // Loop closing object is the only one that needs finalization
  loop_closing.finalize();

  // Optimize camera parameters
  calibration.run();

  ros::shutdown();

  return 0;
}