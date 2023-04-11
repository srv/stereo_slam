#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include "constants.h"
#include "publisher.h"
#include "tracking.h"
#include "graph.h"
#include "loop_closing.h"

namespace fs = boost::filesystem;

/** \brief Read the node parameters
  */
void readTrackingParams(slam::Tracking::Params &tracking_params)
{
  ros::NodeHandle nhp("~");
  nhp.param("odom_topic",   tracking_params.odom_topic,   string(""));
  nhp.param("camera_topic", tracking_params.camera_topic, string(""));
  nhp.param("refine",       tracking_params.refine,       false);
}

/** \brief Main entry point
  */
int main(int argc, char **argv)
{
  // Override SIGINT handler
  ros::init(argc, argv, "stereo_slam");
  ros::start();

  // Create the output directory
  string output_dir = slam::WORKING_DIRECTORY;
  if (fs::is_directory(output_dir))
  {
    ROS_ERROR_STREAM("[Localization:] ERROR -> The output directory already exists: " <<
      output_dir);
    return 0;
  }
  fs::path dir0(output_dir);
  if (!fs::create_directory(dir0))
    ROS_ERROR("[Localization:] ERROR -> Impossible to create the output directory.");

  // For debugging purposes
  slam::Publisher publisher;

  // Threads
  slam::LoopClosing loop_closing;
  slam::Graph graph(&loop_closing);
  slam::Tracking tracker(&publisher, &graph);

  // Read parameters
  slam::Tracking::Params tracking_params;
  readTrackingParams(tracking_params);

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

  ros::shutdown();

  return 0;
}