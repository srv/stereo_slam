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
void readParams(slam::Tracking::Params &tracking_params, slam::Graph::Params &graph_params, slam::LoopClosing::Params &loop_closing_params)
{
  ros::NodeHandle nhp("~");
  nhp.param("refine",                     tracking_params.refine,            false);
  nhp.param("distance_between_kayframes", tracking_params.dist_keyframes,    0.5);
  nhp.param("working_directory",          tracking_params.working_directory, ros::package::getPath("stereo_slam") + "/output/");

  graph_params.working_directory = tracking_params.working_directory;
  loop_closing_params.working_directory = tracking_params.working_directory;
  ROS_INFO_STREAM("TRACKING WORKING DIRECTORY: " << tracking_params.working_directory);
  ROS_INFO_STREAM("GRAPH WORKING DIRECTORY: " << graph_params.working_directory);
  ROS_INFO_STREAM("LOOP CLOSING WORKING DIRECTORY: " << loop_closing_params.working_directory);
  ROS_INFO_STREAM("distance_between_kayframes: " << tracking_params.dist_keyframes);
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
  slam::Tracking tracker(&publisher, &graph);

  // Read parameters
  slam::Tracking::Params tracking_params;
  slam::Graph::Params graph_params;
  slam::LoopClosing::Params loop_closing_params;
  readParams(tracking_params, graph_params, loop_closing_params);

  // Set the parameters for every object
  tracker.setParams(tracking_params);
  graph.setParams(graph_params);
  loop_closing.setParams(loop_closing_params);
  loop_closing.setGraph(&graph);

  // Create the output directory
  string output_dir = tracking_params.working_directory;
  if (fs::is_directory(output_dir))
  {
    ROS_ERROR_STREAM("[Localization:] ERROR -> The output directory already exists: " << output_dir);
    return 0;
  }
  fs::path dir0(output_dir);
  if (!fs::create_directory(dir0))
  {
    ROS_ERROR("[Localization:] ERROR -> Impossible to create the output directory.");
    return 0;
  }

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