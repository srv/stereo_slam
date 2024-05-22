#include <ros/ros.h>
#include <ros/package.h>

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

#include "publisher.hpp"
#include "tracking.hpp"
#include "graph.hpp"
#include "loop_closing.hpp"

/** \brief Read the node parameters
  */
void readParams(slam::Tracking::Params &tracking_params, slam::Graph::Params &graph_params, slam::LoopClosing::Params &loop_closing_params)
{
  ros::NodeHandle nhp("~");
  nhp.param("refine",                     tracking_params.refine,                     false);
  nhp.param("distance_between_keyframes", tracking_params.dist_keyframes,             0.5);
  nhp.param("working_directory",          tracking_params.working_directory,          ros::package::getPath("stereo_slam") + "/output/");
  nhp.param("feature_detector_selection", tracking_params.feature_detector_selection, std::string("ORB"));
  nhp.param("lc_min_inliers",             tracking_params.lc_min_inliers,             30);
  nhp.param("lc_epipolar_thresh",         tracking_params.lc_epipolar_thresh,         1.0);
  nhp.param("map_frame_name",             graph_params.map_frame_id,                  std::string("/robot_0/map"));
  nhp.param("lc_neighbors",               loop_closing_params.lc_neighbors,           5); 
  nhp.param("lc_discard_window",          loop_closing_params.lc_discard_window,      20); 
  nhp.param("ransac_iterations",          loop_closing_params.ransac_iterations,      150);
  

  graph_params.working_directory         = tracking_params.working_directory;
  loop_closing_params.working_directory  = tracking_params.working_directory;
  loop_closing_params.lc_min_inliers     = tracking_params.lc_min_inliers;
  loop_closing_params.lc_epipolar_thresh = tracking_params.lc_epipolar_thresh;

  ROS_INFO_STREAM("PARAMETER SETTING:               " << std::endl <<
                  "TRACKING WORKING DIRECTORY     = " << tracking_params.working_directory << std::endl <<
                  "GRAPH WORKING DIRECTORY        = " << graph_params.working_directory << std::endl <<
                  "LOOP CLOSING WORKING DIRECTORY = " << loop_closing_params.working_directory << std::endl <<
                  "FEATURE DETECTOR               = " << tracking_params.feature_detector_selection << std::endl <<
                  "DISTANCE BETWEEN KEYFRAMES     = " << tracking_params.dist_keyframes << std::endl <<
                  "MAP FRAME NAME                 = " << graph_params.map_frame_id << std::endl <<
                  "LC MIN INLIERS                 = " << loop_closing_params.lc_min_inliers << std::endl <<
                  "LC EPIPOLAR THRESHOLD          = " << loop_closing_params.lc_epipolar_thresh << std::endl <<
                  "LC NEIGHBORS                   = " << loop_closing_params.lc_neighbors << std::endl <<
                  "LC DISCARD WINDOW              = " << loop_closing_params.lc_discard_window << std::endl <<
                  "RANSAC ITERATIONS              = " << loop_closing_params.ransac_iterations);
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
  std::string output_dir = tracking_params.working_directory;
  if (boost::filesystem::is_directory(output_dir))
  {
    ROS_ERROR_STREAM("[Localization:] ERROR -> The output directory already exists: " << output_dir);
    return 0;
  }
  boost::filesystem::path dir0(output_dir);
  if (!boost::filesystem::create_directory(dir0))
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