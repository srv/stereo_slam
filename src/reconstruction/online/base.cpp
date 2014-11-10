#include "reconstruction/online/base.h"
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/transforms.h>
#include "stereo_slam/GetPointCloud.h"
#include "stereo_slam/GetGraph.h"


namespace fs=boost::filesystem;
namespace alg=boost::algorithm;


/** \brief Class constructor. Reads node parameters and initialize some properties.
  * @return
  * \param nh public node handler
  * \param nhp private node handler
  */
reconstruction::ReconstructionBase::ReconstructionBase(
  ros::NodeHandle nh, ros::NodeHandle nhp) : nh_(nh), nh_private_(nhp)
{
  // Read the node parameters
  readParameters();

  // Initialize the node
  init();
}


/** \brief Reads the reconstruction node parameters
  */
void reconstruction::ReconstructionBase::graphCallback(const ros::WallTimerEvent& event)
{
  if (lock_timer_) return;
  lock_timer_ = true;

  // Call the service to request the graph
  ros::ServiceClient graph_client = nh_.serviceClient<stereo_slam::GetGraph>(params_.get_graph_srv);
  stereo_slam::GetGraph srv_graph;
  string graph = "";
  if (graph_client.call(srv_graph))
  {
    graph = srv_graph.response.graph;
  }
  else
  {
    ROS_WARN_STREAM("[StereoSlam:] Failed to call service " << params_.get_graph_srv);
    lock_timer_ = false;
    return;
  }

  // Parse the graph
  vector< pair<string, tf::Transform> > graph_poses;
  parseGraph(graph, graph_poses);

  // Request the needed pointclouds
  for (uint i=0; i<graph_poses.size(); i++)
  {
    string cloud_path = params_.work_dir + graph_poses[i].first + ".pcd";
    if (fs::exists(cloud_path)) continue;

    // Call the service to request the pointcloud
    ros::ServiceClient pointcloud_client = nh_.serviceClient<stereo_slam::GetPointCloud>(params_.get_point_cloud_srv);
    stereo_slam::GetPointCloud srv_pc;
    PointCloud::Ptr cloud(new PointCloud);
    srv_pc.request.id = graph_poses[i].first;
    if (pointcloud_client.call(srv_pc))
    {
      pcl::fromROSMsg(srv_pc.response.cloud, *cloud);
    }
    else
    {
      ROS_WARN_STREAM("[StereoSlam:] Failed to call service " << params_.get_point_cloud_srv);
      continue;
    }

    // Save the cloud
    pcl::io::savePCDFileBinary(cloud_path, *cloud);
  }
  lock_timer_ = false;
}


/** \brief Parse the graph string and save the contents into std vector
  * \param whole graph string.
  * \param graph_poses is a vector with all the graph poses and its id's.
  */
void reconstruction::ReconstructionBase::parseGraph(string graph,
                                                    vector< pair<string, tf::Transform> > &graph_poses)
{
  graph_poses.clear();

  // Parse graph string
  vector<string> graph_vertices = parseString(graph, "\n");

  // Save the graph poses
  for (uint i=0; i<graph_vertices.size(); i++)
  {
    // Parse line (node info)
    vector<string> line = parseString(graph_vertices[i], ",");
    if (line.size() != 12)
      continue;

    // Get the node values
    string id = line[1];
    double x  = boost::lexical_cast<double>(line[5]);
    double y  = boost::lexical_cast<double>(line[6]);
    double z  = boost::lexical_cast<double>(line[7]);
    double qx = boost::lexical_cast<double>(line[8]);
    double qy = boost::lexical_cast<double>(line[9]);
    double qz = boost::lexical_cast<double>(line[10]);
    double qw = boost::lexical_cast<double>(line[11]);

    // Build transform
    tf::Vector3 t(x, y, z);
    tf::Quaternion q(qx, qy, qz, qw);
    tf::Transform transf(q, t);

    // Save
    graph_poses.push_back(make_pair(id, transf));
  }
}


/** \brief Reads the reconstruction node parameters
  * @return A vector with all the tokens into the input string
  * \param input string
  * \param delimiter
  */
vector<string> reconstruction::ReconstructionBase::parseString(string input, string delimiter)
{
  vector<string> output;
  alg::split(output, input, alg::is_any_of(delimiter));
  return output;
}


/** \brief Reads the reconstruction node parameters
  */
void reconstruction::ReconstructionBase::readParameters()
{
  Params params;

  // Operational directory
  string work_dir;
  nh_private_.param("work_dir", work_dir, string(""));
  if (work_dir[work_dir.length()-1] != '/')
    work_dir += "/";
  params.work_dir = work_dir + "reconstruction/";

  // Create the directory
  if (fs::is_directory(params.work_dir))
    fs::remove_all(params.work_dir);
  fs::path dir(params.work_dir);
  if (!fs::create_directory(dir))
    ROS_ERROR("[StereoSlam:] ERROR -> Impossible to create the reconstruction directory.");

  // Service names
  nh_private_.param("get_point_cloud_srv", params.get_point_cloud_srv, string(""));
  nh_private_.param("get_graph_srv", params.get_graph_srv, string(""));

  setParams(params);
}


/** \brief Initialize the node
  */
void reconstruction::ReconstructionBase::init()
{
  // Used to lock the timer
  lock_timer_ = false;

  // Initialize the timer of graph request
  timer_graph_ = nh_private_.createWallTimer(ros::WallDuration(3.0),
                                             &reconstruction::ReconstructionBase::graphCallback,
                                             this);
}