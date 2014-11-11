#include "reconstruction/online/requester.h"
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/transforms.h>
#include "stereo_slam/GetPointCloud.h"
#include "stereo_slam/GetGraph.h"


namespace fs=boost::filesystem;
namespace alg=boost::algorithm;


/** \brief Class constructor.
  */
reconstruction::Requester::Requester() {}


/** \brief Timer event callback for the interaction with slam node
  * @return -1 if the node is not into the table, its index otherwise
  * \param the node id
  */
int reconstruction::Requester::isNode(string id)
{
  int ret = -1;
  for(uint i=0; i<graph_nodes_.size(); i++)
  {
    Node n = graph_nodes_[i];
    if (n.id == id)
    {
      ret = i;
      break;
    }
  }
  return ret;
}


/** \brief Timer event callback for the interaction with slam node
  * @return
  * \param the timer event
  */
void reconstruction::Requester::getThingsCallback(const ros::WallTimerEvent& event)
{
  if (lock_timer_) return;
  lock_timer_ = true;

  // Get the graph poses from the slam node
  getGraph();

  // Get the pointclouds from the slam node
  getPointclouds();

  lock_timer_ = false;
}


/** \brief Gets the graph from the slam node
  */
void reconstruction::Requester::getGraph()
{
  // Call the service to request the graph
  ros::ServiceClient graph_client = params_.nh.serviceClient<stereo_slam::GetGraph>(params_.get_graph_srv);
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
  parseGraph(graph);
}


/** \brief Gets the pointclouds from the slam node
  */
void reconstruction::Requester::getPointclouds()
{
  // Request the needed pointclouds
  for (uint i=0; i<graph_nodes_.size(); i++)
  {
    Node node = graph_nodes_[i];
    string cloud_path = params_.work_dir + node.id + ".pcd";
    if (fs::exists(cloud_path)) continue;

    // Call the service to request the pointcloud
    ros::ServiceClient pointcloud_client = params_.nh.serviceClient<stereo_slam::GetPointCloud>(params_.get_point_cloud_srv);
    stereo_slam::GetPointCloud srv_pc;
    PointCloud::Ptr cloud(new PointCloud);
    srv_pc.request.id = node.id;
    if (pointcloud_client.call(srv_pc))
    {
      pcl::fromROSMsg(srv_pc.response.cloud, *cloud);
    }
    else
    {
      ROS_WARN_STREAM("[StereoSlam:] Failed to call service " << params_.get_point_cloud_srv);
      continue;
    }

    // TODO: handle the write error
    // Save the cloud
    pcl::io::savePCDFileBinary(cloud_path, *cloud);

    // Set this node as pc_saved
    node.pc_saved = true;
    graph_nodes_[i] = node;
  }
}


/** \brief Parse the graph string and save the contents into std vector
  * \param whole graph string.
  */
void reconstruction::Requester::parseGraph(string graph)
{
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
    tf::Transform pose(q, t);

    // Create the node
    Node node(id, pose, false);

    int idx = isNode(id);
    if (idx == -1)
      graph_nodes_.push_back(node);
    else
      graph_nodes_[idx] = node;
  }
}


/** \brief Reads the reconstruction node parameters
  * @return A vector with all the tokens into the input string
  * \param input string
  * \param delimiter
  */
vector<string> reconstruction::Requester::parseString(string input, string delimiter)
{
  vector<string> output;
  alg::split(output, input, alg::is_any_of(delimiter));
  return output;
}



/** \brief Starts the requester timer
  */
void reconstruction::Requester::start()
{
  // Init
  graph_nodes_.clear();

  // Used to lock the timer
  lock_timer_ = false;

  // Initialize the timer of graph request
  timer_graph_ = params_.nh_private.createWallTimer(ros::WallDuration(3.0),
                                                    &reconstruction::Requester::getThingsCallback,
                                                    this);
}

/** \brief Stops the requester timer
  */
void reconstruction::Requester::stop()
{
  timer_graph_.stop();
}