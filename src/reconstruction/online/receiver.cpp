#include "reconstruction/online/receiver.h"
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/transforms.h>
#include <std_srvs/Empty.h>
#include <unistd.h>
#include "stereo_slam/GetPointCloud.h"
#include "stereo_slam/GetGraph.h"


namespace fs=boost::filesystem;
namespace alg=boost::algorithm;


/** \brief Class constructor.
  */
reconstruction::Receiver::Receiver() : lock_graph_nodes_(false) {}


/** \brief Timer event callback for the interaction with slam node
  * @return -1 if the node is not into the table, its index otherwise
  * \param the node id
  */
int reconstruction::Receiver::isNode(string id)
{
  // Wait until unlock
  while (lock_graph_nodes_) {}

  // Lock
  lock_graph_nodes_ = true;

  // Search the node
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

  // Unlock
  lock_graph_nodes_ = false;

  return ret;
}


/** \brief Insert a node into the graph_nodes list
  * \param the node to be inserted
  */
void reconstruction::Receiver::insertNode(Node n)
{
  // Wait until unlock
  while (lock_graph_nodes_) {}

  // Lock
  lock_graph_nodes_ = true;

  // Insert
  graph_nodes_.push_back(n);

  // Unlock
  lock_graph_nodes_ = false;
}


/** \brief Update a node of the graph_nodes list
  * \param node index to be updated
  * \param the node to be updated
  */
void reconstruction::Receiver::updateNode(int pos, Node n)
{
  // Wait until unlock
  while (lock_graph_nodes_) {}

  // Lock
  lock_graph_nodes_ = true;

  // Insert
  graph_nodes_[pos] = n;

  // Unlock
  lock_graph_nodes_ = false;
}


/** \brief Retrieve the pose of a node
  * @return true if node exists, false otherwise
  * \param node index
  * \param pose of the node
  */
bool reconstruction::Receiver::getNodePose(string id, tf::Transform &pose)
{
  // Init
  bool exists = false;
  pose.setIdentity();

  // Search the node
  for (uint i=0; i<graph_nodes_.size(); i++)
  {
    // Get the node
    while (lock_graph_nodes_) {}
    lock_graph_nodes_ = true;
    Node n = graph_nodes_[i];
    lock_graph_nodes_ = false;

    if (n.id == id && n.has_saved)
    {
      pose = n.pose;
      exists = true;
      break;
    }
  }

  return exists;
}


/** \brief Gets the graph from the slam node
  */
bool reconstruction::Receiver::recieveGraph(stereo_slam::SetGraph::Request &req,
                                            stereo_slam::SetGraph::Response &res)
{
  ROS_INFO_STREAM("[Reconstruction:] Graph received.");

  // Parse the graph
  parseGraph(req.graph);

  // Exit
  res.res = "";
  return true;
}


/** \brief Gets the pointclouds from the slam node
  */
bool reconstruction::Receiver::recieveCloud(stereo_slam::SetPointCloud::Request &req,
                                            stereo_slam::SetPointCloud::Response &res)
{
  ROS_INFO_STREAM("[Reconstruction:] Pointcloud received.");

  // Get the pointcloud
  PointCloudRGB::Ptr cloud(new PointCloudRGB);
  pcl::fromROSMsg(req.cloud, *cloud);

  // TODO: handle the write error
  // Save the cloud
  string cloud_path = params_.work_dir + req.id + ".pcd";
  pcl::io::savePCDFileBinary(cloud_path, *cloud);

  // Exit
  res.res = "";
  return true;
}


/** \brief Parse the graph string and save the contents into std vector
  * \param whole graph string.
  */
void reconstruction::Receiver::parseGraph(string graph)
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

    // Insert or update
    int idx = isNode(id);
    if (idx == -1)
      insertNode(node);
    else
      updateNode(idx, node);
  }
}


/** \brief Reads the reconstruction node parameters
  * @return A vector with all the tokens into the input string
  * \param input string
  * \param delimiter
  */
vector<string> reconstruction::Receiver::parseString(string input, string delimiter)
{
  vector<string> output;
  alg::split(output, input, alg::is_any_of(delimiter));
  return output;
}


/** \brief Starts the receiver timer
  */
void reconstruction::Receiver::start()
{
  // Init
  graph_nodes_.clear();

  // Start receiving things
  ros::ServiceClient start = params_.nh.serviceClient<std_srvs::Empty>(params_.start_srv);
  std_srvs::Empty empty;
  while (!start.call(empty))
  {
    ROS_WARN_STREAM("[Reconstruction:] Service " << params_.start_srv << " no yet advertised.");
    usleep(1e9);
  }

}

/** \brief Stops the receiver timer
  */
void reconstruction::Receiver::stop()
{

}