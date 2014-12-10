/**
 * @file
 * @brief Receiver of localization information (graph, pointclouds, etc.)
 */

#ifndef RECEIVER_H
#define RECEIVER_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include "stereo_slam/SetGraph.h"
#include "stereo_slam/SetPointCloud.h"

using namespace std;

typedef pcl::PointXYZRGB                  PointRGB;
typedef pcl::PointCloud<PointRGB>         PointCloudRGB;

namespace reconstruction
{

class Receiver
{

public:

	// Constructor
  Receiver();

  // Structure of parameters
  struct Params
  {
    string work_dir;              //!> Working directory.
    string start_srv;             //!> Global name for the start reconstruction service
    string stop_srv;              //!> Global name for the stop reconstruction service
    ros::NodeHandle nh;           //!> Public ros node handle
    ros::NodeHandle nh_private;   //!> Private ros node handle

    // Default settings
    Params () {
      work_dir                    = "";
      start_srv                   = "";
      stop_srv                    = "";
    }
  };

  // Node structure
  struct Node
  {
    // Motion parameters
    string id;                    //!> Node id (equal to pointcloud)
    tf::Transform pose;           //!> Node pose
    bool has_saved;               //!> True if pointcloud has been received

    // Default settings
    Node () {
      id                          = "";
      has_saved                   = false;
    }
    Node (string id,
          tf::Transform pose,
          bool has_saved) :
          id(id),
          pose(pose),
          has_saved(has_saved) {}
  };

  /**
   * @param params new parameters
   */
  inline void setParams(const Params& params)
  {
    params_ = params;
  }

  /**
   * @return current parameters
   */
  inline Params params() const { return params_; }

  // Start the request timer
  void start();

  // Stop the request timer
  void stop();

  // Service callbacks
  bool recieveGraph(stereo_slam::SetGraph::Request &req,
                    stereo_slam::SetGraph::Response &res);
  bool recieveCloud(stereo_slam::SetPointCloud::Request &req,
                    stereo_slam::SetPointCloud::Response &res);

protected:

  // Protected functions and callbacks
  int isNode(string id);
  int getNode(string id, Node &node);
  void insertNode(Node n);
  void updateNode(int idx, Node n);
  void parseGraph(string graph);
  vector<string> parseString(string input, string delimiter);

private:

  Params params_;                   //!> Stores parameters
  vector<Node> graph_nodes_;        //!> List of graph nodes
  bool lock_graph_nodes_;           //!> Lock the graph nodes list while it is accessed
};

} // namespace

#endif // RECEIVER_H