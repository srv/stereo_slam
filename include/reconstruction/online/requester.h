/**
 * @file
 * @brief Requester of localization information (graph, pointclouds, etc.)
 */

#ifndef REQUESTER_H
#define REQUESTER_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>

using namespace std;

typedef pcl::PointXY                      PointXY;
typedef pcl::PointXYZRGB                  PointRGB;
typedef pcl::PointCloud<PointRGB>         PointCloud;

namespace reconstruction
{

class Requester
{

public:

	// Constructor
  Requester();

  // Structure of parameters
  struct Params
  {
    string work_dir;              //!> Working directory.
    string get_point_cloud_srv;   //!> Global name for the get pointcloud service
    string get_graph_srv;         //!> Global name for the get graph service
    ros::NodeHandle nh;           //!> Public ros node handle
    ros::NodeHandle nh_private;   //!> Private ros node handle

    // Default settings
    Params () {
      work_dir                    = "";
      get_point_cloud_srv         = "";
      get_graph_srv               = "";
    }
  };

  // Node structure
  struct Node
  {
    // Motion parameters
    string id;                    //!> Node id (equal to pointcloud)
    tf::Transform pose;           //!> Node pose
    bool pc_saved;                //!> True if pointcloud has been received

    // Default settings
    Node () {
      id                          = "";
      pc_saved                    = false;
    }
    Node (string id, tf::Transform pose, bool pc_saved) {
      id                          = id;
      pose                        = pose;
      pc_saved                    = pc_saved;
    }
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

protected:

  // Protected functions and callbacks
  int isNode(string id);
  void getThingsCallback(const ros::WallTimerEvent& event);
  void getGraph();
  void getPointclouds();
  void parseGraph(string graph);
  vector<string> parseString(string input, string delimiter);

private:

  Params params_;                   //!> Stores parameters
  bool lock_timer_;                 //!> Lock timer while executing
  ros::WallTimer timer_graph_;      //!> Timer to request the graph to slam node
  vector<Node> graph_nodes_;        //!> List of graph nodes
};

} // namespace

#endif // REQUESTER_H