/**
 * @file
 * @brief 3D reconstruction using the output of the stereo_slam (presentation).
 */

#ifndef BASE_H
#define BASE_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>

using namespace std;

typedef pcl::PointXY                      PointXY;
typedef pcl::PointXYZRGB                  PointRGB;
typedef pcl::PointCloud<PointRGB>         PointCloud;

namespace reconstruction
{

class ReconstructionBase
{

public:

	// Constructor
  ReconstructionBase(ros::NodeHandle nh, ros::NodeHandle nhp);

  struct Params
  {
    // Motion parameters
    string work_dir;              //!> Working directory.
    string get_point_cloud_srv;   //!> Global name for the get pointcloud service
    string get_graph_srv;         //!> Global name for the get graph service

    // Default settings
    Params () {
      work_dir                    = "";
      get_point_cloud_srv         = "";
      get_graph_srv               = "";
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

protected:

	// Node handlers
	ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Protected functions and callbacks
  void readParameters();
  void init();
  void graphCallback(const ros::WallTimerEvent& event);
  void parseGraph(string graph,
                  vector< pair<string, tf::Transform> > &graph_poses);
  vector<string> parseString(string input, string delimiter);

private:

  Params params_;                   //!> Stores parameters
  bool lock_timer_;                 //!> Lock timer while executing
  ros::WallTimer timer_graph_;      //!> Timer to request the graph to slam node
};

} // namespace

#endif // BASE_H