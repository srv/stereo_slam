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
    string clouds_dir;            //!> Directory where stereo_slam pointclouds are saved.
    string graph_file;            //!> stereo_slam output graph file.

    // Default settings
    Params () {
      work_dir                    = "";
      clouds_dir                  = "";
      graph_file                  = "";
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

  // 3D reconstruction
  void build3D();

protected:

	// Node handlers
	ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Protected functions and callbacks
  void readParameters();

private:

  // Read the poses from the graph file
  bool readPoses(vector< pair<string, tf::Transform> > &cloud_poses);

  Params params_;                   //!> Stores parameters
};

} // namespace

#endif // BASE_H