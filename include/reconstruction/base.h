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
typedef pcl::PointXYZ                     PointXYZ;
typedef pcl::PointXYZRGB                  PointRGB;
typedef pcl::PointCloud<PointXYZ>         PointCloudXYZ;
typedef pcl::PointCloud<PointRGB>         PointCloudRGB;

namespace reconstruction
{

class ReconstructionBase
{

public:

	// Constructor
  ReconstructionBase(ros::NodeHandle nh, ros::NodeHandle nhp);

  struct Params
  {
    string work_dir;              //!> Working directory.
    string clouds_dir;            //!> Directory where stereo_slam pointclouds are saved.
    string graph_file;            //!> stereo_slam output graph file.
    string output_dir;            //!> Output directory where the files will be stored.

    // Default settings
    Params () {
      work_dir                    = "";
      clouds_dir                  = "";
      graph_file                  = "";
      output_dir                  = "";
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

  // Set the parameters
  void setParameters(string work_dir);

  // Translate the clouds
  void translateClouds();

  // 3D reconstruction
  void build3D();

protected:

	// Node handlers
	ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

private:

  // Read the poses from the graph file
  bool readPoses(vector< pair<string, tf::Transform> > &cloud_poses);

  Params params_;                   //!> Stores parameters
};

} // namespace

#endif // BASE_H