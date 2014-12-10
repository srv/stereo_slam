/**
 * @file
 * @brief Online viewer
 */

#ifndef VIEWER_H
#define VIEWER_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread.hpp>
#include "../../localization/tools.h"

using namespace std;

typedef pcl::PointXY                      PointXY;
typedef pcl::PointXYZRGB                  PointRGB;
typedef pcl::PointCloud<PointXY>          PointCloudXY;
typedef pcl::PointCloud<PointRGB>         PointCloudRGB;

namespace reconstruction
{

class Viewer
{

public:

	// Constructor
  Viewer();

  // Structure of parameters
  struct Params
  {
    string work_dir;              //!> Working directory.

    // Default settings
    Params () {
      work_dir                    = "";
    }
  };

  // Cloud structure
  struct Cloud
  {
    // Motion parameters
    string id;                    //!> Cloud id (equal to pointcloud)
    tf::Transform pose;           //!> Cloud pose
    PointXY centroid;             //!> Cloud centroid (in xy-plane)
    double radius;                //!> Distance between center and the furthest point of the pointcloud (in xy-plane)
    bool has_changed;             //!> True if the cloud pose has changed

    // Default settings
    Cloud () {
      id                          = "";
      radius                      = 0.0;
      has_changed                 = true;
    }
    Cloud (string id,
           tf::Transform pose,
           PointXY centroid,
           double radius,
           bool has_changed) :
           id(id),
           pose(pose),
           centroid(centroid),
           radius(radius),
           has_changed(has_changed)  {}
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

  // Start the viewer
  void start();

  // Stop the viewer
  void stop();

  // Update the clouds
  void update();

protected:

  // Protected functions and callbacks
  void computeGeometry(string id, PointXY &centroid, double &radius);
  void updateVisualization();

private:

  Params params_;                         //!> Stores parameters
  vector<Cloud> clouds_;                  //!> List of pointclouds
  boost::thread visualization_thread_;    //!> Visualization thread
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_; //!> The viewer
};

} // namespace

#endif // VIEWER_H