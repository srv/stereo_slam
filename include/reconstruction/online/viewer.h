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
#include "receiver.h"

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
    double min_pose_change;       //!> Minimum pose change to update a cloud
    ros::NodeHandle nh;           //!> Public ros node handle
    ros::NodeHandle nh_private;   //!> Private ros node handle

    // Default settings
    Params () {
      work_dir                    = "";
      min_pose_change             = 0.005;
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

  // Access specifiers
  void setReceiver(reconstruction::Receiver receiver);

protected:

  // Protected functions and callbacks
  void buildCallback(const ros::WallTimerEvent& event);
  void updateCloudPoses();
  void getNewClouds();
  void insertCloud();
  void updateCloud(string id);
  void computeGeometry(string id, PointXY &centroid, double &radius);
  void updateVisualization();

private:

  Params params_;                         //!> Stores parameters
  bool lock_timer_;                       //!> Lock timer while executing
  ros::WallTimer timer_update_;           //!> Timer to update the viewer
  reconstruction::Receiver receiver_;     //!> Receiver object
  vector<Cloud> clouds_;                  //!> List of pointclouds
  boost::thread visualization_thread_;    //!> Visualization thread
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_; //!> The viewer
};

} // namespace

#endif // VIEWER_H