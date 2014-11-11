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

using namespace std;

typedef pcl::PointXY                      PointXY;
typedef pcl::PointXYZRGB                  PointRGB;
typedef pcl::PointCloud<PointRGB>         PointCloud;

namespace reconstruction
{

class Viewer
{

public:

	// Constructor
  Viewer();

  // Start the viewer
  void start();

  // Stop the viewer
  void stop();

protected:


private:

  bool lock_timer_;                 //!> Lock timer while executing
  ros::WallTimer timer_update_;     //!> Timer to request the graph to slam node

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_; //!> The viewer
};

} // namespace

#endif // VIEWER_H