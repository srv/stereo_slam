/**
 * @file
 * @brief Collector of pointclouds (presentation).
 */

#ifndef COLLECTOR_BASE_H
#define COLLECTOR_BASE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;

typedef pcl::PointXYZRGB                  Point;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

namespace reconstruction
{

class CollectorBase
{

public:

	// Constructor
  CollectorBase(ros::NodeHandle nh, ros::NodeHandle nhp);


  struct Params
  {
    string work_dir;                  //!> Directory where pointcloud files will be saved.
    double x_filter_min;              //!> Statistical filter
    double x_filter_max;
    double y_filter_min;
    double y_filter_max;
    double z_filter_min;
    double z_filter_max;
    double voxel_size;                //!> Voxel grid
    double radius_search;             //!> Remove isolated points
    int min_neighors_in_radius;

    // Default settings
    Params () {
      work_dir                = "";
      x_filter_min            = -2.0;
      x_filter_max            = 2.0;
      y_filter_min            = -2.0;
      y_filter_max            = 2.0;
      z_filter_min            = 0.2;
      z_filter_max            = 2.0;
      voxel_size              = 0.01;
      radius_search           = 0.2;
      min_neighors_in_radius  = 20;
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
  bool init();
  void readParameters();
  void msgsCallback(  const nav_msgs::Odometry::ConstPtr& graph_msg,
                      const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void filter(PointCloud& cloud);

private:

  // Topic properties
  message_filters::Subscriber<nav_msgs::Odometry> graph_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;

  // Topic sync properties
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, 
                                                    sensor_msgs::PointCloud2> ApproximatePolicy;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  Params params_;                   //!> Stores parameters
};

} // namespace

#endif // COLLECTOR_BASE_H