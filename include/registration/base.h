/**
 * @file
 * @brief Pointcloud registration (presentation).
 */

#ifndef REGISTRATION_BASE_H
#define REGISTRATION_BASE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_transport/subscriber_filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf/transform_broadcaster.h>

//#include "polybool/polybool.h"

using namespace std;

//typedef pcl::PointXYZRGB                  Point;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointNormal                  PointNormalT;
typedef pcl::PointCloud<PointNormalT>     PointCloudWithNormals;

namespace registration
{

class RegistrationBase
{

public:

	// Constructor
  RegistrationBase(ros::NodeHandle nh, ros::NodeHandle nhp);


  struct Params
  {
    double x_filter_min;              //!> Statistical filter
    double x_filter_max;
    double y_filter_min;
    double y_filter_max;
    double z_filter_min;
    double z_filter_max;
    double voxel_size_x;              //!> Voxel grid
    double voxel_size_y;
    double voxel_size_z;
    double radius_search;             //!> Remove isolated points
    int min_neighors_in_radius;

    // Default settings
    Params () {
      x_filter_min            = -2.0;
      x_filter_max            = 2.0;
      y_filter_min            = -2.0;
      y_filter_max            = 2.0;
      z_filter_min            = 0.2;
      z_filter_max            = 2.0;
      voxel_size_x            = 0.005;
      voxel_size_y            = 0.005;
      voxel_size_z            = 0.005;
      radius_search           = 0.2;
      min_neighors_in_radius  = 20;
    }
  };

  struct rectangle
  {
    pcl::PointXYZRGB corner_down_left;
    pcl::PointXYZRGB corner_down_right; 
    pcl::PointXYZRGB corner_up_right; 
    pcl::PointXYZRGB corner_up_left;
  };
  rectangle last_rectangle;
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
  void msgsCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                    const sensor_msgs::ImageConstPtr& l_img_msg,
                    const sensor_msgs::ImageConstPtr& r_img_msg,
                    const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                    const sensor_msgs::CameraInfoConstPtr& r_info_msg,
                    const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void filter(PointCloud::Ptr cloud);
  void compute_2D_vertex(pcl::PointXYZRGB minims, pcl::PointXYZRGB max, rectangle *r1);
  double compute_Intersect_Area(rectangle current_rect, rectangle last_rect);
  

private:

  // Topic properties
  image_transport::SubscriberFilter left_sub_, right_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_, right_info_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;

  // Topic sync properties
  typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry,
                                                    sensor_msgs::Image,
                                                    sensor_msgs::Image,
                                                    sensor_msgs::CameraInfo,
                                                    sensor_msgs::CameraInfo,
                                                    sensor_msgs::PointCloud2> ExactPolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  boost::shared_ptr<ExactSync> exact_sync_;

  Params params_;                   //!> Stores parameters
  bool first_iter_;
  tf::Transform last_odom;
  
};

} // namespace

#endif // REGISTRATION_BASE_H