#include "registration/base.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>


/** \brief Class constructor. Reads node parameters and initialize some properties.
  * @return
  * \param nh public node handler
  * \param nhp private node handler
  */
registration::RegistrationBase::RegistrationBase(
  ros::NodeHandle nh, ros::NodeHandle nhp) : nh_(nh), nh_private_(nhp)
{
  // Read the node parameters
  readParameters();

  // Initialize the registration
  init();
}

/** \brief Messages callback. This function is called when synchronized odometry, images and pointcloud
  * messages are received.
  * @return
  * \param odom_msg ros odometry message of type nav_msgs::Odometry
  * \param l_img left stereo image message of type sensor_msgs::Image
  * \param r_img right stereo image message of type sensor_msgs::Image
  * \param l_info left stereo info message of type sensor_msgs::CameraInfo
  * \param r_info right stereo info message of type sensor_msgs::CameraInfo
  * \param cloud_msg ros pointcloud message of type sensor_msgs::PointCloud2
  */
void registration::RegistrationBase::msgsCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                                                  const sensor_msgs::ImageConstPtr& l_img_msg,
                                                  const sensor_msgs::ImageConstPtr& r_img_msg,
                                                  const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                                  const sensor_msgs::CameraInfoConstPtr& r_info_msg,
                                                  const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  if (!first_iter_) return;

  // Get the cloud
  PointCloud::Ptr pcl_cloud(new PointCloud);
  pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

  // Filter the cloud
  filter(pcl_cloud);

  Point min_pt, max_pt;
  pcl::getMinMax3D(*pcl_cloud, min_pt, max_pt);

  first_iter_ = false;

  // draw the cloud and the box
  pcl::visualization::PCLVisualizer viewer;
  viewer.addPointCloud(pcl_cloud);
  viewer.addCube(min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z);
  viewer.spin();
}

/** \brief Reads the stereo slam node parameters
  * @return
  */
void registration::RegistrationBase::readParameters()
{
  Params params;

  // Topic parameters
  string odom_topic, left_topic, right_topic, left_info_topic, right_info_topic, cloud_topic;
  nh_private_.param("odom_topic", odom_topic, string("/odometry"));
  nh_private_.param("left_topic", left_topic, string("/left/image_rect_color"));
  nh_private_.param("right_topic", right_topic, string("/right/image_rect_color"));
  nh_private_.param("left_info_topic", left_info_topic, string("/left/camera_info"));
  nh_private_.param("right_info_topic", right_info_topic, string("/right/camera_info"));
  nh_private_.param("cloud_topic", cloud_topic, string("/points2"));

  // Filter parameters
  nh_private_.param("x_filter_min", params.x_filter_min, -2.0);
  nh_private_.param("x_filter_max", params.x_filter_max, 2.0);
  nh_private_.param("y_filter_min", params.y_filter_min, -2.0);
  nh_private_.param("y_filter_max", params.y_filter_max, 2.0);
  nh_private_.param("z_filter_min", params.z_filter_min, 0.2);
  nh_private_.param("z_filter_max", params.z_filter_max, 2.0);
  nh_private_.param("voxel_size_x", params.voxel_size_x, 0.005);
  nh_private_.param("voxel_size_y", params.voxel_size_y, 0.005);
  nh_private_.param("voxel_size_z", params.voxel_size_z, 0.005);
  nh_private_.param("radius_search", params.radius_search, 0.2);
  nh_private_.param("min_neighors_in_radius", params.min_neighors_in_radius, 20);

  // Set the class parameters
  setParams(params);

  // Topics subscriptions
  image_transport::ImageTransport it(nh_);
  odom_sub_       .subscribe(nh_, odom_topic,       1);
  left_sub_       .subscribe(it,  left_topic,       1);
  right_sub_      .subscribe(it,  right_topic,      1);
  left_info_sub_  .subscribe(nh_, left_info_topic,  1);
  right_info_sub_ .subscribe(nh_, right_info_topic, 1);
  cloud_sub_      .subscribe(nh_, cloud_topic,      1);
}

/** \brief Initializes the registration node
  * @return true if init OK
  */
bool registration::RegistrationBase::init()
{
  // Callback synchronization
  exact_sync_.reset(new ExactSync(ExactPolicy(5),
                                  odom_sub_,
                                  left_sub_,
                                  right_sub_,
                                  left_info_sub_,
                                  right_info_sub_,
                                  cloud_sub_) );
  exact_sync_->registerCallback(boost::bind(
      &registration::RegistrationBase::msgsCallback,
      this, _1, _2, _3, _4, _5, _6));

  first_iter_ = true;

  return true;
}

/** \brief Filter the input cloud
  * \param input cloud
  */
void registration::RegistrationBase::filter(PointCloud::Ptr cloud)
{
  // NAN and limit filtering
  pcl::PassThrough<Point> pass_;

  // X-filtering
  pass_.setFilterFieldName("x");
  pass_.setFilterLimits(params_.x_filter_min, params_.x_filter_max);
  pass_.setInputCloud(cloud);
  pass_.filter(*cloud);

  // Y-filtering
  pass_.setFilterFieldName("y");
  pass_.setFilterLimits(params_.y_filter_min, params_.y_filter_max);
  pass_.setInputCloud(cloud);
  pass_.filter(*cloud);

  // Z-filtering
  pass_.setFilterFieldName("z");
  pass_.setFilterLimits(params_.z_filter_min, params_.z_filter_max);
  pass_.setInputCloud(cloud);
  pass_.filter(*cloud);


  // Downsampling using voxel grid
  pcl::VoxelGrid<Point> grid_;
  grid_.setLeafSize(params_.voxel_size_x,
                    params_.voxel_size_y,
                    params_.voxel_size_z);
  grid_.setDownsampleAllData(true);
  grid_.setInputCloud(cloud);
  grid_.filter(*cloud);

  // Remove isolated points
  pcl::StatisticalOutlierRemoval<Point> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter(*cloud);

}