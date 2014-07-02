#include "reconstruction/collector_base.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <boost/filesystem.hpp>

namespace fs=boost::filesystem;


/** \brief Class constructor. Reads node parameters and initialize some properties.
  * @return 
  * \param nh public node handler
  * \param nhp private node handler
  */
reconstruction::CollectorBase::CollectorBase(
  ros::NodeHandle nh, ros::NodeHandle nhp) : nh_(nh), nh_private_(nhp)
{
  // Read the node parameters
  readParameters();

  // Initialize the stereo slam
  init();
}


/** \brief Messages callback. This function is called when synchronized odometry and pointcloud
  * message are received.
  * @return 
  * \param graph_msg ros odometry message of type nav_msgs::Odometry
  * \param cloud_msg ros pointcloud message of type sensor_msgs::PointCloud2
  */
void reconstruction::CollectorBase::msgsCallback(
                                  const nav_msgs::Odometry::ConstPtr& graph_msg,
                                  const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Get the cloud
  PointCloud::Ptr pcl_cloud(new PointCloud);
  pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

  // Filter the cloud
  filter(pcl_cloud);

  // Filename will be the timestamp
  double cloud_time = graph_msg->header.stamp.toSec();
  stringstream cloud_time_str_tmp;
  cloud_time_str_tmp << fixed << setprecision(9) << cloud_time;
  string cloud_time_str = cloud_time_str_tmp.str();
  cloud_time_str.erase(remove(cloud_time_str.begin(), cloud_time_str.end(), '.'), cloud_time_str.end());
  string filename = params_.work_dir + "/" + cloud_time_str + ".pcd";

  // Save
  if (pcl::io::savePCDFile(filename, *pcl_cloud) != 0)
    ROS_ERROR_STREAM("[StereoSlam:] Could not save pointcloud into: " << filename);
}


/** \brief Reads the stereo slam node parameters
  * @return
  */
void reconstruction::CollectorBase::readParameters()
{
  Params collector_params;

  // Operational directories
  string work_dir;
  nh_private_.param("work_dir", work_dir, string(""));
  if (work_dir[work_dir.length()-1] != '/')
    work_dir += "/3D";
  collector_params.work_dir = work_dir;

  // Topic parameters
  string graph_topic, cloud_topic;
  nh_private_.param("graph_topic", graph_topic, string("/graph"));
  nh_private_.param("cloud_topic", cloud_topic, string("/points2"));
  nh_private_.param("queue_size", collector_params.queue_size, 15);

  // Filter parameters
  nh_private_.param("x_filter_min", collector_params.x_filter_min, -2.0);
  nh_private_.param("x_filter_max", collector_params.x_filter_max, 2.0);
  nh_private_.param("y_filter_min", collector_params.y_filter_min, -2.0);
  nh_private_.param("y_filter_max", collector_params.y_filter_max, 2.0);
  nh_private_.param("z_filter_min", collector_params.z_filter_min, 0.2);
  nh_private_.param("z_filter_max", collector_params.z_filter_max, 2.0);
  nh_private_.param("voxel_size_x", collector_params.voxel_size_x, 0.005);
  nh_private_.param("voxel_size_y", collector_params.voxel_size_y, 0.005);
  nh_private_.param("voxel_size_z", collector_params.voxel_size_z, 0.005);
  nh_private_.param("radius_search", collector_params.radius_search, 0.2);
  nh_private_.param("min_neighors_in_radius", collector_params.min_neighors_in_radius, 20);

  // Set the class parameters
  setParams(collector_params);

  // Topics subscriptions
  graph_sub_ .subscribe(nh_, graph_topic, 3);
  cloud_sub_ .subscribe(nh_, cloud_topic, 3);
}

/** \brief Initializes the collector node
  * @return true if init OK
  */
bool reconstruction::CollectorBase::init()
{
  // Create the directory to store the pointclouds
  if (fs::is_directory(params_.work_dir))
    fs::remove_all(params_.work_dir);
  fs::path dir(params_.work_dir);
  if (!fs::create_directory(dir))
    ROS_ERROR("[StereoSlam:] ERROR -> Impossible to create the 3D directory.");

  // Callback synchronization
  exact_sync_.reset(new ExactSync(ExactPolicy(params_.queue_size), graph_sub_, cloud_sub_) );
  exact_sync_->registerCallback(boost::bind(
      &reconstruction::CollectorBase::msgsCallback,
      this, _1, _2));

  return true;
}

/** \brief Filter the input cloud
  * \param input cloud
  */
void reconstruction::CollectorBase::filter(PointCloud::Ptr cloud)
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
  pcl::RadiusOutlierRemoval<Point> outrem;  
  outrem.setRadiusSearch(params_.radius_search);
  outrem.setMinNeighborsInRadius(params_.min_neighors_in_radius);
  outrem.setInputCloud(cloud);
  outrem.filter(*cloud);

}