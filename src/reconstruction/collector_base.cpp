#include "reconstruction/collector_base.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>


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
  PointCloud pcl_cloud;
  pcl::fromROSMsg(*cloud_msg, pcl_cloud);

  // Filter the cloud
  filter(pcl_cloud);

  // Filename will be the timestamp
  double cloud_time = graph_msg->header.stamp.toSec();
  std::stringstream cloud_time_str_tmp;
  cloud_time_str_tmp << fixed << setprecision(9) << cloud_time;
  string cloud_time_str = cloud_time_str_tmp.str();
  //string cloud_time_str = boost::lexical_cast<string>(cloud_time);
  cloud_time_str.erase(remove(cloud_time_str.begin(), cloud_time_str.end(), '.'), cloud_time_str.end());
  string filename = params_.work_dir + cloud_time_str + ".pcd";

  // Save
  if (pcl::io::savePCDFile(filename, pcl_cloud) != 0)
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
    work_dir += "/";
  collector_params.work_dir = work_dir;

  // Topic parameters
  string graph_topic, cloud_topic;
  nh_private_.param("graph_topic", graph_topic, string("/graph"));
  nh_private_.param("cloud_topic", cloud_topic, string("/points2"));

  // Filter parameters
  nh_private_.param("x_filter_min", collector_params.x_filter_min, -2.0);
  nh_private_.param("x_filter_max", collector_params.x_filter_max, 2.0);
  nh_private_.param("y_filter_min", collector_params.y_filter_min, -2.0);
  nh_private_.param("y_filter_max", collector_params.y_filter_max, 2.0);
  nh_private_.param("z_filter_min", collector_params.z_filter_min, 0.2);
  nh_private_.param("z_filter_max", collector_params.z_filter_max, 2.0);
  nh_private_.param("voxel_size", collector_params.voxel_size, 0.01);
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
  // Callback synchronization
  approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(15), graph_sub_, cloud_sub_) );
  approximate_sync_->registerCallback(boost::bind(
      &reconstruction::CollectorBase::msgsCallback,
      this, _1, _2));

  return true;
}

/** \brief Filter the input cloud
  * \param input cloud
  */
void reconstruction::CollectorBase::filter(PointCloud& cloud)
{
  // NAN and limit filtering
  pcl::PassThrough<Point> pass_;

  // X-filtering
  pass_.setFilterFieldName("x");
  pass_.setFilterLimits(params_.x_filter_min, params_.x_filter_max);
  pass_.setInputCloud(cloud.makeShared());
  pass_.filter(cloud);

  // Y-filtering
  pass_.setFilterFieldName("y");
  pass_.setFilterLimits(params_.y_filter_min, params_.y_filter_max);
  pass_.setInputCloud(cloud.makeShared());
  pass_.filter(cloud);

  // Z-filtering
  pass_.setFilterFieldName("z");
  pass_.setFilterLimits(params_.z_filter_min, params_.z_filter_max);
  pass_.setInputCloud(cloud.makeShared());
  pass_.filter(cloud);

  // Downsampling using voxel grid
  pcl::VoxelGrid<Point> grid_;
  grid_.setLeafSize(params_.voxel_size,
                    params_.voxel_size,
                    params_.voxel_size);
  grid_.setDownsampleAllData(true);
  grid_.setInputCloud(cloud.makeShared());
  grid_.filter(cloud);

  // Remove isolated points
  pcl::RadiusOutlierRemoval<Point> outrem;  
  outrem.setRadiusSearch(params_.radius_search);
  outrem.setMinNeighborsInRadius(params_.min_neighors_in_radius);
  outrem.setInputCloud(cloud.makeShared());
  outrem.filter(cloud);
}