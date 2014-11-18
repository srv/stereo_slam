/**
 * @file
 * @brief Stereo slam using visual odometry and g2o optimization (presentation).
 */

#ifndef BASE_H
#define BASE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_transport/subscriber_filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <libhaloc/lc.h>
#include "stereo_slam/SetPointCloud.h"
#include "stereo_slam/SetGraph.h"
#include <std_srvs/Empty.h>
#include "pose.h"
#include "graph.h"
#include "tools.h"

using namespace std;
using namespace cv;
using namespace tools;

typedef pcl::PointXYZRGB                  PointRGB;
typedef pcl::PointCloud<PointRGB>         PointCloudRGB;

namespace slam
{

class SlamBase
{

public:

	// Constructor
  SlamBase(ros::NodeHandle nh, ros::NodeHandle nhp);

  // Finalize stereo slam node
  void finalize();

  struct Params
  {
    // Motion parameters
    double min_displacement;          //!> Minimum odometry displacement between poses to be saved as graph vertices.
    bool save_clouds;                 //!> Save the pointclouds
    string clouds_dir;                //!> Directory where pointclouds will be saved
    int min_neighbor;                 //!> Jump this number of neighbors for closer loop closing candidates.
    bool refine_neighbors;            //!> If true, solvePNP will be applied between consecutive nodes. If false, the odometry will be applied.
    double x_filter_min;              //!> Cloud limit filter
    double x_filter_max;              //!> Cloud limit filter
    double y_filter_min;              //!> Cloud limit filter
    double y_filter_max;              //!> Cloud limit filter
    double z_filter_min;              //!> Cloud limit filter
    double z_filter_max;              //!> Cloud limit filter
    bool listen_reconstruction_srv;   //!> Listen for reconstruction services
    string set_cloud_srv;             //!> Name of the service to send the cloud
    string set_graph_srv;             //!> Name of the service to send the graph

    // Default settings
    Params () {
      min_displacement            = 0.2;
      save_clouds                 = false;
      clouds_dir                  = "";
      min_neighbor                = 10;
      refine_neighbors            = false;
      x_filter_min                = 3.0;
      x_filter_max                = -3.0;
      y_filter_min                = 3.0;
      y_filter_max                = -3.0;
      z_filter_min                = 0.2;
      z_filter_max                = 6.0;
      listen_reconstruction_srv   = false;
      set_cloud_srv               = "set_point_cloud";
      set_graph_srv               = "set_graph";
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

  // Services
  bool startReconstruction(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool stopReconstruction(std_srvs::Empty::Request&, std_srvs::Empty::Response&);


protected:

	// Node handlers
	ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Protected functions and callbacks
  void init();
  void readParameters();
  void msgsCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                    const sensor_msgs::ImageConstPtr& l_img_msg,
                    const sensor_msgs::ImageConstPtr& r_img_msg,
                    const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                    const sensor_msgs::CameraInfoConstPtr& r_info_msg);
  void msgsCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                    const sensor_msgs::ImageConstPtr& l_img_msg,
                    const sensor_msgs::ImageConstPtr& r_img_msg,
                    const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                    const sensor_msgs::CameraInfoConstPtr& r_info_msg,
                    const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  PointCloudRGB::Ptr filterCloud(PointCloudRGB::Ptr cloud);
  void processCloud(int cloud_id);
  void sendGraph();

private:

  // Topic properties
  image_transport::SubscriberFilter left_sub_, right_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_, right_info_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;

  // Topic sync properties (no pointcloud)
  typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry,
                                                    sensor_msgs::Image,
                                                    sensor_msgs::Image,
                                                    sensor_msgs::CameraInfo,
                                                    sensor_msgs::CameraInfo> ExactPolicyNoCloud;
  typedef message_filters::Synchronizer<ExactPolicyNoCloud> ExactSyncNoCloud;
  boost::shared_ptr<ExactSyncNoCloud> exact_sync_no_cloud_;

  // Topic sync properties (with pointcloud)
  typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry,
                                                    sensor_msgs::Image,
                                                    sensor_msgs::Image,
                                                    sensor_msgs::CameraInfo,
                                                    sensor_msgs::CameraInfo,
                                                    sensor_msgs::PointCloud2> ExactPolicyCloud;
  typedef message_filters::Synchronizer<ExactPolicyCloud> ExactSyncCloud;
  boost::shared_ptr<ExactSyncCloud> exact_sync_cloud_;

  Params params_;                   //!> Stores parameters
  haloc::LoopClosure lc_;           //!> Loop closure object
  slam::Pose pose_;                 //!> Pose object
  slam::Graph graph_;               //!> Graph object
  bool first_iter_;                 //!> Indicates first iteration
  PointCloudRGB pcl_cloud_;         //!> Current pointcloud to be saved
  bool reconstruction_srv_on_;      //!> True to enable the reconstruction services
};

} // namespace

#endif // BASE_H