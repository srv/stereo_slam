/**
 * @file
 * @brief Stereo localization using visual odometry and g2o optimization (presentation).
 */

#ifndef STEREO_LOCALIZATION_BASE_H
#define STEREO_LOCALIZATION_BASE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/stereo_camera_model.h>
#include <database_interface/postgresql_database.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <opencv2/features2d/features2d.hpp>

namespace stereo_localization
{

class StereoLocalizationBase
{
public:
	// Constructor
  StereoLocalizationBase(ros::NodeHandle nh, ros::NodeHandle nhp);

protected:
	// Node handlers
	ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Protected functions and callbacks
  bool initializeStereoLocalization();
  void readParameters();
  void msgsCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                    const sensor_msgs::ImageConstPtr& l_img,
                    const sensor_msgs::ImageConstPtr& r_img,
                    const sensor_msgs::CameraInfoConstPtr& l_info,
                    const sensor_msgs::CameraInfoConstPtr& r_info);
  void timerCallback(const ros::WallTimerEvent& event);

private:
	// Database properties
	std::string db_host_, db_port_, db_user_, db_pass_, db_name_;
	boost::shared_ptr<database_interface::PostgresqlDatabase> pg_db_ptr_;
	PGconn* connection_init_;

	// Transform properties
  tf::Transform previous_pose_;			//!> Previous node pose
  tf::Transform accumulated_error_;	//!> Accumulated error in the optimization process (this is the key!)

  // G2O Optimization
  double update_rate_;							//!> Timer callback rate (in seconds) to optimize the graph.
  int g2o_algorithm_;								//!> Set to 0 for LinearSlam Solver. Set to 1 for.
  int go2_opt_max_iter_;						//!> Maximum number of iteration for the graph optimization.
  bool go2_verbose_;								//!> True to output the g2o iteration messages
  g2o::SparseOptimizer 
  	graph_optimizer_;								//!> G2O graph optimizer
  ros::WallTimer timer_;						//!> Timer to optimize the graph while it is updated

  // Operational properties
  double min_displacement_;					//!> Minimum odometry displacement between poses to be saved as graph nodes. 
  double min_candidate_threshold_;	//!> Minimum distance between graph nodes to be considered for possible candidates of loop closure.
  bool first_message_;							//!> True when first message is received, false for any other instant.
  bool first_node_;									//!> True when first node is inserted into graph, false for any other instant.
  bool block_update_;								//!> Used to block the timer re-calls when it is executed.
  std::vector<cv::Point2i> 
  	false_candidates_;							//!> Vector of detected false candidates to prevent re-calculation.

  // Computer vision properties
  double descriptors_threshold_;		//!> Matching descriptors threshold used to find loop closures between images.
  int matches_threshold_;						//!> Minimum number of matches to consider that there is overlap between two images.
  image_geometry::StereoCameraModel 
  	stereo_camera_model_;						//!> Object to save the image camera model
  cv::Mat camera_matrix_;						//!> Used to save the camera matrix

  // Topic properties
  int queue_size_;									//!> Indicate the maximum number of messages encued.
  image_transport::SubscriberFilter 
  	left_sub_, right_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_, right_info_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  ros::Publisher odom_pub_;
  std::string map_frame_id_, base_link_frame_id_;

  // Topic sync properties
  typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, 
                                                    sensor_msgs::Image, 
                                                    sensor_msgs::Image, 
                                                    sensor_msgs::CameraInfo, 
                                                    sensor_msgs::CameraInfo> ExactPolicy;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, 
                                                    sensor_msgs::Image, 
                                                    sensor_msgs::Image, 
                                                    sensor_msgs::CameraInfo, 
                                                    sensor_msgs::CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;
};

} // namespace

#endif // STEREO_LOCALIZATION_BASE_H