/**
 * @file
 * @brief Stereo slam using visual odometry and g2o optimization (presentation).
 */

#ifndef STEREO_SLAM_BASE_H
#define STEREO_SLAM_BASE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>
#include <cv_bridge/cv_bridge.h>
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
#include <sensor_msgs/image_encodings.h>

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
namespace enc = sensor_msgs::image_encodings;

namespace stereo_slam
{

class StereoSlamBase
{

public:

	// Constructor
  StereoSlamBase(ros::NodeHandle nh, ros::NodeHandle nhp);

  struct Params
  {
    /**
     * Default constructor sets all values to defaults.
     */
    Params();

    // Database parameters
    std::string db_host, db_port, db_user, db_pass, db_name;

    // G2O Optimization
    double update_rate;              //!> Timer callback rate (in seconds) to optimize the graph.
    int g2o_algorithm;               //!> Set to 0 for LinearSlam Solver with gauss-newton. Set to 1 for LinearSlam Solver with Levenberg.
    int go2_opt_max_iter;            //!> Maximum number of iteration for the graph optimization.
    bool go2_verbose;                //!> True to output the g2o iteration messages

    // Graph operational parameters
    double min_displacement;         //!> Minimum odometry displacement between poses to be saved as graph vertices. 
    double max_candidate_threshold;  //!> Maximum distance between graph vertices to be considered for possible candidates of loop closure.
    int neighbor_offset;             //!> Number of neighbor nodes discarted for loop-closing.
    bool save_graph_to_file;         //!> True if user wants to save the graph into file.
    std::string files_path;          //!> Path where save the graph data.
    bool save_graph_images;          //!> True to save the graph images into the path

    // Stereo vision parameters
    std::string desc_type;           //!> Descriptor type can be SIFT or SURF
    double descriptor_threshold;     //!> Matching descriptors threshold used to find loop closures between images.
    double epipolar_threshold;       //!> Maximum epipolar distance for stereo matching.
    int matches_threshold;           //!> Minimum number of matches to consider that there is overlap between two images.
    int min_inliers;                 //!> Minimum number of inliers found by solvePnPRansac to take into account the edge in the graph.
    int max_inliers;                 //!> Maximum number of inliers for solvePnPRansac, stop if more inliers than this are found.
    int max_solvepnp_iter;           //!> Maximum number of interations of the solvePnPRansac algorithm.
    double allowed_reprojection_err; //!> Maximum reprojection error allowed in solvePnPRansac algorithm.
    double max_edge_err;             //!> Maximum pose difference to take the new edge as valid.
    bool stereo_vision_verbose;      //!> True to output the messages of stereo matching process, false otherwise.
    int bucket_width;                //!> Bucket width.
    int bucket_height;               //!> Bucket height.
    int max_bucket_features;         //!> Maximum number of features per bucket.

    // Topic parameters
    int queue_size;                  //!> Indicate the maximum number of messages encued.
    std::string map_frame_id;        //!> The map frame id.
    std::string base_link_frame_id;  //!> The robot base link frame id.

    // Default values
    static const double       DEFAULT_UPDATE_RATE = 3.0;
    static const int          DEFAULT_G2O_ALGORITHM = 1;
    static const int          DEFAULT_G2O_OPT_MAX_ITER = 10;
    static const bool         DEFAULT_G2O_VERBOSE = false;
    static const double       DEFAULT_MIN_DISPLACEMENT = 0.2;
    static const double       DEFAULT_MAX_CANDIDATE_THRESHOLD = 0.5;
    static const int          DEFAULT_NEIGHBOR_OFFSET = 3;
    static const bool         DEFAULT_SAVE_GRAPH_TO_FILE = false;
    static const bool         DEFAULT_SAVE_GRAPH_IMAGES = false;
    static const double       DEFAULT_DESCRIPTOR_THRESHOLD = 0.5;
    static const double       DEFAULT_EPIPOLAR_THRESHOLD = 3.0;
    static const int          DEFAULT_MATCHES_THRESHOLD = 110;
    static const int          DEFAULT_MIN_INLIERS = 10;
    static const int          DEFAULT_MAX_INLIERS = 50;
    static const int          DEFAULT_MAX_SOLVEPNP_ITER = 100;
    static const double       DEFAULT_ALLOWED_REPROJECTION_ERR = 5.0;
    static const double       DEFAULT_MAX_EDGE_ERR = 10.0;
    static const bool         DEFAULT_STEREO_VISION_VERBOSE = false;    
    static const int          DEFAULT_BUCKET_WIDTH = 50;
    static const int          DEFAULT_BUCKET_HEIGHT = 50;
    static const int          DEFAULT_MAX_BUCKET_FEATURES = 3;
    static const int          DEFAULT_QUEUE_SIZE = 2;

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
  bool initializeStereoSlam();
  void readParameters();
  void msgsCallback(  const nav_msgs::Odometry::ConstPtr& odom_msg,
                      const sensor_msgs::ImageConstPtr& l_img,
                      const sensor_msgs::ImageConstPtr& r_img,
                      const sensor_msgs::CameraInfoConstPtr& l_info,
                      const sensor_msgs::CameraInfoConstPtr& r_info);
  void timerCallback( const ros::WallTimerEvent& event);
  bool saveGraph();
  bool vertexInsertion( cv_bridge::CvImagePtr l_ptr, 
                        cv_bridge::CvImagePtr r_ptr,
                        tf::Transform corrected_pose);
  bool graphUpdater();
  bool getLoopClosing(g2o::VertexSE3* v_i, g2o::VertexSE3* v_j, tf::Transform& output);

private:

	// Database properties
	boost::shared_ptr<database_interface::PostgresqlDatabase> pg_db_ptr_thread_1_;
  boost::shared_ptr<database_interface::PostgresqlDatabase> pg_db_ptr_thread_2_;
	PGconn* connection_init_;

	// Transform properties
  std::vector<cv::Point2i> 
    false_candidates_;              //!> Vector of detected false candidates to prevent re-calculation.
  std::vector<tf::Transform> 
    pose_history_;                  //!> Vector to save the pose history
  std::vector<double> 
    pose_history_stamp_;            //!> Vector to save the timestamp pose history

  // G2O Optimization
  g2o::SparseOptimizer 
  	graph_optimizer_;								//!> G2O graph optimizer
  ros::WallTimer timer_;						//!> Timer to optimize the graph while it is updated

  // Operational properties
  bool first_message_;							//!> True when first message is received, false for any other instant.
  bool first_vertex_;							  //!> True when first vertex is inserted into graph, false for any other instant.
  bool block_update_;								//!> Used to block the timer re-calls when it is executed.
  bool block_insertion_;            //!> Used to block the insertion of nodes while graph is optimized.

  // Stereo vision properties
  image_geometry::StereoCameraModel 
  	stereo_camera_model_;						//!> Object to save the image camera model
  cv::Mat camera_matrix_;						//!> Used to save the camera matrix

  // Topic properties
  image_transport::SubscriberFilter 
  	left_sub_, right_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_, right_info_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  ros::Publisher odom_pub_;

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

  /// Stores parameters
  Params params_;
};

} // namespace

#endif // STEREO_SLAM_BASE_H