/**
 * @file
 * @brief The tracking class is responsive to track consecutive image frames (presentation).
 */

#ifndef TRACKING_H
#define TRACKING_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/stereo_camera_model.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "frame.h"
#include "graph.h"
#include "publisher.h"

using namespace std;
using namespace boost;
namespace fs  = filesystem;

namespace slam
{

class Publisher;
class Graph;

class Tracking
{

public:

  struct Params
  { 
    bool refine;                        //!> Refine odometry.
    int lc_min_inliers;                 //!> Minimum number of inliers to close a loop.
    double lc_epipolar_thresh;          //!> Maximum reprojection error allowed.
    double dist_keyframes;              //!> Distance between keyframes.
    string working_directory;           //!> Directory where all output files will be stored.

    // Default settings
    Params () 
    {
      refine             = false;
      lc_min_inliers     = 40;
      lc_epipolar_thresh = 2.0;
      dist_keyframes     = 0.5;
      working_directory  = "";
    }
  };

  enum trackingState{
    NOT_INITIALIZED   = 0,
    INITIALIZING      = 1,
    WORKING           = 2,
    LOST              = 3
  };

  /** \brief Class constructor
   * \param Frame publisher object pointer
   * \param Graph object pointer
   */
  Tracking(Publisher* f_pub, Graph* graph);

  /** \brief Set class params
   *  \param the parameters struct
   */
  inline void setParams(const Params& params){params_ = params;}

  /** \brief Get class params
   */
  inline Params getParams() const {return params_;}

  /** \brief Get current frame
   */
  inline Frame getCurrentFrame() const {return c_frame_;}

  /** \brief Starts tracking
   */
  void run();

protected:

  /** \brief Messages callback. This function is called when synchronized odometry and image
   * message are received.
   * \param odom_msg ros odometry message of type nav_msgs::Odometry
   * \param l_img left stereo image message of type sensor_msgs::Image
   * \param r_img right stereo image message of type sensor_msgs::Image
   * \param l_info left stereo info message of type sensor_msgs::CameraInfo
   * \param r_info right stereo info message of type sensor_msgs::CameraInfo
   */  
    void msgsCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                    const sensor_msgs::ImageConstPtr& l_img_msg,
                    const sensor_msgs::ImageConstPtr& r_img_msg,
                    const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                    const sensor_msgs::CameraInfoConstPtr& r_info_msg);

  /** \brief Get the transform between odometry frame and camera frame
   * @return true if valid transform, false otherwise
   * \param Odometry msg
   * \param Image msg
   * \param Output transform
   */
  bool getOdom2CameraTf(nav_msgs::Odometry odom_msg,
                        sensor_msgs::Image img_msg,
                        tf::StampedTransform &transform);

  /** \brief Decide if new keyframe is needed
   * @return True if new keyframe will be inserted into the graph
   */
  bool needNewKeyFrame();

  /** \brief Add a frame to the graph if enough inliers
   * @return True if new keyframe will be inserted into the map
   */
  bool addFrameToMap();


  /** \brief Filters a pointcloud
   * @return filtered cloud
   * \param input cloud
   */

  /** \brief Publishes the overlapping debug image
   * @return
   * \param current pointcloud
   * \param the transformation of current pointcloud to the last fixed frame
   * \param the overlap
   */

  /** \brief Refine the keyframe to keyframe position using SolvePnP
   * @return True if a valid transform was found
   * \param current frame
   * \param previous frame
   * \param the estimated transform
   * \param covariance of the transformation
   * \param number of inliers for the refined pose
   */
  bool refinePose(Frame c_frame, Frame p_frame, tf::Transform& out, cv::Mat& sigma, int& num_inliers);

private:

  Params params_; //!> Stores parameters.

  trackingState state_; //!> Tracking state

  tf::StampedTransform odom2camera_; //!> Transformation between robot odometry frame and camera frame.

  tf::TransformListener tf_listener_; //!> Listen for tf between robot and camera.

  Frame c_frame_; //!> Current frame

  Frame p_frame_; //!> Previous frame

  cv::Mat camera_matrix_; //!> Camera matrix

  Publisher* f_pub_; //!> Frame publisher

  ros::Publisher pose_pub_; //!> Corrected pose publisher

  ros::Publisher overlapping_pub_; //!> Consecutive image overlapping publisher

  tf::TransformBroadcaster tf_broadcaster_; //!> Publish transform

  image_geometry::StereoCameraModel camera_model_; //!> Stereo camera model

  Graph* graph_; //!> Graph

  tf::Transform last_fixed_frame_pose_; //!> Stores the last fixed frame pose

  Eigen::Vector4f last_min_pt_, last_max_pt_; // Stores the last fixed frame minimum and maximum points

  int frame_id_; //!> Processed frames counter

  vector<tf::Transform> odom_pose_history_; //!> Stores the odometry poses, relative to camera frame

  tf::Transform prev_robot_pose_; //!> Stores the previous corrected odometry pose

  ros::WallTime jump_time_; //!> Stores the time at which the jump starts

  bool jump_detected_; //!> Indicates when a big correction is detected

  double secs_to_filter_; //!> Number of seconds that filter will be applied

  // Topic sync
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::CameraInfo,
                                                          sensor_msgs::CameraInfo> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;

};

} // namespace

#endif // TRACKING_H
