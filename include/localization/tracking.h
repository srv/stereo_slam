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

#include <opencv2/opencv.hpp>

#include "frame.h"
#include "graph.h"
#include "frame_publisher.h"

using namespace std;
using namespace cv;

namespace slam
{

class FramePublisher;
class Graph;

class Tracking
{

public:

  struct Params
  {
    string odom_topic;                //!> Odometry topic name.
    string camera_topic;              //!> Name of the base camera topic

    // Default settings
    Params () {
      odom_topic   = "/odom";
      camera_topic = "/usb_cam";
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
  Tracking(FramePublisher* f_pub, Graph* graph);

  /** \brief Set class params
   * \param the parameters struct
   */
  inline void setParams(const Params& params){params_ = params;}

  /** \brief Get class params
   */
  inline Params getParams() const {return params_;}

  /** \brief Get current frame
  */
  inline Frame getFixedFrame() const {return f_frame_;}

  /** \brief Get current frame
   */
  inline Frame getCurrentFrame() const {return c_frame_;}

  /** \brief Get tracker matchings
   */
  inline vector<DMatch> getMatches() const {return matches_;}

  /** \brief Get tracker inliers
   */
  inline vector<int> getInliers() const {return inliers_;}

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

  /** \brief Track current frame with fixed frame
   */
  void trackCurrentFrame();

  /** \brief Decide if new fixed frame is needed
   */
  void needNewFixedFrame();

private:

  Params params_; //!> Stores parameters.

  trackingState state_; //!> Tracking state

  tf::StampedTransform odom2camera_; //!> Transformation between robot odometry frame and camera frame.

  tf::TransformListener tf_listener_; //!> Listen for tf between robot and camera.

  Mat camera_matrix_; //!> The camera matrix
  image_geometry::StereoCameraModel camera_model_; //!> Stereo camera model

  Frame f_frame_; //!> Fixed frame
  Frame p_frame_; //!> Previous frame
  Frame c_frame_; //!> Current frame
  Frame last_fixed_frame_before_lost_; //!> The last fixed frame before the system got lost

  vector<DMatch> matches_; //!> Vector of matchings between fixed and current frame
  vector<int> inliers_; //!> Vector of inliers between fixed and current frame

  Mat rvec_, tvec_; //!> Initial approximations of the rotation and translation vectors for the solvePNPransac

  FramePublisher* f_pub_; //!> Frame publisher

  Graph* graph_; //!> Graph

  bool reset_fixed_frame_; //!> Will be true on the next iteration after the fixed frame has been reset.

  ros::WallTime lost_time_; //!> Time at which the tracker got lost.

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