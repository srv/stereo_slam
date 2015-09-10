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

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <opencv2/opencv.hpp>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "frame.h"
#include "graph.h"
#include "publisher.h"

using namespace std;
using namespace boost;
namespace fs  = filesystem;


typedef pcl::PointXYZRGB                  PointRGB;
typedef pcl::PointCloud<PointRGB>         PointCloudRGB;

namespace slam
{

class Publisher;
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
  Tracking(Publisher* f_pub, Graph* graph);

  /** \brief Set class params
   * \param the parameters struct
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
   * \param pointcloud
   */
  void msgsCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                    const sensor_msgs::ImageConstPtr& l_img_msg,
                    const sensor_msgs::ImageConstPtr& r_img_msg,
                    const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                    const sensor_msgs::CameraInfoConstPtr& r_info_msg,
                    const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

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
   */
  void needNewKeyFrame(PointCloudRGB::Ptr cloud);

  /** \brief Add a frame to the graph if enough inliers
   * \param The frame
   */
  void addFrameToMap(Frame frame, PointCloudRGB::Ptr cloud);


  /** \brief Filters a pointcloud
   * @return filtered cloud
   * \param input cloud
   */
  PointCloudRGB::Ptr filterCloud(PointCloudRGB::Ptr in_cloud);

private:

  Params params_; //!> Stores parameters.

  trackingState state_; //!> Tracking state

  tf::StampedTransform odom2camera_; //!> Transformation between robot odometry frame and camera frame.

  tf::TransformListener tf_listener_; //!> Listen for tf between robot and camera.

  Frame c_frame_; //!> Current frame

  Publisher* f_pub_; //!> Frame publisher

  image_geometry::StereoCameraModel camera_model_; //!> Stereo camera model

  Graph* graph_; //!> Graph

  tf::Transform last_fixed_frame_pose_; //!> Stores the last fixed frame pose

  int frame_id_; //!> Processed frames counter

  // Topic sync
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::CameraInfo,
                                                          sensor_msgs::CameraInfo,
                                                          sensor_msgs::PointCloud2> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;

};

} // namespace

#endif // TRACKING_H